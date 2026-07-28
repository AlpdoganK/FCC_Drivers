#include "app.h"
#include "i2c.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

static void I2C_BusRecover(I2C_HandleTypeDef *hi2c,
                            GPIO_TypeDef *scl_port, uint16_t scl_pin,
                            GPIO_TypeDef *sda_port, uint16_t sda_pin)
{
    HAL_I2C_DeInit(hi2c);

    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pin = scl_pin; HAL_GPIO_Init(scl_port, &g);
    g.Pin = sda_pin; HAL_GPIO_Init(sda_port, &g);

    HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_SET);
    HAL_Delay(5);

    for (int i = 0; i < 20; i++) {
        HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
        HAL_Delay(1);
        if (HAL_GPIO_ReadPin(sda_port, sda_pin) == GPIO_PIN_SET) break;
    }

    // STOP: SDA rises while SCL is high
    HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_SET);
    HAL_Delay(5);

    if (hi2c->Instance == I2C1) MX_I2C1_Init();
    else                        MX_I2C2_Init();
}

#define RAD_TO_DEG 57.295779513f

// Pad-pressure calibration (see App_Init). At ~20 ms per round this costs
// about 600 ms of startup, on top of the delays App_Init already takes.
#define BARO_CAL_DISCARD 10u // samples dropped while the 8x IIR filter settles
#define BARO_CAL_SAMPLES 20u // samples averaged into the reference
#define BARO_CAL_MIN_HPA 600.0f
#define BARO_CAL_MAX_HPA 1100.0f

// Sensor & Filter Instances
static MPU6050 myMPU6050;
static BME280 baro1;
static BME280 baro2;
static LoRa_E220 myLora;
static NEO_M8N myGPS;
static SensorStats_t baro1_stats;
static SensorStats_t baro2_stats;
static BaroHealth_t  avionics_health;
static bool mpu_healthy = false;
static ComplementaryFilter_t pitch_cf;
static ComplementaryFilter_t roll_cf;

// Timing Allocations
static uint32_t imu_last_tick  = 0;
static uint32_t baro_last_tick = 0;
const uint32_t imu_interval  = 10; // 100 Hz
const uint32_t baro_interval = 50; // 20 Hz

static uint32_t lora_last_tick = 0;
const uint32_t  lora_interval  = 200;
static uint8_t lorast = 0;


static uint32_t print_last_tick = 0;

// SIT (Sensor Izleme Testi) scheduling. Spec 2.1.2c: sensor data starts
// flowing one second after the command is validated, then at 10 Hz until
// the STOP command arrives.
const uint32_t sit_arm_delay   = 1000; // 1 s between command and first packet
const uint32_t sit_tx_interval = 100;  // 10 Hz
static bool     sit_arming     = false;
static bool     sit_active     = false;
static uint32_t sit_arm_tick   = 0;
static uint32_t sit_last_tick  = 0;
static uint8_t  sitst          = 0;

// ---- RS232 idle heartbeat --------------------------------------------------
// Sent once a second while no test is streaming, so the ground station shows a
// live RX rate instead of a dead link. Suppressed during SİT — that traffic
// already proves the link and an extra frame would only confuse the parser.
//
//   RS232_HB_BINARY — Tablo 6 status packet. The test software parses this, so
//                     it drives the RX counter AND validates our checksum
//                     against its "Checksum Hatası" tally. Default.
//   RS232_HB_ASCII  — human-readable status line for a serial monitor. Not
//                     protocol traffic; the test software will not understand it.
//   RS232_HB_UPATT  — continuous 0x55 ('U') square wave, for diagnosing baud
//                     and signal integrity when the link looks dead.
#define RS232_HB_BINARY 0
#define RS232_HB_ASCII  1
#define RS232_HB_UPATT  2

#define RS232_HB_MODE RS232_HB_BINARY

// ---- LoRa transmit master switch (DIAGNOSTIC) ------------------------------
// The E220-900T30S is a 1 W (30 dBm) transmitter keyed every 200 ms, and an
// RS232 cable makes a fine receiving antenna. Set to 0 to silence the radio
// completely so RF interference can be ruled in or out as the cause of
// corrupted USART6 receive data. The module is still initialised, so it sits
// in a configured idle state rather than an unknown one - it simply never
// gets fed anything to transmit.
// SET BACK TO 1 BEFORE FLIGHT: with this at 0 there is no telemetry at all.
//
// Bench history: with the antenna DISCONNECTED this radio corrupted USART6
// receive badly enough that no command frame ever decoded (garbage bytes,
// framing errors). An unterminated PA reflects its output and radiates from
// the module and traces instead of the antenna. Never key this module without
// an antenna fitted - it is both an EMC problem and a way to damage the PA.
#define LORA_TX_ENABLED 0

#if RS232_HB_MODE == RS232_HB_UPATT
const uint32_t  rs232_hb_interval = 100;  // 10 Hz — near-continuous stream
#else
const uint32_t  rs232_hb_interval = 1000; // 1 Hz
#endif
static uint32_t rs232_hb_last_tick = 0;
static uint8_t  hbst               = 0;

// Flight events mapped onto the Tablo 5 status bits (names from EK-15 Tablo 3).
// The flight SM exposes a single state rather than individual event flags, so
// the bits are derived from how far the progression has advanced — events are
// cumulative, since a state is only reachable by passing through the earlier
// ones. Once SUT lands and real per-event flags exist, prefer those.
static uint16_t Status_BitsFromState(uint8_t state)
{
    uint16_t bits = 0;
    if (state >= FLIGHT_BOOST)                { bits |= (1u << 0); } // KTE liftoff
    if (state >= FLIGHT_COAST)                { bits |= (1u << 1); } // YSD burn time
    if (state >= FLIGHT_MIN_ALTITUDE_REACHED) { bits |= (1u << 2); } // IEA min altitude
    if (state >= FLIGHT_APOGEE)               { bits |= (1u << 3); } // GAA body angle
    if (state >= FLIGHT_DESCENT)              { bits |= (1u << 4)     // ATE descent
                                                      | (1u << 5); }  // SPE drogue cmd
    if (state >= FLIGHT_MAIN)                 { bits |= (1u << 6)     // BIT alt threshold
                                                      | (1u << 7); }  // APE main cmd
    return bits;
}

// Indexed by Test_Status (Test_SIT, Test_SUT, Test_Stop). Only referenced by
// optional paths (DBG_PRINT, ASCII heartbeat), so it goes unused when both
// are compiled out.
__attribute__((unused))
static const char *rs232_mode_name[] = { "SIT", "SUT", "STOP" };

// Local Telemetry Tracks
float rocket_pitch    = 0.0f;
float rocket_roll     = 0.0f;
float rocket_yaw      = 0.0f;
float rocket_altitude = 0.0f;
float sea_level_pressure = 1013.25f;

// IMU data — module-scope so FlightSM_Update can see them
static float ax = 0.0f, ay = 0.0f, az = 0.0f;
static float gy = 0.0f;
static float gz = 0.0f;

void App_Init(I2C_HandleTypeDef *hi2c1, I2C_HandleTypeDef *hi2c2,
              UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2, UART_HandleTypeDef *huart6) {
    
                
    Pyro_Init();
    DBG_PRINT("Pyro Module Initialized\r\n");
    HAL_Delay(3000);
    
    I2C_BusRecover(hi2c1, GPIOB, GPIO_PIN_6, GPIOB, GPIO_PIN_7);
    I2C_BusRecover(hi2c2, GPIOB, GPIO_PIN_10, GPIOB, GPIO_PIN_3);

    // I2C bus scan — probe known addresses to verify buses are alive
    DBG_PRINT("Scanning I2C1...\r\n");
    for (uint8_t addr = 1; addr < 128; addr++) {
        if (HAL_I2C_IsDeviceReady(hi2c1, (uint16_t)(addr << 1), 2, 10) == HAL_OK) {
            DBG_PRINT("  I2C1 device found at 0x%02X\r\n", addr);
        }
    }
    DBG_PRINT("Scanning I2C2...\r\n");
    for (uint8_t addr = 1; addr < 128; addr++) {
        if (HAL_I2C_IsDeviceReady(hi2c2, (uint16_t)(addr << 1), 2, 10) == HAL_OK) {
            DBG_PRINT("  I2C2 device found at 0x%02X\r\n", addr);
        }
    }

    // 1. Initialize MPU6050 on I2C Bus 1
    MPU6050_Config mpuConfig = {
        .accel_fs = MPU6050_ACCEL_FS_16G,
        .gyro_fs  = MPU6050_GYRO_FS_2000,
        .dlpf     = MPU6050_DLPF_44HZ
    };

    uint8_t mpu_err = MPU6050_Initialise(&myMPU6050, hi2c1, &mpuConfig);
    if (mpu_err != 0) {
        DBG_PRINT("Failed to initialize MPU6050: err=0x%02X i2c_err=0x%08lX\r\n",
                  mpu_err, hi2c1->ErrorCode);
        DBG_PRINT("  PWR_MGMT_1 readback=0x%02X (want 0x00)  CONFIG(DLPF) readback=0x%02X (want 0x%02X)  ACCEL_CONFIG readback=0x%02X (want 0x%02X)  GYRO_CONFIG readback=0x%02X (want 0x%02X)\r\n",
                  myMPU6050.pwr_mgmt_1_readback,
                  myMPU6050.dlpf_cfg_readback, (uint8_t)mpuConfig.dlpf,
                  myMPU6050.accel_cfg_readback, (uint8_t)mpuConfig.accel_fs,
                  myMPU6050.gyro_cfg_readback, (uint8_t)mpuConfig.gyro_fs);
        HAL_Delay(5000);
    } else {
        mpu_healthy = true;
        DBG_PRINT("MPU6050 OK\r\n");
    }

    // 2. Initialize Dual Independent BME280s across different I2C lines
    BME280_Config baroConfig = {
        .temp_osr = BME280_OVERSAMPLING_1X,
        .press_osr = BME280_OVERSAMPLING_4X,
        .hum_osr  = BME280_OVERSAMPLING_1X,
        .filter   = BME280_FILTER_8X,
        .mode     = BME280_MODE_NORMAL
    };

    uint8_t b1_err = BME280_Initialise(&baro1, hi2c1, &baroConfig);
    if (b1_err != 0) {
        DBG_PRINT("Failed to initialize BME280 #1: err=0x%02X i2c_err=0x%08lX\r\n",
                  b1_err, hi2c1->ErrorCode);
        HAL_Delay(5000);
    } else {
        DBG_PRINT("BME280 #1 OK\r\n");
    }

    uint8_t b2_err = BME280_Initialise(&baro2, hi2c2, &baroConfig);
    if (b2_err != 0) {
        DBG_PRINT("Failed to initialize BME280 #2: err=0x%02X i2c_err=0x%08lX\r\n",
                  b2_err, hi2c2->ErrorCode);
        HAL_Delay(5000);
    } else {
        DBG_PRINT("BME280 #2 OK\r\n");
    }

    // 2b. Calibrate the barometric reference against pad pressure.
    // Altitude is computed from sea_level_pressure in App_Run; the formula's
    // altitude==0 solution is simply P0 == P_pad, so averaging pad pressure
    // into the reference makes altitude read ~0 m on the ground (AGL).
    // Left at the 1013.25 hPa standard atmosphere, every altitude carries the
    // day's QNH error — enough to sit below the test software's 0-10000 m
    // window on a high-pressure day (EK-15) and to shift the 800 m main-chute
    // threshold in flight_sm.c by the same offset.
    if (b1_err == 0 || b2_err == 0) {
        for (uint8_t i = 0; i < BARO_CAL_DISCARD; i++) {
            if (b1_err == 0) BME280_ReadAll(&baro1);
            if (b2_err == 0) BME280_ReadAll(&baro2);
            HAL_Delay(20);
        }

        float    sum = 0.0f;
        uint16_t n   = 0;
        for (uint8_t i = 0; i < BARO_CAL_SAMPLES; i++) {
            if (b1_err == 0 && BME280_ReadAll(&baro1) == HAL_OK) {
                sum += baro1.pressure_hPa; n++;
            }
            if (b2_err == 0 && BME280_ReadAll(&baro2) == HAL_OK) {
                sum += baro2.pressure_hPa; n++;
            }
            HAL_Delay(20);
        }

        // Gate the result: a wild reading accepted here would silently bias
        // every altitude — and therefore every deploy decision — for the
        // whole flight. Better to fall back to the standard atmosphere.
        float mean = (n > 0) ? (sum / (float)n) : 0.0f;
        if (n > 0 && mean >= BARO_CAL_MIN_HPA && mean <= BARO_CAL_MAX_HPA) {
            sea_level_pressure = mean;
            DBG_PRINT("Baro reference calibrated to %.2f hPa (%u samples)\r\n", mean, n);
        } else {
            DBG_PRINT("Baro calibration rejected (mean=%.2f hPa n=%u), keeping %.2f hPa\r\n",
                      mean, n, sea_level_pressure);
        }
    } else {
        DBG_PRINT("Baro calibration skipped: no healthy barometer\r\n");
    }

    LoRa_Init(&myLora, huart1, LORA_AUX_GPIO_Port, LORA_AUX_Pin);
    // GPS disabled: USART2 is repurposed as the debug console (see
    // debug_uart.c) and not needed alongside GPS at the same time.
    (void)huart2;
    (void)huart6; // UKB_RS232_Init() below talks to huart6 directly
    UKB_RS232_Init();

    // 3. Setup Redundant Fault Latches & Running Statistics Matrices
    // A sensor that failed init never gets a chance to prove itself at
    // runtime — latch it unhealthy immediately so it's excluded from
    // fusion/health-checking from the first sample, instead of feeding
    // Stats_Update a frozen zero until the runtime fault detector catches up.
    avionics_health.sensor1_healthy = (b1_err == 0);
    avionics_health.sensor2_healthy = (b2_err == 0);
    avionics_health.fault_count     = 0;
    avionics_health.last_alt1       = 0.0f;
    avionics_health.last_alt2       = 0.0f;
    avionics_health.stuck_count1    = 0;
    avionics_health.stuck_count2    = 0;

    baro1_stats.mean = 0.0f; baro1_stats.variance = 1.0f; baro1_stats.alpha = 0.05f; baro1_stats.mean_beta = 0.05f;
    baro2_stats.mean = 0.0f; baro2_stats.variance = 1.0f; baro2_stats.alpha = 0.05f; baro2_stats.mean_beta = 0.05f;

    Complementary_Init(&pitch_cf, 0.95f, 0.01f, 0.0f);
    Complementary_Init(&roll_cf,  0.95f, 0.01f, 0.0f);

    imu_last_tick  = HAL_GetTick();
    baro_last_tick = HAL_GetTick();
    lora_last_tick = HAL_GetTick();
    print_last_tick = HAL_GetTick();

    FlightSM_Init();
}

void App_Run(void) {
    uint32_t current_time = HAL_GetTick();
    Pyro_ProcessTimeouts();
    // GPS disabled — see App_Init note; USART2 is the debug console now.

    // ====================================================================
    // PHASE 0: RS232 GROUND-TEST COMMAND HANDLING (USART6)
    // rs232.c only validates frames and raises flags from ISR context;
    // all the actual test-mode behavior is decided here.
    // ====================================================================
    if (flag_sit_pending) {
        flag_sit_pending = 0;
        DBG_PRINT("RS232: SIT command received\r\n");
        sit_arming    = true;
        sit_active    = false;
        sit_arm_tick  = current_time;
    }

    if (flag_sut_pending) {
        flag_sut_pending = 0;
        DBG_PRINT("RS232: SUT command received\r\n");
        // A test-mode switch cancels any SIT stream still running.
        sit_arming = false;
        sit_active = false;
        // TODO: arm 1 s SUT timer, start 10 Hz status TX per Tablo spec
    }

    if (flag_stop_pending) {
        flag_stop_pending = 0;
        DBG_PRINT("RS232: STOP command received\r\n");
        sit_arming = false;
        sit_active = false;
        // TODO: tear down whatever SUT started, return to normal flight-computer operation
    }

    // Spec 2.1.2c: hold off one second after the command before the first packet.
    if (sit_arming && current_time - sit_arm_tick >= sit_arm_delay) {
        sit_arming    = false;
        sit_active    = true;
        sit_last_tick = current_time - sit_tx_interval; // transmit immediately
        DBG_PRINT("RS232: SIT armed, streaming at 10 Hz\r\n");
    }

    if (flag_sut_data_ready) {
        flag_sut_data_ready = 0;
        DBG_PRINT("RS232: SUT data packet received (%u bytes)\r\n", ukb_sut_data_len);
        // TODO: feed ukb_sut_data[0..ukb_sut_data_len) (Tablo 4 synthetic
        // sensor packet) into the algorithm under test instead of live sensors
    }

    // ====================================================================
    // PHASE 1: TIMED NON-BLOCKING TRIGGER REQUESTS
    // ====================================================================

    if (current_time - imu_last_tick >= imu_interval) {
        imu_last_tick = current_time;
        if (mpu_healthy && MPU6050_ReadAll(&myMPU6050) == HAL_OK) {
            myMPU6050.freshData = true;
        }
    }

    if (current_time - baro_last_tick >= baro_interval) {
        baro_last_tick = current_time;

        if (avionics_health.sensor1_healthy && BME280_ReadAll(&baro1) == HAL_OK) {
            baro1.freshData = true;
        }

        if (avionics_health.sensor2_healthy && BME280_ReadAll(&baro2) == HAL_OK) {
            baro2.freshData = true;
        }
    }

    // ====================================================================
    // PHASE 2: ASYNCHRONOUS BACKGROUND COMPLETION CHECKS
    // ====================================================================

    if (myMPU6050.freshData) {
        myMPU6050.freshData = false;

        // Update module-scope variables so FlightSM_Update always has fresh data
        ax = myMPU6050.acc_mps2[0]; // Nose Vector
        ay = myMPU6050.acc_mps2[1]; // Pitch Axis Lateral
        az = myMPU6050.acc_mps2[2]; // Yaw Axis Lateral

        float gx = myMPU6050.gyro[0]; // Roll Rate
        gy = myMPU6050.gyro[1];       // Pitch Rate — module-scope, FlightSM reads it
        gz = myMPU6050.gyro[2];       // Yaw Rate

        float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
        float accel_roll  = atan2f(ay, az) * RAD_TO_DEG;

        rocket_pitch = Complementary_Update(&pitch_cf, accel_pitch, gy);
        rocket_roll  = Complementary_Update(&roll_cf,  accel_roll,  gx);

        // No magnetometer, so yaw is dead-reckoned from the gyro alone and
        // will drift. Good enough for the SIT bench check (the operator is
        // looking for the value to respond to movement), not for flight use.
        // Wrapped to [-180, 180): the test software (EK-15 Sekil 1) counts
        // every sample outside that band as a "Hatali" reading, and free
        // integration walks out of range on drift alone.
        rocket_yaw += gz * (imu_interval * 0.001f);
        if (rocket_yaw >= 180.0f || rocket_yaw < -180.0f) {
            rocket_yaw -= 360.0f * floorf((rocket_yaw + 180.0f) / 360.0f);
        }
    }

    static float alt1 = 0.0f;
    static float alt2 = 0.0f;
    bool process_fusion = false;

    if (baro1.freshData) {
        baro1.freshData = false;
        alt1 = 44330.0f * (1.0f - powf((baro1.pressure_hPa / sea_level_pressure), 0.1902949f));
        process_fusion = true;
    }

    if (baro2.freshData) {
        baro2.freshData = false;
        alt2 = 44330.0f * (1.0f - powf((baro2.pressure_hPa / sea_level_pressure), 0.1902949f));
        process_fusion = true;
    }

    if (process_fusion) {
        rocket_altitude = Baro_FusedAltitude(&avionics_health, &baro1_stats, &baro2_stats, alt1, alt2);
    }

    // ax/ay/az/gy are now module-scope — always valid here
    FlightSM_Update(rocket_altitude, ax, ay, az, rocket_pitch, gy);

    // ====================================================================
    // SIT SENSOR STREAM (Tablo 3, 10 Hz on USART6)
    // Placed after fusion so every packet carries the freshest sample.
    // ====================================================================
    if (sit_active && current_time - sit_last_tick >= sit_tx_interval) {
        // Advance by exactly one interval instead of snapping to now. Snapping
        // makes each period (interval + however long the loop took to notice),
        // so loop latency compounds into a systematically slow rate — roughly
        // 8 Hz instead of the 10 Hz EK-7 2.1.2d requires. If we have fallen
        // more than a full slot behind, resync rather than firing a catch-up
        // burst, which would violate the cadence in the other direction.
        sit_last_tick += sit_tx_interval;
        if (current_time - sit_last_tick >= sit_tx_interval) {
            sit_last_tick = current_time;
        }

        UKB_SensorSample sample;
        // The ground software counts altitudes outside 0-10000 m as "Hatalı"
        // (EK-15 Sekil 1). Sitting on the bench, ambient pressure drifts away
        // from the startup reference and the true value goes a metre or two
        // negative. Clamp for transmission only — rocket_altitude itself stays
        // signed, because the flight state machine needs real altitude for
        // descent detection and the 800 m main-chute threshold.
        sample.altitude_m    = (rocket_altitude < 0.0f) ? 0.0f : rocket_altitude;
        // Report the barometer actually driving the fused altitude; if #1 has
        // been latched faulty its pressure is stale and would look like an
        // anomaly to the test operator.
        sample.pressure_mbar = avionics_health.sensor1_healthy ? baro1.pressure_hPa
                                                               : baro2.pressure_hPa;
        sample.acc_x         = ax;
        sample.acc_y         = ay;
        sample.acc_z         = az;
        // Bolum 1.2 body axes: rotation about X is roll, about Y is pitch,
        // about Z is yaw.
        sample.ang_x         = rocket_roll;
        sample.ang_y         = rocket_pitch;
        sample.ang_z         = rocket_yaw;

        sitst = (uint8_t)UKB_RS232_SendSensorPacket(&sample);
    }

    // Idle link heartbeat — see RS232_HB_MODE.
    if (!sit_active && current_time - rs232_hb_last_tick >= rs232_hb_interval) {
        rs232_hb_last_tick = current_time;

#if RS232_HB_MODE == RS232_HB_BINARY
        hbst = (uint8_t)UKB_RS232_SendStatusPacket(
                   Status_BitsFromState(FlightSM_GetState()));
#elif RS232_HB_MODE == RS232_HB_UPATT
        // 64 x 0x55 then CRLF, so a terminal shows fixed-width rows of 'U'.
        // ~5.7 ms of blocking TX per burst at 115200.
        static const char diag_row[] =
            "UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU"
            "UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU\r\n";
        hbst = (uint8_t)UKB_RS232_SendText(diag_row);
#else
        char line[160];
        snprintf(line, sizeof(line),
                 "UKB t=%lu.%lus mode=%s state=%u alt=%.2fm p=%.2fhPa "
                 "acc=%.2f,%.2f,%.2f ang=%.1f,%.1f,%.1f baro=%c%c\r\n",
                 (unsigned long)(current_time / 1000u),
                 (unsigned long)((current_time % 1000u) / 100u),
                 rs232_mode_name[UKB_RS232_GetMode()],
                 (unsigned)FlightSM_GetState(),
                 rocket_altitude,
                 avionics_health.sensor1_healthy ? baro1.pressure_hPa
                                                 : baro2.pressure_hPa,
                 ax, ay, az,
                 rocket_roll, rocket_pitch, rocket_yaw,
                 avionics_health.sensor1_healthy ? '1' : '-',
                 avionics_health.sensor2_healthy ? '2' : '-');

        hbst = (uint8_t)UKB_RS232_SendText(line);
#endif
    }

    // Skipped entirely while a ground test is running (SIT or SUT, including
    // the 1 s arming window). LoRa_TransmitTelemetry_Blocking waits on an AUX
    // handshake with a 200 ms timeout, which is the largest remaining stall in
    // the superloop and costs 10 Hz RS232 slots when it runs long. The radio
    // is not needed on the bench. On STOP, lora_last_tick is far in the past,
    // so normal telemetry resumes on the next iteration.
#if LORA_TX_ENABLED
    if (UKB_RS232_GetMode() == Test_Stop
        && current_time - lora_last_tick >= lora_interval) {
        lora_last_tick = current_time;

        myLora.packet.timestamp    = current_time;
        myLora.packet.flight_state = (uint8_t)FlightSM_GetState();
        myLora.packet.ax           = ax;
        myLora.packet.ay           = ay;
        myLora.packet.az           = az;
        myLora.packet.gy           = gy;
        myLora.packet.pitch        = rocket_pitch;
        myLora.packet.baro_alt_raw = (alt1 + alt2) * 0.5f; // simple pre-fusion peek
        myLora.packet.baro_alt     = rocket_altitude;
        myLora.packet.gps_lat      = myGPS.latitude_deg;
        myLora.packet.gps_lon      = myGPS.longitude_deg;

        lorast = LoRa_TransmitTelemetry_Blocking(&myLora, 200);
    }
#endif /* LORA_TX_ENABLED */

    if (current_time - print_last_tick >= 1000) {
        print_last_tick = current_time;

        DBG_PRINT("RS232: mode=%s sit=%s tx_status=%d hb_status=%d\r\n",
                  rs232_mode_name[UKB_RS232_GetMode()],
                  sit_active ? "streaming" : (sit_arming ? "arming" : "idle"),
                  sitst, hbst);

        DBG_PRINT("IMU: ax=%.2f ay=%.2f az=%.2f gy=%.2f pitch=%.1f roll=%.1f\r\n",
                  ax, ay, az, gy, rocket_pitch, rocket_roll);

        DBG_PRINT("Baro1: %s Pressure=%.2f hPa  Altitude=%.1f m\r\n",
                  avionics_health.sensor1_healthy ? "OK" : "FAULT", baro1.pressure_hPa, alt1);
        DBG_PRINT("Baro2: %s Pressure=%.2f hPa  Altitude=%.1f m\r\n",
                  avionics_health.sensor2_healthy ? "OK" : "FAULT", baro2.pressure_hPa, alt2);
        DBG_PRINT("Baro fault_count=%d\r\n", avionics_health.fault_count);

        DBG_PRINT("Alt: %.1f m  State: %d  Pitch: %.1f deg\r\n",rocket_altitude, FlightSM_GetState(), rocket_pitch);
        if (lorast != 0) {
            // 1=AUX busy  3=UART timeout  4=UART err  5=no AUX pulse(baud mismatch?)  6=AUX stuck
            DBG_PRINT("LoRa TX failed: code=%d\r\n", lorast);
        } else {
            DBG_PRINT("LoRa TX OK\r\n");
        }

    }

}

// Fires once per received byte on any UART with an active HAL_UART_Receive_IT
// (currently just USART2 / GPS). Re-arming for the next byte is handled inside
// NEO_M8N_RxCpltCallback itself.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    NEO_M8N_RxCpltCallback(&myGPS, huart);
}

// ====================================================================
// PHASE 3: HARDWARE-SAFE INTERRUPT ROUTING INTERFACES
// ====================================================================

/*
LoRa_E220* App_GetLora(void) {
    return &myLora;
}

void App_LoraDmaNotify(void) {
    if (HAL_UART_GetState(myLora.uartHandle) == HAL_UART_STATE_READY) {
        myLora.tx_busy = 0; // Clear software lockout on DMA completion
    }
}

void App_MpuDmaNotify(void) {
    if (HAL_I2C_GetState(myMPU6050.i2cHandle) == HAL_I2C_STATE_READY) {
        myMPU6050.dmaReady = true;
    }
}

void App_Baro1DmaNotify(void) {
    if (HAL_I2C_GetState(baro1.i2cHandle) == HAL_I2C_STATE_READY) {
        baro1.dmaReady = true;
    }
}

void App_Baro2DmaNotify(void) {
    if (HAL_I2C_GetState(baro2.i2cHandle) == HAL_I2C_STATE_READY) {
        baro2.dmaReady = true;
    }
}
*/