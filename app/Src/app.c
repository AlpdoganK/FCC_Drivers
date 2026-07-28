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

// SUT (Sentetik Ucus Testi) scheduling. Spec 2.1.4c: same one-second arming
// delay as SIT. From then on the test device streams Tablo 4 synthetic sensor
// packets to us, and we stream the Tablo 6 status packet back at 10 Hz
// (2.1.4f) until STOP.
const uint32_t sut_arm_delay   = 1000; // 1 s between command and going live
const uint32_t sut_tx_interval = 100;  // 10 Hz status packets
// Tablo 7 "Test Timeout": if the test device goes quiet for this long mid-test
// the session is considered to have broken down. We keep the status stream up
// (so the ground software still sees a live link) but stop advancing the
// algorithm, rather than letting it coast on a stale sample.
const uint32_t sut_data_timeout = 1000;
static bool     sut_arming        = false;
static bool     sut_active        = false;
static uint32_t sut_arm_tick      = 0;
static uint32_t sut_last_tx_tick  = 0;
static uint32_t sut_last_data_tick = 0;
static bool     sut_data_stale    = false;
static uint32_t sut_sample_count  = 0;
static uint8_t  sutst             = 0;
// Last synthetic sample, kept for the debug console. The values that drive the
// algorithm are pushed straight into the shared ax/ay/az/rocket_* variables so
// that everything downstream is indifferent to where the data came from.
static float    sut_pressure_mbar = 0.0f;
// Latched peaks of the RAW inbound data, for reading over SWD after a run.
// These are what settle the units question: a profile whose peak acceleration
// magnitude is ~10 is in g, one peaking near ~100 is in m/s^2.
static float    sut_max_acc_raw   = 0.0f;
static float    sut_max_alt_seen  = 0.0f;
// The three components at the instant of peak magnitude, and the very first
// sample of the run. Together these say which axis the test device puts the
// longitudinal (thrust/drag) acceleration on and with what sign - which the
// magnitude alone cannot, and which EK-7 section 1.2 gets wrong.
static float    sut_peak_ax = 0.0f, sut_peak_ay = 0.0f, sut_peak_az = 0.0f;
static float    sut_first_ax = 0.0f, sut_first_ay = 0.0f, sut_first_az = 0.0f;

// Low-pass filters for the inbound synthetic stream (see UKB_SUT_FILTER_ENABLED).
// Re-initialised at the start of every SUT so one run cannot bias the next.
static LowPassFilter_t sut_lpf_alt;
static LowPassFilter_t sut_lpf_ax,   sut_lpf_ay,   sut_lpf_az;
static LowPassFilter_t sut_lpf_angx, sut_lpf_angy, sut_lpf_angz;

// ---- Inbound SUT acceleration units ---------------------------------------
// The units ARE m/s^2, exactly as EK-7's Tablo 2/4 say. Leave this at 0.
//
// Kept, with its history, because the wrong answer here is seductive. A single
// packet was decoded mid-flight showing an acceleration magnitude of 1.10, and
// that was read as "1 g expressed in g units". It was not: it was ~0.11 g in
// m/s^2, from a packet just after apogee, where a real rocket in ballistic
// descent genuinely is near-weightless. That is the exact condition
// flight_sm.c's own "kinematic weightlessness" apogee vote exists to detect.
//
// A full profile logged over SWD settles it: peak magnitude 221 m/s^2 (22.6 g,
// a sensible boost - 221 g would not be), and along the whole coast the Z
// component decays 18.4 -> 3.9 -> 0.6 m/s^2 as drag falls with airspeed, then
// builds again through the descent. That is a physically coherent flight in
// m/s^2 and nothing else.
//
// Moral: never infer units from one sample taken at an unknown flight phase.
#define UKB_SUT_ACCEL_IN_G 0
#define UKB_G_TO_MS2       9.80665f

// Map the test device's Z axis onto our longitudinal axis during SUT.
// See the block comment at the remap itself for the measured evidence.
#define UKB_SUT_SWAP_XZ 1

// "SUT owns the sensor data" - true from the moment the command is validated,
// not from when the 1 s arming window expires.
//
// The test device starts streaming immediately, so gating data intake on
// sut_active alone silently dropped the opening ~11 samples of every run.
// Measured against the real ground software: rx climbed by 12 accepted packets
// while sut_sample_count was still 0, and the first sample we actually
// processed was already at 113.9 m - a rocket at 221 m/s^2 covers ~110 m in
// that first second. We were joining the flight after liftoff. Here thrust
// outlasted the window so the boost was still detected, but on a short burn
// the liftoff event would be missed outright.
//
// EK-7 2.1.4c only says SUT mode "becomes active" one second after the
// command; it does not require throwing away data that has already arrived.
// The 1 s delay is still honoured for the outbound status stream, which is
// the part the ground software actually observes.
#define SUT_ENGAGED (sut_active || sut_arming)

// ---- Filtering of inbound SUT data ----------------------------------------
// EK-7 2.1.4d does not merely say to run the synthetic data through the flight
// algorithm - it says the data must be subjected to "filtreleme islemlerine ve
// ucus algoritmalarina", filtering operations AND flight algorithms. Feeding
// it in raw was therefore non-compliant, and the SUT_1 "Yuksek Gurultu"
// (high noise) scenario exists precisely to catch that.
//
// Symptom it produced: ATE (bit 4, altitude falling) and GAA (bit 3, body
// angle) latch in the order GAA -> ATE on the clean and normal-noise datasets,
// which is physically right - the rocket pitches past 45 degrees while still
// coasting over the top, before altitude has dropped a measurable 1.5 m. On
// the high-noise dataset the order INVERTED to ATE -> GAA, because a noise
// spike satisfied `altitude < alt_peak - 1.5f` long before any real descent.
// ATE latches off a single sample with no debounce, so one bad reading set it
// permanently and early.
//
// Firing was never affected - DESCENT_CONFIRM's five consecutive confirmations
// already guard the deployment decision - but the reported flag had no such
// protection, and the judges read the flags.
//
// A filter also protects apogee_tracker.alt_peak, which is a running MAXIMUM
// of altitude and only ever increases: one positive spike inflates it forever,
// after which the rocket must fall 1.5 m below an inflated peak, DELAYING the
// drogue. So in noise this should make firing earlier, not later.
//
// Alphas are deliberately light. LPF_Update is y += alpha*(x - y), so at
// alpha 0.5 and the device's 10 Hz the time constant is ~100 ms, one sample.
// Lag on the peak-to-current DIFFERENCE that drives baro_vote is smaller still,
// because both sides of the comparison are delayed together.
#define UKB_SUT_FILTER_ENABLED  1
#define UKB_SUT_LPF_ALPHA_ALT   0.5f
#define UKB_SUT_LPF_ALPHA_ACC   0.5f
#define UKB_SUT_LPF_ALPHA_ANG   0.5f

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
#define LORA_TX_ENABLED 1

// ---- I2C1 master switch (BENCH) -------------------------------------------
// I2C1 (PB6 SCL / PB7 SDA) carries the MPU6050 and BME280 #1. Set to 0 to skip
// that bus entirely: no bus recovery, no address scan, no sensor init, no
// runtime reads. BME280 #2 on I2C2 is untouched and still drives altitude.
//
// Why this exists: on the perf board the I2C1 pull-ups sit at 0 V - both SCL
// and SDA measured LOW with the peripheral idle, I2C1 CR1 stuck with START
// set, SR1 SB never setting, SR2 BUSY. Every blocking call in bme280.c and
// mpu6050.c passes HAL_MAX_DELAY, and I2C_WaitOnFlagUntilTimeout explicitly
// skips its timeout check for that value, so the first read never returns.
// That hangs App_Init before UKB_RS232_Init() ever runs - the board looks
// completely dead on RS232 when the actual fault is two sensor wires.
//
// Note the failure mode is not stable: an ABSENT device NACKs and fails fast,
// while a bus held LOW hangs forever. The same broken bus can therefore boot
// fine one day and hang the next, which is what happened here.
//
// SET BACK TO 1 FOR ANY BOARD WITH A WORKING I2C1: with this at 0 there is no
// IMU at all, so the accel and body-angle apogee votes never fire and the
// state machine is running on barometric data alone.
#define I2C1_ENABLED 1

#if RS232_HB_MODE == RS232_HB_UPATT
const uint32_t  rs232_hb_interval = 100;  // 10 Hz — near-continuous stream
#else
const uint32_t  rs232_hb_interval = 1000; // 1 Hz
#endif
static uint32_t rs232_hb_last_tick = 0;
static uint8_t  hbst               = 0;

// The Tablo 5 status bits now come from FlightSM_GetStatusBits(), which reads
// real latching per-event flags inside the state machine. This replaced an
// earlier version here that inferred them from how far the state enum had
// advanced; that could not represent GAA (a vote, not a state) and forced ATE
// and SPE to appear in the same instant even though the algorithm detects
// descent strictly before it commands the drogue.

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
    
#if I2C1_ENABLED
    I2C_BusRecover(hi2c1, GPIOB, GPIO_PIN_6, GPIOB, GPIO_PIN_7);
#else
    (void)hi2c1;
#endif
    I2C_BusRecover(hi2c2, GPIOB, GPIO_PIN_10, GPIOB, GPIO_PIN_3);

    // I2C bus scan — probe known addresses to verify buses are alive
#if I2C1_ENABLED
    DBG_PRINT("Scanning I2C1...\r\n");
    for (uint8_t addr = 1; addr < 128; addr++) {
        if (HAL_I2C_IsDeviceReady(hi2c1, (uint16_t)(addr << 1), 2, 10) == HAL_OK) {
            DBG_PRINT("  I2C1 device found at 0x%02X\r\n", addr);
        }
    }
#else
    DBG_PRINT("I2C1 disabled (I2C1_ENABLED=0) — skipping scan\r\n");
#endif
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

#if !I2C1_ENABLED
    (void)mpuConfig;
    DBG_PRINT("MPU6050 skipped: I2C1 disabled\r\n");
#else
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
#endif /* I2C1_ENABLED */

    // 2. Initialize Dual Independent BME280s across different I2C lines
    BME280_Config baroConfig = {
        .temp_osr = BME280_OVERSAMPLING_1X,
        .press_osr = BME280_OVERSAMPLING_4X,
        .hum_osr  = BME280_OVERSAMPLING_1X,
        .filter   = BME280_FILTER_8X,
        .mode     = BME280_MODE_NORMAL
    };

#if I2C1_ENABLED
    uint8_t b1_err = BME280_Initialise(&baro1, hi2c1, &baroConfig);
    if (b1_err != 0) {
        DBG_PRINT("Failed to initialize BME280 #1: err=0x%02X i2c_err=0x%08lX\r\n",
                  b1_err, hi2c1->ErrorCode);
        HAL_Delay(5000);
    } else {
        DBG_PRINT("BME280 #1 OK\r\n");
    }
#else
    // Non-zero reads as "failed init" everywhere downstream: the calibration
    // loop below skips baro1, and sensor1_healthy latches false so the fusion
    // and the SIT pressure field both fall through to baro2.
    const uint8_t b1_err = 0xFF;
    DBG_PRINT("BME280 #1 skipped: I2C1 disabled\r\n");
#endif

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

        sut_arming        = true;
        sut_active        = false;
        sut_arm_tick      = current_time;
        sut_data_stale    = false;
        sut_sample_count  = 0;
        sut_max_acc_raw   = 0.0f;
        sut_max_alt_seen  = 0.0f;
        sut_peak_ax = sut_peak_ay = sut_peak_az = 0.0f;
        sut_first_ax = sut_first_ay = sut_first_az = 0.0f;

        // Fresh filters per run. LPF_Update seeds prev_out from the first
        // sample (the `initialized` guard in filters.h), so there is no
        // start-up ramp from zero to worry about.
        LPF_Init(&sut_lpf_alt,  UKB_SUT_LPF_ALPHA_ALT);
        LPF_Init(&sut_lpf_ax,   UKB_SUT_LPF_ALPHA_ACC);
        LPF_Init(&sut_lpf_ay,   UKB_SUT_LPF_ALPHA_ACC);
        LPF_Init(&sut_lpf_az,   UKB_SUT_LPF_ALPHA_ACC);
        LPF_Init(&sut_lpf_angx, UKB_SUT_LPF_ALPHA_ANG);
        LPF_Init(&sut_lpf_angy, UKB_SUT_LPF_ALPHA_ANG);
        LPF_Init(&sut_lpf_angz, UKB_SUT_LPF_ALPHA_ANG);

        // Start every synthetic flight from the pad. EK-7 2.1.4k lets the
        // judges run SUT repeatedly with different profiles, and a state
        // machine still sitting in FLIGHT_LANDED from the previous run would
        // ignore the whole next one. Also clears the Tablo 5 status bits, so
        // the ground software's event panel starts blank.
        FlightSM_Init();
    }

    if (flag_stop_pending) {
        flag_stop_pending = 0;
        DBG_PRINT("RS232: STOP command received\r\n");

        // STOP is the protocol's single "return to normal" boundary, defined
        // identically for both scenarios (2.1.2g/h and 2.1.4i/j): drop every
        // test flag, stop transmitting, and be ready to start a fresh test.
        // Doing the whole teardown here means there is exactly one place that
        // has to be correct, and it is a place the test device always visits -
        // the ground software issues STOP at the end of every run.
        sit_arming     = false;
        sit_active     = false;
        sut_arming     = false;
        sut_active     = false;
        sut_data_stale = false;

        // A synthetic flight leaves the state machine wherever the profile
        // ended, normally FLIGHT_LANDED. Without this reset the board is inert
        // afterwards - the progression only runs forward, so it can never
        // detect a launch again. Measured on the bench: after a full SUT run
        // it sat in LANDED indefinitely and the next test did nothing.
        // Also clears the Tablo 5 status bits, so the ground software's event
        // panel starts blank for the next run.
        FlightSM_Init();

        // Drop the last synthetic IMU sample. These are module-scope and are
        // only overwritten when a real IMU read succeeds, so on a board whose
        // MPU6050 is absent or faulty the synthetic accel/angle values would
        // otherwise keep being reported as live sensor readings - observed
        // exactly that on the perf board, where SIT after a SUT run still
        // showed the profile's final 0.50/0.20/0.30 and 15 deg pitch.
        // Altitude is left alone: the next barometer sample owns it 50 ms later.
        ax = ay = az = 0.0f;
        gy = gz = 0.0f;
        rocket_pitch = rocket_roll = rocket_yaw = 0.0f;
    }

    // Spec 2.1.2c: hold off one second after the command before the first packet.
    if (sit_arming && current_time - sit_arm_tick >= sit_arm_delay) {
        sit_arming    = false;
        sit_active    = true;
        sit_last_tick = current_time - sit_tx_interval; // transmit immediately
        DBG_PRINT("RS232: SIT armed, streaming at 10 Hz\r\n");
    }

    // Spec 2.1.4c: same one-second hold-off before SUT goes live.
    if (sut_arming && current_time - sut_arm_tick >= sut_arm_delay) {
        sut_arming         = false;
        sut_active         = true;
        sut_last_tx_tick   = current_time - sut_tx_interval; // transmit immediately
        sut_last_data_tick = current_time; // don't trip the timeout instantly
        DBG_PRINT("RS232: SUT armed, status TX at 10 Hz\r\n");
    }

    if (flag_sut_data_ready) {
        flag_sut_data_ready = 0;

        UKB_SensorSample s;
        if (SUT_ENGAGED && UKB_ParseSensorPacket((const uint8_t *)ukb_sut_data, &s)) {
            sut_last_data_tick = current_time;
            sut_data_stale     = false;
            sut_sample_count++;
            sut_pressure_mbar  = s.pressure_mbar;

            // Spec 2.1.4d: the synthetic values REPLACE our own sensors. The
            // live IMU and barometer reads are skipped entirely while
            // SUT_ENGAGED (see PHASE 1), so nothing overwrites these before
            // the state machine sees them - including during the arming
            // window, where data is now processed rather than discarded.
            // Filtered before ANY use, so apogee_tracker.alt_peak tracks a
            // smoothed altitude rather than a running maximum of the noise.
#if UKB_SUT_FILTER_ENABLED
            rocket_altitude = LPF_Update(&sut_lpf_alt, s.altitude_m);
#else
            rocket_altitude = s.altitude_m;
#endif

            // Diagnostics, latched for reading over SWD after a run. Recorded
            // BEFORE the unit conversion below, so they show what the test
            // device actually put on the wire.
            float raw_mag = sqrtf(s.acc_x * s.acc_x + s.acc_y * s.acc_y +
                                  s.acc_z * s.acc_z);
            if (raw_mag > sut_max_acc_raw) {
                sut_max_acc_raw = raw_mag;
                sut_peak_ax = s.acc_x;
                sut_peak_ay = s.acc_y;
                sut_peak_az = s.acc_z;
            }
            if (s.altitude_m > sut_max_alt_seen) { sut_max_alt_seen = s.altitude_m; }
            if (sut_sample_count == 1u) {
                sut_first_ax = s.acc_x;
                sut_first_ay = s.acc_y;
                sut_first_az = s.acc_z;
            }

            // AXIS REMAP: the test device's longitudinal axis is Z, not X.
            //
            // EK-7 section 1.2 defines X as the longitudinal (nose) axis and Z
            // as vertical. The ground software does not follow its own spec:
            // it puts thrust and drag on Z and leaves X within +/-2 m/s^2 for
            // an entire flight. flight_sm.c reads ax for both the liftoff gate
            // (`ax > 24.5f`) and the burnout gate (`ax < 2.0f`), so unmapped it
            // sees a rocket that never moves - 1173 valid packets, zero
            // checksum errors, and not one status bit set.
            //
            // Logged over SWD from the real device, the first sample of a run
            // IS the peak (the dataset opens at full thrust, already at 113 m):
            //     (0.94, -3.00, +221.34) m/s^2   = 22.6 g on +Z
            // and through the coast Z runs -18.4 -> -3.9 -> -0.6 as drag decays
            // with airspeed, then rebuilds on the way down. Thrust is +Z,
            // deceleration is -Z: Z behaves exactly as our X is meant to.
            //
            // Swapping X and Z preserves the vector magnitude, so the apogee
            // "weightlessness" vote (net_g < 3.92) is unaffected either way.
            // The negative coast values are a bonus: `ax < 2.0f` now trips the
            // moment the motor quits instead of waiting for drag to fall below
            // 2 m/s^2 near apogee, which would have latched YSD very late.
            //
            // SUT-only. The real MPU6050 path keeps its own axis convention.
#if UKB_SUT_SWAP_XZ
            float in_ax = s.acc_z;   // longitudinal / thrust axis
            float in_ay = s.acc_y;
            float in_az = s.acc_x;
#else
            float in_ax = s.acc_x;
            float in_ay = s.acc_y;
            float in_az = s.acc_z;
#endif

            // The live path's accelerations arrive pre-filtered by the
            // MPU6050's own 44 Hz DLPF; the synthetic ones have no such
            // hardware stage, so they get one here.
#if UKB_SUT_FILTER_ENABLED
            ax = LPF_Update(&sut_lpf_ax, in_ax);
            ay = LPF_Update(&sut_lpf_ay, in_ay);
            az = LPF_Update(&sut_lpf_az, in_az);
#else
            ax = in_ax;
            ay = in_ay;
            az = in_az;
#endif

            // Bolum 1.2 body axes: X roll (longitudinal), Y pitch (lateral),
            // Z yaw (vertical). The live path smooths these through
            // Complementary_Update; with no gyro in Tablo 4 the low-pass is
            // the closest equivalent available here.
#if UKB_SUT_FILTER_ENABLED
            rocket_roll  = LPF_Update(&sut_lpf_angx, s.ang_x);
            rocket_pitch = LPF_Update(&sut_lpf_angy, s.ang_y);
            rocket_yaw   = LPF_Update(&sut_lpf_angz, s.ang_z);
#else
            rocket_roll  = s.ang_x;
            rocket_pitch = s.ang_y;
            rocket_yaw   = s.ang_z;
#endif

            // Tablo 4 carries no angular rates, so the apogee vote's
            // "rate reversed" half has nothing to work from and is fed 0.
            // The orientation vote still functions through its pitch-over
            // half, which reads the synthetic angle directly. Deriving a rate
            // by differencing consecutive angles was considered and rejected:
            // at 10 Hz with 2-decimal values it amplifies a one-degree step
            // into 10 deg/s, which is enough to swing a vote on noise alone.
            gy = 0.0f;

            // Drive the algorithm once per SAMPLE here, not once per superloop
            // iteration as the live-sensor path does below. DESCENT_CONFIRM in
            // flight_sm.c counts consecutive calls, so at loop rate its five
            // "confirmations" elapse in microseconds and provide no filtering
            // at all - one bad sample would fire the drogue. Per-sample, five
            // confirmations is 500 ms of sustained descent at the 10 Hz feed,
            // which is what that constant was written to mean.
            FlightSM_Update(rocket_altitude, ax, ay, az, rocket_pitch, gy);
        }
    }

    // Tablo 7 Test Timeout: the feed has dried up mid-test.
    if (sut_active && !sut_data_stale
        && current_time - sut_last_data_tick >= sut_data_timeout) {
        sut_data_stale = true;
        DBG_PRINT("RS232: SUT data timeout (%lu ms, %lu samples)\r\n",
                  (unsigned long)sut_data_timeout, (unsigned long)sut_sample_count);
    }

    // ====================================================================
    // PHASE 1: TIMED NON-BLOCKING TRIGGER REQUESTS
    // ====================================================================

    // Spec 2.1.4d: during SUT the board must ignore its own sensors and treat
    // the RS232 feed as its sensor input. Skipping the reads outright (rather
    // than reading and discarding) is both the literal requirement and free
    // superloop time - these are blocking I2C transfers.
    if (!SUT_ENGAGED && current_time - imu_last_tick >= imu_interval) {
        imu_last_tick = current_time;
        if (mpu_healthy && MPU6050_ReadAll(&myMPU6050) == HAL_OK) {
            myMPU6050.freshData = true;
        }
    }

    if (!SUT_ENGAGED && current_time - baro_last_tick >= baro_interval) {
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

    if (myMPU6050.freshData && !SUT_ENGAGED) {
        myMPU6050.freshData = false;

        // Update module-scope variables so FlightSM_Update always has fresh data
        ax = myMPU6050.acc_mps2[0]; // Nose Vector
        ay = myMPU6050.acc_mps2[1]; // Pitch Axis Lateral
        az = myMPU6050.acc_mps2[2]; // Yaw Axis Lateral

        float gx = myMPU6050.gyro[0]; // Roll Rate
        gy = myMPU6050.gyro[1];       // Pitch Rate — module-scope, FlightSM reads it
        gz = myMPU6050.gyro[2];       // Yaw Rate

        // Pitch is measured FROM VERTICAL: 0 = nose straight up, 90 =
        // horizontal, 180 = nose down. The +90 offset is what puts it on that
        // scale - atan2f alone returns -90 for a vertical rocket, and
        // flight_sm.c's pitch-over vote was silently always-true as a result
        // (see the note there). The SUT feed already uses this convention, so
        // both data paths now mean the same thing by "pitch".
        //
        // Consequence for the bench: a board lying flat now reports aci_y ~90
        // in SIT packets rather than ~0. Still well inside the +/-180 the
        // ground software accepts, and it is the physically honest number.
        float accel_pitch = 90.0f + atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
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

    if (baro1.freshData && !SUT_ENGAGED) {
        baro1.freshData = false;
        alt1 = 44330.0f * (1.0f - powf((baro1.pressure_hPa / sea_level_pressure), 0.1902949f));
        process_fusion = true;
    }

    if (baro2.freshData && !SUT_ENGAGED) {
        baro2.freshData = false;
        alt2 = 44330.0f * (1.0f - powf((baro2.pressure_hPa / sea_level_pressure), 0.1902949f));
        process_fusion = true;
    }

    if (process_fusion) {
        rocket_altitude = Baro_FusedAltitude(&avionics_health, &baro1_stats, &baro2_stats, alt1, alt2);
    }

    // ax/ay/az/gy are now module-scope — always valid here.
    //
    // Driven by FRESH BAROMETER DATA, not once per superloop iteration.
    // flight_sm.c's DESCENT_CONFIRM counts consecutive calls and is documented
    // as "~250ms", which is only true at one call per barometer sample. Called
    // from the loop instead, the five confirmations completed in tens of
    // microseconds while re-reading the SAME altitude thousands of times —
    // rocket_altitude only changes inside `if (process_fusion)` above. And
    // because the else-branch decrements at the same rate, the counter both
    // saturated and drained instantly, making it exactly equivalent to no
    // debounce at all.
    //
    // That mattered because the accel vote goes true long before apogee: on a
    // real test profile the specific force (drag only, once the motor is out)
    // decays through the 3.92 m/s^2 threshold at ~1929 m, roughly 1000 m below
    // a 2922 m apogee. From there up, a SINGLE barometric sample reading 1.5 m
    // below the running peak was enough to fire the drogue.
    //
    // Gating on process_fusion rather than on a 20 Hz timer is deliberate, and
    // is the safer of the two: if both barometers die, rocket_altitude freezes
    // at its last value, and a timer would go on feeding that stale altitude
    // to the state machine — if it happened to sit 1.5 m below alt_peak, the
    // counter would reach 5 in 250 ms and deploy on dead sensors. No new
    // sample, no count. It also avoids two 50 ms timers drifting out of phase
    // and delivering two calls per sample, or none.
    //
    // Consequence to be aware of: with no barometer data the state machine
    // stops advancing entirely, including the IMU-only PAD->BOOST->COAST
    // transitions. Apogee is unreachable without baro anyway (v.baro_vote is
    // mandatory), so nothing deployable is lost, but flight_state and the
    // Tablo 5 bits will freeze too.
    //
    // Not called at all during SUT: the barometer reads are skipped there, so
    // process_fusion is false, and the state machine is driven once per
    // received synthetic packet up in the flag_sut_data_ready block instead.
    if (!SUT_ENGAGED && process_fusion) {
        FlightSM_Update(rocket_altitude, ax, ay, az, rocket_pitch, gy);
    }

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

    // ====================================================================
    // SUT STATUS STREAM (Tablo 6, 10 Hz on USART6)
    // Spec 2.1.4f: while SUT runs, report which algorithm stage we are in.
    // This is the only thing the ground software has to judge the run by,
    // so it keeps flowing even when the synthetic feed has gone stale - a
    // silent link and a stalled test look identical from the other end,
    // and only one of them is our fault.
    // ====================================================================
    if (sut_active && current_time - sut_last_tx_tick >= sut_tx_interval) {
        // Advance by one whole interval rather than snapping to now, for the
        // same reason as the SIT stream: snapping turns loop latency into a
        // systematically slow rate (it measured ~8 Hz against a 10 Hz target).
        sut_last_tx_tick += sut_tx_interval;
        if (current_time - sut_last_tx_tick >= sut_tx_interval) {
            sut_last_tx_tick = current_time;
        }

        sutst = (uint8_t)UKB_RS232_SendStatusPacket(FlightSM_GetStatusBits());
    }

    // Idle link heartbeat — see RS232_HB_MODE. Suppressed during either test:
    // SIT has its own 10 Hz sensor stream and SUT its own 10 Hz status stream.
    if (!sit_active && !sut_active
        && current_time - rs232_hb_last_tick >= rs232_hb_interval) {
        rs232_hb_last_tick = current_time;

#if RS232_HB_MODE == RS232_HB_BINARY
        hbst = (uint8_t)UKB_RS232_SendStatusPacket(FlightSM_GetStatusBits());
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

        DBG_PRINT("SUT: %s samples=%lu rx=%lu cks_err=%lu bits=0x%04X "
                  "p=%.2fhPa tx_status=%d%s\r\n",
                  sut_active ? "running" : (sut_arming ? "arming" : "idle"),
                  (unsigned long)sut_sample_count,
                  (unsigned long)ukb_sut_rx_count,
                  (unsigned long)ukb_sut_cks_errors,
                  (unsigned)FlightSM_GetStatusBits(),
                  sut_pressure_mbar, sutst,
                  sut_data_stale ? "  [DATA STALE]" : "");

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