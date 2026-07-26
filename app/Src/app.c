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

// Sensor & Filter Instances
static MPU6050 myMPU6050;
static BME280 baro1;
static BME280 baro2;
static LoRa_E220 myLora;
static NEO_M8N myGPS;
static SensorStats_t baro1_stats;
static SensorStats_t baro2_stats;
static BaroHealth_t  avionics_health;
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

// Local Telemetry Tracks
float rocket_pitch    = 0.0f;
float rocket_roll     = 0.0f;
float rocket_altitude = 0.0f;
float sea_level_pressure = 1013.25f;

// IMU data — module-scope so FlightSM_Update can see them
static float ax = 0.0f, ay = 0.0f, az = 0.0f;
static float gy = 0.0f;

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
        HAL_Delay(5000);
    } else {
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

    LoRa_Init(&myLora, huart1, LORA_AUX_GPIO_Port, LORA_AUX_Pin);
    NEO_M8N_Init(&myGPS, huart2); // 5 Hz fix rate, GGA-only NMEA output
    (void)huart6; // UKB_RS232_Init() below talks to huart6 directly
    UKB_RS232_Init();

    // 3. Setup Redundant Fault Latches & Running Statistics Matrices
    avionics_health.sensor1_healthy = 1;
    avionics_health.sensor2_healthy = 1;
    avionics_health.fault_count     = 0;

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
    NEO_M8N_Process(&myGPS); // parse any GPS line buffered by the RX ISR since last loop

    // ====================================================================
    // PHASE 0: RS232 GROUND-TEST COMMAND HANDLING (USART6)
    // rs232.c only validates frames and raises flags from ISR context;
    // all the actual test-mode behavior is decided here.
    // ====================================================================
    if (flag_sit_pending) {
        flag_sit_pending = 0;
        // TODO: arm 1 s SIT timer, start 10 Hz status TX per Tablo spec
    }

    if (flag_sut_pending) {
        flag_sut_pending = 0;
        // TODO: arm 1 s SUT timer, start 10 Hz status TX per Tablo spec
    }

    if (flag_stop_pending) {
        flag_stop_pending = 0;
        // TODO: tear down whatever SIT/SUT started, return to normal flight-computer operation
    }

    if (flag_sut_data_ready) {
        flag_sut_data_ready = 0;
        // TODO: feed ukb_sut_data[0..ukb_sut_data_len) (Tablo 4 synthetic
        // sensor packet) into the algorithm under test instead of live sensors
    }

    // ====================================================================
    // PHASE 1: TIMED NON-BLOCKING TRIGGER REQUESTS
    // ====================================================================

    if (current_time - imu_last_tick >= imu_interval) {
        imu_last_tick = current_time;
        if (MPU6050_ReadAll(&myMPU6050) == HAL_OK) {
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
        float gy = myMPU6050.gyro[1]; // Pitch Rate

        float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
        float accel_roll  = atan2f(ay, az) * RAD_TO_DEG;

        rocket_pitch = Complementary_Update(&pitch_cf, accel_pitch, gy);
        rocket_roll  = Complementary_Update(&roll_cf,  accel_roll,  gx);
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

    if (current_time - lora_last_tick >= lora_interval) {
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

    if (current_time - print_last_tick >= 1000) {
        print_last_tick = current_time;
        DBG_PRINT("Baro1: Pressure=%.2f hPa  Altitude=%.1f m\r\n", baro1.pressure_hPa, alt1);
        DBG_PRINT("Baro2: Pressure=%.2f hPa  Altitude=%.1f m\r\n", baro2.pressure_hPa, alt2);
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