#include "app.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

#define RAD_TO_DEG 57.295779513f

// Sensor & Filter Instances
static MPU6050 myMPU6050;
static BME280 baro1;
static BME280 baro2;
static LoRa_E220 myLora;
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
          
    // 1. Initialize MPU6050 on I2C Bus 1
    MPU6050_Config mpuConfig = {
        .accel_fs = MPU6050_ACCEL_FS_16G,
        .gyro_fs  = MPU6050_GYRO_FS_2000,
        .dlpf     = MPU6050_DLPF_44HZ
    };

    if (MPU6050_Initialise(&myMPU6050, hi2c2, &mpuConfig) != 0) {
    }

    // 2. Initialize Dual Independent BME280s across different I2C lines
    BME280_Config baroConfig = { .temp_osr = 1, .press_osr = 4, .hum_osr = 1, .filter = 3, .mode = 3 };


    if (BME280_Initialise(&baro1, hi2c1, &baroConfig) != 0) {
    }
    if (BME280_Initialise(&baro2, hi2c2, &baroConfig) != 0) {
    } 

    LoRa_Init(&myLora, huart1, LORA_AUX_GPIO_Port, LORA_AUX_Pin);

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

    FlightSM_Init();
}

void App_Run(void) {
    uint32_t current_time = HAL_GetTick();
    Pyro_ProcessTimeouts();

    // ====================================================================
    // PHASE 1: TIMED NON-BLOCKING TRIGGER REQUESTS
    // ====================================================================

    if (current_time - imu_last_tick >= imu_interval) {
        imu_last_tick = current_time;
        MPU6050_ReadGyroAccel_DMA(&myMPU6050);
    }

    if (current_time - baro_last_tick >= baro_interval) {
        baro_last_tick = current_time;

        if (avionics_health.sensor1_healthy) BME280_ReadDMA(&baro1);
        if (avionics_health.sensor2_healthy) BME280_ReadDMA(&baro2);
    }

    // ====================================================================
    // PHASE 2: ASYNCHRONOUS BACKGROUND COMPLETION CHECKS
    // ====================================================================

    if (myMPU6050.dmaReady) {
        myMPU6050.dmaReady = false;

        MPU6050_ProcessDMA(&myMPU6050);

        // Update module-scope variables so FlightSM_Update always has fresh data
        ax = myMPU6050.acc_mps2[0]; // Nose Vector
        ay = myMPU6050.acc_mps2[1]; // Pitch Axis Lateral
        az = myMPU6050.acc_mps2[2]; // Yaw Axis Lateral

        float gx = myMPU6050.gyro[0]; // Roll Rate
        gy        = myMPU6050.gyro[1]; // Pitch Rate

        float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
        float accel_roll  = atan2f(ay, az) * RAD_TO_DEG;

        rocket_pitch = Complementary_Update(&pitch_cf, accel_pitch, gy);
        rocket_roll  = Complementary_Update(&roll_cf,  accel_roll,  gx);
    }

    static float alt1 = 0.0f;
    static float alt2 = 0.0f;
    bool process_fusion = false;

    if (baro1.dmaReady) {
        baro1.dmaReady = false;
        BME280_ProcessDMA(&baro1);
        alt1 = 44330.0f * (1.0f - powf((baro1.pressure_hPa / sea_level_pressure), 0.1902949f));
        process_fusion = true;
    }

    if (baro2.dmaReady) {
        baro2.dmaReady = false;
        BME280_ProcessDMA(&baro2);
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
    myLora.packet.gps_lat      = 0.0f;                  // TODO: wire NEO-M8N parser
    myLora.packet.gps_lon      = 0.0f;                  // TODO: wire NEO-M8N parser

    uint8_t lorast = LoRa_TransmitTelemetry_Blocking(&myLora, 200);
    if (lorast != 0) {  
    } /*else {
        HAL_GPIO_WritePin(PYRO2_GPIO_Port, PYRO2_Pin, GPIO_PIN_SET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(PYRO2_GPIO_Port, PYRO2_Pin, GPIO_PIN_RESET);
        HAL_Delay(100);
    }*/
}

}

// ====================================================================
// PHASE 3: HARDWARE-SAFE INTERRUPT ROUTING INTERFACES
// ====================================================================

LoRa_E220* App_GetLora(void) {
    HAL_GPIO_WritePin(PYRO1_GPIO_Port, PYRO1_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);
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