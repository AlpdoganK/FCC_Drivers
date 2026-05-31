#include "app.h"
#include <math.h>
#include <stdbool.h>

#define RAD_TO_DEG 57.295779513f

// Sensor & Filter Instances
static MPU6050 myMPU6050; 
static BME280 baro1; 
static BME280 baro2;
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

// Local Telemetry Tracks
float rocket_pitch = 0.0f;
float rocket_roll  = 0.0f;
float rocket_altitude = 0.0f;
float sea_level_pressure = 1013.25f;

// Peak tracking variables for apogee detection
static float max_altitude = 0.0f;
static uint32_t apogee_timer = 0;
static uint32_t landing_timer = 0;


void app_init(I2C_HandleTypeDef *hi2c1, I2C_HandleTypeDef *hi2c2, 
              UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2, UART_HandleTypeDef *huart6) {
    
    // 1. Initialize MPU6050 on I2C Bus 1
    MPU6050_Config mpuConfig = {
        .accel_fs = MPU6050_ACCEL_FS_16G, // Handing massive launch forces safely
        .gyro_fs = MPU6050_GYRO_FS_2000,   // High rate spin allowance
        .dlpf = MPU6050_DLPF_42HZ         // On-board vibration suppression
    };

    if (MPU6050_Initialise(&myMPU6050, hi2c1, &mpuConfig) != 0) {
        while(1); // Trap on IMU hardware failure
    }

    // 2. Initialize Dual Independent BME280s across different I2C lines
    BME280_Config baroConfig = { .temp_osr = 1, .press_osr = 4, .hum_osr = 1, .filter = 3, .mode = 3 };
    
    if (BME280_Initialise(&baro1, hi2c1, &baroConfig) != 0) while(1);
    if (BME280_Initialise(&baro2, hi2c2, &baroConfig) != 0) while(1);

    // 3. Setup Redundant Fault Latches & Running Statistics Matrices
    avionics_health.sensor1_healthy = 1;
    avionics_health.sensor2_healthy = 1;
    avionics_health.fault_count = 0;

    baro1_stats.mean = 0.0f; baro1_stats.variance = 1.0f; baro1_stats.alpha = 0.05f; baro1_stats.mean_beta = 0.05f;
    baro2_stats.mean = 0.0f; baro2_stats.variance = 1.0f; baro2_stats.alpha = 0.05f; baro2_stats.mean_beta = 0.05f;

    Complementary_Init(&pitch_cf, 0.95f, 0.01f, 0.0f);
    Complementary_Init(&roll_cf,  0.95f, 0.01f, 0.0f);
    
    imu_last_tick  = HAL_GetTick();
    baro_last_tick = HAL_GetTick();

    FlightSM_Init();
}

void App_Run(void) {
    uint32_t current_time = HAL_GetTick();

    Pyro_ProcessTimeouts();
    
    // ====================================================================
    // PHASE 1: TIMED NON-BLOCKING TRIGGER REQUESTS
    // ====================================================================

    // Trigger IMU Asynchronous DMA Read (100 Hz Loop)
    if (current_time - imu_last_tick >= imu_interval) {
        imu_last_tick = current_time;
        
        // This initiates the read background transfer and returns immediately.
        // It fails gracefully if Bus 1 is busy reading Baro 1.
        MPU6050_ReadGyroAccel_DMA(&myMPU6050); 
    }

    // Trigger Dual Barometer Asynchronous DMA Reads (20 Hz Loop)
    if (current_time - baro_last_tick >= baro_interval) {
        baro_last_tick = current_time;

        if (avionics_health.sensor1_healthy) {
            BME280_ReadDMA(&baro1); // Runs background DMA on I2C 1
        }
        if (avionics_health.sensor2_healthy) {
            BME280_ReadDMA(&baro2); // Runs background DMA on I2C 2
        }
    }

    // ====================================================================
    // PHASE 2: ASYNCHRONOUS BACKGROUND COMPLETION CHECKS
    // ====================================================================

    // 1. Check if IMU background transfer has completed
    if (myMPU6050.dmaReady) {
        myMPU6050.dmaReady = false; // Acknowledge event
        
        MPU6050_ProcessDMA(&myMPU6050);

        float ax = myMPU6050.acc_mps2[0]; // Nose Vector
        float ay = myMPU6050.acc_mps2[1]; // Pitch Axis Lateral
        float az = myMPU6050.acc_mps2[2]; // Yaw Axis Lateral

        float gx = myMPU6050.gyro[0];     // Roll Rate
        float gy = myMPU6050.gyro[1];     // Pitch Rate

        // Acceleration Vector Angle Computations for X-Axis Forward configuration
        float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
        float accel_roll  = atan2f(ay, az) * RAD_TO_DEG;

        // Execute Sensor Fusion Equations
        rocket_pitch = Complementary_Update(&pitch_cf, accel_pitch, gy);
        rocket_roll  = Complementary_Update(&roll_cf,  accel_roll,  gx);
    }

    // Local state trackers inside safe processing loop scope
    static float alt1 = 0.0f;
    static float alt2 = 0.0f;
    bool process_fusion = false;

    // 2. Check Barometer 1 Background Completion
    if (baro1.dmaReady) {
        baro1.dmaReady = false;
        BME280_ProcessDMA(&baro1);
        alt1 = 44330.0f * (1.0f - powf((baro1.pressure_hPa / sea_level_pressure), 0.1902949f));
        process_fusion = true;
    }

    // 3. Check Barometer 2 Background Completion
    if (baro2.dmaReady) {
        baro2.dmaReady = false;
        BME280_ProcessDMA(&baro2);
        alt2 = 44330.0f * (1.0f - powf((baro2.pressure_hPa / sea_level_pressure), 0.1902949f));
        process_fusion = true;
    }
    
    // 4. Run Fusion Module if new altitude updates arrived
    if (process_fusion) {
        // Runs Inverse-variance weighting fusion + software fault isolation
        rocket_altitude = Baro_FusedAltitude(&avionics_health, &baro1_stats, &baro2_stats, alt1, alt2);
    }

    FlightSM_Update(rocket_altitude, ax, ay, az, rocket_pitch, gy);
}

// ====================================================================
// PHASE 3: HARWARE-SAFE INTERRUPT ROUTING INTERFACES
// ====================================================================

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