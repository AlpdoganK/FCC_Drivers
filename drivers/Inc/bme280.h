/*
 *
 *	BME280 Temperature, Humidity and Pressure sensor I2C Driver
 *
 *	Author: Alpdogan
 *	Created: 22 May 2026
 *
 */

#ifndef BME280_I2C_DRIVER_H
#define BME280_I2C_DRIVER_H

#include "stm32f4xx_hal.h" //needed for I2C
#include <stdbool.h>

//	DEFINES

/* * BME280 I2C Address depends on SDO pin state:
 * SDO to GND -> 0x76 (Left shifted: 0xEC)
 * SDO to VDDIO -> 0x77 (Left shifted: 0xEE)
 */
#define BME280_ADDR (0x76 << 1) 
#define BME280_RESET_VALUE  0xB6

//	REGISTERS

#define BME280_REG_CALIB00		  	0x88 // Calibration data registers start
#define BME280_REG_CALIB25		  	0xA1
#define BME280_REG_ID				0xD0 // Chip ID Register (Should return 0x60)
#define BME280_REG_RESET			0xE0 // Soft Reset Register
#define BME280_REG_CALIB26		  	0xE1
#define BME280_REG_CTRL_HUM			0xF2 // Humidity Options Control Register
#define BME280_REG_STATUS			0xF3 // Status Register (measuring/updating)
#define BME280_REG_CTRL_MEAS		0xF4 // Pressure/Temp Options and Mode Control
#define BME280_REG_CONFIG			0xF5 // Rate, Filter and Interface Options
#define BME280_REG_PRESS_MSB		0xF7 // Data registers start
#define BME280_REG_PRESS_LSB		0xF8
#define BME280_REG_PRESS_XLSB		0xF9
#define BME280_REG_TEMP_MSB			0xFA
#define BME280_REG_TEMP_LSB			0xFB
#define BME280_REG_TEMP_XLSB		0xFC
#define BME280_REG_HUM_MSB			0xFD
#define BME280_REG_HUM_LSB			0xFE

// CONFIGURATION ENUMS AND STRUCT

typedef enum {
    BME280_OVERSAMPLING_SKIPPED = 0x00,
    BME280_OVERSAMPLING_1X      = 0x01,
    BME280_OVERSAMPLING_2X      = 0x02,
    BME280_OVERSAMPLING_4X      = 0x03,
    BME280_OVERSAMPLING_8X      = 0x04,
    BME280_OVERSAMPLING_16X     = 0x05
} BME280_Oversampling;

typedef enum {
    BME280_MODE_SLEEP  = 0x00,
    BME280_MODE_FORCED = 0x01,
    BME280_MODE_NORMAL = 0x03
} BME280_Mode;

typedef enum {
    BME280_FILTER_OFF = 0x00,
    BME280_FILTER_2X  = 0x04,
    BME280_FILTER_4X  = 0x08,
    BME280_FILTER_8X  = 0x0C,
    BME280_FILTER_16X = 0x10
} BME280_Filter;

typedef enum {
    BME280_STANDBY_0_5MS  = 0x00,
    BME280_STANDBY_62_5MS = 0x20,
    BME280_STANDBY_125MS  = 0x40,
    BME280_STANDBY_250MS  = 0x60,
    BME280_STANDBY_500MS  = 0x80,
    BME280_STANDBY_1000MS = 0xA0,
    BME280_STANDBY_10MS   = 0xC0,
    BME280_STANDBY_20MS   = 0xE0
} BME280_StandbyTime;

typedef struct {
    BME280_Oversampling press_osr; // Pressure oversampling
    BME280_Oversampling temp_osr;  // Temperature oversampling
    BME280_Oversampling hum_osr;   // Humidity oversampling
    BME280_Mode           mode;    // Sensor mode (Sleep, Forced, Normal)
    BME280_Filter       filter;    // IIR Filter coefficient
    BME280_StandbyTime  standby;   // Standby time in Normal mode
} BME280_Config;

// BME280 CALIBRATION DATA STRUCT

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
    int32_t  t_fine; // Used internally for pressure/humidity math
} BME280_CalibData;

//	SENSOR STRUCT

typedef struct {

	I2C_HandleTypeDef *i2cHandle;
	BME280_Config config;        // Sensor configuration settings
	BME280_CalibData calib;      // Trim parameters read upon initialization

	float temp_C;                // Temperature data in Celcius
	float pressure_hPa;          // Atmospheric pressure in hPa (hectopascals)
	float humidity_pct;          // Relative humidity in %

    // DMA buffer and flag for asynchronous data acquisition
    uint8_t dmaBuf[8] __attribute__((aligned(4)));
    volatile bool dmaReady;

} BME280;

//	INITIALISATION

uint8_t BME280_Initialise( BME280 *dev, I2C_HandleTypeDef *i2cHandle, BME280_Config *config );
uint8_t BME280_ProcessDMA(BME280 *dev);
//	DATA ACQUISITION

HAL_StatusTypeDef BME280_ReadTemperature( BME280 *dev );
HAL_StatusTypeDef BME280_ReadPressure( BME280 *dev );
HAL_StatusTypeDef BME280_ReadHumidity( BME280 *dev );
HAL_StatusTypeDef BME280_ReadDMA( BME280 *dev );

//	LOW-LEVEL FUNCTIONS

HAL_StatusTypeDef BME280_ReadRegister( BME280 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef BME280_ReadRegisters( BME280 *dev, uint8_t reg, uint8_t *data, uint8_t length);

HAL_StatusTypeDef BME280_WriteRegister( BME280 *dev, uint8_t reg, uint8_t *data);

#endif /* BME280_I2C_DRIVER_H */