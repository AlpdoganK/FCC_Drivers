/*
 *
 *	BME280 Temperature, Humidity and Pressure sensor I2C Driver
 *
 *	Author: Alpdogan
 *	Created: 23 May 2026
 *
 */

#include "bme280.h"

// Private Helper Function Prototype to read calibration data
static HAL_StatusTypeDef BME280_ReadCalibrationData(BME280 *dev);

/**
 * @brief  Initialises the BME280 sensor with user configuration
 * @param  dev: Pointer to the BME280 handle structure
 * @param  i2cHandle: Pointer to the HAL I2C handle
 * @param  config: Pointer to the desired configuration settings
 * @retval 0 if successful, 1 if initialisation fails
 */
uint8_t BME280_Initialise(BME280 *dev, I2C_HandleTypeDef *i2cHandle, BME280_Config *config) {
    dev->i2cHandle = i2cHandle;
    dev->config = *config;
    
    uint8_t chip_id = 0;

	uint8_t errNum = 0;
	HAL_StatusTypeDef status;
    
    status = BME280_ReadRegister(dev, BME280_REG_ID, &chip_id);
    if (chip_id != 0x60) return 0xFF; // Wrong device or I2C fault
    
    // 2. Read Factory Calibration Parameters
    status = BME280_ReadCalibrationData(dev);
    if(status != HAL_OK) return 0xFF;
    
    // 3. Write Humidity Control Register (ctrl_hum: 0xF2)
    // Write to ctrl_hum must happen BEFORE writing to ctrl_meas to take effect.
    uint8_t ctrl_hum = (uint8_t)(dev->config.hum_osr);
    status = BME280_WriteRegister(dev, BME280_REG_CTRL_HUM, &ctrl_hum);
    if (status != HAL_OK) return 0xFF;
    
    // 4. Write Configuration Register (config: 0xF5) -> Standby time and IIR Filter
    uint8_t reg_config = (uint8_t)(dev->config.standby | dev->config.filter);
    status = BME280_WriteRegister(dev, BME280_REG_CONFIG, &reg_config);
    if (status != HAL_OK) return 0xFF;
    
    
    // 5. Write Control Measurement Register (ctrl_meas: 0xF4) -> Temp/Press oversampling & Mode
    // Writing here activates the changes made to ctrl_hum as well.
    uint8_t ctrl_meas = (uint8_t)((dev->config.temp_osr << 5) | (dev->config.press_osr << 2) | dev->config.mode);
    status = BME280_WriteRegister(dev, BME280_REG_CTRL_MEAS, &ctrl_meas);
    if (status != HAL_OK) return 0xFF;
    
    return errNum;
}

uint8_t BME280_ProcessDMA(BME280 *dev) {

    if (!dev->dmaReady) return 0xFF;

    uint8_t *data = dev->dmaBuf;
    
    // Extract raw variables
    int32_t adc_P = (int32_t)(((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4));
    int32_t adc_T = (int32_t)(((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | ((uint32_t)data[5] >> 4));
    int32_t adc_H = (int32_t)(((uint32_t)data[6] << 8)  | (uint32_t)data[7]);
    
    // --- 1. Temperature Calculation ---
    int32_t t_var1, t_var2;
    t_var1 = ((((adc_T >> 3) - ((int32_t)dev->calib.dig_T1 << 1))) * ((int32_t)dev->calib.dig_T2)) >> 11;
    t_var2 = (((((adc_T >> 4) - ((int32_t)dev->calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)dev->calib.dig_T1))) >> 12) * ((int32_t)dev->calib.dig_T3)) >> 14;
    dev->calib.t_fine = t_var1 + t_var2;
    dev->temp_C = (float)((dev->calib.t_fine * 5 + 128) >> 8) / 100.0f;
    
    // --- 2. Pressure Calculation ---
    int64_t p_var1, p_var2, p;
    p_var1 = ((int64_t)dev->calib.t_fine) - 128000;
    p_var2 = p_var1 * p_var1 * (int64_t)dev->calib.dig_P6;
    p_var2 = p_var2 + ((p_var1 * (int64_t)dev->calib.dig_P5) << 17);
    p_var2 = p_var2 + (((int64_t)dev->calib.dig_P4) << 35);
    p_var1 = ((p_var1 * p_var1 * (int64_t)dev->calib.dig_P3) >> 8) + ((p_var1 * (int64_t)dev->calib.dig_P2) << 12);
    p_var1 = (((((int64_t)1) << 47) + p_var1)) * ((int64_t)dev->calib.dig_P1) >> 33;
    
    if (p_var1 != 0) {
        p = 1048576 - adc_P;
        p = (((p << 31) - p_var2) * 3125) / p_var1;
        p_var1 = (((int64_t)dev->calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        p_var2 = (((int64_t)dev->calib.dig_P8) * p) >> 19;
        p = ((p + p_var1 + p_var2) >> 8) + (((int64_t)dev->calib.dig_P7) << 4);
        dev->pressure_hPa = (float)p / 25600.0f;
    }
    
    // --- 3. Humidity Calculation ---
    int32_t h_var1;
    h_var1 = (dev->calib.t_fine - ((int32_t)76800));
    h_var1 = (((((adc_H << 14) - (((int32_t)dev->calib.dig_H4) << 20) - (((int32_t)dev->calib.dig_H5) * h_var1)) +
                   ((int32_t)16384)) >> 15) * (((((((h_var1 * ((int32_t)dev->calib.dig_H6)) >> 10) * (((h_var1 * ((int32_t)dev->calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                   ((int32_t)dev->calib.dig_H2) + 8192) >> 14));
    h_var1 = (h_var1 - (((((h_var1 >> 15) * (h_var1 >> 15)) >> 7) * ((int32_t)dev->calib.dig_H1)) >> 4));
    h_var1 = (h_var1 < 0 ? 0 : h_var1);
    h_var1 = (h_var1 > 419430400 ? 419430400 : h_var1);
    dev->humidity_pct = (float)(h_var1 >> 12) / 1024.0f;

    dev->dmaReady = false; // Clear the flag until next DMA completion
    return 0; // Success
}

/**
 * @brief  Reads raw data, calculates and updates temperature in °C
 * @param  dev: Pointer to the BME280 handle structure
 * @retval HAL Status
 */
HAL_StatusTypeDef BME280_ReadTemperature(BME280 *dev) {
    uint8_t data[3];
    HAL_StatusTypeDef status = BME280_ReadRegisters(dev, BME280_REG_TEMP_MSB, data, 3);
    if (status != HAL_OK) return status;
    
    // Reconstruct 20-bit raw temperature data
    int32_t adc_T = (int32_t)(((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4));
    
    // Bosch Datasheet Compensation Formula for Temperature
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)dev->calib.dig_T1 << 1))) * ((int32_t)dev->calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dev->calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)dev->calib.dig_T1))) >> 12) * ((int32_t)dev->calib.dig_T3)) >> 14;
    
    dev->calib.t_fine = var1 + var2; // Store t_fine as it is required for Pressure and Humidity calculations
    
    dev->temp_C = (float)((dev->calib.t_fine * 5 + 128) >> 8) / 100.0f;
    
    return HAL_OK;
}

/**
 * @brief  Reads raw data, calculates and updates pressure in hPa
 * @param  dev: Pointer to the BME280 handle structure
 * @retval HAL Status
 */
HAL_StatusTypeDef BME280_ReadPressure(BME280 *dev) {
    // Temperature must be read first to get an updated dev->calib.t_fine variable
    status = BME280_ReadTemperature(dev);
    if (status != HAL_OK) return status;
    
    uint8_t data[3];
    HAL_StatusTypeDef status = BME280_ReadRegisters(dev, BME280_REG_PRESS_MSB, data, 3);
    if (status != HAL_OK) return status;
    
    // Reconstruct 20-bit raw pressure data
    int32_t adc_P = (int32_t)(((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4));
    
    // Bosch Datasheet Compensation Formula for Pressure (using 64-bit integers to prevent overflow)
    int64_t var1, var2, p;
    var1 = ((int64_t)dev->calib.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dev->calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)dev->calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)dev->calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dev->calib.dig_P3) >> 8) + ((var1 * (int64_t)dev->calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dev->calib.dig_P1) >> 33;
    
    if (var1 == 0) {
        dev->pressure_hPa = 0.0f; // Avoid division by zero exception
        return HAL_ERROR;
    }
    
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dev->calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dev->calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dev->calib.dig_P7) << 4);
    
    dev->pressure_hPa = (float)p / 25600.0f;
    
    return HAL_OK;
}

/**
 * @brief  Reads raw data, calculates and updates relative humidity in %
 * @param  dev: Pointer to the BME280 handle structure
 * @retval HAL Status
 */
HAL_StatusTypeDef BME280_ReadHumidity(BME280 *dev) {
    // Temperature must be read first to get an updated dev->calib.t_fine variable
    HAL_StatusTypeDef status = BME280_ReadTemperature(dev);
    if (status != HAL_OK) return status;

    uint8_t data[2];
    status = BME280_ReadRegisters(dev, BME280_REG_HUM_MSB, data, 2);
    if (status != HAL_OK) return status;
    
    // Reconstruct 16-bit raw humidity data
    int32_t adc_H = (int32_t)(((uint32_t)data[0] << 8) | (uint32_t)data[1]);
    
    // Bosch Datasheet Compensation Formula for Humidity
    int32_t v_x1_u32r;
    v_x1_u32r = (dev->calib.t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dev->calib.dig_H4) << 20) - (((int32_t)dev->calib.dig_H5) * v_x1_u32r)) +
                   ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dev->calib.dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)dev->calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                   ((int32_t)dev->calib.dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dev->calib.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    
    dev->humidity_pct = (float)(v_x1_u32r >> 12) / 1024.0f;
    
    return HAL_OK;
}

/**
 * @brief  Reads all sensor values via a single multi-byte I2C burst read
 * @param  dev: Pointer to the BME280 handle structure
 * @retval HAL Status
 */
HAL_StatusTypeDef BME280_ReadDMA(BME280 *dev) {
    if (HAL_I2C_GetState(dev->i2cHandle) != HAL_I2C_STATE_READY) {
        return HAL_BUSY;
    }
    
    return HAL_I2C_Mem_Read_DMA(dev->i2cHandle, BME280_ADDR, BME280_REG_PRESS_MSB, I2C_MEMADD_SIZE_8BIT, dev->dmaBuf, 8);
}

//	LOW-LEVEL FUNCTIONS

HAL_StatusTypeDef BME280_ReadRegister(BME280 *dev, uint8_t reg, uint8_t *data) {
    return HAL_I2C_Mem_Read(dev->i2cHandle, BME280_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef BME280_ReadRegisters(BME280 *dev, uint8_t reg, uint8_t *data, uint8_t length) {
    return HAL_I2C_Mem_Read(dev->i2cHandle, BME280_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef BME280_WriteRegister(BME280 *dev, uint8_t reg, uint8_t *data) {
    return HAL_I2C_Mem_Write(dev->i2cHandle, BME280_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

// PRIVATE STATIC HELPER

/**
 * @brief  Fills the internal struct with compensation parameters stored in the chip NVM
 * @param  dev: Pointer to the BME280 handle structure
 * @retval HAL Status
 */
static HAL_StatusTypeDef BME280_ReadCalibrationData(BME280 *dev) {
    uint8_t calib_1[26];
    uint8_t calib_2[7];
    
    // Read first block of calibration values (Reg 0x88 to 0xA1)
    HAL_StatusTypeDef status = BME280_ReadRegisters(dev, BME280_REG_CALIB00, calib_1, 26);
    if (status != HAL_OK) return status;
    
    // Read second block of calibration values (Reg 0xE1 to 0xE7)
    status = BME280_ReadRegisters(dev, BME280_REG_CALIB26, calib_2, 7);
    if (status != HAL_OK) return status;
    
    // Parse Temperature Trim
    dev->calib.dig_T1 = (uint16_t)((calib_1[1] << 8) | calib_1[0]);
    dev->calib.dig_T2 = (int16_t)((calib_1[3] << 8) | calib_1[2]);
    dev->calib.dig_T3 = (int16_t)((calib_1[5] << 8) | calib_1[4]);
    
    // Parse Pressure Trim
    dev->calib.dig_P1 = (uint16_t)((calib_1[7] << 8) | calib_1[6]);
    dev->calib.dig_P2 = (int16_t)((calib_1[9] << 8) | calib_1[8]);
    dev->calib.dig_P3 = (int16_t)((calib_1[11] << 8) | calib_1[10]);
    dev->calib.dig_P4 = (int16_t)((calib_1[13] << 8) | calib_1[12]);
    dev->calib.dig_P5 = (int16_t)((calib_1[15] << 8) | calib_1[14]);
    dev->calib.dig_P6 = (int16_t)((calib_1[17] << 8) | calib_1[16]);
    dev->calib.dig_P7 = (int16_t)((calib_1[19] << 8) | calib_1[18]);
    dev->calib.dig_P8 = (int16_t)((calib_1[21] << 8) | calib_1[20]);
    dev->calib.dig_P9 = (int16_t)((calib_1[23] << 8) | calib_1[22]);
    
    // Parse Humidity Trim (Requires custom unpacking due to shared 4-bit configurations)
    dev->calib.dig_H1 = calib_1[25];
    dev->calib.dig_H2 = (int16_t)((calib_2[1] << 8) | calib_2[0]);
    dev->calib.dig_H3 = calib_2[2];
    dev->calib.dig_H4 = (int16_t)((calib_2[3] << 4) | (calib_2[4] & 0x0F));
    dev->calib.dig_H5 = (int16_t)((calib_2[5] << 4) | (calib_2[4] >> 4));
    dev->calib.dig_H6 = (int8_t)calib_2[6];
    
    return HAL_OK;
}