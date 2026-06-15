/*
 *
 *	MPU6050 Accelerometer and Gyroscope sensor I2C Driver
 *
 *	Author: Alpdogan
 *	Created: 25 October 2025
 *
 */

/*
 *	Last Edited: 23 May 2026
 *	Last Changes: Implemented DMA reads
 */

#include "mpu6050.h"

uint8_t MPU6050_Initialise( MPU6050 *dev, I2C_HandleTypeDef *i2cHandle, MPU6050_Config *config ){

	//setting struct params
	dev->i2cHandle = i2cHandle;
	dev->config = *config;

	dev->acc_mps2[0] = 0.0f;
	dev->acc_mps2[1] = 0.0f;
	dev->acc_mps2[2] = 0.0f;

	dev->gyro[0] = 0.0f;
	dev->gyro[1] = 0.0f;
	dev->gyro[2] = 0.0f;

	dev->temp_C = 0.0f;

	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	// WHO AM I CHECK
	uint8_t whoami;
	status = MPU6050_ReadRegister(dev, MPU6050_WHO_AM_I, &whoami);
	if (status != HAL_OK) return 0xFF;  // I2C transaction failed
	if (whoami != 0x68) return 0xFF;  // wrong device or I2C fault

	// CONFIGURATION

	uint8_t val = 0x00;
	status = MPU6050_WriteRegister(dev, MPU6050_PWR_MGMT_1, &val);
	if(status != HAL_OK) return 0xFF;

	val = dev->config.dlpf;
	status = MPU6050_WriteRegister(dev, MPU6050_CONFIG, &val);
	if(status != HAL_OK) return 0xFF;

	val = dev->config.gyro_fs;
	status = MPU6050_WriteRegister(dev, MPU6050_GYRO_CONFIG, &val);
	if(status != HAL_OK) return 0xFF;

	val = dev->config.accel_fs;
	status = MPU6050_WriteRegister(dev, MPU6050_ACCEL_CONFIG, &val);
	if(status != HAL_OK) return 0xFF;

	return errNum;

}

/*uint8_t MPU6050_ProcessDMA(MPU6050 *dev){

	if (!dev->dmaReady) return 0xFF; // No new data

	uint8_t *d = dev->dmaBuf;

    // Register map order: AX AY AZ TEMP GX GY GZ (each 2 bytes, big-endian)
    int16_t axRaw = (int16_t)((d[0]  << 8) | d[1]);
    int16_t ayRaw = (int16_t)((d[2]  << 8) | d[3]);
    int16_t azRaw = (int16_t)((d[4]  << 8) | d[5]);
    // d[6], d[7] = temperature (skip)
    int16_t gxRaw = (int16_t)((d[8]  << 8) | d[9]);
    int16_t gyRaw = (int16_t)((d[10] << 8) | d[11]);
    int16_t gzRaw = (int16_t)((d[12] << 8) | d[13]);

    float lsb_g, lsb_dps;

    switch (dev->config.accel_fs) {
        case MPU6050_ACCEL_FS_2G:  lsb_g = 16384.0f; break;
        case MPU6050_ACCEL_FS_4G:  lsb_g =  8192.0f; break;
        case MPU6050_ACCEL_FS_8G:  lsb_g =  4096.0f; break;
        case MPU6050_ACCEL_FS_16G: lsb_g =  2048.0f; break;
    }
    switch (dev->config.gyro_fs) {
        case MPU6050_GYRO_FS_250:  lsb_dps = 131.0f; break;
        case MPU6050_GYRO_FS_500:  lsb_dps =  65.5f; break;
        case MPU6050_GYRO_FS_1000: lsb_dps =  32.8f; break;
        case MPU6050_GYRO_FS_2000: lsb_dps =  16.4f; break;
    }

    dev->acc_mps2[0] = (axRaw / lsb_g) * 9.80665f;
    dev->acc_mps2[1] = (ayRaw / lsb_g) * 9.80665f;
    dev->acc_mps2[2] = (azRaw / lsb_g) * 9.80665f;

    dev->gyro[0] = gxRaw / lsb_dps;
    dev->gyro[1] = gyRaw / lsb_dps;
    dev->gyro[2] = gzRaw / lsb_dps;

	return 0; // Success
}*/

HAL_StatusTypeDef MPU6050_ReadAll(MPU6050 *dev) {
    uint8_t buf[14];

    HAL_StatusTypeDef status = MPU6050_ReadRegisters(dev, MPU6050_ACCEL_XOUT_H, buf, 14);
    if (status != HAL_OK) return status;

    int16_t axRaw = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t ayRaw = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t azRaw = (int16_t)((buf[4]  << 8) | buf[5]);
    int16_t tRaw  = (int16_t)((buf[6]  << 8) | buf[7]);
    int16_t gxRaw = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t gyRaw = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gzRaw = (int16_t)((buf[12] << 8) | buf[13]);

    float lsb_g = 16384.0f, lsb_dps = 131.0f;
    switch (dev->config.accel_fs) {
        case MPU6050_ACCEL_FS_2G:  lsb_g = 16384.0f; break;
        case MPU6050_ACCEL_FS_4G:  lsb_g =  8192.0f; break;
        case MPU6050_ACCEL_FS_8G:  lsb_g =  4096.0f; break;
        case MPU6050_ACCEL_FS_16G: lsb_g =  2048.0f; break;
    }
    switch (dev->config.gyro_fs) {
        case MPU6050_GYRO_FS_250:  lsb_dps = 131.0f; break;
        case MPU6050_GYRO_FS_500:  lsb_dps =  65.5f; break;
        case MPU6050_GYRO_FS_1000: lsb_dps =  32.8f; break;
        case MPU6050_GYRO_FS_2000: lsb_dps =  16.4f; break;
    }

    dev->acc_mps2[0] = (axRaw / lsb_g) * 9.80665f;
    dev->acc_mps2[1] = (ayRaw / lsb_g) * 9.80665f;
    dev->acc_mps2[2] = (azRaw / lsb_g) * 9.80665f;
    dev->gyro[0] = gxRaw / lsb_dps;
    dev->gyro[1] = gyRaw / lsb_dps;
    dev->gyro[2] = gzRaw / lsb_dps;
    dev->temp_C  = (tRaw / 340.0f) + 36.53f;

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadTemperature( MPU6050 *dev ){

	uint8_t regData[2];

	HAL_StatusTypeDef status = MPU6050_ReadRegisters( dev, MPU6050_TEMP_OUT_H, regData, 2);

	int16_t tempRaw = (int16_t)((regData[0] << 8) | regData[1]);

	dev->temp_C = (tempRaw / 340.0f) + 36.53f;

	return status;
}

HAL_StatusTypeDef MPU6050_ReadAcceleration( MPU6050 *dev ){

	uint8_t regData[6];

	float lsb_per_g;
	switch (dev->config.accel_fs) {
    	case MPU6050_ACCEL_FS_2G:  lsb_per_g = 16384.0f; break;
    	case MPU6050_ACCEL_FS_4G:  lsb_per_g =  8192.0f; break;
    	case MPU6050_ACCEL_FS_8G:  lsb_per_g =  4096.0f; break;
    	case MPU6050_ACCEL_FS_16G: lsb_per_g =  2048.0f; break;
	}

	HAL_StatusTypeDef status = MPU6050_ReadRegisters( dev, MPU6050_ACCEL_XOUT_H, regData, 6);

	int16_t accRaw[3];

	accRaw[0] = (int16_t)((regData[0] << 8) | regData[1]);
	accRaw[1] = (int16_t)((regData[2] << 8) | regData[3]);
	accRaw[2] = (int16_t)((regData[4] << 8) | regData[5]);
	dev->acc_mps2[0] = (accRaw[0] / lsb_per_g) * 9.80665f;
	dev->acc_mps2[1] = (accRaw[1] / lsb_per_g) * 9.80665f;
	dev->acc_mps2[2] = (accRaw[2] / lsb_per_g) * 9.80665f;

	return status;
}

HAL_StatusTypeDef MPU6050_ReadGyro( MPU6050 *dev ){

	uint8_t regData[6];

	float lsb_per_dps;
	switch (dev->config.gyro_fs) {
    	case MPU6050_GYRO_FS_250:  lsb_per_dps = 131.0f;  break;
    	case MPU6050_GYRO_FS_500:  lsb_per_dps = 65.5f;   break;
    	case MPU6050_GYRO_FS_1000: lsb_per_dps = 32.8f;   break;
    	case MPU6050_GYRO_FS_2000: lsb_per_dps = 16.4f;   break;
	}


	HAL_StatusTypeDef status = MPU6050_ReadRegisters( dev, MPU6050_GYRO_XOUT_H, regData, 6);

	int16_t gyroRaw[3];

	gyroRaw[0] = (int16_t)((regData[0] << 8) | regData[1]);
	gyroRaw[1] = (int16_t)((regData[2] << 8) | regData[3]);
	gyroRaw[2] = (int16_t)((regData[4] << 8) | regData[5]);

	dev->gyro[0] = gyroRaw[0] / lsb_per_dps;   // degrees/s
	dev->gyro[1] = gyroRaw[1] / lsb_per_dps;
	dev->gyro[2] = gyroRaw[2] / lsb_per_dps;

	return status;
}

/*HAL_StatusTypeDef MPU6050_ReadGyroAccel_DMA(MPU6050 *dev){
	if (HAL_I2C_GetState(dev->i2cHandle) != HAL_I2C_STATE_READY) {
        return HAL_BUSY;
    }

	return HAL_I2C_Mem_Read_DMA( dev->i2cHandle, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, dev->dmaBuf, 14);
}*/

//	LOW-LEVEL FUNCTIONS

HAL_StatusTypeDef MPU6050_ReadRegister( MPU6050 *dev, uint8_t reg, uint8_t *data){
	
	return HAL_I2C_Mem_Read( dev->i2cHandle, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU6050_ReadRegisters( MPU6050 *dev, uint8_t reg, uint8_t *data, uint8_t length){

	return HAL_I2C_Mem_Read( dev->i2cHandle, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU6050_WriteRegister( MPU6050 *dev, uint8_t reg, uint8_t *data){

	return HAL_I2C_Mem_Write( dev->i2cHandle, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}
