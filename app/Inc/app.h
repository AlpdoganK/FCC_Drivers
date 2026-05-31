#ifndef APP_H_
#define APP_H_

#include "mpu6050.h"
#include "bme280.h"
#include "e220.h"
#include "baro_fusion.h"
#include "filters.h"
#include "flight_sm.h"
#include "main.h"

// Public functions that main.c can see
void App_Init(I2C_HandleTypeDef *hi2c1, I2C_HandleTypeDef *hi2c2, UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2);
void App_Run(void);
void App_MpuDmaNotify(void);
void App_Baro1DmaNotify(void);
void App_Baro2DmaNotify(void);
void App_LoraDmaNotify(void);
LoRa_E220* App_GetLora(void);
#endif /* APP_H_ */