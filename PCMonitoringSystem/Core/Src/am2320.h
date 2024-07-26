#ifndef AM2320_H
#define AM2320_H

#include "stm32f4xx_hal.h"

#define SENSOR_ADDRESS 0x5C

HAL_StatusTypeDef AM2320_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef AM2320_ReadTemperature(I2C_HandleTypeDef *hi2c, float *temperature);
HAL_StatusTypeDef AM2320_ReadHumidity(I2C_HandleTypeDef *hi2c, float *humidity);

#endif // SENSOR_H

