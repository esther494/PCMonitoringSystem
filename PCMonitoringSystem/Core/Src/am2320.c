#include "am2320.h"
extern I2C_HandleTypeDef hi2c1;

HAL_StatusTypeDef AM2320_Init(I2C_HandleTypeDef *hi2c) {
	return HAL_I2C_Master_Transmit(hi2c, SENSOR_ADDRESS << 1, 0x00, 0, HAL_MAX_DELAY);
}

HAL_StatusTypeDef AM2320_ReadTemperature(I2C_HandleTypeDef *hi2c, float *temperature) {
	// cmd + start address + num of regs
	uint8_t msg[3] = {0x03, 0x02, 0x02};
	uint8_t temp_data[8];

	HAL_I2C_Master_Transmit(hi2c, SENSOR_ADDRESS << 1, 0x00, 0, HAL_MAX_DELAY);
	HAL_Delay(1);

	// Send command to read temperature
    if (HAL_I2C_Master_Transmit(hi2c, SENSOR_ADDRESS << 1, msg, 3, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(2);

    // Receive temperature data
    if (HAL_I2C_Master_Receive(hi2c, SENSOR_ADDRESS << 1, temp_data, 8, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(2);

    // Convert received data to temperature
    uint16_t raw_temp = (temp_data[2] << 8) | temp_data[3];
    *temperature = (float) raw_temp / 10;

    return HAL_OK;
}

HAL_StatusTypeDef AM2320_ReadHumidity(I2C_HandleTypeDef *hi2c, float *humidity) {
	// cmd + start address + num of regs
	uint8_t msg[3] = {0x03, 0x00, 0x02};
	uint8_t humidity_data[8];

	HAL_I2C_Master_Transmit(hi2c, SENSOR_ADDRESS << 1, 0x00, 0, HAL_MAX_DELAY);
	HAL_Delay(1);

	// Send command to read temperature
    if (HAL_I2C_Master_Transmit(hi2c, SENSOR_ADDRESS << 1, msg, 3, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(2);

    // Receive temperature data
    if (HAL_I2C_Master_Receive(hi2c, SENSOR_ADDRESS << 1, humidity_data, 8, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(2);

    // Convert received data to temperature
    uint16_t raw_humidity = (humidity_data[2] << 8) | humidity_data[3];
    *humidity = (float) raw_humidity / 10;

    return HAL_OK;
}
