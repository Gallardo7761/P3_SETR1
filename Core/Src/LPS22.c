#include "stm32l4xx_hal.h"

extern I2C_HandleTypeDef hi2c2;

void LPS22_Init() {
	uint8_t buffer[1];
	buffer[0] = 0x42;

	HAL_I2C_Mem_Write(&hi2c2, 0xBA, 0x10, I2C_MEMADD_SIZE_8BIT, buffer, 1 , 1000);
}

float LPS22_ReadPress() {
	float press;

	uint8_t buffer[3];

	HAL_I2C_Mem_Read(&hi2c2, 0xBA, 0x28, I2C_MEMADD_SIZE_8BIT, buffer, 3, 1000);

	uint32_t press_raw = (buffer[2] << 16) | (buffer[1] << 8) | buffer[0];

	press = press_raw / 4096.0f;

	return press;
}
