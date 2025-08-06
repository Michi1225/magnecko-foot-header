/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "platform.h"

extern I2C_HandleTypeDef hi2c3;

__section(".RAM") uint8_t data_write[0x8000];
__section(".RAM") uint8_t data_read[0x8000];

uint8_t RdByte(
		VL53LMZ_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_value)
{
	uint8_t status = 0;

	data_write[0] = (RegisterAdress >> 8) & 0xFF;
	data_write[1] = RegisterAdress & 0xFF;
	status = HAL_I2C_Master_Transmit_DMA(&hi2c3, p_platform->address, data_write, 2);
	while(HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY);
	status = HAL_I2C_Master_Receive_DMA(&hi2c3, p_platform->address, data_read, 1);
	while(HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY);
	*p_value = data_read[0];
	//uart_printf("read 1 byte\n");
	return status;
}

uint8_t WrByte(
		VL53LMZ_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t value)
{
	uint8_t status = 0;

	data_write[0] = (RegisterAdress >> 8) & 0xFF;
	data_write[1] = RegisterAdress & 0xFF;
	data_write[2] = value & 0xFF;
	status = HAL_I2C_Master_Transmit_DMA(&hi2c3, p_platform->address, data_write, 3);
	while(HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY);
	//uart_printf("write 1 byte\n");
	return status;
}

uint8_t WrMulti(
		VL53LMZ_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	memcpy(data_write, p_values, size);
	uint8_t status = HAL_I2C_Mem_Write_DMA(&hi2c3, p_platform->address, RegisterAdress,
			I2C_MEMADD_SIZE_16BIT, data_write, size);
	while(HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY);
	//uart_printf("write %d bytes\n",size);
	return status;
}

uint8_t RdMulti(
		VL53LMZ_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	uint8_t status;
	data_write[0] = (RegisterAdress>>8) & 0xFF;
	data_write[1] = RegisterAdress & 0xFF;

	status = HAL_I2C_Master_Transmit_DMA(&hi2c3, p_platform->address, data_write, 2);
	while(HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY);
	status += HAL_I2C_Master_Receive_DMA(&hi2c3, p_platform->address, data_read, size);
	while(HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY);
	memcpy(p_values, data_read, size);
    //uart_printf("read %d bytes\n",size);

	return status;
}


void SwapBuffer(
		uint8_t 		*buffer,
		uint16_t 	 	 size)
{
	uint32_t i, tmp;

	/* Example of possible implementation using <string.h> */
	for(i = 0; i < size; i = i + 4)
	{
		tmp = (
		  buffer[i]<<24)
		|(buffer[i+1]<<16)
		|(buffer[i+2]<<8)
		|(buffer[i+3]);

		memcpy(&(buffer[i]), &tmp, 4);
	}
}

uint8_t WaitMs(
		VL53LMZ_Platform *p_platform,
		uint32_t TimeMs)
{
	HAL_Delay(TimeMs);
	return 0;
}
