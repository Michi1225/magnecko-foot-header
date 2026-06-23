#pragma once

#include "main.h"


#ifdef EEPROM_WRITE
#include "eeprom_img.h"
#endif



#define EEPROM_I2C_HANDLE &hi2c4

#define EEPROM_I2C_ADDRESS 0x50
#define EEPROM_I2C_ADDRESS_ID 0x58


#define EEPROM_PAGE_SIZE 32
#define EEPROM_PAGE_COUNT 256



/** @brief Write data to the EEPROM
 *  @param page The starting page address
 *  @param offset The offset within the page
 *  @param data Pointer to the data buffer
 *  @param size Number of bytes to write
 */
void EEPROM_Write (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);


/** @brief Read data from the EEPROM
 *  @param page The starting page address
 *  @param offset The offset within the page
 *  @param data Pointer to the data buffer
 *  @param size Number of bytes to read
 */
void EEPROM_Read (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);