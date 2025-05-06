/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

 /** \file
 * \brief
 * ESC hardware layer functions.
 *
 * Function to read and write commands to the ESC. Used to read/write ESC
 * registers and memory.
 */

#include "../SOES/soes/esc.h"
#include "soes_pin_mapping_def.h"
#include <string.h>
#include "stm32h7xx_hal.h"


#define MAX_READ_SIZE   128

#define ESC_CMD_READ    0x02
#define ESC_CMD_READWS  0x03
#define ESC_CMD_WRITE   0x04
#define ESC_CMD_NOP     0x00
#define ESC_TERM        0xff
#define ESC_NEXT        0x00



static uint8_t read_termination[MAX_READ_SIZE] = { 0 };

static pin_mapping_typedef * soes_pin_mapping;

#define GPIO_ECAT_RESET    1 /* specific function to hold ESC reset on startup
                              * when emulating EEPROM
                              */


static void esc_address (uint16_t address, uint8_t command)
{
   /* Device is selected already.
    * We use 2 bytes addressing.
    */
   uint8_t data[2];

   /* address 12:5 */
   data[0] = (address >> 5);
   /* address 4:0 and cmd 2:0 */
   data[1] = ((address & 0x1F) << 3) | command;

   /* Write (and read AL interrupt register) */
   
   HAL_SPI_TransmitReceive(soes_pin_mapping->hspi, (uint8_t *) data, &ESCvar.ALevent, sizeof (data),100);
   
   ESCvar.ALevent = etohs (ESCvar.ALevent);
}

/** ESC read function used by the Slave stack.
 *
 * @param[in]   address     = address of ESC register to read
 * @param[out]  buf         = pointer to buffer to read in
 * @param[in]   len         = number of bytes to read
 */
void ESC_read (uint16_t address, void *buf, uint16_t len)
{
   // ASSERT(len <= MAX_READ_SIZE);
   HAL_GPIO_WritePin(soes_pin_mapping->NCS_port,soes_pin_mapping->NCS_pin, GPIO_PIN_RESET);


   /* Write address and command to device. */
   esc_address (address, ESC_CMD_READ);

   /* Here we want to read data and keep MOSI low (0x00) during
    * all bytes except the last one where we want to pull it high (0xFF).
    * Read (and write termination bytes).
    */
   HAL_StatusTypeDef err = HAL_SPI_TransmitReceive(soes_pin_mapping->hspi, read_termination + (MAX_READ_SIZE - len), buf, len,100);

   HAL_GPIO_WritePin(soes_pin_mapping->NCS_port,soes_pin_mapping->NCS_pin, GPIO_PIN_SET);
}

/** ESC write function used by the Slave stack.
 *
 * @param[in]   address     = address of ESC register to write
 * @param[out]  buf         = pointer to buffer to write from
 * @param[in]   len         = number of bytes to write
 */
void ESC_write (uint16_t address, void *buf, uint16_t len)
{
   /* Select device. */

   uint8_t dummy[len];

   HAL_GPIO_WritePin(soes_pin_mapping->NCS_port,soes_pin_mapping->NCS_pin, GPIO_PIN_RESET);
   /* Write address and command to device. */
   esc_address (address, ESC_CMD_WRITE);
   /* Write data. */
   HAL_SPI_TransmitReceive(soes_pin_mapping->hspi, buf, dummy, len,100);
   /* Un-select device. */
   HAL_GPIO_WritePin(soes_pin_mapping->NCS_port,soes_pin_mapping->NCS_pin, GPIO_PIN_SET);
}

void ESC_reset (void)
{
   volatile int timeout;
   // Due to current hardware, hardware reset of the ET1100 is not possible.
   while(timeout<10000000)
   {
      /* ECAT releases resetpin */
      if(HAL_GPIO_ReadPin (soes_pin_mapping->EEPROM_loaded_port,soes_pin_mapping->EEPROM_loaded_pin)!=0)
      {
         break; // OK
      }
      timeout++;
      HAL_Delay(30);
   }
}

void ESC_init (const esc_cfg_t * config)
{
   // read the mapping of the physical ports from the user.
   soes_pin_mapping = (pin_mapping_typedef *) config->user_arg;
   read_termination[MAX_READ_SIZE - 1] = ESC_TERM;
}
