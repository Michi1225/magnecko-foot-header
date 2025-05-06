// #include "spi.h"
#include "stm32h7xx_hal.h"

typedef struct pin_mapping_typedef {
    SPI_HandleTypeDef * hspi;
    GPIO_TypeDef * NCS_port;
    uint16_t NCS_pin;
    GPIO_TypeDef * EEPROM_loaded_port;
    uint16_t EEPROM_loaded_pin;
 }pin_mapping_typedef;