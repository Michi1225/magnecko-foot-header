#include "TMF8829.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_spi.h"
#include <cmath>
#include <stdint.h>


float TMF8829::confidence_lookup[256] = {0};

TMF8829::TMF8829()
{
    this->data_valid = false;
    std::memset(&this->data_frame, 0, sizeof(this->data_frame));
}

HAL_StatusTypeDef TMF8829::init()
{
    int status = HAL_OK;


    // Init confidence lookup table
    int index;
    for(index = 0; index <= TMF8829_CONF_BREAKPOINT; ++index)
    {
        TMF8829::confidence_lookup[index] = index;
    }
    for (; index <= 255; ++index)
    {
        TMF8829::confidence_lookup[index] = TMF8829_CONF_BREAKPOINT * std::pow(TMF8829_EXP_GROWTH_RATE, (index - TMF8829_CONF_BREAKPOINT));
    }



    struct
    {
        uint8_t cmd;
        uint8_t data;
    } config_data, read_data;


    //Power up
    config_data.cmd = TMF8829_ENABLE;
    config_data.data = 0x04;
    status |= HAL_SPI_Transmit(TOF_SPI_HANDLE, (uint8_t *)(&config_data), sizeof(config_data), 10);

    HAL_Delay(2);

    //Check if device has booted
    config_data.cmd = TMF8829_ENABLE | 0x80; // Set MSB for read operation
    config_data.data = 0x00; // Dummy data for read operation
    do
    {
        status |= HAL_SPI_TransmitReceive(TOF_SPI_HANDLE, (uint8_t *)(&config_data), (uint8_t *)(&read_data), sizeof(config_data), 10);
    } while (!(read_data.data & 0x80));

    // Disable I2C interface
    config_data.cmd = TMF8829_CMD_STAT;
    config_data.data = 0x22;
    status |= HAL_SPI_Transmit(TOF_SPI_HANDLE, (uint8_t *)(&config_data), sizeof(config_data), 10);

    // Set Interrupt Mask
    config_data.cmd = TMF8829_INT_ENAB;
    config_data.data = 0x01; // Enable interrupt for new ranging data
    status |= HAL_SPI_Transmit(TOF_SPI_HANDLE, (uint8_t *)(&config_data), sizeof(config_data), 10);

    HAL_Delay(0);

    // Start application
    config_data.cmd = TMF8829_ENABLE;
    config_data.data = 0xA4;
    status |= HAL_SPI_Transmit(TOF_SPI_HANDLE, (uint8_t *)(&config_data), sizeof(config_data), 10);


    // Load Configuration
    config_data.cmd = TMF8829_CMD_STAT;
    config_data.data = TMF8829_CMD_LOAD_CFG_8X8;
    status |= HAL_SPI_Transmit(TOF_SPI_HANDLE, (uint8_t *)(&config_data), sizeof(config_data), 10);

    HAL_Delay(0);
    read_data.cmd = 0;
    read_data.data = 0;
    config_data.cmd = TMF8829_CMD_STAT | 0x80; // Set MSB for read operation
    do
    {
        status |= HAL_SPI_TransmitReceive(TOF_SPI_HANDLE, (uint8_t *)(&config_data), (uint8_t *)(&read_data), sizeof(config_data), 10);
    } while (read_data.data != 0x00); // Wait for configuration to be loaded

    // Set Ranging Period to 16ms
    config_data.cmd = TMF8829_CFG_PERIOD_MS_LSB;
    config_data.data = 0x10; // 16ms
    status |= HAL_SPI_Transmit(TOF_SPI_HANDLE, (uint8_t *)(&config_data), sizeof(config_data), 10);
    config_data.cmd = TMF8829_CFG_PERIOD_MS_MSB;
    config_data.data = 0x00; // 16ms
    status |= HAL_SPI_Transmit(TOF_SPI_HANDLE, (uint8_t *)(&config_data), sizeof(config_data), 10);

    // Set Iterations
    config_data.cmd = TMF8829_CFG_KILO_ITERATIONS_LSB;
    config_data.data = 0xE1; // 225 Kilo iterations
    status |= HAL_SPI_Transmit(TOF_SPI_HANDLE, (uint8_t *)(&config_data), sizeof(config_data), 10);
    config_data.cmd = TMF8829_CFG_KILO_ITERATIONS_MSB;
    config_data.data = 0x00; // 225 Kilo iterations
    status |= HAL_SPI_Transmit(TOF_SPI_HANDLE, (uint8_t *)(&config_data), sizeof(config_data), 10);

    //Set Result Format
    config_data.cmd = TMF8829_CFG_RESULT_FORMAT;
    config_data.data = 0x01; // Default format, Only distance and SNR
    status |= HAL_SPI_Transmit(TOF_SPI_HANDLE, (uint8_t *)(&config_data), sizeof(config_data), 10);

    if(status == HAL_OK) this->initialized = true;

    return (HAL_StatusTypeDef)status;
}

HAL_StatusTypeDef TMF8829::start_ranging() 
{
    if(!this->initialized) return HAL_ERROR; // Return error if sensor is not initialized
    struct
    {
        uint8_t cmd;
        uint8_t data;
    } config_data;

    config_data.cmd = TMF8829_ENABLE;
    config_data.data = 0xA5; // Start Ranging command

    return HAL_SPI_Transmit(TOF_SPI_HANDLE, (uint8_t *)(&config_data), sizeof(config_data), 10);
}

void TMF8829::get_ranging_data() 
{
    if(!this->initialized) return; // Return if sensor is not initialized
    this->data_valid = false;
    // Clear Interrupt flag
    struct
    {
        uint8_t cmd;
        uint8_t data[sizeof(TMF8829_Frame_t) - 1];
    } txdata;

    txdata.cmd = TMF8829_CMD_STAT;
    txdata.data[0] = 0x0F;
    HAL_SPI_Transmit_IT(TOF_SPI_HANDLE, (uint8_t *)(&txdata), 2);
    while (HAL_SPI_GetState(TOF_SPI_HANDLE) != HAL_SPI_STATE_READY);
    txdata.data[0] = 0x00;

    txdata.cmd = TMF8829_FIFOSTATUS | 0x80; // Set MSB for read operation
    HAL_SPI_TransmitReceive_IT(TOF_SPI_HANDLE, (uint8_t *)(&txdata), (uint8_t *)(&this->data_frame), sizeof(txdata));
}

void TMF8829::update_data()
{
    const TMF8829_Pixel_Data_t* pixels = &(this->data_frame.pixel_data[0][0]);

    float* dist_ptr = &(this->distances[0][0]);
    float* conf_ptr = &(this->confidances[0][0]);

    for (int i = 0; i < 64; ++i)
    {
        dist_ptr[i] = pixels[i].distance * 0.25f; // divide by 4
        conf_ptr[i] = (float)TMF8829::confidence_lookup[pixels[i].snr];
    }
}
