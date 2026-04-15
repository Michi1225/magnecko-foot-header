#include "TMF8829.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_spi.h"

HAL_StatusTypeDef TMF8829::init()
{
    int status = HAL_OK;

    struct
    {
        uint8_t cmd;
        uint8_t data;
    } config_data, read_data;


    //Power up
    config_data.cmd = TMF8829_ENABLE;
    config_data.data = 0x04;
    status |= HAL_SPI_Transmit(TOF_SPI_HANDLE, &config_data, sizeof(config_data), 10);

    HAL_Delay(2);

    //Check if device has booted
    config_data.cmd = TMF8829_ENABLE | 0x80; // Set MSB for read operation
    config_data.data = 0x00; // Dummy data for read operation
    do
    {
        status |= HAL_SPI_TransmitReceive(TOF_SPI_HANDLE, &config_data, &read_data, sizeof(config_data), 10);
    } while (!(read_data.data & 0x80));

    // Disable I2C interface
    config_data.cmd = TMF8829_CMD_STAT;
    config_data.data = 0x22;
    status |= HAL_SPI_Transmit(TOF_SPI_HANDLE, &config_data, sizeof(config_data), 10);

    // Set Interrupt Mask
    config_data.cmd = TMF8829_INT_ENAB;
    config_data.data = 0x01; // Enable interrupt for new ranging data
    status |= HAL_SPI_Transmit(TOF_SPI_HANDLE, &config_data, sizeof(config_data), 10);

    HAL_Delay(0);

    // Start application
    config_data.cmd = TMF8829_ENABLE;
    config_data.data = 0xA4;
    status |= HAL_SPI_Transmit(TOF_SPI_HANDLE, &config_data, sizeof(config_data), 10);


    // Load Configuration
    config_data.cmd = TMF8829_CMD_STAT;
    config_data.data = TMF8829_CMD_LOAD_CFG_8X8;
    status |= HAL_SPI_Transmit(TOF_SPI_HANDLE, &config_data, sizeof(config_data), 10);

    HAL_Delay(0);
    read_data.cmd = 0;
    read_data.data = 0;
    config_data.cmd = TMF8829_CMD_STAT | 0x80; // Set MSB for read operation
    do
    {
        status |= HAL_SPI_TransmitReceive(TOF_SPI_HANDLE, &config_data, &read_data, sizeof(config_data), 10);
    } while (read_data.data != 0x00); // Wait for configuration to be loaded

    // Set Ranging Period to 16ms
    config_data.cmd = TMF8829_CFG_PERIOD_MS_LSB;
    config_data.data = 0x10; // 16ms
    status |= HAL_SPI_Transmit(TOF_SPI_HANDLE, &config_data, sizeof(config_data), 10);
    config_data.cmd = TMF8829_CFG_PERIOD_MS_MSB;
    config_data.data = 0x00; // 16ms
    status |= HAL_SPI_Transmit(TOF_SPI_HANDLE, &config_data, sizeof(config_data), 10);

    // Set Iterations
    config_data.cmd = TMF8829_CFG_KILO_ITERATIONS_LSB;
    config_data.data = 0xE1; // 225 Kilo iterations
    status |= HAL_SPI_Transmit(TOF_SPI_HANDLE, &config_data, sizeof(config_data), 10);
    config_data.cmd = TMF8829_CFG_KILO_ITERATIONS_MSB;
    config_data.data = 0x00; // 225 Kilo iterations
    status |= HAL_SPI_Transmit(TOF_SPI_HANDLE, &config_data, sizeof(config_data), 10);

    //Set Result Format
    config_data.cmd = TMF8829_CFG_RESULT_FORMAT;
    config_data.data = 0x01; // Default format, Only distance and SNR
    status |= HAL_SPI_Transmit(TOF_SPI_HANDLE, &config_data, sizeof(config_data), 10);



    return (HAL_StatusTypeDef)status;
}

void TMF8829::start_ranging() 
{
    struct
    {
        uint8_t cmd;
        uint8_t data;
    } config_data, read_data;

    config_data.cmd = TMF8829_ENABLE;
    config_data.data = 0xA5; // Start Ranging command

    HAL_SPI_Transmit(TOF_SPI_HANDLE, &config_data, sizeof(config_data), 10);
}

void TMF8829::get_ranging_data() 
{
    //TODO: reset interrupt flag in INT_STATUS register after reading data
}
