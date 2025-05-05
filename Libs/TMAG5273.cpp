#include "TMAG5273.h"


TMAG5273::TMAG5273(device_version version)
{
    switch (version)
    {
        case A1:
            this->device_address = 0x35;
            break;
        case B1:
            this->device_address = 0x22;
            break;
        case C1:
            this->device_address = 0x78;
            break;
        case D1:
            this->device_address = 0x44;
            break;
        default:
            this->device_address = 0x35; // Default to A1 if invalid version
            break;
    }
}


uint8_t TMAG5273::init()
{
    uint8_t data[0x0D] =
    {
        DEVICE_CONFIG_1_VAL,
        DEVICE_CONFIG_2_VAL,
        SENSOR_CONFIG_1_VAL,
        SENSOR_CONFIG_2_VAL,
        X_THR_CONFIG_VAL,
        Y_THR_CONFIG_VAL,
        Z_THR_CONFIG_VAL,
        T_CONFIG_VAL,
        INT_CONFIG_1_VAL,
        MAG_GAIN_CONFIG_VAL,
        MAG_OFFSET_CONFIG_1_VAL,
        MAG_OFFSET_CONFIG_2_VAL,
        I2C_ADDRESS_VAL
    };
    uint8_t general_call_write = 0x00;
    return HAL_I2C_Master_Transmit(SENS_I2C_HANDLE, general_call_write, data, 0x0D, 100);
}

float TMAG5273::read_By()
{
    uint8_t txData[1] = {Y_MSB_RESULT_ADDR};
    uint8_t rxData[2] = {0};
    if(HAL_I2C_Master_Transmit(SENS_I2C_HANDLE, this->device_address, txData, 1, 100) != HAL_OK) Error_Handler();
    if(HAL_I2C_Master_Receive(SENS_I2C_HANDLE, this->device_address, rxData, 2, 100) != HAL_OK) Error_Handler();

    float by = ((int16_t)((rxData[0] << 8) | rxData[1])) / 65536 * 2 * 40;
    return 0.0f;
}


float TMAG5273::read_Bx()
{
    uint8_t txData[1] = {X_MSB_RESULT_ADDR};
    uint8_t rxData[2] = {0};
    if(HAL_I2C_Master_Transmit(SENS_I2C_HANDLE, this->device_address, txData, 1, 100) != HAL_OK) Error_Handler();
    if(HAL_I2C_Master_Receive(SENS_I2C_HANDLE, this->device_address, rxData, 2, 100) != HAL_OK) Error_Handler();

    float bx = ((int16_t)((rxData[0] << 8) | rxData[1])) / 65536 * 2 * 40;
    return bx;
}

float TMAG5273::read_Bz()
{
    uint8_t txData[1] = {Z_MSB_RESULT_ADDR};
    uint8_t rxData[2] = {0};
    if(HAL_I2C_Master_Transmit(SENS_I2C_HANDLE, this->device_address, txData, 1, 100) != HAL_OK) Error_Handler();
    if(HAL_I2C_Master_Receive(SENS_I2C_HANDLE, this->device_address, rxData, 2, 100) != HAL_OK) Error_Handler();

    float bz = ((int16_t)((rxData[0] << 8) | rxData[1])) / 65536 * 2 * 40;
    return bz;
}

float TMAG5273::read_T()
{
    uint8_t txData[1] = {T_MSB_RESULT_ADDR};
    uint8_t rxData[2] = {0};
    if(HAL_I2C_Master_Transmit(SENS_I2C_HANDLE, this->device_address, txData, 1, 100) != HAL_OK) Error_Handler();
    if(HAL_I2C_Master_Receive(SENS_I2C_HANDLE, this->device_address, rxData, 2, 100) != HAL_OK) Error_Handler();

    float t = 25.0f + ((int16_t)((rxData[0] << 8) | rxData[1]) - 17508) / 60.1f;
    return t;
}