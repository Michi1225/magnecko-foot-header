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
    uint8_t data[0x0E] =
    {
        DEVICE_CONFIG_1_ADDR,
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
    uint8_t error_code = HAL_I2C_Master_Transmit(SENS_I2C_HANDLE, general_call_write, data, 0x0E, 100);
    return error_code;
}

float TMAG5273::read_Bx()
{
    uint8_t txData[1] = {X_MSB_RESULT_ADDR}; // Read operation
    uint8_t rxData[2] = {0};
    if(HAL_I2C_Master_Transmit(SENS_I2C_HANDLE, (this->device_address << 1), txData, 1, 100) != HAL_OK) Error_Handler();
    if(HAL_I2C_Master_Receive(SENS_I2C_HANDLE, (this->device_address << 1) | 0x01, rxData, 2, 100) != HAL_OK) Error_Handler();

    int16_t raw = (static_cast<int16_t>(rxData[0]) << 8) | rxData[1];
    float bx = (float)(raw) / 65536.0f * 2.0f * MAG_SENSITIVITY;
    return bx;
}


float TMAG5273::read_By()
{
    uint8_t txData[1] = {Y_MSB_RESULT_ADDR};
    uint8_t rxData[2] = {0};
    if(HAL_I2C_Master_Transmit(SENS_I2C_HANDLE, (this->device_address << 1), txData, 1, 100) != HAL_OK) Error_Handler();
    if(HAL_I2C_Master_Receive(SENS_I2C_HANDLE, (this->device_address << 1) | 0x01, rxData, 2, 100) != HAL_OK) Error_Handler();

    int16_t raw = (static_cast<int16_t>(rxData[0]) << 8) | rxData[1];
    float by = (float)(raw) / 65536.0f * 2.0f * MAG_SENSITIVITY;
    return by;
}



float TMAG5273::read_Bz()
{
    uint8_t txData[1] = {Z_MSB_RESULT_ADDR};
    uint8_t rxData[2] = {0};
    if(HAL_I2C_Master_Transmit(SENS_I2C_HANDLE, (this->device_address << 1), txData, 1, 100) != HAL_OK) Error_Handler();
    if(HAL_I2C_Master_Receive(SENS_I2C_HANDLE, (this->device_address << 1) | 0x01, rxData, 2, 100) != HAL_OK) Error_Handler();

    int16_t raw = (static_cast<int16_t>(rxData[0]) << 8) | rxData[1];
    float bz = (float)(raw) / 65536.0f * 2.0f * MAG_SENSITIVITY;
    return bz;
}

float TMAG5273::read_magnitude()
{
    uint8_t txData[1] = {X_MSB_RESULT_ADDR}; // Read operation
    uint8_t rxData[6] = {0};
    if(HAL_I2C_Master_Transmit(SENS_I2C_HANDLE, (this->device_address << 1), txData, 1, 100) != HAL_OK) Error_Handler();
    if(HAL_I2C_Master_Receive(SENS_I2C_HANDLE, (this->device_address << 1) | 0x01, rxData, 6, 100) != HAL_OK) Error_Handler();


    
    int16_t raw_x = (static_cast<int16_t>(rxData[0]) << 8) | rxData[1];
    float bx = (float)(raw_x) / 65536.0f * 2.0f * MAG_SENSITIVITY;
    int16_t raw_y = (static_cast<int16_t>(rxData[2]) << 8) | rxData[3];
    float by = (float)(raw_y) / 65536.0f * 2.0f * MAG_SENSITIVITY;
    int16_t raw_z = (static_cast<int16_t>(rxData[4]) << 8) | rxData[5];
    float bz = (float)(raw_z) / 65536.0f * 2.0f * MAG_SENSITIVITY;
    float magnitude = sqrt(bx * bx + by * by + bz * bz);
    return magnitude;
}

float TMAG5273::read_T()
{
    uint8_t txData[1] = {T_MSB_RESULT_ADDR};
    uint8_t rxData[2] = {0};
    if(HAL_I2C_Master_Transmit(SENS_I2C_HANDLE, (this->device_address << 1), txData, 1, 100) != HAL_OK) Error_Handler();
    if(HAL_I2C_Master_Receive(SENS_I2C_HANDLE, (this->device_address << 1) | 0x01, rxData, 2, 100) != HAL_OK) Error_Handler();

    
    int16_t raw = (static_cast<int16_t>(rxData[0]) << 8) | rxData[1];
    float t = 25.0f + (raw - 17508) / 60.1f;
    return t;
}

bool TMAG5273::estimate_contact()
{
    float b_mag = this->read_magnitude();
    float b_mag_threshold = 0.5f; // Threshold for contact estimation
    return b_mag < b_mag_threshold;
}
