#pragma once
#include "main.h"


//DEVICE_CONFIG_1
#define DEVICE_CONFIG_1_ADDR 0x00
#define DEVICE_CONFIG_1_VAL  0x0C

#define DEVICE_CONFIG_2_ADDR 0x01
#define DEVICE_CONFIG_2_VAL  0x12

#define SENSOR_CONFIG_1_ADDR 0x02
#define SENSOR_CONFIG_1_VAL  0x70

#define SENSOR_CONFIG_2_ADDR 0x03
#define SENSOR_CONFIG_2_VAL  0x03 //angle calculation disabled

#define X_THR_CONFIG_ADDR 0x04
#define X_THR_CONFIG_VAL  0x00

#define Y_THR_CONFIG_ADDR 0x05
#define Y_THR_CONFIG_VAL  0x00

#define Z_THR_CONFIG_ADDR 0x06
#define Z_THR_CONFIG_VAL  0x00

#define T_CONFIG_ADDR 0x07
#define T_CONFIG_VAL  0x01

#define INT_CONFIG_1_ADDR 0x08
#define INT_CONFIG_1_VAL  0x01

#define MAG_GAIN_CONFIG_ADDR 0x09
#define MAG_GAIN_CONFIG_VAL  0x00

#define MAG_OFFSET_CONFIG_1_ADDR 0x0A
#define MAG_OFFSET_CONFIG_1_VAL  0x00

#define MAG_OFFSET_CONFIG_2_ADDR 0x0B
#define MAG_OFFSET_CONFIG_2_VAL  0x00

#define I2C_ADDRESS_ADDR 0x0C
#define I2C_ADDRESS_VAL  0x00 // disable address update from I2C

//read only
#define DEVICE_ID_ADDR 0x0D
#define MANUFACTURER_ID__LSB_ADDR 0x0E
#define MANUFACTURER_ID__MSB_ADDR 0x0F
#define T_MSB_RESULT_ADDR 0x10
#define T_LSB_RESULT_ADDR 0x11
#define X_MSB_RESULT_ADDR 0x12
#define X_LSB_RESULT_ADDR 0x13
#define Y_MSB_RESULT_ADDR 0x14
#define Y_LSB_RESULT_ADDR 0x15
#define Z_MSB_RESULT_ADDR 0x16
#define MAG_MSB_RESULT_ADDR 0x17
#define CONV_STATUS_ADDR 0x18
#define ANGLE_RESULT_MSB_ADDR 0x19
#define ANGLE_RESULT_LSB_ADDR 0x1A
#define MAGNITUDE_RESULT_ADDR 0x1B
#define DEVICE_STATUS_ADDR 0x1C

#define DEVICE_VERSION 2 // 1 for TMAG5273x1, 2 for TMAG5273x2

#if (DEVICE_VERSION == 1)
#define MAG_SENSITIVITY 40 // mT/LSB
#elif (DEVICE_VERSION == 2)
#define MAG_SENSITIVITY 125.0f // mT/LSB
#endif

#define SENS_I2C_HANDLE &hi2c3



class TMAG5273
{
private:
    uint8_t device_address;
    
public:
    enum device_version
    {
        A1,
        B1,
        C1,
        D1
    };

    /**
     * @brief TMAG5273 constructor
     * @param version device version
     */
    TMAG5273(device_version version);

    /**
     * @brief initialize all TMAG5273 on the bus
     * @return HAL_OK if success, HAL_ERROR if failed
     */
    static uint8_t init();

    /**
     * @brief read the X axis B-Field value
     * @return Bx value in mT
     */
    float read_Bx();

    /**
     * @brief read the Y axis B-Field value
     * @return By value in mT
     */
    float read_By();

    /**
     * @brief read the Z axis B-Field value
     * @return Bz value in mT
     */
    float read_Bz();

    /**
     * @brief read all three axes B-Field values
     * @return B-Field Magnitude in mT
     */
    float read_magnitude();

    /**
     * @brief read the temperature value
     * @return temperature value in degree Celsius
     */
    float read_T();

    /**
     * @brief Estimate the Contact
     * @return true if contact, false if no contact
     */
    bool estimate_contact();


};
