#pragma once



#include "main.h"
#include "stm32h725xx.h"
#include <vector>
#include <utility>


//BNO update periods in us
#define BNO086_ID_ACCELEROMETER (uint8_t)0x01
#define BNO086_PERIOD_ACCELEROMETER 2500UL
#define BNO086_Q_POINT_ACCELEROMETER 8

#define BNO086_ID_GYROSCOPE (uint8_t)0x02
#define BNO086_PERIOD_GYROSCOPE 2500UL
#define BNO086_Q_POINT_GYROSCOPE 9

#define BNO086_ID_MAGNETOMETER (uint8_t)0x03
#define BNO086_PERIOD_MAGNETOMETER 2500UL
#define BNO086_Q_POINT_MAGNETOMETER 4

#define BNO086_ID_LINEAR_ACCELERATION (uint8_t)0x04
#define BNO086_PERIOD_LINEAR_ACCELERATION 2500UL
#define BNO086_Q_POINT_LINEAR_ACCELERATION 8

#define BNO086_ID_ROTATION (uint8_t)0x05
#define BNO086_PERIOD_ROTATION 2500UL
#define BNO086_Q_POINT_ROTATION 14
#define BNO086_Q_POINT_ACCURACY_ROTATION 12

#define BNO086_ID_GRAVITY (uint8_t)0x06
#define BNO086_PERIOD_GRAVITY 2500UL
#define BNO086_Q_POINT_GRAVITY 8



#define BNO086_SPI_HANDLE &hspi6

#define PACKED __attribute__((packed))

typedef struct PACKED
{
    int8_t sequence_number;
    int8_t status;
    int8_t delay;
    int16_t axis_x;
    int16_t axis_y;
    int16_t axis_z;
    uint8_t q_point;
}VectorData;

typedef struct PACKED
{
    int8_t sequence_number;
    int8_t status;
    int8_t delay;
    int16_t quaternion_i;
    int16_t quaternion_j;
    int16_t quaternion_k;
    int16_t quaternion_real;
    int16_t accuracy_estimate;
    uint8_t q_point;
    uint8_t q_point_accuracy;
}RotationVectorData;




class BNO086
{
private:
    std::vector<std::pair<uint8_t, uint32_t>> features;

public:
    BNO086();

    /**
     * @brief   Initialize BNO with the correct settings and define the Feature reports. 
     */
    uint8_t init();

    /**
     * @brief   Start feature reports, as defined in init()
     * @retval  HAL Status Code. 0 if all transmissions were successful
     */
    uint8_t start();

    /**
     * @brief   Updates the sensor values from the SPI interface. 
                This function should be called within 10ms of
                the BNO interrupt line assert.
     * @retval  HAL Status Code. 0 if all SPI Receive were successful
     */
    uint8_t update();
    
    static VectorData gyro_data;
    static VectorData accel_data;
    static VectorData mag_data;
    static VectorData lin_accel_data;
    static VectorData grav_data;
    static RotationVectorData rot_data;
    uint8_t seqNum = 0;
    bool msg_ready = false;

};

float q_to_float(int16_t raw, uint8_t q_point);
