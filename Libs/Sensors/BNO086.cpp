#include "BNO086.h"
#include "cmsis_gcc.h"
#include "stm32h7xx_hal_spi.h"
#include <cstdint>


__section(BNO086_SECTION_NAME) __aligned(4) 
struct 
{
    uint8_t shtp_header[4];
    uint8_t time_stamp[5];
    uint8_t report_id;
} bno_header;
__section(BNO086_SECTION_NAME) __aligned(4) struct 
{
    uint16_t length = 0;
    uint8_t channel = 0;
    uint8_t sequence = 0;
    uint8_t report_id = 0;
}shtp_header;
__section(BNO086_SECTION_NAME) __aligned(4) uint8_t dummy[128] = {0};

VectorData BNO086::gyro_data        __section(BNO086_SECTION_NAME) = {0, 0, 0, 0, 0, 0, BNO086_Q_POINT_GYROSCOPE};
VectorData BNO086::accel_data       __section(BNO086_SECTION_NAME) = {0, 0, 0, 0, 0, 0, BNO086_Q_POINT_ACCELEROMETER};
VectorData BNO086::mag_data         __section(BNO086_SECTION_NAME) = {0, 0, 0, 0, 0, 0, BNO086_Q_POINT_MAGNETOMETER};
VectorData BNO086::lin_accel_data   __section(BNO086_SECTION_NAME) = {0, 0, 0, 0, 0, 0, BNO086_Q_POINT_LINEAR_ACCELERATION};
VectorData BNO086::grav_data        __section(BNO086_SECTION_NAME) = {0, 0, 0, 0, 0, 0, BNO086_Q_POINT_GRAVITY};
RotationVectorData BNO086::rot_data __section(BNO086_SECTION_NAME) = {0, 0, 0, 0, 0, 0, 0, 0, BNO086_Q_POINT_ROTATION, BNO086_Q_POINT_ACCURACY_ROTATION};


static void read_dummy()
{
    HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Receive(BNO086_SPI_HANDLE, (uint8_t *)&shtp_header, sizeof(shtp_header), 10);
    if((shtp_header.length & 0x7FFF) > 10) HAL_SPI_Receive(BNO086_SPI_HANDLE, dummy, (shtp_header.length & 0x7FFF) - sizeof(shtp_header), 10);
    HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_SET);
}


BNO086::BNO086()
{
    this->msgs_ready = 0;
    this->seqNum = 0;
    this->features.clear();
    BNO086::gyro_data = {0, 0, 0, 0, 0, 0, BNO086_Q_POINT_GYROSCOPE};
    BNO086::lin_accel_data = {0, 0, 0, 0, 0, 0, BNO086_Q_POINT_LINEAR_ACCELERATION};
    BNO086::rot_data = {0, 0, 0, 0, 0, 0, 0, 0, BNO086_Q_POINT_ROTATION, BNO086_Q_POINT_ACCURACY_ROTATION};
    BNO086::accel_data = {0, 0, 0, 0, 0, 0, BNO086_Q_POINT_ACCELEROMETER};
    __NOP();
}


uint8_t BNO086::init()
{

    HAL_StatusTypeDef status = HAL_OK;

    //Set feature reports to be set up
#if ENABLE_ACCEL
    this->features.push_back(std::make_pair(BNO086_ID_ACCELEROMETER,        BNO086_PERIOD_ACCELEROMETER));
#endif
#if ENABLE_ROT
    this->features.push_back(std::make_pair(BNO086_ID_ROTATION,          BNO086_PERIOD_ROTATION));
#endif
#if ENABLE_GYRO
    this->features.push_back(std::make_pair(BNO086_ID_GYROSCOPE,            BNO086_PERIOD_GYROSCOPE));
#endif
#if ENABLE_LIN_ACCEL
    this->features.push_back(std::make_pair(BNO086_ID_LINEAR_ACCELERATION,  BNO086_PERIOD_LINEAR_ACCELERATION));
#endif
#if ENABLE_GRAVITY
    this->features.push_back(std::make_pair(BNO086_ID_GRAVITY,              BNO086_PERIOD_GRAVITY));
#endif
#if ENABLE_MAG
    this->features.push_back(std::make_pair(BNO086_ID_MAGNETOMETER,         BNO086_PERIOD_MAGNETOMETER));
#endif


    //Reset BNO
    // HAL_NVIC_DisableIRQ(EXTI2_IRQn); //Disable HINT_N interrupt
    HAL_GPIO_WritePin(IMU_NRST_GPIO_Port, IMU_NRST_Pin, GPIO_PIN_RESET);
    HAL_Delay(0);
    HAL_GPIO_WritePin(IMU_NRST_GPIO_Port, IMU_NRST_Pin, GPIO_PIN_SET);

    //After reset, we have to wait for BNO to assert HINT_N
    // while(HAL_GPIO_ReadPin(IMU_INT_GPIO_Port, IMU_INT_Pin) == GPIO_PIN_RESET);
    // HAL_NVIC_EnableIRQ(EXTI2_IRQn); //Enable HINT_N interrupt


    //Read advertisemsent
    while(this->msgs_ready == 0);
    this->msgs_ready = 0;
    uint8_t advertisement[284] = {0};
    if(HAL_SPI_Receive(BNO086_SPI_HANDLE, advertisement, 284, 10) != HAL_OK) status = HAL_ERROR;
    
    //read initialize response
    while(this->msgs_ready == 0);
    this->msgs_ready = 0;
    uint8_t init[40] = {0};
    if(HAL_SPI_Receive(BNO086_SPI_HANDLE, init, 20, 100) != HAL_OK) status = HAL_ERROR;

    //read message from executable
    while(this->msgs_ready == 0);
    this->msgs_ready = 0;
    uint8_t exec[10] = {0};
    if(HAL_SPI_Receive(BNO086_SPI_HANDLE, exec, 5, 10) != HAL_OK) status = HAL_ERROR;

    if(status == HAL_OK) this->initialized = true;

    return status;
}


uint8_t BNO086::start()
{
    if(!this->initialized) return 1; //BNO not initialized

    //Wake up BNO
    HAL_GPIO_WritePin(IMU_WAKE_GPIO_Port, IMU_WAKE_Pin, GPIO_PIN_RESET);
    while(this->msgs_ready == 0);
    this->msgs_ready = 0;
    HAL_GPIO_WritePin(IMU_WAKE_GPIO_Port, IMU_WAKE_Pin, GPIO_PIN_SET);

    uint8_t errorcode = 0;
    uint8_t rxBytes[128] = {0};
    uint8_t txBytes[21];
    

    

    for(auto& [id, period]: this->features)
    {
        txBytes[0] = 0x15;
        txBytes[1] = 0x00;
        txBytes[2] = 0x02;
        txBytes[3] = this->seqNum;
        txBytes[4] = 0xFD; // Set Feature Command (P.63): https://www.ceva-ip.com/wp-content/uploads/2019/10/SH-2-Reference-Manual.pdf


        txBytes[5] = id; //Report ID: Rotation Vector
        txBytes[6] = 0b00001000; // Feature Flags (P. 62)
        txBytes[7] = 0x00; //Change Sensitivity LSB         --> Sensitivity only changes, how often new reports are generated, but not how often the corresponding interrupt is asserted.
        txBytes[8] = 0x00; //Change Sensitivity MSB
        txBytes[9] = (period & 0x000000FF);          //Report Interval in us LSB
        txBytes[10] = (period & 0x0000FF00) >> 8;
        txBytes[11] = (period & 0x00FF0000) >> 16;
        txBytes[12] = (period & 0xFF000000) >> 24;    //Report Interval in us MSB
        txBytes[13] = 0xFF;      //Batch Interval: Don't trigger delivery based on time between sampling and report interval LSB
        txBytes[14] = 0xFF;      
        txBytes[15] = 0xFF;
        txBytes[16] = 0xFF;     //Batch Interval MSB
        txBytes[17] = 0x00;     //Sensor-specific configuration
        txBytes[18] = 0x00;
        txBytes[19] = 0x00;
        txBytes[20] = 0x00;


        errorcode = HAL_SPI_Transmit(BNO086_SPI_HANDLE, txBytes, 21, 1000);
        if(errorcode != HAL_OK) return errorcode;
        this->seqNum = (this->seqNum + 1) % 256;

        //read Get Feature Response
        //this response is sent unsolicited on rate change
        do
        {
            /* code */
            while(this->msgs_ready == 0);
            this->msgs_ready = 0;
            read_dummy();
        } while (shtp_header.report_id != 0xFC);

        HAL_Delay(0);
    }


    return errorcode;

}


uint8_t BNO086::update()
{
    if(!this->initialized) return 1; //BNO not initialized
    uint8_t errorcode = 0;
    while(this->msgs_ready > 0)
    {
        this->msgs_ready--;
        
        //read header
    uint8_t errorcode = HAL_SPI_TransmitReceive_IT(BNO086_SPI_HANDLE, dummy, (uint8_t *)&bno_header, sizeof(bno_header));
    while(HAL_SPI_GetState(BNO086_SPI_HANDLE) != HAL_SPI_STATE_READY);

    switch (bno_header.report_id)
        {
        case BNO086_ID_ACCELEROMETER:
            //NOTE: This might introduce a hard fault, because of the packed struct.
            //      If this is the case, every variable has to be copied seperately
        errorcode |= HAL_SPI_Receive_IT(BNO086_SPI_HANDLE, (uint8_t *)&this->accel_data, 9);
            break;
        case BNO086_ID_GYROSCOPE:
        errorcode |= HAL_SPI_Receive_IT(BNO086_SPI_HANDLE, (uint8_t *)&this->gyro_data, 9);
            break;
        case BNO086_ID_MAGNETOMETER:
        errorcode |= HAL_SPI_Receive_IT(BNO086_SPI_HANDLE, (uint8_t *)&this->mag_data, 9);
            break;
        case BNO086_ID_LINEAR_ACCELERATION:
        errorcode |= HAL_SPI_Receive_IT(BNO086_SPI_HANDLE, (uint8_t *)&this->lin_accel_data, 9);
            break;
        case BNO086_ID_ROTATION:
        errorcode |= HAL_SPI_Receive_IT(BNO086_SPI_HANDLE, (uint8_t *)&this->rot_data, 13);
            break;
        case BNO086_ID_GRAVITY:
        errorcode |= HAL_SPI_Receive_IT(BNO086_SPI_HANDLE, (uint8_t *)&this->grav_data, 9);
            break;
        default:
            errorcode = 1;
            break;
        }

    // Wait for SPI Transaction to complete
    while(HAL_SPI_GetState(BNO086_SPI_HANDLE) != HAL_SPI_STATE_READY);
    
        // Update IMU Data Object
        if(errorcode == 0)
        {
            if(this->accel_data.status >= 2)
            {
                this->output_accel[0] = q_to_float(this->accel_data.axis_x, BNO086_Q_POINT_ACCELEROMETER);
                this->output_accel[1] = q_to_float(this->accel_data.axis_y, BNO086_Q_POINT_ACCELEROMETER);
                this->output_accel[2] = q_to_float(this->accel_data.axis_z, BNO086_Q_POINT_ACCELEROMETER);
            }
            if(this->gyro_data.status >= 2)
            {
                this->output_gyro[0] = q_to_float(this->gyro_data.axis_x, BNO086_Q_POINT_GYROSCOPE);
                this->output_gyro[1] = q_to_float(this->gyro_data.axis_y, BNO086_Q_POINT_GYROSCOPE);
                this->output_gyro[2] = q_to_float(this->gyro_data.axis_z, BNO086_Q_POINT_GYROSCOPE);
            }
            if(this->rot_data.accuracy_estimate == 12868) // Magic Number
            {
                this->output_quat[0] = q_to_float(this->rot_data.quaternion_i, BNO086_Q_POINT_ROTATION);
                this->output_quat[1] = q_to_float(this->rot_data.quaternion_j, BNO086_Q_POINT_ROTATION);
                this->output_quat[2] = q_to_float(this->rot_data.quaternion_k, BNO086_Q_POINT_ROTATION);
                this->output_quat[3] = q_to_float(this->rot_data.quaternion_real, BNO086_Q_POINT_ROTATION);
            }
            if(this->lin_accel_data.status >= 2)
            {
                this->output_lin_accel[0] = q_to_float(this->lin_accel_data.axis_x, BNO086_Q_POINT_LINEAR_ACCELERATION);
                this->output_lin_accel[1] = q_to_float(this->lin_accel_data.axis_y, BNO086_Q_POINT_LINEAR_ACCELERATION);
                this->output_lin_accel[2] = q_to_float(this->lin_accel_data.axis_z, BNO086_Q_POINT_LINEAR_ACCELERATION);

            }
    
        }
        //TODO: Handle errorcode: send warning via EtherCAT??

    }


    return errorcode;

}


/*
 * @brief       Converts 16 bit quaternion to float
 * @param[in]   raw: raw quaternion data
 * @param[in]   q_point: quaternion fixed point position
 * @retval      Returns the quaternion value as float
 */
float q_to_float(int16_t raw, uint8_t q_point)
{
    return static_cast<float>(raw) / (1UL << q_point);
}