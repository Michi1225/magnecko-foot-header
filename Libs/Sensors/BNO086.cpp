#include "BNO086.h"


BNO086::BNO086()
{
    this->msg_ready = false;
    this->seqNum = 0;
    this->features.clear();
    this->accel_data = {0, 0, 0, 0, 0, 0, 0};
    this->gyro_data = {0, 0, 0, 0, 0, 0, 0};
    this->mag_data = {0, 0, 0, 0, 0, 0, 0};
    this->lin_accel_data = {0, 0, 0, 0, 0, 0, 0};
    this->rot_data = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    this->grav_data = {0, 0, 0, 0, 0, 0, 0};
}


uint8_t BNO086::init()
{
    //Set feature reports to be set up
    this->features.push_back(std::make_pair(BNO086_ID_ACCELEROMETER,        BNO086_PERIOD_ACCELEROMETER));
    // this->features.push_back(std::make_pair(BNO086_ID_GYROSCOPE,            BNO086_PERIOD_GYROSCOPE));
    // this->features.push_back(std::make_pair(BNO086_ID_ROTATION,          BNO086_PERIOD_ROTATION));
    // this->features.push_back(std::make_pair(BNO086_ID_MAGNETOMETER,         BNO086_PERIOD_MAGNETOMETER));
    // this->features.push_back(std::make_pair(BNO086_ID_LINEAR_ACCELERATION,  BNO086_PERIOD_LINEAR_ACCELERATION));
    // this->features.push_back(std::make_pair(BNO086_ID_GRAVITY,              BNO086_PERIOD_GRAVITY));


    //Reset BNO
    HAL_NVIC_DisableIRQ(EXTI2_IRQn); //Disable HINT_N interrupt
    HAL_GPIO_WritePin(IMU_NRST_GPIO_Port, IMU_NRST_Pin, GPIO_PIN_RESET);
    HAL_Delay(0);
    HAL_GPIO_WritePin(IMU_NRST_GPIO_Port, IMU_NRST_Pin, GPIO_PIN_SET);

    //After reset, we have to wait for BNO to assert HINT_N
    while(HAL_GPIO_ReadPin(IMU_INT_GPIO_Port, IMU_INT_Pin) == GPIO_PIN_RESET);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn); //Enable HINT_N interrupt


    //Read advertisemsent
    while(!this->msg_ready);
    this->msg_ready = false;
    uint8_t advertisement[284] = {0};
    HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_RESET); //Set CS low
    if(HAL_SPI_Receive(BNO086_SPI_HANDLE, advertisement, 284, 10) != HAL_OK) Error_Handler();
    HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_SET); //Set CS high
    
    //read initialize response
    while(!this->msg_ready);
    this->msg_ready = false;
    uint8_t init[40] = {0};
    HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_RESET); //Set CS low
    if(HAL_SPI_Receive(BNO086_SPI_HANDLE, init, 20, 100) != HAL_OK) Error_Handler();
    HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_SET); //Set CS high

    //read message from executable
    while(!this->msg_ready);
    this->msg_ready = false;
    uint8_t exec[10] = {0};
    HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_RESET); //Set CS low
    if(HAL_SPI_Receive(BNO086_SPI_HANDLE, exec, 5, 10) != HAL_OK) Error_Handler();
    HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_SET); //Set CS high

    return 0;
}


uint8_t BNO086::start()
{

    //Wake up BNO
    HAL_GPIO_WritePin(IMU_WAKE_GPIO_Port, IMU_WAKE_Pin, GPIO_PIN_RESET);
    while(!this->msg_ready);
    this->msg_ready = false;
    HAL_GPIO_WritePin(IMU_WAKE_GPIO_Port, IMU_WAKE_Pin, GPIO_PIN_SET);

    uint8_t errorcode = 0;
    // uint8_t rxBytes[21];
    uint8_t txBytes[21];
    
    txBytes[0] = 0x15;
    txBytes[1] = 0x00;
    txBytes[2] = 0x02;
    txBytes[3] = this->seqNum;
    txBytes[4] = 0xFD; // Set Feature Command (P.63): https://www.ceva-ip.com/wp-content/uploads/2019/10/SH-2-Reference-Manual.pdf
    

    for(auto& [id, period]: this->features)
    {
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

        HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_RESET); //Set CS low
        errorcode = HAL_SPI_Transmit(BNO086_SPI_HANDLE, txBytes, 21, 1000);
        HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_SET); //Set CS low
        if(errorcode != HAL_OK) return errorcode;
        this->seqNum = (this->seqNum + 1) % 256;

        //read Get Feature Response
        //this response is sent unsolicited on rate change
        while(!this->msg_ready);
        this->msg_ready = false;
        uint8_t rxBytes[21] = {0};
        HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_RESET); //Set CS low
        errorcode = HAL_SPI_Receive(BNO086_SPI_HANDLE, rxBytes, 12, 1000);
        HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_SET); //Set CS low
        if(errorcode != HAL_OK) return errorcode;
    }


    return errorcode;

}


uint8_t BNO086::update()
{
    if(!this->msg_ready) return 1; //No new message available
    this->msg_ready = false;

    //read header
    uint8_t shtp_header[4] = {0};
    HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_RESET); //Set CS low
    uint8_t errorcode = HAL_SPI_Receive(BNO086_SPI_HANDLE, shtp_header, 4, 100);
    HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_SET); //Set CS high
    if(errorcode != 0) return errorcode;

    //read time stamp
    uint8_t time_stamp[5] = {0};
    HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_RESET); //Set CS low
    errorcode = HAL_SPI_Receive(BNO086_SPI_HANDLE, time_stamp, 5, 100);
    HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_SET); //Set CS high
    if(errorcode != 0) return errorcode;

    //read report ID
    uint8_t report_id;
    HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_RESET); //Set CS low
    errorcode = HAL_SPI_Receive(BNO086_SPI_HANDLE, &report_id, 1, 100);
    HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_SET); //Set CS high
    if(errorcode != 0) return errorcode;
    //TODO: DMA SPI

    switch (report_id)
    {
    case BNO086_ID_ACCELEROMETER:
        //NOTE: This might introduce a hard fault, because of the packed struct.
        //      If this is the case, every variable has to be copied seperately

        HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_RESET); //Set CS low
        errorcode = HAL_SPI_Receive(BNO086_SPI_HANDLE, (uint8_t *)&this->accel_data, 8, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_SET); //Set CS high
        break;
    case BNO086_ID_GYROSCOPE:
        HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_RESET); //Set CS low
        errorcode = HAL_SPI_Receive(BNO086_SPI_HANDLE, (uint8_t *)&this->gyro_data, 8, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_SET); //Set CS high
        break;
    case BNO086_ID_MAGNETOMETER:
        HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_RESET); //Set CS low
        errorcode = HAL_SPI_Receive(BNO086_SPI_HANDLE, (uint8_t *)&this->mag_data, 8, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_SET); //Set CS high
        break;
    case BNO086_ID_LINEAR_ACCELERATION:
        HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_RESET); //Set CS low
        errorcode = HAL_SPI_Receive(BNO086_SPI_HANDLE, (uint8_t *)&this->lin_accel_data, 8, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_SET); //Set CS high
        break;
    case BNO086_ID_ROTATION:
        HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_RESET); //Set CS low
        errorcode = HAL_SPI_Receive(BNO086_SPI_HANDLE, (uint8_t *)&this->rot_data, 12, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_SET); //Set CS high
        break;
    case BNO086_ID_GRAVITY:
        HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_RESET); //Set CS low
        errorcode = HAL_SPI_Receive(BNO086_SPI_HANDLE, (uint8_t *)&this->grav_data, 8, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_SET); //Set CS high
        break;
    default:
        break;
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
    return raw / (float)(1 << q_point);
}
