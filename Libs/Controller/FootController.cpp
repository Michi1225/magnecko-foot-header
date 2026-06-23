#include "FootController.h"
#include "LEDController.h"
#include "main.h"


FootController::FootController() : fsm_(),
                                   fsmActions_(),
                                   imu(),
                                   hall0(TMAG5273::A1),
                                   hall1(TMAG5273::B1),
                                   hall2(TMAG5273::C1),
                                   hall3(TMAG5273::D1),
                                   tof(),
                                   charger()
{
}

void FootController::init()
{
    //LED Controller Initialization
    this->ledController.init();
    this->ledController.set_animation(LED_ANIMATION_CONFIGURING);

    //FSM initialization
    this->fsmActions_.background_ = std::bind(&FootController::FSM_bg, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    this->fsmActions_.notReadyToSwitchOn_ = std::bind(&FootController::FSM_notReadyToSwitchOn, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    this->fsmActions_.switchOnDisabled_ = std::bind(&FootController::FSM_switchOnDisabled, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    this->fsmActions_.readyToSwitchOn_ = std::bind(&FootController::FSM_readyToSwitchOn, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    this->fsmActions_.switchedOn_ = std::bind(&FootController::FSM_switchedOn, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    this->fsmActions_.operationEnabled_ = std::bind(&FootController::FSM_operationEnabled, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    this->fsmActions_.quickStopActive_ = std::bind(&FootController::FSM_quickStopActive, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    this->fsmActions_.faultReactionActive_ = std::bind(&FootController::FSM_faultReactionActive, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    this->fsmActions_.fault_ = std::bind(&FootController::FSM_fault, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

    fsm_.init(this->fsmActions_);

    // Read force estimation parameters from EEPROM
    #ifdef EEPROM_WRITE
    this->write_force_estimation_params_to_eeprom();
    #endif
    this->read_force_estimation_params_from_eeprom();

    
    //ECAT initialization
    while(!HAL_GPIO_ReadPin(EEPROM_LOADED_GPIO_Port, EEPROM_LOADED_Pin)){} //Wait for EEPROM to be loaded
    ecat_slv_init(&this->config);
    //TODO: Set Obj. constants
    Obj.EPM_Number = EPM_NUMBER; // EPM number, needed for hw interface




    //Sensor initialization 
    //TODO: Send Warnings via EtherCAT for sensor initialization failure, currently indicated via LED animation
    if(imu.init() != 0) 
    {

        this->ledController.set_animation(LED_ANIMATION_SENSOR_INIT_FAILED_IMU);
    }
    
    for(LDC1101 &ldc : this->ldc)
    {
        if(ldc.init() != HAL_OK) this->ledController.set_animation(LED_ANIMATION_SENSOR_INIT_FAILED_LDC);
    }

    this->active_ldc = &this->ldc[0]; // Pointer to the currently active LDC, used for SPI communication
    this->ldc[0].next = &this->ldc[1];
    this->ldc[1].next = &this->ldc[2];
    this->ldc[2].next = &this->ldc[3];
    this->ldc[3].next = &this->ldc[0]; // Circular linked list of LDCs for easy iteration



    if(TMAG5273::init() != 0) this->ledController.set_animation(LED_ANIMATION_SENSOR_INIT_FAILED_MAG);


    if(tof.init() != 0) this->ledController.set_animation(LED_ANIMATION_SENSOR_INIT_FAILED_TOF);


    HAL_Delay(10);
    
    if(imu.start() != 0) this->ledController.set_animation(LED_ANIMATION_SENSOR_INIT_FAILED_IMU);
    if(tof.start_ranging() != 0) this->ledController.set_animation(LED_ANIMATION_SENSOR_INIT_FAILED_TOF);


    // Charger Initialization
    if(charger.wait_ready(1000)) this->ledController.set_animation(LED_ANIMATION_CHARGER_NOT_RESPONDING); //Wait for charger to be ready, if not ready after 1 second, trigger error handler


    // Drive Timer Initialization
    uint8_t errorcode = 0;
    errorcode |= HAL_TIM_OnePulse_Start(TIM_DRV1, CHANNEL_DRV1); //Start One Pulse for DRV1
    errorcode |= HAL_TIM_OnePulse_Start(TIM_DRV2, CHANNEL_DRV2); //Start One Pulse for DRV2

    
    errorcode |= HAL_TIM_Base_Start_IT(TIM_CONTROL); //Start Control Timer
    errorcode |= HAL_TIM_Base_Start_IT(TIM_IMU); //Start BNO Timer

    if(errorcode != HAL_OK) this->controller_error_word.timer_init_failed = 1; //Set timer init failed flag if any of the timers failed to start

    HAL_GPIO_WritePin(GD_nEN_GPIO_Port, GD_nEN_Pin, GPIO_PIN_RESET); //Enable Gate Drivers

    HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, GPIO_PIN_SET);

    this->ledController.set_animation(LED_ANIMATION_SENSOR_INIT_FAILED_TOF); //Set LED animation to operational
}

void FootController::runCommunication()
{
    ecat_slv();
    this->charger.transmit_receive(); //Run Charger Communication Loop
}

void FootController::magnetize(uint8_t time)
{
    if(time < 1000 ||time > 10000) return; 


    if(!this->requested_magnetization && this->requested_demagnetization)
    {
        // Magnetization was requiested
        TIM_DRV1->Instance->CCR1 = time; // pulse width in us
        TIM_DRV1->Instance->CR1 |= TIM_CR1_CEN; // Start Timer
    }else if(this->requested_magnetization && !this->requested_demagnetization)
    {
        // Demagnetization was requested
        TIM_DRV2->Instance->CCR2 = time; // pulse width in us
        TIM_DRV2->Instance->CR1 |= TIM_CR1_CEN; // Start Timer
    } else 
    {
        return; //No valid request, do nothing
    }
    //Set Magnetization Status
    this->status_magnetization = this->requested_magnetization;
}

void FootController::runControl()
{
    this->fsm_.run();
}

FSMStatus FootController::FSM_bg(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
    // Controller Status
    fsm_.setControlWord(Obj.Control_Word);
    Obj.Status_Word = fsm_.getStatusWord();
    Obj.Magnet_Status = this->status_magnetization;



    //Handle Sensors
    //IMU
    Obj.IMU_Data.Acc_X = this->imu.output_lin_accel[0];
    Obj.IMU_Data.Acc_Y = this->imu.output_lin_accel[1];
    Obj.IMU_Data.Acc_Z = this->imu.output_lin_accel[2];

    Obj.IMU_Data.Gyro_X = this->imu.output_gyro[0];
    Obj.IMU_Data.Gyro_Y = this->imu.output_gyro[1];
    Obj.IMU_Data.Gyro_Z = this->imu.output_gyro[2];

    Obj.IMU_Data.Quat_I = this->imu.output_quat[0];
    Obj.IMU_Data.Quat_J = this->imu.output_quat[1];
    Obj.IMU_Data.Quat_K = this->imu.output_quat[2];
    Obj.IMU_Data.Quat_R = this->imu.output_quat[3];


    //ToF
    //TODO: Uncomment when PDO mapping is fixed
    // if(this->tof.data_valid)
    // {
    //     for(int i = 0; i < 8; i++)
    //     {
    //         for(int j = 0; j < 8; j++)
    //         {
    //             Obj.tof_distances[i][j] = this->tof.data_frame.pixel_data[i][j].distance;
    //             Obj.tof_confidences[i][j] = this->tof.data_frame.pixel_data[i][j].snr;
    //         }
    //     }
    // }



    // LDC
    //TODO: Uncomment when PDO mapping is fixed
    // for(int i = 0; i < 4; i++)
    // {
    //     LDC1101 &ldc = this->ldc[i];
    //     if(ldc.data_valid)
    //     {
    //         Obj.ldc_l[i] = ldc.rx_data.l_data; // Store the L data in the Obj structure
    //         Obj.ldc_rp[i] = ldc.rx_data.rp_data; // Store the RP data in the Obj structure
    //     }
    // }


    // TODO: Force estimation
    float current_estimate = this->force_estimation();
    this->force_average.push_back(current_estimate);
    if(this->force_average.size() > FORCE_ESTIMATE_WINDOW_SIZE) this->force_average.pop_front();
    float mag_force_average_sum = 0.0f;
    for(auto &val : this->force_average)
    {
        mag_force_average_sum += val;
    }
    Obj.Force_Estimate = mag_force_average_sum / this->force_average.size();



    return state;
}

FSMStatus FootController::FSM_notReadyToSwitchOn(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
    return state;
}

FSMStatus FootController::FSM_switchOnDisabled(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
    HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, GPIO_PIN_RESET); //Discharge Caps
    this->charger.tx_data.enable = 0; //Disable Charging
    status_word = status_word & ~FSMStatusWord::ROTOR_ALIGNING_STATUS;
    return state;
}

FSMStatus FootController::FSM_readyToSwitchOn(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
    this->charger.tx_data.enable = 1; //Enable Charging
    HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, GPIO_PIN_RESET); //Discharge Caps
    status_word = status_word & ~FSMStatusWord::ROTOR_ALIGNING_STATUS;
    return state;
}

FSMStatus FootController::FSM_switchedOn(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
    status_word = status_word & ~FSMStatusWord::ROTOR_ALIGNING_STATUS;
    return state;
}

FSMStatus FootController::FSM_operationEnabled(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
    HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, GPIO_PIN_SET);
    //Magnetization state
    //Handle Magnetization/Demagnetization requests
    if(this->requested_magnetization && this->requested_demagnetization)
    {
        //TODO: Handle faulty input
        this->requested_magnetization = false; //Reset requested magnetization state
        this->requested_demagnetization = false; //Reset requested demagnetization state
    }
    //Either Mafgnetization or Demagnetization was requested
    else if(this->requested_magnetization != this->requested_demagnetization)
    {
        this->magnetize(MAGNETIZATION_TIME);
        this->requested_magnetization = false; //Reset requested magnetization state
        this->requested_demagnetization = false; //Reset requested demagnetization state
    }
    return state;
}

FSMStatus FootController::FSM_quickStopActive(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
    return state;
}

FSMStatus FootController::FSM_faultReactionActive(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
    HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, GPIO_PIN_RESET); //Discharge Caps
    Error_Handler(); //Handle Fault
    return state;
}

FSMStatus FootController::FSM_fault(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
    return state;
}

float FootController::force_estimation()
{
    this->hall0.read_magnitude();
    this->hall1.read_magnitude();
    this->hall2.read_magnitude();
    this->hall3.read_magnitude();

    return TMAG5273::force_estimation(
        this->hall0.b_mag,
        this->hall1.b_mag,
        this->hall2.b_mag,
        this->hall3.b_mag
    );
}

void FootController::read_force_estimation_params_from_eeprom()
{
    // Read the force estimation parameters from the EEPROM
    uint8_t data[N_PARAMETERS * sizeof(float)];
    EEPROM_Read(0, 0, data, sizeof(data));

    // TODO: Validate parameters
    uint8_t is_valid = 1;
    if(is_valid)
    {
        std::memcpy(this->force_estimation_params, data, sizeof(this->force_estimation_params));
    }else this->controller_error_word.eeprom_params_invalid = 1; //Set error flag if parameters are invalid
}

void FootController::write_force_estimation_params_to_eeprom()
{
    // Write the force estimation parameters to the EEPROM
    #ifdef EEPROM_WRITE
    EEPROM_Write(0, 0, eeprom_img, sizeof(eeprom_img));
    #endif
}

