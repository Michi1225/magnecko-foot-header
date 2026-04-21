#include "FootController.h"


FootController::FootController() : fsm_(),
                                   fsmActions_(),
                                   imu(),
                                   ldc(),
                                   hall0(TMAG5273::A1),
                                   hall1(TMAG5273::B1),
                                   hall2(TMAG5273::C1),
                                   hall3(TMAG5273::D1),
                                   tof()
{
}

void FootController::init()
{
    //LED Timer Initialization
    TIM2->CCR3 = 0; //Status LED Green off
    TIM12->CCR1 = 0; //Status LED Blue off
    TIM12->CCR2 = 0; //Status LED Red off
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);   //Start PWM for Status LED Green
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);  //Start PWM for Status LED Blue
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);  //Start PWM for Status LED Red

    setStatusLEDHex(COLOUR_WHITE); //Indicate initialization start

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

    //ECAT initialization
    
    while(!HAL_GPIO_ReadPin(EEPROM_LOADED_GPIO_Port, EEPROM_LOADED_Pin)){} //Wait for EEPROM to be loaded
    ecat_slv_init(&this->config);
    //TODO: Set Obj. constants
    Obj.EPM_Number = 1; // EPM number, needed for hw interface
    // Obj.Device_Information[6] = 1; // actuator number, needed for hw interface

    //Sensor initialization
    //TODO: Go to FMS Fault state if init fails
    if(imu.init() != 0) Error_Handler();
    
    if(ldc.init() != 0) Error_Handler();
    if(TMAG5273::init() != 0) Error_Handler();
    if(tof.init() != 0) Error_Handler();
    HAL_Delay(10);
    
    if(imu.start() != 0) Error_Handler();
    if(tof.start_ranging() != 0) Error_Handler();

    this->force_estimation = 0;

    setStatusLEDHex(COLOUR_CYAN); //Indicate initialization done
}

void FootController::runCommunication()
{
    ecat_slv();


    
}

void FootController::magnetize(uint8_t time)
{
    if(time < 10 ||time > 100) return; 
    //Set active flag
    this->active_magnetization = true;

    //Disable Charging while magnetizing
    //TODO: ???

    //Ensure no shoot through occurs
    HAL_GPIO_WritePin(DRV_P_GPIO_Port, DRV_P_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DRV_M_GPIO_Port, DRV_M_Pin, GPIO_PIN_RESET);

    // Select GPIO output
    auto gpio_port = DRV_P_GPIO_Port;
    auto gpio_pin = DRV_P_Pin;
    if(!this->requested_magnetization && this->requested_demagnetization)
    {
        gpio_port = DRV_M_GPIO_Port;
        gpio_pin = DRV_M_Pin;
    }
    //Set Magnetization Status
    this->status_magnetization = this->requested_magnetization;
    this->charge_done = false; //Reset charge done flag

    //Start Timer
    TIM1->ARR = time * 100; // time in us
    if(HAL_TIM_Base_Start_IT(&htim1)!= HAL_OK) return;
    HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_SET);
}

void FootController::runControl()
{
    this->fsm_.run();
}

FSMStatus FootController::FSM_bg(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
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


    Obj.Force_Estimate = this->force_estimation;


    //ToF
    
    std::memcpy(Obj.ToF_Data, this->tof.data, sizeof(this->tof.data));

    fsm_.setControlWord(Obj.Control_Word);
    Obj.Status_Word = fsm_.getStatusWord();
    return state;
}

FSMStatus FootController::FSM_notReadyToSwitchOn(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
    
    return state;
}

FSMStatus FootController::FSM_switchOnDisabled(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
    HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, GPIO_PIN_RESET); //Discharge Caps
    //TODO: Handle disabling charging
    status_word = status_word & ~FSMStatusWord::ROTOR_ALIGNING_STATUS;
    setStatusLEDHex(COLOUR_CYAN);
    return state;
}

FSMStatus FootController::FSM_readyToSwitchOn(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
    //TODO: Handle enabling charging
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
    if(state != FSMStatus::OPERATION_ENABLED) // Just once when entering the state
    {
        setStatusLEDHex(COLOUR_GREEN);
    }

    if(Obj.Magnet_Command == 2){
        setStatusLEDHex(COLOUR_MAGENTA);
    }else if(Obj.Magnet_Command == 1){
        setStatusLEDHex(0xFFFF00);
    }else{
        setStatusLEDHex(COLOUR_GREEN);
    }

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

