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
    ecat_slv_init(&this->config);
    //TODO: Set Obj. constants

    //Sensor initialization
    //TODO: Go to FMS Fault state if init fails
    if(imu.init() != 0) Error_Handler();
    // if(ldc.init() != 0) Error_Handler();
    // if(hall0.init() != 0) Error_Handler();
    // if(hall1.init() != 0) Error_Handler();
    // if(hall2.init() != 0) Error_Handler();
    // if(hall3.init() != 0) Error_Handler();
    // if(tof.init() != 0) Error_Handler();

    if(imu.start() != 0) Error_Handler();
    // if(tof.start_ranging() != 0) Error_Handler();
}

void FootController::runCommunication()
{
    ecat_slv();
}

void FootController::magnetize(uint8_t time)
{
    //Set active flag
    this->active_magnetization = true;

    //Disable Charging while magnetizing
    // HAL_GPIO_WritePin(CHARGE_START_GPIO_Port, CHARGE_START_Pin, GPIO_PIN_RESET);

    //Ensure no shoot through occurs
    HAL_GPIO_WritePin(DRV_P_GPIO_Port, DRV_P_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DRV_M_GPIO_Port, DRV_M_Pin, GPIO_PIN_RESET);

    // Select GPIO output
    auto gpio_port = DRV_M_GPIO_Port;
    auto gpio_pin = DRV_M_Pin;
    if(!this->requested_magnetization && this->requested_demagnetization)
    {
        gpio_port = DRV_P_GPIO_Port;
        gpio_pin = DRV_P_Pin;
    }
    //Set Magnetization Status
    this->status_magnetization = this->requested_magnetization;

    //Start Timer
    TIM1->ARR = time * 100; // time in us
    if(HAL_TIM_Base_Start_IT(&htim1)!= HAL_OK) Error_Handler();
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
    imu.update();
    this->rotation_data.quaternion_i = imu.rot_data.quaternion_i;
    this->rotation_data.quaternion_j = imu.rot_data.quaternion_j;
    this->rotation_data.quaternion_k = imu.rot_data.quaternion_k;
    this->rotation_data.quaternion_real = imu.rot_data.quaternion_real;

    //ToF
    // (void)tof.get_ranging_data();
    //TODO: Compress Ranging data in a meaningful way

    //LDC & Hall Sensors
    // if(this->status_magnetization)
    // {
    //     //Perform Contact Estimation
    //     bool contact_0 = hall0.estimate_contact();
    //     bool contact_1 = hall1.estimate_contact();
    //     bool contact_2 = hall2.estimate_contact();
    //     bool contact_3 = hall3.estimate_contact();
    //     // Contact estimation is a bitmap containing the contact estimation for all four magnets
    //     this->contact_estimation = contact_0 + (contact_1 << 1) + (contact_2 << 2) + (contact_3 << 3);

    //     //Perform Force Estimation
    //     this->force_estimation = this->ldc.forceEstimation() * (this->contact_estimation != 0); //Force estimation is only valid, if contact estimation is not 0
    // } else 
    // {
    //     //If the magnet is not active, set contact and force estimation to 0
    //     this->contact_estimation = 0;
    //     this->force_estimation = 0;
    // }
    return state;
}

FSMStatus FootController::FSM_notReadyToSwitchOn(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
    return state;
}

FSMStatus FootController::FSM_switchOnDisabled(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
    // HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, GPIO_PIN_RESET); //Discharge Caps
    // HAL_GPIO_WritePin(CHARGE_START_GPIO_Port, CHARGE_START_Pin, GPIO_PIN_RESET);
    return state;
}

FSMStatus FootController::FSM_readyToSwitchOn(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
    // HAL_GPIO_WritePin(CHARGE_START_GPIO_Port, CHARGE_START_Pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, GPIO_PIN_RESET); //Discharge Caps
    return state;
}

FSMStatus FootController::FSM_switchedOn(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
    // if(HAL_GPIO_ReadPin(CHARGE_DONE_GPIO_Port, CHARGE_DONE_Pin) && !this->active_magnetization) //If the Caps are charged and no magnetization is active
    //     HAL_GPIO_WritePin(CHARGE_START_GPIO_Port, CHARGE_START_Pin, GPIO_PIN_SET); //If the Caps are not charged, start charging
    // else HAL_GPIO_WritePin(CHARGE_START_GPIO_Port, CHARGE_START_Pin, GPIO_PIN_RESET); //Else stop charging
    return state;
}

FSMStatus FootController::FSM_operationEnabled(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
    //Magnetization state
    //Handle Magnetization/Demagnetization requests
    if(this->requested_magnetization && this->requested_demagnetization)
    {
        //TODO: Handle faulty input
    }
    //Either Mafgnetization or Demagnetization was requested
    else if(this->requested_magnetization != this->requested_demagnetization)
    {
        //Check, if the Caps are charged, and no magnetization is active
        if(!HAL_GPIO_ReadPin(CHARGE_DONE_GPIO_Port, CHARGE_DONE_Pin) && !this->active_magnetization)
        {
            this->magnetize(MAGNETIZATION_TIME);
        }
    }
    //Charge Capacitors
    // if(!this->active_magnetization) HAL_GPIO_WritePin(CHARGE_START_GPIO_Port, CHARGE_START_Pin, GPIO_PIN_SET);

    //Discharge state
    HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, this->requested_discharge ? GPIO_PIN_RESET : GPIO_PIN_SET);
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

