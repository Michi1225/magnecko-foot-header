#include "FootController.h"


FootController::FootController() : fsm_(),
                                   fsmActions_(),
                                   imu(),
                                   ldc(),
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
    //TODO: Implement LED Controller over I2C

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

    // Charger Initialization
    if(charger.wait_ready(1000)) Error_Handler(); //Wait for charger to be ready, if not ready after 1 second, trigger error handler


    // Drive Timer Initialization
    HAL_TIM_OnePulse_Start(TIM_DRV1, CHANNEL_DRV1); //Start One Pulse for DRV1
    HAL_TIM_OnePulse_Start(TIM_DRV2, CHANNEL_DRV2); //Start One Pulse for DRV2

    //TODO: LED Indication for successful initialization
}

void FootController::runCommunication()
{
    ecat_slv();
    this->charger.transmit_receive(); //Run Charger Communication Loop
}

void FootController::magnetize(uint8_t time)
{
    if(time < 10 ||time > 100) return; 


    if(!this->requested_magnetization && this->requested_demagnetization)
    {
        // Magnetization was requiested
        TIM2->CCR1 = 100 * time; // pulse width in us
        TIM2->CR1 |= TIM_CR1_CEN; // Start Timer
    }else if(this->requested_magnetization && !this->requested_demagnetization)
    {
        // Demagnetization was requested
        TIM5->CCR2 = 100 * time; // pulse width in us
        TIM5->CR1 |= TIM_CR1_CEN; // Start Timer
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

