#include "FootController.h"

FootController::FootController():
    fsm_(),
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
    fsmActions_.background_ = std::bind(&FootController::FSM_bg, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    fsmActions_.notReadyToSwitchOn_ = std::bind(&FootController::FSM_notReadyToSwitchOn, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    fsmActions_.switchOnDisabled_ = std::bind(&FootController::FSM_switchOnDisabled, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    fsmActions_.readyToSwitchOn_ = std::bind(&FootController::FSM_readyToSwitchOn, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    fsmActions_.switchedOn_ = std::bind(&FootController::FSM_switchedOn, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    fsmActions_.operationEnabled_ = std::bind(&FootController::FSM_operationEnabled, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    fsmActions_.quickStopActive_ = std::bind(&FootController::FSM_quickStopActive, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    fsmActions_.faultReactionActive_ = std::bind(&FootController::FSM_faultReactionActive, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    fsmActions_.fault_ = std::bind(&FootController::FSM_fault, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

    fsm_.init(fsmActions_);

    //ECAT initialization
    ecat_slv_init(&this->config);
    //TODO: Set Obj. constants

    //Sensor initialization
    //TODO: Go to FMS Fault state if init fails
    if(imu.init() != 0) Error_Handler();
    if(ldc.init() != 0) Error_Handler();
    if(hall0.init() != 0) Error_Handler();
    if(hall1.init() != 0) Error_Handler();
    if(hall2.init() != 0) Error_Handler();
    if(hall3.init() != 0) Error_Handler();
    if(tof.init() != 0) Error_Handler();

    if(imu.start() != 0) Error_Handler();
    if(tof.start_ranging() != 0) Error_Handler();
}

FSMStatus FootController::FSM_bg(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
    imu.update();
    float f0 = ldc.readData(0);
    float f1 = ldc.readData(1);
    float f2 = ldc.readData(2);
    float f3 = ldc.readData(3);
    float bx = hall0.read_Bx();
    float by = hall0.read_By();
    float bz = hall0.read_Bz();
    float t = hall0.read_T();
    int status = tof.get_ranging_data();
    return FSMStatus();
}

FSMStatus FootController::FSM_notReadyToSwitchOn(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
    return FSMStatus();
}

FSMStatus FootController::FSM_switchOnDisabled(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
    return FSMStatus();
}

FSMStatus FootController::FSM_readyToSwitchOn(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
    return FSMStatus();
}

FSMStatus FootController::FSM_switchedOn(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
    return FSMStatus();
}

FSMStatus FootController::FSM_operationEnabled(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
    return FSMStatus();
}

FSMStatus FootController::FSM_quickStopActive(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
    return FSMStatus();
}

FSMStatus FootController::FSM_faultReactionActive(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
    return FSMStatus();
}

FSMStatus FootController::FSM_fault(FSMStatus state, uint16_t &status_word, int8_t &mode)
{
    return FSMStatus();
}
