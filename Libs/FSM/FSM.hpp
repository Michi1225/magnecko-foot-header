/**
  * RSL File header
  * @author Lennart Nachtigall
  * @author Original author (C code) prob. Vasilious Tsounis
  * @copyright RSL - 2020
  * Note original implementation that was taken as example for this was from Vasilious
  */
 #pragma once

 #include "FSMTypes.hpp"
 #include <map>
 #include <cstdint>
 enum class ErrorCodes {
    NO_ERROR = 0x0000,
    OVER_CURRENT = 0x2220,
    OVER_TEMPERATURE = 0x4300,
    CONTROL_FAULT = 0x8A00,
    NO_ENCODER_CALIBRATION = 0xFF00,
    TEMPERATURE_SENSORS_UNRELIABLE = 0xFF01
};

class FSM
{
public:
    FSM();
    void setControlWord(uint16_t input);
    void setMode(int8_t input);
    void exitFaultReaction();
    void triggerFaultReaction(ErrorCodes error_code);
    void run();
    const FSMStatus& state(){return  state_;}
    void init(const FSMActions& actions);
    void setWarning(ErrorCodes error_code){error_code_ = static_cast<uint16_t>(error_code);}
    uint16_t getStatusWord(){return status_word_;}
    uint8_t getOperationModeDisplay(){return mode_;}
    uint16_t getErrorCode(){return error_code_;}
    uint16_t error_code_ = 0;
private:
    FSMActions actions_;
    std::map<FSMStatus,std::function<void(void)>> states_;
    FSMStatus state_;
    uint16_t control_word_;
    uint16_t status_word_;
    int8_t mode_;

    std::function<void(void)> next_state_;
    /* State functions declarations */
    void notReadyToSwitchOn();

    void switchOnDisabled();

    void readyToSwitchOn();

    void switchedOn();

    void operationEnabled();

    void quickStopActive();

    void faultReactionActive();

    void fault();
};
 
 
 