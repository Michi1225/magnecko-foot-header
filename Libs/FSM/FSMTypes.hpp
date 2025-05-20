/**
  * RSL File header
  * @author Lennart Nachtigall
  * @copyright RSL - 2020
  * NOTE these definitions come originally from the anydrive code
*/
#pragma once
#include <functional>
#include <cstdint>

namespace FSMBitMask {
    constexpr uint8_t SHUTDOWN = 0b10000111;
    constexpr uint8_t SWITCH_ON = 0b10001111;
    constexpr uint8_t ENABLE_OPERATION = 0b10001111;
    constexpr uint8_t DISABLE_VOLTAGE = 0b10000010;
    constexpr uint8_t QUICK_STOP = 0b10000110;
    constexpr uint8_t DISABLE_OPERATION = 0b10001111;
    constexpr uint8_t FAULT_RESET = 0b10000000;
};

namespace FSMCommand {
    constexpr uint8_t SHUTDOWN = 0b00000110;
    constexpr uint8_t SWITCH_ON = 0b00000111;
    constexpr uint8_t ENABLE_OPERATION = 0b00001111;
    constexpr uint8_t DISABLE_VOLTAGE = 0b00000000;
    constexpr uint8_t QUICK_STOP = 0b00000010;
    constexpr uint8_t DISABLE_OPERATION = 0b00000111;
    constexpr uint8_t FAULT_RESET = 0b10000000;
};
namespace FSMStatusWord {
    constexpr uint16_t MASK =                   0b0000000001111111;
    constexpr uint16_t NOT_READY_TO_SWITCH_ON = 0b0000000000010000;
    constexpr uint16_t SWITCH_ON_DISABLED =     0b0000000001010000;
    constexpr uint16_t READY_TO_SWITCH_ON =     0b0000000000110001;
    constexpr uint16_t SWITCHED_ON =            0b0000000000110011;
    constexpr uint16_t OPERATION_ENABLED =      0b0000000000110111;
    constexpr uint16_t QUICK_STOP_ACTIVE =      0b0000000000010111;
    constexpr uint16_t FAULT_REACTION_ACTIVE =  0b0000000000011111;
    constexpr uint16_t FAULT =                  0b0000000000011000;
    constexpr uint16_t ROTOR_ALIGNING_STATUS =  1 << 14;
    constexpr uint16_t FAULT_STATUS =           1 << 3;
    constexpr uint16_t WARNING_STATUS =         1 << 7;
    constexpr uint16_t CALIBRATION_NEEDED =     1 << 13;
};


enum class FSMStatus
{
    /* The FSM state status */
    NA        = 0,
    NOT_READY_TO_SWITCH_ON = 1,
    SWITCH_ON_DISABLED = 2,
    READY_TO_SWITCH_ON = 3,
    SWITCHED_ON = 4,
    OPERATION_ENABLED   = 5,
    QUICK_STOP_ACTIVE   = 6,
    FAULT_REACTION_ACTIVE = 7,
    FAULT     = 8
};

struct FSMActions
{
    typedef  std::function<FSMStatus(FSMStatus, uint16_t& status_word, int8_t& mode)> action;

    action background_;

    action notReadyToSwitchOn_;

    action switchOnDisabled_;

    action readyToSwitchOn_;

    action switchedOn_;

    action operationEnabled_;

    action quickStopActive_;

    action faultReactionActive_; // only way to exit this function is automatically in the function

    action fault_;

};
  
  
  
  
  