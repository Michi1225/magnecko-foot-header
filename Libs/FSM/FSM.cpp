/**
  * RSL File header
  * @author Lennart Nachtigall
  * @author Original author (C code) prob. Vasilious Tsounis
  * @copyright RSL - 2020
  * Note original implementation that was taken as example for this was from Vasilious
  */
#include "FSM.hpp"
FSM::FSM()
{
    states_[FSMStatus::NOT_READY_TO_SWITCH_ON] = std::bind(&FSM::notReadyToSwitchOn,this);
    states_[FSMStatus::SWITCH_ON_DISABLED] = std::bind(&FSM::switchOnDisabled,this);
    states_[FSMStatus::READY_TO_SWITCH_ON] = std::bind(&FSM::readyToSwitchOn,this);
    states_[FSMStatus::SWITCHED_ON] = std::bind(&FSM::switchedOn,this);
    states_[FSMStatus::OPERATION_ENABLED  ] = std::bind(&FSM::operationEnabled,this);
    states_[FSMStatus::QUICK_STOP_ACTIVE  ] = std::bind(&FSM::quickStopActive,this);
    states_[FSMStatus::FAULT_REACTION_ACTIVE] = std::bind(&FSM::faultReactionActive,this);
    states_[FSMStatus::FAULT    ] = std::bind(&FSM::fault,this);
}


void FSM::exitFaultReaction()
{
    if(state_ == FSMStatus::FAULT_REACTION_ACTIVE) next_state_ = states_[FSMStatus::FAULT];
}

void FSM::triggerFaultReaction(ErrorCodes error_code)
{
    error_code_ = static_cast<uint16_t>(error_code);
    next_state_ = states_[FSMStatus::FAULT_REACTION_ACTIVE];
}


void FSM::setControlWord(uint16_t input)
{
    control_word_ = input;
}

void FSM::setMode(int8_t input)
{
    //kind of hacky way to prevent switching of modes during enabled state
    if(state_ != FSMStatus::OPERATION_ENABLED){
        mode_ = input;
    }
}

void FSM::run()
{
    if(next_state_)
        next_state_();

    actions_.background_(state_, status_word_, mode_);
}

void FSM::init(const FSMActions &actions)
{
    actions_ = actions;
    next_state_ = states_[FSMStatus::NOT_READY_TO_SWITCH_ON];
    state_ = FSMStatus::NOT_READY_TO_SWITCH_ON;

}

void FSM::notReadyToSwitchOn()
{
    auto prev_state = state_;
    state_ = FSMStatus::NOT_READY_TO_SWITCH_ON;

    status_word_ =   (~FSMStatusWord::MASK & status_word_)  | FSMStatusWord::NOT_READY_TO_SWITCH_ON;
    actions_.notReadyToSwitchOn_(prev_state, status_word_, mode_);

    next_state_ = states_[FSMStatus::SWITCH_ON_DISABLED];
    run();
}

void FSM::switchOnDisabled()
{
    auto prev_state = state_;
    state_ = FSMStatus::SWITCH_ON_DISABLED;
  
    if((control_word_ & FSMBitMask::SHUTDOWN) == FSMCommand::SHUTDOWN)
    {
        next_state_ = states_[FSMStatus::READY_TO_SWITCH_ON];
        run();
    }
    else
    {
        status_word_ = (~FSMStatusWord::MASK & status_word_)  | FSMStatusWord::SWITCH_ON_DISABLED;
        actions_.switchOnDisabled_(prev_state, status_word_, mode_);
    }
}

void FSM::readyToSwitchOn()
{
    auto prev_state = state_;
    state_ = FSMStatus::READY_TO_SWITCH_ON;

    if(((control_word_ & FSMBitMask::SWITCH_ON) == FSMCommand::SWITCH_ON) || ((control_word_ & FSMBitMask::ENABLE_OPERATION)== FSMCommand::ENABLE_OPERATION))
    {
        next_state_ = states_[FSMStatus::SWITCHED_ON];
        run();
    }
    else if ((control_word_ & FSMBitMask::DISABLE_VOLTAGE) == FSMCommand::DISABLE_VOLTAGE)
    {
        next_state_ = states_[FSMStatus::SWITCH_ON_DISABLED];
        run();
    }
    else
    {
        status_word_ = (~FSMStatusWord::MASK & status_word_)  | FSMStatusWord::READY_TO_SWITCH_ON;
        actions_.readyToSwitchOn_(prev_state, status_word_, mode_);
    }
}

void FSM::switchedOn()
{
    auto prev_state = state_;
    state_ = FSMStatus::SWITCHED_ON;

    if((control_word_ & FSMBitMask::ENABLE_OPERATION) == FSMCommand::ENABLE_OPERATION)
    {
        // so at least once its executed...
        actions_.switchedOn_(prev_state, status_word_, mode_);
        next_state_ = states_[FSMStatus::OPERATION_ENABLED];
        run();
    } else if ((control_word_ & FSMBitMask::DISABLE_VOLTAGE) == FSMCommand::DISABLE_VOLTAGE)
    {
        next_state_ = states_[FSMStatus::SWITCH_ON_DISABLED];
        run();
    }
    else if ((control_word_ & FSMBitMask::SHUTDOWN) == FSMCommand::SHUTDOWN)
    {
        next_state_ = states_[FSMStatus::READY_TO_SWITCH_ON];
        run();
    }
    else
    {
        status_word_ = (~FSMStatusWord::MASK & status_word_)  | FSMStatusWord::SWITCHED_ON;
        actions_.switchedOn_(prev_state, status_word_, mode_);
    }
}

void FSM::operationEnabled()
{
    auto prev_state = state_;
    state_ = FSMStatus::OPERATION_ENABLED;

    if((control_word_ & FSMBitMask::QUICK_STOP) == FSMCommand::QUICK_STOP)
    {
        next_state_ = states_[FSMStatus::QUICK_STOP_ACTIVE];
        run();
    } else if ((control_word_ & FSMBitMask::DISABLE_OPERATION) == FSMCommand::DISABLE_OPERATION)
    {
        next_state_ = states_[FSMStatus::SWITCHED_ON];
        run();
    }
    else if ((control_word_ & FSMBitMask::SHUTDOWN) == FSMCommand::SHUTDOWN)
    {
        next_state_ = states_[FSMStatus::READY_TO_SWITCH_ON];
        run();
    }
    else if ((control_word_ & FSMBitMask::DISABLE_VOLTAGE) == FSMCommand::DISABLE_VOLTAGE)
    {
        next_state_ = states_[FSMStatus::SWITCH_ON_DISABLED];
        run();
    }
    else
    {
        status_word_ = (~FSMStatusWord::MASK & status_word_)  |  FSMStatusWord::OPERATION_ENABLED;
        actions_.operationEnabled_(prev_state, status_word_, mode_);
    }
}

void FSM::quickStopActive()
{
    auto prev_state = state_;
    state_ = FSMStatus::QUICK_STOP_ACTIVE;

    if((control_word_ & FSMBitMask::DISABLE_VOLTAGE) == FSMCommand::DISABLE_VOLTAGE)
    {
        next_state_ = states_[FSMStatus::SWITCH_ON_DISABLED];
        run();
    }
    else if ((control_word_ & FSMBitMask::ENABLE_OPERATION) == FSMCommand::ENABLE_OPERATION)
    {
        next_state_ = states_[FSMStatus::OPERATION_ENABLED];
        run();
    }
    else
    {
        status_word_ = (~FSMStatusWord::MASK & status_word_)  | FSMStatusWord::QUICK_STOP_ACTIVE;
        actions_.quickStopActive_(prev_state, status_word_, mode_);
    }

}

void FSM::faultReactionActive()
{
    auto prev_state = state_;
    state_ = FSMStatus::FAULT_REACTION_ACTIVE;

    status_word_ = (~FSMStatusWord::MASK & status_word_)  |  FSMStatusWord::FAULT_REACTION_ACTIVE;
    actions_.faultReactionActive_(prev_state, status_word_, mode_);
}

void FSM::fault()
{
    auto prev_state = state_;
    state_ = FSMStatus::FAULT;

    if((control_word_ & FSMBitMask::FAULT_RESET) == FSMCommand::FAULT_RESET)
    {
        next_state_ = states_[FSMStatus::SWITCH_ON_DISABLED];
        run();
    }
    else
    {
        status_word_ = (~FSMStatusWord::MASK & status_word_)  | FSMStatusWord::FAULT;
        actions_.fault_(prev_state, status_word_, mode_);
    }
}
