#pragma once

typedef enum {
    FSM_STATE_INIT,
    FSM_STATE_IDLE,
    FSM_STATE_OP,
    FSM_STATE_ERROR
} fsm_state_t;