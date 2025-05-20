#pragma once

#include "main.h"
#include "BNO086.h"
#include "LDC1614.h"
#include "TMAG5273.h"
#include "VL53L7CH.h"
#include "FSM.hpp"
extern "C" {
    #include "ecat_slv.h"
    #include "utypes.h"
    #include "soes_pin_mapping_def.h"
  }

class FootController
{
private:

    //ECAT variables
    pin_mapping_typedef pin_mapping = {&hspi6, ECAT_NCS_GPIO_Port, ECAT_NCS_Pin, EEPROM_LOADED_GPIO_Port, EEPROM_LOADED_Pin};
    esc_cfg_t config =
    {
       /* User input to stack */
       .user_arg = &pin_mapping, /* passed along to ESC_config and ESC_init */

       /* Mandatory input to stack */
       .use_interrupt = 0, /* flag telling the stack if the user application will use
                              interrupts, 0= Polling, 1 = Mixed Polling/Interrupt
                              and Interrupt */

       .watchdog_cnt = 100, /* non UNIT watchdog counter, for the application
                               developer to decide UNIT. This example set 100
                               cnt and by calling ecat_slv or
                               DIG_process(DIG_PROCESS_WD_FLAG) every 1ms,
                               it creates a watchdog running at ~100ms. */

       /* Optional input to stack for user application interaction with the stack
        * all functions given must be implemented in the application.
        */
       .set_defaults_hook = NULL, /* hook called after stack have loaded known
                                     default values, possible for application
                                     to load values not known in compile time
                                     or that need to be overwritten */
       .pre_state_change_hook = NULL, /* hook called before state transition */
       .post_state_change_hook = NULL, /* hook called after state transition */

       .application_hook = NULL, /* hook in application loop called when
                                    DIG_process(DIG_PROCESS_APP_HOOK_FLAG) */
       .safeoutput_override = NULL, /* user override of default safeoutput when stack
                                       stop outputs */

       .pre_object_download_hook = NULL, /* hook called before object download,
                                            if hook return != 0 the download will not
                                            take place */
       .post_object_download_hook = NULL, /* hook called after object download */

       .rxpdo_override = NULL, /* user override of default rxpdo */
       .txpdo_override = NULL, /* user override of default txpdo */

       /* Mandatory input to stack for SM and DC synchronous applications */
       .esc_hw_interrupt_enable = NULL, /* callback to function that enable IRQ
                                           based on the Event MASK */
       .esc_hw_interrupt_disable = NULL, /* callback to function that disable IRQ
                                            based on the Event MASK */

       /* Mandatory input for emulated eeprom */
       .esc_hw_eep_handler = NULL, /* callback to function that handle an emulated eeprom */

       /* Mandatory if Distributed Clocks get activated by the master */
       .esc_check_dc_handler = NULL /* Check DC synchronous settings, must be set if
                                      DC synchronisation activated, or state change
                                      PREOP->SAFEOP will fail.*/
    };

    //FSM variables
    FSM fsm_;
    FSMActions fsmActions_;

    public:
    //Sensor variables
    BNO086 imu;
    LDC1614 ldc;
    TMAG5273 hall0;
    TMAG5273 hall1;
    TMAG5273 hall2;
    TMAG5273 hall3;
    VL53L7CH tof;
    
    FootController();
    void init();
    void runCommunication();
    void runControl();

    //FSM actions
    FSMStatus FSM_bg(FSMStatus state, uint16_t& status_word, int8_t& mode);
    FSMStatus FSM_notReadyToSwitchOn(FSMStatus state, uint16_t& status_word, int8_t& mode);
    FSMStatus FSM_switchOnDisabled(FSMStatus state, uint16_t& status_word, int8_t& mode);
    FSMStatus FSM_readyToSwitchOn(FSMStatus state, uint16_t& status_word, int8_t& mode);
    FSMStatus FSM_switchedOn(FSMStatus state, uint16_t& status_word, int8_t& mode);
    FSMStatus FSM_operationEnabled(FSMStatus state,  uint16_t& status_word, int8_t& mode);
    FSMStatus FSM_quickStopActive(FSMStatus state, uint16_t& status_word, int8_t& mode);
    FSMStatus FSM_faultReactionActive(FSMStatus state, uint16_t& status_word, int8_t& mode);
    FSMStatus FSM_fault(FSMStatus state, uint16_t& status_word, int8_t& mode);
};