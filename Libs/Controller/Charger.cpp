#include "Charger.h"
#include <cstdint>

ChargerData __attribute__((section(CHARGER_SECTION_NAME))) Charger::status;
TransmitData __attribute__((section(CHARGER_SECTION_NAME))) Charger::tx_data;


Charger::Charger()
{
    Charger::status.ready = 0;
    Charger::status.active = 0;
    Charger::status.OC_fault = 0;
    Charger::status.OV_fault = 0;
    Charger::status.WD_fault = 0;
    Charger::status.vin_10mV = 0;
    Charger::status.vout_10mV = 0;
    Charger::status.imeas_mA = 0;

    Charger::tx_data.enable = 0;
    Charger::tx_data.clear_faults = 0;

    this->initialized = false;
}


/**
 * @brief Wait for the charger to be ready.
 * @param timeout The timeout value in milliseconds.
 * @return true if the charger is ready within the timeout, false otherwise.
 */
bool Charger::wait_ready(uint16_t timeout)
{
    uint32_t start_time = HAL_GetTick();
    while(HAL_GetTick() - start_time < timeout)
    {
        this->transmit_receive();
        if(Charger::status.ready) 
        {
            this->initialized = true;
            return true;
        }
    }
    return false;
}


/**
 * @brief Transmit data to the charger and receive its status. 
 */
void Charger::transmit_receive()
{
    if(!this->initialized) return; // Return if charger is not initialized
    if(HAL_SPI_GetState(CHARGER_SPI_HANDLE) != HAL_SPI_STATE_READY) return;
    HAL_SPI_TransmitReceive_IT(CHARGER_SPI_HANDLE, (uint8_t *)&Charger::tx_data, (uint8_t *)&Charger::status, sizeof(Charger::status));

}
