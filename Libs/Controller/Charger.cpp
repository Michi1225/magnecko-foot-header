#include "Charger.h"
#include "stm32h7xx_hal_crc.h"
#include <cstdint>
#include <cstring>

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
    this->data_valid = false;
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
        this->transmit_receive_init();
        if(Charger::status.ready && this->data_valid) 
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
    size_t tx_size_bytes = sizeof(Charger::tx_data) - sizeof(Charger::tx_data.crc);
    Charger::tx_data.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)&Charger::tx_data, (uint32_t)tx_size_bytes);
    this->data_valid = false; // Reset data_valid flag before starting the transfer
    HAL_SPI_TransmitReceive_IT(CHARGER_SPI_HANDLE, (uint8_t *)&Charger::tx_data, (uint8_t *)&Charger::status, sizeof(Charger::status));

}

void Charger::transmit_receive_init()
{

    if(HAL_SPI_GetState(CHARGER_SPI_HANDLE) != HAL_SPI_STATE_READY) return;
    size_t tx_size_bytes = sizeof(Charger::tx_data) - sizeof(Charger::tx_data.crc);
    Charger::tx_data.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)&Charger::tx_data, (uint32_t)tx_size_bytes);
    this->data_valid = false; // Reset data_valid flag before starting the transfer
    HAL_SPI_TransmitReceive_IT(CHARGER_SPI_HANDLE, (uint8_t *)&Charger::tx_data, (uint8_t *)&Charger::status, sizeof(Charger::status));
}

bool Charger::rx_data_validate()
{
    size_t rx_size_bytes = sizeof(Charger::status) - sizeof(Charger::status.crc);
    uint32_t calculated_crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)&Charger::status, (uint32_t)rx_size_bytes);
    this->data_valid = (calculated_crc == Charger::status.crc);
    if(!this->data_valid)
    {
        memset(&(Charger::status), 0, sizeof(Charger::status));
    }
    return this->data_valid;
}
