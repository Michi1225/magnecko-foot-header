#include "Charger.h"

Charger::Charger()
{
    this->status.ready = 0;
    this->status.active = 0;
    this->status.OC_fault = 0;
    this->status.OV_fault = 0;
    this->status.WD_fault = 0;
    this->status.vin_10mV = 0;
    this->status.vout_10mV = 0;
    this->status.imeas_mA = 0;

    this->tx_data.enable = 0;
    this->tx_data.clear_faults = 0;
}

bool Charger::wait_ready(uint8_t timeout)
{
    uint32_t start_time = HAL_GetTick();
    while(HAL_GetTick() - start_time < timeout)
    {
        this->transmit_receive();
        if(this->status.ready) return true;
    }
    return false;
}

void Charger::transmit_receive()
{
    HAL_GPIO_WritePin(CHARGER_NCS_GPIO_Port, CHARGER_NCS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(CHARGER_SPI_HANDLE, (uint8_t *)&this->tx_data, (uint8_t *)&this->status, sizeof(this->status));
    while(HAL_SPI_GetState(CHARGER_SPI_HANDLE) != HAL_SPI_STATE_READY 
        || HAL_DMA_GetState(((CHARGER_SPI_HANDLE)->hdmarx)) != HAL_DMA_STATE_READY
        || HAL_DMA_GetState(((CHARGER_SPI_HANDLE)->hdmatx)) != HAL_DMA_STATE_READY);
    HAL_GPIO_WritePin(CHARGER_NCS_GPIO_Port, CHARGER_NCS_Pin, GPIO_PIN_SET);
}
