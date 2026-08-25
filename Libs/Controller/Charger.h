#pragma once

#include "BNO086.h"
#include "main.h"
#include <cstdint>

#define CHARGER_SPI_HANDLE (&hspi6)
#define CHARGER_SECTION_NAME ".RAM_D3"

typedef struct PACKED
{
    uint8_t ready      :   1;
    uint8_t active     :   1;
    uint8_t OC_fault   :   1;
    uint8_t OV_fault   :   1;
    uint8_t WD_fault   :   1;
    uint8_t            :   3;
    uint16_t vin_10mV;
    uint16_t vout_10mV;
    uint16_t imeas_mA;
    //TODO: Add CRC
}ChargerData;

typedef struct PACKED
{
    uint8_t enable       :   1;
    uint8_t clear_faults :   1;
    uint8_t              :   6;
    uint16_t reserved[3];
}TransmitData;




class Charger
{
private:
public:
    static ChargerData status;
    static TransmitData tx_data;
    bool initialized;

    Charger();

    bool wait_ready(uint16_t timeout);
    void transmit_receive();
    void transmit_receive_init();

};