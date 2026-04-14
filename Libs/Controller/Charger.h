#pragma once

#include "main.h"

#define CHARGER_SPI_HANDLE &hspi2
#define CHARGER_SECTION_NAME ".RAM_D3"

typedef struct
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
}ChargerData;

typedef struct
{
    uint8_t enable       :   1;
    uint8_t clear_faults :   1;
    uint8_t              :   6;
    uint16_t             :   16;
    uint16_t             :   16;
    uint16_t             :   16;
}TransmitData;

class Charger
{
private:
public:
    static ChargerData status;
    static TransmitData tx_data;

    Charger();

    bool wait_ready(uint8_t timeout);
    void transmit_receive();

};