#pragma once

#include "main.h"


#define TS_ADC_HANDLE &hadc1

typedef enum
{
    PT1000 = 0,
    NTC10K_3977K = 1

}Thermistor_TypeDef_t;



HAL_StatusTypeDef ts_init();
float get_temperature(Thermistor_TypeDef_t type);



