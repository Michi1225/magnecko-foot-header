#include "thermistor.h"
#include "stm32h7xx_hal_adc.h"
#include "stm32h7xx_hal_adc_ex.h"


uint32_t raw_adc_val = 0;

HAL_StatusTypeDef ts_init()
{
    HAL_StatusTypeDef error = HAL_OK;

    error |= HAL_ADCEx_Calibration_Start(TS_ADC_HANDLE, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    error |= HAL_ADC_Start_DMA(TS_ADC_HANDLE, &raw_adc_val, 1);

    return error;
}

float get_temperature(Thermistor_TypeDef_t type)
{

    if(raw_adc_val >= 3000)
        return 250;
    return temperature_lookup[type][raw_adc_val];

}