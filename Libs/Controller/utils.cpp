#include "utils.h"


// LED Utility Functions and Color Definitions
#ifdef LED_UTILS
void setStatusLED(uint8_t r, uint8_t g, uint8_t b, uint8_t intensity)
{
    //Clamp intensity to 0-255
    if(intensity > 255) intensity = 255;
    //Scale colors by intensity
    uint8_t r_scaled = (r * intensity) / 255;
    uint8_t g_scaled = (g * intensity) / 255;
    uint8_t b_scaled = (b * intensity) / 255;

    //Assuming Timer period is 255 for 8-bit resolution
    TIM2->CCR3 = g_scaled; //Green
    TIM12->CCR1 = b_scaled; //Blue
    TIM12->CCR2 = r_scaled; //Red
}

void disableStatusLED()
{
    TIM2->CCR3 = 0; //Green
    TIM12->CCR1 = 0; //Blue
    TIM12->CCR2 = 0; //Red
}

void setStatusLEDWarning()
{
    //Yellow color for warning
    setStatusLED(255, 255, 0);
}

void setStatusLEDError()
{
    //Red color for error
    setStatusLED(255, 0, 0);
}

void setStatusLEDHex(uint32_t hexColor, uint8_t intensity)
{
    uint8_t r = (hexColor >> 16) & 0xFF;
    uint8_t g = (hexColor >> 8) & 0xFF;
    uint8_t b = hexColor & 0xFF;
    setStatusLED(r, g, b, intensity);
}

#endif


enum ErrorCode {
    ERROR_NONE = 0,
    ERROR_GENERIC = 1,
    // Add more error codes as needed
};

void errorHandler()
{
    HAL_GPIO_WritePin(DRV_M_GPIO_Port, DRV_M_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DRV_P_GPIO_Port, DRV_P_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DISCHARGE_GPIO_Port, DISCHARGE_Pin, GPIO_PIN_RESET); //Discharge Caps
    //TODO: Disable Charging

    setStatusLEDError();
  
    for(int i = 0; i <= 162; ++i)
    {
      if( i != SysTick_IRQn) NVIC_DisableIRQ((IRQn_Type)i); //Disable all interrupts except SysTick
    }
    while(1)
    {
        // Blink Error LED
        setStatusLEDError();
        HAL_Delay(200);
        disableStatusLED();
        HAL_Delay(200);
    }
}





// SWD utils
#ifdef SWD_UTILS
    extern "C" int _write(int file, char *ptr, int len)
    {
    (void)file;
    for (int i = 0; i < len; i++) {
        ITM_SendChar((uint32_t)*ptr++);
    }
        return len;
    }


    void SWD_Init(void)
    {
    *(__IO uint32_t*)(0x5C001004) |= 0x00700000; // DBGMCU_CR D3DBGCKEN D1DBGCKEN TRACECLKEN
    
    //UNLOCK FUNNEL
    *(__IO uint32_t*)(0x5C004FB0) = 0xC5ACCE55; // SWTF_LAR
    *(__IO uint32_t*)(0x5C003FB0) = 0xC5ACCE55; // SWO_LAR
    
    //SWO current output divisor register
    //This divisor value (0x000000C7) corresponds to 400Mhz
    //To change it, you can use the following rule
    // value = (CPU Freq/sw speed )-1
    // devided by 2 because swo runs on half the System clock
    *(__IO uint32_t*)(0x5C003010) = ((SystemCoreClock / SWO_SPEED / 2) - 1); // SWO_CODR
    
    //SWO selected pin protocol register
    *(__IO uint32_t*)(0x5C0030F0) = 0x00000002; // SWO_SPPR
    
    //Enable ITM input of SWO trace funnel
    *(__IO uint32_t*)(0x5C004000) |= ENABLE_PORT0 << 0;
    *(__IO uint32_t*)(0x5C004000) |= ENABLE_PORT1 << 1;
    *(__IO uint32_t*)(0x5C004000) |= ENABLE_PORT2 << 2;
    *(__IO uint32_t*)(0x5C004000) |= ENABLE_PORT3 << 3;
    *(__IO uint32_t*)(0x5C004000) |= ENABLE_PORT4 << 4;
    *(__IO uint32_t*)(0x5C004000) |= ENABLE_PORT5 << 5;
    *(__IO uint32_t*)(0x5C004000) |= ENABLE_PORT6 << 6;
    *(__IO uint32_t*)(0x5C004000) |= ENABLE_PORT7 << 7;
    
    //RCC_AHB4ENR enable GPIOB clock
    *(__IO uint32_t*)(0x580244E0) |= 0x00000002;
    
    // Configure GPIOB pin 3 as AF
    *(__IO uint32_t*)(0x58020400) = (*(__IO uint32_t*)(0x58020400) & 0xffffff3f) | 0x00000080;
    
    // Configure GPIOB pin 3 Speed
    *(__IO uint32_t*)(0x58020408) |= 0x00000080;
    
    // Force AF0 for GPIOB pin 3
    *(__IO uint32_t*)(0x58020420) &= 0xFFFF0FFF;
    }
#endif
