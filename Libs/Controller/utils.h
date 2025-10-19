#pragma once
#include "main.h"
#include "tim.h"

#define LED_UTILS
#define SWD_UTILS


// LED Utility Functions and Color Definitions
#ifdef LED_UTILS
    #define COLOUR_RED     0xFF0000
    #define COLOUR_GREEN   0x00FF00
    #define COLOUR_BLUE    0x0000FF
    #define COLOUR_YELLOW  0xFFFF00
    #define COLOUR_CYAN    0x00FFFF
    #define COLOUR_MAGENTA 0xFF00FF
    #define COLOUR_WHITE   0xFFFFFF


    void setStatusLED(uint8_t r, uint8_t g, uint8_t b, uint8_t intensity=10);
    void disableStatusLED();
    void setStatusLEDWarning();
    void setStatusLEDError();
    void setStatusLEDHex(uint32_t hexColor, uint8_t intensity=10);
#endif

//Error Handling
void errorHandler();


// SWD utils
#ifdef SWD_UTILS
    #define SWO_SPEED 5E6 // 5Mhz
    #define ENABLE_PORT0 1
    #define ENABLE_PORT1 1
    #define ENABLE_PORT2 0
    #define ENABLE_PORT3 0
    #define ENABLE_PORT4 0
    #define ENABLE_PORT5 0
    #define ENABLE_PORT6 0
    #define ENABLE_PORT7 0

    extern "C" int _write(int file, char *ptr, int len);
    void SWD_Init();
#endif

