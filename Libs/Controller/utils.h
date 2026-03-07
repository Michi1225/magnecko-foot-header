#pragma once
#include "main.h"
#include "tim.h"

#define SWD_UTILS


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

