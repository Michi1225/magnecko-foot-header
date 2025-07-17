#pragma once
#include "main.h"
#include <cstring>
#include <stdint.h>


// read only
#define DATA0_MSB_ADDR 0x00
#define DATA0_LSB_ADDR 0x01
#define DATA1_MSB_ADDR 0x02
#define DATA1_LSB_ADDR 0x03
#define DATA2_MSB_ADDR 0x04
#define DATA2_LSB_ADDR 0x05
#define DATA3_MSB_ADDR 0x06
#define DATA3_LSB_ADDR 0x07
#define STATUS_ADDR 0x18
#define MANUFACTURER_ID_ADDR 0x7E
#define LDC_DEVICE_ID_ADDR 0x7F

// read/write
#define RCOUNT0_ADDR 0x08
#define RCOUNT0_VAL  0x054B //500us

#define RCOUNT1_ADDR 0x09
#define RCOUNT1_VAL  0x054B //500us

#define RCOUNT2_ADDR 0x0A
#define RCOUNT2_VAL  0x054B //500us

#define RCOUNT3_ADDR 0x0B
#define RCOUNT3_VAL  0x054B //500us

#define OFFSET0_ADDR 0x0C
#define OFFSET0_VAL  0x00

#define OFFSET1_ADDR 0x0D
#define OFFSET1_VAL  0x00

#define OFFSET2_ADDR 0x0E
#define OFFSET2_VAL  0x00

#define OFFSET3_ADDR 0x0F
#define OFFSET3_VAL  0x00

#define SETTLECOUNT0_ADDR 0x10
#define SETTLECOUNT0_VAL  0x001B //10us

#define SETTLECOUNT1_ADDR 0x11
#define SETTLECOUNT1_VAL  0x001B //10us

#define SETTLECOUNT2_ADDR 0x12
#define SETTLECOUNT2_VAL  0x001B //10us

#define SETTLECOUNT3_ADDR 0x13
#define SETTLECOUNT3_VAL  0x001B //10us

#define CLOCK_DIVIDERS0_ADDR 0x14
#define CLOCK_DIVIDERS0_VAL  0x1001 // No divider

#define CLOCK_DIVIDERS1_ADDR 0x15
#define CLOCK_DIVIDERS1_VAL  0x1001 // No divider

#define CLOCK_DIVIDERS2_ADDR 0x16
#define CLOCK_DIVIDERS2_VAL  0x1001 // No divider

#define CLOCK_DIVIDERS3_ADDR 0x17
#define CLOCK_DIVIDERS3_VAL  0x1001 // No divider

#define ERROR_CONFIG_ADDR 0x19
#define ERROR_CONFIG_VAL  0xF800 // All errors on DATAx, no errors on INTB

#define CONFIG_ADDR 0x1A
#define CONFIG_VAL  0x81 //Disable interrupts

#define MUX_CONFIG_ADDR 0x1B
#define MUX_CONFIG_VAL  0xC20D //Auto-Scan enable, 10MHz deglitch

#define RESET_DEV_ADDR 0x1C
#define RESET_DEV_VAL  0x00

#define DRIVE_CURRENT0_ADDR 0x1E
#define DRIVE_CURRENT0_VAL  0x3F

#define DRIVE_CURRENT1_ADDR 0x1F
#define DRIVE_CURRENT1_VAL  0x3F

#define DRIVE_CURRENT2_ADDR 0x20
#define DRIVE_CURRENT2_VAL  0x3F

#define DRIVE_CURRENT3_ADDR 0x21
#define DRIVE_CURRENT3_VAL  0x3F


#define LDC_FREF 43350000.0 // 43.35MHz reference clock

#define DATAx_MSB_MASK 0x0F

#define LDC_I2C_ADDRESS 0x2B // LDC1614 I2C address

#define LDC_I2C_HANDLE &hi2c3 // I2C handle





typedef enum {
    LDC_OK,
    LDC_UNDER_RANGE,
    LDC_OVER_RANGE,
    LDC_WD_TIMEOUT,
    LDC_AMPLITUDE_ERROR
} LDC1614_Status;


class LDC1614 
{
public:
    LDC1614();
    uint8_t init();
    float readData(uint8_t channel);
    uint16_t readStatus();

    /**
     * @brief   Force estimation of the force on the sensor.
     * @retval  Force estimation in N. 0, if Contact estimation is 0.
     * @note    The resolution of the force estimation is 4N/LSB.
     */
    uint8_t forceEstimation();
};