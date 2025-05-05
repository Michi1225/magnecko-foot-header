#include "LDC1614.h"

uint8_t LDC1614::init() {

    uint8_t txData[2];

    txData[0] = (RCOUNT0_VAL >> 8) & 0xFF; // High byte
    txData[1] = RCOUNT0_VAL & 0xFF;        // Low byte
    uint8_t status = HAL_I2C_Mem_Write(LDC_I2C_HANDLE, LDC_I2C_ADDRESS << 1, RCOUNT0_ADDR, 1, txData, 2, 100);

    txData[0] = (RCOUNT1_VAL >> 8) & 0xFF; // High byte
    txData[1] = RCOUNT1_VAL & 0xFF;        // Low byte
    status |= HAL_I2C_Mem_Write(LDC_I2C_HANDLE, LDC_I2C_ADDRESS << 1, RCOUNT1_ADDR, 1, txData, 2, 100);

    txData[0] = (RCOUNT2_VAL >> 8) & 0xFF; // High byte
    txData[1] = RCOUNT2_VAL & 0xFF;        // Low byte
    status |= HAL_I2C_Mem_Write(LDC_I2C_HANDLE, LDC_I2C_ADDRESS << 1, RCOUNT2_ADDR, 1, txData, 2, 100);

    txData[0] = (RCOUNT3_VAL >> 8) & 0xFF; // High byte
    txData[1] = RCOUNT3_VAL & 0xFF;        // Low byte
    status |= HAL_I2C_Mem_Write(LDC_I2C_HANDLE, LDC_I2C_ADDRESS << 1, RCOUNT3_ADDR, 1, txData, 2, 100);

    txData[0] = (OFFSET0_VAL >> 8) & 0xFF; // High byte
    txData[1] = OFFSET0_VAL & 0xFF;        // Low byte
    status |= HAL_I2C_Mem_Write(LDC_I2C_HANDLE, LDC_I2C_ADDRESS << 1, OFFSET0_ADDR, 1, txData, 2, 100);

    txData[0] = (OFFSET1_VAL >> 8) & 0xFF; // High byte
    txData[1] = OFFSET1_VAL & 0xFF;        // Low byte
    status |= HAL_I2C_Mem_Write(LDC_I2C_HANDLE, LDC_I2C_ADDRESS << 1, OFFSET1_ADDR, 1, txData, 2, 100);

    txData[0] = (OFFSET2_VAL >> 8) & 0xFF; // High byte
    txData[1] = OFFSET2_VAL & 0xFF;        // Low byte
    status |= HAL_I2C_Mem_Write(LDC_I2C_HANDLE, LDC_I2C_ADDRESS << 1, OFFSET2_ADDR, 1, txData, 2, 100);

    txData[0] = (OFFSET3_VAL >> 8) & 0xFF; // High byte
    txData[1] = OFFSET3_VAL & 0xFF;        // Low byte
    status |= HAL_I2C_Mem_Write(LDC_I2C_HANDLE, LDC_I2C_ADDRESS << 1, OFFSET3_ADDR, 1, txData, 2, 100);

    txData[0] = (SETTLECOUNT0_VAL >> 8) & 0xFF; // High byte
    txData[1] = SETTLECOUNT0_VAL & 0xFF;        // Low byte
    status |= HAL_I2C_Mem_Write(LDC_I2C_HANDLE, LDC_I2C_ADDRESS << 1, SETTLECOUNT0_ADDR, 1, txData, 2, 100);

    txData[0] = (SETTLECOUNT1_VAL >> 8) & 0xFF; // High byte
    txData[1] = SETTLECOUNT1_VAL & 0xFF;        // Low byte
    status |= HAL_I2C_Mem_Write(LDC_I2C_HANDLE, LDC_I2C_ADDRESS << 1, SETTLECOUNT1_ADDR, 1, txData, 2, 100);

    txData[0] = (SETTLECOUNT2_VAL >> 8) & 0xFF; // High byte
    txData[1] = SETTLECOUNT2_VAL & 0xFF;        // Low byte
    status |= HAL_I2C_Mem_Write(LDC_I2C_HANDLE, LDC_I2C_ADDRESS << 1, SETTLECOUNT2_ADDR, 1, txData, 2, 100);

    txData[0] = (SETTLECOUNT3_VAL >> 8) & 0xFF; // High byte
    txData[1] = SETTLECOUNT3_VAL & 0xFF;        // Low byte
    status |= HAL_I2C_Mem_Write(LDC_I2C_HANDLE, LDC_I2C_ADDRESS << 1, SETTLECOUNT3_ADDR, 1, txData, 2, 100);

    txData[0] = (CLOCK_DIVIDERS0_VAL >> 8) & 0xFF; // High byte
    txData[1] = CLOCK_DIVIDERS0_VAL & 0xFF;        // Low byte
    status |= HAL_I2C_Mem_Write(LDC_I2C_HANDLE, LDC_I2C_ADDRESS << 1, CLOCK_DIVIDERS0_ADDR, 1, txData, 2, 100);

    txData[0] = (CLOCK_DIVIDERS1_VAL >> 8) & 0xFF; // High byte
    txData[1] = CLOCK_DIVIDERS1_VAL & 0xFF;        // Low byte
    status |= HAL_I2C_Mem_Write(LDC_I2C_HANDLE, LDC_I2C_ADDRESS << 1, CLOCK_DIVIDERS1_ADDR, 1, txData, 2, 100);

    txData[0] = (CLOCK_DIVIDERS2_VAL >> 8) & 0xFF; // High byte
    txData[1] = CLOCK_DIVIDERS2_VAL & 0xFF;        // Low byte
    status |= HAL_I2C_Mem_Write(LDC_I2C_HANDLE, LDC_I2C_ADDRESS << 1, CLOCK_DIVIDERS2_ADDR, 1, txData, 2, 100);

    txData[0] = (CLOCK_DIVIDERS3_VAL >> 8) & 0xFF; // High byte
    txData[1] = CLOCK_DIVIDERS3_VAL & 0xFF;        // Low byte
    status |= HAL_I2C_Mem_Write(LDC_I2C_HANDLE, LDC_I2C_ADDRESS << 1, CLOCK_DIVIDERS3_ADDR, 1, txData, 2, 100);

    txData[0] = (ERROR_CONFIG_VAL >> 8) & 0xFF; // High byte
    txData[1] = ERROR_CONFIG_VAL & 0xFF;        // Low byte
    status |= HAL_I2C_Mem_Write(LDC_I2C_HANDLE, LDC_I2C_ADDRESS << 1, ERROR_CONFIG_ADDR, 1, txData, 2, 100);

    txData[0] = (MUX_CONFIG_VAL >> 8) & 0xFF; // High byte
    txData[1] = MUX_CONFIG_VAL & 0xFF;        // Low byte
    status |= HAL_I2C_Mem_Write(LDC_I2C_HANDLE, LDC_I2C_ADDRESS << 1, MUX_CONFIG_ADDR, 1, txData, 2, 100);


    // Set the configuration register and start conversion
    txData[0] = (CONFIG_VAL >> 8) & 0xFF; // High byte
    txData[1] = CONFIG_VAL & 0xFF;        // Low byte
    status |= HAL_I2C_Mem_Write(LDC_I2C_HANDLE, LDC_I2C_ADDRESS << 1, CONFIG_ADDR, 1, txData, 2, 100);

    return status;
}

float LDC1614::readData(uint8_t channel) {
    uint8_t data_addr = 0x00;

    switch (channel)  // Check the channel number    
    {
    case 0:
        data_addr = DATA0_MSB_ADDR; // Channel 0
        break;
    case 1:
        data_addr = DATA1_MSB_ADDR; // Channel 1
        break;
    case 2:
        data_addr = DATA2_MSB_ADDR; // Channel 2
        break;
    case 3:
        data_addr = DATA3_MSB_ADDR; // Channel 3
        break;
    
    default:
        data_addr = DATA0_MSB_ADDR; // Default to channel 0
        break;
    }

    //read MSB
    uint8_t rxData[2];
    if(HAL_I2C_Mem_Read(LDC_I2C_HANDLE, LDC_I2C_ADDRESS << 1, data_addr, 1, rxData, 1, 100) != HAL_OK) return 0; // Error in reading

    //read LSB
    if(HAL_I2C_Mem_Read(LDC_I2C_HANDLE, LDC_I2C_ADDRESS << 1, data_addr + 1, 1, rxData + 1, 1, 100) != HAL_OK) return 0; // Error in reading

    // Check for errors
    if(rxData[0] & 0x80) {
        return LDC_UNDER_RANGE; // Under range error
    } else if (rxData[0] & 0x40) {
        return LDC_OVER_RANGE; // Over range error
    } else if (rxData[0] & 0x20) {
        return LDC_WD_TIMEOUT; // Watchdog timeout error
    } else if (rxData[0] & 0x10) {
        return LDC_AMPLITUDE_ERROR; // Amplitude error
    }

    // Combine the two bytes into a single 16-bit value
    uint16_t data = (((uint16_t)rxData[0] << 8) | rxData[1]) & DATAx_MSB_MASK;

    float frequency = (float)(data * LDC_FREF) / (1 << 28); // Calculate frequency

    return frequency; // Mask to get only the relevant bits
}

uint16_t LDC1614::readStatus() {
    uint16_t status = 0;
    if(HAL_I2C_Mem_Read(LDC_I2C_HANDLE, LDC_I2C_ADDRESS << 1, STATUS_ADDR, 1, (uint8_t *)&status, 2, 100) != HAL_OK) return 0; // Error in reading
    return status;
}