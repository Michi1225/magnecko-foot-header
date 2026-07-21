#ifndef __UTYPES_H__
#define __UTYPES_H__

#include "cc.h"

/* Object dictionary storage */

typedef struct
{
   /* Identity */

   uint32_t serial;

   /* Inputs */
   uint16_t Status_Word;
   uint8_t Operation_Mode_Display;
   
   uint8_t Magnet_Status;
   float Force_Estimate;


   int16_t Temperature;

   struct
   {
      int16_t Gyro_X;
      int16_t Gyro_Y;
      int16_t Gyro_Z;
      int16_t Acc_X;
      int16_t Acc_Y;
      int16_t Acc_Z;
      int16_t Quat_R;
      int16_t Quat_I;
      int16_t Quat_J;
      int16_t Quat_K;
   } IMU_Data;

   int16_t ToF_Distance[64];
   uint8_t ToF_SNR[64];

   uint16_t LDC_Frequency[4];
   uint16_t LDC_RP[4];

   uint16_t HALL_Mag_X[4];
   uint16_t HALL_Mag_Y[4];
   uint16_t HALL_Mag_Z[4];

   uint16_t Capacitor_Voltage;

   /* Outputs */

   uint16_t Control_Word;
   uint8_t Operation_Mode;
   uint8_t Magnet_Command;

   /* Parameters */

   uint16_t Pulse_Time;
   uint8_t EPM_Number;
   float Force_Estimate_Params[250];
   uint16_t Error_Code;
} _Objects;

extern _Objects Obj;

#endif /* __UTYPES_H__ */
