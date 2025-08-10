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
   struct
   {
      float Gyro_X;
      float Gyro_Y;
      float Gyro_Z;
      float Acc_X;
      float Acc_Y;
      float Acc_Z;
      float Quat_R;
      float Quat_I;
      float Quat_J;
      float Quat_K;
   } IMU_Data;
   int16_t ToF_Data[16];
   float Force_Estimate;

   /* Outputs */

   uint16_t Control_Word;
   uint8_t Operation_Mode;
   uint8_t Magnet_Command;

   /* Parameters */

   uint16_t Pulse_Time;
   uint8_t EPM_Number;
   float Force_Estimate_Params[2];
   uint16_t Error_Code;
} _Objects;

extern _Objects Obj;

#endif /* __UTYPES_H__ */
