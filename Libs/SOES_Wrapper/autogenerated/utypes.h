#ifndef __UTYPES_H__
#define __UTYPES_H__

#include "cc.h"

/* Object dictionary storage */

typedef struct
{
   /* Identity */

   uint32_t serial;

   /* Inputs */

   int16_t Temperatures[4];
   uint16_t Status_word;
   int8_t Operation_modes_display;
   int32_t Position_Demand;
   int32_t Position_Actual;
   int32_t Velocity_Demand;
   int32_t Velocity_Actual;
   int16_t Torque_Demand;
   int32_t Torque_Actual;
   int32_t Current_Actual;
   uint32_t DC_Link_Voltage;

   /* Outputs */

   struct
   {
      float Kp;
      float Ki;
      float Kd;
   } Controller_Gains;
   uint16_t Control_Word;
   int8_t Operation_Modes;
   int32_t Target_Torque;
   int32_t Target_Position;
   int32_t Target_Velocity;

   /* Parameters */

   uint16_t Error_Code;
   int16_t Quick_stop_option_code;
   uint16_t Max_Torque;
   uint16_t Max_Current;
   uint32_t Supported_drive_modes;

   /* Manufacturer specific data */

   uint16_t Device_Information[7];
   struct
   {
      uint32_t Phase_resistance;
      uint32_t D_Axis_inductance;
      uint32_t Q_Axis_inductance;
      uint8_t Pole_pairs;
      uint32_t Torque_constatnt;
      uint32_t Inertia;
      uint32_t Rated_Voltage;
      uint32_t Rated_Current;
      uint32_t Rated_speed;
   } Motor_Parameters;
   struct
   {
      float Iq;
      float Id;
      float Uq;
      float Ud;
   } Internal_Info;
   uint8_t Fan_Speed;
   uint8_t GPIO_Toggle[2];
   uint8_t GPIO_State[2];
   uint8_t Encodre_Offset_Calibrated;
   uint16_t Special_Command;
   uint32_t Timestamp;

   /* Dynamic Sync Managers */

   struct
   {
      uint8_t maxsub;
      uint32_t value[1];
   } SM1C12;
   struct
   {
      uint8_t maxsub;
      uint32_t value[3];
   } SM1C13;

} _Objects;

extern _Objects Obj;
#endif /* __UTYPES_H__ */
