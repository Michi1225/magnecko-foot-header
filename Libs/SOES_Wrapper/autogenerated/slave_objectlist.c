#include "esc_coe.h"
#include "utypes.h"
#include <stddef.h>

#ifndef HW_REV
#define HW_REV "1.0"
#endif

#ifndef SW_REV
#define SW_REV "1.0"
#endif

static const char acName1000[] = "Device Type";
static const char acName1008[] = "Manufacturer Device Name";
static const char acName1009[] = "Manufacturer Hardware Version";
static const char acName100A[] = "Manufacturer Software Version";
static const char acName1018[] = "Identity Object";
static const char acName1018_00[] = "Max SubIndex";
static const char acName1018_01[] = "Vendor ID";
static const char acName1018_02[] = "Product Code";
static const char acName1018_03[] = "Revision Number";
static const char acName1018_04[] = "Serial Number";
static const char acName1603[] = "Full";
static const char acName1603_00[] = "Max SubIndex";
static const char acName1603_01[] = "Control Word";
static const char acName1603_02[] = "Operation Modes";
static const char acName1603_03[] = "Padding 3";
static const char acName1603_04[] = "Target Position";
static const char acName1603_05[] = "Target Velocity";
static const char acName1603_06[] = "Target Torque";
static const char acName1603_07[] = "Kp";
static const char acName1603_08[] = "Ki";
static const char acName1603_09[] = "Kd";
static const char acName1603_0A[] = "GPIO0";
static const char acName1603_0B[] = "GPIO1";
static const char acName1A03[] = "Full";
static const char acName1A03_00[] = "Max SubIndex";
static const char acName1A03_01[] = "Status word";
static const char acName1A03_02[] = "Operation modes display";
static const char acName1A03_03[] = "Padding 3";
static const char acName1A03_04[] = "Position Actual";
static const char acName1A03_05[] = "Velocity Actual";
static const char acName1A03_06[] = "Torque Actual";
static const char acName1A03_07[] = "PCB Temperature";
static const char acName1A03_08[] = "Motor Temperature 1";
static const char acName1A03_09[] = "Motor Temperature 2";
static const char acName1A03_0A[] = "Motor Temperature 3";
static const char acName1A03_0B[] = "Total Current";
static const char acName1A03_0C[] = "GPIO0";
static const char acName1A03_0D[] = "GPIO1";
static const char acName1A04[] = "Debug";
static const char acName1A04_00[] = "Max SubIndex";
static const char acName1A04_01[] = "Status word";
static const char acName1A04_02[] = "Operation modes display";
static const char acName1A04_03[] = "Padding 3";
static const char acName1A04_04[] = "Position Actual";
static const char acName1A04_05[] = "Velocity Actual";
static const char acName1A04_06[] = "Torque Actual";
static const char acName1A04_07[] = "PCB Temperature";
static const char acName1A04_08[] = "Motor Temperature 1";
static const char acName1A04_09[] = "Motor Temperature 2";
static const char acName1A04_0A[] = "Motor Temperature 3";
static const char acName1A04_0B[] = "Total Current";
static const char acName1A04_0C[] = "GPIO0";
static const char acName1A04_0D[] = "GPIO1";
static const char acName1A04_0E[] = "DC Link Voltage";
static const char acName1A04_0F[] = "Iq";
static const char acName1A04_10[] = "Id";
static const char acName1A04_11[] = "Uq Demand";
static const char acName1A04_12[] = "Ud Demand";
static const char acName1A04_13[] = "Ia";
static const char acName1A04_14[] = "Ib";
static const char acName1A04_15[] = "Ic";
static const char acName1A04_16[] = "Position Demand";
static const char acName1A04_17[] = "Velocity Demand";
static const char acName1A04_18[] = "Torque Demand";
static const char acName1A04_19[] = "Timestamp";
static const char acName1C00[] = "Sync Manager Communication Type";
static const char acName1C00_00[] = "Max SubIndex";
static const char acName1C00_01[] = "Communications Type SM0";
static const char acName1C00_02[] = "Communications Type SM1";
static const char acName1C00_03[] = "Communications Type SM2";
static const char acName1C00_04[] = "Communications Type SM3";
static const char acName1C12[] = "Sync Manager 2 PDO Assignment";
static const char acName1C12_00[] = "Max SubIndex";
static const char acName1C12_01[] = "PDO Mapping";
static const char acName1C13[] = "Sync Manager 3 PDO Assignment";
static const char acName1C13_00[] = "Max SubIndex";
static const char acName1C13_01[] = "PDO Mapping";
static const char acName1C13_02[] = "SM Entry 1C13:02";
static const char acName1C13_03[] = "SM Entry 1C13:03";
static const char acName2000[] = "Device Information";
static const char acName2000_00[] = "Max SubIndex";
static const char acName2000_01[] = "Controller part number";
static const char acName2000_02[] = "Motor Part Number";
static const char acName2000_03[] = "Encoder Part Number";
static const char acName2000_04[] = "Software Version";
static const char acName2000_05[] = "Hardware Version";
static const char acName2000_06[] = "Runtime";
static const char acName2000_07[] = "Actuator Number";
static const char acName2001[] = "Motor Parameters";
static const char acName2001_00[] = "Max SubIndex";
static const char acName2001_01[] = "Phase resistance";
static const char acName2001_02[] = "D-Axis inductance";
static const char acName2001_03[] = "Q-Axis inductance";
static const char acName2001_04[] = "Pole pairs";
static const char acName2001_05[] = "Torque constatnt";
static const char acName2001_06[] = "Inertia";
static const char acName2001_07[] = "Rated Voltage";
static const char acName2001_08[] = "Rated Current";
static const char acName2001_09[] = "Rated speed";
static const char acName200E[] = "Internal Info";
static const char acName200E_00[] = "Max SubIndex";
static const char acName200E_01[] = "Iq";
static const char acName200E_02[] = "Id";
static const char acName200E_03[] = "Uq Demand";
static const char acName200E_04[] = "Ud Demand";
static const char acName200E_05[] = "Ia";
static const char acName200E_06[] = "Ib";
static const char acName200E_07[] = "Ic";
static const char acName200F[] = "Fan Speed";
static const char acName2010[] = "GPIO Toggle";
static const char acName2010_00[] = "Max SubIndex";
static const char acName2010_01[] = "GPIO0";
static const char acName2010_02[] = "GPIO1";
static const char acName2011[] = "GPIO State";
static const char acName2011_00[] = "Max SubIndex";
static const char acName2011_01[] = "GPIO0";
static const char acName2011_02[] = "GPIO1";
static const char acName2012[] = "Encoder Offset Calibrated";
static const char acName2013[] = "Zero Position Offset";
static const char acName2014[] = "Timestamp";
static const char acName2015[] = "Cutoff Frequency Impeadance";
static const char acName2016[] = "Quick Stop PID";
static const char acName2016_00[] = "Max SubIndex";
static const char acName2016_01[] = "Kp";
static const char acName2016_02[] = "Ki";
static const char acName2016_03[] = "Kd";
static const char acName2017[] = "Tunes";
static const char acName2018[] = "Torque Rate Limit";
static const char acName2040[] = "Temperatures";
static const char acName2040_00[] = "Max SubIndex";
static const char acName2040_01[] = "PCB Temperature";
static const char acName2040_02[] = "Motor Temperature 1";
static const char acName2040_03[] = "Motor Temperature 2";
static const char acName2040_04[] = "Motor Temperature 3";
static const char acName2080[] = "Controller Gains";
static const char acName2080_00[] = "Max SubIndex";
static const char acName2080_01[] = "Kp";
static const char acName2080_02[] = "Ki";
static const char acName2080_03[] = "Kd";
static const char acName603F[] = "Error Code";
static const char acName6040[] = "Control Word";
static const char acName6041[] = "Status word";
static const char acName605A[] = "Quick stop option code";
static const char acName6060[] = "Operation Modes";
static const char acName6061[] = "Operation modes display";
static const char acName6062[] = "Position Demand";
static const char acName6064[] = "Position Actual";
static const char acName606B[] = "Velocity Demand";
static const char acName606C[] = "Velocity Actual";
static const char acName6071[] = "Target Torque";
static const char acName6072[] = "Max Torque";
static const char acName6073[] = "Max Current";
static const char acName6074[] = "Torque Demand";
static const char acName6077[] = "Torque Actual";
static const char acName6078[] = "Total Current";
static const char acName6079[] = "DC Link Voltage";
static const char acName607A[] = "Target Position";
static const char acName60FF[] = "Target Velocity";
static const char acName6502[] = "Supported drive modes";

const _objd SDO1000[] =
{
  {0x0, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1000, 0x00001389, NULL},
};
const _objd SDO1008[] =
{
  {0x0, DTYPE_VISIBLE_STRING, 64, ATYPE_RO, acName1008, 0, "actuator"},
};
const _objd SDO1009[] =
{
  {0x0, DTYPE_VISIBLE_STRING, 24, ATYPE_RO, acName1009, 0, HW_REV},
};
const _objd SDO100A[] =
{
  {0x0, DTYPE_VISIBLE_STRING, 24, ATYPE_RO, acName100A, 0, SW_REV},
};
const _objd SDO1018[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1018_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_01, 0, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_02, 0, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_03, 0, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_04, 0x00000000, &Obj.serial},
};
const _objd SDO1603[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1603_00, 11, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1603_01, 0x60400010, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1603_02, 0x60600008, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1603_03, 0x00000008, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1603_04, 0x607A0020, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1603_05, 0x60FF0020, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1603_06, 0x60710020, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1603_07, 0x20800120, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1603_08, 0x20800220, NULL},
  {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1603_09, 0x20800320, NULL},
  {0x0A, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1603_0A, 0x20100108, NULL},
  {0x0B, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1603_0B, 0x20100208, NULL},
};
const _objd SDO1A03[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A03_00, 13, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_01, 0x60410010, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_02, 0x60610008, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_03, 0x00000008, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_04, 0x60640020, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_05, 0x606C0020, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_06, 0x60770020, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_07, 0x20400110, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_08, 0x20400210, NULL},
  {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_09, 0x20400310, NULL},
  {0x0A, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_0A, 0x20400410, NULL},
  {0x0B, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_0B, 0x60780020, NULL},
  {0x0C, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_0C, 0x20110108, NULL},
  {0x0D, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_0D, 0x20110208, NULL},
};
const _objd SDO1A04[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A04_00, 25, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_01, 0x60410010, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_02, 0x60610008, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_03, 0x00000008, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_04, 0x60640020, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_05, 0x606C0020, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_06, 0x60770020, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_07, 0x20400110, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_08, 0x20400210, NULL},
  {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_09, 0x20400310, NULL},
  {0x0A, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_0A, 0x20400410, NULL},
  {0x0B, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_0B, 0x60780020, NULL},
  {0x0C, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_0C, 0x20110108, NULL},
  {0x0D, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_0D, 0x20110208, NULL},
  {0x0E, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_0E, 0x60790020, NULL},
  {0x0F, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_0F, 0x200E0120, NULL},
  {0x10, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_10, 0x200E0220, NULL},
  {0x11, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_11, 0x200E0320, NULL},
  {0x12, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_12, 0x200E0420, NULL},
  {0x13, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_13, 0x200E0520, NULL},
  {0x14, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_14, 0x200E0620, NULL},
  {0x15, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_15, 0x200E0720, NULL},
  {0x16, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_16, 0x60620020, NULL},
  {0x17, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_17, 0x606B0020, NULL},
  {0x18, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_18, 0x60740020, NULL},
  {0x19, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_19, 0x20140020, NULL},
};
const _objd SDO1C00[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_01, 1, NULL},
  {0x02, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_02, 2, NULL},
  {0x03, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_03, 3, NULL},
  {0x04, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_04, 4, NULL},
};
const _objd SDO1C12[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RWpre, acName1C12_00, 1, &Obj.SM1C12.maxsub},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acName1C12_01, 0x1603, &Obj.SM1C12.value[0]},
};
const _objd SDO1C13[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RWpre, acName1C13_00, 3, &Obj.SM1C13.maxsub},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acName1C13_01, 0x1A03, &Obj.SM1C13.value[0]},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acName1C13_02, 0x0000, &Obj.SM1C13.value[1]},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RWpre, acName1C13_03, 0x0000, &Obj.SM1C13.value[2]},
};
const _objd SDO2000[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName2000_00, 7, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName2000_01, 0, &Obj.Device_Information[0]},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName2000_02, 0, &Obj.Device_Information[1]},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName2000_03, 0, &Obj.Device_Information[2]},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName2000_04, 0, &Obj.Device_Information[3]},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName2000_05, 0, &Obj.Device_Information[4]},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName2000_06, 0, &Obj.Device_Information[5]},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName2000_07, 0, &Obj.Device_Information[6]},
};
const _objd SDO2001[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName2001_00, 9, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RW, acName2001_01, 0, &Obj.Motor_Parameters.Phase_resistance},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RW, acName2001_02, 0, &Obj.Motor_Parameters.D_Axis_inductance},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RW, acName2001_03, 0, &Obj.Motor_Parameters.Q_Axis_inductance},
  {0x04, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName2001_04, 0, &Obj.Motor_Parameters.Pole_pairs},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RW, acName2001_05, 0, &Obj.Motor_Parameters.Torque_constatnt},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RW, acName2001_06, 0, &Obj.Motor_Parameters.Inertia},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RW, acName2001_07, 0, &Obj.Motor_Parameters.Rated_Voltage},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RW, acName2001_08, 0, &Obj.Motor_Parameters.Rated_Current},
  {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RW, acName2001_09, 0, &Obj.Motor_Parameters.Rated_speed},
};
const _objd SDO200E[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName200E_00, 7, NULL},
  {0x01, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName200E_01, 0x00000000, &Obj.Internal_Info.Iq},
  {0x02, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName200E_02, 0x00000000, &Obj.Internal_Info.Id},
  {0x03, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName200E_03, 0x00000000, &Obj.Internal_Info.Uq_Demand},
  {0x04, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName200E_04, 0x00000000, &Obj.Internal_Info.Ud_Demand},
  {0x05, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName200E_05, 0x00000000, &Obj.Internal_Info.Ia},
  {0x06, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName200E_06, 0x00000000, &Obj.Internal_Info.Ib},
  {0x07, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName200E_07, 0x00000000, &Obj.Internal_Info.Ic},
};
const _objd SDO200F[] =
{
  {0x0, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_RXPDO, acName200F, 0, &Obj.Fan_Speed},
};
const _objd SDO2010[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName2010_00, 2, NULL},
  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_RXPDO, acName2010_01, 0, &Obj.GPIO_Toggle[0]},
  {0x02, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_RXPDO, acName2010_02, 0, &Obj.GPIO_Toggle[1]},
};
const _objd SDO2011[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName2011_00, 2, NULL},
  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_TXPDO, acName2011_01, 0, &Obj.GPIO_State[0]},
  {0x02, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_TXPDO, acName2011_02, 0, &Obj.GPIO_State[1]},
};
const _objd SDO2012[] =
{
  {0x0, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName2012, 0, &Obj.Encoder_Offset_Calibrated},
};
const _objd SDO2013[] =
{
  {0x0, DTYPE_REAL32, 32, ATYPE_RW, acName2013, 0x00000000, &Obj.Zero_Position_Offset},
};
const _objd SDO2014[] =
{
  {0x0, DTYPE_UNSIGNED32, 32, ATYPE_RW | ATYPE_TXPDO, acName2014, 0, &Obj.Timestamp},
};
const _objd SDO2015[] =
{
  {0x0, DTYPE_REAL32, 32, ATYPE_RW, acName2015, 0x00000000, &Obj.Cutoff_Frequency_Impeadance},
};
const _objd SDO2016[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName2016_00, 3, NULL},
  {0x01, DTYPE_REAL32, 32, ATYPE_RW, acName2016_01, 0x00000000, &Obj.Quick_Stop_PID[0]},
  {0x02, DTYPE_REAL32, 32, ATYPE_RW, acName2016_02, 0x00000000, &Obj.Quick_Stop_PID[1]},
  {0x03, DTYPE_REAL32, 32, ATYPE_RW, acName2016_03, 0x00000000, &Obj.Quick_Stop_PID[2]},
};
const _objd SDO2017[] =
{
  {0x0, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName2017, 0, &Obj.Tunes},
};

const _objd SDO2018[] =
{
  {0x0, DTYPE_REAL32, 32, ATYPE_RWpre, acName2018, 0, &Obj.Torque_Rate_Limit},
};

const _objd SDO2040[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName2040_00, 4, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName2040_01, 0, &Obj.Temperatures[0]},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName2040_02, 0, &Obj.Temperatures[1]},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName2040_03, 0, &Obj.Temperatures[2]},
  {0x04, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName2040_04, 0, &Obj.Temperatures[3]},
};
const _objd SDO2080[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName2080_00, 3, NULL},
  {0x01, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_RXPDO, acName2080_01, 0x00000000, &Obj.Controller_Gains.Kp},
  {0x02, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_RXPDO, acName2080_02, 0x00000000, &Obj.Controller_Gains.Ki},
  {0x03, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_RXPDO, acName2080_03, 0x00000000, &Obj.Controller_Gains.Kd},
};
const _objd SDO603F[] =
{
  {0x0, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName603F, 0, &Obj.Error_Code},
};
const _objd SDO6040[] =
{
  {0x0, DTYPE_UNSIGNED16, 16, ATYPE_RW | ATYPE_RXPDO, acName6040, 0, &Obj.Control_Word},
};
const _objd SDO6041[] =
{
  {0x0, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_TXPDO, acName6041, 0, &Obj.Status_word},
};
const _objd SDO605A[] =
{
  {0x0, DTYPE_INTEGER16, 16, ATYPE_RW, acName605A, 0, &Obj.Quick_stop_option_code},
};
const _objd SDO6060[] =
{
  {0x0, DTYPE_INTEGER8, 8, ATYPE_RW | ATYPE_RXPDO, acName6060, 0, &Obj.Operation_Modes},
};
const _objd SDO6061[] =
{
  {0x0, DTYPE_INTEGER8, 8, ATYPE_RO | ATYPE_TXPDO, acName6061, 0, &Obj.Operation_modes_display},
};
const _objd SDO6062[] =
{
  {0x0, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6062, 0x00000000, &Obj.Position_Demand},
};
const _objd SDO6064[] =
{
  {0x0, DTYPE_INTEGER32, 32, ATYPE_RO | ATYPE_TXPDO, acName6064, 0, &Obj.Position_Actual},
};
const _objd SDO606B[] =
{
  {0x0, DTYPE_REAL32, 32, ATYPE_RW | ATYPE_TXPDO, acName606B, 0x00000000, &Obj.Velocity_Demand},
};
const _objd SDO606C[] =
{
  {0x0, DTYPE_INTEGER32, 32, ATYPE_RO | ATYPE_TXPDO, acName606C, 0, &Obj.Velocity_Actual},
};
const _objd SDO6071[] =
{
  {0x0, DTYPE_INTEGER32, 32, ATYPE_RO | ATYPE_RXPDO, acName6071, 0, &Obj.Target_Torque},
};
const _objd SDO6072[] =
{
  {0x0, DTYPE_REAL32, 32, ATYPE_RWpre, acName6072, 0, &Obj.Max_Torque},
};
const _objd SDO6073[] =
{
  {0x0, DTYPE_REAL32, 32, ATYPE_RWpre, acName6073, 0, &Obj.Max_Current},
};
const _objd SDO6074[] =
{
  {0x0, DTYPE_REAL32, 32, ATYPE_RW | ATYPE_TXPDO, acName6074, 0x00000000, &Obj.Torque_Demand},
};
const _objd SDO6077[] =
{
  {0x0, DTYPE_INTEGER32, 32, ATYPE_RO | ATYPE_TXPDO, acName6077, 0, &Obj.Torque_Actual},
};
const _objd SDO6078[] =
{
  {0x0, DTYPE_INTEGER32, 32, ATYPE_RO | ATYPE_TXPDO, acName6078, 0, &Obj.Total_Current},
};
const _objd SDO6079[] =
{
  {0x0, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6079, 0x00000000, &Obj.DC_Link_Voltage},
};
const _objd SDO607A[] =
{
  {0x0, DTYPE_INTEGER32, 32, ATYPE_RW | ATYPE_RXPDO, acName607A, 0, &Obj.Target_Position},
};
const _objd SDO60FF[] =
{
  {0x0, DTYPE_INTEGER32, 32, ATYPE_RW | ATYPE_RXPDO, acName60FF, 0, &Obj.Target_Velocity},
};
const _objd SDO6502[] =
{
  {0x0, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName6502, 0, &Obj.Supported_drive_modes},
};

const _objectlist SDOobjects[] =
{
  {0x1000, OTYPE_VAR, 0, 0, acName1000, SDO1000},
  {0x1008, OTYPE_VAR, 0, 0, acName1008, SDO1008},
  {0x1009, OTYPE_VAR, 0, 0, acName1009, SDO1009},
  {0x100A, OTYPE_VAR, 0, 0, acName100A, SDO100A},
  {0x1018, OTYPE_RECORD, 4, 0, acName1018, SDO1018},
  {0x1603, OTYPE_RECORD, 11, 0, acName1603, SDO1603},
  {0x1A03, OTYPE_RECORD, 13, 0, acName1A03, SDO1A03},
  {0x1A04, OTYPE_RECORD, 25, 0, acName1A04, SDO1A04},
  {0x1C00, OTYPE_ARRAY, 4, 0, acName1C00, SDO1C00},
  {0x1C12, OTYPE_ARRAY, 1, 0, acName1C12, SDO1C12},
  {0x1C13, OTYPE_ARRAY, 3, 0, acName1C13, SDO1C13},
  {0x2000, OTYPE_ARRAY, 7, 0, acName2000, SDO2000},
  {0x2001, OTYPE_RECORD, 9, 0, acName2001, SDO2001},
  {0x200E, OTYPE_RECORD, 7, 0, acName200E, SDO200E},
  {0x200F, OTYPE_VAR, 0, 0, acName200F, SDO200F},
  {0x2010, OTYPE_ARRAY, 2, 0, acName2010, SDO2010},
  {0x2011, OTYPE_ARRAY, 2, 0, acName2011, SDO2011},
  {0x2012, OTYPE_VAR, 0, 0, acName2012, SDO2012},
  {0x2013, OTYPE_VAR, 0, 0, acName2013, SDO2013},
  {0x2014, OTYPE_VAR, 0, 0, acName2014, SDO2014},
  {0x2015, OTYPE_VAR, 0, 0, acName2015, SDO2015},
  {0x2016, OTYPE_ARRAY, 3, 0, acName2016, SDO2016},
  {0x2017, OTYPE_VAR, 0, 0, acName2017, SDO2017},
  {0x2018, OTYPE_VAR, 0, 0, acName2018, SDO2018},
  {0x2040, OTYPE_ARRAY, 4, 0, acName2040, SDO2040},
  {0x2080, OTYPE_RECORD, 3, 0, acName2080, SDO2080},
  {0x603F, OTYPE_VAR, 0, 0, acName603F, SDO603F},
  {0x6040, OTYPE_VAR, 0, 0, acName6040, SDO6040},
  {0x6041, OTYPE_VAR, 0, 0, acName6041, SDO6041},
  {0x605A, OTYPE_VAR, 0, 0, acName605A, SDO605A},
  {0x6060, OTYPE_VAR, 0, 0, acName6060, SDO6060},
  {0x6061, OTYPE_VAR, 0, 0, acName6061, SDO6061},
  {0x6062, OTYPE_VAR, 0, 0, acName6062, SDO6062},
  {0x6064, OTYPE_VAR, 0, 0, acName6064, SDO6064},
  {0x606B, OTYPE_VAR, 0, 0, acName606B, SDO606B},
  {0x606C, OTYPE_VAR, 0, 0, acName606C, SDO606C},
  {0x6071, OTYPE_VAR, 0, 0, acName6071, SDO6071},
  {0x6072, OTYPE_VAR, 0, 0, acName6072, SDO6072},
  {0x6073, OTYPE_VAR, 0, 0, acName6073, SDO6073},
  {0x6074, OTYPE_VAR, 0, 0, acName6074, SDO6074},
  {0x6077, OTYPE_VAR, 0, 0, acName6077, SDO6077},
  {0x6078, OTYPE_VAR, 0, 0, acName6078, SDO6078},
  {0x6079, OTYPE_VAR, 0, 0, acName6079, SDO6079},
  {0x607A, OTYPE_VAR, 0, 0, acName607A, SDO607A},
  {0x60FF, OTYPE_VAR, 0, 0, acName60FF, SDO60FF},
  {0x6502, OTYPE_VAR, 0, 0, acName6502, SDO6502},
  {0xffff, 0xff, 0xff, 0xff, NULL, NULL}
};
