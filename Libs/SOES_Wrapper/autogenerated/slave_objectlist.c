#include "esc_coe.h"
#include "utypes.h"
#include <stddef.h>


static const char acName1000[] = "Device Type";
static const char acName1008[] = "Device Name";
static const char acName1009[] = "Hardware Version";
static const char acName100A[] = "Software Version";
static const char acName1018[] = "Identity Object";
static const char acName1018_00[] = "Max SubIndex";
static const char acName1018_01[] = "Vendor ID";
static const char acName1018_02[] = "Product Code";
static const char acName1018_03[] = "Revision Number";
static const char acName1018_04[] = "Serial Number";
static const char acName1600[] = "Control Word";
static const char acName1600_00[] = "Max SubIndex";
static const char acName1600_01[] = "Control Word";
static const char acName1601[] = "Operation Mode";
static const char acName1601_00[] = "Max SubIndex";
static const char acName1601_01[] = "Operation Mode";
static const char acName1602[] = "Magnet Command";
static const char acName1602_00[] = "Max SubIndex";
static const char acName1602_01[] = "Magnet Command";
static const char acName1A00[] = "Status Word";
static const char acName1A00_00[] = "Max SubIndex";
static const char acName1A00_01[] = "Status Word";
static const char acName1A01[] = "Operation Mode Display";
static const char acName1A01_00[] = "Max SubIndex";
static const char acName1A01_01[] = "Operation Mode Display";
static const char acName1A02[] = "Magnet Status";
static const char acName1A02_00[] = "Max SubIndex";
static const char acName1A02_01[] = "Magnet Status";
static const char acName1A03[] = "IMU Data";
static const char acName1A03_00[] = "Max SubIndex";
static const char acName1A03_01[] = "Gyro X";
static const char acName1A03_02[] = "Gyro Y";
static const char acName1A03_03[] = "Gyro Z";
static const char acName1A03_04[] = "Acc X";
static const char acName1A03_05[] = "Acc Y";
static const char acName1A03_06[] = "Acc Z";
static const char acName1A03_07[] = "Quat R";
static const char acName1A03_08[] = "Quat I";
static const char acName1A03_09[] = "Quat J";
static const char acName1A03_0A[] = "Quat K";
static const char acName1A04[] = "ToF Data";
static const char acName1A04_00[] = "Max SubIndex";
static const char acName1A04_01[] = "0 0";
static const char acName1A04_02[] = "0 1";
static const char acName1A04_03[] = "0 2";
static const char acName1A04_04[] = "0 3";
static const char acName1A04_05[] = "1 0";
static const char acName1A04_06[] = "1 1";
static const char acName1A04_07[] = "1 2";
static const char acName1A04_08[] = "1 3";
static const char acName1A04_09[] = "2 0";
static const char acName1A04_0A[] = "2 1";
static const char acName1A04_0B[] = "2 2";
static const char acName1A04_0C[] = "2 3";
static const char acName1A04_0D[] = "3 0";
static const char acName1A04_0E[] = "3 1";
static const char acName1A04_0F[] = "3 2";
static const char acName1A04_10[] = "3 3";
static const char acName1A05[] = "Force Estimate";
static const char acName1A05_00[] = "Max SubIndex";
static const char acName1A05_01[] = "Force Estimate";
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
static const char acName2000[] = "Pulse Time";
static const char acName2001[] = "EPM Number";
static const char acName2002[] = "Force Estimate Params";
static const char acName2002_00[] = "Max SubIndex";
static const char acName2002_01[] = "Offset";
static const char acName2002_02[] = "Slope";
static const char acName603F[] = "Error Code";
static const char acName6040[] = "Control Word";
static const char acName6041[] = "Status Word";
static const char acName6060[] = "Operation Mode";
static const char acName6061[] = "Operation Mode Display";
static const char acName6080[] = "Magnet Command";
static const char acName6081[] = "Magnet Status";
static const char acName6090[] = "IMU Data";
static const char acName6090_00[] = "Max SubIndex";
static const char acName6090_01[] = "Gyro X";
static const char acName6090_02[] = "Gyro Y";
static const char acName6090_03[] = "Gyro Z";
static const char acName6090_04[] = "Acc X";
static const char acName6090_05[] = "Acc Y";
static const char acName6090_06[] = "Acc Z";
static const char acName6090_07[] = "Quat R";
static const char acName6090_08[] = "Quat I";
static const char acName6090_09[] = "Quat J";
static const char acName6090_0A[] = "Quat K";
static const char acName6091[] = "ToF Data";
static const char acName6091_00[] = "Max SubIndex";
static const char acName6091_01[] = "0 0";
static const char acName6091_02[] = "0 1";
static const char acName6091_03[] = "0 2";
static const char acName6091_04[] = "0 3";
static const char acName6091_05[] = "1 0";
static const char acName6091_06[] = "1 1";
static const char acName6091_07[] = "1 2";
static const char acName6091_08[] = "1 3";
static const char acName6091_09[] = "2 0";
static const char acName6091_0A[] = "2 1";
static const char acName6091_0B[] = "2 2";
static const char acName6091_0C[] = "2 3";
static const char acName6091_0D[] = "3 0";
static const char acName6091_0E[] = "3 1";
static const char acName6091_0F[] = "3 2";
static const char acName6091_10[] = "3 3";
static const char acName6092[] = "Force Estimate";

const _objd SDO1000[] =
{
  {0x0, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1000, 5001, NULL},
};
const _objd SDO1008[] =
{
  {0x0, DTYPE_VISIBLE_STRING, 376, ATYPE_RO, acName1008, 0, "2-channel Hypergalactic input superimpermanator"},
};
const _objd SDO1009[] =
{
  {0x0, DTYPE_VISIBLE_STRING, 40, ATYPE_RO, acName1009, 0, "1.0.0"},
};
const _objd SDO100A[] =
{
  {0x0, DTYPE_VISIBLE_STRING, 40, ATYPE_RO, acName100A, 0, "1.0.0"},
};
const _objd SDO1018[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1018_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_01, 69, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_02, 0, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_03, 0, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_04, 0, &Obj.serial},
};
const _objd SDO1600[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1600_00, 3, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_01, 0x60400010, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1601_01, 0x60600008, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1602_01, 0x60800008, NULL},
};
const _objd SDO1601[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1601_00, 1, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1601_01, 0x60600008, NULL},
};
const _objd SDO1602[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1602_00, 1, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1602_01, 0x60800008, NULL},
};
const _objd SDO1A00[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A00_00, 30, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_01, 0x60410010, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_01, 0x60610008, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_01, 0x60810008, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_01, 0x60900120, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_02, 0x60900220, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_03, 0x60900320, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_04, 0x60900420, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_05, 0x60900520, NULL},
  {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_06, 0x60900620, NULL},
  {0x0A, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_07, 0x60900720, NULL},
  {0x0B, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_08, 0x60900820, NULL},
  {0x0C, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_09, 0x60900920, NULL},
  {0x0D, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_0A, 0x60900a20, NULL},
  {0x0E, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_01, 0x60910110, NULL},
  {0x0F, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_02, 0x60910210, NULL},
  {0x10, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_03, 0x60910310, NULL},
  {0x11, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_04, 0x60910410, NULL},
  {0x12, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_05, 0x60910510, NULL},
  {0x13, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_06, 0x60910610, NULL},
  {0x14, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_07, 0x60910710, NULL},
  {0x15, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_08, 0x60910810, NULL},
  {0x16, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_09, 0x60910910, NULL},
  {0x17, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_0A, 0x60910a10, NULL},
  {0x18, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_0B, 0x60910b10, NULL},
  {0x19, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_0C, 0x60910c10, NULL},
  {0x1A, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_0D, 0x60910d10, NULL},
  {0x1B, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_0E, 0x60910e10, NULL},
  {0x1C, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_0F, 0x60910f10, NULL},
  {0x1D, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_10, 0x60911010, NULL},
  {0x1E, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_01, 0x60920020, NULL},
};
const _objd SDO1A01[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A01_00, 1, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_01, 0x60610008, NULL},
};
const _objd SDO1A02[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A02_00, 1, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_01, 0x60810008, NULL},
};
const _objd SDO1A03[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A03_00, 10, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_01, 0x60900120, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_02, 0x60900220, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_03, 0x60900320, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_04, 0x60900420, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_05, 0x60900520, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_06, 0x60900620, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_07, 0x60900720, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_08, 0x60900820, NULL},
  {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_09, 0x60900920, NULL},
  {0x0A, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_0A, 0x60900a20, NULL},
};
const _objd SDO1A04[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A04_00, 16, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_01, 0x60910110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_02, 0x60910210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_03, 0x60910310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_04, 0x60910410, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_05, 0x60910510, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_06, 0x60910610, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_07, 0x60910710, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_08, 0x60910810, NULL},
  {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_09, 0x60910910, NULL},
  {0x0A, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_0A, 0x60910a10, NULL},
  {0x0B, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_0B, 0x60910b10, NULL},
  {0x0C, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_0C, 0x60910c10, NULL},
  {0x0D, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_0D, 0x60910d10, NULL},
  {0x0E, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_0E, 0x60910e10, NULL},
  {0x0F, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_0F, 0x60910f10, NULL},
  {0x10, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_10, 0x60911010, NULL},
};
const _objd SDO1A05[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A05_00, 1, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_01, 0x60920020, NULL},
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
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C12_00, 1, NULL},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_01, 0x1600, NULL},
};
const _objd SDO1C13[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C13_00, 1, NULL},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_01, 0x1A00, NULL},
};
const _objd SDO2000[] =
{
  {0x0, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName2000, 0, &Obj.Pulse_Time},
};
const _objd SDO2001[] =
{
  {0x0, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName2001, 0, &Obj.EPM_Number},
};
const _objd SDO2002[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName2002_00, 2, NULL},
  {0x01, DTYPE_REAL32, 32, ATYPE_RO, acName2002_01, 0x00000000, &Obj.Force_Estimate_Params[0]},
  {0x02, DTYPE_REAL32, 32, ATYPE_RO, acName2002_02, 0x00000000, &Obj.Force_Estimate_Params[1]},
};
const _objd SDO603F[] =
{
  {0x0, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName603F, 0, &Obj.Error_Code},
};
const _objd SDO6040[] =
{
  {0x0, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_RXPDO, acName6040, 0, &Obj.Control_Word},
};
const _objd SDO6041[] =
{
  {0x0, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_TXPDO, acName6041, 0, &Obj.Status_Word},
};
const _objd SDO6060[] =
{
  {0x0, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_RXPDO, acName6060, 0, &Obj.Operation_Mode},
};
const _objd SDO6061[] =
{
  {0x0, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName6061, 0, &Obj.Operation_Mode_Display},
};
const _objd SDO6080[] =
{
  {0x0, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_RXPDO, acName6080, 0, &Obj.Magnet_Command},
};
const _objd SDO6081[] =
{
  {0x0, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName6081, 0, &Obj.Magnet_Status},
};
const _objd SDO6090[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6090_00, 10, NULL},
  {0x01, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6090_01, 0x00000000, &Obj.IMU_Data.Gyro_X},
  {0x02, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6090_02, 0x00000000, &Obj.IMU_Data.Gyro_Y},
  {0x03, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6090_03, 0x00000000, &Obj.IMU_Data.Gyro_Z},
  {0x04, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6090_04, 0x00000000, &Obj.IMU_Data.Acc_X},
  {0x05, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6090_05, 0x00000000, &Obj.IMU_Data.Acc_Y},
  {0x06, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6090_06, 0x00000000, &Obj.IMU_Data.Acc_Z},
  {0x07, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6090_07, 0x00000000, &Obj.IMU_Data.Quat_R},
  {0x08, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6090_08, 0x00000000, &Obj.IMU_Data.Quat_I},
  {0x09, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6090_09, 0x00000000, &Obj.IMU_Data.Quat_J},
  {0x0A, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6090_0A, 0x00000000, &Obj.IMU_Data.Quat_K},
};
const _objd SDO6091[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6091_00, 16, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6091_01, 0, &Obj.ToF_Data[0]},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6091_02, 0, &Obj.ToF_Data[1]},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6091_03, 0, &Obj.ToF_Data[2]},
  {0x04, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6091_04, 0, &Obj.ToF_Data[3]},
  {0x05, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6091_05, 0, &Obj.ToF_Data[4]},
  {0x06, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6091_06, 0, &Obj.ToF_Data[5]},
  {0x07, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6091_07, 0, &Obj.ToF_Data[6]},
  {0x08, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6091_08, 0, &Obj.ToF_Data[7]},
  {0x09, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6091_09, 0, &Obj.ToF_Data[8]},
  {0x0A, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6091_0A, 0, &Obj.ToF_Data[9]},
  {0x0B, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6091_0B, 0, &Obj.ToF_Data[10]},
  {0x0C, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6091_0C, 0, &Obj.ToF_Data[11]},
  {0x0D, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6091_0D, 0, &Obj.ToF_Data[12]},
  {0x0E, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6091_0E, 0, &Obj.ToF_Data[13]},
  {0x0F, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6091_0F, 0, &Obj.ToF_Data[14]},
  {0x10, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6091_10, 0, &Obj.ToF_Data[15]},
};
const _objd SDO6092[] =
{
  {0x0, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6092, 0x00000000, &Obj.Force_Estimate},
};

const _objectlist SDOobjects[] =
{
  {0x1000, OTYPE_VAR, 0, 0, acName1000, SDO1000},
  {0x1008, OTYPE_VAR, 0, 0, acName1008, SDO1008},
  {0x1009, OTYPE_VAR, 0, 0, acName1009, SDO1009},
  {0x100A, OTYPE_VAR, 0, 0, acName100A, SDO100A},
  {0x1018, OTYPE_RECORD, 4, 0, acName1018, SDO1018},
  {0x1600, OTYPE_RECORD, 3, 0, acName1600, SDO1600},
  {0x1601, OTYPE_RECORD, 1, 0, acName1601, SDO1601},
  {0x1602, OTYPE_RECORD, 1, 0, acName1602, SDO1602},
  {0x1A00, OTYPE_RECORD, 30, 0, acName1A00, SDO1A00},
  {0x1A01, OTYPE_RECORD, 1, 0, acName1A01, SDO1A01},
  {0x1A02, OTYPE_RECORD, 1, 0, acName1A02, SDO1A02},
  {0x1A03, OTYPE_RECORD, 10, 0, acName1A03, SDO1A03},
  {0x1A04, OTYPE_RECORD, 16, 0, acName1A04, SDO1A04},
  {0x1A05, OTYPE_RECORD, 1, 0, acName1A05, SDO1A05},
  {0x1C00, OTYPE_ARRAY, 4, 0, acName1C00, SDO1C00},
  {0x1C12, OTYPE_ARRAY, 1, 0, acName1C12, SDO1C12},
  {0x1C13, OTYPE_ARRAY, 1, 0, acName1C13, SDO1C13},
  {0x2000, OTYPE_VAR, 0, 0, acName2000, SDO2000},
  {0x2001, OTYPE_VAR, 0, 0, acName2001, SDO2001},
  {0x2002, OTYPE_ARRAY, 2, 0, acName2002, SDO2002},
  {0x603F, OTYPE_VAR, 0, 0, acName603F, SDO603F},
  {0x6040, OTYPE_VAR, 0, 0, acName6040, SDO6040},
  {0x6041, OTYPE_VAR, 0, 0, acName6041, SDO6041},
  {0x6060, OTYPE_VAR, 0, 0, acName6060, SDO6060},
  {0x6061, OTYPE_VAR, 0, 0, acName6061, SDO6061},
  {0x6080, OTYPE_VAR, 0, 0, acName6080, SDO6080},
  {0x6081, OTYPE_VAR, 0, 0, acName6081, SDO6081},
  {0x6090, OTYPE_RECORD, 10, 0, acName6090, SDO6090},
  {0x6091, OTYPE_ARRAY, 16, 0, acName6091, SDO6091},
  {0x6092, OTYPE_VAR, 0, 0, acName6092, SDO6092},
  {0xffff, 0xff, 0xff, 0xff, NULL, NULL}
};
