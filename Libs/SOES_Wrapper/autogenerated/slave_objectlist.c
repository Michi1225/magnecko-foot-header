#include "esc_coe.h"
#include "utypes.h"
#include <stddef.h>

#ifndef HW_REV
#define HW_REV "1.0"
#endif

#ifndef SW_REV
#define SW_REV "1.0"
#endif

/*
SM2 / 0x1C12
└── 0x1600 Command RxPDO
    ├── Control Word
    ├── Operation Mode
    └── Magnet Command

SM3 / 0x1C13
├── 0x1A00 General status
├── 0x1A01 IMU
├── 0x1A02 ToF distances
├── 0x1A03 ToF SNR
├── 0x1A04 LDC data
├── 0x1A05 Hall data
└── 0x1A06 Capacitor voltage
 */


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
static const char acName1600[] = "Command RxPDO";
static const char acName1600_00[] = "Max SubIndex";
static const char acName1600_01[] = "Control Word";
static const char acName1600_02[] = "Operation Modes";
static const char acName1600_03[] = "Magnet Command";
static const char acName1A00[] = "General Status TxPDO";
static const char acName1A00_00[] = "Max SubIndex";
static const char acName1A00_01[] = "Status word";
static const char acName1A00_02[] = "Operation modes display";
static const char acName1A00_03[] = "Magnet Status";
static const char acName1A00_04[] = "Force Estimate";
static const char acName1A00_05[] = "Temperature";
static const char acName1A01[] = "IMU TxPDO";
static const char acName1A01_00[] = "Max SubIndex";
static const char acName1A01_01[] = "Gyro X";
static const char acName1A01_02[] = "Gyro Y";
static const char acName1A01_03[] = "Gyro Z";
static const char acName1A01_04[] = "Acc X";
static const char acName1A01_05[] = "Acc Y";
static const char acName1A01_06[] = "Acc Z";
static const char acName1A01_07[] = "Quat R";
static const char acName1A01_08[] = "Quat I";
static const char acName1A01_09[] = "Quat J";
static const char acName1A01_0A[] = "Quat K";
static const char acName1A02[] = "ToF Distance TxPDO";
static const char acName1A02_00[] = "Max SubIndex";
static const char acName1A02_01[] = "ToF Distance 0";
static const char acName1A02_02[] = "ToF Distance 1";
static const char acName1A02_03[] = "ToF Distance 2";
static const char acName1A02_04[] = "ToF Distance 3";
static const char acName1A02_05[] = "ToF Distance 4";
static const char acName1A02_06[] = "ToF Distance 5";
static const char acName1A02_07[] = "ToF Distance 6";
static const char acName1A02_08[] = "ToF Distance 7";
static const char acName1A02_09[] = "ToF Distance 8";
static const char acName1A02_0A[] = "ToF Distance 9";
static const char acName1A02_0B[] = "ToF Distance 10";
static const char acName1A02_0C[] = "ToF Distance 11";
static const char acName1A02_0D[] = "ToF Distance 12";
static const char acName1A02_0E[] = "ToF Distance 13";
static const char acName1A02_0F[] = "ToF Distance 14";
static const char acName1A02_10[] = "ToF Distance 15";
static const char acName1A02_11[] = "ToF Distance 16";
static const char acName1A02_12[] = "ToF Distance 17";
static const char acName1A02_13[] = "ToF Distance 18";
static const char acName1A02_14[] = "ToF Distance 19";
static const char acName1A02_15[] = "ToF Distance 20";
static const char acName1A02_16[] = "ToF Distance 21";
static const char acName1A02_17[] = "ToF Distance 22";
static const char acName1A02_18[] = "ToF Distance 23";
static const char acName1A02_19[] = "ToF Distance 24";
static const char acName1A02_1A[] = "ToF Distance 25";
static const char acName1A02_1B[] = "ToF Distance 26";
static const char acName1A02_1C[] = "ToF Distance 27";
static const char acName1A02_1D[] = "ToF Distance 28";
static const char acName1A02_1E[] = "ToF Distance 29";
static const char acName1A02_1F[] = "ToF Distance 30";
static const char acName1A02_20[] = "ToF Distance 31";
static const char acName1A02_21[] = "ToF Distance 32";
static const char acName1A02_22[] = "ToF Distance 33";
static const char acName1A02_23[] = "ToF Distance 34";
static const char acName1A02_24[] = "ToF Distance 35";
static const char acName1A02_25[] = "ToF Distance 36";
static const char acName1A02_26[] = "ToF Distance 37";
static const char acName1A02_27[] = "ToF Distance 38";
static const char acName1A02_28[] = "ToF Distance 39";
static const char acName1A02_29[] = "ToF Distance 40";
static const char acName1A02_2A[] = "ToF Distance 41";
static const char acName1A02_2B[] = "ToF Distance 42";
static const char acName1A02_2C[] = "ToF Distance 43";
static const char acName1A02_2D[] = "ToF Distance 44";
static const char acName1A02_2E[] = "ToF Distance 45";
static const char acName1A02_2F[] = "ToF Distance 46";
static const char acName1A02_30[] = "ToF Distance 47";
static const char acName1A02_31[] = "ToF Distance 48";
static const char acName1A02_32[] = "ToF Distance 49";
static const char acName1A02_33[] = "ToF Distance 50";
static const char acName1A02_34[] = "ToF Distance 51";
static const char acName1A02_35[] = "ToF Distance 52";
static const char acName1A02_36[] = "ToF Distance 53";
static const char acName1A02_37[] = "ToF Distance 54";
static const char acName1A02_38[] = "ToF Distance 55";
static const char acName1A02_39[] = "ToF Distance 56";
static const char acName1A02_3A[] = "ToF Distance 57";
static const char acName1A02_3B[] = "ToF Distance 58";
static const char acName1A02_3C[] = "ToF Distance 59";
static const char acName1A02_3D[] = "ToF Distance 60";
static const char acName1A02_3E[] = "ToF Distance 61";
static const char acName1A02_3F[] = "ToF Distance 62";
static const char acName1A02_40[] = "ToF Distance 63";
static const char acName1A03[] = "ToF SNR TxPDO";
static const char acName1A03_00[] = "Max SubIndex";
static const char acName1A03_01[] = "ToF SNR 0";
static const char acName1A03_02[] = "ToF SNR 1";
static const char acName1A03_03[] = "ToF SNR 2";
static const char acName1A03_04[] = "ToF SNR 3";
static const char acName1A03_05[] = "ToF SNR 4";
static const char acName1A03_06[] = "ToF SNR 5";
static const char acName1A03_07[] = "ToF SNR 6";
static const char acName1A03_08[] = "ToF SNR 7";
static const char acName1A03_09[] = "ToF SNR 8";
static const char acName1A03_0A[] = "ToF SNR 9";
static const char acName1A03_0B[] = "ToF SNR 10";
static const char acName1A03_0C[] = "ToF SNR 11";
static const char acName1A03_0D[] = "ToF SNR 12";
static const char acName1A03_0E[] = "ToF SNR 13";
static const char acName1A03_0F[] = "ToF SNR 14";
static const char acName1A03_10[] = "ToF SNR 15";
static const char acName1A03_11[] = "ToF SNR 16";
static const char acName1A03_12[] = "ToF SNR 17";
static const char acName1A03_13[] = "ToF SNR 18";
static const char acName1A03_14[] = "ToF SNR 19";
static const char acName1A03_15[] = "ToF SNR 20";
static const char acName1A03_16[] = "ToF SNR 21";
static const char acName1A03_17[] = "ToF SNR 22";
static const char acName1A03_18[] = "ToF SNR 23";
static const char acName1A03_19[] = "ToF SNR 24";
static const char acName1A03_1A[] = "ToF SNR 25";
static const char acName1A03_1B[] = "ToF SNR 26";
static const char acName1A03_1C[] = "ToF SNR 27";
static const char acName1A03_1D[] = "ToF SNR 28";
static const char acName1A03_1E[] = "ToF SNR 29";
static const char acName1A03_1F[] = "ToF SNR 30";
static const char acName1A03_20[] = "ToF SNR 31";
static const char acName1A03_21[] = "ToF SNR 32";
static const char acName1A03_22[] = "ToF SNR 33";
static const char acName1A03_23[] = "ToF SNR 34";
static const char acName1A03_24[] = "ToF SNR 35";
static const char acName1A03_25[] = "ToF SNR 36";
static const char acName1A03_26[] = "ToF SNR 37";
static const char acName1A03_27[] = "ToF SNR 38";
static const char acName1A03_28[] = "ToF SNR 39";
static const char acName1A03_29[] = "ToF SNR 40";
static const char acName1A03_2A[] = "ToF SNR 41";
static const char acName1A03_2B[] = "ToF SNR 42";
static const char acName1A03_2C[] = "ToF SNR 43";
static const char acName1A03_2D[] = "ToF SNR 44";
static const char acName1A03_2E[] = "ToF SNR 45";
static const char acName1A03_2F[] = "ToF SNR 46";
static const char acName1A03_30[] = "ToF SNR 47";
static const char acName1A03_31[] = "ToF SNR 48";
static const char acName1A03_32[] = "ToF SNR 49";
static const char acName1A03_33[] = "ToF SNR 50";
static const char acName1A03_34[] = "ToF SNR 51";
static const char acName1A03_35[] = "ToF SNR 52";
static const char acName1A03_36[] = "ToF SNR 53";
static const char acName1A03_37[] = "ToF SNR 54";
static const char acName1A03_38[] = "ToF SNR 55";
static const char acName1A03_39[] = "ToF SNR 56";
static const char acName1A03_3A[] = "ToF SNR 57";
static const char acName1A03_3B[] = "ToF SNR 58";
static const char acName1A03_3C[] = "ToF SNR 59";
static const char acName1A03_3D[] = "ToF SNR 60";
static const char acName1A03_3E[] = "ToF SNR 61";
static const char acName1A03_3F[] = "ToF SNR 62";
static const char acName1A03_40[] = "ToF SNR 63";
static const char acName1A04[] = "LDC TxPDO";
static const char acName1A04_00[] = "Max SubIndex";
static const char acName1A04_01[] = "LDC Frequency 0";
static const char acName1A04_02[] = "LDC Frequency 1";
static const char acName1A04_03[] = "LDC Frequency 2";
static const char acName1A04_04[] = "LDC Frequency 3";
static const char acName1A04_05[] = "LDC RP 0";
static const char acName1A04_06[] = "LDC RP 1";
static const char acName1A04_07[] = "LDC RP 2";
static const char acName1A04_08[] = "LDC RP 3";
static const char acName1A05[] = "Hall TxPDO";
static const char acName1A05_00[] = "Max SubIndex";
static const char acName1A05_01[] = "Hall X 0";
static const char acName1A05_02[] = "Hall X 1";
static const char acName1A05_03[] = "Hall X 2";
static const char acName1A05_04[] = "Hall X 3";
static const char acName1A05_05[] = "Hall Y 0";
static const char acName1A05_06[] = "Hall Y 1";
static const char acName1A05_07[] = "Hall Y 2";
static const char acName1A05_08[] = "Hall Y 3";
static const char acName1A05_09[] = "Hall Z 0";
static const char acName1A05_0A[] = "Hall Z 1";
static const char acName1A05_0B[] = "Hall Z 2";
static const char acName1A05_0C[] = "Hall Z 3";
static const char acName1A06[] = "Capacitor Voltage TxPDO";
static const char acName1A06_00[] = "Max SubIndex";
static const char acName1A06_01[] = "Capacitor Voltage";
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
static const char acName1C13_01[] = "PDO Mapping 1";
static const char acName1C13_02[] = "PDO Mapping 2";
static const char acName1C13_03[] = "PDO Mapping 3";
static const char acName1C13_04[] = "PDO Mapping 4";
static const char acName1C13_05[] = "PDO Mapping 5";
static const char acName1C13_06[] = "PDO Mapping 6";
static const char acName1C13_07[] = "PDO Mapping 7";
static const char acName2000[] = "Magnet Status";
static const char acName2001[] = "Force Estimate";
static const char acName2040[] = "Temperatures";
static const char acName2040_00[] = "Max SubIndex";
static const char acName2040_01[] = "Temperature";
static const char acName2100[] = "Device Parameters";
static const char acName2100_00[] = "Max SubIndex";
static const char acName2100_01[] = "Pulse Time";
static const char acName2100_02[] = "EPM Number";
static const char acName2101[] = "Force Estimate Parameters";
static const char acName2101_00[] = "Max SubIndex";
static const char acName2101_01[] = "Parameter 0";
static const char acName2101_02[] = "Parameter 1";
static const char acName2101_03[] = "Parameter 2";
static const char acName2101_04[] = "Parameter 3";
static const char acName2101_05[] = "Parameter 4";
static const char acName2101_06[] = "Parameter 5";
static const char acName2101_07[] = "Parameter 6";
static const char acName2101_08[] = "Parameter 7";
static const char acName2101_09[] = "Parameter 8";
static const char acName2101_0A[] = "Parameter 9";
static const char acName2101_0B[] = "Parameter 10";
static const char acName2101_0C[] = "Parameter 11";
static const char acName2101_0D[] = "Parameter 12";
static const char acName2101_0E[] = "Parameter 13";
static const char acName2101_0F[] = "Parameter 14";
static const char acName2101_10[] = "Parameter 15";
static const char acName2101_11[] = "Parameter 16";
static const char acName2101_12[] = "Parameter 17";
static const char acName2101_13[] = "Parameter 18";
static const char acName2101_14[] = "Parameter 19";
static const char acName2101_15[] = "Parameter 20";
static const char acName2101_16[] = "Parameter 21";
static const char acName2101_17[] = "Parameter 22";
static const char acName2101_18[] = "Parameter 23";
static const char acName2101_19[] = "Parameter 24";
static const char acName2101_1A[] = "Parameter 25";
static const char acName2101_1B[] = "Parameter 26";
static const char acName2101_1C[] = "Parameter 27";
static const char acName2101_1D[] = "Parameter 28";
static const char acName2101_1E[] = "Parameter 29";
static const char acName2101_1F[] = "Parameter 30";
static const char acName2101_20[] = "Parameter 31";
static const char acName2101_21[] = "Parameter 32";
static const char acName2101_22[] = "Parameter 33";
static const char acName2101_23[] = "Parameter 34";
static const char acName2101_24[] = "Parameter 35";
static const char acName2101_25[] = "Parameter 36";
static const char acName2101_26[] = "Parameter 37";
static const char acName2101_27[] = "Parameter 38";
static const char acName2101_28[] = "Parameter 39";
static const char acName2101_29[] = "Parameter 40";
static const char acName2101_2A[] = "Parameter 41";
static const char acName2101_2B[] = "Parameter 42";
static const char acName2101_2C[] = "Parameter 43";
static const char acName2101_2D[] = "Parameter 44";
static const char acName2101_2E[] = "Parameter 45";
static const char acName2101_2F[] = "Parameter 46";
static const char acName2101_30[] = "Parameter 47";
static const char acName2101_31[] = "Parameter 48";
static const char acName2101_32[] = "Parameter 49";
static const char acName2101_33[] = "Parameter 50";
static const char acName2101_34[] = "Parameter 51";
static const char acName2101_35[] = "Parameter 52";
static const char acName2101_36[] = "Parameter 53";
static const char acName2101_37[] = "Parameter 54";
static const char acName2101_38[] = "Parameter 55";
static const char acName2101_39[] = "Parameter 56";
static const char acName2101_3A[] = "Parameter 57";
static const char acName2101_3B[] = "Parameter 58";
static const char acName2101_3C[] = "Parameter 59";
static const char acName2101_3D[] = "Parameter 60";
static const char acName2101_3E[] = "Parameter 61";
static const char acName2101_3F[] = "Parameter 62";
static const char acName2101_40[] = "Parameter 63";
static const char acName2101_41[] = "Parameter 64";
static const char acName2101_42[] = "Parameter 65";
static const char acName2101_43[] = "Parameter 66";
static const char acName2101_44[] = "Parameter 67";
static const char acName2101_45[] = "Parameter 68";
static const char acName2101_46[] = "Parameter 69";
static const char acName2101_47[] = "Parameter 70";
static const char acName2101_48[] = "Parameter 71";
static const char acName2101_49[] = "Parameter 72";
static const char acName2101_4A[] = "Parameter 73";
static const char acName2101_4B[] = "Parameter 74";
static const char acName2101_4C[] = "Parameter 75";
static const char acName2101_4D[] = "Parameter 76";
static const char acName2101_4E[] = "Parameter 77";
static const char acName2101_4F[] = "Parameter 78";
static const char acName2101_50[] = "Parameter 79";
static const char acName2101_51[] = "Parameter 80";
static const char acName2101_52[] = "Parameter 81";
static const char acName2101_53[] = "Parameter 82";
static const char acName2101_54[] = "Parameter 83";
static const char acName2101_55[] = "Parameter 84";
static const char acName2101_56[] = "Parameter 85";
static const char acName2101_57[] = "Parameter 86";
static const char acName2101_58[] = "Parameter 87";
static const char acName2101_59[] = "Parameter 88";
static const char acName2101_5A[] = "Parameter 89";
static const char acName2101_5B[] = "Parameter 90";
static const char acName2101_5C[] = "Parameter 91";
static const char acName2101_5D[] = "Parameter 92";
static const char acName2101_5E[] = "Parameter 93";
static const char acName2101_5F[] = "Parameter 94";
static const char acName2101_60[] = "Parameter 95";
static const char acName2101_61[] = "Parameter 96";
static const char acName2101_62[] = "Parameter 97";
static const char acName2101_63[] = "Parameter 98";
static const char acName2101_64[] = "Parameter 99";
static const char acName2101_65[] = "Parameter 100";
static const char acName2101_66[] = "Parameter 101";
static const char acName2101_67[] = "Parameter 102";
static const char acName2101_68[] = "Parameter 103";
static const char acName2101_69[] = "Parameter 104";
static const char acName2101_6A[] = "Parameter 105";
static const char acName2101_6B[] = "Parameter 106";
static const char acName2101_6C[] = "Parameter 107";
static const char acName2101_6D[] = "Parameter 108";
static const char acName2101_6E[] = "Parameter 109";
static const char acName2101_6F[] = "Parameter 110";
static const char acName2101_70[] = "Parameter 111";
static const char acName2101_71[] = "Parameter 112";
static const char acName2101_72[] = "Parameter 113";
static const char acName2101_73[] = "Parameter 114";
static const char acName2101_74[] = "Parameter 115";
static const char acName2101_75[] = "Parameter 116";
static const char acName2101_76[] = "Parameter 117";
static const char acName2101_77[] = "Parameter 118";
static const char acName2101_78[] = "Parameter 119";
static const char acName2101_79[] = "Parameter 120";
static const char acName2101_7A[] = "Parameter 121";
static const char acName2101_7B[] = "Parameter 122";
static const char acName2101_7C[] = "Parameter 123";
static const char acName2101_7D[] = "Parameter 124";
static const char acName2101_7E[] = "Parameter 125";
static const char acName2101_7F[] = "Parameter 126";
static const char acName2101_80[] = "Parameter 127";
static const char acName2101_81[] = "Parameter 128";
static const char acName2101_82[] = "Parameter 129";
static const char acName2101_83[] = "Parameter 130";
static const char acName2101_84[] = "Parameter 131";
static const char acName2101_85[] = "Parameter 132";
static const char acName2101_86[] = "Parameter 133";
static const char acName2101_87[] = "Parameter 134";
static const char acName2101_88[] = "Parameter 135";
static const char acName2101_89[] = "Parameter 136";
static const char acName2101_8A[] = "Parameter 137";
static const char acName2101_8B[] = "Parameter 138";
static const char acName2101_8C[] = "Parameter 139";
static const char acName2101_8D[] = "Parameter 140";
static const char acName2101_8E[] = "Parameter 141";
static const char acName2101_8F[] = "Parameter 142";
static const char acName2101_90[] = "Parameter 143";
static const char acName2101_91[] = "Parameter 144";
static const char acName2101_92[] = "Parameter 145";
static const char acName2101_93[] = "Parameter 146";
static const char acName2101_94[] = "Parameter 147";
static const char acName2101_95[] = "Parameter 148";
static const char acName2101_96[] = "Parameter 149";
static const char acName2101_97[] = "Parameter 150";
static const char acName2101_98[] = "Parameter 151";
static const char acName2101_99[] = "Parameter 152";
static const char acName2101_9A[] = "Parameter 153";
static const char acName2101_9B[] = "Parameter 154";
static const char acName2101_9C[] = "Parameter 155";
static const char acName2101_9D[] = "Parameter 156";
static const char acName2101_9E[] = "Parameter 157";
static const char acName2101_9F[] = "Parameter 158";
static const char acName2101_A0[] = "Parameter 159";
static const char acName2101_A1[] = "Parameter 160";
static const char acName2101_A2[] = "Parameter 161";
static const char acName2101_A3[] = "Parameter 162";
static const char acName2101_A4[] = "Parameter 163";
static const char acName2101_A5[] = "Parameter 164";
static const char acName2101_A6[] = "Parameter 165";
static const char acName2101_A7[] = "Parameter 166";
static const char acName2101_A8[] = "Parameter 167";
static const char acName2101_A9[] = "Parameter 168";
static const char acName2101_AA[] = "Parameter 169";
static const char acName2101_AB[] = "Parameter 170";
static const char acName2101_AC[] = "Parameter 171";
static const char acName2101_AD[] = "Parameter 172";
static const char acName2101_AE[] = "Parameter 173";
static const char acName2101_AF[] = "Parameter 174";
static const char acName2101_B0[] = "Parameter 175";
static const char acName2101_B1[] = "Parameter 176";
static const char acName2101_B2[] = "Parameter 177";
static const char acName2101_B3[] = "Parameter 178";
static const char acName2101_B4[] = "Parameter 179";
static const char acName2101_B5[] = "Parameter 180";
static const char acName2101_B6[] = "Parameter 181";
static const char acName2101_B7[] = "Parameter 182";
static const char acName2101_B8[] = "Parameter 183";
static const char acName2101_B9[] = "Parameter 184";
static const char acName2101_BA[] = "Parameter 185";
static const char acName2101_BB[] = "Parameter 186";
static const char acName2101_BC[] = "Parameter 187";
static const char acName2101_BD[] = "Parameter 188";
static const char acName2101_BE[] = "Parameter 189";
static const char acName2101_BF[] = "Parameter 190";
static const char acName2101_C0[] = "Parameter 191";
static const char acName2101_C1[] = "Parameter 192";
static const char acName2101_C2[] = "Parameter 193";
static const char acName2101_C3[] = "Parameter 194";
static const char acName2101_C4[] = "Parameter 195";
static const char acName2101_C5[] = "Parameter 196";
static const char acName2101_C6[] = "Parameter 197";
static const char acName2101_C7[] = "Parameter 198";
static const char acName2101_C8[] = "Parameter 199";
static const char acName2101_C9[] = "Parameter 200";
static const char acName2101_CA[] = "Parameter 201";
static const char acName2101_CB[] = "Parameter 202";
static const char acName2101_CC[] = "Parameter 203";
static const char acName2101_CD[] = "Parameter 204";
static const char acName2101_CE[] = "Parameter 205";
static const char acName2101_CF[] = "Parameter 206";
static const char acName2101_D0[] = "Parameter 207";
static const char acName2101_D1[] = "Parameter 208";
static const char acName2101_D2[] = "Parameter 209";
static const char acName2101_D3[] = "Parameter 210";
static const char acName2101_D4[] = "Parameter 211";
static const char acName2101_D5[] = "Parameter 212";
static const char acName2101_D6[] = "Parameter 213";
static const char acName2101_D7[] = "Parameter 214";
static const char acName2101_D8[] = "Parameter 215";
static const char acName2101_D9[] = "Parameter 216";
static const char acName2101_DA[] = "Parameter 217";
static const char acName2101_DB[] = "Parameter 218";
static const char acName2101_DC[] = "Parameter 219";
static const char acName2101_DD[] = "Parameter 220";
static const char acName2101_DE[] = "Parameter 221";
static const char acName2101_DF[] = "Parameter 222";
static const char acName2101_E0[] = "Parameter 223";
static const char acName2101_E1[] = "Parameter 224";
static const char acName2101_E2[] = "Parameter 225";
static const char acName2101_E3[] = "Parameter 226";
static const char acName2101_E4[] = "Parameter 227";
static const char acName2101_E5[] = "Parameter 228";
static const char acName2101_E6[] = "Parameter 229";
static const char acName2101_E7[] = "Parameter 230";
static const char acName2101_E8[] = "Parameter 231";
static const char acName2101_E9[] = "Parameter 232";
static const char acName2101_EA[] = "Parameter 233";
static const char acName2101_EB[] = "Parameter 234";
static const char acName2101_EC[] = "Parameter 235";
static const char acName2101_ED[] = "Parameter 236";
static const char acName2101_EE[] = "Parameter 237";
static const char acName2101_EF[] = "Parameter 238";
static const char acName2101_F0[] = "Parameter 239";
static const char acName2101_F1[] = "Parameter 240";
static const char acName2101_F2[] = "Parameter 241";
static const char acName2101_F3[] = "Parameter 242";
static const char acName2101_F4[] = "Parameter 243";
static const char acName2101_F5[] = "Parameter 244";
static const char acName2101_F6[] = "Parameter 245";
static const char acName2101_F7[] = "Parameter 246";
static const char acName2101_F8[] = "Parameter 247";
static const char acName2101_F9[] = "Parameter 248";
static const char acName2101_FA[] = "Parameter 249";
static const char acName603F[] = "Error Code";
static const char acName6040[] = "Control Word";
static const char acName6041[] = "Status word";
static const char acName6060[] = "Operation Modes";
static const char acName6061[] = "Operation modes display";
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
static const char acName60A0[] = "ToF Distance";
static const char acName60A0_00[] = "Max SubIndex";
static const char acName60A0_01[] = "Distance 0";
static const char acName60A0_02[] = "Distance 1";
static const char acName60A0_03[] = "Distance 2";
static const char acName60A0_04[] = "Distance 3";
static const char acName60A0_05[] = "Distance 4";
static const char acName60A0_06[] = "Distance 5";
static const char acName60A0_07[] = "Distance 6";
static const char acName60A0_08[] = "Distance 7";
static const char acName60A0_09[] = "Distance 8";
static const char acName60A0_0A[] = "Distance 9";
static const char acName60A0_0B[] = "Distance 10";
static const char acName60A0_0C[] = "Distance 11";
static const char acName60A0_0D[] = "Distance 12";
static const char acName60A0_0E[] = "Distance 13";
static const char acName60A0_0F[] = "Distance 14";
static const char acName60A0_10[] = "Distance 15";
static const char acName60A0_11[] = "Distance 16";
static const char acName60A0_12[] = "Distance 17";
static const char acName60A0_13[] = "Distance 18";
static const char acName60A0_14[] = "Distance 19";
static const char acName60A0_15[] = "Distance 20";
static const char acName60A0_16[] = "Distance 21";
static const char acName60A0_17[] = "Distance 22";
static const char acName60A0_18[] = "Distance 23";
static const char acName60A0_19[] = "Distance 24";
static const char acName60A0_1A[] = "Distance 25";
static const char acName60A0_1B[] = "Distance 26";
static const char acName60A0_1C[] = "Distance 27";
static const char acName60A0_1D[] = "Distance 28";
static const char acName60A0_1E[] = "Distance 29";
static const char acName60A0_1F[] = "Distance 30";
static const char acName60A0_20[] = "Distance 31";
static const char acName60A0_21[] = "Distance 32";
static const char acName60A0_22[] = "Distance 33";
static const char acName60A0_23[] = "Distance 34";
static const char acName60A0_24[] = "Distance 35";
static const char acName60A0_25[] = "Distance 36";
static const char acName60A0_26[] = "Distance 37";
static const char acName60A0_27[] = "Distance 38";
static const char acName60A0_28[] = "Distance 39";
static const char acName60A0_29[] = "Distance 40";
static const char acName60A0_2A[] = "Distance 41";
static const char acName60A0_2B[] = "Distance 42";
static const char acName60A0_2C[] = "Distance 43";
static const char acName60A0_2D[] = "Distance 44";
static const char acName60A0_2E[] = "Distance 45";
static const char acName60A0_2F[] = "Distance 46";
static const char acName60A0_30[] = "Distance 47";
static const char acName60A0_31[] = "Distance 48";
static const char acName60A0_32[] = "Distance 49";
static const char acName60A0_33[] = "Distance 50";
static const char acName60A0_34[] = "Distance 51";
static const char acName60A0_35[] = "Distance 52";
static const char acName60A0_36[] = "Distance 53";
static const char acName60A0_37[] = "Distance 54";
static const char acName60A0_38[] = "Distance 55";
static const char acName60A0_39[] = "Distance 56";
static const char acName60A0_3A[] = "Distance 57";
static const char acName60A0_3B[] = "Distance 58";
static const char acName60A0_3C[] = "Distance 59";
static const char acName60A0_3D[] = "Distance 60";
static const char acName60A0_3E[] = "Distance 61";
static const char acName60A0_3F[] = "Distance 62";
static const char acName60A0_40[] = "Distance 63";
static const char acName60A1[] = "ToF SNR";
static const char acName60A1_00[] = "Max SubIndex";
static const char acName60A1_01[] = "SNR 0";
static const char acName60A1_02[] = "SNR 1";
static const char acName60A1_03[] = "SNR 2";
static const char acName60A1_04[] = "SNR 3";
static const char acName60A1_05[] = "SNR 4";
static const char acName60A1_06[] = "SNR 5";
static const char acName60A1_07[] = "SNR 6";
static const char acName60A1_08[] = "SNR 7";
static const char acName60A1_09[] = "SNR 8";
static const char acName60A1_0A[] = "SNR 9";
static const char acName60A1_0B[] = "SNR 10";
static const char acName60A1_0C[] = "SNR 11";
static const char acName60A1_0D[] = "SNR 12";
static const char acName60A1_0E[] = "SNR 13";
static const char acName60A1_0F[] = "SNR 14";
static const char acName60A1_10[] = "SNR 15";
static const char acName60A1_11[] = "SNR 16";
static const char acName60A1_12[] = "SNR 17";
static const char acName60A1_13[] = "SNR 18";
static const char acName60A1_14[] = "SNR 19";
static const char acName60A1_15[] = "SNR 20";
static const char acName60A1_16[] = "SNR 21";
static const char acName60A1_17[] = "SNR 22";
static const char acName60A1_18[] = "SNR 23";
static const char acName60A1_19[] = "SNR 24";
static const char acName60A1_1A[] = "SNR 25";
static const char acName60A1_1B[] = "SNR 26";
static const char acName60A1_1C[] = "SNR 27";
static const char acName60A1_1D[] = "SNR 28";
static const char acName60A1_1E[] = "SNR 29";
static const char acName60A1_1F[] = "SNR 30";
static const char acName60A1_20[] = "SNR 31";
static const char acName60A1_21[] = "SNR 32";
static const char acName60A1_22[] = "SNR 33";
static const char acName60A1_23[] = "SNR 34";
static const char acName60A1_24[] = "SNR 35";
static const char acName60A1_25[] = "SNR 36";
static const char acName60A1_26[] = "SNR 37";
static const char acName60A1_27[] = "SNR 38";
static const char acName60A1_28[] = "SNR 39";
static const char acName60A1_29[] = "SNR 40";
static const char acName60A1_2A[] = "SNR 41";
static const char acName60A1_2B[] = "SNR 42";
static const char acName60A1_2C[] = "SNR 43";
static const char acName60A1_2D[] = "SNR 44";
static const char acName60A1_2E[] = "SNR 45";
static const char acName60A1_2F[] = "SNR 46";
static const char acName60A1_30[] = "SNR 47";
static const char acName60A1_31[] = "SNR 48";
static const char acName60A1_32[] = "SNR 49";
static const char acName60A1_33[] = "SNR 50";
static const char acName60A1_34[] = "SNR 51";
static const char acName60A1_35[] = "SNR 52";
static const char acName60A1_36[] = "SNR 53";
static const char acName60A1_37[] = "SNR 54";
static const char acName60A1_38[] = "SNR 55";
static const char acName60A1_39[] = "SNR 56";
static const char acName60A1_3A[] = "SNR 57";
static const char acName60A1_3B[] = "SNR 58";
static const char acName60A1_3C[] = "SNR 59";
static const char acName60A1_3D[] = "SNR 60";
static const char acName60A1_3E[] = "SNR 61";
static const char acName60A1_3F[] = "SNR 62";
static const char acName60A1_40[] = "SNR 63";
static const char acName60B0[] = "LDC Data";
static const char acName60B0_00[] = "Max SubIndex";
static const char acName60B0_01[] = "Frequency 0";
static const char acName60B0_02[] = "Frequency 1";
static const char acName60B0_03[] = "Frequency 2";
static const char acName60B0_04[] = "Frequency 3";
static const char acName60B0_05[] = "RP 0";
static const char acName60B0_06[] = "RP 1";
static const char acName60B0_07[] = "RP 2";
static const char acName60B0_08[] = "RP 3";
static const char acName60C0[] = "Hall Magnetic Field";
static const char acName60C0_00[] = "Max SubIndex";
static const char acName60C0_01[] = "X 0";
static const char acName60C0_02[] = "X 1";
static const char acName60C0_03[] = "X 2";
static const char acName60C0_04[] = "X 3";
static const char acName60C0_05[] = "Y 0";
static const char acName60C0_06[] = "Y 1";
static const char acName60C0_07[] = "Y 2";
static const char acName60C0_08[] = "Y 3";
static const char acName60C0_09[] = "Z 0";
static const char acName60C0_0A[] = "Z 1";
static const char acName60C0_0B[] = "Z 2";
static const char acName60C0_0C[] = "Z 3";
static const char acName60D0[] = "Capacitor Voltage";
static const char acName7000[] = "Magnet Command";

typedef struct
{
  uint8_t maxsub;
  uint16_t value[1];
} _sm2_assignment_storage;

typedef struct
{
  uint8_t maxsub;
  uint16_t value[7];
} _sm3_assignment_storage;

static _sm2_assignment_storage SM1C12 = {1, {0x1600}};
static _sm3_assignment_storage SM1C13 =
{
  7, {0x1A00, 0x1A01, 0x1A02, 0x1A03, 0x1A04, 0x1A05, 0x1A06}
};

const _objd SDO1000[] =
{
  {0x00, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1000, 0x00000000, NULL},
};
const _objd SDO1008[] =
{
  {0x00, DTYPE_VISIBLE_STRING, 160, ATYPE_RO, acName1008, 0, "Magnecko EPM Controller"},
};
const _objd SDO1009[] =
{
  {0x00, DTYPE_VISIBLE_STRING, 24, ATYPE_RO, acName1009, 0, HW_REV},
};
const _objd SDO100A[] =
{
  {0x00, DTYPE_VISIBLE_STRING, 24, ATYPE_RO, acName100A, 0, SW_REV},
};
const _objd SDO1018[] =
{
  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RO, acName1018_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_01, 0, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_02, 0, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_03, 0, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_04, 0, &Obj.serial},
};

const _objd SDO1600[] =
{
  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RO, acName1600_00, 3, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_01, 0x60400010, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_02, 0x60600008, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_03, 0x70000008, NULL},
};

const _objd SDO1A00[] =
{
  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RO, acName1A00_00, 5, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_01, 0x60410010, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_02, 0x60610008, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_03, 0x20000008, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_04, 0x20010020, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_05, 0x20400110, NULL},
};

const _objd SDO1A01[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A01_00, 10, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_01, 0x60900110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_02, 0x60900210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_03, 0x60900310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_04, 0x60900410, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_05, 0x60900510, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_06, 0x60900610, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_07, 0x60900710, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_08, 0x60900810, NULL},
  {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_09, 0x60900910, NULL},
  {0x0A, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_0A, 0x60900A10, NULL},
};

const _objd SDO1A02[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A02_00, 64, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_01, 0x60A00110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_02, 0x60A00210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_03, 0x60A00310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_04, 0x60A00410, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_05, 0x60A00510, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_06, 0x60A00610, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_07, 0x60A00710, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_08, 0x60A00810, NULL},
  {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_09, 0x60A00910, NULL},
  {0x0A, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_0A, 0x60A00A10, NULL},
  {0x0B, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_0B, 0x60A00B10, NULL},
  {0x0C, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_0C, 0x60A00C10, NULL},
  {0x0D, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_0D, 0x60A00D10, NULL},
  {0x0E, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_0E, 0x60A00E10, NULL},
  {0x0F, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_0F, 0x60A00F10, NULL},
  {0x10, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_10, 0x60A01010, NULL},
  {0x11, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_11, 0x60A01110, NULL},
  {0x12, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_12, 0x60A01210, NULL},
  {0x13, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_13, 0x60A01310, NULL},
  {0x14, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_14, 0x60A01410, NULL},
  {0x15, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_15, 0x60A01510, NULL},
  {0x16, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_16, 0x60A01610, NULL},
  {0x17, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_17, 0x60A01710, NULL},
  {0x18, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_18, 0x60A01810, NULL},
  {0x19, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_19, 0x60A01910, NULL},
  {0x1A, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_1A, 0x60A01A10, NULL},
  {0x1B, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_1B, 0x60A01B10, NULL},
  {0x1C, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_1C, 0x60A01C10, NULL},
  {0x1D, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_1D, 0x60A01D10, NULL},
  {0x1E, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_1E, 0x60A01E10, NULL},
  {0x1F, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_1F, 0x60A01F10, NULL},
  {0x20, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_20, 0x60A02010, NULL},
  {0x21, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_21, 0x60A02110, NULL},
  {0x22, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_22, 0x60A02210, NULL},
  {0x23, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_23, 0x60A02310, NULL},
  {0x24, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_24, 0x60A02410, NULL},
  {0x25, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_25, 0x60A02510, NULL},
  {0x26, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_26, 0x60A02610, NULL},
  {0x27, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_27, 0x60A02710, NULL},
  {0x28, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_28, 0x60A02810, NULL},
  {0x29, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_29, 0x60A02910, NULL},
  {0x2A, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_2A, 0x60A02A10, NULL},
  {0x2B, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_2B, 0x60A02B10, NULL},
  {0x2C, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_2C, 0x60A02C10, NULL},
  {0x2D, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_2D, 0x60A02D10, NULL},
  {0x2E, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_2E, 0x60A02E10, NULL},
  {0x2F, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_2F, 0x60A02F10, NULL},
  {0x30, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_30, 0x60A03010, NULL},
  {0x31, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_31, 0x60A03110, NULL},
  {0x32, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_32, 0x60A03210, NULL},
  {0x33, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_33, 0x60A03310, NULL},
  {0x34, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_34, 0x60A03410, NULL},
  {0x35, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_35, 0x60A03510, NULL},
  {0x36, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_36, 0x60A03610, NULL},
  {0x37, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_37, 0x60A03710, NULL},
  {0x38, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_38, 0x60A03810, NULL},
  {0x39, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_39, 0x60A03910, NULL},
  {0x3A, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_3A, 0x60A03A10, NULL},
  {0x3B, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_3B, 0x60A03B10, NULL},
  {0x3C, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_3C, 0x60A03C10, NULL},
  {0x3D, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_3D, 0x60A03D10, NULL},
  {0x3E, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_3E, 0x60A03E10, NULL},
  {0x3F, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_3F, 0x60A03F10, NULL},
  {0x40, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_40, 0x60A04010, NULL},
};

const _objd SDO1A03[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A03_00, 64, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_01, 0x60A10108, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_02, 0x60A10208, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_03, 0x60A10308, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_04, 0x60A10408, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_05, 0x60A10508, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_06, 0x60A10608, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_07, 0x60A10708, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_08, 0x60A10808, NULL},
  {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_09, 0x60A10908, NULL},
  {0x0A, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_0A, 0x60A10A08, NULL},
  {0x0B, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_0B, 0x60A10B08, NULL},
  {0x0C, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_0C, 0x60A10C08, NULL},
  {0x0D, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_0D, 0x60A10D08, NULL},
  {0x0E, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_0E, 0x60A10E08, NULL},
  {0x0F, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_0F, 0x60A10F08, NULL},
  {0x10, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_10, 0x60A11008, NULL},
  {0x11, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_11, 0x60A11108, NULL},
  {0x12, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_12, 0x60A11208, NULL},
  {0x13, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_13, 0x60A11308, NULL},
  {0x14, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_14, 0x60A11408, NULL},
  {0x15, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_15, 0x60A11508, NULL},
  {0x16, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_16, 0x60A11608, NULL},
  {0x17, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_17, 0x60A11708, NULL},
  {0x18, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_18, 0x60A11808, NULL},
  {0x19, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_19, 0x60A11908, NULL},
  {0x1A, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_1A, 0x60A11A08, NULL},
  {0x1B, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_1B, 0x60A11B08, NULL},
  {0x1C, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_1C, 0x60A11C08, NULL},
  {0x1D, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_1D, 0x60A11D08, NULL},
  {0x1E, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_1E, 0x60A11E08, NULL},
  {0x1F, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_1F, 0x60A11F08, NULL},
  {0x20, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_20, 0x60A12008, NULL},
  {0x21, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_21, 0x60A12108, NULL},
  {0x22, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_22, 0x60A12208, NULL},
  {0x23, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_23, 0x60A12308, NULL},
  {0x24, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_24, 0x60A12408, NULL},
  {0x25, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_25, 0x60A12508, NULL},
  {0x26, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_26, 0x60A12608, NULL},
  {0x27, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_27, 0x60A12708, NULL},
  {0x28, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_28, 0x60A12808, NULL},
  {0x29, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_29, 0x60A12908, NULL},
  {0x2A, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_2A, 0x60A12A08, NULL},
  {0x2B, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_2B, 0x60A12B08, NULL},
  {0x2C, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_2C, 0x60A12C08, NULL},
  {0x2D, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_2D, 0x60A12D08, NULL},
  {0x2E, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_2E, 0x60A12E08, NULL},
  {0x2F, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_2F, 0x60A12F08, NULL},
  {0x30, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_30, 0x60A13008, NULL},
  {0x31, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_31, 0x60A13108, NULL},
  {0x32, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_32, 0x60A13208, NULL},
  {0x33, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_33, 0x60A13308, NULL},
  {0x34, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_34, 0x60A13408, NULL},
  {0x35, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_35, 0x60A13508, NULL},
  {0x36, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_36, 0x60A13608, NULL},
  {0x37, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_37, 0x60A13708, NULL},
  {0x38, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_38, 0x60A13808, NULL},
  {0x39, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_39, 0x60A13908, NULL},
  {0x3A, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_3A, 0x60A13A08, NULL},
  {0x3B, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_3B, 0x60A13B08, NULL},
  {0x3C, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_3C, 0x60A13C08, NULL},
  {0x3D, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_3D, 0x60A13D08, NULL},
  {0x3E, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_3E, 0x60A13E08, NULL},
  {0x3F, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_3F, 0x60A13F08, NULL},
  {0x40, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_40, 0x60A14008, NULL},
};

const _objd SDO1A04[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A04_00, 8, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_01, 0x60B00110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_02, 0x60B00210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_03, 0x60B00310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_04, 0x60B00410, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_05, 0x60B00510, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_06, 0x60B00610, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_07, 0x60B00710, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_08, 0x60B00810, NULL},
};

const _objd SDO1A05[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A05_00, 12, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_01, 0x60C00110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_02, 0x60C00210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_03, 0x60C00310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_04, 0x60C00410, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_05, 0x60C00510, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_06, 0x60C00610, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_07, 0x60C00710, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_08, 0x60C00810, NULL},
  {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_09, 0x60C00910, NULL},
  {0x0A, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_0A, 0x60C00A10, NULL},
  {0x0B, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_0B, 0x60C00B10, NULL},
  {0x0C, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_0C, 0x60C00C10, NULL},
};

const _objd SDO1A06[] =
{
  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RO, acName1A06_00, 1, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A06_01, 0x60D00010, NULL},
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
  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RO, acName1C12_00, 1, &SM1C12.maxsub},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_01, 0x1600, &SM1C12.value[0]},
};
const _objd SDO1C13[] =
{
  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RO, acName1C13_00, 7, &SM1C13.maxsub},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_01, 0x1A00, &SM1C13.value[0]},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_02, 0x1A01, &SM1C13.value[1]},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_03, 0x1A02, &SM1C13.value[2]},
  {0x04, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_04, 0x1A03, &SM1C13.value[3]},
  {0x05, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_05, 0x1A04, &SM1C13.value[4]},
  {0x06, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_06, 0x1A05, &SM1C13.value[5]},
  {0x07, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_07, 0x1A06, &SM1C13.value[6]},
};

const _objd SDO2000[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName2000, 0, &Obj.Magnet_Status},
};
const _objd SDO2001[] =
{
  {0x00, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName2001, 0, &Obj.Force_Estimate},
};
const _objd SDO2040[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName2040_00, 1, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName2040_01, 0, &Obj.Temperature},
};
const _objd SDO2100[] =
{
  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RO, acName2100_00, 2, NULL},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName2100_01, 0, &Obj.Pulse_Time},
  {0x02, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName2100_02, 0, &Obj.EPM_Number},
};
const _objd SDO2101[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName2101_00, 250, NULL},
  {0x01, DTYPE_REAL32, 32, ATYPE_RW, acName2101_01, 0, &Obj.Force_Estimate_Params[0]},
  {0x02, DTYPE_REAL32, 32, ATYPE_RW, acName2101_02, 0, &Obj.Force_Estimate_Params[1]},
  {0x03, DTYPE_REAL32, 32, ATYPE_RW, acName2101_03, 0, &Obj.Force_Estimate_Params[2]},
  {0x04, DTYPE_REAL32, 32, ATYPE_RW, acName2101_04, 0, &Obj.Force_Estimate_Params[3]},
  {0x05, DTYPE_REAL32, 32, ATYPE_RW, acName2101_05, 0, &Obj.Force_Estimate_Params[4]},
  {0x06, DTYPE_REAL32, 32, ATYPE_RW, acName2101_06, 0, &Obj.Force_Estimate_Params[5]},
  {0x07, DTYPE_REAL32, 32, ATYPE_RW, acName2101_07, 0, &Obj.Force_Estimate_Params[6]},
  {0x08, DTYPE_REAL32, 32, ATYPE_RW, acName2101_08, 0, &Obj.Force_Estimate_Params[7]},
  {0x09, DTYPE_REAL32, 32, ATYPE_RW, acName2101_09, 0, &Obj.Force_Estimate_Params[8]},
  {0x0A, DTYPE_REAL32, 32, ATYPE_RW, acName2101_0A, 0, &Obj.Force_Estimate_Params[9]},
  {0x0B, DTYPE_REAL32, 32, ATYPE_RW, acName2101_0B, 0, &Obj.Force_Estimate_Params[10]},
  {0x0C, DTYPE_REAL32, 32, ATYPE_RW, acName2101_0C, 0, &Obj.Force_Estimate_Params[11]},
  {0x0D, DTYPE_REAL32, 32, ATYPE_RW, acName2101_0D, 0, &Obj.Force_Estimate_Params[12]},
  {0x0E, DTYPE_REAL32, 32, ATYPE_RW, acName2101_0E, 0, &Obj.Force_Estimate_Params[13]},
  {0x0F, DTYPE_REAL32, 32, ATYPE_RW, acName2101_0F, 0, &Obj.Force_Estimate_Params[14]},
  {0x10, DTYPE_REAL32, 32, ATYPE_RW, acName2101_10, 0, &Obj.Force_Estimate_Params[15]},
  {0x11, DTYPE_REAL32, 32, ATYPE_RW, acName2101_11, 0, &Obj.Force_Estimate_Params[16]},
  {0x12, DTYPE_REAL32, 32, ATYPE_RW, acName2101_12, 0, &Obj.Force_Estimate_Params[17]},
  {0x13, DTYPE_REAL32, 32, ATYPE_RW, acName2101_13, 0, &Obj.Force_Estimate_Params[18]},
  {0x14, DTYPE_REAL32, 32, ATYPE_RW, acName2101_14, 0, &Obj.Force_Estimate_Params[19]},
  {0x15, DTYPE_REAL32, 32, ATYPE_RW, acName2101_15, 0, &Obj.Force_Estimate_Params[20]},
  {0x16, DTYPE_REAL32, 32, ATYPE_RW, acName2101_16, 0, &Obj.Force_Estimate_Params[21]},
  {0x17, DTYPE_REAL32, 32, ATYPE_RW, acName2101_17, 0, &Obj.Force_Estimate_Params[22]},
  {0x18, DTYPE_REAL32, 32, ATYPE_RW, acName2101_18, 0, &Obj.Force_Estimate_Params[23]},
  {0x19, DTYPE_REAL32, 32, ATYPE_RW, acName2101_19, 0, &Obj.Force_Estimate_Params[24]},
  {0x1A, DTYPE_REAL32, 32, ATYPE_RW, acName2101_1A, 0, &Obj.Force_Estimate_Params[25]},
  {0x1B, DTYPE_REAL32, 32, ATYPE_RW, acName2101_1B, 0, &Obj.Force_Estimate_Params[26]},
  {0x1C, DTYPE_REAL32, 32, ATYPE_RW, acName2101_1C, 0, &Obj.Force_Estimate_Params[27]},
  {0x1D, DTYPE_REAL32, 32, ATYPE_RW, acName2101_1D, 0, &Obj.Force_Estimate_Params[28]},
  {0x1E, DTYPE_REAL32, 32, ATYPE_RW, acName2101_1E, 0, &Obj.Force_Estimate_Params[29]},
  {0x1F, DTYPE_REAL32, 32, ATYPE_RW, acName2101_1F, 0, &Obj.Force_Estimate_Params[30]},
  {0x20, DTYPE_REAL32, 32, ATYPE_RW, acName2101_20, 0, &Obj.Force_Estimate_Params[31]},
  {0x21, DTYPE_REAL32, 32, ATYPE_RW, acName2101_21, 0, &Obj.Force_Estimate_Params[32]},
  {0x22, DTYPE_REAL32, 32, ATYPE_RW, acName2101_22, 0, &Obj.Force_Estimate_Params[33]},
  {0x23, DTYPE_REAL32, 32, ATYPE_RW, acName2101_23, 0, &Obj.Force_Estimate_Params[34]},
  {0x24, DTYPE_REAL32, 32, ATYPE_RW, acName2101_24, 0, &Obj.Force_Estimate_Params[35]},
  {0x25, DTYPE_REAL32, 32, ATYPE_RW, acName2101_25, 0, &Obj.Force_Estimate_Params[36]},
  {0x26, DTYPE_REAL32, 32, ATYPE_RW, acName2101_26, 0, &Obj.Force_Estimate_Params[37]},
  {0x27, DTYPE_REAL32, 32, ATYPE_RW, acName2101_27, 0, &Obj.Force_Estimate_Params[38]},
  {0x28, DTYPE_REAL32, 32, ATYPE_RW, acName2101_28, 0, &Obj.Force_Estimate_Params[39]},
  {0x29, DTYPE_REAL32, 32, ATYPE_RW, acName2101_29, 0, &Obj.Force_Estimate_Params[40]},
  {0x2A, DTYPE_REAL32, 32, ATYPE_RW, acName2101_2A, 0, &Obj.Force_Estimate_Params[41]},
  {0x2B, DTYPE_REAL32, 32, ATYPE_RW, acName2101_2B, 0, &Obj.Force_Estimate_Params[42]},
  {0x2C, DTYPE_REAL32, 32, ATYPE_RW, acName2101_2C, 0, &Obj.Force_Estimate_Params[43]},
  {0x2D, DTYPE_REAL32, 32, ATYPE_RW, acName2101_2D, 0, &Obj.Force_Estimate_Params[44]},
  {0x2E, DTYPE_REAL32, 32, ATYPE_RW, acName2101_2E, 0, &Obj.Force_Estimate_Params[45]},
  {0x2F, DTYPE_REAL32, 32, ATYPE_RW, acName2101_2F, 0, &Obj.Force_Estimate_Params[46]},
  {0x30, DTYPE_REAL32, 32, ATYPE_RW, acName2101_30, 0, &Obj.Force_Estimate_Params[47]},
  {0x31, DTYPE_REAL32, 32, ATYPE_RW, acName2101_31, 0, &Obj.Force_Estimate_Params[48]},
  {0x32, DTYPE_REAL32, 32, ATYPE_RW, acName2101_32, 0, &Obj.Force_Estimate_Params[49]},
  {0x33, DTYPE_REAL32, 32, ATYPE_RW, acName2101_33, 0, &Obj.Force_Estimate_Params[50]},
  {0x34, DTYPE_REAL32, 32, ATYPE_RW, acName2101_34, 0, &Obj.Force_Estimate_Params[51]},
  {0x35, DTYPE_REAL32, 32, ATYPE_RW, acName2101_35, 0, &Obj.Force_Estimate_Params[52]},
  {0x36, DTYPE_REAL32, 32, ATYPE_RW, acName2101_36, 0, &Obj.Force_Estimate_Params[53]},
  {0x37, DTYPE_REAL32, 32, ATYPE_RW, acName2101_37, 0, &Obj.Force_Estimate_Params[54]},
  {0x38, DTYPE_REAL32, 32, ATYPE_RW, acName2101_38, 0, &Obj.Force_Estimate_Params[55]},
  {0x39, DTYPE_REAL32, 32, ATYPE_RW, acName2101_39, 0, &Obj.Force_Estimate_Params[56]},
  {0x3A, DTYPE_REAL32, 32, ATYPE_RW, acName2101_3A, 0, &Obj.Force_Estimate_Params[57]},
  {0x3B, DTYPE_REAL32, 32, ATYPE_RW, acName2101_3B, 0, &Obj.Force_Estimate_Params[58]},
  {0x3C, DTYPE_REAL32, 32, ATYPE_RW, acName2101_3C, 0, &Obj.Force_Estimate_Params[59]},
  {0x3D, DTYPE_REAL32, 32, ATYPE_RW, acName2101_3D, 0, &Obj.Force_Estimate_Params[60]},
  {0x3E, DTYPE_REAL32, 32, ATYPE_RW, acName2101_3E, 0, &Obj.Force_Estimate_Params[61]},
  {0x3F, DTYPE_REAL32, 32, ATYPE_RW, acName2101_3F, 0, &Obj.Force_Estimate_Params[62]},
  {0x40, DTYPE_REAL32, 32, ATYPE_RW, acName2101_40, 0, &Obj.Force_Estimate_Params[63]},
  {0x41, DTYPE_REAL32, 32, ATYPE_RW, acName2101_41, 0, &Obj.Force_Estimate_Params[64]},
  {0x42, DTYPE_REAL32, 32, ATYPE_RW, acName2101_42, 0, &Obj.Force_Estimate_Params[65]},
  {0x43, DTYPE_REAL32, 32, ATYPE_RW, acName2101_43, 0, &Obj.Force_Estimate_Params[66]},
  {0x44, DTYPE_REAL32, 32, ATYPE_RW, acName2101_44, 0, &Obj.Force_Estimate_Params[67]},
  {0x45, DTYPE_REAL32, 32, ATYPE_RW, acName2101_45, 0, &Obj.Force_Estimate_Params[68]},
  {0x46, DTYPE_REAL32, 32, ATYPE_RW, acName2101_46, 0, &Obj.Force_Estimate_Params[69]},
  {0x47, DTYPE_REAL32, 32, ATYPE_RW, acName2101_47, 0, &Obj.Force_Estimate_Params[70]},
  {0x48, DTYPE_REAL32, 32, ATYPE_RW, acName2101_48, 0, &Obj.Force_Estimate_Params[71]},
  {0x49, DTYPE_REAL32, 32, ATYPE_RW, acName2101_49, 0, &Obj.Force_Estimate_Params[72]},
  {0x4A, DTYPE_REAL32, 32, ATYPE_RW, acName2101_4A, 0, &Obj.Force_Estimate_Params[73]},
  {0x4B, DTYPE_REAL32, 32, ATYPE_RW, acName2101_4B, 0, &Obj.Force_Estimate_Params[74]},
  {0x4C, DTYPE_REAL32, 32, ATYPE_RW, acName2101_4C, 0, &Obj.Force_Estimate_Params[75]},
  {0x4D, DTYPE_REAL32, 32, ATYPE_RW, acName2101_4D, 0, &Obj.Force_Estimate_Params[76]},
  {0x4E, DTYPE_REAL32, 32, ATYPE_RW, acName2101_4E, 0, &Obj.Force_Estimate_Params[77]},
  {0x4F, DTYPE_REAL32, 32, ATYPE_RW, acName2101_4F, 0, &Obj.Force_Estimate_Params[78]},
  {0x50, DTYPE_REAL32, 32, ATYPE_RW, acName2101_50, 0, &Obj.Force_Estimate_Params[79]},
  {0x51, DTYPE_REAL32, 32, ATYPE_RW, acName2101_51, 0, &Obj.Force_Estimate_Params[80]},
  {0x52, DTYPE_REAL32, 32, ATYPE_RW, acName2101_52, 0, &Obj.Force_Estimate_Params[81]},
  {0x53, DTYPE_REAL32, 32, ATYPE_RW, acName2101_53, 0, &Obj.Force_Estimate_Params[82]},
  {0x54, DTYPE_REAL32, 32, ATYPE_RW, acName2101_54, 0, &Obj.Force_Estimate_Params[83]},
  {0x55, DTYPE_REAL32, 32, ATYPE_RW, acName2101_55, 0, &Obj.Force_Estimate_Params[84]},
  {0x56, DTYPE_REAL32, 32, ATYPE_RW, acName2101_56, 0, &Obj.Force_Estimate_Params[85]},
  {0x57, DTYPE_REAL32, 32, ATYPE_RW, acName2101_57, 0, &Obj.Force_Estimate_Params[86]},
  {0x58, DTYPE_REAL32, 32, ATYPE_RW, acName2101_58, 0, &Obj.Force_Estimate_Params[87]},
  {0x59, DTYPE_REAL32, 32, ATYPE_RW, acName2101_59, 0, &Obj.Force_Estimate_Params[88]},
  {0x5A, DTYPE_REAL32, 32, ATYPE_RW, acName2101_5A, 0, &Obj.Force_Estimate_Params[89]},
  {0x5B, DTYPE_REAL32, 32, ATYPE_RW, acName2101_5B, 0, &Obj.Force_Estimate_Params[90]},
  {0x5C, DTYPE_REAL32, 32, ATYPE_RW, acName2101_5C, 0, &Obj.Force_Estimate_Params[91]},
  {0x5D, DTYPE_REAL32, 32, ATYPE_RW, acName2101_5D, 0, &Obj.Force_Estimate_Params[92]},
  {0x5E, DTYPE_REAL32, 32, ATYPE_RW, acName2101_5E, 0, &Obj.Force_Estimate_Params[93]},
  {0x5F, DTYPE_REAL32, 32, ATYPE_RW, acName2101_5F, 0, &Obj.Force_Estimate_Params[94]},
  {0x60, DTYPE_REAL32, 32, ATYPE_RW, acName2101_60, 0, &Obj.Force_Estimate_Params[95]},
  {0x61, DTYPE_REAL32, 32, ATYPE_RW, acName2101_61, 0, &Obj.Force_Estimate_Params[96]},
  {0x62, DTYPE_REAL32, 32, ATYPE_RW, acName2101_62, 0, &Obj.Force_Estimate_Params[97]},
  {0x63, DTYPE_REAL32, 32, ATYPE_RW, acName2101_63, 0, &Obj.Force_Estimate_Params[98]},
  {0x64, DTYPE_REAL32, 32, ATYPE_RW, acName2101_64, 0, &Obj.Force_Estimate_Params[99]},
  {0x65, DTYPE_REAL32, 32, ATYPE_RW, acName2101_65, 0, &Obj.Force_Estimate_Params[100]},
  {0x66, DTYPE_REAL32, 32, ATYPE_RW, acName2101_66, 0, &Obj.Force_Estimate_Params[101]},
  {0x67, DTYPE_REAL32, 32, ATYPE_RW, acName2101_67, 0, &Obj.Force_Estimate_Params[102]},
  {0x68, DTYPE_REAL32, 32, ATYPE_RW, acName2101_68, 0, &Obj.Force_Estimate_Params[103]},
  {0x69, DTYPE_REAL32, 32, ATYPE_RW, acName2101_69, 0, &Obj.Force_Estimate_Params[104]},
  {0x6A, DTYPE_REAL32, 32, ATYPE_RW, acName2101_6A, 0, &Obj.Force_Estimate_Params[105]},
  {0x6B, DTYPE_REAL32, 32, ATYPE_RW, acName2101_6B, 0, &Obj.Force_Estimate_Params[106]},
  {0x6C, DTYPE_REAL32, 32, ATYPE_RW, acName2101_6C, 0, &Obj.Force_Estimate_Params[107]},
  {0x6D, DTYPE_REAL32, 32, ATYPE_RW, acName2101_6D, 0, &Obj.Force_Estimate_Params[108]},
  {0x6E, DTYPE_REAL32, 32, ATYPE_RW, acName2101_6E, 0, &Obj.Force_Estimate_Params[109]},
  {0x6F, DTYPE_REAL32, 32, ATYPE_RW, acName2101_6F, 0, &Obj.Force_Estimate_Params[110]},
  {0x70, DTYPE_REAL32, 32, ATYPE_RW, acName2101_70, 0, &Obj.Force_Estimate_Params[111]},
  {0x71, DTYPE_REAL32, 32, ATYPE_RW, acName2101_71, 0, &Obj.Force_Estimate_Params[112]},
  {0x72, DTYPE_REAL32, 32, ATYPE_RW, acName2101_72, 0, &Obj.Force_Estimate_Params[113]},
  {0x73, DTYPE_REAL32, 32, ATYPE_RW, acName2101_73, 0, &Obj.Force_Estimate_Params[114]},
  {0x74, DTYPE_REAL32, 32, ATYPE_RW, acName2101_74, 0, &Obj.Force_Estimate_Params[115]},
  {0x75, DTYPE_REAL32, 32, ATYPE_RW, acName2101_75, 0, &Obj.Force_Estimate_Params[116]},
  {0x76, DTYPE_REAL32, 32, ATYPE_RW, acName2101_76, 0, &Obj.Force_Estimate_Params[117]},
  {0x77, DTYPE_REAL32, 32, ATYPE_RW, acName2101_77, 0, &Obj.Force_Estimate_Params[118]},
  {0x78, DTYPE_REAL32, 32, ATYPE_RW, acName2101_78, 0, &Obj.Force_Estimate_Params[119]},
  {0x79, DTYPE_REAL32, 32, ATYPE_RW, acName2101_79, 0, &Obj.Force_Estimate_Params[120]},
  {0x7A, DTYPE_REAL32, 32, ATYPE_RW, acName2101_7A, 0, &Obj.Force_Estimate_Params[121]},
  {0x7B, DTYPE_REAL32, 32, ATYPE_RW, acName2101_7B, 0, &Obj.Force_Estimate_Params[122]},
  {0x7C, DTYPE_REAL32, 32, ATYPE_RW, acName2101_7C, 0, &Obj.Force_Estimate_Params[123]},
  {0x7D, DTYPE_REAL32, 32, ATYPE_RW, acName2101_7D, 0, &Obj.Force_Estimate_Params[124]},
  {0x7E, DTYPE_REAL32, 32, ATYPE_RW, acName2101_7E, 0, &Obj.Force_Estimate_Params[125]},
  {0x7F, DTYPE_REAL32, 32, ATYPE_RW, acName2101_7F, 0, &Obj.Force_Estimate_Params[126]},
  {0x80, DTYPE_REAL32, 32, ATYPE_RW, acName2101_80, 0, &Obj.Force_Estimate_Params[127]},
  {0x81, DTYPE_REAL32, 32, ATYPE_RW, acName2101_81, 0, &Obj.Force_Estimate_Params[128]},
  {0x82, DTYPE_REAL32, 32, ATYPE_RW, acName2101_82, 0, &Obj.Force_Estimate_Params[129]},
  {0x83, DTYPE_REAL32, 32, ATYPE_RW, acName2101_83, 0, &Obj.Force_Estimate_Params[130]},
  {0x84, DTYPE_REAL32, 32, ATYPE_RW, acName2101_84, 0, &Obj.Force_Estimate_Params[131]},
  {0x85, DTYPE_REAL32, 32, ATYPE_RW, acName2101_85, 0, &Obj.Force_Estimate_Params[132]},
  {0x86, DTYPE_REAL32, 32, ATYPE_RW, acName2101_86, 0, &Obj.Force_Estimate_Params[133]},
  {0x87, DTYPE_REAL32, 32, ATYPE_RW, acName2101_87, 0, &Obj.Force_Estimate_Params[134]},
  {0x88, DTYPE_REAL32, 32, ATYPE_RW, acName2101_88, 0, &Obj.Force_Estimate_Params[135]},
  {0x89, DTYPE_REAL32, 32, ATYPE_RW, acName2101_89, 0, &Obj.Force_Estimate_Params[136]},
  {0x8A, DTYPE_REAL32, 32, ATYPE_RW, acName2101_8A, 0, &Obj.Force_Estimate_Params[137]},
  {0x8B, DTYPE_REAL32, 32, ATYPE_RW, acName2101_8B, 0, &Obj.Force_Estimate_Params[138]},
  {0x8C, DTYPE_REAL32, 32, ATYPE_RW, acName2101_8C, 0, &Obj.Force_Estimate_Params[139]},
  {0x8D, DTYPE_REAL32, 32, ATYPE_RW, acName2101_8D, 0, &Obj.Force_Estimate_Params[140]},
  {0x8E, DTYPE_REAL32, 32, ATYPE_RW, acName2101_8E, 0, &Obj.Force_Estimate_Params[141]},
  {0x8F, DTYPE_REAL32, 32, ATYPE_RW, acName2101_8F, 0, &Obj.Force_Estimate_Params[142]},
  {0x90, DTYPE_REAL32, 32, ATYPE_RW, acName2101_90, 0, &Obj.Force_Estimate_Params[143]},
  {0x91, DTYPE_REAL32, 32, ATYPE_RW, acName2101_91, 0, &Obj.Force_Estimate_Params[144]},
  {0x92, DTYPE_REAL32, 32, ATYPE_RW, acName2101_92, 0, &Obj.Force_Estimate_Params[145]},
  {0x93, DTYPE_REAL32, 32, ATYPE_RW, acName2101_93, 0, &Obj.Force_Estimate_Params[146]},
  {0x94, DTYPE_REAL32, 32, ATYPE_RW, acName2101_94, 0, &Obj.Force_Estimate_Params[147]},
  {0x95, DTYPE_REAL32, 32, ATYPE_RW, acName2101_95, 0, &Obj.Force_Estimate_Params[148]},
  {0x96, DTYPE_REAL32, 32, ATYPE_RW, acName2101_96, 0, &Obj.Force_Estimate_Params[149]},
  {0x97, DTYPE_REAL32, 32, ATYPE_RW, acName2101_97, 0, &Obj.Force_Estimate_Params[150]},
  {0x98, DTYPE_REAL32, 32, ATYPE_RW, acName2101_98, 0, &Obj.Force_Estimate_Params[151]},
  {0x99, DTYPE_REAL32, 32, ATYPE_RW, acName2101_99, 0, &Obj.Force_Estimate_Params[152]},
  {0x9A, DTYPE_REAL32, 32, ATYPE_RW, acName2101_9A, 0, &Obj.Force_Estimate_Params[153]},
  {0x9B, DTYPE_REAL32, 32, ATYPE_RW, acName2101_9B, 0, &Obj.Force_Estimate_Params[154]},
  {0x9C, DTYPE_REAL32, 32, ATYPE_RW, acName2101_9C, 0, &Obj.Force_Estimate_Params[155]},
  {0x9D, DTYPE_REAL32, 32, ATYPE_RW, acName2101_9D, 0, &Obj.Force_Estimate_Params[156]},
  {0x9E, DTYPE_REAL32, 32, ATYPE_RW, acName2101_9E, 0, &Obj.Force_Estimate_Params[157]},
  {0x9F, DTYPE_REAL32, 32, ATYPE_RW, acName2101_9F, 0, &Obj.Force_Estimate_Params[158]},
  {0xA0, DTYPE_REAL32, 32, ATYPE_RW, acName2101_A0, 0, &Obj.Force_Estimate_Params[159]},
  {0xA1, DTYPE_REAL32, 32, ATYPE_RW, acName2101_A1, 0, &Obj.Force_Estimate_Params[160]},
  {0xA2, DTYPE_REAL32, 32, ATYPE_RW, acName2101_A2, 0, &Obj.Force_Estimate_Params[161]},
  {0xA3, DTYPE_REAL32, 32, ATYPE_RW, acName2101_A3, 0, &Obj.Force_Estimate_Params[162]},
  {0xA4, DTYPE_REAL32, 32, ATYPE_RW, acName2101_A4, 0, &Obj.Force_Estimate_Params[163]},
  {0xA5, DTYPE_REAL32, 32, ATYPE_RW, acName2101_A5, 0, &Obj.Force_Estimate_Params[164]},
  {0xA6, DTYPE_REAL32, 32, ATYPE_RW, acName2101_A6, 0, &Obj.Force_Estimate_Params[165]},
  {0xA7, DTYPE_REAL32, 32, ATYPE_RW, acName2101_A7, 0, &Obj.Force_Estimate_Params[166]},
  {0xA8, DTYPE_REAL32, 32, ATYPE_RW, acName2101_A8, 0, &Obj.Force_Estimate_Params[167]},
  {0xA9, DTYPE_REAL32, 32, ATYPE_RW, acName2101_A9, 0, &Obj.Force_Estimate_Params[168]},
  {0xAA, DTYPE_REAL32, 32, ATYPE_RW, acName2101_AA, 0, &Obj.Force_Estimate_Params[169]},
  {0xAB, DTYPE_REAL32, 32, ATYPE_RW, acName2101_AB, 0, &Obj.Force_Estimate_Params[170]},
  {0xAC, DTYPE_REAL32, 32, ATYPE_RW, acName2101_AC, 0, &Obj.Force_Estimate_Params[171]},
  {0xAD, DTYPE_REAL32, 32, ATYPE_RW, acName2101_AD, 0, &Obj.Force_Estimate_Params[172]},
  {0xAE, DTYPE_REAL32, 32, ATYPE_RW, acName2101_AE, 0, &Obj.Force_Estimate_Params[173]},
  {0xAF, DTYPE_REAL32, 32, ATYPE_RW, acName2101_AF, 0, &Obj.Force_Estimate_Params[174]},
  {0xB0, DTYPE_REAL32, 32, ATYPE_RW, acName2101_B0, 0, &Obj.Force_Estimate_Params[175]},
  {0xB1, DTYPE_REAL32, 32, ATYPE_RW, acName2101_B1, 0, &Obj.Force_Estimate_Params[176]},
  {0xB2, DTYPE_REAL32, 32, ATYPE_RW, acName2101_B2, 0, &Obj.Force_Estimate_Params[177]},
  {0xB3, DTYPE_REAL32, 32, ATYPE_RW, acName2101_B3, 0, &Obj.Force_Estimate_Params[178]},
  {0xB4, DTYPE_REAL32, 32, ATYPE_RW, acName2101_B4, 0, &Obj.Force_Estimate_Params[179]},
  {0xB5, DTYPE_REAL32, 32, ATYPE_RW, acName2101_B5, 0, &Obj.Force_Estimate_Params[180]},
  {0xB6, DTYPE_REAL32, 32, ATYPE_RW, acName2101_B6, 0, &Obj.Force_Estimate_Params[181]},
  {0xB7, DTYPE_REAL32, 32, ATYPE_RW, acName2101_B7, 0, &Obj.Force_Estimate_Params[182]},
  {0xB8, DTYPE_REAL32, 32, ATYPE_RW, acName2101_B8, 0, &Obj.Force_Estimate_Params[183]},
  {0xB9, DTYPE_REAL32, 32, ATYPE_RW, acName2101_B9, 0, &Obj.Force_Estimate_Params[184]},
  {0xBA, DTYPE_REAL32, 32, ATYPE_RW, acName2101_BA, 0, &Obj.Force_Estimate_Params[185]},
  {0xBB, DTYPE_REAL32, 32, ATYPE_RW, acName2101_BB, 0, &Obj.Force_Estimate_Params[186]},
  {0xBC, DTYPE_REAL32, 32, ATYPE_RW, acName2101_BC, 0, &Obj.Force_Estimate_Params[187]},
  {0xBD, DTYPE_REAL32, 32, ATYPE_RW, acName2101_BD, 0, &Obj.Force_Estimate_Params[188]},
  {0xBE, DTYPE_REAL32, 32, ATYPE_RW, acName2101_BE, 0, &Obj.Force_Estimate_Params[189]},
  {0xBF, DTYPE_REAL32, 32, ATYPE_RW, acName2101_BF, 0, &Obj.Force_Estimate_Params[190]},
  {0xC0, DTYPE_REAL32, 32, ATYPE_RW, acName2101_C0, 0, &Obj.Force_Estimate_Params[191]},
  {0xC1, DTYPE_REAL32, 32, ATYPE_RW, acName2101_C1, 0, &Obj.Force_Estimate_Params[192]},
  {0xC2, DTYPE_REAL32, 32, ATYPE_RW, acName2101_C2, 0, &Obj.Force_Estimate_Params[193]},
  {0xC3, DTYPE_REAL32, 32, ATYPE_RW, acName2101_C3, 0, &Obj.Force_Estimate_Params[194]},
  {0xC4, DTYPE_REAL32, 32, ATYPE_RW, acName2101_C4, 0, &Obj.Force_Estimate_Params[195]},
  {0xC5, DTYPE_REAL32, 32, ATYPE_RW, acName2101_C5, 0, &Obj.Force_Estimate_Params[196]},
  {0xC6, DTYPE_REAL32, 32, ATYPE_RW, acName2101_C6, 0, &Obj.Force_Estimate_Params[197]},
  {0xC7, DTYPE_REAL32, 32, ATYPE_RW, acName2101_C7, 0, &Obj.Force_Estimate_Params[198]},
  {0xC8, DTYPE_REAL32, 32, ATYPE_RW, acName2101_C8, 0, &Obj.Force_Estimate_Params[199]},
  {0xC9, DTYPE_REAL32, 32, ATYPE_RW, acName2101_C9, 0, &Obj.Force_Estimate_Params[200]},
  {0xCA, DTYPE_REAL32, 32, ATYPE_RW, acName2101_CA, 0, &Obj.Force_Estimate_Params[201]},
  {0xCB, DTYPE_REAL32, 32, ATYPE_RW, acName2101_CB, 0, &Obj.Force_Estimate_Params[202]},
  {0xCC, DTYPE_REAL32, 32, ATYPE_RW, acName2101_CC, 0, &Obj.Force_Estimate_Params[203]},
  {0xCD, DTYPE_REAL32, 32, ATYPE_RW, acName2101_CD, 0, &Obj.Force_Estimate_Params[204]},
  {0xCE, DTYPE_REAL32, 32, ATYPE_RW, acName2101_CE, 0, &Obj.Force_Estimate_Params[205]},
  {0xCF, DTYPE_REAL32, 32, ATYPE_RW, acName2101_CF, 0, &Obj.Force_Estimate_Params[206]},
  {0xD0, DTYPE_REAL32, 32, ATYPE_RW, acName2101_D0, 0, &Obj.Force_Estimate_Params[207]},
  {0xD1, DTYPE_REAL32, 32, ATYPE_RW, acName2101_D1, 0, &Obj.Force_Estimate_Params[208]},
  {0xD2, DTYPE_REAL32, 32, ATYPE_RW, acName2101_D2, 0, &Obj.Force_Estimate_Params[209]},
  {0xD3, DTYPE_REAL32, 32, ATYPE_RW, acName2101_D3, 0, &Obj.Force_Estimate_Params[210]},
  {0xD4, DTYPE_REAL32, 32, ATYPE_RW, acName2101_D4, 0, &Obj.Force_Estimate_Params[211]},
  {0xD5, DTYPE_REAL32, 32, ATYPE_RW, acName2101_D5, 0, &Obj.Force_Estimate_Params[212]},
  {0xD6, DTYPE_REAL32, 32, ATYPE_RW, acName2101_D6, 0, &Obj.Force_Estimate_Params[213]},
  {0xD7, DTYPE_REAL32, 32, ATYPE_RW, acName2101_D7, 0, &Obj.Force_Estimate_Params[214]},
  {0xD8, DTYPE_REAL32, 32, ATYPE_RW, acName2101_D8, 0, &Obj.Force_Estimate_Params[215]},
  {0xD9, DTYPE_REAL32, 32, ATYPE_RW, acName2101_D9, 0, &Obj.Force_Estimate_Params[216]},
  {0xDA, DTYPE_REAL32, 32, ATYPE_RW, acName2101_DA, 0, &Obj.Force_Estimate_Params[217]},
  {0xDB, DTYPE_REAL32, 32, ATYPE_RW, acName2101_DB, 0, &Obj.Force_Estimate_Params[218]},
  {0xDC, DTYPE_REAL32, 32, ATYPE_RW, acName2101_DC, 0, &Obj.Force_Estimate_Params[219]},
  {0xDD, DTYPE_REAL32, 32, ATYPE_RW, acName2101_DD, 0, &Obj.Force_Estimate_Params[220]},
  {0xDE, DTYPE_REAL32, 32, ATYPE_RW, acName2101_DE, 0, &Obj.Force_Estimate_Params[221]},
  {0xDF, DTYPE_REAL32, 32, ATYPE_RW, acName2101_DF, 0, &Obj.Force_Estimate_Params[222]},
  {0xE0, DTYPE_REAL32, 32, ATYPE_RW, acName2101_E0, 0, &Obj.Force_Estimate_Params[223]},
  {0xE1, DTYPE_REAL32, 32, ATYPE_RW, acName2101_E1, 0, &Obj.Force_Estimate_Params[224]},
  {0xE2, DTYPE_REAL32, 32, ATYPE_RW, acName2101_E2, 0, &Obj.Force_Estimate_Params[225]},
  {0xE3, DTYPE_REAL32, 32, ATYPE_RW, acName2101_E3, 0, &Obj.Force_Estimate_Params[226]},
  {0xE4, DTYPE_REAL32, 32, ATYPE_RW, acName2101_E4, 0, &Obj.Force_Estimate_Params[227]},
  {0xE5, DTYPE_REAL32, 32, ATYPE_RW, acName2101_E5, 0, &Obj.Force_Estimate_Params[228]},
  {0xE6, DTYPE_REAL32, 32, ATYPE_RW, acName2101_E6, 0, &Obj.Force_Estimate_Params[229]},
  {0xE7, DTYPE_REAL32, 32, ATYPE_RW, acName2101_E7, 0, &Obj.Force_Estimate_Params[230]},
  {0xE8, DTYPE_REAL32, 32, ATYPE_RW, acName2101_E8, 0, &Obj.Force_Estimate_Params[231]},
  {0xE9, DTYPE_REAL32, 32, ATYPE_RW, acName2101_E9, 0, &Obj.Force_Estimate_Params[232]},
  {0xEA, DTYPE_REAL32, 32, ATYPE_RW, acName2101_EA, 0, &Obj.Force_Estimate_Params[233]},
  {0xEB, DTYPE_REAL32, 32, ATYPE_RW, acName2101_EB, 0, &Obj.Force_Estimate_Params[234]},
  {0xEC, DTYPE_REAL32, 32, ATYPE_RW, acName2101_EC, 0, &Obj.Force_Estimate_Params[235]},
  {0xED, DTYPE_REAL32, 32, ATYPE_RW, acName2101_ED, 0, &Obj.Force_Estimate_Params[236]},
  {0xEE, DTYPE_REAL32, 32, ATYPE_RW, acName2101_EE, 0, &Obj.Force_Estimate_Params[237]},
  {0xEF, DTYPE_REAL32, 32, ATYPE_RW, acName2101_EF, 0, &Obj.Force_Estimate_Params[238]},
  {0xF0, DTYPE_REAL32, 32, ATYPE_RW, acName2101_F0, 0, &Obj.Force_Estimate_Params[239]},
  {0xF1, DTYPE_REAL32, 32, ATYPE_RW, acName2101_F1, 0, &Obj.Force_Estimate_Params[240]},
  {0xF2, DTYPE_REAL32, 32, ATYPE_RW, acName2101_F2, 0, &Obj.Force_Estimate_Params[241]},
  {0xF3, DTYPE_REAL32, 32, ATYPE_RW, acName2101_F3, 0, &Obj.Force_Estimate_Params[242]},
  {0xF4, DTYPE_REAL32, 32, ATYPE_RW, acName2101_F4, 0, &Obj.Force_Estimate_Params[243]},
  {0xF5, DTYPE_REAL32, 32, ATYPE_RW, acName2101_F5, 0, &Obj.Force_Estimate_Params[244]},
  {0xF6, DTYPE_REAL32, 32, ATYPE_RW, acName2101_F6, 0, &Obj.Force_Estimate_Params[245]},
  {0xF7, DTYPE_REAL32, 32, ATYPE_RW, acName2101_F7, 0, &Obj.Force_Estimate_Params[246]},
  {0xF8, DTYPE_REAL32, 32, ATYPE_RW, acName2101_F8, 0, &Obj.Force_Estimate_Params[247]},
  {0xF9, DTYPE_REAL32, 32, ATYPE_RW, acName2101_F9, 0, &Obj.Force_Estimate_Params[248]},
  {0xFA, DTYPE_REAL32, 32, ATYPE_RW, acName2101_FA, 0, &Obj.Force_Estimate_Params[249]},
};

const _objd SDO603F[] =
{
  {0x00, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName603F, 0, &Obj.Error_Code},
};
const _objd SDO6040[] =
{
  {0x00, DTYPE_UNSIGNED16, 16, ATYPE_RW | ATYPE_RXPDO, acName6040, 0, &Obj.Control_Word},
};
const _objd SDO6041[] =
{
  {0x00, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_TXPDO, acName6041, 0, &Obj.Status_Word},
};
const _objd SDO6060[] =
{
  {0x00, DTYPE_INTEGER8, 8, ATYPE_RW | ATYPE_RXPDO, acName6060, 0, &Obj.Operation_Mode},
};
const _objd SDO6061[] =
{
  {0x00, DTYPE_INTEGER8, 8, ATYPE_RO | ATYPE_TXPDO, acName6061, 0, &Obj.Operation_Mode_Display},
};

const _objd SDO6090[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6090_00, 10, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6090_01, 0, &Obj.IMU_Data.Gyro_X},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6090_02, 0, &Obj.IMU_Data.Gyro_Y},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6090_03, 0, &Obj.IMU_Data.Gyro_Z},
  {0x04, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6090_04, 0, &Obj.IMU_Data.Acc_X},
  {0x05, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6090_05, 0, &Obj.IMU_Data.Acc_Y},
  {0x06, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6090_06, 0, &Obj.IMU_Data.Acc_Z},
  {0x07, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6090_07, 0, &Obj.IMU_Data.Quat_R},
  {0x08, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6090_08, 0, &Obj.IMU_Data.Quat_I},
  {0x09, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6090_09, 0, &Obj.IMU_Data.Quat_J},
  {0x0A, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6090_0A, 0, &Obj.IMU_Data.Quat_K},
};

const _objd SDO60A0[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName60A0_00, 64, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_01, 0, &Obj.ToF_Distance[0]},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_02, 0, &Obj.ToF_Distance[1]},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_03, 0, &Obj.ToF_Distance[2]},
  {0x04, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_04, 0, &Obj.ToF_Distance[3]},
  {0x05, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_05, 0, &Obj.ToF_Distance[4]},
  {0x06, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_06, 0, &Obj.ToF_Distance[5]},
  {0x07, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_07, 0, &Obj.ToF_Distance[6]},
  {0x08, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_08, 0, &Obj.ToF_Distance[7]},
  {0x09, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_09, 0, &Obj.ToF_Distance[8]},
  {0x0A, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_0A, 0, &Obj.ToF_Distance[9]},
  {0x0B, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_0B, 0, &Obj.ToF_Distance[10]},
  {0x0C, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_0C, 0, &Obj.ToF_Distance[11]},
  {0x0D, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_0D, 0, &Obj.ToF_Distance[12]},
  {0x0E, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_0E, 0, &Obj.ToF_Distance[13]},
  {0x0F, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_0F, 0, &Obj.ToF_Distance[14]},
  {0x10, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_10, 0, &Obj.ToF_Distance[15]},
  {0x11, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_11, 0, &Obj.ToF_Distance[16]},
  {0x12, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_12, 0, &Obj.ToF_Distance[17]},
  {0x13, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_13, 0, &Obj.ToF_Distance[18]},
  {0x14, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_14, 0, &Obj.ToF_Distance[19]},
  {0x15, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_15, 0, &Obj.ToF_Distance[20]},
  {0x16, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_16, 0, &Obj.ToF_Distance[21]},
  {0x17, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_17, 0, &Obj.ToF_Distance[22]},
  {0x18, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_18, 0, &Obj.ToF_Distance[23]},
  {0x19, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_19, 0, &Obj.ToF_Distance[24]},
  {0x1A, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_1A, 0, &Obj.ToF_Distance[25]},
  {0x1B, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_1B, 0, &Obj.ToF_Distance[26]},
  {0x1C, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_1C, 0, &Obj.ToF_Distance[27]},
  {0x1D, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_1D, 0, &Obj.ToF_Distance[28]},
  {0x1E, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_1E, 0, &Obj.ToF_Distance[29]},
  {0x1F, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_1F, 0, &Obj.ToF_Distance[30]},
  {0x20, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_20, 0, &Obj.ToF_Distance[31]},
  {0x21, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_21, 0, &Obj.ToF_Distance[32]},
  {0x22, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_22, 0, &Obj.ToF_Distance[33]},
  {0x23, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_23, 0, &Obj.ToF_Distance[34]},
  {0x24, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_24, 0, &Obj.ToF_Distance[35]},
  {0x25, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_25, 0, &Obj.ToF_Distance[36]},
  {0x26, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_26, 0, &Obj.ToF_Distance[37]},
  {0x27, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_27, 0, &Obj.ToF_Distance[38]},
  {0x28, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_28, 0, &Obj.ToF_Distance[39]},
  {0x29, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_29, 0, &Obj.ToF_Distance[40]},
  {0x2A, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_2A, 0, &Obj.ToF_Distance[41]},
  {0x2B, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_2B, 0, &Obj.ToF_Distance[42]},
  {0x2C, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_2C, 0, &Obj.ToF_Distance[43]},
  {0x2D, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_2D, 0, &Obj.ToF_Distance[44]},
  {0x2E, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_2E, 0, &Obj.ToF_Distance[45]},
  {0x2F, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_2F, 0, &Obj.ToF_Distance[46]},
  {0x30, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_30, 0, &Obj.ToF_Distance[47]},
  {0x31, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_31, 0, &Obj.ToF_Distance[48]},
  {0x32, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_32, 0, &Obj.ToF_Distance[49]},
  {0x33, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_33, 0, &Obj.ToF_Distance[50]},
  {0x34, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_34, 0, &Obj.ToF_Distance[51]},
  {0x35, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_35, 0, &Obj.ToF_Distance[52]},
  {0x36, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_36, 0, &Obj.ToF_Distance[53]},
  {0x37, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_37, 0, &Obj.ToF_Distance[54]},
  {0x38, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_38, 0, &Obj.ToF_Distance[55]},
  {0x39, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_39, 0, &Obj.ToF_Distance[56]},
  {0x3A, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_3A, 0, &Obj.ToF_Distance[57]},
  {0x3B, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_3B, 0, &Obj.ToF_Distance[58]},
  {0x3C, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_3C, 0, &Obj.ToF_Distance[59]},
  {0x3D, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_3D, 0, &Obj.ToF_Distance[60]},
  {0x3E, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_3E, 0, &Obj.ToF_Distance[61]},
  {0x3F, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_3F, 0, &Obj.ToF_Distance[62]},
  {0x40, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60A0_40, 0, &Obj.ToF_Distance[63]},
};

const _objd SDO60A1[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName60A1_00, 64, NULL},
  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_01, 0, &Obj.ToF_SNR[0]},
  {0x02, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_02, 0, &Obj.ToF_SNR[1]},
  {0x03, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_03, 0, &Obj.ToF_SNR[2]},
  {0x04, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_04, 0, &Obj.ToF_SNR[3]},
  {0x05, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_05, 0, &Obj.ToF_SNR[4]},
  {0x06, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_06, 0, &Obj.ToF_SNR[5]},
  {0x07, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_07, 0, &Obj.ToF_SNR[6]},
  {0x08, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_08, 0, &Obj.ToF_SNR[7]},
  {0x09, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_09, 0, &Obj.ToF_SNR[8]},
  {0x0A, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_0A, 0, &Obj.ToF_SNR[9]},
  {0x0B, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_0B, 0, &Obj.ToF_SNR[10]},
  {0x0C, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_0C, 0, &Obj.ToF_SNR[11]},
  {0x0D, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_0D, 0, &Obj.ToF_SNR[12]},
  {0x0E, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_0E, 0, &Obj.ToF_SNR[13]},
  {0x0F, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_0F, 0, &Obj.ToF_SNR[14]},
  {0x10, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_10, 0, &Obj.ToF_SNR[15]},
  {0x11, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_11, 0, &Obj.ToF_SNR[16]},
  {0x12, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_12, 0, &Obj.ToF_SNR[17]},
  {0x13, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_13, 0, &Obj.ToF_SNR[18]},
  {0x14, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_14, 0, &Obj.ToF_SNR[19]},
  {0x15, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_15, 0, &Obj.ToF_SNR[20]},
  {0x16, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_16, 0, &Obj.ToF_SNR[21]},
  {0x17, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_17, 0, &Obj.ToF_SNR[22]},
  {0x18, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_18, 0, &Obj.ToF_SNR[23]},
  {0x19, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_19, 0, &Obj.ToF_SNR[24]},
  {0x1A, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_1A, 0, &Obj.ToF_SNR[25]},
  {0x1B, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_1B, 0, &Obj.ToF_SNR[26]},
  {0x1C, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_1C, 0, &Obj.ToF_SNR[27]},
  {0x1D, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_1D, 0, &Obj.ToF_SNR[28]},
  {0x1E, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_1E, 0, &Obj.ToF_SNR[29]},
  {0x1F, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_1F, 0, &Obj.ToF_SNR[30]},
  {0x20, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_20, 0, &Obj.ToF_SNR[31]},
  {0x21, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_21, 0, &Obj.ToF_SNR[32]},
  {0x22, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_22, 0, &Obj.ToF_SNR[33]},
  {0x23, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_23, 0, &Obj.ToF_SNR[34]},
  {0x24, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_24, 0, &Obj.ToF_SNR[35]},
  {0x25, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_25, 0, &Obj.ToF_SNR[36]},
  {0x26, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_26, 0, &Obj.ToF_SNR[37]},
  {0x27, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_27, 0, &Obj.ToF_SNR[38]},
  {0x28, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_28, 0, &Obj.ToF_SNR[39]},
  {0x29, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_29, 0, &Obj.ToF_SNR[40]},
  {0x2A, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_2A, 0, &Obj.ToF_SNR[41]},
  {0x2B, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_2B, 0, &Obj.ToF_SNR[42]},
  {0x2C, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_2C, 0, &Obj.ToF_SNR[43]},
  {0x2D, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_2D, 0, &Obj.ToF_SNR[44]},
  {0x2E, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_2E, 0, &Obj.ToF_SNR[45]},
  {0x2F, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_2F, 0, &Obj.ToF_SNR[46]},
  {0x30, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_30, 0, &Obj.ToF_SNR[47]},
  {0x31, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_31, 0, &Obj.ToF_SNR[48]},
  {0x32, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_32, 0, &Obj.ToF_SNR[49]},
  {0x33, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_33, 0, &Obj.ToF_SNR[50]},
  {0x34, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_34, 0, &Obj.ToF_SNR[51]},
  {0x35, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_35, 0, &Obj.ToF_SNR[52]},
  {0x36, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_36, 0, &Obj.ToF_SNR[53]},
  {0x37, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_37, 0, &Obj.ToF_SNR[54]},
  {0x38, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_38, 0, &Obj.ToF_SNR[55]},
  {0x39, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_39, 0, &Obj.ToF_SNR[56]},
  {0x3A, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_3A, 0, &Obj.ToF_SNR[57]},
  {0x3B, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_3B, 0, &Obj.ToF_SNR[58]},
  {0x3C, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_3C, 0, &Obj.ToF_SNR[59]},
  {0x3D, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_3D, 0, &Obj.ToF_SNR[60]},
  {0x3E, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_3E, 0, &Obj.ToF_SNR[61]},
  {0x3F, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_3F, 0, &Obj.ToF_SNR[62]},
  {0x40, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName60A1_40, 0, &Obj.ToF_SNR[63]},
};

const _objd SDO60B0[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName60B0_00, 8, NULL},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_TXPDO, acName60B0_01, 0, &Obj.LDC_Frequency[0]},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_TXPDO, acName60B0_02, 0, &Obj.LDC_Frequency[1]},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_TXPDO, acName60B0_03, 0, &Obj.LDC_Frequency[2]},
  {0x04, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_TXPDO, acName60B0_04, 0, &Obj.LDC_Frequency[3]},
  {0x05, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_TXPDO, acName60B0_05, 0, &Obj.LDC_RP[0]},
  {0x06, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_TXPDO, acName60B0_06, 0, &Obj.LDC_RP[1]},
  {0x07, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_TXPDO, acName60B0_07, 0, &Obj.LDC_RP[2]},
  {0x08, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_TXPDO, acName60B0_08, 0, &Obj.LDC_RP[3]},
};

const _objd SDO60C0[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName60C0_00, 12, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60C0_01, 0, &Obj.HALL_Mag_X[0]},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60C0_02, 0, &Obj.HALL_Mag_X[1]},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60C0_03, 0, &Obj.HALL_Mag_X[2]},
  {0x04, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60C0_04, 0, &Obj.HALL_Mag_X[3]},
  {0x05, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60C0_05, 0, &Obj.HALL_Mag_Y[0]},
  {0x06, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60C0_06, 0, &Obj.HALL_Mag_Y[1]},
  {0x07, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60C0_07, 0, &Obj.HALL_Mag_Y[2]},
  {0x08, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60C0_08, 0, &Obj.HALL_Mag_Y[3]},
  {0x09, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60C0_09, 0, &Obj.HALL_Mag_Z[0]},
  {0x0A, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60C0_0A, 0, &Obj.HALL_Mag_Z[1]},
  {0x0B, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60C0_0B, 0, &Obj.HALL_Mag_Z[2]},
  {0x0C, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName60C0_0C, 0, &Obj.HALL_Mag_Z[3]},
};

const _objd SDO60D0[] =
{
  {0x00, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_TXPDO, acName60D0, 0, &Obj.Capacitor_Voltage},
};

const _objd SDO7000[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_RXPDO, acName7000, 0, &Obj.Magnet_Command},
};

const _objectlist SDOobjects[] =
{
  {0x1000, OTYPE_VAR, 0, 0, acName1000, SDO1000},
  {0x1008, OTYPE_VAR, 0, 0, acName1008, SDO1008},
  {0x1009, OTYPE_VAR, 0, 0, acName1009, SDO1009},
  {0x100A, OTYPE_VAR, 0, 0, acName100A, SDO100A},
  {0x1018, OTYPE_RECORD, 4, 0, acName1018, SDO1018},
  {0x1600, OTYPE_RECORD, 3, 0, acName1600, SDO1600},
  {0x1A00, OTYPE_RECORD, 5, 0, acName1A00, SDO1A00},
  {0x1A01, OTYPE_RECORD, 10, 0, acName1A01, SDO1A01},
  {0x1A02, OTYPE_RECORD, 64, 0, acName1A02, SDO1A02},
  {0x1A03, OTYPE_RECORD, 64, 0, acName1A03, SDO1A03},
  {0x1A04, OTYPE_RECORD, 8, 0, acName1A04, SDO1A04},
  {0x1A05, OTYPE_RECORD, 12, 0, acName1A05, SDO1A05},
  {0x1A06, OTYPE_RECORD, 1, 0, acName1A06, SDO1A06},
  {0x1C00, OTYPE_ARRAY, 4, 0, acName1C00, SDO1C00},
  {0x1C12, OTYPE_ARRAY, 1, 0, acName1C12, SDO1C12},
  {0x1C13, OTYPE_ARRAY, 7, 0, acName1C13, SDO1C13},
  {0x2000, OTYPE_VAR, 0, 0, acName2000, SDO2000},
  {0x2001, OTYPE_VAR, 0, 0, acName2001, SDO2001},
  {0x2040, OTYPE_ARRAY, 1, 0, acName2040, SDO2040},
  {0x2100, OTYPE_RECORD, 2, 0, acName2100, SDO2100},
  {0x2101, OTYPE_ARRAY, 250, 0, acName2101, SDO2101},
  {0x603F, OTYPE_VAR, 0, 0, acName603F, SDO603F},
  {0x6040, OTYPE_VAR, 0, 0, acName6040, SDO6040},
  {0x6041, OTYPE_VAR, 0, 0, acName6041, SDO6041},
  {0x6060, OTYPE_VAR, 0, 0, acName6060, SDO6060},
  {0x6061, OTYPE_VAR, 0, 0, acName6061, SDO6061},
  {0x6090, OTYPE_RECORD, 10, 0, acName6090, SDO6090},
  {0x60A0, OTYPE_ARRAY, 64, 0, acName60A0, SDO60A0},
  {0x60A1, OTYPE_ARRAY, 64, 0, acName60A1, SDO60A1},
  {0x60B0, OTYPE_RECORD, 8, 0, acName60B0, SDO60B0},
  {0x60C0, OTYPE_RECORD, 12, 0, acName60C0, SDO60C0},
  {0x60D0, OTYPE_VAR, 0, 0, acName60D0, SDO60D0},
  {0x7000, OTYPE_VAR, 0, 0, acName7000, SDO7000},
  {0xFFFF, 0xFF, 0xFF, 0xFF, NULL, NULL}
};