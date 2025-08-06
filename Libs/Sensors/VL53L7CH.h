#pragma once
#include "main.h"
#include "vl53lmz_api.h"
#include "vl53lmz_plugin_infos.h"

#define RANGING_FREQUENCY_HZ 60
#define RANGING_RESOLUTION 16U // VL53LMZ_RESOLUTION_4X4 (16U) or VL53LMZ_RESOLUTION_8X8 (64U)
#define OUTLIER_THRESHOLD 300 

class VL53L7CH
{
private:
    static VL53LMZ_Configuration config;
    static VL53LMZ_ResultsData results;
    static VL53LMZ_ModuleInfo module_info;
    static VL53LMZ_FWVersion fw_version;

    uint8_t device_id = 0;
    uint8_t revision_id = 0;
    float is_ranging = false;
    bool first_start = true;
    uint8_t n_outliers[8][8] = {0};
public:
    int data[8][8] = {0};

    VL53L7CH();
    int init();
    int start_ranging();
    int stop_ranging();
    int get_ranging_data();

};
