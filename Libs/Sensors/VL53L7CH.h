#pragma once
#include "main.h"
#include "vl53lmz_api.h"
#include "vl53lmz_plugin_infos.h"

class VL53L7CH
{
private:
    VL53LMZ_Configuration config;
    VL53LMZ_ResultsData results;
    VL53LMZ_ModuleInfo module_info;
    VL53LMZ_FWVersion fw_version;

    uint8_t device_id = 0;
    uint8_t revision_id = 0;
    float is_ranging = false;
    bool first_start = true;
public:
    int data[8][8] = {0};

    VL53L7CH();
    int init();
    int start_ranging();
    int stop_ranging();
    int get_ranging_data();

};
