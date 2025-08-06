
#include "VL53L7CH.h"

VL53LMZ_Configuration VL53L7CH::config __section(".RAM");
VL53LMZ_ResultsData VL53L7CH::results __section(".RAM");
VL53LMZ_ModuleInfo VL53L7CH::module_info __section(".RAM");
VL53LMZ_FWVersion VL53L7CH::fw_version __section(".RAM");

VL53L7CH::VL53L7CH()
{
    this->config.platform.address = 0x52;
}

int VL53L7CH::init()
{
    HAL_GPIO_WritePin(TOF_LP_GPIO_Port, TOF_LP_Pin, GPIO_PIN_SET); // Enable I2C Comms
    HAL_Delay(10);
    HAL_GPIO_WritePin(TOF_I2C_RST_GPIO_Port, TOF_I2C_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(TOF_I2C_RST_GPIO_Port, TOF_I2C_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(TOF_I2C_RST_GPIO_Port, TOF_I2C_RST_Pin, GPIO_PIN_RESET);

    // Initialize the sensor
    int status = vl53lmz_init(&this->config);

    // Read sensor ID and revision
	status |= vl53lmz_check_sensor(&this->config, &this->device_id, &this->revision_id);

    // Read Firmware version
    // status |= vl53lmz_get_fw_version(&this->config, &this->fw_version);

    //Set WAKEUP mode
    status |= vl53lmz_set_power_mode(&this->config, VL53LMZ_POWER_MODE_WAKEUP);

    // Set resolution
    status |= vl53lmz_set_resolution(&this->config, RANGING_RESOLUTION);

    // Set ranging frequency
    status |= vl53lmz_set_ranging_frequency_hz(&this->config, RANGING_FREQUENCY_HZ);

    // Set Integration time
    status |= vl53lmz_set_integration_time_ms(&this->config, 50);

    //set sharpener percentage
    status |= vl53lmz_set_sharpener_percent(&this->config, 5);

    // Set target Order
    status |= vl53lmz_set_target_order(&this->config, VL53LMZ_TARGET_ORDER_CLOSEST);

    // Set ranging mode
    status |= vl53lmz_set_ranging_mode(&this->config, VL53LMZ_RANGING_MODE_CONTINUOUS);

    // Disable internal charge pump
    status |= vl53lmz_disable_internal_cp(&this->config);

    // Disable synch pin
    // status |= vl53lmz_set_external_sync_pin_enable(&this->config, 0);

    // Set Glare filter
    status |= vl53lmz_set_glare_filter_cfg(&this->config, 0, 0); // No glare filter

    // Create output config
    status |= vl53lmz_create_output_config(&this->config);






    return status;
}

int VL53L7CH::start_ranging()
{
    if(this->first_start)
    {
        this->first_start = false;
        // Start ranging
        int status = vl53lmz_send_output_config_and_start(&this->config);
        if (status != 0) return status;
    }
    else
    {
        // Restart ranging
        int status = vl53lmz_start_ranging(&this->config);
        if (status != 0) return status;
    }
    return 0;
}

int VL53L7CH::stop_ranging()
{
    // Stop ranging
    int status = vl53lmz_stop_ranging(&this->config);
    return status;
}

int VL53L7CH::get_ranging_data()
{
    int status = vl53lmz_get_ranging_data(&this->config, &this->results);
    // if (status != 0) return status;
#if RANGING_RESOLUTION == 64U // VL53LMZ_RESOLUTION_8X8
    for(int i = 0; i < 8; i++)
    {
        for(int j = 0; j < 8; j++)
        {
            this->data[i][j] = this->results.distance_mm[i * 8 + j];
        }
    }
#elif RANGING_RESOLUTION == 16U // VL53LMZ_RESOLUTION_4X4
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            //outlier rejection
            if(this->results.distance_mm[i * 4 + j] > this->data[i][j] + OUTLIER_THRESHOLD 
                || this->results.distance_mm[i * 4 + j] < this->data[i][j] - OUTLIER_THRESHOLD)
            {
                continue; // reject outlier
            }
            if(this->results.distance_mm[i * 4 + j] < 0)
            {
                this->data[i][j] = 0;
            }
            else
            {
                this->data[i][j] = this->results.distance_mm[i * 4 + j];
            }
        }
    }
#endif
    return 0;
}
