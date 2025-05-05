/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef VL53LMZ_PLUGIN_INFOS_H_
#define VL53LMZ_PLUGIN_INFOS_H_

#include "vl53lmz_api.h"


/**
 * @brief Structure VL53LMZ_FWVersion is used to store the Firmware version of
 * VL53L5.
 */
typedef struct
{
	/* Map version */
	uint16_t					map_major;
	uint16_t					map_minor;

	/* Firmware version */
	uint16_t					fw_major;
	uint16_t					fw_minor;
	uint16_t					fw_build;
	uint16_t					fw_revision;

} VL53LMZ_FWVersion;


/**
 * @brief Structure VL53LMZ_ModuleInfo is used to get the unique id of the
 * sensor.
 */
typedef struct
{
	/* Unique string of 18 bytes (contains wafer id, lot nb, etc... ) */
	uint8_t fgc[19];

	/* Module id */
	uint32_t module_id_lo;
	uint32_t module_id_hi;

} VL53LMZ_ModuleInfo;


struct range_meta_data_t
{
	/**
	 * - format = unsigned 28.4
	 * - units = ms
	*/
	uint32_t	time_stamp;
	uint8_t		device_status;			// 8.0
	uint8_t		transaction_id;			// 8.0
	uint8_t		buffer_id;				// 8.0
	/**
	 * - max_value = 255
	 * - min_value = 0
	 * - format = unsigned 8.0
	*/
	uint8_t		streamcount;
	/**
	 * - max_value = 125
	 * - min_value = -40
	 * - units = degreeC
	*/
	int8_t		silicon_temp_degc;		// 8.0
	uint8_t		pad_0;					// 8.0
	uint8_t		pad_1;					// 8.0
	uint8_t		pad_2;					// 8.0
};

struct range_common_data_t
{
	/**
	 * - max_value = 16383
	 * - min_value = 0
	 * - format = unsigned 14.0
	 * - units = mm
	 * - resolution = 1.0000
	*/
	uint16_t	wrap_dmax_mm;
	uint8_t		nb_zones;				// 8.0
	uint8_t		max_targets_per_zone;	// 8.0
};



/**
 * @brief This function reads the device id and the revision of the sensor.
 * This API is compatible with device_id = 0xF0 and rev_id = 0x2 (L5 Cut 1.2).
 * The function can be used at any time, before or after calling vl53lmz_init()
 * function.
 * @param (VL53LMZ_Configuration) *p_dev : VL53L5 configuration structure.
 * @param (uint8_t) *p_device_id : VL53L5 device id (0xF0)
 * @param (uint8_t) *p_revision_id : VL53L5 revision id (0x2).
 * @return (uint8_t) status : 0 if OK
 */
uint8_t vl53lmz_check_sensor(
		VL53LMZ_Configuration		 *p_dev,
		uint8_t						 *p_device_id,
		uint8_t						 *p_revision_id);


/**
 * @brief This function reads the unique infos of the module (module id + 18
 * bytes of unique data). It directly reads the information loaded into the
 * sensor, so the function must be used after the init() function (FW loaded).
 * @param (VL53LMZ_Configuration) *p_dev : VL53L5 configuration structure.
 * @param (VL53LMZ_ModuleInfo) *p_module_infos : VL53L5 module infos.
 * @return (uint8_t) status : 0 if OK
 */
uint8_t vl53lmz_get_module_info(
		VL53LMZ_Configuration		 *p_dev,
		VL53LMZ_ModuleInfo			 *p_module_infos);


/**
 * @brief This function allows getting the map and firmware version. It directly
 * reads the information loaded into the sensor, so the function must be used
 * after the init() function (FW loaded).
 * @param (VL53LMZ_Configuration) *p_dev : VL53L5 configuration structure.
 * @param (VL53LMZ_FWVersion) *p_fw_version : VL53L5 FW information.
 * @return (uint8_t) status : 0 if OK
 */
uint8_t vl53lmz_get_fw_version(
		VL53LMZ_Configuration		 *p_dev,
		VL53LMZ_FWVersion			 *p_fw_version);



#endif /* VL53LMZ_PLUGIN_INFOS_H_ */
