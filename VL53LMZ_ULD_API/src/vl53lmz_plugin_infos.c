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

#include "vl53lmz_plugin_infos.h"


/**
 * @brief All union below come from the Bare Driver. In a future release, they
 * will be simplified.
 */
union fgc_0_u {
	uint32_t bytes;
	struct {
		uint32_t fgc_4__6_3 : 4;
		uint32_t fgc_3 : 7;
		uint32_t fgc_2 : 7;
		uint32_t fgc_1 : 7;
		uint32_t fgc_0 : 7;
	};
};
union fgc_1_u {
	uint32_t bytes;
	struct {
		uint32_t fgc_9__6 : 1;
		uint32_t fgc_8 : 7;
		uint32_t fgc_7 : 7;
		uint32_t fgc_6 : 7;
		uint32_t fgc_5 : 7;
		uint32_t fgc_4__2_0 : 3;
	};
};
union fgc_2_u {
	uint32_t bytes;
	struct {
		uint32_t fgc_13__6_2 : 5;
		uint32_t fgc_12 : 7;
		uint32_t fgc_11 : 7;
		uint32_t fgc_10 : 7;
		uint32_t fgc_9__5_0 : 6;
	};
};
union fgc_3_u {
	uint32_t bytes;
	struct {
		uint32_t word32_250__pad_0 : 2;
		uint32_t fgc_17 : 7;
		uint32_t fgc_16 : 7;
		uint32_t fgc_15 : 7;
		uint32_t fgc_14 : 7;
		uint32_t fgc_13__1_0 : 2;
	};
};
union identification_0_u {
	uint32_t bytes;
	struct {
		uint32_t module_date_phase : 3;
		uint32_t day : 5;
		uint32_t month : 4;
		uint32_t year : 4;
		uint32_t map_minor : 5;
		uint32_t map_major : 3;
		uint32_t test_prog_fmt_minor : 5;
		uint32_t test_mrog_fmt_major : 3;
	};
};
union identification_1_u {
	uint32_t bytes;
	struct {
		uint32_t code_site_id_fmt : 8;
		uint32_t code_tester_id_fmt : 8;
		uint32_t time : 16;
	};
};

struct dci_grp__fmt_traceability_t
{
	union fgc_0_u fgc_0_u;
	union fgc_1_u fgc_1_u;
	union fgc_2_u fgc_2_u;
	union fgc_3_u fgc_3_u;
	union identification_0_u identification_0_u;
	union identification_1_u identification_1_u;
};


struct dci_grp__map_version_t {
	uint16_t map__major;
	uint16_t map__minor;
};


struct dci_grp__fw_version_t {
	uint16_t fw__revision;
	uint16_t fw__build;
	uint16_t fw__minor;
	uint16_t fw__major;
};


uint8_t vl53lmz_check_sensor(
		VL53LMZ_Configuration 		 *p_dev,
		uint8_t 				 	 *p_device_id,
		uint8_t 				 	 *p_revision_id)
{
	uint8_t status = VL53LMZ_STATUS_OK;

	/* Device id is 0xF0 and rev_id is 0x2 (for cut 1.2) */
	status += WrByte(&(p_dev->platform), 0x7fff, 0x00);
	status += RdByte(&(p_dev->platform), 0, p_device_id);
	status += RdByte(&(p_dev->platform), 1, p_revision_id);
	status += WrByte(&(p_dev->platform), 0x7fff, 0x02);

	return status;
}

uint8_t vl53lmz_get_module_info(
		VL53LMZ_Configuration 		 *p_dev,
		VL53LMZ_ModuleInfo			 *p_module_infos)
{
	uint8_t status = VL53LMZ_STATUS_OK;
	uint32_t tmp;
	struct dci_grp__fmt_traceability_t fmt_data;

	status = vl53lmz_dci_read_data(p_dev, (uint8_t*)&fmt_data, 0x541C, sizeof(fmt_data));

	tmp = ((fmt_data.fgc_0_u.fgc_4__6_3 & 0x0f) << 3)
			| (fmt_data.fgc_1_u.fgc_4__2_0 & 0x07);
	p_module_infos->fgc[4] = tmp + 32;
	tmp = ((fmt_data.fgc_1_u.fgc_9__6 & 0x01) << 6)
			| (fmt_data.fgc_2_u.fgc_9__5_0 & 0x3f);
	p_module_infos->fgc[9] = tmp + 32;
	tmp =((fmt_data.fgc_2_u.fgc_13__6_2 & 0x1f) << 2)
			| (fmt_data.fgc_3_u.fgc_13__1_0 & 0x03);
	p_module_infos->fgc[13] = tmp + 32;

	p_module_infos->fgc[0] = fmt_data.fgc_0_u.fgc_0 + 32;
	p_module_infos->fgc[1] = fmt_data.fgc_0_u.fgc_1 + 32;
	p_module_infos->fgc[2] = fmt_data.fgc_0_u.fgc_2 + 32;
	p_module_infos->fgc[3] = fmt_data.fgc_0_u.fgc_3 + 32;
	p_module_infos->fgc[5] = fmt_data.fgc_1_u.fgc_5 + 32;
	p_module_infos->fgc[6] = fmt_data.fgc_1_u.fgc_6 + 32;
	p_module_infos->fgc[7] = fmt_data.fgc_1_u.fgc_7 + 32;
	p_module_infos->fgc[8] = fmt_data.fgc_1_u.fgc_8 + 32;
	p_module_infos->fgc[10] = fmt_data.fgc_2_u.fgc_10 + 32;
	p_module_infos->fgc[11] = fmt_data.fgc_2_u.fgc_11 + 32;
	p_module_infos->fgc[12] = fmt_data.fgc_2_u.fgc_12 + 32;
	p_module_infos->fgc[14] = fmt_data.fgc_3_u.fgc_14 + 32;
	p_module_infos->fgc[15] = fmt_data.fgc_3_u.fgc_15 + 32;
	p_module_infos->fgc[16] = fmt_data.fgc_3_u.fgc_16 + 32;
	p_module_infos->fgc[17] = fmt_data.fgc_3_u.fgc_17 + 32;
	p_module_infos->fgc[18]= '\0';

	// Don't know what meaing these have for VL53L5
	p_module_infos->module_id_hi =  0;
	p_module_infos->module_id_lo =  0;

	return status;
}


uint8_t vl53lmz_get_fw_version(
		VL53LMZ_Configuration 		 *p_dev,
		VL53LMZ_FWVersion			 *p_fw_version)
{
	uint8_t status = VL53LMZ_STATUS_OK;
	struct dci_grp__map_version_t map_ver;
	struct dci_grp__fw_version_t fw_ver;

	status = vl53lmz_dci_read_data(p_dev, (uint8_t*)&map_ver, 0x5400, sizeof(map_ver));
	status += vl53lmz_dci_read_data(p_dev, (uint8_t*)&fw_ver, 0x5404, sizeof(fw_ver));

	p_fw_version->map_major = map_ver.map__major;
	p_fw_version->map_minor = map_ver.map__minor;
	p_fw_version->fw_revision = fw_ver.fw__revision;
	p_fw_version->fw_build = fw_ver.fw__build;
	p_fw_version->fw_minor = fw_ver.fw__minor;
	p_fw_version->fw_major = fw_ver.fw__major;

	return status;
}


