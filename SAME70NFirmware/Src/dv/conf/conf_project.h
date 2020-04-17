/**
 * \file
 *
 * \brief Example general configuration.
 *
 * Copyright (c) 2019 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef CONF_PROJECT_H_INCLUDED
#define CONF_PROJECT_H_INCLUDED

/* Enable setting Tone Mask (Static Notching) in order to not use all band carriers *********************************************************************/
/* If the following line is uncommented, ATPL360_REG_TONE_MASK is set at initialization with value defined in TONE_MASK_STATIC_NOTCHING *****************/
/* #define CONF_TONEMASK_STATIC_NOTCHING */

/* Configure Coupling Parameters for PL360 device *******************************************************************************************************/
/* If the following line is uncommented, Coupling parameters are set at initialization with the values defined below ************************************/
/* #define CONF_COUP_PARAMETERS */

/* PL360G55CF_EK uses PLCOUP011_v1 by default. For this project, CENELEC-A and FCC binaries are linked, so Multiband is supported ***********************/
#define CONF_ENABLE_MULTIBAND
#define CONF_ENABLE_C11_CFG

#ifdef CONF_TONEMASK_STATIC_NOTCHING
/* Each carrier corresponding to the band can be notched (no energy is sent in those carriers) */
/* Each carrier is represented by one byte (0: carrier used; 1: carrier notched). By default it is all 0's in PL360 device */
/* The length is the number of carriers corresponding to the band in use (see "general_defs.h"). In this case 36 (CENELEC-A) */
/* The same Tone Mask must be set in both transmitter and receiver. Otherwise they don't understand each other */
	#define TONE_MASK_STATIC_NOTCHING     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#endif

#ifdef CONF_COUP_PARAMETERS
/* CENELEC-A: Recommended to use PLCOUP007_v2/PLCOUP008_v2/PLCOUP011_v1 coupling board (Single Branch) */
/* The following values are the default values, calibrated for MCHP PLCOUP007_v2/PLCOUP011_v1 reference designs */
/* For other Hardware designs, it may be needed to calibrate and obtain your own values. MCHP PHY Calibration Tool should be used */

/* ATPL360_REG_NUM_TX_LEVELS: Number of TX levels */
/* Number of Tx attenuation levels (3 dB steps) for normal behaviour. Next levels use always LO mode. Maximum values is 8 */
	#define NUM_TX_LEVELS                 8

/* ATPL360_REG_MAX_RMS_TABLE_HI: Target RMS_CALC value in HI mode when dynamic gain is enabled (ATPL360_REG_CFG_AUTODETECT_IMPEDANCE = 1 or 2) */
/* Each value (up to 8) corresponds to the first attenuation levels (3 dB steps) */
	#define MAX_RMS_HI_TABLE              {1991, 1381, 976, 695, 495, 351, 250, 179}

/* ATPL360_REG_MAX_RMS_TABLE_VLO: Target RMS_CALC value in VLO mode when dynamic gain is enabled (ATPL360_REG_CFG_AUTODETECT_IMPEDANCE = 1 or 2) */
/* Each value (up to 8) corresponds to the first attenuation levels (3 dB steps) */
	#define MAX_RMS_VLO_TABLE             {6356, 4706, 3317, 2308, 1602, 1112, 778, 546}

/* ATPL360_REG_THRESHOLDS_TABLE_HI: Thresholds to change impedance mode (ATPL360_REG_CFG_AUTODETECT_IMPEDANCE = 1) from HI mode */
/* First 8 values (one per Tx level) are thresholds to change from HI to LO (0 to disable) */
/* Next 8 values (one per Tx level) are thresholds to change from HI to VLO. When RMS_CALC is below threshold, impedance mode changes to VLO */
	#define TH1_HI_TABLE                  {0, 0, 0, 0, 0, 0, 0, 0}
	#define TH2_HI_TABLE                  {1685, 1173, 828, 589, 419, 298, 212, 151}

/* ATPL360_REG_THRESHOLDS_TABLE_VLO: Thresholds to change impedance mode (ATPL360_REG_CFG_AUTODETECT_IMPEDANCE = 1) from VLO mode */
/* First 8 values (one per Tx level) are thresholds to change from VLO to LO (0 to disable) */
/* Next 8 values (one per Tx level) are thresholds to change from VLO to HI. When RMS_CALC is above threshold, impedance mode changes to HI */
	#define TH1_VLO_TABLE                 {0, 0, 0, 0, 0, 0, 0, 0}
	#define TH2_VLO_TABLE                 {8988, 6370, 4466, 3119, 2171, 1512, 1061, 752}

/* ATPL360_REG_PREDIST_COEF_TABLE_HI: Equalization values for HI mode */
/* Specific gain for each carrier to equalize transmission and compensate HW filter frequency response */
	#define PREDIST_COEF_HI_TABLE         {0x670A, 0x660F, 0x676A, 0x6A6B, 0x6F3F, 0x7440, 0x74ED, 0x7792, 0x762D, 0x7530, 0x7938, 0x7C0A, 0x7C2A, 0x7B0E, 0x7AF2, 0x784B, 0x7899, 0x76F9, 0x76D6, 0x769F, 0x775D, 0x70C0, 0x6EB9, 0x6F18, 0x6F1E, \
					       0x6FA2, 0x6862, 0x67C9, 0x68F9, 0x68A5, 0x6CA3, 0x7153, 0x7533, 0x750B, 0x7B59, 0x7FFF}

/* ATPL360_REG_PREDIST_COEF_TABLE_VLO: Equalization values for VLO mode */
/* Specific gain for each carrier to equalize transmission and compensate HW filter frequency response */
	#define PREDIST_COEF_VLO_TABLE        {0x7FFF, 0x7DB1, 0x7CE6, 0x7B36, 0x772F, 0x7472, 0x70AA, 0x6BC2, 0x682D, 0x6618, 0x6384, 0x6210, 0x61D7, 0x6244, 0x6269, 0x63A8, 0x6528, 0x65CC, 0x67F6, 0x693B, 0x6B13, 0x6C29, 0x6D43, 0x6E26, 0x6D70, \
					       0x6C94, 0x6BB5, 0x6AC9, 0x6A5F, 0x6B65, 0x6B8C, 0x6A62, 0x6CEC, 0x6D5A, 0x6F9D, 0x6FD3}

/* ATPL360_REG_GAIN_TABLE_HI: Gain values for HI mode. Initial, minimum and maximum */
/* MIN and MAX values are used when dynamic gain is enabled (ATPL360_REG_CFG_AUTODETECT_IMPEDANCE = 1 or 2) */
	#define IFFT_GAIN_HI_INI              142
	#define IFFT_GAIN_HI_MIN              70
	#define IFFT_GAIN_HI_MAX              336

/* ATPL360_REG_GAIN_TABLE_VLO: Gain values for VLO mode. Initial, minimum and maximum */
/* MIN and MAX values are used when dynamic gain is enabled (ATPL360_REG_CFG_AUTODETECT_IMPEDANCE = 1 or 2) */
	#define IFFT_GAIN_VLO_INI             474
	#define IFFT_GAIN_VLO_MIN             230
	#define IFFT_GAIN_VLO_MAX             597

/* ATPL360_REG_DACC_TABLE_CFG: DAC Table configuration. Configuration for PLCOUP007/PLCOUP008/PLCOUP011 (only use Branch 0) */
	#define DACC_CFG_TABLE                {0x00000000, 0x00002120, 0x0000073F, 0x00003F3F, 0x00000333, 0x00000000, 0x610800FF, 0x14141414, \
					       0x00002020, 0x00000044, 0x0FD20004, 0x00000355, 0x0F000000, 0x001020F0, 0x00000355, 0x0F000000, 0x001020FF}

/* FCC: Recommended to use PLCOUP006_v2 (Double Branch) / PLCOUP011_v1 (Single Branch) coupling board */
/* The following values are the default values, calibrated for MCHP PLCOUP006_v2 reference design */
/* For other Hardware designs, it may be needed to calibrate and obtain your own values. MCHP PHY Calibration Tool should be used */

/* ATPL360_REG_NUM_TX_LEVELS: Number of TX levels */
/* Number of Tx attenuation levels (3 dB steps) for normal behaviour. Next levels use always LO mode. Maximum values is 8 */
	#define NUM_TX_LEVELS_FCC                 8

/* ATPL360_REG_MAX_RMS_TABLE_HI: Target RMS_CALC value in HI mode when dynamic gain is enabled (ATPL360_REG_CFG_AUTODETECT_IMPEDANCE = 1 or 2) */
/* Each value (up to 8) corresponds to the first attenuation levels (3 dB steps) */
	#define MAX_RMS_HI_TABLE_FCC              {1355, 960, 681, 485, 345, 246, 177, 129}

/* ATPL360_REG_MAX_RMS_TABLE_VLO: Target RMS_CALC value in VLO mode when dynamic gain is enabled (ATPL360_REG_CFG_AUTODETECT_IMPEDANCE = 1 or 2) */
/* Each value (up to 8) corresponds to the first attenuation levels (3 dB steps) */
	#define MAX_RMS_VLO_TABLE_FCC             {5656, 4174, 2877, 1987, 1413, 1020, 736, 532}

/* ATPL360_REG_THRESHOLDS_TABLE_HI: Thresholds to change impedance mode (ATPL360_REG_CFG_AUTODETECT_IMPEDANCE = 1) from HI mode */
/* First 8 values (one per Tx level) are thresholds to change from HI to LO (0 to disable) */
/* Next 8 values (one per Tx level) are thresholds to change from HI to VLO. When RMS_CALC is below threshold, impedance mode changes to VLO */
	#define TH1_HI_TABLE_FCC                  {0, 0, 0, 0, 0, 0, 0, 0}
	#define TH2_HI_TABLE_FCC                  {1147, 811, 576, 409, 291, 208, 150, 109}

/* ATPL360_REG_THRESHOLDS_TABLE_VLO: Thresholds to change impedance mode (ATPL360_REG_CFG_AUTODETECT_IMPEDANCE = 1) from VLO mode */
/* First 8 values (one per Tx level) are thresholds to change from VLO to LO (0 to disable) */
/* Next 8 values (one per Tx level) are thresholds to change from VLO to HI. When RMS_CALC is above threshold, impedance mode changes to HI */
	#define TH1_VLO_TABLE_FCC                 {0, 0, 0, 0, 0, 0, 0, 0}
	#define TH2_VLO_TABLE_FCC                 {8153, 5718, 4007, 2871, 2080, 1506, 1083, 778}

/* ATPL360_REG_PREDIST_COEF_TABLE_HI: Equalization values for HI mode */
/* Specific gain for each carrier to equalize transmission and compensate HW filter frequency response */
	#define PREDIST_COEF_HI_TABLE_FCC         {0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
						   0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
						   0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF}

/* ATPL360_REG_PREDIST_COEF_TABLE_VLO: Equalization values for VLO mode */
/* Specific gain for each carrier to equalize transmission and compensate HW filter frequency response */
	#define PREDIST_COEF_VLO_TABLE_FCC        {0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
						   0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
						   0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF}

/* ATPL360_REG_GAIN_TABLE_HI: Gain values for HI mode. Initial, minimum and maximum */
/* MIN and MAX values are used when dynamic gain is enabled (ATPL360_REG_CFG_AUTODETECT_IMPEDANCE = 1 or 2) */
	#define IFFT_GAIN_HI_INI_FCC              109
	#define IFFT_GAIN_HI_MIN_FCC              50
	#define IFFT_GAIN_HI_MAX_FCC              256

/* ATPL360_REG_GAIN_TABLE_VLO: Gain values for VLO mode. Initial, minimum and maximum */
/* MIN and MAX values are used when dynamic gain is enabled (ATPL360_REG_CFG_AUTODETECT_IMPEDANCE = 1 or 2) */
	#define IFFT_GAIN_VLO_INI_FCC             364
	#define IFFT_GAIN_VLO_MIN_FCC             180
	#define IFFT_GAIN_VLO_MAX_FCC             408

/* ATPL360_REG_DACC_TABLE_CFG: DAC Table configuration. Configuration for PLCOUP011 (only use Branch 1) */
	#define DACC_CFG_TABLE_FCC                {0x00000000, 0x21202120, 0x073F073F, 0x3F3F3F3F, 0x00000FFF, 0x00000000, 0x2A3000FF, 0x1B1B1B1B, \
						   0x10101010, 0x00001111, 0x04380006, 0x000003AA, 0xF0000000, 0x001020FF, 0x000003AA, 0xF0000000, 0x001020FF}
#endif
/*************************************************************************************************************************************************/

#endif /* CONF_PROJECT_H_INCLUDED */
