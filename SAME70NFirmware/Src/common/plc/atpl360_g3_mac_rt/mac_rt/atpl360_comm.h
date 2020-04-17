/**
 * \file
 *
 * \brief atpl360_host G3 Physical layer
 *
 * Copyright (c) 2018 Atmel Corporation. All rights reserved.
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

#ifndef ATPL360_COMM_H_INCLUDED
#define ATPL360_COMM_H_INCLUDED

#include "conf_atpl360.h"
#include "MacRt.h"
#include "MacRtDefs.h"

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

#define ATPL360_DATA_MAX_PKT_SIZE                (MAC_RT_MAX_PAYLOAD_SIZE + MAC_RT_FULL_HEADER_SIZE)

/* ! Carriers for Cenelec-A bandplan */
#define NUM_CARRIERS_CENELEC_A                     36
/* ! Carriers for FCC bandplan */
#define NUM_CARRIERS_FCC                           72
/* ! Carriers for ARIB bandplan */
#define NUM_CARRIERS_ARIB                          54
/* ! Carriers for Cenelec-B bandplan */
#define NUM_CARRIERS_CENELEC_B                     16

/* ! Subbands for Cenelec-A bandplan */
#define NUM_SUBBANDS_CENELEC_A                     6
/* ! Subbands for FCC bandplan */
#define NUM_SUBBANDS_FCC                           24
/* ! Subbands for ARIB bandplan */
#define NUM_SUBBANDS_ARIB                          18
/* ! Subbands for Cenelec-B bandplan */
#define NUM_SUBBANDS_CENELEC_B                     4

/* ! Tone Map size for Cenelec bandplan */
#define TONE_MAP_SIZE_CENELEC                      1
/* ! Tone Map size for FCC and ARIB bandplans */
#define TONE_MAP_SIZE_FCC_ARIB                     3
/* ! Maximum number of protocol carriers */
#define PROTOCOL_CARRIERS_MAX                      NUM_CARRIERS_FCC
/* ! Maximum number of tone map */
#define TONE_MAP_SIZE_MAX                          TONE_MAP_SIZE_FCC_ARIB
/* ! Maximum number of subbands */
#define NUM_SUBBANDS_MAX                           NUM_SUBBANDS_FCC

/* ! Number of TX levels to manage */
#define NUM_TX_LEVELS                          8

enum _imp_state {
	IMPEDANCE_STATE_HI,
	IMPEDANCE_STATE_LO,
	IMPEDANCE_STATE_VLO,
	IMPEDANCE_STATE_NUM,
};

/* ! Internal Memory Map */
typedef enum atpl360_mem_id {
	ATPL360_STATUS_INFO_ID = 0,
	ATPL360_SET_COORD_ID,
	ATPL360_SET_SPEC15_ID,
	ATPL360_TONE_MAP_REQ_ID,
	ATPL360_TX_REQ_ID,
	ATPL360_TX_CFM_ID,
	ATPL360_MLME_SET_ID,
	ATPL360_DATA_IND_ID,
	ATPL360_MLME_GET_CFM_ID,
	ATPL360_REG_INFO_ID,
	ATPL360_PHY_SNF_ID
} atpl360_mem_id_t;

/** Table of configuration parameters */
typedef struct {
	/** (ro) Version number */
	uint32_t ul_version;
	/** (ro) Model identifier */
	uint16_t us_model_id;
	/** (ro) Product identifier */
	uint16_t us_product_id;
	/** (ro) Product description */
	char puc_description[10];
} atpl360_id_t;

/* ! \name G3 communication results */
typedef enum atpl360_comm_status {
	ATPL360_COMM_SUCCESS = 0,               /* Current operation successful */
	ATPL360_COMM_ERROR,                     /* Current operation failed */
} atpl360_comm_status_t;

#pragma pack(push,1)

/* ! \name G3 Modulation types */
enum mod_types {
	MOD_TYPE_BPSK = 0,
	MOD_TYPE_QPSK = 1,
	MOD_TYPE_8PSK = 2,
	MOD_TYPE_QAM = 3,
	MOD_TYPE_BPSK_ROBO = 4,
};

/* ! \name G3 Modulation schemes */
enum mod_schemes {
	MOD_SCHEME_DIFFERENTIAL = 0,
	MOD_SCHEME_COHERENT = 1,
};

/* ! \name G3 Structure defining tone map response data */
typedef struct tm_rsp_data {
	enum mod_types uc_mod_type;                    /* Modulation type */
	enum mod_schemes uc_mod_scheme;                /* Modulation scheme */
	uint8_t puc_tone_map[TONE_MAP_SIZE_MAX];       /* Tone Map */
} tm_rsp_data_t;

/* ! \name G3 Structure defining PHY sniffer frame */
typedef struct phy_snf_frm {
	uint8_t uc_snf_cmd_vs;                         /* SNIFFER_IF_PHY_COMMAND_G3_VERSION */
	uint8_t uc_snf_vs;                             /* SNIFFER_VERSION */
	uint8_t uc_snf_dev_vs;                         /* SNIFFER_PL360_G3 */
	uint8_t uc_mod_type_scheme;                    /* ModType (high) + ModScheme (low) */
	uint8_t puc_tone_map[3];                       /* Tone Map */
	uint16_t us_num_symbols;                       /* Number of symbols */
	uint8_t uc_lqi;                                /* LQI */
	uint8_t uc_dt;                                 /* Delimiter Type */
	uint32_t ul_time_ini;                          /* Init time */
	uint32_t ul_time_end;                          /* End time */
	uint16_t us_rssi;                              /* RSSI */
	uint16_t us_agc_factor;                        /* AGC factor */
	uint16_t us_data_len;                          /* Data length */
	uint8_t *puc_data;                             /* Pointer to data message */
} phy_snf_frm_t;

/* ! \name G3 Structure defining events */
typedef struct atpl360_events {
	/* HW Timer reference */
	uint32_t ul_timer_ref;
	/* Info relative to events */
	uint32_t ul_event_info;
	/* Flag to indicate if TX CONFIRMATION MSG event is enable */
	bool b_tx_cfm_event_enable;
	/* Flag to indicate if RX DATA INDICATION event is enable */
	bool b_data_indication_event_enable;
	/* Flag to indicate if RX PARAMETERS INDICATION event is enable */
	bool b_mlme_get_cfm_event_enable;
	/* Flag to indicate if TONE MAP RESPONSE event is enable */
	bool b_tm_rsp_event_enable;
	/* Flag to indicate if REGISTER DATA RESPONSE event is enable */
	bool b_reg_data_enable;
	/* Flag to indicate if PHY Sniffer Frame event is enable */
	bool b_phy_snf_event_enable;
} atpl360_events_t;

#pragma pack(pop)

/* ! Data struct used for Phy Coupling Configuration */
typedef struct TPhyCoupling {
	uint32_t pul_max_rms_hi[NUM_TX_LEVELS];
	uint32_t pul_max_rms_vlo[NUM_TX_LEVELS];
	uint32_t pul_th1_hi[NUM_TX_LEVELS];
	uint32_t pul_th2_hi[NUM_TX_LEVELS];
	uint32_t pul_th1_vlo[NUM_TX_LEVELS];
	uint32_t pul_th2_vlo[NUM_TX_LEVELS];
	uint32_t pul_dacc_cfg[17];
	uint16_t pus_predist_coef_hi[PROTOCOL_CARRIERS_MAX];
	uint16_t pus_predist_coef_vlo[PROTOCOL_CARRIERS_MAX];
	uint16_t pus_ifft_gain_hi[3];
	uint16_t pus_ifft_gain_vlo[3];
	uint8_t uc_num_tx_levels;
} phy_coup_t;

#define ATPL360_EVENT_DATA_LENGTH                8

/* ! FLAG MASKs for set events */
#define ATPL360_TX_CFM_FLAG_MASK                 0x01
#define ATPL360_DATA_IND_FLAG_MASK               0x02
#define ATPL360_MLME_GET_CFM_FLAG_MASK           0x04
#define ATPL360_REG_RSP_MASK                     0x08
#define ATPL360_TM_RSP_FLAG_MASK                 0x10
#define ATPL360_PHY_SNF_FLAG_MASK                0x20

/* ! Event Info MASKs */
#define ATPL360_EV_DAT_LEN_MASK                  0x0000FFFF
#define ATPL360_EV_REG_LEN_MASK                  0xFFFF0000
#define ATPL360_GET_EV_DAT_LEN_INFO(x)           ((uint32_t)x & ATPL360_EV_DAT_LEN_MASK)
#define ATPL360_GET_EV_REG_LEN_INFO(x)           (((uint32_t)x & ATPL360_EV_REG_LEN_MASK) >> 16)

void atpl360_comm_set_event_info(atpl360_events_t *px_events_info, uint16_t us_int_flags);
uint32_t atpl360_comm_get_cfg_param_delay_us(uint16_t us_param_id);

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */

#endif /* ATPL360_COMM_H_INCLUDED */
