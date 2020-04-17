/**
 * \file
 *
 * \brief API driver for ATPL360 PLC transceiver.
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

#ifndef ATPL360_H_INCLUDED
#define ATPL360_H_INCLUDED

/* System includes */
#include "compiler.h"
#include "atpl360_comm.h"
#include "atpl360_exception.h"
#include "atpl360_hal_spi.h"
#include "conf_atpl360.h"

/* Mac Rt includes */
#include "MacRt.h"
#include "MacRtDefs.h"
#include "MacRtMib.h"

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

/** ATPL360 Restuls */
typedef enum atpl360_res {
	ATPL360_SUCCESS = 0,
	ATPL360_ERROR,
} atpl360_res_t;

/** Time reference delay */
enum atpl360_delay_mode {
	DELAY_TREF_SEC = 0,
	DELAY_TREF_MS,
	DELAY_TREF_US
};

/* Defines relatives to configuration parameter (G3) */
typedef enum atpl360_phy_id {
	ATPL360_PHY_PRODID = 0,
	ATPL360_PHY_MODEL,
	ATPL360_PHY_VERSION_STR,
	ATPL360_PHY_VERSION_NUM,
	ATPL360_PHY_TONE_MASK,
	ATPL360_PHY_TONE_MAP_RSP_DATA,
	ATPL360_PHY_TX_TOTAL,
	ATPL360_PHY_TX_TOTAL_BYTES,
	ATPL360_PHY_TX_TOTAL_ERRORS,
	ATPL360_PHY_TX_BAD_BUSY_TX,
	ATPL360_PHY_TX_BAD_BUSY_CHANNEL,
	ATPL360_PHY_TX_BAD_LEN,
	ATPL360_PHY_TX_BAD_FORMAT,
	ATPL360_PHY_TX_TIMEOUT,
	ATPL360_PHY_RX_TOTAL,
	ATPL360_PHY_RX_TOTAL_BYTES,
	ATPL360_PHY_RX_RS_ERRORS,
	ATPL360_PHY_RX_EXCEPTIONS,
	ATPL360_PHY_RX_BAD_LEN,
	ATPL360_PHY_RX_BAD_CRC_FCH,
	ATPL360_PHY_RX_FALSE_POSITIVE,
	ATPL360_PHY_RX_BAD_FORMAT,
	ATPL360_PHY_ENABLE_AUTO_NOISE_CAPTURE,
	ATPL360_PHY_TIME_BETWEEN_NOISE_CAPTURES,
	ATPL360_PHY_DELAY_NOISE_CAPTURE_AFTER_RX,
	ATPL360_PHY_RRC_NOTCH_ACTIVE,
	ATPL360_PHY_RRC_NOTCH_INDEX,
	ATPL360_PHY_NOISE_PEAK_POWER,
	ATPL360_PHY_CRC_TX_RX_CAPABILITY,
	ATPL360_PHY_RX_BAD_CRC_PAY,
	ATPL360_PHY_CFG_AUTODETECT_IMPEDANCE,
	ATPL360_PHY_CFG_IMPEDANCE,
	ATPL360_PHY_ZC_PERIOD,
	ATPL360_PHY_FCH_SYMBOLS,
	ATPL360_PHY_PAY_SYMBOLS_TX,
	ATPL360_PHY_PAY_SYMBOLS_RX,
	ATPL360_PHY_RRC_NOTCH_AUTODETECT,
	ATPL360_PHY_MAX_RMS_TABLE_HI,
	ATPL360_PHY_MAX_RMS_TABLE_VLO,
	ATPL360_PHY_THRESHOLDS_TABLE_HI,
	ATPL360_PHY_THRESHOLDS_TABLE_LO,
	ATPL360_PHY_THRESHOLDS_TABLE_VLO,
	ATPL360_PHY_PREDIST_COEF_TABLE_HI,
	ATPL360_PHY_PREDIST_COEF_TABLE_LO,
	ATPL360_PHY_PREDIST_COEF_TABLE_VLO,
	ATPL360_PHY_GAIN_TABLE_HI,
	ATPL360_PHY_GAIN_TABLE_LO,
	ATPL360_PHY_GAIN_TABLE_VLO,
	ATPL360_PHY_DACC_TABLE_CFG,
	ATPL360_PHY_RSV0,
	ATPL360_PHY_NUM_TX_LEVELS,
	ATPL360_PHY_CORRECTED_RMS_CALC,
	ATPL360_PHY_RRC_NOTCH_THR_ON,
	ATPL360_PHY_RRC_NOTCH_THR_OFF,
	ATPL360_PHY_CURRENT_GAIN,
	ATPL360_PHY_ZC_CONF_INV,
	ATPL360_PHY_ZC_CONF_FREQ,
	ATPL360_PHY_ZC_CONF_DELAY,
	ATPL360_PHY_NOISE_PER_CARRIER,
	ATPL360_PHY_SYNC_XCORR_THRESHOLD,
	ATPL360_PHY_SYNC_XCORR_PEAK_VALUE,
	ATPL360_PHY_SYNC_SYNCM_THRESHOLD,
	ATPL360_PHY_TONE_MAP_RSP_ENABLED_MODS,
	ATPL360_PHY_PPM_CALIB_ON,
	ATPL360_PHY_SFO_ESTIMATION_LAST_RX,
	ATPL360_PHY_PDC_LAST_RX,
	ATPL360_PHY_MAX_PSDU_LEN_PARAMS,
	ATPL360_PHY_MAX_PSDU_LEN,
	ATPL360_PHY_END_ID,
} atpl360_phy_id_t;

/* ATPL360 device Callbacks */
typedef struct atpl360_dev_callbacks {
	/* Callback for TX Confirm Event */
	MacRtTxConfirm tx_confirm;
	/* Callback for Data Indication Event */
	MacRtDataIndication data_indication;
	/* Callback for MAC RT LME Get Confirm Event */
	MacRtlmeGetConfirm mlme_get_cfm;
	/* Callback for Exceptions triggered by the component */
	pf_exeption_event_t exception_event;
} atpl360_dev_callbacks_t;

typedef void (*pf_set_callbacks_t)(atpl360_dev_callbacks_t *dev_cb);
typedef atpl360_res_t (*pf_tx_request_t)(uint8_t *pTxSdu, uint16_t u16TxSduLen);
typedef void (*pf_mlme_set_req_t)(struct TMacRtTxParameters *pParameters);
typedef void (*pf_get_tm_rsp_t)(struct TRtToneMapResponseData *pParameters);
typedef void (*pf_void_t)(void);
typedef enum EMacRtStatus (*pf_get_req_t)(enum EMacRtPibAttribute eAttribute, uint16_t u16Index, struct TMacRtPibValue *pValue);
typedef enum EMacRtStatus (*pf_set_req_t)(enum EMacRtPibAttribute eAttribute, uint16_t u16Index, const struct TMacRtPibValue *pValue);
typedef uint32_t (*pf_get_timer_ref_t)(void);

/* ATPL360 descriptor */
typedef struct atpl360_descriptor {
	pf_set_callbacks_t set_callbacks;
	pf_tx_request_t tx_request;
	pf_mlme_set_req_t mlme_set_request;
	pf_get_tm_rsp_t get_tone_map_rsp;
	pf_void_t set_coordinator;
	pf_void_t set_spec15_compliance;
	pf_get_req_t get_req;
	pf_set_req_t set_req;
	pf_get_timer_ref_t get_timer_ref;
} atpl360_descriptor_t;

/* ATPL360 Hardware Wrapper */
typedef void (*pf_plc_init_t)(void);
typedef void (*pf_plc_reset_t)(void);
typedef void (*pf_plc_set_handler_t)(void (*p_handler)(void));
typedef bool (*pf_plc_bootloader_cmd_t)(uint16_t us_cmd, uint32_t ul_addr, uint32_t ul_data_len, uint8_t *puc_data_buf, uint8_t *puc_data_read);
typedef bool (*pf_plc_write_read_cmd_t)(uint8_t uc_cmd, void *px_spi_data, void *px_spi_status_info);
typedef void (*pf_plc_enable_int_t)(bool enable);
typedef void (*pf_plc_delay_t)(uint8_t uc_tref, uint32_t ul_delay);

typedef struct atpl360_hal_wrapper {
	pf_plc_init_t plc_init;
	pf_plc_reset_t plc_reset;
	pf_plc_set_handler_t plc_set_handler;
	pf_plc_bootloader_cmd_t plc_send_boot_cmd;
	pf_plc_write_read_cmd_t plc_write_read_cmd;
	pf_plc_enable_int_t plc_enable_int;
	pf_plc_delay_t plc_delay;
} atpl360_hal_wrapper_t;

/* ! \name ATPL360 Interface */
/* @{ */
void atpl360_init(atpl360_descriptor_t *const descr, atpl360_hal_wrapper_t *px_hal_wrapper);
atpl360_res_t atpl360_enable(uint32_t ul_binary_address, uint32_t ul_binary_len);
void atpl360_disable(void);
void atpl360_handle_events(void);
void atpl360_sniffer_mode_enable(uint8_t *puc_buffer, pf_void_t pf_callback);
void atpl360_sniffer_mode_disable(void);

/* @} */

/* ! @} */

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */

#endif /* ATPL360_H_INCLUDED */
