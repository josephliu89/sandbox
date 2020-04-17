/**
 * \file
 *
 * \brief Management of the ATPL360 PLC transceiver.
 * This file manages the accesses to the ATPL360 component.
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

#include "compiler.h"

/* ATPL360 includes */
#include "conf_atpl360.h"
#include "atpl360_hal_spi.h"
#include "atpl360_boot.h"
#include "atpl360.h"
#include "atpl360_comm.h"

/* HAL wrapper */
atpl360_hal_wrapper_t sx_atpl360_hal_wrapper;

/* Mac Rt includes */
#include "MacRt.h"
#include "MacRtDefs.h"

#define ATPL360_MAX_STATUS_CHECK                  2

#define ATPL360_REG_CMD_RD                        0
#define ATPL360_REG_CMD_WR                        1

#define ATPL360_DATA_PKT_SIZE                     ATPL360_DATA_MAX_PKT_SIZE
#define ATPL360_MLME_SET_PKT_SIZE                 (div8_ceil(sizeof(struct TMacRtTxParameters)) << 3)
#define ATPL360_MLME_GET_PKT_SIZE                 (div8_ceil(sizeof(struct TMacRtRxParameters)) << 3)
#define ATPL360_TX_CMF_PKT_SIZE                   3
#define ATPL360_TONE_MAP_PKT_SIZE                 (div8_ceil(sizeof(struct TRtToneMapResponseData)) << 3)
#define ATPL360_REG_PKT_SIZE                      156
#define ATPL360_PHY_SNF_PKT_SIZE                  (div8_ceil(sizeof(phy_snf_frm_t) + 404) << 3)

/* Buffer definition to communicate with ATPL360 */
static uint8_t spuc_tx_dat_buffer[ATPL360_DATA_PKT_SIZE];
static uint8_t spuc_mlme_set_buffer[ATPL360_MLME_SET_PKT_SIZE];
static uint8_t spuc_data_ind_buffer[ATPL360_DATA_PKT_SIZE];
static uint8_t spuc_mlme_get_buffer[ATPL360_MLME_GET_PKT_SIZE];
static uint8_t spuc_tx_cfm_buffer[ATPL360_TX_CMF_PKT_SIZE];
static uint8_t spuc_tm_buffer[ATPL360_TONE_MAP_PKT_SIZE];
static uint8_t spuc_reg_buffer[ATPL360_REG_PKT_SIZE];

/* Flags to manage events */
static volatile bool sb_tx_cfm_event_enable;
static volatile bool sb_mlme_get_cfm_event_enable;
static volatile bool sb_tm_event_enable;
static volatile bool sb_phy_snf_event_enable;
static volatile uint16_t sus_data_ind_event_enable;
static volatile uint16_t sus_reg_event_len;

/* Device Enable/Disable Status */
static bool sb_component_enabled;

/* Internal Handlers to manage ATPL360 event notifications */
static MacRtTxConfirm _tx_confirm_cb_handler;
static MacRtDataIndication _data_indication_cb_handler;
static MacRtlmeGetConfirm _mlme_get_cfm_cb_handler;
static pf_exeption_event_t _exception_event_cb_handler;

/* Callback for Phy Sniffer mode */
static pf_void_t _phy_sniffer_cb_handler;
static uint8_t *spuc_phy_snf_buff;

/* SPI Commands to get/set spi data buffer */
#define SPI_RD_CMD      0
#define SPI_WR_CMD      1

static uint32_t _get_cfg_param_delay_us(uint16_t us_param_id)
{
	uint32_t ul_delay = 50;

	switch (us_param_id) {
	case ATPL360_PHY_TONE_MASK:
		ul_delay = 600;
		break;

	case ATPL360_PHY_PREDIST_COEF_TABLE_HI:
	case ATPL360_PHY_PREDIST_COEF_TABLE_LO:
		ul_delay = 250;
		break;

	case ATPL360_PHY_PREDIST_COEF_TABLE_VLO:
		ul_delay = 350;
		break;
	}

	return ul_delay;
}

/**
 * \brief Function to read/write through SPI
 *
 * \return  ATPL360_SUCCESS if there is no error, otherwise returns ATPL360_ERROR.
 */
static atpl360_res_t _spi_send_cmd(uint8_t uc_cmd, atpl360_spi_data_t *px_spi_data)
{
	uint8_t uc_cnt;
	atpl360_spi_status_t uc_status;

	if (uc_cmd == SPI_WR_CMD) {
		atpl360_spi_write_buf(px_spi_data);
	} else {
		atpl360_spi_read_buf(px_spi_data);
	}

	uc_cnt = ATPL360_MAX_STATUS_CHECK;
	uc_status = atpl360_spi_get_status();

	while (uc_status != ATPL360_SPI_STATUS_CORTEX) {
		if (uc_status == ATPL360_SPI_STATUS_UNKNOWN) {
			/* Safety condition in loop */
			if (_exception_event_cb_handler) {
				/* Report SPI Error to Application */
				_exception_event_cb_handler(ATPL360_EXCEPTION_UNEXPECTED_SPI_STATUS);
			}

			/* CRITICAL Safety condition in loop */
			if (!uc_cnt--) {
				if (_exception_event_cb_handler) {
					/* Report Critical SPI Error to Application */
					_exception_event_cb_handler(ATPL360_EXCEPTION_SPI_CRITICAL_ERROR);
				}

				uc_cnt = ATPL360_MAX_STATUS_CHECK;
				return ATPL360_ERROR;
			}

			/* Wait to system re-start */
			sx_atpl360_hal_wrapper.plc_delay(DELAY_TREF_MS, ATPL360_RST_WAIT_MS);

			/* ATPL360 hard reset */
			sx_atpl360_hal_wrapper.plc_reset();
		} else if (uc_status == ATPL360_SPI_STATUS_FW_VALIDATING) {
			/* Report Reset detection to Application */
			if (_exception_event_cb_handler) {
				_exception_event_cb_handler(ATPL360_EXCEPTION_RESET);
			}
		}

		/* Validate phase */
		if (uc_cmd == SPI_WR_CMD) {
			atpl360_spi_write_buf(px_spi_data);
		} else {
			atpl360_spi_read_buf(px_spi_data);
		}

		/* Update SPI status */
		uc_status = atpl360_spi_get_status();
	}

	return ATPL360_SUCCESS;
}

/**
 * \brief Function to get Interrupt info
 *
 * \return Interrupt info in case of ATPl360 is enable. 0 in otherwise
 */
static void _get_interrupt_info(atpl360_events_t *px_events_info, atpl360_spi_status_info_t *x_status_info)
{
	atpl360_spi_data_t x_spi_data;
	uint8_t *puc_info;
	uint8_t puc_int_buffer[ATPL360_EVENT_DATA_LENGTH];

	puc_info = puc_int_buffer;

	/* Read Time Ref and Event flags */
	x_spi_data.us_mem_id = ATPL360_STATUS_INFO_ID;
	x_spi_data.us_len = sizeof(puc_int_buffer);
	x_spi_data.puc_data_buf = puc_info;
	if (_spi_send_cmd(SPI_RD_CMD, &x_spi_data) == ATPL360_SUCCESS) {
		/* Extract events info */
		atpl360_spi_get_status_info(x_status_info);
		atpl360_comm_set_event_info(px_events_info, x_status_info->ul_flags);

		/* Extract Timer info */
		px_events_info->ul_timer_ref = ((uint32_t)*puc_info++);
		px_events_info->ul_timer_ref += ((uint32_t)*puc_info++) << 8;
		px_events_info->ul_timer_ref += ((uint32_t)*puc_info++) << 16;
		px_events_info->ul_timer_ref += ((uint32_t)*puc_info++) << 24;

		/* Extract Event info */
		px_events_info->ul_event_info = ((uint32_t)*puc_info++);
		px_events_info->ul_event_info += ((uint32_t)*puc_info++) << 8;
		px_events_info->ul_event_info += ((uint32_t)*puc_info++) << 16;
		px_events_info->ul_event_info += ((uint32_t)*puc_info++) << 24;
	} else {
		/* Handle error in getting events info */
		memset(px_events_info, 0, sizeof(atpl360_events_t));
	}
}

/**
 * \brief External interrupt handler
 */
static void _handler_atpl360_ext_int(void)
{
	atpl360_events_t x_events_info;
	atpl360_spi_data_t x_spi_data;
	atpl360_spi_status_info_t x_spi_status_info;

	if (sb_component_enabled) {
		/* Time guard */
		sx_atpl360_hal_wrapper.plc_delay(DELAY_TREF_US, 20);

		/* capture information relative to ATPL360 events */
		_get_interrupt_info(&x_events_info, &x_spi_status_info);

		/* Check Tx Cfm event */
		if (x_events_info.b_tx_cfm_event_enable) {
			/* Read confirm message */
			x_spi_data.us_mem_id = ATPL360_TX_CFM_ID;
			x_spi_data.puc_data_buf = spuc_tx_cfm_buffer;
			x_spi_data.us_len = sizeof(spuc_tx_cfm_buffer);
			if (_spi_send_cmd(SPI_RD_CMD, &x_spi_data) == ATPL360_SUCCESS) {
				sb_tx_cfm_event_enable = true;
			}
		}

		/* Check Tone Map Response event */
		if (x_events_info.b_tm_rsp_event_enable) {
			/* Read tm response */
			x_spi_data.us_mem_id = ATPL360_TONE_MAP_REQ_ID;
			x_spi_data.puc_data_buf = spuc_tm_buffer;
			x_spi_data.us_len = sizeof(spuc_tm_buffer);
			if (_spi_send_cmd(SPI_RD_CMD, &x_spi_data) == ATPL360_SUCCESS) {
				sb_tm_event_enable = true;
			}
		}

		/* Check MLME Get Confirm event */
		if (x_events_info.b_mlme_get_cfm_event_enable) {
			/* Read PARAMS from indication message */
			x_spi_data.us_mem_id = ATPL360_MLME_GET_CFM_ID;
			x_spi_data.puc_data_buf = spuc_mlme_get_buffer;
			x_spi_data.us_len = sizeof(spuc_mlme_get_buffer);
			if (_spi_send_cmd(SPI_RD_CMD, &x_spi_data) == ATPL360_SUCCESS) {
				sb_mlme_get_cfm_event_enable = true;
			}
		}

		/* Check Data Indication event */
		if (x_events_info.b_data_indication_event_enable) {
			uint16_t ul_data_len;

			/* Read DATA from indication message */
			x_spi_data.us_mem_id = ATPL360_DATA_IND_ID;
			x_spi_data.puc_data_buf = spuc_data_ind_buffer;
			ul_data_len = ATPL360_GET_EV_DAT_LEN_INFO(x_events_info.ul_event_info);
			x_spi_data.us_len = ul_data_len;
			if (ul_data_len > ATPL360_DATA_PKT_SIZE) {
				x_spi_data.us_len = 1;
			}

			if (_spi_send_cmd(SPI_RD_CMD, &x_spi_data) == ATPL360_SUCCESS) {
				sus_data_ind_event_enable = ul_data_len;
			}
		}

		/* Check REG_RSP_EV_TYPE event */
		if (x_events_info.b_reg_data_enable) {
			/* Extract data and pkt len from event info */
			sus_reg_event_len = ATPL360_GET_EV_REG_LEN_INFO(x_events_info.ul_event_info);
			if ((sus_reg_event_len == 0) || (sus_reg_event_len > ATPL360_REG_PKT_SIZE)) {
				sus_reg_event_len = 1;
			}

			x_spi_data.us_mem_id = ATPL360_REG_INFO_ID;
			x_spi_data.puc_data_buf = spuc_reg_buffer;
			x_spi_data.us_len = 8 + sus_reg_event_len;
			_spi_send_cmd(SPI_RD_CMD, &x_spi_data);
		}

		/* Check PHY Sniffer event */
		if (x_events_info.b_phy_snf_event_enable) {
			uint16_t ul_data_len;

			/* Read PHY sniffer frame */
			x_spi_data.us_mem_id = ATPL360_PHY_SNF_ID;
			x_spi_data.puc_data_buf = spuc_phy_snf_buff;
			ul_data_len = ATPL360_GET_EV_REG_LEN_INFO(x_events_info.ul_event_info);
			x_spi_data.us_len = sizeof(phy_snf_frm_t) + ul_data_len;
			if (_spi_send_cmd(SPI_RD_CMD, &x_spi_data) == ATPL360_SUCCESS) {
				sb_phy_snf_event_enable = true;
			}
		}

		/* Time guard */
		sx_atpl360_hal_wrapper.plc_delay(DELAY_TREF_US, 20);
	} else {
		/* Disable EXT INT */
		sx_atpl360_hal_wrapper.plc_enable_int(false);
	}
}

/**
 * \brief Function to get PIB information
 *
 * \param eAttribute   Identification number of configuration parameter
 * \param u16Index     Offset index
 * \param pValue       Pointer to value to write PIB information
 *
 * \return Mac Rt Status
 */
static enum EMacRtStatus _get_req(enum EMacRtPibAttribute eAttribute, uint16_t u16Index, struct TMacRtPibValue *pValue)
{
	uint16_t us_sec_cnt;
	uint16_t us_dummy;

	if (sb_component_enabled) {
		/* set reg value through spi */
		atpl360_spi_data_t x_spi_data;
		uint8_t *puc_buf;

		puc_buf = spuc_reg_buffer;

		/* Build command */
		*puc_buf++ = ATPL360_REG_CMD_RD;
		*puc_buf++ = (uint8_t)(eAttribute >> 24);
		*puc_buf++ = (uint8_t)(eAttribute >> 16);
		*puc_buf++ = (uint8_t)(eAttribute >> 8);
		*puc_buf++ = (uint8_t)(eAttribute);
		*puc_buf++ = (uint8_t)(u16Index >> 8);
		*puc_buf++ = (uint8_t)(u16Index);

		/* Send command */
		x_spi_data.us_len = puc_buf - spuc_reg_buffer;
		x_spi_data.us_mem_id = ATPL360_REG_INFO_ID;
		x_spi_data.puc_data_buf = spuc_reg_buffer;
		_spi_send_cmd(SPI_WR_CMD, &x_spi_data);

		/* Sync function: Wait to response from interrupt */
		us_sec_cnt = 100;
		us_dummy = sus_reg_event_len;
		while (!us_dummy) {
			/* Wait for interrupt. The CPU is in sleep mode until an interrupt occurs. */
			__asm volatile ("WFI");
			if (!us_sec_cnt--) {
				/* Error in get cmd */
				return MAC_RT_STATUS_TRANSACTION_OVERFLOW;
			}

			us_dummy = sus_reg_event_len;
		}

		/* Check Response Content */
		if (spuc_reg_buffer[0] != MAC_RT_STATUS_SUCCESS) {
			/* Not success process */
			return (enum EMacRtStatus)spuc_reg_buffer[0];
		}

		/* copy reg info in data pointer */
		memcpy(pValue->m_au8Value, spuc_reg_buffer + 8, sus_reg_event_len);
		pValue->m_u8Length = sus_reg_event_len;

		/* Reset event flag */
		sus_reg_event_len = 0;

		return MAC_RT_STATUS_SUCCESS;
	} else {
		return MAC_RT_STATUS_DENIED;
	}
}

/**
 * \brief Function to set PIB information
 *
 * \param eAttribute   Identification number of configuration parameter
 * \param u16Index     Offset index
 * \param pValue       Pointer to value to read PIB information
 *
 * \return Mac Rt Status
 */
static enum EMacRtStatus _set_req(enum EMacRtPibAttribute eAttribute, uint16_t u16Index, const struct TMacRtPibValue *pValue)
{
	uint16_t us_sec_cnt;
	uint16_t us_dummy;

	if (sb_component_enabled) {
		/* set reg value through spi */
		atpl360_spi_data_t x_spi_data;
		uint8_t *puc_buf;

		/* Check Length */
		if (pValue->m_u8Length > MAC_RT_PIB_MAX_VALUE_LENGTH) {
			/* len error */
			return MAC_RT_STATUS_INVALID_PARAMETER;
		}

		puc_buf = spuc_reg_buffer;

		/* Build command */
		*puc_buf++ = ATPL360_REG_CMD_WR;
		*puc_buf++ = (uint8_t)(eAttribute >> 24);
		*puc_buf++ = (uint8_t)(eAttribute >> 16);
		*puc_buf++ = (uint8_t)(eAttribute >> 8);
		*puc_buf++ = (uint8_t)(eAttribute);
		*puc_buf++ = (uint8_t)(u16Index >> 8);
		*puc_buf++ = (uint8_t)(u16Index);
		*puc_buf++ = pValue->m_u8Length;
		memcpy(puc_buf, pValue->m_au8Value, pValue->m_u8Length);
		puc_buf += pValue->m_u8Length;

		/* Send command */
		x_spi_data.us_len = puc_buf - spuc_reg_buffer;
		x_spi_data.us_mem_id = ATPL360_REG_INFO_ID;
		x_spi_data.puc_data_buf = spuc_reg_buffer;
		_spi_send_cmd(SPI_WR_CMD, &x_spi_data);

		/* Sync function: Wait to response from interrupt */
		us_sec_cnt = 100;
		us_dummy = sus_reg_event_len;
		while (!us_dummy) {
			/* Wait for interrupt. The CPU is in sleep mode until an interrupt occurs. */
			__asm volatile ("WFI");
			if (!us_sec_cnt--) {
				/* Error in get cmd */
				return MAC_RT_STATUS_TRANSACTION_OVERFLOW;
			}

			us_dummy = sus_reg_event_len;
		}

		/* Reset event flag */
		sus_reg_event_len = 0;

		/* additional delay */
		if (eAttribute == MAC_RT_PIB_MANUF_PHY_PARAM) {
			uint32_t ul_delay;

			ul_delay = _get_cfg_param_delay_us(u16Index);
			sx_atpl360_hal_wrapper.plc_delay(DELAY_TREF_US, ul_delay);
		}

		return (enum EMacRtStatus)spuc_reg_buffer[0];
	} else {
		return MAC_RT_STATUS_DENIED;
	}
}

/**
 * \brief Function to transmit message through ATPL360 device
 *
 * \param pTxSdu         Pointer to data message
 * \param u16TxSduLen    Length of the data in bytes
 *
 * \return ATPL360_SUCCESS if transmission is correctly programmed. ATPL360_ERROR if anycase.
 */
static atpl360_res_t _tx_request(uint8_t *pTxSdu, uint16_t u16TxSduLen)
{
	if (sb_component_enabled) {
		uint8_t *puc_data;
		atpl360_spi_data_t x_spi_data;

		if (u16TxSduLen > MAC_RT_MAX_PAYLOAD_SIZE + MAC_RT_FULL_HEADER_SIZE) {
			return ATPL360_ERROR;
		}

		puc_data = spuc_tx_dat_buffer;

		*puc_data++ = (uint8_t)(u16TxSduLen >> 8);
		*puc_data++ = (uint8_t)u16TxSduLen;

		memcpy(puc_data, pTxSdu, u16TxSduLen);
		puc_data += u16TxSduLen;

		/* Send tx msg */
		x_spi_data.us_len = puc_data - spuc_tx_dat_buffer;
		x_spi_data.us_mem_id = ATPL360_TX_REQ_ID;
		x_spi_data.puc_data_buf = spuc_tx_dat_buffer;
		_spi_send_cmd(SPI_WR_CMD, &x_spi_data);

		return ATPL360_SUCCESS;
	} else {
		return ATPL360_ERROR;
	}
}

/**
 * \brief Function to set parameters of the TX message
 *
 * \param pParameters  Pointer to transmission parameters
 *
 */
static void _mlme_set_request(struct TMacRtTxParameters *pParameters)
{
	if (sb_component_enabled) {
		atpl360_spi_data_t x_spi_data;

		memcpy(spuc_mlme_set_buffer, (uint8_t *)pParameters, sizeof(struct TMacRtTxParameters));

		/* Send tx msg */
		x_spi_data.us_len = sizeof(struct TMacRtTxParameters);
		x_spi_data.us_mem_id = ATPL360_MLME_SET_ID;
		x_spi_data.puc_data_buf = spuc_mlme_set_buffer;
		_spi_send_cmd(SPI_WR_CMD, &x_spi_data);
	}
}

/**
 * \brief Get Tone Map Response
 *
 * \param pParameters       Pointer to write Tone Map Response Data
 */
static void _get_tone_map_rsp(struct TRtToneMapResponseData *pParameters)
{
	atpl360_spi_data_t x_spi_data;
	uint8_t uc_dummy;
	uint8_t us_sec_cnt;

	/* Reset TM flag */
	sb_tm_event_enable = false;

	x_spi_data.us_len = 1;
	x_spi_data.us_mem_id = ATPL360_TONE_MAP_REQ_ID;
	x_spi_data.puc_data_buf = &uc_dummy;

	/* Disable all interrupts */
	cpu_irq_disable();
	/* Send SPI command */
	_spi_send_cmd(SPI_WR_CMD, &x_spi_data);
	/* Re-enable all interrupts */
	cpu_irq_enable();

	/* Wait answer */
	uc_dummy = sb_tm_event_enable;
	us_sec_cnt = 100;
	while (!uc_dummy) {
		/* Wait for interrupt. The CPU is in sleep mode until an interrupt occurs. */
		__asm volatile ("WFI");
		if (!us_sec_cnt--) {
			/* Error in get cmd */
			return;
		}

		uc_dummy = sb_tm_event_enable;
	}

	memcpy(pParameters, spuc_tm_buffer, ATPL360_TONE_MAP_PKT_SIZE);
}

/**
 * \brief Function to get Timer reference from ATPL360 device
 *
 * \return ATPL360 Timer reference in microseconds
 *
 */
static uint32_t _get_timer_ref(void)
{
	uint32_t phy_time;
	atpl360_spi_data_t x_spi_data;

	/* Disable all interrupts */
	cpu_irq_disable();

	x_spi_data.us_len = 4;
	x_spi_data.us_mem_id = ATPL360_STATUS_INFO_ID;
	x_spi_data.puc_data_buf = (uint8_t *)&phy_time;
	/* Send SPI command */
	_spi_send_cmd(SPI_RD_CMD, &x_spi_data);

	/* Re-enable all interrupts */
	cpu_irq_enable();

	return phy_time;
}

/**
 * \brief Function to set ATPL360 device in Coordinator Mode
 *
 */
static void _set_coordinator(void)
{
	atpl360_spi_data_t x_spi_data;
	uint8_t uc_dummy;

	x_spi_data.us_len = 1;
	x_spi_data.us_mem_id = ATPL360_SET_COORD_ID;
	x_spi_data.puc_data_buf = &uc_dummy;

	/* Disable all interrupts */
	cpu_irq_disable();
	/* Send SPI command */
	_spi_send_cmd(SPI_WR_CMD, &x_spi_data);
	/* Re-enable all interrupts */
	cpu_irq_enable();
}

/**
 * \brief Function to set ATPL360 device in Spec 1.5 Compliance
 *
 */
static void _set_spec15_compliance(void)
{
	atpl360_spi_data_t x_spi_data;
	uint8_t uc_value;

	x_spi_data.us_len = 1;
	x_spi_data.us_mem_id = ATPL360_SET_SPEC15_ID;
	uc_value = 15;
	x_spi_data.puc_data_buf = &uc_value;

	/* Disable all interrupts */
	cpu_irq_disable();
	/* Send SPI command */
	_spi_send_cmd(SPI_WR_CMD, &x_spi_data);
	/* Re-enable all interrupts */
	cpu_irq_enable();
}

/**
 * \brief Function to enable/disable PHY sniffer
 *
 */
static void _set_sniffer_mode(bool b_enable)
{
	atpl360_spi_data_t x_spi_data;
	uint8_t uc_value;

	x_spi_data.us_len = 1;
	x_spi_data.us_mem_id = ATPL360_PHY_SNF_ID;
	uc_value = (uint8_t)b_enable;
	x_spi_data.puc_data_buf = &uc_value;

	/* Disable all interrupts */
	cpu_irq_disable();
	/* Send SPI command */
	_spi_send_cmd(SPI_WR_CMD, &x_spi_data);
	/* Re-enable all interrupts */
	cpu_irq_enable();
}

/**
 * \brief Function to set callbacks to call on upper layer
 *
 * \param px_cbs  Pointer to ATPL360 Callback struct
 *
 */
static void _set_callbacks(atpl360_dev_callbacks_t *px_cbs)
{
	_tx_confirm_cb_handler = px_cbs->tx_confirm;
	_data_indication_cb_handler = px_cbs->data_indication;
	_mlme_get_cfm_cb_handler = px_cbs->mlme_get_cfm;
	_exception_event_cb_handler = px_cbs->exception_event;
}

/**
 * \brief Function to init internal system
 *
 */
static void _init_system(void)
{
	/* Init Callback function pointers */
	_tx_confirm_cb_handler = NULL;
	_data_indication_cb_handler = NULL;
	_mlme_get_cfm_cb_handler = NULL;
	_exception_event_cb_handler = NULL;

	/* Init PHY sniffer callback */
	_phy_sniffer_cb_handler = NULL;
	spuc_phy_snf_buff = NULL;

	/* Init internal SPI controller */
	atpl360_spi_initialize();

	/* Initialize HAL SPI */
	sx_atpl360_hal_wrapper.plc_init();
	sx_atpl360_hal_wrapper.plc_reset();

	/* Set ATPl360 handler and enable int */
	sx_atpl360_hal_wrapper.plc_set_handler(_handler_atpl360_ext_int);

	/* Init event indicators */
	sb_tx_cfm_event_enable = false;
	sb_mlme_get_cfm_event_enable = false;
	sb_tm_event_enable = false;
	sb_phy_snf_event_enable = false;
	sus_data_ind_event_enable = 0;
	sus_reg_event_len = 0;

	/* Init internal buffers */
	memset(spuc_tx_dat_buffer, 0, sizeof(spuc_tx_dat_buffer));
	memset(spuc_mlme_set_buffer, 0, sizeof(spuc_mlme_set_buffer));
	memset(spuc_data_ind_buffer, 0, sizeof(spuc_data_ind_buffer));
	memset(spuc_mlme_get_buffer, 0, sizeof(spuc_mlme_get_buffer));
	memset(spuc_tx_cfm_buffer, 0, sizeof(spuc_tx_cfm_buffer));
	memset(spuc_tm_buffer, 0, sizeof(spuc_tm_buffer));
	memset(spuc_reg_buffer, 0, sizeof(spuc_reg_buffer));
}

/**
 * \brief Function to initialize ATPL360 instance
 *
 * \param descr           Pointer to ATPL360 descriptor
 * \param px_hal_wrapper  Pointer to HAL wrapper (hardware abstraction layer functions)
 *
 */
void atpl360_init(atpl360_descriptor_t *const px_descr, atpl360_hal_wrapper_t *px_hal_wrapper)
{
	/* Fill HAL wrapper functions to access hardware peripherals */
	memcpy(&sx_atpl360_hal_wrapper, px_hal_wrapper, sizeof(atpl360_hal_wrapper_t));

	px_descr->get_req = _get_req;
	px_descr->get_timer_ref = _get_timer_ref;
	px_descr->get_tone_map_rsp = _get_tone_map_rsp;
	px_descr->set_callbacks = _set_callbacks;
	px_descr->set_coordinator = _set_coordinator;
	px_descr->set_spec15_compliance = _set_spec15_compliance;
	px_descr->set_req = _set_req;
	px_descr->tx_request = _tx_request;
	px_descr->mlme_set_request = _mlme_set_request;

	/* component must be explicitly enabled */
	atpl360_disable();

	/* init internal vars */
	_init_system();
}

/**
 * \brief Function to enable ATPL360 instance
 *
 * \param ul_binary_address   Address where binary file for ATPL360 is stored in internal flash memory
 * \param ul_binary_len       Size of binary file for ATPl360
 *
 * \return Result of enable operation
 */
atpl360_res_t atpl360_enable(uint32_t ul_binary_address, uint32_t ul_binary_len)
{
	atpl360_spi_data_t x_spi_data;
	uint8_t puc_int_buffer[ATPL360_EVENT_DATA_LENGTH];

	/* Disable EXT INT */
	sx_atpl360_hal_wrapper.plc_enable_int(false);

	atpl360_boot_init(ul_binary_address, ul_binary_len);

	/* Read Time Ref to get SPI status */
	x_spi_data.us_mem_id = ATPL360_STATUS_INFO_ID;
	x_spi_data.us_len = sizeof(puc_int_buffer);
	x_spi_data.puc_data_buf = puc_int_buffer;
	if (_spi_send_cmd(SPI_RD_CMD, &x_spi_data) == ATPL360_ERROR) {
		return ATPL360_ERROR;
	}

	sb_component_enabled = true;

	/* Enable EXT INT */
	sx_atpl360_hal_wrapper.plc_enable_int(true);

	return ATPL360_SUCCESS;
}

void atpl360_disable(void)
{
	sb_component_enabled = false;

	/* Disable EXT INT by default */
	sx_atpl360_hal_wrapper.plc_enable_int(false);

	/* PL360 reset */
	sx_atpl360_hal_wrapper.plc_reset();
}

void atpl360_sniffer_mode_enable(uint8_t *puc_buffer, pf_void_t pf_callback)
{
	_phy_sniffer_cb_handler = pf_callback;
	spuc_phy_snf_buff = puc_buffer;
	_set_sniffer_mode(true);
}

void atpl360_sniffer_mode_disable(void)
{
	_set_sniffer_mode(false);
	_phy_sniffer_cb_handler = NULL;
	spuc_phy_snf_buff = NULL;
}

/**
 * \brief Function to Check ATPL360 pending events
 *
 */
void atpl360_handle_events(void)
{
	if (sb_component_enabled) {
		/* Check Tx Cfm event */
		if (sb_tx_cfm_event_enable) {
			if (_tx_confirm_cb_handler) {
				_tx_confirm_cb_handler((enum EMacRtStatus)spuc_tx_cfm_buffer[0],
						(bool)spuc_tx_cfm_buffer[1],
						(enum ERtModulationType)spuc_tx_cfm_buffer[2]);
			}

			/* Reset event flag */
			sb_tx_cfm_event_enable = false;
		}

		/* Check MLME Get Cfm event */
		if (sb_mlme_get_cfm_event_enable) {
			if (_mlme_get_cfm_cb_handler) {
				_mlme_get_cfm_cb_handler((struct TMacRtRxParameters *)spuc_mlme_get_buffer);
			}

			/* Reset event flag */
			sb_mlme_get_cfm_event_enable = false;
		}

		/* Check Data Indication event */
		if (sus_data_ind_event_enable) {
			if (_data_indication_cb_handler) {
				_data_indication_cb_handler(spuc_data_ind_buffer, sus_data_ind_event_enable);
			}

			/* Reset event flag */
			sus_data_ind_event_enable = 0;
		}

		/* Check Phy Sniffer event */
		if (sb_phy_snf_event_enable) {
			if (_phy_sniffer_cb_handler) {
				_phy_sniffer_cb_handler();
			}

			/* Reset event flag */
			sb_phy_snf_event_enable = false;
		}
	}
}
