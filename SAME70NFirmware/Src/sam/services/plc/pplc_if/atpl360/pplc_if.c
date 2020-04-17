/**
 * \file
 *
 * \brief Proxy PLC Controller interface layer implementation.
 *
 * Copyright (c) 2014 Atmel Corporation. All rights reserved.
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

#include "string.h"
#include "pplc_if.h"
#include "conf_pplc_if.h"
#include "ioport.h"
#include "sysclk.h"
#include "spi.h"
#if (BOARD == USER_BOARD) || (BOARD == PL360BN)
#include "xdmac.h"
#else
#include "pdc.h"
#endif
#include "pio.h"
#include "pio_handler.h"
#include "delay.h"

/* @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* @endcond */

/**
 * \weakgroup pplc_plc_group
 * @{
 */

/* PPLC clock setting (Hz). */
static uint32_t gs_ul_pplc_clock = PPLC_CLOCK;

/** SPI Header size. */
#define PDC_SPI_HEADER_SIZE            4
/** SPI Max Msg_Data size. */
#define PDC_SPI_MSG_DATA_SIZE          512
/** SPI Max Msg_Data size. */
#define PDC_SPI_MSG_PARAMS_SIZE        118   /* Worst case = 118: sizeof(rx_msg_t) [G3] */
/** PDC buffer us_size. */
#define PDC_PPLC_BUFFER_SIZE           (PDC_SPI_HEADER_SIZE + PDC_SPI_MSG_DATA_SIZE + PDC_SPI_MSG_PARAMS_SIZE)

/* Max number of folowing reads. */
#define MAX_NUM_READ_RX_COUNTER    10

/* PDC Receive buffer */
static uint8_t gs_pplc_rx_buffer[PDC_PPLC_BUFFER_SIZE];
/* PDC Transmission buffer */
static uint8_t gs_pplc_tx_buffer[PDC_PPLC_BUFFER_SIZE];
#if (BOARD == USER_BOARD) || (BOARD == PL360BN)
/** XDMA channel used for PPLC_IF */
#define XDMA_CH    0 /* Used for DEBUG */
#define PPLC_XDMAC_CH_TX 0
#define PPLC_XDMAC_CH_RX 1
/* XDMAC channel status  */
#define PPLC_XDMAC_TX_STATUS XDMAC_GS_ST0
#define PPLC_XDMAC_RX_STATUS XDMAC_GS_ST1
/* XDMAC Peripheral IDs */
#define PPLC_XDMAC_SPI0_TX_PERID 1
#define PPLC_XDMAC_SPI0_RX_PERID 2
/** XDMA channel configuration. */
static xdmac_channel_config_t xdmac_tx_channel_cfg_boot;
static xdmac_channel_config_t xdmac_rx_channel_cfg_boot;
static xdmac_channel_config_t xdmac_tx_channel_cfg_cortex;
static xdmac_channel_config_t xdmac_rx_channel_cfg_cortex;
#else
/* PDC RX data packet */
pdc_packet_t g_pplc_rx_packet;
/* PDC TX data packet */
pdc_packet_t g_pplc_tx_packet;
/* Pointer to PDC register base */
Pdc *g_pplc_pdc;
#endif

/* PPLC chip select config value */
#define PPLC_PCS    spi_get_pcs(PPLC_CS)

/* Global SPI status */
static bool sb_spi_busy;

/**
 * Describes an PPLC interrupt handler
 */
static void (*pplc_handler)(void);

static void pplc_if_int_handler(uint32_t ul_id, uint32_t ul_mask)
{
	UNUSED(ul_id);
	UNUSED(ul_mask);
	if (pplc_handler != NULL) {
		pplc_handler();
		/* Clear INT info */
		pio_get_interrupt_status(PPLC_INT_PIO);
	}
}

#if (BOARD == USER_BOARD) || (BOARD == PL360BN)

/**
 * \brief Disable XDMAC for spi and forbidden transmit and receive by XDMAC.
 *
 */
static void _xdmac_disable(void)
{
	uint32_t xdmaint;

	xdmaint = (XDMAC_CIE_BIE |
			XDMAC_CIE_DIE   |
			XDMAC_CIE_FIE   |
			XDMAC_CIE_RBIE  |
			XDMAC_CIE_WBIE  |
			XDMAC_CIE_ROIE);

	xdmac_channel_disable_interrupt(XDMAC, PPLC_XDMAC_CH_RX, xdmaint);
	xdmac_channel_disable(XDMAC, PPLC_XDMAC_CH_RX);
	xdmac_disable_interrupt(XDMAC, PPLC_XDMAC_CH_RX);

	xdmac_channel_disable_interrupt(XDMAC, PPLC_XDMAC_CH_TX, xdmaint);
	xdmac_channel_disable(XDMAC, PPLC_XDMAC_CH_TX);
	xdmac_disable_interrupt(XDMAC, PPLC_XDMAC_CH_TX);

	NVIC_ClearPendingIRQ(XDMAC_IRQn);
	NVIC_DisableIRQ(XDMAC_IRQn);
}

#endif

/**
 * \internal
 * \brief Initialize Proxy PLC controller.
 *
 * This function will change the system clock prescaler configuration to
 * match the parameters.
 *
 * \note The parameters to this function are device-specific.
 *
 */
void pplc_if_config(void)
{
	uint32_t ul_cpuhz;
	uint8_t uc_div;

	ul_cpuhz = sysclk_get_cpu_hz();
	uc_div = ul_cpuhz / gs_ul_pplc_clock;

	if (ul_cpuhz % gs_ul_pplc_clock) {
		uc_div++;
	}

	/* Enable SPI peripheral. */
	spi_enable_clock(PPLC_SPI_MODULE);

	/* Reset SPI */
	spi_disable(PPLC_SPI_MODULE);
	spi_reset(PPLC_SPI_MODULE);

	/* Configure SPI */
	spi_set_master_mode(PPLC_SPI_MODULE);
	spi_disable_mode_fault_detect(PPLC_SPI_MODULE);
	spi_set_peripheral_chip_select_value(PPLC_SPI_MODULE, PPLC_PCS);
	spi_set_clock_polarity(PPLC_SPI_MODULE, PPLC_CS, 0);
	spi_set_clock_phase(PPLC_SPI_MODULE, PPLC_CS, 1);
	spi_set_bits_per_transfer(PPLC_SPI_MODULE, PPLC_CS, SPI_CSR_BITS_16_BIT);
	spi_set_fixed_peripheral_select(PPLC_SPI_MODULE);
	spi_set_baudrate_div(PPLC_SPI_MODULE, PPLC_CS, uc_div);
	spi_set_transfer_delay(PPLC_SPI_MODULE, PPLC_CS, PPLC_DLYBS, PPLC_DLYBCT);
	spi_configure_cs_behavior(PPLC_SPI_MODULE, PPLC_CS, SPI_CS_RISE_NO_TX);

#if (BOARD == USER_BOARD) || (BOARD == PL360BN)
	/* Initialize and enable DMA controller */
	pmc_enable_periph_clk(ID_XDMAC);

	/* Turn off XDMAC initially */
	_xdmac_disable();

	/* Configure TX and RX XDMAC channels */
	xdmac_tx_channel_cfg_boot.mbr_sa = (uint32_t)gs_pplc_tx_buffer;
	xdmac_tx_channel_cfg_boot.mbr_da = (uint32_t)spi_get_tx_access(PPLC_SPI_MODULE);
	xdmac_tx_channel_cfg_boot.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
			XDMAC_CC_DSYNC_MEM2PER |
			XDMAC_CC_PERID(PPLC_XDMAC_SPI0_TX_PERID) |
			XDMAC_CC_CSIZE_CHK_1 |
			XDMAC_CC_MEMSET_NORMAL_MODE |
			XDMAC_CC_MBSIZE_SINGLE |
			XDMAC_CC_DWIDTH_BYTE |
			XDMAC_CC_SIF_AHB_IF0 |
			XDMAC_CC_DIF_AHB_IF1 |
			XDMAC_CC_SAM_INCREMENTED_AM |
			XDMAC_CC_DAM_FIXED_AM;
	xdmac_tx_channel_cfg_boot.mbr_bc = 0;
	xdmac_tx_channel_cfg_boot.mbr_ds = 0;
	xdmac_tx_channel_cfg_boot.mbr_sus = 0;
	xdmac_tx_channel_cfg_boot.mbr_dus = 0;

	xdmac_rx_channel_cfg_boot.mbr_sa = (uint32_t)spi_get_rx_access(PPLC_SPI_MODULE);
	xdmac_rx_channel_cfg_boot.mbr_da = (uint32_t)gs_pplc_rx_buffer;
	xdmac_rx_channel_cfg_boot.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
			XDMAC_CC_DSYNC_PER2MEM |
			XDMAC_CC_PERID(PPLC_XDMAC_SPI0_RX_PERID) |
			XDMAC_CC_CSIZE_CHK_1 |
			XDMAC_CC_MEMSET_NORMAL_MODE |
			XDMAC_CC_MBSIZE_SINGLE |
			XDMAC_CC_DWIDTH_BYTE |
			XDMAC_CC_SIF_AHB_IF1 |
			XDMAC_CC_DIF_AHB_IF0 |
			XDMAC_CC_SAM_FIXED_AM |
			XDMAC_CC_DAM_INCREMENTED_AM;
	xdmac_rx_channel_cfg_boot.mbr_bc = 0;
	xdmac_rx_channel_cfg_boot.mbr_ds = 0;
	xdmac_rx_channel_cfg_boot.mbr_sus = 0;
	xdmac_rx_channel_cfg_boot.mbr_dus = 0;

	xdmac_tx_channel_cfg_cortex.mbr_sa = (uint32_t)gs_pplc_tx_buffer;
	xdmac_tx_channel_cfg_cortex.mbr_da = (uint32_t)spi_get_tx_access(PPLC_SPI_MODULE);
	xdmac_tx_channel_cfg_cortex.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
			XDMAC_CC_DSYNC_MEM2PER |
			XDMAC_CC_PERID(PPLC_XDMAC_SPI0_TX_PERID) |
			XDMAC_CC_CSIZE_CHK_1 |
			XDMAC_CC_MEMSET_NORMAL_MODE |
			XDMAC_CC_MBSIZE_SINGLE |
			XDMAC_CC_DWIDTH_HALFWORD |
			XDMAC_CC_SIF_AHB_IF0 |
			XDMAC_CC_DIF_AHB_IF1 |
			XDMAC_CC_SAM_INCREMENTED_AM |
			XDMAC_CC_DAM_FIXED_AM;
	xdmac_tx_channel_cfg_cortex.mbr_bc = 0;
	xdmac_tx_channel_cfg_cortex.mbr_ds = 0;
	xdmac_tx_channel_cfg_cortex.mbr_sus = 0;
	xdmac_tx_channel_cfg_cortex.mbr_dus = 0;

	xdmac_rx_channel_cfg_cortex.mbr_sa = (uint32_t)spi_get_rx_access(PPLC_SPI_MODULE);
	xdmac_rx_channel_cfg_cortex.mbr_da = (uint32_t)gs_pplc_rx_buffer;
	xdmac_rx_channel_cfg_cortex.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
			XDMAC_CC_DSYNC_PER2MEM |
			XDMAC_CC_PERID(PPLC_XDMAC_SPI0_RX_PERID) |
			XDMAC_CC_CSIZE_CHK_1 |
			XDMAC_CC_MEMSET_NORMAL_MODE |
			XDMAC_CC_MBSIZE_SINGLE |
			XDMAC_CC_DWIDTH_HALFWORD |
			XDMAC_CC_SIF_AHB_IF1 |
			XDMAC_CC_DIF_AHB_IF0 |
			XDMAC_CC_SAM_FIXED_AM |
			XDMAC_CC_DAM_INCREMENTED_AM;
	xdmac_rx_channel_cfg_cortex.mbr_bc = 0;
	xdmac_rx_channel_cfg_cortex.mbr_ds = 0;
	xdmac_rx_channel_cfg_cortex.mbr_sus = 0;
	xdmac_rx_channel_cfg_cortex.mbr_dus = 0;
#else
	/* Get board PPLC PDC base address and enable receiver and transmitter */
	g_pplc_pdc = spi_get_pdc_base(PPLC_SPI_MODULE);
#endif
	spi_enable(PPLC_SPI_MODULE);
}

/**
 * \brief Initialize PPLC interface
 *
 */
void pplc_if_init(void)
{
	/* Init PPLC handler */
	pplc_handler = NULL;
	sb_spi_busy = false;

	/* Initialize PPLC */
	pplc_if_config();

	/* Configure PLC reset pins */
#if BOARD == SAM4CP16CMB
	ioport_set_pin_level(PPLC_ARST_GPIO, PPLC_ARST_INACTIVE_LEVEL);
	ioport_set_pin_dir(PPLC_ARST_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PPLC_SRST_GPIO, PPLC_SRST_INACTIVE_LEVEL);
	ioport_set_pin_dir(PPLC_SRST_GPIO, IOPORT_DIR_OUTPUT);
#elif BOARD == ATPL230AMB
	ioport_set_pin_level(PPLC_ARST_GPIO, PPLC_ARST_INACTIVE_LEVEL);
	ioport_set_pin_dir(PPLC_ARST_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PPLC_SRST_GPIO, PPLC_SRST_INACTIVE_LEVEL);
	ioport_set_pin_dir(PPLC_SRST_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(PPLC_RST_GPIO, IOPORT_DIR_OUTPUT);
#elif BOARD == KINTEX_FPGA
	ioport_set_pin_level(PPLC_RST_GPIO, PPLC_RST_INACTIVE_LEVEL);
	ioport_set_pin_dir(PPLC_RST_GPIO, IOPORT_DIR_OUTPUT);
#elif BOARD == ATPL360AMB || BOARD == ATPL360ASB || BOARD == ATPL360MB || BOARD == PL360G55CB_EK || BOARD == PL360G55CF_EK
	ioport_set_pin_level(ATPL360_RESET_GPIO, ATPL360_RESET_ACTIVE_LEVEL);
	ioport_set_pin_dir(ATPL360_RESET_GPIO, IOPORT_DIR_OUTPUT);
#elif BOARD == USER_BOARD || BOARD == PL360BN
	ioport_set_pin_level(ATPL360_RESET_GPIO, ATPL360_RESET_ACTIVE_LEVEL);
	ioport_set_pin_dir(ATPL360_RESET_GPIO, IOPORT_DIR_OUTPUT);
#elif BOARD == PL485_VB
	ioport_set_pin_level(PL360_RESET_GPIO, PL360_RESET_ACTIVE_LEVEL);
	ioport_set_pin_dir(PL360_RESET_GPIO, IOPORT_DIR_OUTPUT);
#else
	;
#endif
}

/**
 * \brief Reset PPLC interface
 *
 */
void pplc_if_reset(void)
{
#if BOARD == SAM4CP16CMB
	/* Reset on ARST of modem PLC */
	ioport_set_pin_level(PPLC_ARST_GPIO, PPLC_ARST_ACTIVE_LEVEL);
	delay_ms(10);
	/* Clear ARST of modem PLC */
	ioport_set_pin_level(PPLC_ARST_GPIO, PPLC_ARST_INACTIVE_LEVEL);
#elif BOARD == ATPL230AMB
	/* Reset on ARST of modem PLC */
	gpio_set_pin_low(PPLC_ARST_GPIO);
	delay_ms(10);
	/* Clear ARST of modem PLC */
	gpio_set_pin_high(PPLC_ARST_GPIO);
#elif BOARD == ATPL360AMB || BOARD == ATPL360ASB || BOARD == ATPL360MB || BOARD == PL360G55CB_EK || BOARD == PL360G55CF_EK
	/* Enable LDO line */
	ioport_set_pin_level(ATPL360_LDO_EN_GPIO, ATPL360_LDO_EN_ACTIVE_LEVEL);
	ioport_set_pin_dir(ATPL360_LDO_EN_GPIO, IOPORT_DIR_OUTPUT);
	delay_ms(1);
	/* Reset on RST of modem PLC */
	ioport_set_pin_level(ATPL360_RESET_GPIO, ATPL360_RESET_ACTIVE_LEVEL);
	delay_ms(1);
	/* Clear RST of modem PLC */
	ioport_set_pin_level(ATPL360_RESET_GPIO, ATPL360_RESET_INACTIVE_LEVEL);
#elif BOARD == PL485_VB
	/* Enable LDO line */
	ioport_set_pin_level(PL360_LDO_EN_GPIO, PL360_LDO_EN_ACTIVE_LEVEL);
	ioport_set_pin_dir(PL360_LDO_EN_GPIO, IOPORT_DIR_OUTPUT);
	delay_ms(1);
	/* Reset on RST of modem PLC */
	ioport_set_pin_level(PL360_RESET_GPIO, PL360_RESET_ACTIVE_LEVEL);
	delay_ms(1);
	/* Clear RST of modem PLC */
	ioport_set_pin_level(PL360_RESET_GPIO, PL360_RESET_INACTIVE_LEVEL);
#elif BOARD == KINTEX_FPGA
	/* Reset on RST of modem PLC */
	ioport_set_pin_level(PPLC_RST_GPIO, PPLC_RST_ACTIVE_LEVEL);
	delay_ms(10);
	/* Clear RST of modem PLC */
	ioport_set_pin_level(PPLC_RST_GPIO, PPLC_RST_INACTIVE_LEVEL);
#elif BOARD == USER_BOARD || BOARD == PL360BN
	/* Enable LDO line */
	ioport_set_pin_level(ATPL360_LDO_EN_GPIO, ATPL360_LDO_EN_ACTIVE_LEVEL);
	ioport_set_pin_dir(ATPL360_LDO_EN_GPIO, IOPORT_DIR_OUTPUT);
	delay_ms(1);
	/* Reset on RST of modem PLC */
	ioport_set_pin_level(ATPL360_RESET_GPIO, ATPL360_RESET_ACTIVE_LEVEL);
	delay_ms(1);
	/* Clear RST of modem PLC */
	ioport_set_pin_level(ATPL360_RESET_GPIO, ATPL360_RESET_INACTIVE_LEVEL);
#else
	/* Modify for customer board */;
	return;
#endif

	delay_ms(50);
}

/**
 * \brief Set an interrupt handler for the specified interrput source.
 */
void pplc_if_set_handler(void (*p_handler)(void))
{
	pplc_handler = p_handler;

	/* Configure PPLC interruption pin */
	ioport_set_pin_mode(PPLC_INT_GPIO, IOPORT_MODE_PULLDOWN);
	ioport_set_pin_dir(PPLC_INT_GPIO, IOPORT_DIR_INPUT);

	/* Configure PPLC Interruption */
	pmc_enable_periph_clk(PPLC_INT_ID);
	pio_handler_set(PPLC_INT_PIO, PPLC_INT_ID, PPLC_INT_MASK, PPLC_INT_ATTR, pplc_if_int_handler);

	NVIC_SetPriority(PPLC_INT_IRQn, PPLC_PRIO);
	NVIC_ClearPendingIRQ(PPLC_INT_IRQn);
	NVIC_EnableIRQ(PPLC_INT_IRQn);
}

#if (BOARD == USER_BOARD) || (BOARD == PL360BN)

bool pplc_if_send_boot_cmd(uint16_t us_cmd, uint32_t ul_addr, uint32_t ul_data_len, uint8_t *puc_data_buf, uint8_t *puc_data_read)
{
	uint8_t *puc_tx_buf;
	uint32_t ul_spi_busy_cnt;
	uint16_t us_tx_size;

	/* Waiting transfer done*/
	ul_spi_busy_cnt = 0;
	while (xdmac_channel_get_status(XDMAC) & PPLC_XDMAC_RX_STATUS) {
		ul_spi_busy_cnt++;
		if (ul_spi_busy_cnt > 5000000) {
			return false;
		}
	}

	NVIC_DisableIRQ(PPLC_INT_IRQn);

	/* Disable the RX and TX PDC transfer requests */
	xdmac_channel_disable(XDMAC, PPLC_XDMAC_CH_RX);
	xdmac_channel_disable(XDMAC, PPLC_XDMAC_CH_TX);

	/* Set 8 bits per transfer */
	spi_set_bits_per_transfer(PPLC_SPI_MODULE, PPLC_CS, SPI_CSR_BITS_8_BIT);

	/* Configure Tx buffer */
	puc_tx_buf = gs_pplc_tx_buffer;

	memcpy(puc_tx_buf, &ul_addr, sizeof(uint32_t));
	puc_tx_buf +=  sizeof(uint32_t);
	memcpy(puc_tx_buf, &us_cmd, sizeof(uint16_t));
	puc_tx_buf +=  sizeof(uint16_t);

	memcpy(puc_tx_buf, puc_data_buf, ul_data_len);

	puc_tx_buf += ul_data_len;

	us_tx_size = puc_tx_buf - gs_pplc_tx_buffer;

#ifdef CONF_BOARD_ENABLE_CACHE
	SCB_InvalidateDCache_by_Addr((uint32_t *)gs_pplc_tx_buffer, us_tx_size);
	SCB_InvalidateDCache_by_Addr((uint32_t *)gs_pplc_rx_buffer, us_tx_size);
#endif

	/* Configure TX and RX XDMAC channels */
	xdmac_rx_channel_cfg_boot.mbr_ubc = us_tx_size;
	xdmac_tx_channel_cfg_boot.mbr_ubc = us_tx_size;

	xdmac_configure_transfer(XDMAC, PPLC_XDMAC_CH_RX, &xdmac_rx_channel_cfg_boot);
	xdmac_channel_set_descriptor_control(XDMAC, PPLC_XDMAC_CH_RX, 0);

	xdmac_configure_transfer(XDMAC, PPLC_XDMAC_CH_TX, &xdmac_tx_channel_cfg_boot);
	xdmac_channel_set_descriptor_control(XDMAC, PPLC_XDMAC_CH_TX, 0);

	/* Enable the RX and TX PDC transfer requests */
	xdmac_channel_enable(XDMAC, PPLC_XDMAC_CH_RX);
	xdmac_channel_enable(XDMAC, PPLC_XDMAC_CH_TX);

	/* Waiting transfer done and read */
	if (puc_data_read) {
		/* while(pdc_read_tx_counter(g_pdc) > 0); */
		ul_spi_busy_cnt = 0;
		while (xdmac_channel_get_status(XDMAC) & PPLC_XDMAC_RX_STATUS) {
			ul_spi_busy_cnt++;
			if (ul_spi_busy_cnt > 5000000) {
				/* Enable PLC interrupt(); */
				NVIC_EnableIRQ(PPLC_INT_IRQn);
				return false;
			}
		}

		memcpy(puc_data_read, &gs_pplc_rx_buffer[6], ul_data_len);
	}

	/* Enable PLC interrupt(); */
	NVIC_EnableIRQ(PPLC_INT_IRQn);

	return true;
}

bool pplc_if_send_wrrd_cmd(uint8_t uc_cmd, void *px_spi_data, void *px_spi_status_info)
{
	uint8_t *puc_tx_buf;
	uint32_t ul_spi_busy_cnt;
	uint16_t us_tx_size, us_tx_hdr_size;
	uint16_t us_len_wr_rd;
	spi_data_t *px_data;
	spi_status_info_t *px_status_info;

	px_data = (spi_data_t *)px_spi_data;
	px_status_info = (spi_status_info_t *)px_spi_status_info;

	us_len_wr_rd = (((px_data->us_len + 1) / 2) & PPLC_LEN_MASK) | (uc_cmd << PPLC_WR_RD_POS);

	/* Check length */
	if (!us_len_wr_rd) {
		return false;
	}

	/* Waiting transfer done while(pdc_read_tx_counter(g_pdc) > 0); */
	ul_spi_busy_cnt = 0;
	while (xdmac_channel_get_status(XDMAC) & PPLC_XDMAC_TX_STATUS) {
		ul_spi_busy_cnt++;
		if (ul_spi_busy_cnt > 5000000) {
			return false;
		}
	}
	ul_spi_busy_cnt = 0;
	while (xdmac_channel_get_status(XDMAC) & PPLC_XDMAC_RX_STATUS) {
		ul_spi_busy_cnt++;
		if (ul_spi_busy_cnt > 5000000) {
			return false;
		}
	}

	NVIC_DisableIRQ(PPLC_INT_IRQn);

	/* Disable the RX and TX PDC transfer requests */
	xdmac_channel_disable(XDMAC, PPLC_XDMAC_CH_RX);
	xdmac_channel_disable(XDMAC, PPLC_XDMAC_CH_TX);

	/* Set 16 bits per transfer */
	spi_set_bits_per_transfer(PPLC_SPI_MODULE, PPLC_CS, SPI_CSR_BITS_16_BIT);

	/** Configure PPLC Tx buffer **/
	puc_tx_buf = gs_pplc_tx_buffer;
	/* Address */
	*puc_tx_buf++ = (uint8_t)(px_data->us_address);
	*puc_tx_buf++ = (uint8_t)(px_data->us_address >> 8);
	/* Length & read/write */
	*puc_tx_buf++ = (uint8_t)(us_len_wr_rd);
	*puc_tx_buf++ = (uint8_t)(us_len_wr_rd >> 8);

	us_tx_hdr_size = puc_tx_buf - gs_pplc_tx_buffer;

	if (uc_cmd == PPLC_CMD_WRITE) {
		memcpy(puc_tx_buf, px_data->puc_data_buf, px_data->us_len);
	} else {
		memset(puc_tx_buf, 0, px_data->us_len);
	}

	puc_tx_buf += px_data->us_len;

	us_tx_size = puc_tx_buf - gs_pplc_tx_buffer;
	if (us_tx_size % 2) {
		*puc_tx_buf++ = 0;
		us_tx_size++;
	}

#ifdef CONF_BOARD_ENABLE_CACHE
	SCB_InvalidateDCache_by_Addr((uint32_t *)gs_pplc_tx_buffer, us_tx_size / 2);
	SCB_InvalidateDCache_by_Addr((uint32_t *)gs_pplc_rx_buffer, us_tx_size / 2);
#endif
	/* Configure TX and RX XDMAC channels */
	xdmac_rx_channel_cfg_cortex.mbr_ubc = us_tx_size / 2;
	xdmac_tx_channel_cfg_cortex.mbr_ubc = us_tx_size / 2;

	xdmac_configure_transfer(XDMAC, PPLC_XDMAC_CH_RX, &xdmac_rx_channel_cfg_cortex);
	xdmac_channel_set_descriptor_control(XDMAC, PPLC_XDMAC_CH_RX, 0);

	xdmac_configure_transfer(XDMAC, PPLC_XDMAC_CH_TX, &xdmac_tx_channel_cfg_cortex);
	xdmac_channel_set_descriptor_control(XDMAC, PPLC_XDMAC_CH_TX, 0);

	/* Enable the RX and TX PDC transfer requests */
	xdmac_channel_enable(XDMAC, PPLC_XDMAC_CH_RX);
	xdmac_channel_enable(XDMAC, PPLC_XDMAC_CH_TX);

	/* Waiting transfer done*/
	/* while(pdc_read_tx_counter(g_pdc) > 0); */
	ul_spi_busy_cnt = 0;
	while (xdmac_channel_get_status(XDMAC) & PPLC_XDMAC_TX_STATUS) {
		ul_spi_busy_cnt++;
		if (ul_spi_busy_cnt > 5000000) {
			/* Enable PLC interrupt(); */
			NVIC_EnableIRQ(PPLC_INT_IRQn);
			return false;
		}
	}
	ul_spi_busy_cnt = 0;
	while (xdmac_channel_get_status(XDMAC) & PPLC_XDMAC_RX_STATUS) {
		ul_spi_busy_cnt++;
		if (ul_spi_busy_cnt > 5000000) {
			/* Enable PLC interrupt(); */
			NVIC_EnableIRQ(PPLC_INT_IRQn);
			return false;
		}
	}

	if (uc_cmd == PPLC_CMD_READ) {
		memcpy(px_data->puc_data_buf, &gs_pplc_rx_buffer[us_tx_hdr_size], px_data->us_len);
	}

	px_status_info->us_header_id = PPLC_GET_ID_HEADER(gs_pplc_rx_buffer[0], gs_pplc_rx_buffer[1]);
	if (PPLC_CHECK_ID_BOOT_HEADER(px_status_info->us_header_id)) {
		px_status_info->ul_flags = PPLC_GET_FLAGS_FROM_BOOT(gs_pplc_rx_buffer[0], gs_pplc_rx_buffer[2], gs_pplc_rx_buffer[3]);
	} else if (PPLC_CHECK_ID_CORTEX_HEADER(px_status_info->us_header_id)) {
		px_status_info->ul_flags = PPLC_GET_FLAGS_FROM_CORTEX(gs_pplc_rx_buffer[2], gs_pplc_rx_buffer[3]);
	} else {
		px_status_info->ul_flags = 0;
	}

	/* Enable PLC interrupt(); */
	NVIC_EnableIRQ(PPLC_INT_IRQn);

	return true;
}

#else

bool pplc_if_send_boot_cmd(uint16_t us_cmd, uint32_t ul_addr, uint32_t ul_data_len, uint8_t *puc_data_buf, uint8_t *puc_data_read)
{
	uint8_t *puc_tx_buf;
	uint32_t ul_spi_busy_cnt;
	uint16_t us_tx_size;

	NVIC_DisableIRQ(PPLC_INT_IRQn);

	if (sb_spi_busy) {
		return false;
	}

	/* Update SPI status */
	sb_spi_busy = true;

	/* Waiting transfer done*/
	ul_spi_busy_cnt = 0;
	while ((spi_read_status(PPLC_SPI_MODULE) & SPI_SR_RXBUFF) == 0) {
		ul_spi_busy_cnt++;
		if (ul_spi_busy_cnt > 5000000) {
			/* Update SPI status */
			sb_spi_busy = false;
			/* Enable PLC interrupt(); */
			NVIC_EnableIRQ(PPLC_INT_IRQn);
			return false;
		}
	}

	/* Disable_global_interrupt(); */

	/* Disable the RX and TX PDC transfer requests */
	pdc_disable_transfer(g_pplc_pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);

	/* Set 8 bits per transfer */
	spi_set_bits_per_transfer(PPLC_SPI_MODULE, PPLC_CS, SPI_CSR_BITS_8_BIT);

	/* Configure Tx buffer */
	puc_tx_buf = gs_pplc_tx_buffer;

	memcpy(puc_tx_buf, &ul_addr, sizeof(uint32_t));
	puc_tx_buf +=  sizeof(uint32_t);
	memcpy(puc_tx_buf, &us_cmd, sizeof(uint16_t));
	puc_tx_buf +=  sizeof(uint16_t);

	memcpy(puc_tx_buf, puc_data_buf, ul_data_len);

	puc_tx_buf += ul_data_len;

	us_tx_size = puc_tx_buf - gs_pplc_tx_buffer;

	/* Configure DMA channels */
	g_pplc_rx_packet.ul_addr = (uint32_t)gs_pplc_rx_buffer;
	g_pplc_rx_packet.ul_size = us_tx_size;
	pdc_rx_init(g_pplc_pdc, &g_pplc_rx_packet, NULL);

	g_pplc_tx_packet.ul_addr = (uint32_t)gs_pplc_tx_buffer;
	g_pplc_tx_packet.ul_size = us_tx_size;
	pdc_tx_init(g_pplc_pdc, &g_pplc_tx_packet, NULL);

	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(g_pplc_pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

	/* Waiting transfer done and read */
	if (puc_data_read) {
		/* while(pdc_read_tx_counter(g_pdc) > 0); */
		ul_spi_busy_cnt = 0;
		while ((spi_read_status(PPLC_SPI_MODULE) & SPI_SR_RXBUFF) == 0) {
			ul_spi_busy_cnt++;
			if (ul_spi_busy_cnt > 5000000) {
				return false;
			}
		}

		memcpy(puc_data_read, &gs_pplc_rx_buffer[6], ul_data_len);
	}

	/* Update SPI status */
	sb_spi_busy = false;

	/* Enable PLC interrupt(); */
	NVIC_EnableIRQ(PPLC_INT_IRQn);

	return true;
}

bool pplc_if_send_wrrd_cmd(uint8_t uc_cmd, void *px_spi_data, void *px_spi_status_info)
{
	uint8_t *puc_tx_buf;
	uint32_t ul_spi_busy_cnt;
	uint16_t us_tx_size;
	uint16_t us_len_wr_rd;
	spi_data_t *px_data;
	spi_status_info_t *px_status_info;

	NVIC_DisableIRQ(PPLC_INT_IRQn);

	if (sb_spi_busy) {
		return false;
	}

	/* Update SPI status */
	sb_spi_busy = true;

	px_data = (spi_data_t *)px_spi_data;
	px_status_info = (spi_status_info_t *)px_spi_status_info;

	us_len_wr_rd = (((px_data->us_len + 1) / 2) & PPLC_LEN_MASK)    | (uc_cmd << PPLC_WR_RD_POS);

	/* Check length */
	if (!us_len_wr_rd) {
		/* Update SPI status */
		sb_spi_busy = false;
		/* Enable PLC interrupt(); */
		NVIC_EnableIRQ(PPLC_INT_IRQn);
		return false;
	}

	/* Waiting transfer done while(pdc_read_tx_counter(g_pdc) > 0); */
	ul_spi_busy_cnt = 0;
	while ((spi_read_status(PPLC_SPI_MODULE) & SPI_SR_RXBUFF) == 0) {
		ul_spi_busy_cnt++;
		if (ul_spi_busy_cnt > 5000000) {
			/* Update SPI status */
			sb_spi_busy = false;
			/* Enable PLC interrupt(); */
			NVIC_EnableIRQ(PPLC_INT_IRQn);
			return false;
		}
	}

	/* Disable the RX and TX PDC transfer requests */
	pdc_disable_transfer(g_pplc_pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);

	/* Set 16 bits per transfer */
	spi_set_bits_per_transfer(PPLC_SPI_MODULE, PPLC_CS, SPI_CSR_BITS_16_BIT);

	/** Configure PPLC Tx buffer **/
	puc_tx_buf = gs_pplc_tx_buffer;
	/* Address */
	*puc_tx_buf++ = (uint8_t)(px_data->us_address & 0xFF);
	*puc_tx_buf++ = (uint8_t)(px_data->us_address >> 8);
	/* Length & read/write */
	*puc_tx_buf++ = (uint8_t)(us_len_wr_rd & 0xFF);
	*puc_tx_buf++ = (uint8_t)(us_len_wr_rd >> 8);

	if (uc_cmd == PPLC_CMD_WRITE) {
		memcpy(puc_tx_buf, px_data->puc_data_buf, px_data->us_len);
	} else {
		memset(puc_tx_buf, 0, px_data->us_len);
	}

	puc_tx_buf += px_data->us_len;

	us_tx_size = puc_tx_buf - gs_pplc_tx_buffer;
	if (us_tx_size % 2) {
		*puc_tx_buf++ = 0;
		us_tx_size++;
	}

	/* Configure DMA channels */
	g_pplc_rx_packet.ul_addr = (uint32_t)gs_pplc_rx_buffer;
	g_pplc_rx_packet.ul_size = us_tx_size / 2;
	pdc_rx_init(g_pplc_pdc, &g_pplc_rx_packet, NULL);

	g_pplc_tx_packet.ul_addr = (uint32_t)gs_pplc_tx_buffer;
	g_pplc_tx_packet.ul_size = us_tx_size / 2;
	pdc_tx_init(g_pplc_pdc, &g_pplc_tx_packet, NULL);

	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(g_pplc_pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

	/* Waiting transfer done*/
	ul_spi_busy_cnt = 0;
	while ((spi_read_status(PPLC_SPI_MODULE) & SPI_SR_RXBUFF) == 0) {
		ul_spi_busy_cnt++;
		if (ul_spi_busy_cnt > 5000000) {
			/* Update SPI status */
			sb_spi_busy = false;
			/* Enable PLC interrupt(); */
			NVIC_EnableIRQ(PPLC_INT_IRQn);
			return false;
		}
	}

	if (uc_cmd == PPLC_CMD_READ) {
		memcpy(px_data->puc_data_buf, &gs_pplc_rx_buffer[PDC_SPI_HEADER_SIZE], px_data->us_len);
	}

	px_status_info->us_header_id = PPLC_GET_ID_HEADER(gs_pplc_rx_buffer[0], gs_pplc_rx_buffer[1]);
	if (PPLC_CHECK_ID_BOOT_HEADER(px_status_info->us_header_id)) {
		px_status_info->ul_flags = PPLC_GET_FLAGS_FROM_BOOT(gs_pplc_rx_buffer[0], gs_pplc_rx_buffer[2], gs_pplc_rx_buffer[3]);
	} else if (PPLC_CHECK_ID_CORTEX_HEADER(px_status_info->us_header_id)) {
		px_status_info->ul_flags = PPLC_GET_FLAGS_FROM_CORTEX(gs_pplc_rx_buffer[2], gs_pplc_rx_buffer[3]);
	} else {
		px_status_info->ul_flags = 0;
	}

	/* Update SPI status */
	sb_spi_busy = false;

	/* Enable PLC interrupt(); */
	NVIC_EnableIRQ(PPLC_INT_IRQn);

	return true;
}

#endif

void pplc_if_enable_interrupt(bool enable)
{
	if (enable) {
		pio_enable_interrupt(PPLC_INT_PIO, PPLC_INT_MASK);
	} else {
		pio_disable_interrupt(PPLC_INT_PIO, PPLC_INT_MASK);
	}
}

void pplc_if_delay(uint8_t uc_tref, uint32_t ul_delay)
{
	if (uc_tref == PPLC_DELAY_TREF_SEC) {
		delay_s(ul_delay);
	} else if (uc_tref == PPLC_DELAY_TREF_MS) {
		delay_ms(ul_delay);
	} else if (uc_tref == PPLC_DELAY_TREF_US) {
		delay_us(ul_delay);
	}
}

/* @} */

/* @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* @endcond */
