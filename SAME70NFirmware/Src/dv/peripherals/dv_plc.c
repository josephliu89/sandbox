/*
 * dv_plc.c
 *
 * Created: 2019-12-12 3:40:07 PM
 *  Author: wrinch
 */ 

#include "dv_plc.h"
#include "dv/telem/dv_handler.h"
#include "dv/telemlib/DvTelemUtils.h"
#include "dv/telemlib/DvTelemPacket.h"
#include "dv/telemlib/DvTelemFraming.h"
#include "asf.h"

DV_warnPush()
DV_warnDisableGCC("-Wcast-align")

/* Time in ms that LED is ON after message reception */
#define COUNT_MS_IND_LED        50
/* Personal Address Network Identificator */
#define APP_G3_PAN_ID           0x781D

// G3 Extended Address 
static uint8_t spuc_extended_address[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};

#ifdef CONF_COUP_PARAMETERS
/* Coupling configuration parameters. Values defined in "conf_project.h" are the default values, calibrated for MCHP reference designs */
/* For other Hardware designs, it may be needed to calibrate and obtain your own values. MCHP PHY Calibration Tool should be used */
static const phy_coup_t sx_pl360_coup = {MAX_RMS_HI_TABLE, MAX_RMS_VLO_TABLE,
    TH1_HI_TABLE, TH2_HI_TABLE, TH1_VLO_TABLE, TH2_VLO_TABLE,
    DACC_CFG_TABLE,
    PREDIST_COEF_HI_TABLE, PREDIST_COEF_VLO_TABLE,
    IFFT_GAIN_HI_INI, IFFT_GAIN_HI_MIN, IFFT_GAIN_HI_MAX,
    IFFT_GAIN_VLO_INI, IFFT_GAIN_VLO_MIN, IFFT_GAIN_VLO_MAX,
NUM_TX_LEVELS};

#  ifdef CONF_ENABLE_MULTIBAND
/* Coupling configuration parameters for FCC (if Multiband is enabled, band can change dynamically between CENELEC-A or FCC) */
static const phy_coup_t sx_pl360_coup_fcc = {MAX_RMS_HI_TABLE_FCC, MAX_RMS_VLO_TABLE_FCC,
    TH1_HI_TABLE_FCC, TH2_HI_TABLE_FCC, TH1_VLO_TABLE_FCC, TH2_VLO_TABLE_FCC,
    DACC_CFG_TABLE_FCC,
    PREDIST_COEF_HI_TABLE_FCC, PREDIST_COEF_VLO_TABLE_FCC,
    IFFT_GAIN_HI_INI_FCC, IFFT_GAIN_HI_MIN_FCC, IFFT_GAIN_HI_MAX_FCC,
    IFFT_GAIN_VLO_INI_FCC, IFFT_GAIN_VLO_MIN_FCC, IFFT_GAIN_VLO_MAX_FCC,
NUM_TX_LEVELS_FCC};
#  endif
#endif

#ifdef CONF_TONEMASK_STATIC_NOTCHING
/* Tone Mask (Static Notching) array. Each carrier corresponding to the band can be notched (no energy is sent in those carriers) */
/* Each carrier is represented by one byte (0: carrier used; 1: carrier notched). By default it is all 0's in PL360 device */
/* The same Tone Mask must be set in both transmitter and receiver. Otherwise they don't understand each other */
static const uint8_t spuc_tone_mask[PROTOCOL_CARRIERS_MAX] = TONE_MASK_STATIC_NOTCHING;
#endif

#if defined(CONF_COUP_PARAMETERS)
/* Number of carriers {CENELEC-A, FCC, ARIB, CENELEC-B} */
static const uint8_t spuc_num_carriers[4] = {NUM_CARRIERS_CENELEC_A, NUM_CARRIERS_FCC, NUM_CARRIERS_ARIB, NUM_CARRIERS_CENELEC_B};
#endif

struct TAppMacRtMhr {
    struct TMacRtFc m_Fc;
    uint8_t m_u8SequenceNumber;
    uint16_t m_nDestinationPanIdentifier;
    uint16_t m_DestinationShortAddress;
    uint16_t m_nSourcePanIdentifier;
    uint16_t m_SourceShortAddress;
};

#define HIGH_PRIORITY_WHEN_GREATER_THAN_BYTES_PENDING 1000u

typedef struct {
    
    int m_role; 
    uint32_t m_deviceId;
    
    uint8_t m_phyBand;
    bool m_transmitting;
    bool m_flushToSwitchAddress;
    
    bool m_enablingPl360;
    bool m_exceptionPending;
    atpl360_descriptor_t m_pl360Desc;
    struct TMacRtPibValue m_pl360G3Pib;

    struct TMacRtTxParameters m_txParams;
    struct TMacRtRxParameters m_rxParams;
    struct TAppMacRtMhr m_macRtHeader;

    // buffer read from the handler.
    size_t m_txFifoBufferOffset;
    uint8_t m_txFifoBuffer[HANDLER_BUFFER_SIZE];

    // buffer for the current address, potentially
    // encode multiple packets into this buffer.
    uint8_t m_txCurrentAddress;
    size_t m_txCurrentAddressBufferOffset;
    uint8_t m_txCurrentAddressBuffer[HANDLER_BUFFER_SIZE];
    DvTelemPacketStatic m_txPacket;

    // buffer for concatenating plc header and packet data.
    uint8_t m_plcPacketBuffer[ATPL360_DATA_MAX_PKT_SIZE];
    
} DvPlcStateData;

static DvPlcStateData s_state;

// Select FCC binary
extern uint8_t atpl_bin_fcc_start;
extern uint8_t atpl_bin_fcc_end;

static uint32_t getPlc360BinaryAndSize(uint32_t* address) {
	*address = (uint32_t)&atpl_bin_fcc_start;
    uint32_t endPos = (uint32_t)&atpl_bin_fcc_end;
    uint32_t startPos = (uint32_t)&atpl_bin_fcc_start;
    return (endPos - startPos);
}

static void enablePlc360(void) {

    // Get PL360 binary address and size
    uint32_t binAddress = 0;
    uint32_t binSize = getPlc360BinaryAndSize(&binAddress);

    // Enable PL360: Load binary
    DV_info("[plc] Enabling PL360 device: Loading PHY binary");
    s_state.m_enablingPl360 = true;
    uint8_t result = atpl360_enable(binAddress, binSize);
    if (result == ATPL360_ERROR) {
        DV_error("[plc] CRITICAL ERROR: PL360 binary load failed (%d)", result);
        while (1) {
        }
    }

    DV_info("[plc] PL360 binary loaded correctly");

    // Get PHY version
    s_state.m_pl360Desc.get_req(MAC_RT_PIB_MANUF_PHY_PARAM, PHY_PARAM_VERSION, &s_state.m_pl360G3Pib);
    DV_info("[plc] PHY version: 0x%08lx", *(uint32_t *)s_state.m_pl360G3Pib.m_au8Value);

    // s_state.m_pl360G3Pib.m_au8Value[2] corresponds to G3 band [0x01: CEN-A, 0x02: FCC, 0x03: ARIB, 0x04: CEN-B] 
    switch (s_state.m_pl360G3Pib.m_au8Value[2]) {
        case ATPL360_WB_CENELEC_A: DV_info("[plc] CENELEC-A band: 35 - 91 kHz)"); break;
        case ATPL360_WB_FCC: DV_info("[plc] FCC band: 154 - 488 kHz)"); break;
        case ATPL360_WB_ARIB: DV_info("[plc] ARIB band: 154 - 404 kHz)"); break;
        case ATPL360_WB_CENELEC_B: DV_info("[plc] CENELEC-B band: 98 - 122 kHz)"); break;
        default: DV_error("[plc] Unknown band");
        break;
    }

    if (s_state.m_pl360G3Pib.m_au8Value[2] != s_state.m_phyBand) {
        DV_error("[plc] ERROR: PHY band does not match with band configured in application");
    }
}


// Set PL360 configuration. Called at initialization (once the binary is loaded) to configure required parameters in PL360 device
static void setupPL360Config(void) {

#ifdef CONF_COUP_PARAMETERS
	phy_coup_t* couplingConfig;
#endif

#if (defined(CONF_ENABLE_C11_CFG) || (BOARD == PL360G55CF_EK))
	/* Specific configuration for PLCOUP011 (or PL360G55CF_EK, which uses PLCOUP011 reference design) */
	if ((s_state.m_phyBand == ATPL360_WB_FCC) || (s_state.m_phyBand == ATPL360_WB_ARIB)) {
		/* FCC/ARIB binaries initialize ATPL360_REG_DACC_TABLE_CFG assuming PLCOUP006 (2 transmission branches: Branch 0 for VLO, Branch for HI) */
		/* PLCOUP011 has 2 branches: Branch 0 for CENELEC-A (PLCOUP007) and Branch for FCC (PLCOUP006 VLO) */
		/* Thus, it is mandataory to reconfigure ATPL360_REG_DACC_TABLE_CFG in FCC/ARIB band with MCHP EKs designed with PLCOUP011 reference design */
		/* In CENELEC-A ATPL360_REG_DACC_TABLE_CFG is the same for PLCOUP007 or PLCOUP011 because in both cases only Branch 0 is used */
		/* Commenting this out for DV13335A implementation because FCC is now using EMIT0, EMIT1, and TXRX0 instead of EMIT2, EMIT3, and TXRX1

		uint32_t pul_dacc_c11_cfg[] = {0x00000000, 0x21202120, 0x073F073F, 0x3F3F3F3F, 0x00000FFF, 0x00000000, 0x2A3000FF, 0x1B1B1B1B, \
					       0x10101010, 0x00001111, 0x04380006, 0x000003AA, 0xF0000000, 0x001020FF, 0x000003AA, 0xF0000000, 0x001020FF};

		*/

		/* Adjust ATPL360_REG_DACC_TABLE_CFG for FCC/ARIB with PLCOUP011 */
		
		/* Commenting this out for DV13335A implementation because FCC is now using EMIT0, EMIT1, and TXRX0 instead of EMIT2, EMIT3, and TXRX1
		
		memcpy(s_state.m_pl360G3Pib.m_au8Value, (uint8_t *)pul_dacc_c11_cfg, sizeof(pul_dacc_c11_cfg));
		s_state.m_pl360G3Pib.m_u8Length = sizeof(pul_dacc_c11_cfg);
		s_state.m_pl360Desc.set_req(MAC_RT_PIB_MANUF_PHY_PARAM, ATPL360_PHY_DACC_TABLE_CFG, &s_state.m_pl360G3Pib);

		*/

		/* Mandatory in FCC/ARIB with PLCOUP011: Force Transmission to VLO mode because PLCOUP011 only has the VLO branch from PLCOUP006 */
		/* Disable autodetect mode */
		s_state.m_pl360G3Pib.m_au8Value[0] = 0;
		s_state.m_pl360G3Pib.m_u8Length = 1;
		s_state.m_pl360Desc.set_req(MAC_RT_PIB_MANUF_PHY_PARAM, ATPL360_PHY_CFG_AUTODETECT_IMPEDANCE, &s_state.m_pl360G3Pib);

		/* Set VLO mode */
		s_state.m_pl360G3Pib.m_au8Value[0] = 2;
		s_state.m_pl360G3Pib.m_u8Length = 1;
		s_state.m_pl360Desc.set_req(MAC_RT_PIB_MANUF_PHY_PARAM, ATPL360_PHY_CFG_IMPEDANCE, &s_state.m_pl360G3Pib);
	}
#endif

	/********* The following lines show how to configure different parameters on PL360 device *********/
	/********* The user can customize it depending on the requirements ********************************/

	/* Force Transmission to VLO mode by default in order to maximize signal level in anycase */
	/* Disable autodetect mode */
	s_state.m_pl360G3Pib.m_au8Value[0] = 0;
	s_state.m_pl360G3Pib.m_u8Length = 1;
	s_state.m_pl360Desc.set_req(MAC_RT_PIB_MANUF_PHY_PARAM, ATPL360_PHY_CFG_AUTODETECT_IMPEDANCE, &s_state.m_pl360G3Pib);
	/* Set VLO mode */
	s_state.m_pl360G3Pib.m_au8Value[0] = 2;
	s_state.m_pl360G3Pib.m_u8Length = 1;
	s_state.m_pl360Desc.set_req(MAC_RT_PIB_MANUF_PHY_PARAM, ATPL360_PHY_CFG_IMPEDANCE, &s_state.m_pl360G3Pib);

#ifdef CONF_COUP_PARAMETERS
	/* Configure Coupling Parameters. Values defined in "conf_project.h" are the default values, calibrated for MCHP reference designs */
	/* For other Hardware designs, it may be needed to calibrate and obtain your own values. MCHP PHY Calibration Tool should be used */
#  ifdef CONF_ENABLE_MULTIBAND
	/* Multiband enabled: Different coupling configuration for each band (CENELEC-A or FCC) */
	if (s_state.m_phyBand == ATPL360_WB_FCC) {
		/* Take Coupling configuration for FCC */
		couplingConfig = (phy_coup_t *)&sx_pl360_coup_fcc;
	} else {
		/* Take Coupling configuration for CENELEC-A */
		couplingConfig = (phy_coup_t *)&sx_pl360_coup;
	}

#  else
	/* Take Coupling configuration for corresponding band (Multiband disabled) */
	couplingConfig = (phy_coup_t *)&sx_pl360_coup;
#  endif

	DV_info("[plc] Configuring PL360 device: Loading PHY Coupling parameters");

	/* Adjust your own parameters : ATPL360_REG_MAX_RMS_TABLE_HI. 32 bytes */
	memcpy(s_state.m_pl360G3Pib.m_au8Value, (uint8_t *)couplingConfig->pul_max_rms_hi, sizeof(couplingConfig->pul_max_rms_hi));
	s_state.m_pl360G3Pib.m_u8Length = sizeof(couplingConfig->pul_max_rms_hi);
	s_state.m_pl360Desc.set_req(MAC_RT_PIB_MANUF_PHY_PARAM, ATPL360_PHY_MAX_RMS_TABLE_HI, &s_state.m_pl360G3Pib);

	/* Adjust your own parameters : ATPL360_REG_MAX_RMS_TABLE_VLO. 32 bytes */
	memcpy(s_state.m_pl360G3Pib.m_au8Value, (uint8_t *)couplingConfig->pul_max_rms_vlo, sizeof(couplingConfig->pul_max_rms_vlo));
	s_state.m_pl360G3Pib.m_u8Length = sizeof(couplingConfig->pul_max_rms_vlo);
	s_state.m_pl360Desc.set_req(MAC_RT_PIB_MANUF_PHY_PARAM, ATPL360_PHY_MAX_RMS_TABLE_VLO, &s_state.m_pl360G3Pib);

	/* Adjust your own parameters : ATPL360_REG_THRESHOLDS_TABLE_HI. 64 bytes */
	memcpy(s_state.m_pl360G3Pib.m_au8Value, (uint8_t *)couplingConfig->pul_th1_hi, sizeof(couplingConfig->pul_th1_hi) << 1);
	s_state.m_pl360G3Pib.m_u8Length = sizeof(couplingConfig->pul_th1_hi) << 1;
	s_state.m_pl360Desc.set_req(MAC_RT_PIB_MANUF_PHY_PARAM, ATPL360_PHY_THRESHOLDS_TABLE_HI, &s_state.m_pl360G3Pib);

	/* Adjust your own parameters : ATPL360_REG_THRESHOLDS_TABLE_VLO. 64 bytes */
	memcpy(s_state.m_pl360G3Pib.m_au8Value, (uint8_t *)couplingConfig->pul_th1_vlo, sizeof(couplingConfig->pul_th1_vlo) << 1);
	s_state.m_pl360G3Pib.m_u8Length = sizeof(couplingConfig->pul_th1_vlo) << 1;
	s_state.m_pl360Desc.set_req(MAC_RT_PIB_MANUF_PHY_PARAM, ATPL360_PHY_THRESHOLDS_TABLE_VLO, &s_state.m_pl360G3Pib);

	/* Adjust your own parameters : ATPL360_REG_GAIN_TABLE_HI. 6 bytes */
	memcpy(s_state.m_pl360G3Pib.m_au8Value, (uint8_t *)couplingConfig->pus_ifft_gain_hi, sizeof(couplingConfig->pus_ifft_gain_hi));
	s_state.m_pl360G3Pib.m_u8Length = sizeof(couplingConfig->pus_ifft_gain_hi);
	s_state.m_pl360Desc.set_req(MAC_RT_PIB_MANUF_PHY_PARAM, ATPL360_PHY_GAIN_TABLE_HI, &s_state.m_pl360G3Pib);

	/* Adjust your own parameters : ATPL360_REG_GAIN_TABLE_VLO. 6 bytes */
	memcpy(s_state.m_pl360G3Pib.m_au8Value, (uint8_t *)couplingConfig->pus_ifft_gain_vlo, sizeof(couplingConfig->pus_ifft_gain_vlo));
	s_state.m_pl360G3Pib.m_u8Length = sizeof(couplingConfig->pus_ifft_gain_vlo);
	s_state.m_pl360Desc.set_req(MAC_RT_PIB_MANUF_PHY_PARAM, ATPL360_PHY_GAIN_TABLE_VLO, &s_state.m_pl360G3Pib);

	/* Adjust your own parameters : ATPL360_REG_NUM_TX_LEVELS. 1 byte */
	s_state.m_pl360G3Pib.m_au8Value[0] = couplingConfig->uc_num_tx_levels;
	s_state.m_pl360G3Pib.m_u8Length = 1;
	s_state.m_pl360Desc.set_req(MAC_RT_PIB_MANUF_PHY_PARAM, ATPL360_PHY_NUM_TX_LEVELS, &s_state.m_pl360G3Pib);

	/* Adjust your own parameters : ATPL360_REG_DACC_TABLE_CFG. 68 bytes */
	memcpy(s_state.m_pl360G3Pib.m_au8Value, (uint8_t *)couplingConfig->pul_dacc_cfg, sizeof(couplingConfig->pul_dacc_cfg));
	s_state.m_pl360G3Pib.m_u8Length = sizeof(couplingConfig->pul_dacc_cfg);
	s_state.m_pl360Desc.set_req(MAC_RT_PIB_MANUF_PHY_PARAM, ATPL360_PHY_DACC_TABLE_CFG, &s_state.m_pl360G3Pib);

	/* Adjust your own parameters : ATPL360_REG_PREDIST_COEF_TABLE_HI. (num_carriers * 2) bytes */
	memcpy(s_state.m_pl360G3Pib.m_au8Value, (uint8_t *)couplingConfig->pus_predist_coef_hi, spuc_num_carriers[s_state.m_phyBand - 1] << 1);
	s_state.m_pl360G3Pib.m_u8Length = spuc_num_carriers[s_state.m_phyBand - 1] << 1;
	s_state.m_pl360Desc.set_req(MAC_RT_PIB_MANUF_PHY_PARAM, ATPL360_PHY_PREDIST_COEF_TABLE_HI, &s_state.m_pl360G3Pib);

	/* Adjust your own parameters : ATPL360_REG_PREDIST_COEF_TABLE_VLO. (num_carriers * 2) bytes */
	memcpy(s_state.m_pl360G3Pib.m_au8Value, (uint8_t *)couplingConfig->pus_predist_coef_vlo, spuc_num_carriers[s_state.m_phyBand - 1] << 1);
	s_state.m_pl360G3Pib.m_u8Length = spuc_num_carriers[s_state.m_phyBand - 1] << 1;
	s_state.m_pl360Desc.set_req(MAC_RT_PIB_MANUF_PHY_PARAM, ATPL360_PHY_PREDIST_COEF_TABLE_VLO, &s_state.m_pl360G3Pib);
#endif

#ifdef CONF_TONEMASK_STATIC_NOTCHING
	/* Example to configure Tone Mask (Static Notching). Each carrier corresponding to the band can be notched (no energy is sent in those carriers) */
	/* Each carrier is represented by one byte (0: carrier used; 1: carrier notched). By default it is all 0's in PL360 device */
	/* The length is the number of carriers corresponding to the band in use (see "general_defs.h") */
	/* The same Tone Mask must be set in both transmitter and receiver. Otherwise they don't understand each other */
	memcpy(s_state.m_pl360G3Pib.m_au8Value, (uint8_t *)spuc_tone_mask, spuc_num_carriers[s_state.m_phyBand - 1] << 1);
	s_state.m_pl360G3Pib.m_u8Length = spuc_num_carriers[s_state.m_phyBand - 1] << 1;
	s_state.m_pl360Desc.set_req(MAC_RT_PIB_MANUF_PHY_PARAM, ATPL360_PHY_TONE_MASK, &s_state.m_pl360G3Pib);
#endif
}

static void setupTxParams(void) {

	/* Gain Resolution. Tx Gain resolution corresponding to one gain step. [0: 6dB, 1: 3dB]  */
	s_state.m_txParams.m_nTxRes = 1;

	/* Transmission Gain. Desired transmitter gain specifying how many gain steps are requested. Range value: 0 - 15. */
	s_state.m_txParams.m_nTxGain = 0;

	/* MAC High Priority. Set to 1 in order to set high priority for the next message to be transmitted */
	s_state.m_txParams.m_bHighPriority = 0;

	/* Tone Map Request. Set to 1 in order to force Tone Map response from the other side */
	s_state.m_txParams.m_bToneMapRequest = 0;

	/* Modulation Scheme. Differential or Coherent. See "enum ERtModulationScheme" in "MacRtDefs.h" file */
	/* Coherent Scheme supports worst SNR (about 3 dB) than Differential Scheme */
	/* Differential Scheme provides a bit higher data rate because Coherent Scheme uses some carriers for pilots */
	/* Coherent Scheme requires an accurate crystal oscillator. G3-PLC specifies that the frequency error must be less than 25 PPM */
	s_state.m_txParams.m_eModulationScheme = RT_MODULATION_SCHEME_DIFFERENTIAL;

	/* Modulation Type. See "enum ERtModulationType" in "MacRtDefs.h" file */
	/* Ordered from higher to lower data rate and from higher to lower required SNR (Signal to Noise Ratio): 8PSK, QPSK, BPSK, Robust BPSK */
	s_state.m_txParams.m_eModulationType = RT_MODULATION_DBPSK_BPSK;

	/* Configure Tone Map (Dynamic Notching) to use all carriers */
	/* Each bit corresponds to a subband */
	/* If the bit is '1' the subband is used to carry data. If the bit is '0' the subband does not carry data, but energy is sent in those carriers */
	/* The number of subbands is different in each G3 band (CENELEC-A, CENELEC-B, FCC, ARIB). See "general_defs.h" */
	/* The number of carriers per subband is 6 for CENELEC-A, 4 for CENELEC-B and 3 for FCC and ARIB */
	/* Full Tone Map: 0x3F0000 (CENELEC-A, 6 subbands); 0x0F0000 (CENELEC-B, 4 subbands); 0xFFFFFF (FCC, 24 subbands); 0xFFFF03 (ARIB, 18 subbands) */
	/* The next loop shows how to go across all subbands from lower to higher frequency */
	if (s_state.m_phyBand == ATPL360_WB_CENELEC_A) {
		s_state.m_txParams.m_ToneMap.m_au8Tm[0] = 0x3F;
		s_state.m_txParams.m_ToneMap.m_au8Tm[1] = 0x00;
		s_state.m_txParams.m_ToneMap.m_au8Tm[2] = 0x00;
	} else if (s_state.m_phyBand == ATPL360_WB_FCC) {
		s_state.m_txParams.m_ToneMap.m_au8Tm[0] = 0xFF;
		s_state.m_txParams.m_ToneMap.m_au8Tm[1] = 0xFF;
		s_state.m_txParams.m_ToneMap.m_au8Tm[2] = 0xFF;
	} else if (s_state.m_phyBand == ATPL360_WB_CENELEC_B) {
		s_state.m_txParams.m_ToneMap.m_au8Tm[0] = 0x0F;
		s_state.m_txParams.m_ToneMap.m_au8Tm[1] = 0x00;
		s_state.m_txParams.m_ToneMap.m_au8Tm[2] = 0x00;
	} else if (s_state.m_phyBand == ATPL360_WB_ARIB) {
		s_state.m_txParams.m_ToneMap.m_au8Tm[0] = 0x03;
		s_state.m_txParams.m_ToneMap.m_au8Tm[1] = 0xFF;
		s_state.m_txParams.m_ToneMap.m_au8Tm[2] = 0xFF;
	}

	/* Specifies the number of gain steps requested for the tones represented by Tone Map. Range value: 0 - 15. */
	/* TXCOEF[0] = Specifies the number of gain steps requested for the tones represented by ToneMap[0] & 0x0F */
	/* TXCOEF[1] = Specifies the number of gain steps requested for the tones represented by ToneMap[0] & 0xF0 */
	/* TXCOEF[2] = Specifies the number of gain steps requested for the tones represented by ToneMap[1] & 0x0F */
	/* TXCOEF[3] = Specifies the number of gain steps requested for the tones represented by ToneMap[1] & 0xF0 */
	/* TXCOEF[4] = Specifies the number of gain steps requested for the tones represented by ToneMap[2] & 0x0F */
	/* TXCOEF[5] = Specifies the number of gain steps requested for the tones represented by ToneMap[2] & 0xF0 */
	memset(s_state.m_txParams.m_au8TxCoef, 0, sizeof(s_state.m_txParams.m_au8TxCoef));
}

static void setMacSourceAddress(uint16_t address) {

	// Set Source Address 
	s_state.m_macRtHeader.m_SourceShortAddress = address;

	// Set PIB Short Address 
	memcpy(s_state.m_pl360G3Pib.m_au8Value, (uint8_t *)&s_state.m_macRtHeader.m_SourceShortAddress, 2);
	s_state.m_pl360G3Pib.m_u8Length = 2;
	s_state.m_pl360Desc.set_req(MAC_RT_PIB_SHORT_ADDRESS, 0, &s_state.m_pl360G3Pib);

	// Update Extended Address 
	spuc_extended_address[6] = (uint8_t)(address >> 8);
	spuc_extended_address[7] = (uint8_t)address;

	// Set PIB Extended Address
	memcpy(s_state.m_pl360G3Pib.m_au8Value, (uint8_t *)spuc_extended_address, 8);
	s_state.m_pl360G3Pib.m_u8Length = 8;
	s_state.m_pl360Desc.set_req(MAC_RT_PIB_MANUF_EXTENDED_ADDRESS, 0, &s_state.m_pl360G3Pib);
}

static void setMacDestAddress(uint16_t address) {
	s_state.m_macRtHeader.m_DestinationShortAddress = address;
}

static void setupMacRtHeader(void) {

    // MAC RT Frame Control information 
    s_state.m_macRtHeader.m_Fc.m_nFrameType = 1; // DAT
    s_state.m_macRtHeader.m_Fc.m_nSecurityEnabled = 0;
    s_state.m_macRtHeader.m_Fc.m_nFramePending = 0;
    s_state.m_macRtHeader.m_Fc.m_nAckRequest = 0;
    s_state.m_macRtHeader.m_Fc.m_nPanIdCompression = 1;
    s_state.m_macRtHeader.m_Fc.m_nReserved = 0;
    s_state.m_macRtHeader.m_Fc.m_nDestAddressingMode = 2; // 16b
    s_state.m_macRtHeader.m_Fc.m_nFrameVersion = 0; // 0 = 802.15.4_2003
    s_state.m_macRtHeader.m_Fc.m_nSrcAddressingMode = 2; // 16b

    // MAC RT Frame Header information 
    s_state.m_macRtHeader.m_u8SequenceNumber = 0;
    s_state.m_macRtHeader.m_nDestinationPanIdentifier = APP_G3_PAN_ID;
    s_state.m_macRtHeader.m_nSourcePanIdentifier = APP_G3_PAN_ID;

    uint8_t address = dvhandler_getCurrentAddress();
    if (address == TELEM_ADDRESS_UNASSIGNED) {
        setMacSourceAddress((uint16_t)(s_state.m_deviceId & 0xffff));
    }
    else {
        setMacSourceAddress((uint16_t)address);
    }
    setMacDestAddress(MAC_RT_SHORT_ADDRESS_BROADCAST);

    // Set PIB PAN ID Address 
    memcpy(s_state.m_pl360G3Pib.m_au8Value, (uint8_t *)&s_state.m_macRtHeader.m_nDestinationPanIdentifier, 2);
    s_state.m_pl360G3Pib.m_u8Length = 2;
    s_state.m_pl360Desc.set_req(MAC_RT_PIB_PAN_ID, 0, &s_state.m_pl360G3Pib);

    DV_info("[plc] Configuring G3 MAC RT Header");
    DV_info("[plc] G3 MAC PAN ID: 0x%04X", s_state.m_macRtHeader.m_nDestinationPanIdentifier);
    DV_info("[plc] G3 MAC Source Address: 0x%04X", s_state.m_macRtHeader.m_SourceShortAddress);
    DV_info("[plc] G3 MAC Destination Address: 0x%04X", s_state.m_macRtHeader.m_DestinationShortAddress);
}

static void handleDataConfirm(enum EMacRtStatus eStatus, bool bUpdateTimestamp, enum ERtModulationType eModType) {
    
    switch (eStatus) {
		case MAC_RT_STATUS_SUCCESS: break;
		case MAC_RT_STATUS_CHANNEL_ACCESS_FAILURE: DV_error("[plc] ...MAC_RT_STATUS_CHANNEL_ACCESS_FAILURE"); break;
		case MAC_RT_STATUS_NO_ACK: DV_error("[plc] ...MAC_RT_STATUS_NO_ACK"); break;
		case MAC_RT_STATUS_DENIED: DV_error("[plc] ...MAC_RT_STATUS_DENIED"); break;
		case MAC_RT_STATUS_INVALID_INDEX: DV_error("[plc] ...MAC_RT_STATUS_INVALID_INDEX"); break;
		case MAC_RT_STATUS_INVALID_PARAMETER: DV_error("[plc] ...MAC_RT_STATUS_INVALID_PARAMETER"); break;
		case MAC_RT_STATUS_TRANSACTION_OVERFLOW: DV_error("[plc] ...MAC_RT_STATUS_TRANSACTION_OVERFLOW"); break;
		case MAC_RT_STATUS_UNSUPPORTED_ATTRIBUTE: DV_error("[plc] ...MAC_RT_STATUS_UNSUPPORTED_ATTRIBUTE"); break;
		default: DV_error("[plc] MAC RT UNEXPECTED STATUS"); break;
	}

	// Update Sequence Number for next transmission
	s_state.m_macRtHeader.m_u8SequenceNumber++;
    s_state.m_transmitting = false;
}

static bool headerOkToReceive(uint8_t* pMrtsdu) {
    struct TAppMacRtMhr* macHeader = (struct TAppMacRtMhr *)pMrtsdu;
    if (macHeader->m_nDestinationPanIdentifier == s_state.m_macRtHeader.m_nSourcePanIdentifier) {
        if ((macHeader->m_DestinationShortAddress == s_state.m_macRtHeader.m_SourceShortAddress) ||
            (macHeader->m_DestinationShortAddress == MAC_RT_SHORT_ADDRESS_BROADCAST)){
            return true;
        }
    }
    return false;
}

static void handleNewRxParamReceived(struct TMacRtRxParameters *pParameters) {
    // Capture parameters of the new received message
    s_state.m_rxParams = *pParameters;
}

static void handleNewDataReceived(uint8_t* pMrtsdu, uint16_t u16MrtsduLen) {
    // Look for data. Skip MAC Header
    if (headerOkToReceive(pMrtsdu)) {
        int dataLength = (int)(u16MrtsduLen - sizeof(s_state.m_macRtHeader));
        uint8_t* dataBuffer = pMrtsdu + sizeof(s_state.m_macRtHeader);

	    // Show LQI (Link Quality Indicator). It is in quarters of dB and 10-dB offset: SNR(dB) = (LQI - 40) / 4 
	    DV_debug("[plc] Rx (%d bytes, LQI %ddB)", dataLength, div_round((int16_t)s_state.m_rxParams.m_u8PpduLinkQuality - 40, 4));
	    dvhandler_bytesFromPlc(dataBuffer, dataLength);
    }
}

static void handleException(atpl360_exception_t exception) {
    switch (exception) {
    case ATPL360_EXCEPTION_UNEXPECTED_SPI_STATUS:
        /* SPI has detected an unexpected status, reset is recommended */
        DV_error("[plc] ATPL360_EXCEPTION_UNEXPECTED_SPI_STATUS");
        break;

    case ATPL360_EXCEPTION_SPI_CRITICAL_ERROR:
        /* SPI critical error */
        DV_error("[plc] ATPL360_EXCEPTION_SPI_CRITICAL_ERROR");
        break;

    case ATPL360_EXCEPTION_RESET:
        // Device Reset
        if (s_state.m_enablingPl360) {
            // This callback is also called after loading binary at initization
            // This message is shown to indicate that the followitn exception is normal because PL360 binary has just been loaded
            s_state.m_enablingPl360 = false;
        }
        DV_info("[plc] PL360 initialization event: ATPL360_EXCEPTION_RESET");
        break;

    default:
        DV_error("[plc] ATPL360_EXCEPTION_UNKNOWN");
    }
    // Set flag to manage exception in app_plc_process() 
    s_state.m_exceptionPending = true;
}

static bool sendMessage(uint8_t* dataBuffer, uint16_t dataLength, uint8_t address, bool highPriority) {

    // Set TX parameters
    setMacDestAddress((address == TELEM_ADDRESS_BROADCAST) ? MAC_RT_SHORT_ADDRESS_BROADCAST : address);
    s_state.m_txParams.m_bHighPriority = highPriority ? 1 : 0;
    s_state.m_pl360Desc.mlme_set_request(&s_state.m_txParams);

    /* Build MAC RT DATA Frame */
    uint8_t* packetData = s_state.m_plcPacketBuffer;

    /* Insert MAC header information */
    memcpy(packetData, (uint8_t *)&s_state.m_macRtHeader, sizeof(s_state.m_macRtHeader));
    packetData += sizeof(s_state.m_macRtHeader);

    /* Insert Data Payload to be transmitted */
    memcpy(packetData, dataBuffer, dataLength);
    packetData += dataLength;

    /* Send PLC message. send_data returns ATPL360_SUCCESS if transmission was correctly programmed */
    /* The result will be reported in Tx Confirm (_handler_data_cfm) when message is completely sent */
    if (s_state.m_pl360Desc.tx_request(s_state.m_plcPacketBuffer, packetData - s_state.m_plcPacketBuffer) == ATPL360_SUCCESS) {
        DV_debug("[plc] Tx (%lu bytes): ", (uint32_t)(packetData - s_state.m_plcPacketBuffer));
        return true;
    } else {
        DV_error("[plc] Error sending PLC message: Transmission has not been programmed\r\n");
        return false;
    }
}

void dv_plc_init(int role, uint32_t deviceId) {

    // clear memory.
    memset(&s_state, 0, sizeof(s_state));

    s_state.m_role = role;
    s_state.m_deviceId = deviceId;
    s_state.m_txCurrentAddress = TELEM_ADDRESS_BROADCAST;
    
	atpl360_dev_callbacks_t callbacks;
	atpl360_hal_wrapper_t halWrapper;

	// Initialize G3 band static variable (ATPL360_WB defined in conf_atpl360.h) 
	s_state.m_phyBand = ATPL360_WB;

	// Initialize PL360 controller 
	halWrapper.plc_init = pplc_if_init;
	halWrapper.plc_reset = pplc_if_reset;
	halWrapper.plc_set_handler = pplc_if_set_handler;
	halWrapper.plc_send_boot_cmd = pplc_if_send_boot_cmd;
	halWrapper.plc_write_read_cmd = pplc_if_send_wrrd_cmd;
	halWrapper.plc_enable_int = pplc_if_enable_interrupt;
	halWrapper.plc_delay = pplc_if_delay;
	atpl360_init(&s_state.m_pl360Desc, &halWrapper);

	// Callback functions configuration. Set NULL as Not used 
	callbacks.tx_confirm = handleDataConfirm;
	callbacks.data_indication = handleNewDataReceived;
	callbacks.mlme_get_cfm = handleNewRxParamReceived;
	callbacks.exception_event = handleException;
	s_state.m_pl360Desc.set_callbacks(&callbacks);

	// Enable PL360 device: Load binary 
	enablePlc360();
}

void dv_plc_setAddress(uint8_t address) {
    setMacSourceAddress((uint16_t)address);
    DV_info("[plc] G3 MAC Source Address: 0x%04X", s_state.m_macRtHeader.m_SourceShortAddress);       
}

void dv_plc_handleRxData(void) {

    // Manage PL360 exceptions. At initialization ATPL360_EXCEPTION_RESET is reported
	if (s_state.m_exceptionPending) {
    	// Clear exception flag 
    	s_state.m_exceptionPending = false;
    	// Set PL360 specific configuration from application 
    	// Called at initialization and if an exception occurs 
    	// If an exception occurs, PL360 is reset and some parameters may have to be reconfigured
    	setupPL360Config();
    	// Setup G3 MAC RT parameters to use in transmission 
    	setupTxParams();
    	// Setup G3 MAC RT header
    	setupMacRtHeader();

	    DV_info("[plc] PL360 reset occurred.");
        s_state.m_transmitting = false;
	}

	// Check ATPL360 pending events. It must be called from application periodically to handle G3 MAC RT events
	atpl360_handle_events();
}    

void dv_plc_handleTxData() {

    // first we want to copy bytes into our local fifo buffer from the handler.
    int fifoBytesAvailable = (int)(sizeof(s_state.m_txFifoBuffer) - s_state.m_txFifoBufferOffset);
    int fifoBytesRead = dvhandler_bytesToPlc(s_state.m_txFifoBuffer + s_state.m_txFifoBufferOffset, fifoBytesAvailable);
    if (fifoBytesRead > 0) {
        s_state.m_txFifoBufferOffset += fifoBytesRead;
    }

    // next we want to decode bytes from the fifo into a packet and append
    // that onto the current packet buffer if it matches the current address
    if ((s_state.m_txFifoBufferOffset > 0) && (!s_state.m_flushToSwitchAddress)) {
        size_t srcOffset = 0;
        if (decodeTelemPacketStaticFromBuffer(&s_state.m_txPacket, s_state.m_txFifoBuffer, s_state.m_txFifoBufferOffset, &srcOffset) == DECODE_RESULT_OK) {
            // we can encode the data on the buffer if we are not changing address, or we have
            // no address already then encode the result packet on the txCurrentAddressBuffer
            bool hasSameAddress = s_state.m_txPacket.m_header.m_destAddr == s_state.m_txCurrentAddress;
            bool hasNoPendingData = s_state.m_txCurrentAddressBufferOffset == 0;
            if (hasSameAddress || hasNoPendingData) {
                if (encodeTelemPacketStaticToBuffer(&s_state.m_txPacket, s_state.m_txCurrentAddressBuffer, sizeof(s_state.m_txCurrentAddressBuffer), &(s_state.m_txCurrentAddressBufferOffset))) {
                    // consume the bytes read from the fifo buffer.
                    memcpy(s_state.m_txFifoBuffer, s_state.m_txFifoBuffer + srcOffset, (size_t)(srcOffset));
                    s_state.m_txFifoBufferOffset -= srcOffset;
                    s_state.m_txCurrentAddress = s_state.m_txPacket.m_header.m_destAddr;
                    s_state.m_flushToSwitchAddress = false;
                }
            }
            else {
                // flush until the address changes...
                s_state.m_flushToSwitchAddress = true;
            }
        }        
    }

    // finally, when we aren't transmitting create a packet from the 
    // current address buffer and then send it via the PLC network.
    if (!s_state.m_transmitting) {
        bool shouldSendHighPriority = (s_state.m_txCurrentAddressBufferOffset + s_state.m_txFifoBufferOffset) > HIGH_PRIORITY_WHEN_GREATER_THAN_BYTES_PENDING;
        size_t bytesToSend = min(MAC_RT_MAX_PAYLOAD_SIZE, s_state.m_txCurrentAddressBufferOffset);
        if (bytesToSend > 0) {

            // append the end of transmission sentinel to the end of the packet
            // if this is a small packet or a end of a larger transmission packet.
            if ((bytesToSend < MAC_RT_MAX_PAYLOAD_SIZE) && (s_state.m_txCurrentAddressBuffer[bytesToSend-1] != DV_TELEM_FRAME_BYTE)) {
                if (encodeEndOfTransmissionToBuffer(s_state.m_txCurrentAddressBuffer, sizeof(s_state.m_txCurrentAddressBuffer), &s_state.m_txCurrentAddressBufferOffset)) {
                    ++bytesToSend;
                }
            }
            
            // send the packet through the PLC network...
            if (sendMessage(s_state.m_txCurrentAddressBuffer, (uint16_t)bytesToSend, s_state.m_txCurrentAddress, shouldSendHighPriority)) {
                memcpy(s_state.m_txCurrentAddressBuffer, s_state.m_txCurrentAddressBuffer + bytesToSend, bytesToSend);
                s_state.m_txCurrentAddressBufferOffset -= bytesToSend;
                s_state.m_transmitting = true;
            }
        }
    }
}

DV_warnPop()