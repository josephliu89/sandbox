/**********************************************************************************************************************/
/** \addtogroup MacSublayer
 * @{
 **********************************************************************************************************************/

/**********************************************************************************************************************/
/** This file contains the MAC MIB.
 ***********************************************************************************************************************
 *
 * @file
 *
 **********************************************************************************************************************/

#ifndef MAC_RT_MIB_H_
#define MAC_RT_MIB_H_

#include <MacRtDefs.h>

enum EMacRtPibAttribute {
  MAC_RT_PIB_ACK_WAIT_DURATION = 0x00000040,
  MAC_RT_PIB_MAX_BE = 0x00000047,
  MAC_RT_PIB_MAX_CSMA_BACKOFFS = 0x0000004E,
  MAC_RT_PIB_MIN_BE = 0x0000004F,
  MAC_RT_PIB_PAN_ID = 0x00000050,
  MAC_RT_PIB_SHORT_ADDRESS = 0x00000053,
  MAC_RT_PIB_MAX_FRAME_RETRIES = 0x00000059,
  MAC_RT_PIB_HIGH_PRIORITY_WINDOW_SIZE = 0x00000100,
  MAC_RT_PIB_CSMA_NO_ACK_COUNT = 0x00000106,
  MAC_RT_PIB_BAD_CRC_COUNT = 0x00000109,
  MAC_RT_PIB_CSMA_FAIRNESS_LIMIT = 0x0000010C,
  MAC_RT_PIB_TONE_MASK = 0x00000110,
  MAC_RT_PIB_A = 0x00000112,
  MAC_RT_PIB_K = 0x00000113,
  MAC_RT_PIB_MIN_CW_ATTEMPTS = 0x00000114,
  MAC_RT_PIB_CENELEC_LEGACY_MODE = 0x00000115,
  MAC_RT_PIB_FCC_LEGACY_MODE = 0x00000116,
  MAC_RT_PIB_BROADCAST_MAX_CW_ENABLE = 0x0000011E,
  MAC_RT_PIB_TRANSMIT_ATTEN = 0x0000011F,
  // manufacturer specific
  // Extended address of this node.
  MAC_RT_PIB_MANUF_EXTENDED_ADDRESS = 0x08000001,
  // returns the maximum number of tones used by the band
  MAC_RT_PIB_MANUF_BAND_INFORMATION = 0x08000003,
  // Forces Modulation Scheme in every transmitted frame
  // 0 - Not forced, 1 - Force Differential, 2 - Force Coherent
  MAC_RT_PIB_MANUF_FORCED_MOD_SCHEME = 0x08000007,
  // Forces Modulation Type in every transmitted frame
  // 0 - Not forced, 1 - Force BPSK_ROBO, 2 - Force BPSK, 3 - Force QPSK, 4 - Force 8PSK
  MAC_RT_PIB_MANUF_FORCED_MOD_TYPE = 0x08000008,
  // Forces ToneMap in every transmitted frame
  // {0} - Not forced, other value will be used as tonemap
  MAC_RT_PIB_MANUF_FORCED_TONEMAP = 0x08000009,
  // Gets number of discarded packets due to Segment Decode Error
  MAC_RT_PIB_MANUF_RX_SEGMENT_DECODE_ERROR_COUNT = 0x0800001C,
  // Gets or Sets number of retires left before forcing ROBO mode
  MAC_RT_PIB_MANUF_RETRIES_LEFT_TO_FORCE_ROBO = 0x0800001F,
  // Gets internal MAC RT version
  MAC_RT_PIB_MANUF_MAC_RT_INTERNAL_VERSION = 0x08000022,
  // IB used to set the complete MIB structure at once
  MAC_RT_PIB_GET_SET_ALL_MIB = 0x08000100,
  // Gets or sets a parameter in Phy layer. Index will be used to contain PHY parameter ID.
  // Check 'enum EPhyParam' below for available Phy parameter IDs
  MAC_RT_PIB_MANUF_PHY_PARAM = 0x08000020
};

enum EPhyParam {
  // Phy layer version number. 32 bits.
  PHY_PARAM_VERSION = 0x010c,
  // Correctly transmitted frame count. 32 bits.
  PHY_PARAM_TX_TOTAL = 0x0110,
  // Transmitted bytes count. 32 bits.
  PHY_PARAM_TX_TOTAL_BYTES = 0x0114,
  // Transmission errors count. 32 bits.
  PHY_PARAM_TX_TOTAL_ERRORS = 0x0118,
  // Transmission failure due to already in transmission. 32 bits.
  PHY_PARAM_BAD_BUSY_TX = 0x011C,
  // Transmission failure due to busy channel. 32 bits.
  PHY_PARAM_TX_BAD_BUSY_CHANNEL = 0x0120,
  // Bad len in message (too short - too long). 32 bits.
  PHY_PARAM_TX_BAD_LEN = 0x0124,
  // Message to transmit in bad format. 32 bits.
  PHY_PARAM_TX_BAD_FORMAT = 0x0128,
  // Timeout error in transmission. 32 bits.
  PHY_PARAM_TX_TIMEOUT = 0x012C,
  // Received correctly messages count. 32 bits.
  PHY_PARAM_RX_TOTAL = 0x0130,
  // Received bytes count. 32 bits.
  PHY_PARAM_RX_TOTAL_BYTES = 0x0134,
  // Reception RS errors count. 32 bits.
  PHY_PARAM_RX_RS_ERRORS = 0x0138,
  // Reception Exceptions count. 32 bits.
  PHY_PARAM_RX_EXCEPTIONS = 0x013C,
  // Bad len in message (too short - too long). 32 bits.
  PHY_PARAM_RX_BAD_LEN = 0x0140,
  // Bad CRC in received FCH. 32 bits.
  PHY_PARAM_RX_BAD_CRC_FCH = 0x0144,
  // CRC correct but invalid protocol. 32 bits.
  PHY_PARAM_RX_FALSE_POSITIVE = 0x0148,
  // Received message in bad format. 32 bits.
  PHY_PARAM_RX_BAD_FORMAT = 0x014C,
  // Time between noise captures (in ms). 32 bits.
  PHY_PARAM_TIME_BETWEEN_NOISE_CAPTURES = 0x0158,
  // Auto detect impedance
  PHY_PARAM_CFG_AUTODETECT_BRANCH = 0x0161,
  // Manual impedance configuration
  PHY_PARAM_CFG_IMPEDANCE = 0x0162,
  // Indicate if notch filter is active or not. 8 bits.
  PHY_PARAM_RRC_NOTCH_ACTIVE = 0x0163,
  // Index of the notch filter. 8 bits.
  PHY_PARAM_RRC_NOTCH_INDEX = 0x0164,
  // Enable periodic noise autodetect and adaptation. 8 bits.
  PHY_PARAM_ENABLE_AUTO_NOISE_CAPTURE = 0x0166,
  // Noise detection timer reloaded after a correct reception. 8 bits.
  PHY_PARAM_DELAY_NOISE_CAPTURE_AFTER_RX = 0x0167,
  // Disable PLC Tx/Rx. 8 bits.
  PHY_PARAM_PLC_DISABLE = 0x016A,
  // Indicate noise power in dBuV for the noisier carrier
  PHY_PARAM_NOISE_PEAK_POWER = 0x016B,
  // LQI value of the last received message
  PHY_PARAM_LAST_MSG_LQI = 0x016C,
  // LQI value of the last received message
  PHY_PARAM_LAST_MSG_RSSI = 0x016D,
  // Success transmission of ACK packets
  PHY_PARAM_ACK_TX_CFM = 0x016E,
  // Inform PHY layer about enabled modulations on TMR
  PHY_PARAM_TONE_MAP_RSP_ENABLED_MODS = 0x0174
};

struct TMacRtMib {
  uint32_t m_u32CsmaNoAckCount;
  uint32_t m_u32BadCrcCount;
  uint32_t m_u32RxSegmentDecodeErrorCount;
  uint16_t m_nPanId;
  uint16_t m_nShortAddress;
  struct TRtToneMask m_ToneMask;
  struct TExtAddress m_ExtendedAddress;
  struct TRtToneMap m_ForcedToneMap;
  uint8_t m_u8HighPriorityWindowSize;
  uint8_t m_u8CsmaFairnessLimit;
  uint8_t m_u8A;
  uint8_t m_u8K;
  uint8_t m_u8MinCwAttempts;
  uint8_t m_u8MaxBe;
  uint8_t m_u8MaxCsmaBackoffs;
  uint8_t m_u8MaxFrameRetries;
  uint8_t m_u8MinBe;
  uint8_t m_u8ForcedModScheme;
  uint8_t m_u8ForcedModType;
  uint8_t m_u8RetriesToForceRobo;
  uint8_t m_u8TransmitAtten;
  bool m_bBroadcastMaxCwEnable;
  bool m_bCoordinator;
};

#define MAC_RT_PIB_MAX_VALUE_LENGTH (144)

struct TMacRtPibValue {
  uint8_t m_u8Length;
  uint8_t m_au8Value[MAC_RT_PIB_MAX_VALUE_LENGTH];
};

enum EMacRtStatus MacRtGetRequestSync(enum EMacRtPibAttribute eAttribute, uint16_t u16Index, struct TMacRtPibValue *pValue);
enum EMacRtStatus MacRtSetRequestSync(enum EMacRtPibAttribute eAttribute, uint16_t u16Index, const struct TMacRtPibValue *pValue);

void MacRtMibReset(void);
void MacRtMibInitialize(uint8_t u8SpecCompliance);
void MacRtSetCoordinator(void);

#endif

/**********************************************************************************************************************/
/** @}
 **********************************************************************************************************************/
