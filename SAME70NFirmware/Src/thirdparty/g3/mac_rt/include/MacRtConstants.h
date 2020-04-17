/**********************************************************************************************************************/
/** \addtogroup MacSublayer
 * @{
 **********************************************************************************************************************/

/**********************************************************************************************************************/
/** This file contains constants used by the MAC layer.
 ***********************************************************************************************************************
 *
 * @file
 *
 **********************************************************************************************************************/

#ifndef MAC_RT_CONSTANTS_H_
#define MAC_RT_CONSTANTS_H_

extern uint32_t g_u32MacSlotTime;
extern uint32_t g_u32MacCifsTime;
extern uint32_t g_u32MacCifsRetransmitTime;
extern uint32_t g_u32MacRifsTime;
extern uint32_t g_u32MacIdleSpacingTime;
extern uint32_t g_u32MacAckTime;
extern uint32_t g_u32MacEifsTime;
extern uint32_t g_u32MacAckWaitDuration;

struct TRtToneMask;
extern struct TRtToneMask g_ToneMaskDefault;
extern struct TRtToneMask g_ToneMaskNotched;
extern struct TRtToneMap g_RtToneMapDefault;

struct TPhyBandInformation {
  uint16_t m_u16FlMax;
  uint8_t m_u8Band;
  uint8_t m_u8Tones;
  uint8_t m_u8Carriers;
  uint8_t m_u8TonesInCarrier;
  uint8_t m_u8FlBand;
  uint8_t m_u8MaxRsBlocks;
  uint8_t m_u8TxCoefBits;
  uint8_t m_u8PilotsFreqSpa;
};
extern struct TPhyBandInformation g_PhyBandInformation;

void MacRtConstantsInitialize(uint8_t u8Band, struct TRtToneMask *pToneMask);

#endif

/**********************************************************************************************************************/
/** @}
 **********************************************************************************************************************/
