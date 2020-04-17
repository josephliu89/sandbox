/**********************************************************************************************************************/
/** \addtogroup MacSublayer
 * @{
 **********************************************************************************************************************/

/**********************************************************************************************************************/
/** This file contains data types and functions of the MAC Real Time RX/TX API.
 ***********************************************************************************************************************
 *
 * @file
 *
 **********************************************************************************************************************/

#ifndef MAC_RT_H_
#define MAC_RT_H_

#include <MacRtDefs.h>

struct TMacRtTxParameters {
  uint8_t m_nTxGain;
  uint8_t m_au8TxCoef[6];
  uint8_t m_nTxRes;
  enum ERtModulationType m_eModulationType;
  enum ERtModulationScheme m_eModulationScheme;
  struct TRtToneMap m_ToneMap;
  bool m_bHighPriority;
  bool m_bToneMapRequest;
};

struct TMacRtRxParameters {
  bool m_bToneMapRequest;
  bool m_bHighPriority;
  uint8_t m_u8PpduLinkQuality;
  uint8_t m_u8PhaseDifferential;
  enum ERtModulationType m_eModulationType;
  enum ERtModulationScheme m_eModulationScheme;
  struct TRtToneMap m_ToneMap;
};

typedef void (*MacRtDataIndication)(uint8_t *pMrtsdu, uint16_t u16MrtsduLen);
typedef void (*MacRtTxConfirm)(enum EMacRtStatus eStatus, bool bUpdateTimestamp, enum ERtModulationType eModType);
typedef void (*MacRtlmeGetConfirm)(struct TMacRtRxParameters *pParameters);

struct TMacRtNotifications {
  MacRtDataIndication m_pDataIndication;
  MacRtTxConfirm m_pMacRtTxConfirm;
  MacRtlmeGetConfirm m_pMacRtlmeGetConfirm;
};

void MacRtInitialize(uint8_t u8Band, struct TMacRtNotifications *pNotifications, uint8_t u8SpecCompliance);
void MacRtEventHandler(void);

void MacRtlmeSetRequest(struct TMacRtTxParameters *pParameters);
void MacRtTxRequest(uint8_t *pTxSdu, uint16_t u16TxSduLen);
void MacRtResetRequest(bool bResetMib);
void MacRtGetToneMapResponseData(struct TRtToneMapResponseData *pParameters);
uint32_t MacRtGetPhyTime(void);

#endif

/**********************************************************************************************************************/
/** @}
 **********************************************************************************************************************/
