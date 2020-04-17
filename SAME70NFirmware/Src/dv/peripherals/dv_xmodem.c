#include "dv_xmodem.h"
#include "sysclk.h"

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

typedef struct {
    uint8_t     startOfHeader;              // start of header
    uint8_t     packetNumber;               // packet number
    uint8_t     packetNumberCompl;          // complement of packet number
    uint8_t     data[128];                  // packet data
    uint16_t    crc16;                      // 16-bit CRC of packet data
} xmodemPacket;

void initiateXmodem(DvTelemFirmwareUpdateData* outgoingData) {
    char charVal = 'C';
    strncpy(outgoingData->m_data, &charVal, 1);
}

xmdmResult decodeXmodemPacket(DvTelemPacket* packet, DvTelemFirmwareUpdateData* outPacket, uint8_t decodedXmdmData[], uint8_t* expectedPacketNum, uint32_t *progSize) {    
    xmodemPacket xmdmPacket;
    
    xmdmPacket.startOfHeader        = *(packet->m_payload);
    xmdmPacket.packetNumber         = *(packet->m_payload + 1);
    xmdmPacket.packetNumberCompl    = *(packet->m_payload + 2);
    memcpy(xmdmPacket.data, packet->m_payload + 3, PKTLEN_128);
    uint16_t crcEndian              = *(packet->m_payload + 131);
    xmdmPacket.crc16                = ((crcEndian & 0x00FF) << 8) | ((crcEndian & 0xFF00) >> 8);
    
    /* Check integrity of packet */
    uint8_t c_char;
    uint16_t packet_crc = 0;
    uint8_t invPacketNumber = ~(*expectedPacketNum);
    
    /* Calculate CRC16 for 128-byte data only */
	for (uint8_t i = 0; i < PKTLEN_128; ++i) {
    	c_char = xmdmPacket.data[i];
    	packet_crc = packet_crc ^ (int32_t) c_char << 8;
    	for (uint8_t j = 0; j < 8; j++) {
        	if (packet_crc & 0x8000) {
            	packet_crc = packet_crc << 1 ^ CRC16POLY;
            	} else {
            	packet_crc = packet_crc << 1;
        	}
    	}
    	packet_crc = packet_crc & 0xFFFF;
	}
    
    /* Check CRC */
    if (packet_crc != xmdmPacket.crc16) {
        DV_error("XMODEM CRC does not match!");
        return XMDM_FAILED_CRC;
    }
    /* Check packet sequence number */
    else if ((xmdmPacket.packetNumber != *expectedPacketNum) || (xmdmPacket.packetNumberCompl != invPacketNumber)) {
        DV_error("XMODEM CRC does not match!");
        return XMDM_FAILED_SEQ;        
    }
    /* All is good, proceed with analyzing XMODEM packet */
    else { 
        switch(xmdmPacket.startOfHeader) {
        
        /* Start of transfer */
        case XMDM_SOH: 
            memcpy(decodedXmdmData, xmdmPacket.data, PKTLEN_128);
            expectedPacketNum++;
            *(outPacket->m_data) = XMDM_ACK;
            *progSize += PKTLEN_128;
            return XMDM_RESULT_OK;
            break;
        
        /* End of transfer */    
        case XMDM_EOT:
            *(outPacket->m_data) = XMDM_ACK;
            outPacket->m_action = FIRMWARE_UPDATE_FINISH;
            return XMDM_RESULT_EOT;
            break;
        
        case XMDM_CAN:
        case XMDM_ESC:
        default:
            *(outPacket->m_data) = XMDM_ACK;
            outPacket->m_action = FIRMWARE_UPDATE_ABORT;
            DV_info("Firmware update aborted!");
            return XMDM_RESULT_ABORT;            
            break;            
        }        
    }    
}

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond

