#ifndef DV_TELEM_PACKET_H
#define DV_TELEM_PACKET_H

#include "DvTelemInterop.h"

DV_warnPush()
DV_warnDisableMSVC(4214) // nonstandard extension used: bit field types other than int
DV_warnDisableMSVC(4200) // nonstandard extension used: zero-sized array in struct/union

// Packet Type
typedef enum {
    DV_TELEM_PACKET_TYPE_DATA,    // general pass through data packet, not actionable by telemetry MCUs
    DV_TELEM_PACKET_TYPE_COMMAND, // command packet actionable by the telemetry system
    DV_TELEM_PACKET_TYPE_ACK,     // respond with an ACK for a specific command, payload body should include command being acked
    DV_TELEM_PACKET_TYPE_NACK,    // respond with a NACK for a specific command, payload body should include command being nacked
} DvTelemPacketType;

#define DV_TELEM_MAX_ADDRESSES 16

// Address up to 15 unique devices...
typedef enum {
    TELEM_ADDRESS_UPLINK = 0x0,
    TELEM_ADDRESS_BROADCAST = 0xf,
    TELEM_ADDRESS_UNASSIGNED = 0xf,
} DvTelemAddressType;

// Packet constants
#define DV_TELEM_PACKET_HEADER_SIZE 4
#define DV_TELEM_PACKET_FOOTER_SIZE 2
#define DV_TELEM_PACKET_MAX_PAYLOAD_LENGTH 4095
#define DV_TELEM_PACKET_MIN_SIZE_FOR_LENGTH  DV_TELEM_PACKET_HEADER_SIZE
#define DV_TELEM_PACKET_MAX_SIZE (DV_TELEM_PACKET_HEADER_SIZE + DV_TELEM_PACKET_MAX_PAYLOAD_LENGTH + DV_TELEM_PACKET_FOOTER_SIZE)
#define DV_TELEM_PACKET_SEQUENCE_MOD_VALUE (1 << 6)

// 4 byte header. If we need resolution on the packet addressing
// we can shift the bits around which will make the structure less
// user friendly to use, but worth keeping this small.
typedef struct {
    uint32_t m_type : 2;          // packet type, see enumeration...
    uint32_t m_sequence : 6;      // packet sequence, (rolling 64 numbers sequential between telemetry links)
    uint32_t m_destAddr : 4;      // packet destination address
    uint32_t m_sourceAddr : 4;    // packet source address
    uint32_t m_length : 16;       // data length in bytes max(4095) use.
} DvTelemHeader;

// footer comes after the data in the same stream as the payload.
typedef struct {
    uint16_t m_crc16;  // crc16 of packet header + data
} DvTelemFooter;

// generalized packet structure
// payload is C99 standard flexible array member pointing to the data
// immediately following it in the structure
typedef struct {
    DvTelemHeader m_header;
    uint8_t m_payload[];	
} DvTelemPacket;

void initCommandPacket(DvTelemPacket* packet, uint32_t srcAddress, uint32_t destAddress, uint32_t length);
void initAckPacket(DvTelemPacket* packet, uint32_t srcAddress, uint32_t destAddress, uint32_t length);
void initNackPacket(DvTelemPacket* packet, uint32_t srcAddress, uint32_t destAddress, uint32_t length);


// static sized structure used as a helper object.
typedef struct {
    DvTelemHeader m_header;
    uint8_t m_payload[DV_TELEM_PACKET_MAX_SIZE];
} DvTelemPacketStatic;

// Utility Functions...
int byteSizeFromTelemHeader(const DvTelemHeader* header);
int byteSizeFromTelemPacket(const DvTelemPacket* packet);
uint16_t calcTelemPacketCrc(const DvTelemPacket* packet);

DV_warnPop()

#endif // DV_TELEM_PACKET_H 