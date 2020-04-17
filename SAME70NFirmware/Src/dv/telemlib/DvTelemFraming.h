#ifndef DV_TELEM_FRAMING_H
#define DV_TELEM_FRAMING_H

#include "DvTelemPacket.h"

//
// Serial Framing Protocol
//
// several serial line protocols exist that use the concept of "byte-stuffing", 
// where there are special markers in the data that represent the start and end of a frame.  
//
// Examples are:
//
//   High-Level Data Link Control (https://en.m.wikipedia.org/wiki/High-Level_Data_Link_Control)
//   Serial Line Internet Protocol (https://en.m.wikipedia.org/wiki/Serial_Line_Internet_Protocol)
//   Point-to-Point Protocol (https://en.m.wikipedia.org/wiki/Point-to-Point_Protocol)
//
// Typically these mechanisms, in addition to packet framing have additional header structures 
// and framing definition, but they are all quite similar.  Essentially packet frames are 
// coupled with special byte characters which mark the start and end of packets.  
//
// For instance PPP uses a control byte of 0x7E to denote the beginning and end of a frame, 
// if a 0x7E exists in the data packet it is "escaped" with a 0x7D before the data character.
//
// This framing protocol uses the same basic framing as PPP, but with our own header structure.
//

#define DV_TELEM_FRAME_BYTE (0x7E)
#define DV_TELEM_FRAME_ESCAPE_BYTE (0x7D)
#define DV_TELEM_FRAME_INVERT_MASK (0x10)

// encode a packet into a destination buffer.
bool encodeTelemPacketToBuffer(DvTelemPacket* packet, uint8_t* destBuffer, size_t destLength, size_t* destOffset);
bool encodeTelemPacketStaticToBuffer(DvTelemPacketStatic* packet, uint8_t* destBuffer, size_t destLength, size_t* destOffset);

// after packet stream is complete, a final end of transmission byte should be added.
// this is analogous to a "flush", ending the in-progress packet in the stream.
bool encodeEndOfTransmissionToBuffer(uint8_t* destBuffer, size_t destLength, size_t* destOffset);

typedef enum {
    DECODE_RESULT_OK,        // successfully decoded a single packet
    DECODE_FAILED_CRC,       // packet length matches expected length, but crc error detected.
    DECODE_INVALID_LENGTH,   // packet framing was too short or too long.
    DECODE_INCOMPLETE_BUFFER // end of packet not detected yet, wait for more data.
} DvTelemDecodeResult;

DvTelemDecodeResult decodeTelemPacketFromBuffer(DvTelemPacket* packet, size_t packetBufferLength, const uint8_t* src, size_t srcLength, size_t* srcOffset);
DvTelemDecodeResult decodeTelemPacketStaticFromBuffer(DvTelemPacketStatic* packet, const uint8_t* src, size_t srcLength, size_t* srcOffset);

#endif // DV_TELEM_FRAMING_H
