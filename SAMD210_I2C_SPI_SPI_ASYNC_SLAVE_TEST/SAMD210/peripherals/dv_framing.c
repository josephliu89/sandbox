#include <atmel_start.h>
#include "dv_framing.h"

// from: https://barrgroup.com/Embedded-Systems/How-To/CRC-Calculation-C-Code
/**********************************************************************
* Filename:    crc.h & crc.c
* Copyright (c) 2000 by Michael Barr.  This software is placed into
* the public domain and may be used for any purpose.  However, this
* notice must not be changed or removed and no warranty is either
* expressed or implied by its publication or distribution.
**********************************************************************/

#define POLYNOMIAL			0x8FDB
#define INITIAL_REMAINDER	0xFFFF
#define FINAL_XOR_VALUE		0x0000
#define REFLECT_DATA		FALSE
#define REFLECT_REMAINDER	FALSE
#define WIDTH    (8 * sizeof(uint16_t))
#define TOPBIT   (1 << (WIDTH - 1))

//#if (REFLECT_DATA == TRUE)
//#undef  REFLECT_DATA
//#define REFLECT_DATA(X)			((unsigned char) reflect((X), 8))
//#else
#undef  REFLECT_DATA
#define REFLECT_DATA(X)			(X)
//#endif

//#if (REFLECT_REMAINDER == TRUE)
//#undef  REFLECT_REMAINDER
//#define REFLECT_REMAINDER(X)	((uint16_t) reflect((X), WIDTH))
//#else
#undef  REFLECT_REMAINDER
#define REFLECT_REMAINDER(X)	(X)
//#endif

//static unsigned long reflect(unsigned long data, unsigned char nBits) {
//	unsigned long  reflection = 0x00000000;
//	unsigned char  bit;
//	for (bit = 0; bit < nBits; ++bit) {
//		if (data & 0x01) {
//			reflection |= (1 << ((nBits - 1) - bit));
//		}
//		data = (data >> 1);
//	}
//	return (reflection);
//}

static uint16_t s_crcTable[256];

static void crcInit(void) {
    uint16_t remainder = 0;
    unsigned char bit = 0;
    for (int dividend = 0; dividend < 256; ++dividend) {
        remainder = (uint16_t)(dividend << (WIDTH - 8));
        for (bit = 8; bit > 0; --bit) {
            if (remainder & TOPBIT) {
                remainder = (uint16_t)((remainder << 1) ^ POLYNOMIAL);
            }
            else {
                remainder = (uint16_t)(remainder << 1);
            }
        }
        s_crcTable[dividend] = remainder;
    }
}

uint16_t calcDataCrc(const void* data, int size) {
    static bool initCrc = false;
    if (!initCrc) {
        crcInit();
        initCrc = true;
    }
    unsigned char* const message = (unsigned char*)data;
    uint16_t remainder = (uint16_t)(INITIAL_REMAINDER);
    for (int byte = 0; byte < size; ++byte) {
        unsigned char index = (unsigned char)(REFLECT_DATA(message[byte]) ^ (remainder >> (WIDTH - 8)));
        remainder = (uint16_t)(s_crcTable[index] ^ (remainder << 8));
    }
    return (uint16_t)(REFLECT_REMAINDER(remainder) ^ FINAL_XOR_VALUE);
}

static uint8_t escapeByte(uint8_t byte) {
    return (uint8_t)(byte ^ FRAME_INVERT_MASK);
}

// returns the size in bytes of the source buffer encoded.
static size_t encodeFrameInByteStream(const uint8_t* src, size_t srcLength, uint8_t* dest, size_t destLength) {

    size_t bytesEncoded = 0;
    dest[bytesEncoded++] = FRAME_BYTE;
    const uint8_t* srcEnd = src + srcLength;
    while ((src < srcEnd) && (bytesEncoded < destLength)) {
        if ((*src == FRAME_BYTE) || (*src == FRAME_ESCAPE_BYTE)) {
            dest[bytesEncoded++] = FRAME_ESCAPE_BYTE;
            dest[bytesEncoded++] = escapeByte(*src);
        }
        else {
            dest[bytesEncoded++] = *src;
        }
        ++src;
    }

    // if not enough bytes to encode this stream return zero.
    if ((bytesEncoded == destLength) && (src != srcEnd)) {
        return 0;
    }
    return bytesEncoded;
}

// walks the byte stream and attempts to decode the framing bytes
static bool decodeFrameFromByteStream(const uint8_t* src, size_t srcLength, uint8_t* dest, size_t destLength, size_t* srcOffset, size_t* destOffset) {

    const uint8_t* srcStart = src;
    const uint8_t* srcEnd = src + srcLength;

    // start at the current source offset in the buffer...
    src += (*srcOffset);

    // are we at the end of the buffer?
    if (src == srcEnd) {
        return false;
    }

    // walk through the data until we hit the start of a frame.
    while ((src < srcEnd) && (*src != FRAME_BYTE)) {
        ++src;
        ++(*srcOffset);
    }

    // if we have a valid frame start packet start decoding
    if (*src == FRAME_BYTE) {

        // found the frame, copy bytes into the destination buffer;
        ++src;
        while (*src == FRAME_BYTE) { ++src; }
        while ((src < srcEnd) && (*src != FRAME_BYTE)) {
            if ((*src == FRAME_ESCAPE_BYTE) && ((src + 1) < srcEnd)) {
                ++src;
                dest[(*destOffset)++] = escapeByte(*src);
            }
            else if (*src != FRAME_ESCAPE_BYTE) {
                dest[(*destOffset)++] = *src;
            }
            else {
                // we are at the end of the source buffer.
                break;
            }
            ++src;
        }

        // only increment the source offset if we have a completed a
        // valid frame packet and have rolled around to the next packet
        (void)(destLength);
        //assert((*destOffset) <= destLength);
        if (*src == FRAME_BYTE) {
            (*srcOffset) = (size_t)(src - srcStart);
            return true;
        }
    }
    return false;
}

bool encodePacketToBuffer(commPacket* packet, uint8_t* destBuffer, size_t destLength, size_t* destOffset) {

    // compute and embed the CRC values in the packet buffer.
    uint8_t* sourceToEncode = (uint8_t*)packet;
    const int bytesWithHeader = 5;  // header + device address + slave address + data0 + data1
    const size_t bytesToEncode = bytesWithHeader + 2; 
    const uint16_t crc16 = calcDataCrc(packet, bytesWithHeader);
    memcpy(sourceToEncode + bytesWithHeader, &crc16, sizeof(crc16));

    // minimum encode size requires several escape bytes.
    const size_t minimumEncodedSize = bytesToEncode + 2;
    const size_t bytesRemainingInBuffer = (destLength >= (*destOffset)) ? destLength - (*destOffset) : 0;
    if (minimumEncodedSize <= bytesRemainingInBuffer) {
        // bytes written returning zero means there is not enough room in the buffer to encode this packet.
        const size_t bytesWritten = encodeFrameInByteStream(sourceToEncode, bytesToEncode, destBuffer + (*destOffset), destLength - (*destOffset));
        if (bytesWritten > 0) {
            (*destOffset) += bytesWritten;
            return true;
        }
    }
    return false;
}

bool encodeEndOfTransmissionToBuffer(uint8_t* destBuffer, size_t destLength, size_t* destOffset) {
    // end of frames are signaled by a FRAME_BYTE, when we have a final packet we
    // want to book-end it with a frame sentinel byte.
    if ((*destOffset) < destLength) {
        destBuffer[(*destOffset)++] = FRAME_BYTE;
        return true;
    }
    return false;
}

DecodeResult decodePacketFromBuffer(commPacket* packet, size_t packetBufferLength, const uint8_t* src, size_t srcLength, size_t* srcOffset) {

    // attempt to find a valid framing entry in the data.
    size_t decodeBufferOffset = 0;
    if (decodeFrameFromByteStream(src, srcLength, (uint8_t*)packet, packetBufferLength, srcOffset, &decodeBufferOffset)) {

        // each packet should include at a minimum a header and a footer...
        const size_t minForCrc = 3;
        if ((decodeBufferOffset >= minForCrc) && (decodeBufferOffset <= PACKET_MAX_SIZE)) {

            // check that the length and the crc are valid.
            const int packetByteSize = 5;
            if (packetByteSize + 2 == decodeBufferOffset) {
                const uint16_t crc16 = calcDataCrc(packet, 5);
                const uint8_t* packetCrc = (uint8_t*)(packet) + packetByteSize;
                if ((packetCrc[0] == (uint8_t)(crc16 & 0xff)) && (packetCrc[1] == (uint8_t)(crc16 >> 8))) {
                    return DECODE_RESULT_OK;
                }
                else {
                    return DECODE_FAILED_CRC;
                }
            }
            else {
                return DECODE_INVALID_LENGTH;
            }
        }
        else {
            return DECODE_INVALID_LENGTH;
        }
    }
    return DECODE_INCOMPLETE_BUFFER;
}
