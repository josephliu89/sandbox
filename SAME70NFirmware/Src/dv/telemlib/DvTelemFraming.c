
#include "DvTelemFraming.h"

static uint8_t escapeByte(uint8_t byte) {
    return (uint8_t)(byte ^ DV_TELEM_FRAME_INVERT_MASK);
}

// returns the size in bytes of the source buffer encoded.
static size_t encodeFrameInByteStream(const uint8_t* src, size_t srcLength, uint8_t* dest, size_t destLength) {

    size_t bytesEncoded = 0;
    dest[bytesEncoded++] = DV_TELEM_FRAME_BYTE;
    const uint8_t* srcEnd = src + srcLength;
    while ((src < srcEnd) && (bytesEncoded < destLength)) {
        if ((*src == DV_TELEM_FRAME_BYTE) || (*src == DV_TELEM_FRAME_ESCAPE_BYTE)) {
            dest[bytesEncoded++] = DV_TELEM_FRAME_ESCAPE_BYTE;
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
    while ((src < srcEnd) && (*src != DV_TELEM_FRAME_BYTE)) {
        ++src;
        ++(*srcOffset);
    }

    // if we have a valid frame start packet start decoding
    if (*src == DV_TELEM_FRAME_BYTE) {

        // found the frame, copy bytes into the destination buffer;
        ++src;
        while (*src == DV_TELEM_FRAME_BYTE) { ++src; }            
        while ((src < srcEnd) && (*src != DV_TELEM_FRAME_BYTE)) {
            if ((*src == DV_TELEM_FRAME_ESCAPE_BYTE) && ((src + 1) < srcEnd)) {
                ++src;
                dest[(*destOffset)++] = escapeByte(*src);
            }
            else if (*src != DV_TELEM_FRAME_ESCAPE_BYTE) {
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
        DV_unused(destLength);
        DV_ensure((*destOffset) <= destLength);
        if (*src == DV_TELEM_FRAME_BYTE) {
            (*srcOffset) = (size_t)(src - srcStart);
            return true;
        }
    }
    return false;
}

bool encodeTelemPacketToBuffer(DvTelemPacket* packet, uint8_t* destBuffer, size_t destLength, size_t* destOffset) {

    // compute and embed the CRC values in the packet buffer.
    uint8_t* sourceToEncode = (uint8_t*)packet;
    const int bytesWithHeader = byteSizeFromTelemPacket(packet);
    const size_t bytesToEncode = bytesWithHeader + sizeof(DvTelemFooter);
    const uint16_t crc16 = calcTelemPacketCrc(packet);
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

bool encodeTelemPacketStaticToBuffer(DvTelemPacketStatic* packet, uint8_t* destBuffer, size_t destLength, size_t* destOffset) {
    return encodeTelemPacketToBuffer((DvTelemPacket*)packet, destBuffer, destLength, destOffset);
}

bool encodeEndOfTransmissionToBuffer(uint8_t* destBuffer, size_t destLength, size_t* destOffset) {
    // end of frames are signaled by a FRAME_BYTE, when we have a final packet we 
    // want to book-end it with a frame sentinel byte.
    if ((*destOffset) < destLength) {
        destBuffer[(*destOffset)++] = DV_TELEM_FRAME_BYTE;
        return true;
    }
    return false;
}

DvTelemDecodeResult decodeTelemPacketFromBuffer(DvTelemPacket* packet, size_t packetBufferLength, const uint8_t* src, size_t srcLength, size_t* srcOffset) {

    // attempt to find a valid framing entry in the data.
    size_t decodeBufferOffset = 0;
    if (decodeFrameFromByteStream(src, srcLength, (uint8_t*)packet, packetBufferLength, srcOffset, &decodeBufferOffset)) {

        // each packet should include at a minimum a header and a footer...
        const size_t minForCrc = sizeof(DvTelemHeader) + sizeof(DvTelemFooter);
        if ((decodeBufferOffset >= minForCrc) && (decodeBufferOffset <= DV_TELEM_PACKET_MAX_SIZE)) {

            // check that the length and the crc are valid.
            const int packetByteSize = byteSizeFromTelemPacket(packet);
            if (packetByteSize + sizeof(DvTelemFooter) == decodeBufferOffset) {
                const uint16_t crc16 = calcTelemPacketCrc(packet);
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

DvTelemDecodeResult decodeTelemPacketStaticFromBuffer(DvTelemPacketStatic* packet, const uint8_t* src, size_t srcLength, size_t* srcOffset) {
    return decodeTelemPacketFromBuffer((DvTelemPacket*)packet, sizeof(DvTelemPacketStatic), src, srcLength, srcOffset);
}
