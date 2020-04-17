#include "DvTelemPacket.h"
#include "DvTelemUtils.h"

void initCommandPacket(DvTelemPacket* packet, uint32_t srcAddress, uint32_t destAddress, uint32_t length) {
    DV_ensureAlignment(packet, DV_alignOf(DvTelemPacket));
    DV_ensure(srcAddress < DV_TELEM_MAX_ADDRESSES);
    DV_ensure(destAddress < DV_TELEM_MAX_ADDRESSES);
    DV_ensure(length <= DV_TELEM_PACKET_MAX_PAYLOAD_LENGTH);
    packet->m_header.m_type = DV_TELEM_PACKET_TYPE_COMMAND;
    packet->m_header.m_sourceAddr = srcAddress;
    packet->m_header.m_destAddr = destAddress;
    packet->m_header.m_length = length;   
}

void initAckPacket(DvTelemPacket* packet, uint32_t srcAddress, uint32_t destAddress, uint32_t length) {
    DV_ensureAlignment(packet, DV_alignOf(DvTelemPacket));
    DV_ensure(srcAddress < DV_TELEM_MAX_ADDRESSES);
    DV_ensure(destAddress < DV_TELEM_MAX_ADDRESSES);
    DV_ensure(length <= DV_TELEM_PACKET_MAX_PAYLOAD_LENGTH);
    packet->m_header.m_type = DV_TELEM_PACKET_TYPE_ACK;
    packet->m_header.m_sourceAddr = srcAddress;
    packet->m_header.m_destAddr = destAddress;
    packet->m_header.m_length = length;
}

void initNackPacket(DvTelemPacket* packet, uint32_t srcAddress, uint32_t destAddress, uint32_t length) {
    DV_ensureAlignment(packet, DV_alignOf(DvTelemPacket));
    DV_ensure(srcAddress < DV_TELEM_MAX_ADDRESSES);
    DV_ensure(destAddress < DV_TELEM_MAX_ADDRESSES);
    DV_ensure(length <= DV_TELEM_PACKET_MAX_PAYLOAD_LENGTH);
    packet->m_header.m_type = DV_TELEM_PACKET_TYPE_NACK;
    packet->m_header.m_sourceAddr = srcAddress;
    packet->m_header.m_destAddr = destAddress;
    packet->m_header.m_length = length;   
}

int byteSizeFromTelemHeader(const DvTelemHeader* header) {
    DV_ensure(header != nullptr);
    return (int)(sizeof(DvTelemHeader) + header->m_length);
}

int byteSizeFromTelemPacket(const DvTelemPacket* packet) {
    DV_ensure(packet != nullptr);
    return byteSizeFromTelemHeader(&(packet->m_header));
}

uint16_t calcTelemPacketCrc(const DvTelemPacket* packet) {
    return calcTelemDataCrc((void*)packet, byteSizeFromTelemPacket(packet));
}