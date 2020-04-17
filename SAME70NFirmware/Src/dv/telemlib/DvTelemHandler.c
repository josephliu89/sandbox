
#include "DvTelemHandler.h"
#include "DvTelemFraming.h"

#include <stdio.h>

DV_warnPush()
DV_warnDisableGCC("-Wcast-align")
DV_warnDisableGCC("-Wunused-parameter")

void initTelemChannel(DvTelemChannel* state, uint8_t* rxBuffer, size_t rxBufferSize, uint8_t* txBuffer, size_t txBufferSize) {
    DV_ensureAlignment(state, DV_alignOf(DvTelemChannel));
    memset(state, 0, sizeof(DvTelemChannel));
    state->m_rxBuffer = rxBuffer;
    state->m_rxBufferSize = rxBufferSize;
    state->m_txBuffer = txBuffer;
    state->m_txBufferSize = txBufferSize;
    for (int i = 0; i < DV_TELEM_MAX_ADDRESSES; ++i) {
        state->m_rxSequence[i] = -1;
        state->m_txSequence[i] = -1;
    }
}

void initTelemHandlerState(DvTelemHandlerState* state, DvTelemDefaultRouteFn routeFn) {
    DV_ensureAlignment(state, DV_alignOf(DvTelemHandlerState));
    memset(state, 0, sizeof(DvTelemHandlerState));
    state->m_routeFn = routeFn;
    state->m_address = TELEM_ADDRESS_UNASSIGNED;
    initDeviceStatsData(&(state->m_statsData));
    initDeviceStatsState(&(state->m_statsState));
}

void attachTelemChannel(DvTelemHandlerState* state, uint8_t address, DvTelemChannel* channel) {
    DV_ensureAlignment(state, DV_alignOf(DvTelemHandlerState));
    DV_ensure(address < DV_TELEM_MAX_ADDRESSES);
    state->m_channels[address] = channel;
}

void attachDataHandler(DvTelemHandlerState* state, DvTelemOnPacketFn dataFn, void* userData) {
    DV_ensureAlignment(state, DV_alignOf(DvTelemHandlerState));
    state->m_dataFn = dataFn;
    state->m_dataUserData = userData;
}

void attachCmdHandler(DvTelemHandlerState* state, DvTelemCmdType command, DvTelemOnPacketFn cmdFn, void* userData) {
    DV_ensureAlignment(state, DV_alignOf(DvTelemHandlerState));
    DV_ensure(command < DV_TELEM_MAX_COMMANDS);
    DvTelemHandlerInfo* handler = &state->m_handlers[command];
    handler->m_cmdFn = cmdFn;
    handler->m_cmdUserData = userData;
    handler->m_initialized = true;
}

void attachAckHandler(DvTelemHandlerState* state, DvTelemCmdType command, DvTelemOnPacketFn ackFn, void* userData) {
    DV_ensureAlignment(state, DV_alignOf(DvTelemHandlerState));
    DV_ensure(command < DV_TELEM_MAX_COMMANDS);
    DvTelemHandlerInfo* handler = &state->m_handlers[command];
    handler->m_ackFn = ackFn;
    handler->m_ackUserData = userData;
    handler->m_initialized = true;
}
void attachNackHandler(DvTelemHandlerState* state, DvTelemCmdType command, DvTelemOnPacketFn nackFn, void* userData) {
    DV_ensureAlignment(state, DV_alignOf(DvTelemHandlerState));
    DV_ensure(command < DV_TELEM_MAX_COMMANDS);
    DvTelemHandlerInfo* handler = &state->m_handlers[command];
    handler->m_nackFn = nackFn;
    handler->m_nackUserData = userData;
    handler->m_initialized = true;
}

void attachUpdateFunction(DvTelemHandlerState* state, DvTelemCmdType command, DvTelemOnUpdateFn updateFn, void* userData) {
    DV_ensureAlignment(state, DV_alignOf(DvTelemHandlerState));
    DV_ensure(command < DV_TELEM_MAX_COMMANDS);
    DvTelemHandlerInfo* handler = &state->m_handlers[command];
    handler->m_updateFn = updateFn;
    handler->m_updateUserData = userData;
    handler->m_initialized = true;
}

static void updateRxStats(DvTelemHandlerState* state, DvTelemChannel* channel, DvTelemPacket* packet, DvTelemDecodeResult result) {
    DV_ensureAlignment(state, DV_alignOf(DvTelemHandlerState));
    DV_ensureAlignment(channel, DV_alignOf(DvTelemChannel));
    if (result == DECODE_RESULT_OK) {
        // track lost packets.  initialize the last sequence to negative one initially.
        int droppedPackets = 0;
        int lastReadSequence = channel->m_rxSequence[packet->m_header.m_sourceAddr];
        if (lastReadSequence >= 0) {

            int expectedSeq = ((lastReadSequence + 1) % DV_TELEM_PACKET_SEQUENCE_MOD_VALUE);
            int actualSeq = (int)(packet->m_header.m_sequence);
            if (actualSeq != expectedSeq) {
                droppedPackets = actualSeq - expectedSeq;
                if (actualSeq < expectedSeq) {
                    droppedPackets = actualSeq + ((DV_TELEM_PACKET_SEQUENCE_MOD_VALUE - 1) - expectedSeq);
                }
            }
        }

        // accumulate packet counts.
        state->m_statsData.m_rxStats.m_totalPacketCount += (uint16_t)(1 + droppedPackets);
        state->m_statsData.m_rxStats.m_droppedPacketCount += (uint16_t)droppedPackets;
        channel->m_rxSequence[packet->m_header.m_sourceAddr] = packet->m_header.m_sequence;
    }
    else if (result == DECODE_FAILED_CRC) {
        state->m_statsData.m_rxStats.m_crcErrors++;
    }
    else if (result == DECODE_INVALID_LENGTH) {
        state->m_statsData.m_rxStats.m_packetLengthErrors++;
    }
}

static DvTelemHandlerResult executeHandlerForPacket(DvTelemHandlerState* state, DvTelemChannel* channel, DvTelemPacket* decodedPacket) {
    DV_ensureAlignment(state, DV_alignOf(DvTelemHandlerState));
    DV_ensureAlignment(channel, DV_alignOf(DvTelemChannel));
    DV_ensureAlignment(decodedPacket, DV_alignOf(DvTelemPacket));
    if (decodedPacket->m_header.m_type == DV_TELEM_PACKET_TYPE_DATA) {
        DV_ensure(state->m_dataFn != nullptr);
        return (state->m_dataFn)(state, decodedPacket, channel, state->m_dataUserData);
    }
    else if (decodedPacket->m_header.m_length >= sizeof(DvTelemCommandAndVersion)) {
        // decode the command type and version, all command packets must
        // start with this structure
        DvTelemCommandAndVersion* commandType = (DvTelemCommandAndVersion*)decodedPacket->m_payload;
        DV_ensureAlignment(commandType, DV_alignOf(DvTelemCommandAndVersion));       
        if (commandType->m_type < DV_TELEM_MAX_COMMANDS) {
            // call a specific handler function if the handler for this command was initialized.
            DvTelemHandlerInfo* handler = &(state->m_handlers[commandType->m_type]);
            if (handler->m_initialized) {

                // handle the command type.
                switch (decodedPacket->m_header.m_type) {
                case DV_TELEM_PACKET_TYPE_COMMAND:
                    if (handler->m_cmdFn != nullptr) {
                        return (handler->m_cmdFn)(state, decodedPacket, channel, handler->m_cmdUserData);
                    }
                    return HANDLER_RESULT_OK;
                case DV_TELEM_PACKET_TYPE_ACK:
                    if (handler->m_ackFn != nullptr) {
                        return (handler->m_ackFn)(state, decodedPacket, channel, handler->m_ackUserData);
                    }
                    return HANDLER_RESULT_OK;
                case DV_TELEM_PACKET_TYPE_NACK:
                    if (handler->m_nackFn != nullptr) {
                        return (handler->m_nackFn)(state, decodedPacket, channel, handler->m_nackUserData);
                    }
                    return HANDLER_RESULT_OK;
                default:
                    DV_ensure(false);
                    break;
                }
            }
            else {
                // perform pass through non-data packets if there was no handler registered for this type.
                DvTelemChannel* destChannel = getChannelRouteForPacket(state, decodedPacket);
                return encodePacketOnChannel(state, decodedPacket, destChannel);
            }
        }
    }
    else {
        // packet length is too short to be a command or an ack packet.
        state->m_statsData.m_rxStats.m_packetLengthErrors++;
    }
    return HANDLER_RESULT_ERROR;
}

static void decodePacketsOnChannel(DvTelemHandlerState* state, DvTelemChannel* channel) {

    size_t readOffset = 0;
    size_t finalReadOffset = 0;
    DvTelemHandlerResult handlerResults = HANDLER_RESULT_OK;
    DvTelemPacket* decodedPacket = (DvTelemPacket*)&(state->m_decodePacket);

    // parse packets out of the channel rx buffer until we 
    // have read all of the packets or we don't have enough
    // room in the output channel to send the packet.

    DV_ensureAlignment(state, DV_alignOf(DvTelemHandlerState));
    DV_ensureAlignment(channel, DV_alignOf(DvTelemChannel));

    DvTelemDecodeResult decodeResults = decodeTelemPacketFromBuffer(decodedPacket, sizeof(state->m_decodePacket), channel->m_rxBuffer, channel->m_rxBufferOffset, &readOffset);
    while ((decodeResults != DECODE_INCOMPLETE_BUFFER) && (handlerResults != HANDLER_RESULT_OUTPUT_BUFFER_FULL)) {

        updateRxStats(state, channel, decodedPacket, decodeResults);
        if (decodeResults == DECODE_RESULT_OK) {
            DV_debug("[telem] decode packet, src: %d, dest: %d, type: %d, length: %d", decodedPacket->m_header.m_sourceAddr, decodedPacket->m_header.m_destAddr, decodedPacket->m_header.m_type, decodedPacket->m_header.m_length);
            handlerResults = executeHandlerForPacket(state, channel, decodedPacket);
            // "consume" this packet if we were able to output the result or there was an error.
            if (handlerResults != HANDLER_RESULT_OUTPUT_BUFFER_FULL) {
                finalReadOffset = readOffset;
            }
        }
        decodeResults = decodeTelemPacketFromBuffer(decodedPacket, sizeof(state->m_decodePacket), channel->m_rxBuffer, channel->m_rxBufferOffset, &readOffset);
    }

    // accumulate bytes read stats...
    state->m_statsData.m_rxStats.m_bytes += (uint32_t)finalReadOffset;

    // after reading from the buffer "consume" the contents of the buffer and copy the
    // residual amount up to the front of the buffer for reading next time.
    if ((finalReadOffset > 0) && (channel->m_rxBufferOffset >= finalReadOffset)) {
        memcpy(channel->m_rxBuffer, channel->m_rxBuffer + finalReadOffset, (size_t)(channel->m_rxBufferOffset - finalReadOffset));
        channel->m_rxBufferOffset -= finalReadOffset;
    }
}

void processReceivedData(DvTelemHandlerState* state) {
    // process packets on both the TX and RX channels
    DV_ensureAlignment(state, DV_alignOf(DvTelemHandlerState));
    for (int i = 0; i < DV_TELEM_MAX_ADDRESSES; ++i) {
        DvTelemChannel* channel = state->m_channels[i];
        if ((channel != nullptr) && (channel->m_rxBufferOffset > 0)) {
            decodePacketsOnChannel(state, channel);
        }
    }
}

void updateHandlerStates(DvTelemHandlerState* state) {
    // call the update state functions for each update handler...
    DV_ensureAlignment(state, DV_alignOf(DvTelemHandlerState));
    for (int i = 0; i < DV_TELEM_MAX_COMMANDS; ++i) {
        DvTelemHandlerInfo* handler = &state->m_handlers[i];
        if (handler->m_updateFn != nullptr) {
            (handler->m_updateFn)(state, handler->m_updateUserData);
        }
    }
}

void finalizeTransmitData(DvTelemHandlerState* state) {
    // process packets on both the TX and RX channels
    DV_ensureAlignment(state, DV_alignOf(DvTelemHandlerState));
    for (int i = 0; i < DV_TELEM_MAX_ADDRESSES; ++i) {
        DvTelemChannel* channel = state->m_channels[i];
        if (channel != nullptr) {
            DV_ensure(channel->m_txBufferOffset <= channel->m_txBufferSize);
            if ((channel->m_txBufferOffset > 0) && (channel->m_txBuffer[channel->m_txBufferOffset-1] != DV_TELEM_FRAME_BYTE)) {
                if (encodeEndOfTransmissionToBuffer(channel->m_txBuffer, channel->m_txBufferSize, &channel->m_txBufferOffset)) {
                    state->m_statsData.m_txStats.m_bytes++;
                }
            }
        }
    }
}

DvTelemChannel* defaultLeafNodeRouting(DvTelemHandlerState* state, DvTelemPacket* packet) {
    // only one channel, tx and rx.
    DV_ensure(state->m_channels[0] != nullptr);
    DV_ensureAlignment(state, DV_alignOf(DvTelemHandlerState));
    DV_ensureAlignment(packet, DV_alignOf(DvTelemPacket));
    return state->m_channels[0];
}

DvTelemChannel* defaultPanCoordinatorRouting(DvTelemHandlerState* state, DvTelemPacket* packet) {
    // channel 0 is the uplink to the surface
    // channel 1 is the local PAN network to all the PLC devices.
    DV_ensureAlignment(state, DV_alignOf(DvTelemHandlerState));
    DV_ensureAlignment(packet, DV_alignOf(DvTelemPacket));
    if (packet->m_header.m_destAddr == TELEM_ADDRESS_UPLINK) {
        DV_ensure(state->m_channels[0] != nullptr);
        return state->m_channels[0];
    }
    else {
        DV_ensure(state->m_channels[1] != nullptr);
        return state->m_channels[1];
    }
}

DvTelemChannel* defaultPanSlaveRouting(DvTelemHandlerState* state, DvTelemPacket* packet) {
    // channel 0 is the downlink to the tool board.
    // channel 1 is the local PAN network to all the PLC devices.
    DV_ensureAlignment(state, DV_alignOf(DvTelemHandlerState));
    DV_ensureAlignment(packet, DV_alignOf(DvTelemPacket));
    if (packet->m_header.m_destAddr == (uint32_t)state->m_address) {
        DV_ensure(state->m_channels[0] != nullptr);
        return state->m_channels[0];
    }
    else {
        DV_ensure(state->m_channels[1] != nullptr);
        return state->m_channels[1];
    }
}

DvTelemChannel* defaultClientRouting(DvTelemHandlerState* state, DvTelemPacket* packet) {
    // channels in the client router all have one uplink channel
    // and then each attached tool has a channel for it's client side handler
    // so if a packet comes up with a destination of the uplink address then
    // we switch on the source address to find the corresponding destination.
    DV_ensureAlignment(state, DV_alignOf(DvTelemHandlerState));
    DV_ensureAlignment(packet, DV_alignOf(DvTelemPacket));
    if (packet->m_header.m_destAddr == TELEM_ADDRESS_UPLINK) {
        DV_ensure(state->m_channels[packet->m_header.m_sourceAddr] != nullptr);
        return state->m_channels[packet->m_header.m_sourceAddr];
    }
    else {
        DV_ensure(packet->m_header.m_sourceAddr == TELEM_ADDRESS_UPLINK);
        DV_ensure(state->m_channels[TELEM_ADDRESS_UPLINK] != nullptr);
        return state->m_channels[TELEM_ADDRESS_UPLINK];
    }
}

DvTelemHandlerResult defaultRouterDataFn(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData) {
    // perform pass through for router functions
    DvTelemChannel* destChannel = getChannelRouteForPacket(state, packet);
    return encodePacketOnChannel(state, packet, destChannel);
}

DvTelemChannel* getChannelRouteForPacket(DvTelemHandlerState* state, DvTelemPacket* packet) {
    DV_ensure(state->m_routeFn != nullptr);
    return (state->m_routeFn(state, packet));
}

DvTelemHandlerResult encodePacketOnChannel(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel) {

    DV_ensureAlignment(state, DV_alignOf(DvTelemHandlerState));
    DV_ensureAlignment(packet, DV_alignOf(DvTelemPacket));
    DV_ensureAlignment(channel, DV_alignOf(DvTelemChannel));

    DvTelemHandlerResult results = HANDLER_RESULT_OK;
    int lastWriteSequence = channel->m_txSequence[packet->m_header.m_sourceAddr];
    packet->m_header.m_sequence = (lastWriteSequence + 1) % DV_TELEM_PACKET_SEQUENCE_MOD_VALUE;

    size_t initialBufferOffset = channel->m_txBufferOffset;
    if (encodeTelemPacketToBuffer(packet, channel->m_txBuffer, channel->m_txBufferSize, &(channel->m_txBufferOffset))) {
        channel->m_txSequence[packet->m_header.m_sourceAddr] = packet->m_header.m_sequence;
        state->m_statsData.m_txStats.m_bytes += (uint32_t)(channel->m_txBufferOffset - initialBufferOffset);
        state->m_statsData.m_txStats.m_totalPacketCount += 1;
        results = HANDLER_RESULT_OK;
    }
    else {
        state->m_statsData.m_txStats.m_bufferFullErrors += 1;
        results = HANDLER_RESULT_OUTPUT_BUFFER_FULL;
    }
    return results;
}

DvTelemHandlerResult receiveDataFn_Printf(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData) {
    DV_info("[telem] data packet, from: %d, to: %d, seq: %d, length: %d", (int)packet->m_header.m_sourceAddr, (int)packet->m_header.m_destAddr, (int)packet->m_header.m_sequence, (int)packet->m_header.m_length);
    return HANDLER_RESULT_OK;
}

// logging handlers
DvTelemHandlerResult sendLoggingCommand(DvTelemHandlerState* state, const char* message, uint8_t address) {

    DvTelemPacket* packet = (DvTelemPacket*)&state->m_encodePacket;
    size_t maxStringSize = DV_TELEM_PACKET_MAX_PAYLOAD_LENGTH - sizeof(DvTelemLoggingData) - 1;
    size_t messageLength = (size_t)(strlen(message) + 1);
    size_t messagePayload = maxStringSize < messageLength ? maxStringSize : messageLength;

    DvTelemLoggingData* cmdData = (DvTelemLoggingData*)packet->m_payload;
    DV_ensureAlignment(cmdData, DV_alignOf(DvTelemLoggingData));  
    cmdData->m_command.m_type = COMMAND_LOGGING;
    cmdData->m_command.m_cookie = LOGGING_VERSION_COOKIE;
    strncpy(cmdData->m_message, message, messagePayload);

    initCommandPacket(packet, state->m_address, address, (uint16_t)(sizeof(DvTelemLoggingData) + messagePayload));
    return encodePacketOnChannel(state, packet, getChannelRouteForPacket(state, packet));
}

DvTelemHandlerResult receiveLoggingCommandFn_Printf(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData) {
    DvTelemLoggingData* cmdData = (DvTelemLoggingData*)packet->m_payload;
    DV_ensureAlignment(cmdData, DV_alignOf(DvTelemLoggingData));   
    if (cmdData->m_command.m_cookie == LOGGING_VERSION_COOKIE) {
        DV_info("[telem] log: %s", cmdData->m_message);
        return HANDLER_RESULT_OK;
    }
    else {
        DV_error("[telem] received logging message with bad cookie, expected %d got %d.", (int)LOGGING_VERSION_COOKIE, (int)cmdData->m_command.m_cookie);
        return HANDLER_RESULT_ERROR;
    }
}

DvTelemHandlerResult sendPingCommand(DvTelemHandlerState* state, DvTelemPingState* pingState, uint8_t address) {

    DvTelemPacket* packet = (DvTelemPacket*)&state->m_encodePacket;
    DvTelemCommandAndVersion* cmdData = (DvTelemCommandAndVersion*)packet->m_payload;
    DV_ensureAlignment(cmdData, DV_alignOf(DvTelemCommandAndVersion));   
    cmdData->m_type = COMMAND_PING;
    cmdData->m_cookie = PING_VERSION_COOKIE;

    // start the timer...
    initCommandPacket(packet, state->m_address, address, (uint16_t)(sizeof(DvTelemCommandAndVersion)));
    restartStopWatch(&(pingState->m_pingTimers[address]));
    return encodePacketOnChannel(state, packet, getChannelRouteForPacket(state, packet));
}

DvTelemHandlerResult receivePingCmdFn(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData) {

    DvTelemPacket* encodePacket = (DvTelemPacket*)&state->m_encodePacket;
    memcpy(encodePacket, packet, byteSizeFromTelemPacket(packet));
    initAckPacket(encodePacket, packet->m_header.m_destAddr, packet->m_header.m_sourceAddr, packet->m_header.m_length);
    return encodePacketOnChannel(state, encodePacket, channel);
}

DvTelemHandlerResult receivePingAckFn_Printf(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData) {

    DvTelemPingState* pingState = (DvTelemPingState*)userData;
    DvTelemCommandAndVersion* cmdData = (DvTelemCommandAndVersion*)packet->m_payload;
    DV_ensureAlignment(cmdData, DV_alignOf(DvTelemCommandAndVersion));
    if (cmdData->m_cookie == PING_VERSION_COOKIE) {
        uint32_t pingTimeMs = elapsedMsStopWatch(&pingState->m_pingTimers[packet->m_header.m_sourceAddr]);
        DV_info("[telem] received ping message in %d milliseconds from address %d.", (int)pingTimeMs, (int)packet->m_header.m_sourceAddr);
        return HANDLER_RESULT_OK;
    }
    else {
        DV_error("[telem] received ping message with bad cookie, expected %d got %d.", (int)PING_VERSION_COOKIE, (int)cmdData->m_cookie);
        return HANDLER_RESULT_ERROR;
    }
}

void updateDeviceStatsFn(DvTelemHandlerState* state, void* userData) {
    
    if (elapsedMsStopWatch(&(state->m_statsState.m_transmitTimer)) > state->m_statsState.m_transmitPeriodMs) {
        
        DvTelemPacket* packet = (DvTelemPacket*)&state->m_encodePacket;
        DvTelemDeviceStatsData* cmdData = (DvTelemDeviceStatsData*)packet->m_payload;
        DV_ensureAlignment(cmdData, DV_alignOf(DvTelemDeviceStatsData));

        memcpy(cmdData, &(state->m_statsData), sizeof(DvTelemDeviceStatsData));

        cmdData->m_command.m_type = COMMAND_DEVICE_STATS;
        cmdData->m_command.m_cookie = DEVICE_STATS_VERSION_COOKIE;
        cmdData->m_stateTimerSeconds = elapsedMsStopWatch(&(state->m_statsState.m_stateTimer)) / 1000.0f;
        cmdData->m_rxStats.m_bitRate = (cmdData->m_rxStats.m_bytes * 8) / (elapsedMsStopWatch(&(state->m_statsState.m_bitRateTimer)) / 1000.0f);

        initCommandPacket(packet, state->m_address, TELEM_ADDRESS_UPLINK, (uint16_t)sizeof(DvTelemDeviceStatsData));
        DV_debug("[telem] sending stats to uplink...");       
        // if we failed to encode the uplink stats, dont' worry about it, we will send it next time.
        encodePacketOnChannel(state, packet, getChannelRouteForPacket(state, packet));
        restartStopWatch(&(state->m_statsState.m_transmitTimer));
    }
}

DvTelemHandlerResult receiveDeviceStatsCmdFn_Printf(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData) {

    DvTelemDeviceStatsData* cmdData = (DvTelemDeviceStatsData*)packet->m_payload;
    DV_ensureAlignment(cmdData, DV_alignOf(DvTelemDeviceStatsData));   
    if (cmdData->m_command.m_cookie == DEVICE_STATS_VERSION_COOKIE) {
        DV_info("[telem] device stats (%d): state: %d, %.2f secs", (int)(packet->m_header.m_sourceAddr), cmdData->m_state, cmdData->m_stateTimerSeconds);
        DV_info("[telem] device stats (%d): rx: %d bytes, %.2f bits/sec", (int)(packet->m_header.m_sourceAddr), (int)cmdData->m_rxStats.m_bytes, cmdData->m_rxStats.m_bitRate);
        DV_info("[telem] device stats (%d): rx: %d packets, %d dropped, %d crc errors, %d length errors", (int)(packet->m_header.m_sourceAddr), cmdData->m_rxStats.m_totalPacketCount, cmdData->m_rxStats.m_droppedPacketCount, cmdData->m_rxStats.m_crcErrors, cmdData->m_rxStats.m_packetLengthErrors);
        DV_info("[telem] device stats (%d): tx: %d bytes, %.2f bits/sec", (int)(packet->m_header.m_sourceAddr), (int)cmdData->m_txStats.m_bytes, cmdData->m_txStats.m_bitRate);
        DV_info("[telem] device stats (%d): tx: %d packets, %d buffer full errors", (int)(packet->m_header.m_sourceAddr), cmdData->m_txStats.m_totalPacketCount, cmdData->m_txStats.m_bufferFullErrors);
        return HANDLER_RESULT_OK;
    }
    else {
        DV_error("[telem] received device stats message with bad cookie, expected %d got %d.", (int)DEVICE_STATS_VERSION_COOKIE, (int)cmdData->m_command.m_cookie);
        return HANDLER_RESULT_ERROR;
    }
}

DvTelemHandlerResult sendDeviceStatsResetCommand(DvTelemHandlerState* state, uint8_t address) {
    DvTelemPacket* packet = (DvTelemPacket*)&state->m_encodePacket;
    DvTelemCommandAndVersion* cmdData = (DvTelemCommandAndVersion*)packet->m_payload;
    DV_ensureAlignment(cmdData, DV_alignOf(DvTelemCommandAndVersion));   
    cmdData->m_type = COMMAND_DEVICE_STATS_RESET;
    cmdData->m_cookie = DEVICE_STATS_RESET_VERSION_COOKIE;   
    initCommandPacket(packet, TELEM_ADDRESS_UPLINK, state->m_address, (uint16_t)sizeof(DvTelemCommandAndVersion));
    return encodePacketOnChannel(state, packet, getChannelRouteForPacket(state, packet));
}

DvTelemHandlerResult receiveDeviceStatsResetCmdFn(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData) {

    DvTelemCommandAndVersion* cmdData = (DvTelemCommandAndVersion*)packet->m_payload;
    DV_ensureAlignment(cmdData, DV_alignOf(DvTelemCommandAndVersion));   
    if (cmdData->m_cookie == DEVICE_STATS_RESET_VERSION_COOKIE) {
        resetDeviceStatsData(&(state->m_statsData));
        return HANDLER_RESULT_OK;
    }
    else {
        DV_error("[telem] received device stats reset message with bad cookie, expected %d got %d.", (int)DEVICE_STATS_RESET_VERSION_COOKIE, (int)cmdData->m_cookie);
        return HANDLER_RESULT_ERROR;
    }
}


DvTelemHandlerResult startNetwork(DvTelemHandlerState* state, DvTelemNetworkStartupState* startupState) {
    
    // initialize the state variables first time through...
    if (!startupState->m_active) {
        state->m_address = TELEM_ADDRESS_UPLINK;
        startupState->m_active = true;
        startupState->m_deviceCount = 1;
        startupState->m_deviceIds[0] = state->m_deviceId;
        restartStopWatch(&(startupState->m_timer));
    }
    
    DvTelemPacket* packet = (DvTelemPacket*)&state->m_encodePacket;
    initCommandPacket(packet, state->m_address, TELEM_ADDRESS_BROADCAST, (uint16_t)sizeof(DvTelemCommandAndVersion));
    initNetworkStartupData((DvTelemCommandAndVersion*)packet->m_payload);

    DV_debug("[telem] sending network startup packet...");
    return encodePacketOnChannel(state, packet, getChannelRouteForPacket(state, packet));
}

DvTelemHandlerResult receiveNetworkStartupCmdFn(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData) {
    DvTelemCommandAndVersion* cmdData = (DvTelemCommandAndVersion*)packet->m_payload;
    DV_ensureAlignment(cmdData, DV_alignOf(DvTelemCommandAndVersion));   
    if (cmdData->m_cookie == NETWORK_STARTUP_VERSION_COOKIE) {

        // if we have not received a valid address then respond to this request.
        if (state->m_address == TELEM_ADDRESS_UNASSIGNED) {

            DvTelemPacket* encodePacket = (DvTelemPacket*)&state->m_encodePacket;
            initCommandPacket(encodePacket, state->m_address, packet->m_header.m_sourceAddr, (uint16_t)sizeof(DvTelemJoinRequest));
            initNetworkJoinRequest((DvTelemJoinRequest*)encodePacket->m_payload, state->m_deviceId);
            
            DV_info("[telem] received network startup packet, sending join request with deviceId %x.", (unsigned int)state->m_deviceId);           
            return encodePacketOnChannel(state, encodePacket, channel);
        }
        else {
            return HANDLER_RESULT_OK;            
        }
    }    
    else {
        DV_error("[telem] received network startup message with bad cookie, expected %d got %d.", (int)NETWORK_STARTUP_VERSION_COOKIE, (int)cmdData->m_cookie);
        return HANDLER_RESULT_ERROR;
    }   
}

DvTelemHandlerResult receiveNetworkJoinCmdFn(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData) {
    DvTelemNetworkStartupState* startupState = (DvTelemNetworkStartupState*)userData;
    DvTelemJoinRequest* joinData = (DvTelemJoinRequest*)packet->m_payload;
    DV_ensureAlignment(joinData, DV_alignOf(DvTelemJoinRequest));   
    if (joinData->m_command.m_cookie == NETWORK_JOIN_VERSION_COOKIE) {
        
        DV_info("[telem] received network join request from deviceId %x.", (unsigned int)joinData->m_deviceUniqueId);
        
        DvTelemPacket* encodePacket = (DvTelemPacket*)&state->m_encodePacket;
        initAckPacket(encodePacket, state->m_address, TELEM_ADDRESS_BROADCAST, (uint16_t)sizeof(DvTelemJoinAcknowledge));

        // see if this device id exists or not...
        bool foundDevice = false;
        uint8_t address = 0;
        for (uint8_t i = 0; i < startupState->m_deviceCount; ++i) {
            if (startupState->m_deviceIds[i] == joinData->m_deviceUniqueId) {
                foundDevice = true;
                address = i;
                break;
            }
        }
        if (!foundDevice) {
            address = startupState->m_deviceCount;
            startupState->m_deviceIds[startupState->m_deviceCount] = joinData->m_deviceUniqueId;
            startupState->m_deviceCount++;
        }

        DV_info("[telem] sending join acknowledge, assigning deviceId %x to address %d.", (unsigned int)joinData->m_deviceUniqueId, address);
        initNetworkJoinAck((DvTelemJoinAcknowledge*)encodePacket->m_payload, joinData->m_deviceUniqueId, address);
        
        // return on the same channel it was received on.
        return encodePacketOnChannel(state, encodePacket, channel);  
    }    
    else {
        DV_error("[telem] received network join message with bad cookie, expected %d got %d.", (int)NETWORK_JOIN_VERSION_COOKIE, (int)joinData->m_command.m_cookie);
        return HANDLER_RESULT_ERROR;
    }   
}

DvTelemHandlerResult receiveNetworkJoinAckFn(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData) {
    DvTelemJoinAcknowledge* joinData = (DvTelemJoinAcknowledge*)packet->m_payload;
    DV_ensureAlignment(joinData, DV_alignOf(DvTelemJoinAcknowledge));   
    if (joinData->m_command.m_cookie == NETWORK_JOIN_VERSION_COOKIE) {
        if (joinData->m_deviceUniqueId == state->m_deviceId) {
            state->m_address = joinData->m_address;
            resetDeviceStatsData(&(state->m_statsData));
            DV_info("[telem] received network join ack, address is now %d.", (int)state->m_address);
        }
        return HANDLER_RESULT_OK;
    }
    else {
        DV_error("[telem] received network join ack message with bad cookie, expected %d got %d.", (int)NETWORK_JOIN_VERSION_COOKIE, (int)joinData->m_command.m_cookie);
        return HANDLER_RESULT_ERROR;
    }
}

void updateNetworkStartupFn(DvTelemHandlerState* state, void* userData) {
    DvTelemNetworkStartupState* startupState = (DvTelemNetworkStartupState*)userData;
    if (startupState->m_active) {
        if (elapsedMsStopWatch(&(startupState->m_timer)) > NETWORK_STARTUP_BROADCAST_PERIOD_MS) {
            startNetwork(state, startupState);
            restartStopWatch(&(startupState->m_timer));
            startupState->m_broadcastCount++;
            startupState->m_active = startupState->m_broadcastCount < NETWORK_STARTUP_BROADCAST_COUNT;            
            if (!startupState->m_active) {
                DV_info("[telem] network startup complete: ");
                for (uint8_t i = 0; i < startupState->m_deviceCount; ++i) {
                    DV_info("[telem] device: %x, address: %d", (unsigned int)startupState->m_deviceIds[i], i);
                }
            }
        }        
    }
}

DvTelemHandlerResult broadcastNetworkQoS(DvTelemHandlerState* state, DvTelemNetworkQoS* qoS) {

    DvTelemPacket* packet = (DvTelemPacket*)&state->m_encodePacket;
    initCommandPacket(packet, state->m_address, TELEM_ADDRESS_BROADCAST, (uint16_t)sizeof(DvTelemNetworkQoSData));
    DvTelemNetworkQoSData* cmdData = (DvTelemNetworkQoSData*)packet->m_payload;
    initNetworkQoSData(cmdData);
    memcpy(&(cmdData->m_networkQos), qoS, sizeof(DvTelemNetworkQoS));
    return encodePacketOnChannel(state, packet, getChannelRouteForPacket(state, packet));
}

DvTelemHandlerResult sendGetNetworkIdentity(DvTelemHandlerState* state, uint8_t address) {

    DvTelemPacket* packet = (DvTelemPacket*)&state->m_encodePacket;
    initCommandPacket(packet, state->m_address, address, (uint16_t)sizeof(DvTelemCommandAndVersion));
    initDeviceGetNetworkIdentityData((DvTelemCommandAndVersion*)packet->m_payload);
    return encodePacketOnChannel(state, packet, getChannelRouteForPacket(state, packet));
}

DvTelemHandlerResult sendSetNetworkIdentity(DvTelemHandlerState* state, uint8_t address, char* json, int jsonLength) {
    
    DvTelemPacket* packet = (DvTelemPacket*)&state->m_encodePacket;
    initCommandPacket(packet, state->m_address, address, (uint16_t)(sizeof(DvTelemDeviceIdentityData) + jsonLength));
    
    DvTelemDeviceIdentityData* cmdData = (DvTelemDeviceIdentityData*)packet->m_payload;
    initDeviceSetNetworkIdentityData(cmdData);
    memcpy(cmdData->m_json, json, (size_t)jsonLength);
    return encodePacketOnChannel(state, packet, getChannelRouteForPacket(state, packet));
}

DvTelemHandlerResult sendNetworkInfoRequest(DvTelemHandlerState* state) {

    DvTelemPacket* packet = (DvTelemPacket*)&state->m_encodePacket;
    initCommandPacket(packet, state->m_address, TELEM_ADDRESS_BROADCAST, (uint16_t)sizeof(DvTelemCommandAndVersion));

    DvTelemCommandAndVersion* cmdData = (DvTelemCommandAndVersion*)packet->m_payload;
    DV_ensureAlignment(cmdData, DV_alignOf(DvTelemCommandAndVersion));   
    cmdData->m_type = COMMAND_DEVICE_NETWORK_INFO;
    cmdData->m_cookie = DEVICE_NETWORK_INFO_VERSION_COOKIE;

    return encodePacketOnChannel(state, packet, getChannelRouteForPacket(state, packet));
}

DvTelemHandlerResult sendPowerOnCommand(DvTelemHandlerState* state, DvTelemPowerState* powerState, uint8_t address) {

    DvTelemPacket* packet = (DvTelemPacket*)&state->m_encodePacket;
    initCommandPacket(packet, state->m_address, address, (uint16_t)sizeof(DvTelemCommandAndVersion));

    DvTelemCommandAndVersion* cmdData = (DvTelemCommandAndVersion*)packet->m_payload;
    DV_ensureAlignment(cmdData, DV_alignOf(DvTelemCommandAndVersion));
    cmdData->m_type = COMMAND_DEVICE_POWER_ON;
    cmdData->m_cookie = DEVICE_POWER_ON_VERSION_COOKIE;

    powerState->m_entries[address].m_ackPending = true;
    powerState->m_entries[address].m_powerOnState = true;
    restartStopWatch(&(powerState->m_entries[address].m_retryTimer));

    return encodePacketOnChannel(state, packet, getChannelRouteForPacket(state, packet));    
}

DvTelemHandlerResult sendPowerOffCommand(DvTelemHandlerState* state, DvTelemPowerState* powerState, uint8_t address) {

    DvTelemPacket* packet = (DvTelemPacket*)&state->m_encodePacket;
    initCommandPacket(packet, state->m_address, address, (uint16_t)sizeof(DvTelemCommandAndVersion));

    DvTelemCommandAndVersion* cmdData = (DvTelemCommandAndVersion*)packet->m_payload;
    DV_ensureAlignment(cmdData, DV_alignOf(DvTelemCommandAndVersion));   
    cmdData->m_type = COMMAND_DEVICE_POWER_OFF;
    cmdData->m_cookie = DEVICE_POWER_OFF_VERSION_COOKIE;

    powerState->m_entries[address].m_ackPending = true;
    powerState->m_entries[address].m_powerOnState = false;
    restartStopWatch(&(powerState->m_entries[address].m_retryTimer));

    return encodePacketOnChannel(state, packet, getChannelRouteForPacket(state, packet));
}

DvTelemHandlerResult receivePowerOnCmdFn(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData) {

    DvTelemCommandAndVersion* cmdData = (DvTelemCommandAndVersion*)packet->m_payload;
    DV_ensureAlignment(cmdData, DV_alignOf(DvTelemCommandAndVersion));   
    if (cmdData->m_cookie == DEVICE_POWER_ON_VERSION_COOKIE) {

        // update the power state...
        state->m_statsData.m_state = DEVICE_STATE_POWERON;

        DvTelemPacket* encodePacket = (DvTelemPacket*)&state->m_encodePacket;
        initAckPacket(encodePacket, packet->m_header.m_destAddr, packet->m_header.m_sourceAddr, (uint16_t)sizeof(DvTelemCommandAndVersion));
        initDevicePowerOnData((DvTelemCommandAndVersion*)encodePacket->m_payload);

        // re-encode on the channel that the message came from..
        DV_info("[telem] received device power on message, acknowledging.");
        return encodePacketOnChannel(state, encodePacket, channel);
    }
    else {
        DV_error("[telem] received device power on message with bad cookie, expected %d got %d.", (int)DEVICE_POWER_ON_VERSION_COOKIE, (int)cmdData->m_cookie);
        return HANDLER_RESULT_ERROR;
    }
}

DvTelemHandlerResult receivePowerOffCmdFn(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData) {

    DvTelemCommandAndVersion* cmdData = (DvTelemCommandAndVersion*)packet->m_payload;
    DV_ensureAlignment(cmdData, DV_alignOf(DvTelemCommandAndVersion));   
    if (cmdData->m_cookie == DEVICE_POWER_OFF_VERSION_COOKIE) {

        // update the power state...
        state->m_statsData.m_state = DEVICE_STATE_POWEROFF;

        DvTelemPacket* encodePacket = (DvTelemPacket*)&state->m_encodePacket;
        initAckPacket(encodePacket, packet->m_header.m_destAddr, packet->m_header.m_sourceAddr, (uint16_t)sizeof(DvTelemCommandAndVersion));
        initDevicePowerOffData((DvTelemCommandAndVersion*)encodePacket->m_payload);

        // re-encode on the channel that the message came from..
        return encodePacketOnChannel(state, encodePacket, channel);
    }
    else {
        DV_info("[telem] received device power off message with bad cookie, expected %d got %d.", (int)DEVICE_POWER_OFF_VERSION_COOKIE, (int)cmdData->m_cookie);
        return HANDLER_RESULT_ERROR;
    }
}

void updatePowerStateFn(DvTelemHandlerState* state, void* userData) {
    
    // handle retry mechanism for the power on and power off commands    
    DvTelemPowerState* powerState = (DvTelemPowerState*)userData;
    DV_ensureAlignment(powerState, DV_alignOf(DvTelemPowerState));    
    for (uint8_t i = 0; i < DV_TELEM_MAX_ADDRESSES; ++i) {
        if (powerState->m_entries[i].m_ackPending && elapsedMsStopWatch(&(powerState->m_entries[i].m_retryTimer)) > DEVICE_POWER_COMMAND_RETRY_MS) {
            if (powerState->m_entries[i].m_powerOnState) {
                sendPowerOnCommand(state, powerState, i);
            }
            else {
                sendPowerOffCommand(state, powerState, i);                
            }
        }
    }
}

DvTelemHandlerResult sendFirmwareVersionRequest(DvTelemHandlerState* state, uint8_t address) {
    
    DvTelemPacket* packet = (DvTelemPacket*)&state->m_encodePacket;
    initCommandPacket(packet, state->m_address, address, (uint16_t)sizeof(DvTelemCommandAndVersion));

    DvTelemCommandAndVersion* cmdData = (DvTelemCommandAndVersion*)packet->m_payload;
    DV_ensureAlignment(cmdData, DV_alignOf(DvTelemCommandAndVersion));   
    cmdData->m_type = COMMAND_FIRMWARE_VERSION;
    cmdData->m_cookie = FIRMWARE_VERSION_COOKIE;

    return encodePacketOnChannel(state, packet, getChannelRouteForPacket(state, packet));
}

DvTelemHandlerResult sendFirmwareUpdateRequest(DvTelemHandlerState* state, uint8_t address) {
    
    DvTelemPacket* packet = (DvTelemPacket*)&state->m_encodePacket;
    initCommandPacket(packet, state->m_address, address, (uint16_t)sizeof(DvTelemCommandAndVersion));

    DvTelemCommandAndVersion* cmdData = (DvTelemCommandAndVersion*)packet->m_payload;
    DV_ensureAlignment(cmdData, DV_alignOf(DvTelemCommandAndVersion));
    cmdData->m_type = COMMAND_FIRMWARE_UPDATE;
    cmdData->m_cookie = FIRMWARE_UPDATE_COOKIE;

    return encodePacketOnChannel(state, packet, getChannelRouteForPacket(state, packet));
}

DV_warnPop()