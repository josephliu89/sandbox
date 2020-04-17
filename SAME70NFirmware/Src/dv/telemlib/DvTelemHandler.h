#ifndef DV_TELEM_HANDLER_H
#define DV_TELEM_HANDLER_H

#include "DvTelemCmds.h"

//
// Telemetry Handlers and Implementation Abstraction
//
// As a method to abstract away the complexities of which telemetry system we 
// are running over, be it Hunter, NSE, Edge Energy or a TCP network.  In an 
// attempt to make life easier, we can generalize the data connections along 
// the chain as a series of telemetry handlers.  
// 
// From either the Client or Server perspective, they shouldn’t need to know 
// or care how many telemetry handlers are in the chain from one end to the 
// other, just that when messages are initiated that the appropriate response 
// occurs.
//
// If we define a protocol and system that encapsulates all of the functionality 
// that we can foresee with each telemetry system, then we can replace existing 
// implementations with specialized sub-classes that work within the defined 
// protocol, transforming their own data into a format that matches the protocol.  
// For instance, we can make a system that transforms data to and from the hunter 
// domain into our common protocol we are defining here.
//
// A telemetry handler has a small number of functions:
//  - It can send packets that it has knowledge about (it is the source of the information)
//  - It routes packets based on address, packets that are not addressed to it should be forwarded along the network.
//  - It “handles” packets (not forwarding) and returns ACKs or NACKs based on type and specifics of the handler.
//
// Some Examples:
// 
// A handler can generate and send messages
//  - PLC Coordinator generating NETWORK_DISCOVERY packet
//  - PLC Slave + Coordinator generating DEVICE_STATE_AND_STATS messages
//  - PLC Coordinator generates NETWOK_QOS packets describing max bitrate
//  - Hunter Client Proxy generating NETWORK_DISCOVERY packet from parsing the Hunter SRO Log
//  - Hunter Client Proxy generating DEVICE_STATE_AND_STATS messages from HwsTelem packets.
//  - Hunter Client Proxy generates NETWORK_QOS packets describing max bitrate
//
// A handler can route packets
//  - PLC Coordinator receives data packets and turns them into PLC packets addressed to the correct PLC device.
//  - Hunter Client Proxy receives data packets and sends them down the hunter link with the correct Tarp Address and sends the data via hunter packets.
//
// A handler can handle packets
//  - PLC Slave handles “DEVICE_POWER_ON” packets by turning on 
//  - Hunter Client Proxy handles “DEVICE_POWER_ON” packets transforming them into Hunter specific power-on packets and transmitting them.
//
// Tracking Packet Loss:
//
// In order to track packet loss, the Packet header has a sequence number in it.  
// When sending or forwarding packets from one link to the next each Telemetry Handler 
// will have a rolling sequence ID tied to that source address, which will enable us 
// to determine the number of packets that were dropped along that telemetry link.  
// The dropped packet count will be sent up via the DEVICE_STATE_AND_STATS command.
//
// Addressed Serial Byte Streams:
//
// Ultimately, for each telemetry handler, there will be a conversion from serial 
// byte stream, to a packet, and then a translation back to a serial byte stream.  
// This will help us optimize our packet sizes to get the best throughput for each 
// medium.  For instance Hunter uses 142 byte packets, NSE Telemetry uses 256 byte 
// packets, and the G3-PLC bandwidth might be maximized at around 400 byte 
// packets, for TCP/IP, who cares...

typedef struct {

    uint8_t* m_rxBuffer;     // incoming receive buffer
    size_t m_rxBufferSize;   // incoming receive buffer size
    size_t m_rxBufferOffset; // current offset into the receive buffer

    uint8_t* m_txBuffer;     // outgoing transmit buffer 
    size_t m_txBufferSize;   // outgoing transmit size
    size_t m_txBufferOffset; // current offset into the transmit buffer

    // track expected sequences per address to track dropped packets.
    int m_txSequence[DV_TELEM_MAX_ADDRESSES];
    int m_rxSequence[DV_TELEM_MAX_ADDRESSES];

} DvTelemChannel;

typedef enum {
    HANDLER_RESULT_OK,
    HANDLER_RESULT_ERROR,
    HANDLER_RESULT_OUTPUT_BUFFER_FULL,
} DvTelemHandlerResult;

struct DvTelemHandlerState;
typedef DvTelemHandlerResult (*DvTelemOnPacketFn)(struct DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData);
typedef DvTelemChannel* (*DvTelemDefaultRouteFn)(struct DvTelemHandlerState* state, DvTelemPacket* packet);
typedef void (*DvTelemOnUpdateFn)(struct DvTelemHandlerState* state, void* userData);

typedef struct {
    bool m_initialized;
    DvTelemOnPacketFn m_cmdFn;
    void* m_cmdUserData;
    DvTelemOnPacketFn m_ackFn;
    void* m_ackUserData;
    DvTelemOnPacketFn m_nackFn;
    void* m_nackUserData;
    DvTelemOnUpdateFn m_updateFn;
    void* m_updateUserData;
} DvTelemHandlerInfo;

typedef struct DvTelemHandlerState {

    // current address of the handler.
    DvTelemAddressType m_address;
    // unique device id for the handler.
    uint32_t m_deviceId;
    
    // packet routing callback for default routing of packets
    DvTelemDefaultRouteFn m_routeFn;

    // data packet function, for dealing with data packets.
    DvTelemOnPacketFn m_dataFn;
    void* m_dataUserData;

    // channel buffer pointers, up to one per-address in some configurations
    DvTelemChannel* m_channels[DV_TELEM_MAX_ADDRESSES];
    // handler callback functions and state values
    DvTelemHandlerInfo m_handlers[DV_TELEM_MAX_COMMANDS];

    // device state and stats...
    DvTelemDeviceStatsData m_statsData;
    DvTelemDeviceStatsState m_statsState;

    // work buffers for parsing frames..
    DvTelemPacketStatic m_decodePacket;
    DvTelemPacketStatic m_encodePacket;

} DvTelemHandlerState;

// initialization functions.
void initTelemChannel(DvTelemChannel* state, uint8_t* rxBuffer, size_t rxBufferSize, uint8_t* txBuffer, size_t txBufferSize);
void initTelemHandlerState(DvTelemHandlerState* state, DvTelemDefaultRouteFn routeFn);
void attachTelemChannel(DvTelemHandlerState* state, uint8_t address, DvTelemChannel* channel);

// binding command handler functions.
void attachDataHandler(DvTelemHandlerState* state, DvTelemOnPacketFn cmdFn, void* userData);
void attachCmdHandler(DvTelemHandlerState* state, DvTelemCmdType command, DvTelemOnPacketFn cmdFn, void* userData);
void attachAckHandler(DvTelemHandlerState* state, DvTelemCmdType command, DvTelemOnPacketFn ackFn, void* userData);
void attachNackHandler(DvTelemHandlerState* state, DvTelemCmdType command, DvTelemOnPacketFn nackFn, void* userData);
void attachUpdateFunction(DvTelemHandlerState* state, DvTelemCmdType command, DvTelemOnUpdateFn updateFn, void* userData);

// state processing functions...
void processReceivedData(DvTelemHandlerState* state);   // call each update to process packets that have been received.
void updateHandlerStates(DvTelemHandlerState* state);   // call each update to process update stateful logic
void finalizeTransmitData(DvTelemHandlerState* state);  // before transmitting, close pending serial streams

// default routing functions, to be assiged when the handler is initialized
// these functions are used to determine which channel buffer that the packet
// will be encoded on.  For "leaf" nodes that run in the client and the server
// these will only contain one channel, but for telemetry firmware in the
// coordinator, slave, and client configuration there are multiple channels
// that the router needs to decide upon which channel the packet should be
// sent to, based on the packet header's source and destination addresses
DvTelemChannel* defaultLeafNodeRouting(DvTelemHandlerState* state, DvTelemPacket* packet);
DvTelemChannel* defaultPanCoordinatorRouting(DvTelemHandlerState* state, DvTelemPacket* packet);
DvTelemChannel* defaultPanSlaveRouting(DvTelemHandlerState* state, DvTelemPacket* packet);
DvTelemChannel* defaultClientRouting(DvTelemHandlerState* state, DvTelemPacket* packet);
DvTelemHandlerResult defaultRouterDataFn(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData);

// helper functions.
DvTelemChannel* getChannelRouteForPacket(DvTelemHandlerState* state, DvTelemPacket* packet);
DvTelemHandlerResult encodePacketOnChannel(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel); 

// data handlers
DvTelemHandlerResult receiveDataFn_Printf(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData);

// logging handlers
DvTelemHandlerResult sendLoggingCommand(DvTelemHandlerState* state, const char* message, uint8_t address);
DvTelemHandlerResult receiveLoggingCommandFn_Printf(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData);

// ping handlers
DvTelemHandlerResult sendPingCommand(DvTelemHandlerState* state, DvTelemPingState* pingState, uint8_t address);
DvTelemHandlerResult receivePingCmdFn(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData);
DvTelemHandlerResult receivePingAckFn_Printf(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData);

// stats handlers
void updateDeviceStatsFn(DvTelemHandlerState* state, void* userData);
DvTelemHandlerResult receiveDeviceStatsCmdFn_Printf(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData);
DvTelemHandlerResult sendDeviceStatsResetCommand(DvTelemHandlerState* state, uint8_t address);
DvTelemHandlerResult receiveDeviceStatsResetCmdFn(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData);

// network join/enumeration handlers.
DvTelemHandlerResult startNetwork(DvTelemHandlerState* state, DvTelemNetworkStartupState* startupState);
DvTelemHandlerResult receiveNetworkStartupCmdFn(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData);
DvTelemHandlerResult receiveNetworkJoinCmdFn(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData);
DvTelemHandlerResult receiveNetworkJoinAckFn(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData);
void updateNetworkStartupFn(DvTelemHandlerState* state, void* userData);

// network quality of service.
DvTelemHandlerResult broadcastNetworkQoS(DvTelemHandlerState* state, DvTelemNetworkQoS* qoS);

// network identity functions.
DvTelemHandlerResult sendGetNetworkIdentity(DvTelemHandlerState* state, uint8_t address);
DvTelemHandlerResult sendSetNetworkIdentity(DvTelemHandlerState* state, uint8_t address, char* json, int jsonLength);

// network info for leaf nodes to retrieve their address values and quality of service data.
DvTelemHandlerResult sendNetworkInfoRequest(DvTelemHandlerState* state);

// power on/off handlers
DvTelemHandlerResult sendPowerOnCommand(DvTelemHandlerState* state, DvTelemPowerState* powerState, uint8_t address);
DvTelemHandlerResult sendPowerOffCommand(DvTelemHandlerState* state, DvTelemPowerState* powerState, uint8_t address);
DvTelemHandlerResult receivePowerOnCmdFn(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData);
DvTelemHandlerResult receivePowerOffCmdFn(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData);
DvTelemHandlerResult receivePowerOnAckFn(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData);
DvTelemHandlerResult receivePowerOffAckFn(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData);
void updatePowerStateFn(DvTelemHandlerState* state, void* userData);

// firmware handlers
DvTelemHandlerResult sendFirmwareVersionRequest(DvTelemHandlerState* state, uint8_t address);
DvTelemHandlerResult sendFirmwareUpdateRequest(DvTelemHandlerState* state, uint8_t address);


#endif // DV_TELEM_HANDLER_H