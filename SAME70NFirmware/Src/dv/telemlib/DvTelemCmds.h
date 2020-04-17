#ifndef DV_TELEM_CMDS_H
#define DV_TELEM_CMDS_H

#include "DvTelemPacket.h"
#include "DvTelemUtils.h"

DV_warnPush()
DV_warnDisableMSVC(4214) // nonstandard extension used: bit field types other than int
DV_warnDisableMSVC(4200) // nonstandard extension used: zero-sized array in struct/union

#define DV_TELEM_MAX_COMMANDS      48

// Assign block ranges so future commands can be added 
// without causing version compatibility issues
typedef enum {

    // base commands
    COMMAND_LOGGING                = 0, // critical debug logging over SPI interface
    COMMAND_PING                   = 1, // round trip ping calculations...

    // network enumeration and addressing
    COMMAND_NETWORK_STARTUP        = 5, // broadcast network initiate packet.
    COMMAND_NETWORK_JOIN           = 6, // nodes will send a join request in response to the startup packet.
    COMMAND_NETWORK_DISCOVERY      = 7, // respond with devices and addresses on the network
    COMMAND_NETWORK_QOS            = 8, // broadcast message regulating available bandwidth and general health

    // telemetry MCU specific
    COMMAND_DEVICE_GET_IDENTITY    = 16, // respond with human readable details to display for a user, include MCU firmware versions
    COMMAND_DEVICE_SET_IDENTITY    = 17, // set identity for human readable details store in MCU flash
    COMMAND_DEVICE_NETWORK_INFO    = 18, // tool requests its address and current network QoS from telemetry MCUs
    COMMAND_DEVICE_STATS           = 19, // device performance statistics, and current state
    COMMAND_DEVICE_STATS_RESET     = 20, // reset the stats timers and counters.
    COMMAND_DEVICE_POWER_ON        = 21, // turn on the connected tool.
    COMMAND_DEVICE_POWER_OFF       = 22, // turn off the connected tool.   

    // firmware updates/programming
    COMMAND_FIRMWARE_VERSION       = 32, // retrieve PLC firmware version
    COMMAND_FIRMWARE_UPDATE        = 33, // initiate/complete firmware updates over SPI interface

    // software commands
    COMMAND_SOFTWARE_RESETCOMMS    = 40, // reset network links to re-establish connection handshaking when a link times out.
    COMMAND_SOFTWARE_ENABLE_DATA   = 41, // enable data transfer from a particular node
    COMMAND_SOFTWARE_DISABLE_DATA  = 42, // disable data transfer from a particular node
    
} DvTelemCmdType;

// Command with version cookie, version cookie is used to version specific 
// command structures so that we can detect incompatibilities 
typedef struct {
    uint8_t m_type;
    uint8_t m_cookie;
} DvTelemCommandAndVersion;

///////////////////////////////////////////////////////////////////////////////
/////////////////////////     Base Commands         ///////////////////////////
///////////////////////////////////////////////////////////////////////////////

// change the cookie when the structure changes to allow for graceful updates.
#define LOGGING_VERSION_COOKIE 0xa0

// m_message is C99 standard flexible array member pointing to the data
// immediately following it in the structure
typedef struct {
    DvTelemCommandAndVersion m_command;
    char m_message[];
} DvTelemLoggingData;

#define PING_VERSION_COOKIE 0xa0
typedef struct {
    DvStopWatchState m_pingTimers[DV_TELEM_MAX_ADDRESSES];
} DvTelemPingState;

void initPingState(DvTelemPingState* state);

///////////////////////////////////////////////////////////////////////////////
/////////////////////////     Network Commands      ///////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define NETWORK_STARTUP_VERSION_COOKIE 0xa0
#define NETWORK_JOIN_VERSION_COOKIE 0xa0

#define NETWORK_STARTUP_BROADCAST_PERIOD_MS 200
#define NETWORK_STARTUP_BROADCAST_COUNT 10

// when receiving a network startup packet each node will send a join request
// in response with a unique device identifier.  the device id will be a unique
// id that can identify the telemetry board.  This can be a CPU identifier or
// some other mechanism.
typedef struct {
    DvTelemCommandAndVersion m_command;
    uint32_t m_deviceUniqueId;
} DvTelemJoinRequest;

// when the join request has been successful it will respond with a join ack
// structure, sending back the same unique device id, as well as the network
// address that the device should use from there on out.
typedef struct {
    DvTelemCommandAndVersion m_command;
    DvTelemAddressType m_address;
    uint32_t m_deviceUniqueId;
} DvTelemJoinAcknowledge;

// state used for initializing the network, track devices found and at the
// end of the device enumeration process, at the end of the process addresses
// will be broadcast.
typedef struct {
    bool m_active;
    uint8_t m_broadcastCount;
    uint8_t m_deviceCount;
    uint32_t m_deviceIds[DV_TELEM_MAX_ADDRESSES];
    DvStopWatchState m_timer;
} DvTelemNetworkStartupState;

void initNetworkStartupData(DvTelemCommandAndVersion* data);
void initNetworkStartupState(DvTelemNetworkStartupState* state);
void initNetworkJoinRequest(DvTelemJoinRequest* data, uint32_t deviceId);
void initNetworkJoinAck(DvTelemJoinAcknowledge* data, uint32_t deviceId, DvTelemAddressType address);

#define NETWORK_DISCOVERY_VERSION_COOKIE 0xa0

typedef struct {
    uint16_t m_jsonSize;
    uint16_t m_jsonOffset;
    uint8_t m_address;
    uint8_t m_pad0;
} DvTelemNetworkEntry;

// m_nodes is C99 standard flexible array member pointing to the data
// immediately following it in the structure. 
typedef struct {
    DvTelemCommandAndVersion m_command;
    uint16_t m_nodeCount;
    DvTelemNetworkEntry m_nodes[];
} DvTelemNetworkDiscovery;

void initNetworkDiscovery(DvTelemNetworkDiscovery* data);

// capacity, is the maximum allowable burst capacity this represents to some 
// degree the amount of flexibility in the system, with regards to how much
// buffering and latency there is.  These values can be fed directly into the
// TokenBucket implementation in the main software.
typedef struct {

    uint16_t m_uplinkBytesPerSecond;
    uint16_t m_uplinkCapacityBytes;
    uint16_t m_downlinkBytesPerSecond;
    uint16_t m_downlinkCapacityBytes;

} DvTelemNetworkQoS;

void initNetworkQoS(DvTelemNetworkQoS* data);


#define NETWORK_QOS_VERSION_COOKIE 0xa0

typedef struct {
    DvTelemCommandAndVersion m_command;
    DvTelemNetworkQoS m_networkQos;
} DvTelemNetworkQoSData;

void initNetworkQoSData(DvTelemNetworkQoSData* data);

///////////////////////////////////////////////////////////////////////////////
/////////////////////////     Device Commands       ///////////////////////////
///////////////////////////////////////////////////////////////////////////////

// m_json is C99 standard flexible array member pointing to the data
// immediately following it in the structure. m_json contains a json
// serialized description of the tool.  json data might look like this:
// { 
//   "m_ipAddress":"192.168.24.234",
//   "m_port" : 11018,
//   "m_toolId" : "DV12571-A 010",
//   "m_toolSerial" : "000",
//   "m_toolName" : "MkII-MkTube",
//   "m_toolDesc" : "MkII",
//   "m_serverStatus" : "Server running: no connection",
//   "m_osBranch" : "field-6.0",
//   "m_osDate" : "Dec 4 2019",
//   "m_osVersion" : "554a3bf89"
// }
typedef struct {
    DvTelemCommandAndVersion m_command;
    char m_json[];
} DvTelemDeviceIdentityData;

#define DEVICE_IDENTITY_VERSION_COOKIE 0xa0

void initDeviceGetNetworkIdentityData(DvTelemCommandAndVersion* data);
void initDeviceSetNetworkIdentityData(DvTelemDeviceIdentityData* data);

#define DEVICE_NETWORK_INFO_VERSION_COOKIE 0xa0

typedef struct {
    DvTelemCommandAndVersion m_command;
    DvTelemNetworkQoS m_networkQos;
    DvTelemAddressType m_address;
} DvTelemNetworkInfoData;

void initDeviceNetworkInfoData(DvTelemNetworkInfoData* data);

// device state enum
typedef enum {
    DEVICE_STATE_UNKOWNN,
    DEVICE_STATE_POWEROFF,
    DEVICE_STATE_POWERINGOFF,
    DEVICE_STATE_POWERINGON,
    DEVICE_STATE_POWERON,
    DEVICE_STATE_WAITING_FOR_COMMS,
    DEVICE_STATE_COMMS_READY
} DvTelemDeviceState;

// change the cookie when the structure changes to allow for graceful updates.
#define DEVICE_STATS_VERSION_COOKIE 0xa0
#define DEVICE_STATS_RESET_VERSION_COOKIE 0xa0

typedef struct {

    uint16_t m_crcErrors;
    uint16_t m_packetLengthErrors;
    uint16_t m_droppedPacketCount;
    uint16_t m_totalPacketCount;
    uint32_t m_bytes;
    float m_bitRate;

} DvTelemRxPacketStats;

typedef struct {

    uint16_t m_totalPacketCount;
    uint16_t m_bufferFullErrors;
    uint32_t m_bytes;
    float m_bitRate;

} DvTelemTxPacketStats;

typedef struct {

    DvTelemCommandAndVersion m_command;
    DvTelemRxPacketStats m_rxStats;
    DvTelemTxPacketStats m_txStats;
    DvTelemDeviceState m_state;
    float m_stateTimerSeconds;

} DvTelemDeviceStatsData;

#define DEVICE_STATS_DEFAULT_TRANSMIT_PERIOD_MS (10 * 1000)

typedef struct {

    uint32_t m_transmitPeriodMs;
    // time in state.
    DvStopWatchState m_stateTimer;
    // time since last transmit.
    DvStopWatchState m_transmitTimer;
    // timer for calculating bitrate.
    DvStopWatchState m_bitRateTimer;

} DvTelemDeviceStatsState;

void initDeviceStatsData(DvTelemDeviceStatsData* stats);
void initDeviceStatsState(DvTelemDeviceStatsState* state);
void resetDeviceStatsData(DvTelemDeviceStatsData* stats);

#define DEVICE_POWER_ON_VERSION_COOKIE 0xa0
#define DEVICE_POWER_OFF_VERSION_COOKIE 0xa0
#define DEVICE_POWER_COMMAND_RETRY_MS 1000

// track the current power state so that retries can be handled appropriately.
// by the telemetry library under the hood.
typedef struct {
    bool m_ackPending;
    bool m_powerOnState;
    DvStopWatchState m_retryTimer;
} DvTelemPowerPendingEntry;    

typedef struct {
    DvTelemPowerPendingEntry m_entries[DV_TELEM_MAX_ADDRESSES];
} DvTelemPowerState;

void initDevicePowerOnData(DvTelemCommandAndVersion* data);
void initDevicePowerOffData(DvTelemCommandAndVersion* data);
void initDevicePowerState(DvTelemPowerState* state);

///////////////////////////////////////////////////////////////////////////////
/////////////////////////     Firmware Commands     ///////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define FIRMWARE_VERSION_COOKIE 0xa0
#define FIRMWARE_VERSION_SIZE_date 28
#define FIRMWARE_VERSION_SIZE_branch 32
#define FIRMWARE_VERSION_SIZE_version 12

typedef struct {
    char m_date[FIRMWARE_VERSION_SIZE_date];
    char m_branch[FIRMWARE_VERSION_SIZE_branch];
    char m_version[FIRMWARE_VERSION_SIZE_version];
} DvTelemVersionBlock;

typedef struct {
    DvTelemCommandAndVersion m_command;
    DvTelemVersionBlock m_version;
} DvTelemFirmwareVersionData;

#define FIRMWARE_UPDATE_COOKIE 0xa0

typedef enum {
    FIRMWARE_UPDATE_INITITATE,
    FIRMWARE_UPDATE_XMODEM,
    FIRMWARE_UPDATE_FINISH,
    FIRMWARE_UPDATE_ABORT
} DvTelemFirmwareUpdateAction;

// m_data is C99 standard flexible array member pointing to the data
// immediately following it in the structure
typedef struct {
    DvTelemCommandAndVersion m_command;
    DvTelemFirmwareUpdateAction m_action;
    char m_data[];
} DvTelemFirmwareUpdateData;

void initFirmwareVersionData(DvTelemFirmwareVersionData* data);
void initFirmwareUpdateData(DvTelemFirmwareUpdateData* data);

DV_warnPop()

#endif // DV_TELEM_CMDS_H