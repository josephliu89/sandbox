#include "DvTelemCmds.h"

void initPingState(DvTelemPingState* state) {
    DV_ensureAlignment(state, DV_alignOf(DvTelemPingState));
    memset(state, 0, sizeof(DvTelemPingState));
}

void initNetworkStartupData(DvTelemCommandAndVersion* data) {
    DV_ensureAlignment(data, DV_alignOf(DvTelemCommandAndVersion));
    memset(data, 0, sizeof(DvTelemCommandAndVersion));
    data->m_type = COMMAND_NETWORK_STARTUP;
    data->m_cookie = NETWORK_STARTUP_VERSION_COOKIE;
}

void initNetworkStartupState(DvTelemNetworkStartupState* state) {
    DV_ensureAlignment(state, DV_alignOf(DvTelemNetworkStartupState));
    memset(state, 0, sizeof(DvTelemNetworkStartupState));
}

void initNetworkJoinRequest(DvTelemJoinRequest* data, uint32_t deviceId) {
    DV_ensureAlignment(data, DV_alignOf(DvTelemJoinRequest));
    memset(data, 0, sizeof(DvTelemJoinRequest));
    data->m_command.m_type = COMMAND_NETWORK_JOIN;
    data->m_command.m_cookie = NETWORK_JOIN_VERSION_COOKIE;
    data->m_deviceUniqueId = deviceId;
}

void initNetworkJoinAck(DvTelemJoinAcknowledge* data, uint32_t deviceId, DvTelemAddressType address) {
    DV_ensureAlignment(data, DV_alignOf(DvTelemJoinAcknowledge));
    memset(data, 0, sizeof(DvTelemJoinAcknowledge));
    data->m_command.m_type = COMMAND_NETWORK_JOIN;
    data->m_command.m_cookie = NETWORK_JOIN_VERSION_COOKIE;
    data->m_deviceUniqueId = deviceId;
    data->m_address = address;
}

void initNetworkDiscovery(DvTelemNetworkDiscovery* data) {
    DV_ensureAlignment(data, DV_alignOf(DvTelemNetworkDiscovery));
    memset(data, 0, sizeof(DvTelemNetworkQoSData));
    data->m_command.m_type = COMMAND_NETWORK_DISCOVERY;
    data->m_command.m_cookie = NETWORK_DISCOVERY_VERSION_COOKIE;
}

void initNetworkQoS(DvTelemNetworkQoS* data) {
    DV_ensureAlignment(data, DV_alignOf(DvTelemNetworkQoS));
    // initialize with hunter bandwidth..
    data->m_downlinkBytesPerSecond = 150;
    data->m_downlinkCapacityBytes = 400;
    data->m_uplinkBytesPerSecond = 11000;
    data->m_uplinkCapacityBytes = 23000;
}

void initNetworkQoSData(DvTelemNetworkQoSData* data) {
    DV_ensureAlignment(data, DV_alignOf(DvTelemNetworkQoSData));
    memset(data, 0, sizeof(DvTelemNetworkQoSData));
    data->m_command.m_type = COMMAND_NETWORK_QOS;
    data->m_command.m_cookie = NETWORK_QOS_VERSION_COOKIE;
    initNetworkQoS(&(data->m_networkQos));
}

void initDeviceGetNetworkIdentityData(DvTelemCommandAndVersion* data) {
    DV_ensureAlignment(data, DV_alignOf(DvTelemCommandAndVersion));
    memset(data, 0, sizeof(DvTelemCommandAndVersion));
    data->m_cookie = DEVICE_IDENTITY_VERSION_COOKIE;
    data->m_type = COMMAND_DEVICE_GET_IDENTITY;
}

void initDeviceSetNetworkIdentityData(DvTelemDeviceIdentityData* data) {
    DV_ensureAlignment(data, DV_alignOf(DvTelemDeviceIdentityData));
    memset(data, 0, sizeof(DvTelemDeviceIdentityData));
    data->m_command.m_cookie = DEVICE_IDENTITY_VERSION_COOKIE;
    data->m_command.m_type = COMMAND_DEVICE_GET_IDENTITY;
}

void initDeviceNetworkInfoData(DvTelemNetworkInfoData* data) {
    DV_ensureAlignment(data, DV_alignOf(DvTelemNetworkInfoData));
    memset(data, 0, sizeof(DvTelemNetworkInfoData));
    data->m_command.m_type = COMMAND_DEVICE_NETWORK_INFO;
    data->m_command.m_cookie = DEVICE_NETWORK_INFO_VERSION_COOKIE;
}

void initDeviceStatsData(DvTelemDeviceStatsData* stats) {
    DV_ensureAlignment(stats, DV_alignOf(DvTelemNetworkInfoData));
    memset(stats, 0, sizeof(DvTelemDeviceStatsData));
    stats->m_command.m_type = COMMAND_DEVICE_STATS;
    stats->m_command.m_cookie = DEVICE_STATS_VERSION_COOKIE;
    stats->m_state = DEVICE_STATE_UNKOWNN;
}

void initDeviceStatsState(DvTelemDeviceStatsState* state) {
    DV_ensureAlignment(state, DV_alignOf(DvTelemDeviceStatsState));
    memset(state, 0, sizeof(DvTelemDeviceStatsState));
    state->m_transmitPeriodMs = DEVICE_STATS_DEFAULT_TRANSMIT_PERIOD_MS;
    restartStopWatch(&(state->m_stateTimer));
    restartStopWatch(&(state->m_transmitTimer));
    restartStopWatch(&(state->m_bitRateTimer));
}

void resetDeviceStatsData(DvTelemDeviceStatsData* stats) {
    DV_ensureAlignment(stats, DV_alignOf(DvTelemDeviceStatsData));
    memset(&(stats->m_rxStats), 0, sizeof(stats->m_rxStats));
    memset(&(stats->m_txStats), 0, sizeof(stats->m_txStats));
}

void initDevicePowerOnData(DvTelemCommandAndVersion* data) {
    DV_ensureAlignment(data, DV_alignOf(DvTelemCommandAndVersion));
    data->m_cookie = DEVICE_POWER_ON_VERSION_COOKIE;
    data->m_type = COMMAND_DEVICE_POWER_ON;
}

void initDevicePowerOffData(DvTelemCommandAndVersion* data) {
    DV_ensureAlignment(data, DV_alignOf(DvTelemCommandAndVersion));
    data->m_cookie = DEVICE_POWER_OFF_VERSION_COOKIE;
    data->m_type = COMMAND_DEVICE_POWER_OFF;
}

void initDevicePowerState(DvTelemPowerState* state) {
    DV_ensureAlignment(state, DV_alignOf(DvTelemPowerState));
    memset(state, 0, sizeof(DvTelemPowerState));
}

void initFirmwareVersionData(DvTelemFirmwareVersionData* data) {
    DV_ensureAlignment(data, DV_alignOf(DvTelemFirmwareVersionData));
    memset(data, 0, sizeof(DvTelemFirmwareVersionData));
    data->m_command.m_cookie = FIRMWARE_VERSION_COOKIE;
    data->m_command.m_type = COMMAND_FIRMWARE_VERSION;
}

void initFirmwareUpdateData(DvTelemFirmwareUpdateData* data) {
    DV_ensureAlignment(data, DV_alignOf(DvTelemFirmwareUpdateData));
    memset(data, 0, sizeof(DvTelemFirmwareUpdateData));
    data->m_command.m_cookie = FIRMWARE_UPDATE_COOKIE;
    data->m_command.m_type = COMMAND_FIRMWARE_UPDATE;
}