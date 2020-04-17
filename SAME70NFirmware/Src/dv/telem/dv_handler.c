/*
 * dv_handler.c
 *
 * Created: 12/9/2019 3:52:26 PM
 *  Author: liu
 */ 

#include "asf.h"
#include "dv/telemlib/DvTelemHandler.h"
#include "dv/telem/dv_handler.h"
#include "dv/peripherals/dv_plc.h"
#include "dv/peripherals/dv_xmodem.h"
#include "dv/peripherals/dv_efc.h"
#include "dv/Version.h"
#include <stdio.h>

DV_warnPush()
DV_warnDisableGCC("-Wcast-align")
DV_warnDisableGCC("-Wunused-parameter")

typedef struct {
	DvTelemChannel m_channelSpi;
	DvTelemChannel m_channelPlc;
	DvTelemHandlerState m_handler;
	DvTelemPingState m_pings;
    DvTelemNetworkStartupState m_network;
    DvTelemNetworkQoS m_networkQoS;
	uint8_t m_txBufferSpi[HANDLER_BUFFER_SIZE];
	uint8_t m_rxBufferSpi[HANDLER_BUFFER_SIZE];
	uint8_t m_txBufferPlc[HANDLER_BUFFER_SIZE];
	uint8_t m_rxBufferPlc[HANDLER_BUFFER_SIZE];
} HandlerState;

static HandlerState s_state;

extern uint32_t __temp__;
extern uint32_t __shared__;
extern uint32_t __flwrite__;
static uint8_t xmdmPacketNum;

static void jumpToApp(uint32_t address)
{
	uint8_t i;

	__disable_irq();
	/* Disable SysTick */
	SysTick->CTRL = 0;
	/* Disable IRQs & clear pending IRQs */
	for (i = 0; i < 8; i++) {
		NVIC->ICER[i] = 0xFFFFFFFF;
		NVIC->ICPR[i] = 0xFFFFFFFF;
	}

	/* Modify vector table location */
	__DSB();
	__ISB();

	/* set the stack pointer also to the start of the firmware app */
	__set_MSP(*(int *)(address));

	/* offset the start of the vector table (first 6 bits must be zero) */
	SCB->VTOR = ((uint32_t)address & SCB_VTOR_TBLOFF_Msk);
	__DSB();
	__ISB();
	__enable_irq();

	/* jump to the start of the firmware, casting the address as function
	 * pointer to the start of the firmware. We want to jump to the address
	 * of the reset */

	/* handler function, that is the value that is being pointed at position */
	void (*runFirmware)(void) = NULL;
	runFirmware = (void (*)(void))(*(uint32_t *)(address + 4));
	runFirmware();
}

static DvTelemHandlerResult dvhandler_receiveGetNeworkIdentityCmdFn(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData) {
    DvTelemCommandAndVersion* cmdData = (DvTelemCommandAndVersion*)packet->m_payload;
    DV_ensureAlignment(cmdData, DV_alignOf(DvTelemCommandAndVersion));
    if (cmdData->m_cookie == DEVICE_IDENTITY_VERSION_COOKIE) {

#pragma message("*** TODO: Read current device identity from the device flash...")

        const char* jsonData = "{ \"m_ipAddress\":\"192.168.24.234\",\"m_port\":11018,\"m_toolId\":\"DV12571-A 010\",\"m_toolSerial\":\"000\",\"m_toolName\":\"MkII-MkTube\"}";
        int jsonStringSize = strlen(jsonData) + 1; 
        DvTelemPacket* encodePacket = (DvTelemPacket*)&state->m_encodePacket;
        initAckPacket(encodePacket, packet->m_header.m_destAddr, packet->m_header.m_sourceAddr, (uint16_t)(sizeof(DvTelemDeviceIdentityData) + jsonStringSize));
        
        // re-encode on the channel that the message came from
        return encodePacketOnChannel(state, encodePacket, channel);
    }
    else {
        DV_error("[telem] received get network identity message with bad cookie, expected %d got %d.", (int)DEVICE_IDENTITY_VERSION_COOKIE, (int)cmdData->m_cookie);
        return HANDLER_RESULT_ERROR;
    }
}    

static DvTelemHandlerResult dvhandler_receiveSetNeworkIdentityCmdFn(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData) {
    DvTelemDeviceIdentityData* cmdData = (DvTelemDeviceIdentityData*)packet->m_payload;
    DV_ensureAlignment(cmdData, DV_alignOf(DvTelemDeviceIdentityData));   
    if (cmdData->m_command.m_cookie == DEVICE_IDENTITY_VERSION_COOKIE) {

        //const char* jsonData = "{ \"m_ipAddress\":\"192.168.24.234\",\"m_port\":11018,\"m_toolId\":\"DV12571-A 010\",\"m_toolSerial\":\"000\",\"m_toolName\":\"MkII-MkTube\"}";
        //int jsonStringSize = packet->m_header.m_length - sizeof(DvTelemDeviceIdentityData);

#pragma message("*** TODO: Write device identity to the flash...")

        return HANDLER_RESULT_OK;
    }
    else {
        DV_error("[telem] received set network identity message with bad cookie, expected %d got %d.", (int)DEVICE_IDENTITY_VERSION_COOKIE, (int)cmdData->m_command.m_cookie);
        return HANDLER_RESULT_ERROR;
    }
}

static DvTelemHandlerResult dvhandler_receiveNetworkInfoCmdFn(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData) {

    DvTelemCommandAndVersion* cmdData = (DvTelemCommandAndVersion*)packet->m_payload;
    DV_ensureAlignment(cmdData, DV_alignOf(DvTelemCommandAndVersion));
    if (cmdData->m_cookie == DEVICE_NETWORK_INFO_VERSION_COOKIE) {

        DvTelemPacket* encodePacket = (DvTelemPacket*)&state->m_encodePacket;
        initAckPacket(encodePacket, packet->m_header.m_destAddr, packet->m_header.m_sourceAddr, (uint16_t)sizeof(DvTelemNetworkInfoData));

        DvTelemNetworkInfoData* networkData = (DvTelemNetworkInfoData*)encodePacket->m_payload;
        initDeviceNetworkInfoData(networkData);
        networkData->m_address = state->m_address;
        memcpy(&(networkData->m_networkQos), &(s_state.m_networkQoS), sizeof(s_state.m_networkQoS));

        // set the power state to powered on.
        state->m_statsData.m_state = DEVICE_STATE_POWERON;
        
        // re-encode on the channel that the message came from
        return encodePacketOnChannel(state, encodePacket, channel);
    }
    else {
        DV_error("[telem] received device network info request message with bad cookie, expected %d got %d.", (int)DEVICE_NETWORK_INFO_VERSION_COOKIE, (int)cmdData->m_cookie);
        return HANDLER_RESULT_ERROR;
    }
}

static DvTelemHandlerResult dvhandler_receiveNetworkJoinAckFn(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData) {
    DvTelemJoinAcknowledge* joinData = (DvTelemJoinAcknowledge*)packet->m_payload;
    DV_ensureAlignment(joinData, DV_alignOf(DvTelemJoinAcknowledge));
    if (joinData->m_command.m_cookie == NETWORK_JOIN_VERSION_COOKIE) {
        if (joinData->m_deviceUniqueId == state->m_deviceId) {
            state->m_address = joinData->m_address;
            resetDeviceStatsData(&(state->m_statsData));
            dv_plc_setAddress(state->m_address);
            DV_info("[telem] received network join ack, address is now %d.", (int)state->m_address);
        }
        return HANDLER_RESULT_OK;
    }
    else {
        DV_error("[telem] received network join ack message with bad cookie, expected %d got %d.", (int)NETWORK_JOIN_VERSION_COOKIE, (int)joinData->m_command.m_cookie);
        return HANDLER_RESULT_ERROR;
    }
}

static DvTelemHandlerResult dvhandler_receivePowerOnCmdFn(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData) {

    DvTelemCommandAndVersion* cmdData = (DvTelemCommandAndVersion*)packet->m_payload;
    DV_ensureAlignment(cmdData, DV_alignOf(DvTelemCommandAndVersion));   
    if (cmdData->m_cookie == DEVICE_POWER_ON_VERSION_COOKIE) {

        DvTelemPacket* encodePacket = (DvTelemPacket*)&state->m_encodePacket;
        initAckPacket(encodePacket, packet->m_header.m_destAddr, packet->m_header.m_sourceAddr, (uint16_t)sizeof(DvTelemCommandAndVersion));
        initDevicePowerOnData((DvTelemCommandAndVersion*)encodePacket->m_payload);

        // update the power state...
        state->m_statsData.m_state = DEVICE_STATE_POWERINGON;
#pragma message("*** TODO: Turn on the device here..., more sophistication here...")
        gpio_set_pin_low(NSTART);

        // re-encode on the channel that the message came from..
        DV_info("[telem] received device power on message for address: %d, acknowledging.", state->m_address);
        return encodePacketOnChannel(state, encodePacket, channel);
    }
    else {
        DV_error("[telem] received device power on message with bad cookie, expected %d got %d.", (int)DEVICE_POWER_ON_VERSION_COOKIE, (int)cmdData->m_cookie);
        return HANDLER_RESULT_ERROR;
    }
}

static DvTelemHandlerResult dvhandler_receivePowerOffCmdFn(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData) {

    DvTelemCommandAndVersion* cmdData = (DvTelemCommandAndVersion*)packet->m_payload;
    DV_ensureAlignment(cmdData, DV_alignOf(DvTelemCommandAndVersion));   
    if (cmdData->m_cookie == DEVICE_POWER_OFF_VERSION_COOKIE) {

        DvTelemPacket* encodePacket = (DvTelemPacket*)&state->m_encodePacket;
        initAckPacket(encodePacket, packet->m_header.m_destAddr, packet->m_header.m_sourceAddr, (uint16_t)sizeof(DvTelemCommandAndVersion));
        initDevicePowerOffData((DvTelemCommandAndVersion*)encodePacket->m_payload);

#pragma message("*** TODO: Turn off the device here..., more sophistication here...")
        gpio_set_pin_high(NSTART);

        // update the power state...
        state->m_statsData.m_state = DEVICE_STATE_POWERINGOFF;

        // re-encode on the channel that the message came from..
        DV_info("[telem] received device power off message for address: %d, acknowledging.", state->m_address);
        return encodePacketOnChannel(state, encodePacket, channel);
    }
    else {
        DV_error("[telem] received device power off message with bad cookie, expected %d got %d.", (int)DEVICE_POWER_OFF_VERSION_COOKIE, (int)cmdData->m_cookie);
        return HANDLER_RESULT_ERROR;
    }
}

static DvTelemHandlerResult dvhandler_receiveFirmwareVersionCmdFn(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData) {

    DvTelemCommandAndVersion* cmdData = (DvTelemCommandAndVersion*)packet->m_payload;
    DV_ensureAlignment(cmdData, DV_alignOf(DvTelemCommandAndVersion));   
    if (cmdData->m_cookie == FIRMWARE_VERSION_COOKIE) {

        DvTelemPacket* encodePacket = (DvTelemPacket*)&state->m_encodePacket;
        initAckPacket(encodePacket, packet->m_header.m_destAddr, packet->m_header.m_sourceAddr, (uint16_t)sizeof(DvTelemFirmwareVersionData));

        DvTelemFirmwareVersionData* versionData = (DvTelemFirmwareVersionData*)encodePacket->m_payload;
        initFirmwareVersionData(versionData);

        // populate version strings.       
        strncpy(versionData->m_version.m_date, __DATE__, sizeof(versionData->m_version.m_date) - 1);
        strncpy(versionData->m_version.m_branch, GIT_BRANCH, sizeof(versionData->m_version.m_branch) - 1);
        strncpy(versionData->m_version.m_version, GIT_VERSION, sizeof(versionData->m_version.m_version) - 1);
        
        // re-encode on the channel that the message came from
        return encodePacketOnChannel(state, encodePacket, channel);
    }
    else {
        DV_error("[telem] received firmware version request message with bad cookie, expected %d got %d.", (int)FIRMWARE_VERSION_COOKIE, (int)cmdData->m_cookie);
        return HANDLER_RESULT_ERROR;
    }
}

static DvTelemHandlerResult dvhandler_receiveFirmwareUpdateCmdFn(DvTelemHandlerState* state, DvTelemPacket* packet, DvTelemChannel* channel, void* userData) {

    uint32_t ul_rc = 0;                 /* Flash status result */
    uint32_t progSize = 0;              /* Track size of program */
    uint8_t writeRetries = 0;           /* Flash write retires */
    uint8_t programBytes[4];
    uint8_t decodedXmdmData[128];      /* Holds raw data after decoding XMODEM packet */

    DvTelemCommandAndVersion* cmdData = (DvTelemCommandAndVersion*)packet->m_payload;
    DV_ensureAlignment(cmdData, DV_alignOf(DvTelemCommandAndVersion));
    if (cmdData->m_cookie == FIRMWARE_UPDATE_COOKIE) {

        DvTelemPacket* encodePacket = (DvTelemPacket*)&state->m_encodePacket;
        initAckPacket(encodePacket, packet->m_header.m_destAddr, packet->m_header.m_sourceAddr, (uint16_t)sizeof(DvTelemFirmwareUpdateData));
        
        DvTelemFirmwareUpdateData* decodedData = (DvTelemFirmwareUpdateData*)packet->m_payload;
        
        DvTelemFirmwareUpdateData* returnData = (DvTelemFirmwareUpdateData*)encodePacket->m_payload;
        initFirmwareUpdateData(returnData);

        if (decodedData->m_action == FIRMWARE_UPDATE_ABORT) {
            DV_info("[telem] firmware update aborted");
            
            // return confirmation status
            returnData->m_action = FIRMWARE_UPDATE_ABORT;
        }
        else if (decodedData->m_action == FIRMWARE_UPDATE_INITITATE) {
            /* Send out 'C' to initiate start of XMODEM transfer */
            initiateXmodem(returnData);
            
            /* Packet number starts out at 1 and rolls over to 0 if there there are more than 255 packets */
            xmdmPacketNum = 1;
            
            /* Prepare flash area to be written to by erasing them, 0x00480000 -> 0x004DFFFF*/
            erase_partitions(6, 8);             
        } 
        else if (decodedData->m_action == FIRMWARE_UPDATE_XMODEM) {
            /* Decode DvTelm packet stream using XMODEM protocol into buffer then copy contents to flash. 
             * If this is an end of transmission packet, decodeXmodemPacket will return XMDM_RESULT_EOT 
             * and not write to flash. Similar behaviour for XMDM_RESULT_ABORT. 
             */
            xmdmResult xmdmStatus = decodeXmodemPacket(packet, returnData, decodedXmdmData, &xmdmPacketNum, &progSize);
            if (xmdmStatus == XMDM_RESULT_OK) {
                /* Write to flash */
                writeRetries = 3;
                
                do {
                    flash_write(((uint32_t)&__temp__) - PKTLEN_128 + progSize, decodedXmdmData, PKTLEN_128, 0);
                    if(--writeRetries == 0) {
                        DV_error("Flash writing error, retried 3 times!");
                        break;
                    }
                } while (validate_data(((uint32_t)&__temp__) - PKTLEN_128 + progSize, decodedXmdmData, PKTLEN_128) == FLASH_RC_ERROR); 
            }
            else if ((xmdmStatus == XMDM_FAILED_CRC) || (xmdmStatus == XMDM_FAILED_SEQ)) {
                /* Return NACK */
                encodePacket->m_header.m_type = DV_TELEM_PACKET_TYPE_NACK;
            }
            /* Else XMDM_RESULT_ABORT or XMDM_RESULT_EOT status will result in an ACK by default */
        }
        else if (decodedData->m_action == FIRMWARE_UPDATE_FINISH) {
            /* Write size of programming size into shared memory */                        
            flash_erase_sector((uint32_t)&__shared__);
            
            for (uint8_t i = 0; i < 4; i++) {
                programBytes[i] = ((uint8_t*)&progSize)[3-i];
            }
            
            writeRetries = 3;
            
            do {
                flash_write((uint32_t)&__shared__, programBytes, sizeof(programBytes), 0);
                ul_rc = validate_data((uint32_t)&__shared__, programBytes, sizeof(programBytes));
                if(--writeRetries == 0) {
                    DV_error("Flash writing error, retried 3 times!");
                    break;
                }
            } while (ul_rc == FLASH_RC_ERROR);            
            
            if (ul_rc == FLASH_RC_OK) {
                DV_info("New firmware written successfully to temporary storage, transferring to flash writer application");
                
                /* Jump to flash writer application */
                jumpToApp((uint32_t)(&__flwrite__));
            } 
            else {
                DV_error("Failed to properly write new firmware size to shared memory, aborting update.");
            }                            
        }            
                                   
        // re-encode on the channel that the message came from
        return encodePacketOnChannel(state, encodePacket, channel);
    }
    else {
        DV_error("[telem] received firmware update request message with bad cookie, expected %d got %d.", (int)FIRMWARE_UPDATE_COOKIE, (int)cmdData->m_cookie);
        return HANDLER_RESULT_ERROR;
    }
}

void dvhandler_init(uint8_t role, uint32_t deviceId) {

    memset(&s_state, 0, sizeof(HandlerState));

    // initialize buffers
    initTelemChannel(&(s_state.m_channelSpi), s_state.m_rxBufferSpi, HANDLER_BUFFER_SIZE, s_state.m_txBufferSpi, HANDLER_BUFFER_SIZE);
    initTelemChannel(&(s_state.m_channelPlc), s_state.m_rxBufferPlc, HANDLER_BUFFER_SIZE, s_state.m_txBufferPlc, HANDLER_BUFFER_SIZE);
    
    if (role == ROLE_COORDINATOR) {
        initTelemHandlerState(&(s_state.m_handler), defaultPanCoordinatorRouting);
    } 
    else {
        initTelemHandlerState(&(s_state.m_handler), defaultPanSlaveRouting);
    }

    // set device id
    s_state.m_handler.m_deviceId = deviceId;
    
    initPingState(&(s_state.m_pings));
    initNetworkStartupState(&(s_state.m_network));
    initNetworkQoS(&(s_state.m_networkQoS));

    // attach channel
    attachTelemChannel(&(s_state.m_handler), 0, &(s_state.m_channelSpi));
    attachTelemChannel(&(s_state.m_handler), 1, &(s_state.m_channelPlc));

    // initialize data callbacks
    attachDataHandler(&(s_state.m_handler), defaultRouterDataFn, nullptr);

    // initialize command callbacks
    attachCmdHandler(&(s_state.m_handler), COMMAND_LOGGING, receiveLoggingCommandFn_Printf, nullptr);
    attachCmdHandler(&(s_state.m_handler), COMMAND_PING, receivePingCmdFn, nullptr);
    attachAckHandler(&(s_state.m_handler), COMMAND_PING, receivePingAckFn_Printf, nullptr);
    attachCmdHandler(&(s_state.m_handler), COMMAND_DEVICE_STATS, receiveDeviceStatsCmdFn_Printf, nullptr);
    attachUpdateFunction(&(s_state.m_handler), COMMAND_DEVICE_STATS, updateDeviceStatsFn, nullptr);
    attachCmdHandler(&(s_state.m_handler), COMMAND_DEVICE_STATS_RESET, receiveDeviceStatsResetCmdFn, nullptr);

    // firmware handlers..
    attachCmdHandler(&(s_state.m_handler), COMMAND_FIRMWARE_VERSION, dvhandler_receiveFirmwareVersionCmdFn, nullptr);
    attachCmdHandler(&(s_state.m_handler), COMMAND_FIRMWARE_UPDATE, dvhandler_receiveFirmwareUpdateCmdFn, nullptr);

    // coordinator responds to join requests and slave responds to network startup commands
    // as well as poweron/poweroff packets amongst other things.
    if (role == ROLE_COORDINATOR) {
        startNetwork(&(s_state.m_handler), &(s_state.m_network));
        attachCmdHandler(&(s_state.m_handler), COMMAND_NETWORK_JOIN, receiveNetworkJoinCmdFn, &(s_state.m_network));
        attachUpdateFunction(&(s_state.m_handler), COMMAND_NETWORK_STARTUP, updateNetworkStartupFn, &(s_state.m_network));
    }
    else {
        attachCmdHandler(&(s_state.m_handler), COMMAND_NETWORK_STARTUP, receiveNetworkStartupCmdFn, nullptr);
        attachAckHandler(&(s_state.m_handler), COMMAND_NETWORK_JOIN, dvhandler_receiveNetworkJoinAckFn, nullptr);        
        attachCmdHandler(&(s_state.m_handler), COMMAND_DEVICE_GET_IDENTITY, dvhandler_receiveGetNeworkIdentityCmdFn, nullptr);
        attachCmdHandler(&(s_state.m_handler), COMMAND_DEVICE_SET_IDENTITY, dvhandler_receiveSetNeworkIdentityCmdFn, nullptr);
        attachCmdHandler(&(s_state.m_handler), COMMAND_DEVICE_NETWORK_INFO, dvhandler_receiveNetworkInfoCmdFn, nullptr);
        attachCmdHandler(&(s_state.m_handler), COMMAND_DEVICE_POWER_ON, dvhandler_receivePowerOnCmdFn, nullptr);
        attachCmdHandler(&(s_state.m_handler), COMMAND_DEVICE_POWER_OFF, dvhandler_receivePowerOffCmdFn, nullptr);
    }
}

void dvhandler_update(void) {
    processReceivedData(&(s_state.m_handler));
    updateHandlerStates(&(s_state.m_handler));
    finalizeTransmitData(&(s_state.m_handler));
}

uint8_t dvhandler_getCurrentAddress(void) {
    return s_state.m_handler.m_address;
}

void dvhandler_bytesFromSpi(uint8_t* data, int size) {
    DV_ensure(s_state.m_channelSpi.m_rxBufferOffset + size <= HANDLER_BUFFER_SIZE);
    memcpy(s_state.m_channelSpi.m_rxBuffer + s_state.m_channelSpi.m_rxBufferOffset, data, size);
    s_state.m_channelSpi.m_rxBufferOffset += size;
}

void dvhandler_bytesFromPlc(uint8_t* data, int size) {
    DV_ensure(s_state.m_channelPlc.m_rxBufferOffset + size <= HANDLER_BUFFER_SIZE);
    memcpy(s_state.m_channelPlc.m_rxBuffer + s_state.m_channelPlc.m_rxBufferOffset, data, size);
    s_state.m_channelPlc.m_rxBufferOffset += size;
}

int dvhandler_bytesToSpi(uint8_t* buffer, int buffSize) {
    int bytesToWrite = (buffSize < (int)s_state.m_channelSpi.m_txBufferOffset) ? buffSize : (int)s_state.m_channelSpi.m_txBufferOffset;
    memcpy(buffer, s_state.m_channelSpi.m_txBuffer, bytesToWrite);
    memcpy(s_state.m_channelSpi.m_txBuffer, s_state.m_channelSpi.m_txBuffer + bytesToWrite, (size_t)(s_state.m_channelSpi.m_txBufferOffset - bytesToWrite));
    s_state.m_channelSpi.m_txBufferOffset -= bytesToWrite;
    return bytesToWrite;
}

int dvhandler_bytesToPlc(uint8_t* buffer, int buffSize) {
    int bytesToWrite = (buffSize < (int)s_state.m_channelPlc.m_txBufferOffset) ? buffSize : (int)s_state.m_channelPlc.m_txBufferOffset;
    memcpy(buffer, s_state.m_channelPlc.m_txBuffer, bytesToWrite);
    memcpy(s_state.m_channelPlc.m_txBuffer, s_state.m_channelPlc.m_txBuffer + bytesToWrite, (size_t)(s_state.m_channelPlc.m_txBufferOffset - bytesToWrite));
    s_state.m_channelPlc.m_txBufferOffset -= bytesToWrite;
    return bytesToWrite;
}

DV_warnPop()