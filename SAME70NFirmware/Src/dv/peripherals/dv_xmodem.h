/**
 * \file
 *
 * \brief Function for the XMODEM transfer protocol
 *
 * Copyright (c) 2014-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef DV_XMODEM_H_
#define DV_XMODEM_H_

#include "compiler.h"
#include "serial.h"
#include "dv/telemlib/DvTelemPacket.h"
#include "dv/telemlib/DvTelemCmds.h"

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

/**
 * \defgroup common_services_xmodem_group XMODEM transfer protocol service
 *
 * The XMODEM transfer protocol service provides function to receive/send a file
 * from USART/UART using XMODEM protocol.
 *
 * @{
 */

/** The definitions are followed by the XMODEM protocol */
#define XMDM_SOH     0x01 /**< Start of heading */
#define XMDM_EOT     0x04 /**< End of text */
#define XMDM_ACK     0x06 /**< Acknowledge  */
#define XMDM_NAK     0x15 /**< negative acknowledge */
#define XMDM_CAN     0x18 /**< Cancel */
#define XMDM_ESC     0x1b /**< Escape */

#define CRC16POLY   0x1021  /**< CRC 16 polynomial */
#define PKTLEN_128  128u    /**< Data packet length */
#define XMDM_SIZE   133u    /**< XMODEM packet length */

typedef enum {
    XMDM_RESULT_OK,
    XMDM_RESULT_EOT,
    XMDM_RESULT_ABORT,
    XMDM_FAILED_CRC,
    XMDM_FAILED_SEQ,
} xmdmResult;

void initiateXmodem(DvTelemFirmwareUpdateData* outgoingData);
xmdmResult decodeXmodemPacket(DvTelemPacket* packet, DvTelemFirmwareUpdateData* outData, uint8_t decodedXmdmData[], uint8_t* expectedPacketNum, uint32_t *progSize);

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond


#endif /* DV_XMODEM_H_ */