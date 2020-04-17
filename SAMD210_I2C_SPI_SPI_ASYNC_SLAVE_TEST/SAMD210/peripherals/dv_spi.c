#include <atmel_start.h>
#include <string.h>
#include "dv_spi.h"
#include "dv_adc.h"

/* SPI-I2C packet format definitions */
#define SPI_COM     0
#define I2C0_COM    1
#define I2C1_COM    2
#define I2C2_COM    3
#define MASK_BITS   (0x08)
#define MASK_RW     (0x04)
#define MASK_INT    (0x03)
#define STROFFRAME  (0x7E)
#define ENDOFFRAME  (0x7D)

/* Acknowledgment codes */
#define ACK         (0xAA)
#define NACK        (0xBB)
#define INT_INVALID (0xCC)
#define REG_INVALID (0xDD)
#define LEN_INVALID (0xEE)

/* Z-Board specific registers */
#define REG_IR_CTRL (0x00)
#define REG_IR_VAL  (0x01)
#define REG_ANALOG0 (0x02)
#define REG_ANALOG1 (0x03)

/* SPI transaction counters */
static uint32_t rxCounter = 0;
static uint32_t txCounter = 0;

/* SPI RX/TX ring buffer variables */
#define RING_BUFFER_SIZE    128

typedef struct {
    uint16_t m_headIndex;
    uint16_t m_tailIndex;
    uint16_t m_bytesRemaining;
    char buffer[RING_BUFFER_SIZE];
} RingBuffer;

static struct io_descriptor *io;
static RingBuffer rxRingBuffer, txRingBuffer;
static commPacket rxPacket;
static bool txAllowed = false;

void printRxTxCount(void) {
    printf("rxCount: %lu\ttxCount: %lu\r\n", rxCounter, txCounter);    
}

static void initRingBuffer(RingBuffer* ringBuffer) {
    ringBuffer->m_headIndex = 0;
    ringBuffer->m_tailIndex = 0;
    ringBuffer->m_bytesRemaining = RING_BUFFER_SIZE;
    memset(ringBuffer->buffer, 0, sizeof(ringBuffer->buffer));
}

static int readFromRingBuffer(RingBuffer* ringBuffer, uint8_t* buffer, uint16_t readSize) {
    int numFormerBytes = 0;
    int numLatterBytes = 0;

    int bytesLeftToRead = RING_BUFFER_SIZE - ringBuffer->m_bytesRemaining;

    if (bytesLeftToRead == 0) {
        printf("Nothing to read...\n\r");
        return 0;
    }

    // adjust new read size depending on how much new data is available to be handled
    readSize = readSize > bytesLeftToRead ? bytesLeftToRead : readSize;

    if (readSize > 0) {
        // wrapping will occur
        if (ringBuffer->m_tailIndex + readSize > sizeof(ringBuffer->buffer)) {
            numFormerBytes = sizeof(ringBuffer->buffer) - ringBuffer->m_tailIndex;
            numLatterBytes = readSize - numFormerBytes;

            // move ring buffer data to buffer
            memcpy(buffer, ringBuffer->buffer + ringBuffer->m_tailIndex, numFormerBytes);
            memcpy(buffer + numFormerBytes, ringBuffer->buffer, numLatterBytes);
        }
        else {
            memcpy(buffer, ringBuffer->buffer + ringBuffer->m_tailIndex, readSize);
        }

        // update tail index
        ringBuffer->m_tailIndex = ((ringBuffer->m_tailIndex + readSize) % sizeof(ringBuffer->buffer));
    }

    // update ring buffer size
    ringBuffer->m_bytesRemaining += readSize;
    
    return readSize;
}

static int writeToRingBuffer(RingBuffer* ringBuffer, uint8_t* buffer, uint16_t writeSize) {
    int numFormerBytes = 0;
    int numLatterBytes = 0;
    
    if (ringBuffer->m_bytesRemaining == 0) {
        printf("Ring buffer full, ignoring writes!\n\r");
        return 0;
    }

    // adjust write size depending on space remaining in ring buffer
    writeSize = ringBuffer->m_bytesRemaining > writeSize ? writeSize : ringBuffer->m_bytesRemaining;
    
    if (writeSize > 0) {
        // wrapping will occur
        if (ringBuffer->m_headIndex + writeSize > sizeof(ringBuffer->buffer)) {
            numFormerBytes = sizeof(ringBuffer->buffer) - ringBuffer->m_headIndex;
            numLatterBytes = writeSize - numFormerBytes;

            // move buffer data to ring buffer
            memcpy(ringBuffer->buffer + ringBuffer->m_headIndex, buffer, numFormerBytes);
            memcpy(ringBuffer->buffer, buffer + numFormerBytes, numLatterBytes);
        }
        else {
            // move buffer data to ring buffer
            memcpy(ringBuffer->buffer + ringBuffer->m_headIndex, buffer, writeSize);
        }

        // update head index
        ringBuffer->m_headIndex = (ringBuffer->m_headIndex + writeSize) % sizeof(ringBuffer->buffer);
    }

    // update ring buffer size
    ringBuffer->m_bytesRemaining -= writeSize;

    return writeSize;
}

/*!
 *  /brief  Callback function for rising edge of CS
 */
static void complete_cb_SPI_0(const struct spi_s_async_descriptor *const desc) {
    /* Process data only when there is at least a minimum amount of bytes */
    //if ((RING_BUFFER_SIZE - rxRingBuffer.m_bytesRemaining) >= (sizeof(commPacket) + 2)) { 
        //processData();
    //}        
}

/*!
 *  /brief  Callback function for TX finish
 */
static void tx_cb_SPI_0(const struct spi_s_async_descriptor *const desc) {
    //uint8_t value = 0x00;
    //if (txAllowed) {
        //if (readFromRingBuffer(&txRingBuffer, &value, 1) != 0) {
            //io_write(io, &value, 1);
        //}
        //else {
            //txAllowed = false;
        //}        
    //}
    
    txCounter++;
}

/*!
 *  /brief  Callback function for RX notification
 */
static void rx_cb_SPI_0(const struct spi_s_async_descriptor *const desc) {
    uint8_t value;
    io_read(io, &value, 1);
    writeToRingBuffer(&rxRingBuffer, &value, 1);
    rxCounter++;
}

void initSpi(void) {
    spi_s_async_get_io_descriptor(&SPI_0, &io);
    spi_s_async_register_callback(&SPI_0, SPI_S_CB_TX, (FUNC_PTR)tx_cb_SPI_0);
    spi_s_async_register_callback(&SPI_0, SPI_S_CB_COMPLETE, (FUNC_PTR)complete_cb_SPI_0);
    spi_s_async_register_callback(&SPI_0, SPI_S_CB_RX, (FUNC_PTR)rx_cb_SPI_0);
    spi_s_async_enable(&SPI_0);
    
    initRingBuffer(&rxRingBuffer);
    initRingBuffer(&txRingBuffer);
}

typedef enum {
    NOPACKET,
    OKPACKET
} decodeResults;

/*!
 *  ---------------------------------------------------------------------------------------------------------
 *  |   SPI-I2C Packet Format                                                                          |
 *  ---------------------------------------------------------------------------------------------------------
 *  |   1 byte  |   1 byte  |   1 byte      |   1 byte      |   1 byte      |   1 byte          |   1 byte  |
 *  ---------------------------------------------------------------------------------------------------------
 *  |   SOF     |   HEADER  |   DEV_ADR     |   REGISTER    |   DATA0 (MSB)  |   DATA1 (LSB)    |   EOF     |
 *  ---------------------------------------------------------------------------------------------------------
 *
 *  SOF[7:0]        Start of frame      0x7E
 *
 *  HEADER[7:4]     RFU
 *  HEADER[3]       Data length         0:  1 byte   |   1: 2 bytes
 *  HEADER[2]       Read/Write          0:  Write    |   1: Read
 *  HEADER[1:0]     Bus Interface       00: SPI (direct communication with DV13131B peripherals)
 *                                      01: I2C0
 *                                      10: I2C1
 *                                      11: I2C2
 *
 *  DEV_ADR[7:0]    Device address      0x00 for SPI interface, otherwise, 7-bit slave address for I2C
 *
 *  REGISTER[7:0]   Register address    Device specific register for I2C devices, otherwise, for DV1313B:
 *                                      0x00:   IR Control
 *                                      0x01:   IR Value
 *                                      0x02:   Analog 0
 *                                      0x03:   Analog 1
 *
 *  DATA0[7:0]      Data (MSB)          MSB for 2-byte transaction, otherwise data for 1-byte transaction
 *
 *  DATA1[7:0]      Data (LSB)          LSB for 2-byte transaction, otherwise 0x00 for 1-byte transaction
 *
 *  EOF[7:0]        End of frame        0x7D
 * 
*/

void printBuffer(uint8_t length) {
    printf("\r\ntxAllowed: %i", txAllowed);
    printf("\r\nrxheadIndex: %i\t\t\trxheadIndex: %i\r\n", rxRingBuffer.m_headIndex, txRingBuffer.m_headIndex);
    printf("rxtailIndex: %i\t\t\ttxtailIndex: %i\r\n", rxRingBuffer.m_tailIndex, txRingBuffer.m_tailIndex);
    printf("rxBytesRemaining: %i\t\ttxBytesRemaining %i\r\n", rxRingBuffer.m_bytesRemaining, txRingBuffer.m_bytesRemaining);
    for (uint8_t i = 0; i < length; i++) {
        printf("rxData[%02i]: 0x%02X\t\ttxData[%02i]: 0x%02X\r\n", i, rxRingBuffer.buffer[i], i, txRingBuffer.buffer[i]);
    }
}

/*!
 *  /brief  Find valid packet from byte stream/ring buffer while stripping off SOF and EOF bytes
 *  
 *  /param  ringBuffer  Ring buffer to process
 *  /param  packet      Pointer to valid packet, if packet is found, it is populated
 *
 *  /retval Returns true if valid packet was found
 */
static bool getPacketFromStream(RingBuffer* ringBuffer, commPacket* validPacket) {   
    
    uint8_t workBuffer[sizeof(commPacket)];
    uint8_t tempVar = 0;
    
    /* Check if ring buffer is empty */
    if (ringBuffer->m_bytesRemaining == RING_BUFFER_SIZE) {
        return false;
    } 
    else {
        /* Traverse through ring buffer to find SOF */
        do {
            readFromRingBuffer(ringBuffer, &tempVar, 1);
            if (tempVar == STROFFRAME) {
                break;
            }
        } while (ringBuffer->m_bytesRemaining != RING_BUFFER_SIZE);
    
        /* SOF was the last byte in the ring buffer, exiting */
        if (ringBuffer->m_bytesRemaining == RING_BUFFER_SIZE) {
            return false;
        }
        /* More data to process */
        else {
            for (uint8_t i = 0; i < sizeof(workBuffer); i++) {
                readFromRingBuffer(ringBuffer, &tempVar, 1);
                workBuffer[i] = tempVar;
                                
                if (ringBuffer->m_bytesRemaining == RING_BUFFER_SIZE) {
                    return false;
                }                
            }
            
            readFromRingBuffer(ringBuffer, &tempVar, 1);
            
            if (tempVar == ENDOFFRAME) {
                /* Valid frame, populate packet */
                validPacket->header     = workBuffer[0];
                validPacket->devAddr    = workBuffer[1];
                validPacket->regAddr    = workBuffer[2];
                validPacket->data[0]    = workBuffer[3];
                validPacket->data[1]    = workBuffer[4];
                return true;
            }
            else {
                /* Recursively go through ringBuffer to look for valid frame */
                if (getPacketFromStream(ringBuffer, validPacket)) {
                    return true;
                }
                else {
                    return false;
                }
            }
        }
    }
}

/*!
 *  /brief  Takes incoming data and repackages SPI-I2C packet format with SOF and EOF bytes
 *  
 *  /param  packet      Packet to respond to. Contents are taken from this packet to be resent back to
 *                      master to correct data returning
 *  /param  data        Raw data going back to master
 *
 */
static void encodeResults(commPacket *packet, uint8_t* data) {
    uint8_t byteValue = STROFFRAME;
    writeToRingBuffer(&txRingBuffer, &byteValue, 1);
    writeToRingBuffer(&txRingBuffer, &(packet->header), 1);
    writeToRingBuffer(&txRingBuffer, &(packet->devAddr), 1);
    writeToRingBuffer(&txRingBuffer, &(packet->regAddr), 1);
    writeToRingBuffer(&txRingBuffer, data, 1);
    writeToRingBuffer(&txRingBuffer, data + 1, 1);
    byteValue = ENDOFFRAME;
    writeToRingBuffer(&txRingBuffer, &byteValue, 1);
    
    uint8_t value = 0x00;
    uint8_t arrayValue[7];
    if (readFromRingBuffer(&txRingBuffer, arrayValue, 7) != 0) {
        io_write(io, arrayValue, 7);
    }
    
    txAllowed = true;
}

/*!
 *  /brief  Main service routine that deciphers, executes get/set commands, and returns ACK/NACK
 *  
 *  /param  packet      Packet to respond to. Contents are taken from this packet to be resent back to
 *                      master to correct data returning
 *  /param  data        Raw data going back to master
 *
 */
bool processData(void) {
    printBuffer(30);
    
    commPacket* rxPkt = &rxPacket;
    uint8_t txData[2] = {0};
    bool decodeStatus = getPacketFromStream(&rxRingBuffer, rxPkt);

    if (decodeStatus == NOPACKET) {
        /* Do nothing */
        return 0;
    }
    else {
                      
        uint8_t hwInterface = rxPkt->header & MASK_INT;
        uint8_t hwLength = ((rxPkt->header & MASK_BITS) == 0) ? 8 : 16;
        uint16_t rawAdcVal = 0;
        
        /* Handle get command */
        if ((rxPkt->header & MASK_RW) == MASK_RW) {
            if (hwInterface == SPI_COM) {  
                /* Handle SPI transactions */
                if (rxPkt->regAddr == REG_IR_CTRL) {
                    txData[0] = gpio_get_pin_level(IR_CTRL);
                    txData[1] = 0x00;
                }
                else if ((rxPkt->regAddr == REG_IR_VAL) || (rxPkt->regAddr == REG_ANALOG0) || (rxPkt->regAddr == REG_ANALOG1)) {
                    if (rxPkt->regAddr == REG_IR_VAL) {
                        adcSetChannel(IR);
                    }
                    else if (rxPkt->regAddr == REG_ANALOG0) {
                        adcSetChannel(ANALOG0);
                    }
                    else if (rxPkt->regAddr == REG_ANALOG1) {
                        adcSetChannel(ANALOG1);
                    }
                    while(!getConversionStatus());
                    rawAdcVal = getRawAdcVal();     
                    txData[0] = (rawAdcVal & 0xFF00) >> 8;
                    txData[1] = rawAdcVal & 0xFF;
                }
                else {
                    printf("Z-Board register invalid\r\n");
                    txData[0] = REG_INVALID;
                    txData[1] = 0x00;
                }                
            }
            else if ((hwInterface == I2C0_COM) || (hwInterface == I2C1_COM) || (hwInterface == I2C2_COM)) {
                /* Determine I2C interface to pull from */
                if (hwInterface == I2C0_COM) {
                    /* Handle I2C0 transactions */
                    i2c_m_sync_set_slaveaddr(&I2C_0, rxPkt->devAddr, I2C_M_SEVEN);
                }
                else if (hwInterface == I2C1_COM) {
                    /* Handle I2C1 transactions */
                    i2c_m_sync_set_slaveaddr(&I2C_1, rxPkt->devAddr, I2C_M_SEVEN);
                }
                else if (hwInterface == I2C2_COM) {
                    /* Handle I2C2 transactions */
                    i2c_m_sync_set_slaveaddr(&I2C_2, rxPkt->devAddr, I2C_M_SEVEN);
                }
            
                /* Return I2C read-back data back on SPI */
                if (hwLength == 8) {
                    i2c_m_sync_cmd_read(&I2C_0, rxPkt->regAddr, txData, 1);
                }
                else {
                    i2c_m_sync_cmd_read(&I2C_0, rxPkt->regAddr, txData, 2);
                }
            }
            else {
                printf("Hardware interface not valid\r\n");
                txData[0] = INT_INVALID;
                txData[1] = 0x00;                
            }
        }
        /* Handle set command */
        else {
            if (hwInterface == SPI_COM) {
                /* Handle SPI transactions */
                if (rxPkt->regAddr == REG_IR_CTRL) {
                    if (rxPkt->data[0] == 0x01) {
                        gpio_set_pin_level(IR_CTRL, 1);
                    }
                    else {
                        gpio_set_pin_level(IR_CTRL, 0);
                    }
                }
                else {
                    printf("Z-Board register invalid\r\n");
                    txData[0] = REG_INVALID;
                    txData[1] = 0x00;
                }    
            }
            else if ((hwInterface == I2C0_COM) || (hwInterface == I2C1_COM) || (hwInterface == I2C2_COM)) {
                /* Determine I2C interface to pull from */
                if (hwInterface == I2C0_COM) {
                    /* Handle I2C0 transactions */
                    i2c_m_sync_set_slaveaddr(&I2C_0, rxPkt->devAddr, I2C_M_SEVEN);
                }
                else if (hwInterface == I2C1_COM) {
                    /* Handle I2C1 transactions */
                    i2c_m_sync_set_slaveaddr(&I2C_1, rxPkt->devAddr, I2C_M_SEVEN);
                }
                else if (hwInterface == I2C2_COM) {
                    /* Handle I2C2 transactions */
                    i2c_m_sync_set_slaveaddr(&I2C_2, rxPkt->devAddr, I2C_M_SEVEN);
                }
            
                uint8_t writeData[3] = {rxPkt->regAddr, rxPkt->data[0], rxPkt->data[1]};
            
                /* Return I2C read-back ACK/NACK back on SPI */
                if (hwLength == 8) {
                    if (i2c_m_sync_cmd_write_m(&I2C_0, writeData, 1) == 0) {
                        txData[0] = ACK;
                        txData[1] = 0x00;
                    }
                    else {
                        txData[0] = NACK;
                        txData[1] = 0x00;                        
                    }
                }
                else if (hwLength == 16) {
                    if (i2c_m_sync_cmd_write_m(&I2C_0, writeData, 2) == 0) {
                        txData[0] = ACK;
                        txData[1] = 0x00;                        
                    }
                    else {
                        txData[0] = NACK;
                        txData[1] = 0x00;                         
                    }
                }
                else {
                    printf("8/16 bit data length invalid\r\n");                    
                    txData[0] = LEN_INVALID;
                    txData[1] = 0x00;
                }
            }
            else {
                printf("Hardware interface not valid\r\n");
                txData[0] = INT_INVALID;
                txData[1] = 0x00;                
            }
        }   
        encodeResults(rxPkt, txData);             
    }
    return 1;
}