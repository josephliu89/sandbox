/*
 * dv_spi.c
 *
 * Created: 11/13/2019 4:29:50 PM
 *  Author: liu
 */ 

#include "asf.h"
#include "dv_spi.h"
#include "dv/telem/dv_handler.h"
#include "dv/telemlib/DvTelemUtils.h"

/* SPI RX/TX ring buffer variables */
#define MAX_SPI_PACKET_XFER_SIZE   255
#define RING_BUFFER_SIZE    1024

typedef struct {
    uint16_t m_headIndex;
    uint16_t m_tailIndex;
    uint16_t m_bytesRemaining;
    char buffer[RING_BUFFER_SIZE];
} RingBuffer;

static RingBuffer rxRingBuffer, txRingBuffer;

static void initRingBuffer(RingBuffer* ringBuffer) {
    ringBuffer->m_headIndex = 0;
    ringBuffer->m_tailIndex = 0;
    ringBuffer->m_bytesRemaining = RING_BUFFER_SIZE;
    memset(ringBuffer->buffer, 0, sizeof(ringBuffer->buffer));
}

static int readFromRingBuffer(RingBuffer* ringBuffer, char* buffer, uint16_t readSize) {
    int numFormerBytes = 0;
    int numLatterBytes = 0;

    int bytesLeftToRead = RING_BUFFER_SIZE - ringBuffer->m_bytesRemaining;

    if (bytesLeftToRead == 0) {
        DV_info("Nothing to read...");
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

static int writeToRingBuffer(RingBuffer* ringBuffer, char* buffer, uint16_t writeSize) {
    int numFormerBytes = 0;
    int numLatterBytes = 0;
    
    if (ringBuffer->m_bytesRemaining == 0) {
        DV_error("Ring buffer full, ignoring writes!");
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

/**
 * \brief Initialize SPI as slave while PoEM board will be master.
 */
void init_spi_slave(void)
{
    usart_spi_opt_t usart_spi_slave_settings = {
        .baudrate = 0,                          // clock controlled by master, this parameter is ignored as slave
        .char_length = US_MR_CHRL_8_BIT,        // 8 bit transfer
        .spi_mode = SPI_MODE_0,
        .channel_mode = US_MR_CHMODE_NORMAL
    };

    sysclk_enable_peripheral_clock(ID_USART0);

    usart_init_spi_slave(USART0, &usart_spi_slave_settings);

    /* Enable TX & RX function */
    usart_enable_tx(USART0);
    usart_enable_rx(USART0);

    /* Configure and enable interrupt from USART in IRQ controller. */
    NVIC_DisableIRQ(USART0_IRQn);
    NVIC_ClearPendingIRQ(USART0_IRQn);
    NVIC_SetPriority(USART0_IRQn, 1);
    usart_enable_interrupt(USART0, US_IER_RXRDY | US_IER_CTSIC);
    NVIC_EnableIRQ(USART0_IRQn);

    initRingBuffer(&rxRingBuffer);
    initRingBuffer(&txRingBuffer);

    DV_info("SPI slave interface initialized successfully!");
}

/*!
 * \brief   On falling edge of nCS, incoming SPI data is placed onto RX ring buffer at the same time
 *          TX data is sent. On the rising edge of CS, data is read from TX ring buffer and is placed
 *          into temporary storage. No new data is retrieved from TX ring buffer for subsequent rising
 *          edges of nCS until temporary storage has been fully consumed. 
 *
 *          As an expected initial condition, no TX bytes will be sent on the very first assertion of nCS.
 *          
 *          Interrupt handler for the SPI slave using USART interface.
 * 
 * \param   None
 *
 * \retval  None 
 */
 void USART0_Handler(void)
 {
     static uint32_t rxData32;          // temporary storage read from USART interface
     static uint8_t rxData8;            // rxData32 reduced to a byte to match ringBuffer data width
     static uint8_t edgeCnt = 0;        // since USART doesn't have a falling edge detector, use US_IER_CTSIC to count
     static char txData[3 + MAX_SPI_PACKET_XFER_SIZE + 2];		// header + max packet size + crc
     static uint8_t numTxBytesData;
     static uint16_t numTotalTxBytes;
     static volatile uint16_t spiTxCounter = 0;
     static volatile bool writeCompleted = false;
     
     /* Get interrupt source */
     uint32_t dvSpiInterruptStatus = usart_get_status(USART0);
     
     /* When nCS is asserted, do this */
     if (dvSpiInterruptStatus & US_IER_CTSIC) {
         
         /* Falling edge of nCS */
         if (++edgeCnt == 1) {

             /* Reset states */         
             writeCompleted = false;
             spiTxCounter = 0;
         }

         /* Rising edge of nCS */
         else if (edgeCnt == 2) {

             /* The transmit buffer is loaded from txRingBuffer when CS is idle, otherwise there are timing issues
              *
              * Generate packet = header + data from ring buffer + crc (of header + data) 
              */
                
             numTxBytesData = readFromRingBuffer(&txRingBuffer, txData + 3, MAX_SPI_PACKET_XFER_SIZE);

             if (numTxBytesData > 0) {
                 
                 /* Hard-coding spiHeader.m_address and spiHeader.command to zero */
                 memset(txData, 0x00, 2);
                 
                 /* Add length */
                 memcpy(txData + 2, &numTxBytesData, 1);
                 
                 /* Calculate CRC of header + data */
                 const uint16_t crc16 = calcTelemDataCrc(txData, 3 + numTxBytesData);
                 memcpy(txData + 3 + numTxBytesData, &crc16, sizeof(crc16));
                 
                 /* Determine total bytes in packet */
                 numTotalTxBytes = 3 + numTxBytesData + sizeof(crc16);
             }

             /* Queue write hold register so that the PoEM receives the correct first byte on the first read 
              * Read and writes are done simultaneously in a SPI transfer
              */
             usart_write(USART0, txData[0]);

             /* Reset falling/rising flag */
             edgeCnt = 0;
         }
     }
     
     /* Falling edge of nCS */
     if (edgeCnt == 1) {
         if (dvSpiInterruptStatus & US_IER_RXRDY) {
             
             /* Store incoming data into rxRingBuffer */
             if(usart_read(USART0, &rxData32) == 0) {
                 rxData8 = rxData32 & 0x000000FF;
                 writeToRingBuffer(&rxRingBuffer, (char *)&rxData8, 1);
             }

             /* Queue up next byte to SPI interface */
             if ((numTxBytesData > 0) && (!writeCompleted)) {

                 // done transfer packet of data
                 if (++spiTxCounter > numTotalTxBytes) {
                     writeCompleted = true;
                 }
                 else {
                     // write to SPI interface
                     if (usart_write(USART0, txData[spiTxCounter]) != 0) {
                         DV_error("Problems writing to PoEM board via SPI!");
                     }
                 }
             }
         }
     }
 }

/*!
 * \brief   Pulls SPI data from ring buffer and transfers it to channel RX buffer
 * 
 * \param   None
 *
 * \retval  None 
 */
void handleSpiRxData(void) {
    int bytesToRead;
    static char data[MAX_SPI_PACKET_XFER_SIZE];

    bytesToRead = readFromRingBuffer(&rxRingBuffer, data, sizeof(data));
    
    dvhandler_bytesFromSpi((uint8_t*)data, bytesToRead);
}

/*!
 * \brief   Pulls SPI data from channel SPI TX buffer and places it into TX ring buffer to be transferred via SPI hardware
 * 
 * \param   None
 *
 * \retval  None 
 */
void handleSpiTxData(void) {
    int bytesToWrite;
    static char data[MAX_SPI_PACKET_XFER_SIZE];
    
    bytesToWrite = dvhandler_bytesToSpi((uint8_t*)data, sizeof(data));

    writeToRingBuffer(&txRingBuffer, data, bytesToWrite);
}