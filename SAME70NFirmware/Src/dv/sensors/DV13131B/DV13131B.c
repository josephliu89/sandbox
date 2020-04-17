#include "asf.h"
#include "dv/sensors/DV13131B/DV13131B.h"
//#include "dv/communication/dv_console.h"

/*!
 *  ---------------------------------------------------------------------------------
 *  |   SPI-I2C Packet Format                                                       |
 *  ---------------------------------------------------------------------------------
 *  |   1 byte  |   1 byte      |   1 byte      |   1 byte      |   1 byte          |
 *  ---------------------------------------------------------------------------------
 *  |   HEADER  |   DEV_ADR     |   REGISTER    |   DATA0 (MSB)  |   DATA1 (LSB)    |
 *  ---------------------------------------------------------------------------------
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
 *  REGISTER[7:0]   Register address    Device specific register for I2C devices, otherwise, for DV13131B:
 *                                      0x00:   IR Control
 *                                      0x01:   IR Value
 *                                      0x02:   Analog 0
 *                                      0x03:   Analog 1
 *                                      0x04:   Green LED
 *                                      0x05:   Red LED 
 *
 *  DATA0[7:0]      Data (MSB)          MSB for 2-byte transaction, otherwise data for 1-byte transaction
 *
 *  DATA1[7:0]      Data (LSB)          LSB for 2-byte transaction, otherwise 0x00 for 1-byte transaction
 *
*/

/*!
 *  /brief  Sets bit position to '1'
 *  
 *  /param  pos     Position to set
 *  /param  data    Pointer to data
 *  
 *  /retval None
 */

static void setBit(uint8_t pos, uint8_t *data) {
    *data |= (1 << pos);
}

/*!
 *  /brief  Clears bit position to '0'
 *  
 *  /param  pos     Position to clear
 *  /param  data    Pointer to data
 *  
 *  /retval None
 */

static void clearBit(uint8_t pos, uint8_t *data) {
    *data &= ~(1 << pos);
}


/* Sample SPI Configurations */
#define SPI0_PCS        3       /* Chip select channel */
#define SPI0_DLYBS      1       /* Delay before SPCK */
#define SPI0_DLYBBCT    0       /* Delay between consecutive transfers */

#pragma message("@TODO: change SPI interface as required, this will need to be done on both sides if it differs from this sample init")
void sampleSpiInit(void) {
    /* Enable SPI peripheral. */
    spi_enable_clock(SPI0);

    /* Reset SPI */
    spi_disable(SPI0);
    spi_reset(SPI0);

    /* Configure SPI */
    spi_set_master_mode(SPI0);
    spi_disable_mode_fault_detect(SPI0);
    spi_set_peripheral_chip_select_value(SPI0, spi_get_pcs(SPI0_PCS));
    spi_set_clock_polarity(SPI0, SPI0_PCS, 0);
    spi_set_clock_phase(SPI0, SPI0_PCS, 1);
    spi_set_bits_per_transfer(SPI0, SPI0_PCS, SPI_CSR_BITS_8_BIT);
    spi_set_baudrate_div(SPI0, SPI0_PCS, 255);
    //spi_set_baudrate_dv(SPI0, SPI0_PCS, 400);
    spi_set_transfer_delay(SPI0, SPI0_PCS, SPI0_DLYBS, SPI0_DLYBBCT);
    spi_configure_cs_behavior(SPI0, SPI0_PCS, SPI_CS_RISE_NO_TX);
    spi_enable(SPI0);    
}

/*!
 *  /brief  Sends SPI-I2C packet to Z-board DV13131B
 *  
 *  /param  intf        Interface, see SPI-I2C packet description
 *  /param  devAddr     Device address, see SPI-I2C packet description
 *  /param  regAddr     Register address, see SPI-I2C packet description
 *  /param  data        Pointer to data to send to destination
 *  /param  numBytes    1 or 2 byte transaction
 *  
 *  /retval boolean to determine if transaction was valid
 */
spiState writeZBoard(uint8_t intf, uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t numBytes) {
    static uint8_t header = 0x00;
    static uint8_t read_cs = 0;
    static commPacket packet;
    static uint16_t tempVal = 0;     
    
    if (intf > 3) {
        DV_error("Incorrect write interface to Z-Board");
        return SPI_FAILED;
    }
    else if ((numBytes > 2) || (numBytes == 0)) {
        DV_error("Incorrect write byte transfer length to Z-Board");
        return SPI_FAILED;
    }
    /* Initialize header byte */    
    else {
        /* Set interface */
        header = intf;
        
        /* Set number of bytes */
        if (numBytes == 1) {
            clearBit(POS_BITS, &header);
        }
        else if (numBytes == 2) {
            setBit(POS_BITS, &header);
        }
    
        /* Set write command */
        clearBit(POS_RW, &header);        
    }
           
    //spi_write(SPI0, STROFFRAME, CS_ASSERTED, 0);
    //delay_us(SPI_DBC);
        
    spi_write(SPI0, header, CS_ASSERTED, 0);
    delay_us(SPI_DBC);
    
    spi_write(SPI0, devAddr, CS_ASSERTED, 0);
    delay_us(SPI_DBC);
        
    spi_write(SPI0, regAddr, CS_ASSERTED, 0);
    delay_us(SPI_DBC);
        
    if (numBytes == 1) {
        spi_write(SPI0, data[0], CS_ASSERTED, 0);
        delay_us(SPI_DBC);
        
        spi_write(SPI0, 0x00, CS_DEASSERTED, 1);
        delay_us(SPI_DBC);        
    }
    else {
        spi_write(SPI0, data[0], CS_ASSERTED, 0);
        delay_us(SPI_DBC);
        
        spi_write(SPI0, data[1], CS_DEASSERTED, 1);
        delay_us(SPI_DBC);        
    }
        
    /* Check whether command has been received by reading back ACK and data */
    delay_us(PROCESS_TIME);   
    
    spi_write(SPI0, 0xAA, CS_ASSERTED, 0);
    delay_us(SPI_DBC);
    spi_read(SPI0, &tempVal, &read_cs);     // read header
    packet.header = (uint8_t)tempVal;
    
    spi_write(SPI0, 0xBB, CS_ASSERTED, 0);
    delay_us(SPI_DBC);
    spi_read(SPI0, &tempVal, &read_cs);     // read device address
    packet.devAddr = (uint8_t)tempVal;

    spi_write(SPI0, 0xCC, CS_ASSERTED, 0);
    delay_us(SPI_DBC);
    spi_read(SPI0, &tempVal, &read_cs);     // read register address
    packet.regAddr = (uint8_t)tempVal;
    
    spi_write(SPI0, 0xDD, CS_ASSERTED, 0);
    delay_us(SPI_DBC);
    spi_read(SPI0, &tempVal, &read_cs);     // read MSB
    packet.data[0] = (uint8_t)tempVal;
    
    spi_write(SPI0, 0xEE, CS_DEASSERTED, 1);
    delay_us(SPI_DBC);
    spi_read(SPI0, &tempVal, &read_cs);     // read LSB
    packet.data[1] = (uint8_t)tempVal;   
    
    /* Error check return packet */
    if (packet.header != header) {
        //DV_error("Write header does not match; header: 0x%02X, expected: 0x%02X", packet.header, header);
        return SPI_FAILED;
    }
    else if (packet.devAddr!= devAddr) {
        //DV_error("Write device address does not match; devAddr: 0x%02X, expected: 0x%02X", packet.devAddr, devAddr);
        return SPI_FAILED;
    }
    else if (packet.regAddr!= regAddr) {
        //DV_error("Write register address does not match; regAddr: 0x%02X, expected: 0x%02X", packet.regAddr, regAddr);
        return SPI_FAILED;
    }
    else if (packet.data[0]!= ACK) {
        //DV_error("No ACK from Z-position board received! ack response: 0x%02X, expected: 0x%02X", packet.data[0], ACK);
        return SPI_FAILED;
    }    
            
    return SPI_SUCCESS;
}

/*!
 *  /brief  Read SPI-I2C packet from Z-board DV13131B
 *  
 *  /param  intf        Interface, see SPI-I2C packet description
 *  /param  devAddr     Device address, see SPI-I2C packet description
 *  /param  regAddr     Register address, see SPI-I2C packet description
 *  /param  data        Pointer to data to send to destination
 *  /param  numBytes    1 or 2 byte transaction
 *  
 *  /retval Boolean to determine if transaction was valid
 */
spiState readZBoard(uint8_t intf, uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t numBytes) {
    static uint8_t header = 0x00;
    static uint8_t read_cs = 0;
    static commPacket packet;
    static uint16_t tempVal = 0;    
    
    if (intf > 3) {
        DV_error("Incorrect read interface from Z-Board");
        return SPI_FAILED;
    }
    else if ((numBytes > 2) || (numBytes == 0)) {
        DV_error("Incorrect read byte transfer length to Z-Board");
        return SPI_FAILED;
    }
    /* Initialize header byte */    
    else {
        /* Set interface */
        header = intf;
        
        /* Set number of bytes */
        if (numBytes == 1) {
            clearBit(POS_BITS, &header);
        }
        else if (numBytes == 2) {
            setBit(POS_BITS, &header);
        }
    
        /* Set read command */
        setBit(POS_RW, &header);        
    }
        
    spi_write(SPI0, header, CS_ASSERTED, 0);
    delay_us(SPI_DBC);
    spi_write(SPI0, devAddr, CS_ASSERTED, 0);
    delay_us(SPI_DBC);
    spi_write(SPI0, regAddr, CS_ASSERTED, 0);
    delay_us(SPI_DBC);
    if (numBytes == 1) {
        spi_write(SPI0, data[0], CS_ASSERTED, 0);
        delay_us(SPI_DBC);
        spi_write(SPI0, 0x00, CS_DEASSERTED, 1);
        delay_us(SPI_DBC);        
    }
    else {
        spi_write(SPI0, data[0], CS_ASSERTED, 0);
        delay_us(SPI_DBC);
        spi_write(SPI0, data[1], CS_DEASSERTED, 1);
        delay_us(SPI_DBC);        
    }
    
    delay_us(PROCESS_TIME);
        
    spi_write(SPI0, 0x00, CS_ASSERTED, 0);
    delay_us(SPI_DBC);
    spi_read(SPI0, &tempVal, &read_cs);     // read header
    packet.header = (uint8_t)tempVal;       
    
    spi_write(SPI0, 0x00, CS_ASSERTED, 0);
    delay_us(SPI_DBC);
    spi_read(SPI0, &tempVal, &read_cs);     // read device address
    packet.devAddr = (uint8_t)tempVal;      

    spi_write(SPI0, 0x00, CS_ASSERTED, 0);
    delay_us(SPI_DBC);
    spi_read(SPI0, &tempVal, &read_cs);     // read register address
    packet.regAddr = (uint8_t)tempVal;
    
    spi_write(SPI0, 0x00, CS_ASSERTED, 0);
    delay_us(SPI_DBC);
    spi_read(SPI0, &tempVal, &read_cs);     // read MSB
    packet.data[0] = (uint8_t)tempVal;
    
    spi_write(SPI0, 0x00, CS_DEASSERTED, 1);
    delay_us(SPI_DBC);
    spi_read(SPI0, &tempVal, &read_cs);     // read LSB
    packet.data[1] = (uint8_t)tempVal;
   
    /* Error check return packet */
    if (packet.header != header) {
        //DV_error("Read header does not match; header: 0x%02X, expected: 0x%02X", packet.header, header);
        return SPI_FAILED;
    }
    else if (packet.devAddr!= devAddr) {
        //DV_error("Read device address does not match; devAddr: 0x%02X, expected: 0x%02X", packet.devAddr, devAddr);
        return SPI_FAILED;
    }
    else if (packet.regAddr!= regAddr) {
        //DV_error("Read register address does not match; regAddr: 0x%02X, expected: 0x%02X", packet.regAddr, regAddr);
        return SPI_FAILED;
    }    
    
    /* Return values from read */
    data[0] = packet.data[0];
    data[1] = packet.data[1];
    
    return SPI_SUCCESS;
}

/*!
 *  /brief  Sends SPI-I2C packet to Z-board DV13131B with embedded retries
 *  
 *  /param  intf        Interface, see SPI-I2C packet description
 *  /param  devAddr     Device address, see SPI-I2C packet description
 *  /param  regAddr     Register address, see SPI-I2C packet description
 *  /param  data        Pointer to data to send to destination
 *  /param  numBytes    1 or 2 byte transaction
 *  
 *  /retval boolean to determine if transaction was valid
 */
spiState writeZBoardMulti(uint8_t intf, uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t numBytes) {
    const uint8_t maxNumRetries = 3;
    uint8_t numRetries = 0;
    
    while(numRetries < maxNumRetries) {
        if (writeZBoard(intf, devAddr, regAddr, data, numBytes) == 0) {
            numRetries++;
            if (numRetries == maxNumRetries) {
                DV_error("Bad Z-Board SPI write after %u retries!", maxNumRetries);                   
                return SPI_FAILED;
            }
            delay_us(numRetries*10);
        }
        else {
            break;            
        }
    }
    
    return SPI_SUCCESS;
}

/*!
 *  /brief  Read SPI-I2C packet from Z-board DV13131B with embedded retries
 *  
 *  /param  intf        Interface, see SPI-I2C packet description
 *  /param  devAddr     Device address, see SPI-I2C packet description
 *  /param  regAddr     Register address, see SPI-I2C packet description
 *  /param  data        Pointer to data to send to destination
 *  /param  numBytes    1 or 2 byte transaction
 *  
 *  /retval boolean to determine if transaction was valid
 */
spiState readZBoardMulti(uint8_t intf, uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t numBytes) {
    const uint8_t maxNumRetries = 3;
    uint8_t numRetries = 0;
    
    while(numRetries < maxNumRetries) {
        if (readZBoard(intf, devAddr, regAddr, data, numBytes) == 0) {
            numRetries++;
            if (numRetries == maxNumRetries) {
                DV_error("Bad Z-Board SPI read after %u retries!", maxNumRetries);   
                return SPI_FAILED;
            }
            delay_us(numRetries*10);
        }
        else {
            break;
        }
    }
    
    return SPI_SUCCESS;
}

/*!
 *  /brief  Enables on-board IR
 *  
 *  /param  none
 *  
 *  /retval none
 */
void enableIr(void) {
    uint8_t command[2] = {0x01, 0x00};
    
    if (writeZBoardMulti(SPI_COM, 0x00, REG_IR_CTRL, command, 2)) {
        DV_info("IR turned on!");
    } 
    else {
        DV_error("IR did not turn on successfully!");
    }
}

/*!
 *  /brief  Disables on-board IR
 *  
 *  /param  none
 *  
 *  /retval none
 */
void disableIr(void) {
    uint8_t command[2] = {0x00, 0x00};
    
    if (writeZBoardMulti(SPI_COM, 0x00, REG_IR_CTRL, command, 2)) {
        DV_info("IR turned off!");
    }
    else {
        DV_error("IR did not turn off successfully!");
    }
}

/*!
 *  /brief  Enables on-board green LED
 *  
 *  /param  none
 *  
 *  /retval none
 */
void enableGLed(void) {
    uint8_t command[2] = {0x00, 0x00};
    
    if (writeZBoardMulti(SPI_COM, 0x00, REG_G_LED, command, 2)) {
        DV_info("Green LED turned on!");
    } 
    else {
        DV_error("Green LED did not turn on successfully!");
    }
}

/*!
 *  /brief  Disables on-board green LED
 *  
 *  /param  none
 *  
 *  /retval none
 */
void disableGLed(void) {
    uint8_t command[2] = {0x01, 0x00};
    
    if (writeZBoardMulti(SPI_COM, 0x00, REG_G_LED, command, 2)) {
        DV_info("Green LED turned off!");
    }
    else {
        DV_error("Green LED did not turn off successfully!");
    }
}
 
/*!
 *  /brief  Enables on-board red LED
 *  
 *  /param  none
 *  
 *  /retval none
 */
void enableRLed(void) {
    uint8_t command[2] = {0x00, 0x00};
    
    if (writeZBoardMulti(SPI_COM, 0x00, REG_G_LED, command, 2)) {
        DV_info("Red LED turned on!");
    } 
    else {
        DV_error("Red LED did not turn on successfully!");
    }
}

/*!
 *  /brief  Disables on-board red LED
 *  
 *  /param  none
 *  
 *  /retval none
 */
void disableRLed(void) {
    uint8_t command[2] = {0x01, 0x00};
    
    if (writeZBoardMulti(SPI_COM, 0x00, REG_G_LED, command, 2)) {
        DV_info("Red LED turned off!");
    }
    else {
        DV_error("Red LED did not turn off successfully!");
    }
}
 
/*!
*  /brief  Obtains raw 12-bit ADC value from Z-board of IR analog value
*  
*  /param  none
*  
*  /retval 12-bit ADC value
*/   
uint16_t getIrAnalogVal(void) {
    uint8_t rxData[2] = {0x00, 0x00};
    uint16_t rawAdc = 0;

    if (readZBoardMulti(SPI_COM, 0x00, REG_IR_VAL, rxData, 2)) {
        rawAdc = (rxData[0] << 8) | rxData[1];
        //DV_info("IR Value: %.2f", (3.3*rawAdc)/4095);
    }
    else {
        DV_error("Unable to read IR value");
        return 0;        
    }    
    
    return rawAdc;
}

/*!
*  /brief  Obtains raw 12-bit ADC value from Z-board analog channel
*  
*  /param  channel     Channel to obtain result from 0 or 1
*  
*  /retval 12-bit ADC value from specific channel
*/   
uint16_t getAnalogVal(uint8_t channel) {
    uint8_t rxData[2] = {0x00, 0x00};
    uint16_t rawAdc = 0;
    uint8_t reg = REG_ANALOG0;  
    
    if (channel == 0) {
        reg = REG_ANALOG0;
    }
    else if (channel == 1) {
        reg = REG_ANALOG1;
    }
    else {
        DV_error("getIrAnalogVal channel not supported!");
    }
    
    if (readZBoardMulti(SPI_COM, 0x00, reg, rxData, 2)) {
        rawAdc = (rxData[0] << 8) | rxData[1];
        DV_info("Analog Value[%u]: %.2f", channel, (3.3*rawAdc)/4095);        
    }
    else {
        DV_error("Unable to read Z-board analog value on channel: %u", channel);
        return 0;
    }
    
    return rawAdc;
}

/*!
*  /brief  Obtains raw 12-bit ADC value from spine board ADC
*  
*  /param  channel     Channel to obtain result from 0, 1, 2, or 3
*  
*  /retval 12-bit ADC value from specific channel
*/   
uint16_t getSpineBoardAdc(uint8_t channel) {
    uint8_t rxData[2] = {0x00, 0x00};
    uint8_t txData;
    uint16_t rawAdc = 0;
    uint8_t returnCh = 0;           // holds which channel the raw ADC came from
    
    switch (channel) {
        case 0:
            txData = 0x10;
            break;
            
        case 1:
            txData = 0x20;        
            break;
            
        case 2:
            txData = 0x40;        
            break;
            
        case 3:
            txData = 0x80;                
            break;
            
        default:
            break;        
    }
    
    /*  Write configuration byte to select channel */
    if(writeZBoardMulti(I2C0_COM, DEV_ID_AD7991_1, 0x00, &txData, 1) == 0) {
        DV_error("Not able write configuration byte to spine board ADC!");
        return 0;
    }

    /*  The AD7991 ADC does not follow the standard I2C protocol of having a register address so using 0x00 in place
     *  The operation will be adjusted and accounted for on the Z-board firmware
     */        
    if(readZBoardMulti(I2C0_COM, DEV_ID_AD7991_1, 0x00, rxData, 2)) {
        returnCh = rxData[0] >> 4;
        rawAdc = ((rxData[0] & 0x0F) << 8) | rxData[1];    
        
        DV_info("Spine Board Analog Value[%u]: %.2f", returnCh, (3.3*rawAdc)/4095);          
        return rawAdc;
    }
    else {
        DV_error("Not able read from spine board ADC!");
        return 0;
    }        
}

/*!
*  /brief  Test communication link with Z-board by writing and reading back a register
*
*  /param  none
*
*  /retval SPI_SUCCESS if communication OK, otherwise, SPI_FAILED
*/
spiState testZBoardIntf(void) {
    uint8_t txData[2] = {0x01, 0x00};
    uint8_t rxData[2] = {0x00, 0x00};
    writeZBoardMulti(SPI_COM, 0x00, REG_R_LED, txData, 1);
    readZBoardMulti(SPI_COM, 0x00, REG_R_LED, rxData, 1);
    
    if (rxData[0] != txData[0]) {
        return SPI_FAILED;
    }
    
    txData[0] = 0x00;
    txData[1] = 0x00;
    rxData[0] = 0xFF;
    rxData[1] = 0x00;
    
    writeZBoardMulti(SPI_COM, 0x00, REG_R_LED, txData, 1);
    readZBoardMulti(SPI_COM, 0x00, REG_R_LED, rxData, 1);
    
    if (rxData[0] != txData[0]) {
        return SPI_FAILED;
    }
    
    return SPI_SUCCESS;
}