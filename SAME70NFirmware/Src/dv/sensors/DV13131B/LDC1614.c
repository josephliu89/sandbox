#include "dv/sensors/LDC1614.h"
#include "dv/sensors/DV13131B/DV13131B.h"
//#include "dv/communication/dv_console.h"

static const uint8_t tx_config_addr[] = {
	RCOUNT0_ADDR,
	RCOUNT1_ADDR,
	RCOUNT2_ADDR,
	RCOUNT3_ADDR,
	SETTLECOUNT0_ADDR,
	SETTLECOUNT1_ADDR,
	SETTLECOUNT2_ADDR,
	SETTLECOUNT3_ADDR,
	CLOCK_DIVIDERS0_ADDR,
	CLOCK_DIVIDERS1_ADDR,
	CLOCK_DIVIDERS2_ADDR,
	CLOCK_DIVIDERS3_ADDR,
	ERROR_CONFIG_ADDR,
	MUX_CONFIG_ADDR,
	//DRIVE_CURRENT0_ADDR,
	//DRIVE_CURRENT1_ADDR,
	CONFIG_ADDR
};
static const uint8_t tx_config_value[] = {	
	0xFF, 0xFF,
	0xFF, 0xFF,
	0xFF, 0xFF,
	0xFF, 0xFF,
	0x04, 0x00,
	0x04, 0x00,
	0x04, 0x00,
	0x04, 0x00,
	0x10, 0x01,
	0x10, 0x01,
	0x10, 0x01,
	0x10, 0x01,
	0x00, 0x01,
	0xC2, 0x0D,
	//0x8C, 0x40,
	//0x8C, 0x40,
	0x1C, 0x01
};
static const uint8_t rx_ch_addr[] =	{	
	DATA0_MSB_ADDR,
	DATA0_LSB_ADDR,
	DATA1_MSB_ADDR,
	DATA1_LSB_ADDR,
	DATA2_MSB_ADDR,
	DATA2_LSB_ADDR,
	DATA3_MSB_ADDR,
	DATA3_LSB_ADDR
};

#define RX_DATA_LENGTH  (sizeof(rx_ch_addr) / sizeof(uint8_t))
#define TX_CONFIG_LENGTH  (sizeof(tx_config_addr) / sizeof(uint8_t))
#define RX_CONFIG_LENGTH  TX_CONFIG_LENGTH

/*!
 *  /brief  Configures LDC1614 and validates configuration via SPI using Z-position board to perform I2C transactions.
 *  
 *  /param  none
 *  
 *  /retval none
 */
void initLDC(void) {
    uint8_t i;
	uint8_t rx_data[2] = {0};
        
    /* Write default configuration to LDC1614 */  
      
	for (i = 0; i < TX_CONFIG_LENGTH; i++) {
    	if (writeZBoardMulti(I2C0_COM, LDC1614_ID_ADDR, tx_config_addr[i], (uint8_t *)&tx_config_value[2*i], 2) != SPI_SUCCESS) {
        	DV_error("Writing individual LDC1614 default configuration failed!");
    	}
    	//delay_ms(WAIT_TIME);        
	}                 
    
	/* Read back configuration */

	for (i = 0; i < RX_CONFIG_LENGTH; i++) {
    	if (readZBoardMulti(I2C0_COM, LDC1614_ID_ADDR, tx_config_addr[i], rx_data, 2) != SPI_SUCCESS) {
        	DV_error("Reading individual LDC1614 default configuration failed!");
        	return;
    	}
    	//delay_ms(WAIT_TIME);                        
        
        //printf("txdata[%i]: \t0x%02X\trxdata[%i]: \t0x%02X\ttxdata[%i]: \t0x%02X\trxdata[%i]: \t0x%02X\r\n",
                //2*i, tx_config_value[2*i],
                //2*i, rx_data[0],
                //(2*i)+1, tx_config_value[(2*i)+1],
                //(2*i)+1, rx_data[1]);

    	/* Compare values originally sent and values read from device */
    	if ((tx_config_value[2*i] != rx_data[0]) || (tx_config_value[(2*i)+1] != rx_data[1])) {
        	DV_error("Coil board configuration read-back error at index i = %u!", i);
    	}        
	}
}

/*!
 *  /brief  Gets raw coil data via SPI using Z-position board to perform I2C transactions.
 *  
 *  /param  coil_data   Pointer to array hold coil data
 *  
 *  /retval bool        Returns true if there are no errors
 */
bool getCoilValues(uint32_t coil_data [NUM_COILS]) {
	
	uint32_t i;
	uint8_t rx_ch_data[2*RX_DATA_LENGTH] = {0};
	
	for (i = 0; i < RX_DATA_LENGTH; i++) {
        if (readZBoardMulti(I2C0_COM, LDC1614_ID_ADDR, rx_ch_addr[i], &rx_ch_data[2*i], 2) != SPI_SUCCESS) {
			DV_error("Reading individual coil data failed!");
			return false;
		}
	}
	
	for (i = 0; i < NUM_COILS; i++)
	{
		coil_data[i] = rx_ch_data[i*4] << 24 | rx_ch_data[i*4+1] << 16 | rx_ch_data[i*4+2] << 8 | rx_ch_data[i*4+3];
	}
	
 	return true;
}