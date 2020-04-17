/*
 * dv_i2c.c
 *
 * Created: 11/13/2019 4:29:50 PM
 *  Author: liu
 */ 

#include "asf.h"
#include "dv_i2c.h"

/**
 * \brief Configure and initialize I2C interface
 */
void init_i2c(void) 
{
    /* Enable peripheral clock for TWI */
    pmc_enable_periph_clk(ID_TWIHS0);

    /* Configure the options of TWI driver */
    twihs_options_t opt;
    opt.master_clk = sysclk_get_peripheral_hz();
    opt.speed      = 100000;        // TWI bus clock set to 100 kHz

    /* Initialize I2C bus with proper speed */
    if (twihs_master_init(TWIHS0, &opt) != TWIHS_SUCCESS) {
        DV_error("I2C master interface initialized failed.");
    }
    else {
        DV_info("I2C master interface initialized successfully!");
    }
}

/**
 * \brief Test code for I2C communication	
 */
void test_i2c(void) 
{
	twihs_packet_t packet_tx, packet_rx;			// Struct for TWI packet
	uint8_t tx_value[2] = { 0 };					// TODO: Remove, used to test interface; I2C transmit buffer
	uint8_t rx_value[2] = { 0 };					// TODO: Remove, used to test interface; I2C receive buffer

	/* Configure the data packet to be transmitted */
	packet_tx.chip        = RTC_MCP;
	packet_tx.addr[0]     = 0x00;
	packet_tx.addr_length = 1;
	packet_tx.buffer      = &tx_value;
	packet_tx.length      = 1;

	/* Configure the data packet to be received */
	packet_rx.chip        = RTC_MCP;
	packet_rx.addr[0]     = 0x03;
	packet_rx.addr_length = 1;
	packet_rx.buffer      = &rx_value;
	packet_rx.length      = 1;

	/* Probe I2C bus for device */
	if (twihs_probe(TWIHS0, RTC_MCP) != TWIHS_SUCCESS) {
		puts("-E-\tTWI probe failed.\r");
	}

	/* Sample I2C write */
	if (twihs_master_write(TWIHS0, &packet_tx) != TWIHS_SUCCESS) {
		puts("-E-\tTWI master tx packet failed.\r");
	}

	/* Sample I2C read */
	if (twihs_master_read(TWIHS0, &packet_rx) != TWIHS_SUCCESS) {
		puts("-E-\tTWI master rx packet failed.\r");
	}
		
	printf("[INFO]\tI2C read-back\t\tSlave Address: 0x%X\tRegister: 0x%X\tValue: 0x%X\r\n", packet_rx.chip, packet_rx.addr[0], rx_value[0]);
}
