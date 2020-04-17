#ifndef LDC1614_H
#define LDC1614_H

#include <asf.h>
//#include "dv/sensors/twi_wrapper.h"

/** Wait Time */
#define WAIT_TIME   10

//Number of coils (visible for external calls///
#define NUM_COILS 4

/** TWI Bus Clock 100kHz */
#define TWIHS_CLK     100000

/** LDC1614 Registers */
#define LDC1614_ID_ADDR			0x2A
#define	DATA0_MSB_ADDR          0x00
#define	DATA0_LSB_ADDR	        0x01
#define	DATA1_MSB_ADDR   	    0x02
#define	DATA1_LSB_ADDR   	    0x03
#define	DATA2_MSB_ADDR   	    0x04
#define	DATA2_LSB_ADDR   	    0x05
#define	DATA3_MSB_ADDR   	    0x06
#define	DATA3_LSB_ADDR   	    0x07
#define	RCOUNT0_ADDR            0x08
#define	RCOUNT1_ADDR            0x09
#define	RCOUNT2_ADDR            0x0A
#define	RCOUNT3_ADDR            0x0B
#define	OFFSET0_ADDR            0x0C
#define	OFFSET1_ADDR            0x0D
#define	OFFSET2_ADDR            0x0E
#define	OFFSET3_ADDR            0x0F
#define	SETTLECOUNT0_ADDR       0x10
#define	SETTLECOUNT1_ADDR	    0x11
#define	SETTLECOUNT2_ADDR	    0x12
#define	SETTLECOUNT3_ADDR	    0x13
#define	CLOCK_DIVIDERS0_ADDR	0x14
#define	CLOCK_DIVIDERS1_ADDR	0x15
#define	CLOCK_DIVIDERS2_ADDR	0x16
#define	CLOCK_DIVIDERS3_ADDR	0x17
#define	STATUS_ADDR	            0x18
#define	ERROR_CONFIG_ADDR	    0x19
#define	CONFIG_ADDR	            0x1A
#define	MUX_CONFIG_ADDR	        0x1B
#define	RESET_DEV_ADDR	        0x1C
#define	DRIVE_CURRENT0_ADDR	    0x1E
#define	DRIVE_CURRENT1_ADDR	    0x1F
#define	DRIVE_CURRENT2_ADDR	    0x20
#define	DRIVE_CURRENT3_ADDR	    0x21
#define	MANUFACTURER_ID_ADDR	0x7E
#define	DEVICE_ID_ADDR	        0x7F

typedef struct spi_packet {
	//! TWIHS address/commands to issue to the other chip (node).
	uint8_t addr[3];
	//! Length of the TWIHS data address segment (1-3 bytes).
	uint32_t addr_length;
	//! Where to find the data to be transferred.
	void *buffer;
	//! How many bytes do we want to transfer.
	uint32_t length;
	//! TWIHS chip address to communicate with.
	uint8_t chip;
} spi_packet_t;

#if (SAMV70 || SAMV71 || SAME70 || SAMS70)
/** TWI ID for simulated LDC1614 application to use */
#define BOARD_ID_TWIHS_LDC1614         ID_TWIHS0
/** TWI Base for simulated TWI LDC1614 application to use */
#define BOARD_BASE_TWIHS_LDC1614       TWIHS0
#endif

void initLDC(void);
bool getCoilValues(uint32_t coil_data [NUM_COILS]);

#endif //LDC1614_H