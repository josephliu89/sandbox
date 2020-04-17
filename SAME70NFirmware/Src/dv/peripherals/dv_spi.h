/*
 * dv_i2c.h
 *
 * Created: 11/13/2019 4:30:40 PM
 *  Author: liu
 */ 

#ifndef DV_SPI_H_
#define DV_SPI_H_

/* SPI0 definitions */
#define SPI_CHIP_SEL				0			// Chip select
#define SPI_CHIP_PCS				spi_get_pcs(SPI_CHIP_SEL)
#define SPI_CLK_POLARITY			0			// Clock polarity
#define SPI_CLK_PHASE				1			// Clock phase
#define SPI_DLYBS					0x40		// Delay before SPCK
#define SPI_DLYBCT					0x10		// Delay between consecutive transfers

//
// to maintain compatibility with the hunter SPI interface we will
// need to send the same packet header and footer that Kelvin is
// expecting in the firmware, so we need to wrap all of our
// transmission byte stream with this header and footer
//
// command is used here for hunter transmissions, for DV telemetry
// we can just set it to a known value and then use that
// to differentiate whether or not we are connected via hunter
// telemetry...
//
// the hunter command enum look like this:
// 
//  enum class HwsCommand : uint8_t {
//      INVALID = 0,
//      SET_ADDRESS = 0x06, 				 // Send from surface to configure tool address (used by PL directly)
//      SET_ADDRESS_RESPONSE = 0x07, 		 // Response to address configuration (send by PL directly)
//      TARP_ENTRY = 0x12, 					 // Defines the address of our tool/tools
//      DOWNLINK_REQUEST = 0x20,
//      SENSOR_RESPONSE = 0x22,
//      SENSOR_DATA_RESPONSE = 0x32,
//      SENSOR_DATA_RESPONSE_MULTIPACKET = 0x33,
//      TELEMETRY_BROADCAST = 0xff,	 		 // Sent from the Hunter telemetry board with no data (need to ignore these)
//  };
// 
typedef struct {
    uint8_t m_address;
    uint8_t m_command;
    uint8_t m_length;
} SpiHeader;

typedef struct {
    uint8_t m_crc[2];
} SpiFooter;

void init_spi_slave(void);
void handleSpiRxData(void);
void handleSpiTxData(void);

#endif	/* DV_SPI_H_ */