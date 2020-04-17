/**
 * \file
 *
 * \brief User board initialization template
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include <asf.h>
#include <board.h>
#include <conf_board.h>

/**
 * \brief Set peripheral mode for IOPORT pins.
 * It will configure port mode and disable pin mode (but enable peripheral).
 * \param port IOPORT port to configure
 * \param masks IOPORT pin masks to configure
 * \param mode Mode masks to configure for the specified pin (\ref ioport_modes)
 */
#define ioport_set_port_peripheral_mode(port, masks, mode) \
	do { \
		ioport_set_port_mode(port, masks, mode); \
		ioport_disable_port(port, masks); \
	} \
	while (0)

/**
 * \brief Set peripheral mode for one single IOPORT pin.
 * It will configure port mode and disable pin mode (but enable peripheral).
 * \param pin IOPORT pin to configure
 * \param mode Mode masks to configure for the specified pin (\ref ioport_modes)
 */
#define ioport_set_pin_peripheral_mode(pin, mode) \
	do { \
		ioport_set_pin_mode(pin, mode);	\
		ioport_disable_pin(pin); \
	} \
	while (0)

/**
 * \brief Set input mode for one single IOPORT pin.
 * It will configure port mode and disable pin mode (but enable peripheral).
 * \param pin IOPORT pin to configure
 * \param mode Mode masks to configure for the specified pin (\ref ioport_modes)
 * \param sense Sense for interrupt detection (\ref ioport_sense)
 */
#define ioport_set_pin_input_mode(pin, mode, sense) \
	do { \
		ioport_set_pin_dir(pin, IOPORT_DIR_INPUT); \
		ioport_set_pin_mode(pin, mode);	\
		ioport_set_pin_sense_mode(pin, sense); \
	} \
	while (0)

void board_init(void)
{
    /* Disable watchdog for now */
    WDT->WDT_MR = WDT_MR_WDDIS;
    
    ioport_init();

    /* Configure ATPL360 SPI pins */
    ioport_set_pin_peripheral_mode(SPI0_MISO_GPIO, SPI0_MISO_FLAGS);
    ioport_set_pin_peripheral_mode(SPI0_MOSI_GPIO, SPI0_MOSI_FLAGS);
    ioport_set_pin_peripheral_mode(SPI0_SPCK_GPIO, SPI0_SPCK_FLAGS);
    ioport_set_pin_peripheral_mode(SPI0_NPCS3_GPIO, SPI0_NPCS3_FLAGS);

    /* Configure USART/SPI pins */
    ioport_set_pin_peripheral_mode(PIN_USART0_TXD_IDX, PIN_USART0_TXD_FLAGS);
    ioport_set_pin_peripheral_mode(PIN_USART0_RXD_IDX, PIN_USART0_RXD_FLAGS);
    ioport_set_pin_peripheral_mode(PIN_USART0_SCK_IDX, PIN_USART0_SCK_FLAGS);
    ioport_set_pin_peripheral_mode(PIN_USART0_CTS_IDX, PIN_USART0_CTS_FLAGS);

    /* Configure I2C pins */
    ioport_set_pin_peripheral_mode(TWIHS0_DATA_GPIO, TWIHS0_DATA_FLAGS);
    ioport_set_pin_peripheral_mode(TWIHS0_CLK_GPIO, TWIHS0_CLK_FLAGS);

    /* Configure UART pins */
    ioport_set_pin_peripheral_mode(UART0_RXD_GPIO, UART0_RXD_FLAGS);
    ioport_set_pin_peripheral_mode(UART0_TXD_GPIO, UART0_TXD_FLAGS);

    /* Configure LEDx pins */
    ioport_set_pin_dir(NRLED_PIN, IOPORT_DIR_OUTPUT);
    ioport_set_pin_dir(NGLED_PIN, IOPORT_DIR_OUTPUT);
    ioport_set_pin_level(NRLED_PIN, 1);        // off by default
    ioport_set_pin_level(NGLED_PIN, 1);        // off by default

    /* Configure mode detection pin */
    ioport_set_pin_dir(CTRL_NDEV, IOPORT_DIR_INPUT);
 
    /* Configure incoming voltage detection pin */
    ioport_set_pin_dir(ANA_VBAT_PIN, IOPORT_DIR_INPUT);

    /* Configure LT8619 sync signal */
	ioport_set_pin_mode(PIO_PA0_IDX, IOPORT_MODE_MUX_B);
	ioport_disable_pin(PIO_PA0_IDX);

    /* Configure start signal for PoEM */
    ioport_set_pin_dir(NSTART, IOPORT_DIR_OUTPUT);
    ioport_set_pin_level(NSTART, 1);        // off by default

    /* Configure shutdown signal from PoEM */
    ioport_set_pin_dir(NSHDN, IOPORT_DIR_INPUT);
}
