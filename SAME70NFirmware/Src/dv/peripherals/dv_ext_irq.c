/*
 * dv_ext_irq.c
 *
 * Created: 11/14/2019 11:25:45 AM
 *  Author: liu
 */ 

#include <asf.h>
#include "dv_ext_irq.h"

/**
 * \brief De-assert start pin when shutdown pin is asserted
 */
static void handler_shdn(long unsigned int unused0,  long unsigned int unused1)
{
	/* Disable front-end DC-DC converter on PoEM board */
    DV_unused(unused0);
    DV_unused(unused1);
	gpio_set_pin_high(NSTART);
	DV_info("Shutdown pin asserted, powering down tool...");
}

/**
 * \brief Initialize external interrupt for nSHDN pin (PA18)
 */
void init_shdn_ext_irq(void)
{
	/* Configure PA26 as input with pull-up and debouncing */
	pio_set_input(PIN_SHDN_PIO, PIN_SHDN_PIO_MASK, PIO_INPUT | PIO_PULLUP | PIO_DEBOUNCE);

	/* Configure debounce filter at 25Hz */
	pio_set_debounce_filter(PIN_SHDN_PIO, PIN_SHDN_PIO_MASK, 25);

	/* Configure external interrupt on falling edge of PA26 */
	pio_handler_set(PIN_SHDN_PIO, PIN_SHDN_PIO_ID, PIN_SHDN_PIO_MASK, PIN_SHDN_ATTR, handler_shdn);
	
	/* Clear interrupt status */
	pio_get_interrupt_status(PIN_SHDN_PIO);

	/* Enable PIO line interrupts. */
	pio_enable_interrupt(PIN_SHDN_PIO, PIN_SHDN_PIO_MASK);

	/* Configure external interrupt in NVIC */
	NVIC_EnableIRQ((IRQn_Type)PIN_SHDN_PIO_ID);
}