/*
 * dv_ext_irq.h
 *
 * Created: 11/14/2019 11:25:32 AM
 *  Author: liu
 */ 

#ifndef DV_EXT_IRQ_H_
#define DV_EXT_IRQ_H_

#define PIN_SHDN_PIO		PIOA
#define PIN_SHDN_PIO_MASK   PIO_PA1
#define PIN_SHDN_PIO_ID     ID_PIOA
#define PIN_SHDN_ATTR		PIO_IT_FALL_EDGE

void init_shdn_ext_irq(void);

#endif /* DV_EXT_IRQ_H_ */