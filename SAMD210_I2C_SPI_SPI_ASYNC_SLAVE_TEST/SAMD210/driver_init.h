/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef DRIVER_INIT_INCLUDED
#define DRIVER_INIT_INCLUDED

#include "atmel_start_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <hal_atomic.h>
#include <hal_delay.h>
#include <hal_gpio.h>
#include <hal_init.h>
#include <hal_io.h>
#include <hal_sleep.h>

#include <hal_adc_async.h>

#include <hal_i2c_m_sync.h>

#include <hal_i2c_m_sync.h>

#include <hal_i2c_m_sync.h>

#include <hal_spi_s_async.h>

#include <hal_usart_sync.h>
#include <hal_timer.h>
#include <hpl_tc_base.h>

extern struct adc_async_descriptor ADC_0;

extern struct i2c_m_sync_desc I2C_0;

extern struct i2c_m_sync_desc I2C_1;

extern struct i2c_m_sync_desc I2C_2;

extern struct spi_s_async_descriptor SPI_0;

extern struct usart_sync_descriptor TARGET_IO;
extern struct timer_descriptor      TIMER_0;

void ADC_0_init(void);

void I2C_0_CLOCK_init(void);
void I2C_0_init(void);
void I2C_0_PORT_init(void);

void I2C_1_CLOCK_init(void);
void I2C_1_init(void);
void I2C_1_PORT_init(void);

void I2C_2_CLOCK_init(void);
void I2C_2_init(void);
void I2C_2_PORT_init(void);

void SPI_0_PORT_init(void);
void SPI_0_CLOCK_init(void);
void SPI_0_init(void);

void TARGET_IO_PORT_init(void);
void TARGET_IO_CLOCK_init(void);
void TARGET_IO_init(void);

/**
 * \brief Perform system initialization, initialize pins and clocks for
 * peripherals
 */
void system_init(void);

#ifdef __cplusplus
}
#endif
#endif // DRIVER_INIT_INCLUDED
