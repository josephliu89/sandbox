/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>
#include <hpl_gclk_base.h>
#include <hpl_pm_base.h>

#include <hpl_adc_base.h>

/* The channel amount for ADC */
#define ADC_0_CH_AMOUNT 1

/* The buffer size for ADC */
#define ADC_0_BUFFER_SIZE 16

/* The maximal channel number of enabled channels */
#define ADC_0_CH_MAX 0

struct adc_async_descriptor         ADC_0;
struct adc_async_channel_descriptor ADC_0_ch[ADC_0_CH_AMOUNT];
struct timer_descriptor             TIMER_0;

static uint8_t ADC_0_buffer[ADC_0_BUFFER_SIZE];
static uint8_t ADC_0_map[ADC_0_CH_MAX + 1];

struct i2c_m_sync_desc I2C_0;

struct i2c_m_sync_desc I2C_1;

struct i2c_m_sync_desc I2C_2;

struct spi_s_async_descriptor SPI_0;
static uint16_t               SPI_0_buf[16];

struct usart_sync_descriptor TARGET_IO;

/**
 * \brief ADC initialization function
 *
 * Enables ADC peripheral, clocks and initializes ADC driver
 */
void ADC_0_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, ADC);
	_gclk_enable_channel(ADC_GCLK_ID, CONF_GCLK_ADC_SRC);
	adc_async_init(&ADC_0, ADC, ADC_0_map, ADC_0_CH_MAX, ADC_0_CH_AMOUNT, &ADC_0_ch[0], (void *)NULL);
	adc_async_register_channel_buffer(&ADC_0, 0, ADC_0_buffer, ADC_0_BUFFER_SIZE);

	// Disable digital pin circuitry
	gpio_set_pin_direction(IR_READING, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(IR_READING, PINMUX_PA02B_ADC_AIN0);

	// Disable digital pin circuitry
	gpio_set_pin_direction(AUX, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(AUX, PINMUX_PA03B_ADC_AIN1);

	// Disable digital pin circuitry
	gpio_set_pin_direction(PA06, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(PA06, PINMUX_PA06B_ADC_AIN6);
}

void I2C_0_PORT_init(void)
{

	gpio_set_pin_pull_mode(SDA0,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(SDA0, PINMUX_PA16C_SERCOM1_PAD0);

	gpio_set_pin_pull_mode(SCL0,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(SCL0, PINMUX_PA17C_SERCOM1_PAD1);
}

void I2C_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM1);
	_gclk_enable_channel(SERCOM1_GCLK_ID_CORE, CONF_GCLK_SERCOM1_CORE_SRC);
	_gclk_enable_channel(SERCOM1_GCLK_ID_SLOW, CONF_GCLK_SERCOM1_SLOW_SRC);
}

void I2C_0_init(void)
{
	I2C_0_CLOCK_init();
	i2c_m_sync_init(&I2C_0, SERCOM1);
	I2C_0_PORT_init();
}

void I2C_1_PORT_init(void)
{

	gpio_set_pin_pull_mode(SDA1,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(SDA1, PINMUX_PA08D_SERCOM2_PAD0);

	gpio_set_pin_pull_mode(SCL1,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(SCL1, PINMUX_PA09D_SERCOM2_PAD1);
}

void I2C_1_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM2);
	_gclk_enable_channel(SERCOM2_GCLK_ID_CORE, CONF_GCLK_SERCOM2_CORE_SRC);
	_gclk_enable_channel(SERCOM2_GCLK_ID_SLOW, CONF_GCLK_SERCOM2_SLOW_SRC);
}

void I2C_1_init(void)
{
	I2C_1_CLOCK_init();
	i2c_m_sync_init(&I2C_1, SERCOM2);
	I2C_1_PORT_init();
}

void I2C_2_PORT_init(void)
{

	gpio_set_pin_pull_mode(SDA2,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(SDA2, PINMUX_PA22C_SERCOM3_PAD0);

	gpio_set_pin_pull_mode(SCL2,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(SCL2, PINMUX_PA23C_SERCOM3_PAD1);
}

void I2C_2_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM3);
	_gclk_enable_channel(SERCOM3_GCLK_ID_CORE, CONF_GCLK_SERCOM3_CORE_SRC);
	_gclk_enable_channel(SERCOM3_GCLK_ID_SLOW, CONF_GCLK_SERCOM3_SLOW_SRC);
}

void I2C_2_init(void)
{
	I2C_2_CLOCK_init();
	i2c_m_sync_init(&I2C_2, SERCOM3);
	I2C_2_PORT_init();
}

void SPI_0_PORT_init(void)
{

	gpio_set_pin_level(MISO,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(MISO, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(MISO, PINMUX_PB08D_SERCOM4_PAD0);

	// Set pin direction to input
	gpio_set_pin_direction(CS_B, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(CS_B,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(CS_B, PINMUX_PB09D_SERCOM4_PAD1);

	// Set pin direction to input
	gpio_set_pin_direction(MOSI, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(MOSI,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(MOSI, PINMUX_PB10D_SERCOM4_PAD2);

	gpio_set_pin_level(SCK,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(SCK, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(SCK, PINMUX_PB11D_SERCOM4_PAD3);
}

void SPI_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM4);
	_gclk_enable_channel(SERCOM4_GCLK_ID_CORE, CONF_GCLK_SERCOM4_CORE_SRC);
}

void SPI_0_init(void)
{
	SPI_0_CLOCK_init();
	spi_s_async_init(&SPI_0, SERCOM4, (uint8_t *)SPI_0_buf, 32);
	SPI_0_PORT_init();
}

void TARGET_IO_PORT_init(void)
{

	gpio_set_pin_function(PB00, PINMUX_PB00D_SERCOM5_PAD2);

	gpio_set_pin_function(PB01, PINMUX_PB01D_SERCOM5_PAD3);
}

void TARGET_IO_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM5);
	_gclk_enable_channel(SERCOM5_GCLK_ID_CORE, CONF_GCLK_SERCOM5_CORE_SRC);
}

void TARGET_IO_init(void)
{
	TARGET_IO_CLOCK_init();
	usart_sync_init(&TARGET_IO, SERCOM5, (void *)NULL);
	TARGET_IO_PORT_init();
}

/**
 * \brief Timer initialization function
 *
 * Enables Timer peripheral, clocks and initializes Timer driver
 */
static void TIMER_0_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, TC3);
	_gclk_enable_channel(TC3_GCLK_ID, CONF_GCLK_TC3_SRC);

	timer_init(&TIMER_0, TC3, _tc_get_timer());
}

void system_init(void)
{
	init_mcu();

	// GPIO on PB02

	gpio_set_pin_level(GLED_B,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(GLED_B, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(GLED_B, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB03

	gpio_set_pin_level(RLED_B,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(RLED_B, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(RLED_B, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB04

	gpio_set_pin_level(IR_CTRL,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(IR_CTRL, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(IR_CTRL, GPIO_PIN_FUNCTION_OFF);

	ADC_0_init();

	I2C_0_init();

	I2C_1_init();

	I2C_2_init();

	SPI_0_init();

	TARGET_IO_init();

	TIMER_0_init();
}
