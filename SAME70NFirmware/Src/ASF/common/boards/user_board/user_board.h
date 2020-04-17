/**
 * \file
 *
 * \brief User board definition template
 *
 */

 /* This file is intended to contain definitions and configuration details for
 * features and devices that are available on the board, e.g., frequency and
 * startup time for an external crystal, external memory devices, LED and USART
 * pins.
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef USER_BOARD_H
#define USER_BOARD_H

#include <conf_board.h>

/* Configure clocks */
#define BOARD_FREQ_SLCK_XTAL      (32768U)
#define BOARD_FREQ_SLCK_BYPASS    (32768U)
#define BOARD_FREQ_MAINCK_XTAL    (12000000U)
#define BOARD_FREQ_MAINCK_BYPASS  (12000000U)
#define BOARD_MCK                 CHIP_FREQ_CPU_MAX
#define BOARD_OSC_STARTUP_US      15625

/* ATPL360 defines */
#define ATPL360_GPIO0        (PIO_PD28_IDX)
#define ATPL360_GPIO1        (PIO_PA24_IDX)
#define ATPL360_GPIO2        (PIO_PA15_IDX)
#define ATPL360_GPIO3        (PIO_PD18_IDX)
#define ATPL360_GPIO4        (PIO_PA11_IDX)
#define ATPL360_GPIO5        (PIO_PA26_IDX)

#define ATPL360_RESET_GPIO               PIO_PD17_IDX
#define ATPL360_RESET_ACTIVE_LEVEL       IOPORT_PIN_LEVEL_LOW
#define ATPL360_RESET_INACTIVE_LEVEL     IOPORT_PIN_LEVEL_HIGH

#define ATPL360_LDO_EN_GPIO              PIO_PA12_IDX
#define ATPL360_LDO_EN_ACTIVE_LEVEL      IOPORT_PIN_LEVEL_HIGH
#define ATPL360_LDO_EN_INACTIVE_LEVEL    IOPORT_PIN_LEVEL_LOW

#define ATPL360_CD_EN_GPIO   ATPL360_GPIO0

#define ATPL360_INT_GPIO     ATPL360_GPIO3
#define ATPL360_INT_FLAGS    IOPORT_MODE_DEBOUNCE
#define ATPL360_INT_SENSE    IOPORT_SENSE_FALLING
#define ATPL360_INT          {PIO_PD18, PIOD, ID_PIOD, PIO_INPUT, PIO_DEGLITCH | PIO_IT_LOW_LEVEL}
#define ATPL360_INT_MASK     PIO_PD18
#define ATPL360_INT_PIO      PIOD
#define ATPL360_INT_ID       ID_PIOD
#define ATPL360_INT_TYPE     PIO_INPUT
#define ATPL360_INT_ATTR     (PIO_DEGLITCH | PIO_IT_LOW_LEVEL)
#define ATPL360_INT_IRQn     PIOD_IRQn

#define ATPL360_SPI          SPI0
#define ATPL360_SPI_CS       3

/* PL360 SPI Peripheral */
#define SPI0_MISO_GPIO    PIO_PD20_IDX
#define SPI0_MISO_FLAGS   (IOPORT_MODE_MUX_B)
#define SPI0_MOSI_GPIO    PIO_PD21_IDX
#define SPI0_MOSI_FLAGS   (IOPORT_MODE_MUX_B)
//#define SPI0_NPCS0_GPIO   PIO_PB2_IDX
//#define SPI0_NPCS0_FLAGS  (IOPORT_MODE_MUX_D)
//#define SPI0_NPCS1_GPIO   PIO_PA31_IDX
//#define SPI0_NPCS1_FLAGS  (IOPORT_MODE_MUX_A)
//#define SPI0_NPCS2_GPIO   PIO_PD12_IDX
//#define SPI0_NPCS2_FLAGS  (IOPORT_MODE_MUX_C)
#define SPI0_NPCS3_GPIO   PIO_PD27_IDX
#define SPI0_NPCS3_FLAGS  (IOPORT_MODE_MUX_B)
#define SPI0_SPCK_GPIO    PIO_PD22_IDX
#define SPI0_SPCK_FLAGS   (IOPORT_MODE_MUX_B)

/* PoEM SPI Peripheral */
#define PIN_USART0_TXD_IDX          PIO_PB1_IDX
#define PIN_USART0_TXD_FLAGS        IOPORT_MODE_MUX_C
#define PIN_USART0_RXD_IDX          PIO_PB0_IDX
#define PIN_USART0_RXD_FLAGS        IOPORT_MODE_MUX_C
#define PIN_USART0_SCK_IDX          PIO_PB13_IDX
#define PIN_USART0_SCK_FLAGS        IOPORT_MODE_MUX_C
#define PIN_USART0_CTS_IDX          PIO_PB2_IDX
#define PIN_USART0_CTS_FLAGS        IOPORT_MODE_MUX_C

/* I2C Peripheral */
#define TWIHS0_DATA_GPIO   PIO_PA3_IDX
#define TWIHS0_DATA_FLAGS  (IOPORT_MODE_MUX_A)
#define TWIHS0_CLK_GPIO    PIO_PA4_IDX
#define TWIHS0_CLK_FLAGS   (IOPORT_MODE_MUX_A)

/* UART0 Peripheral */
#define UART0_RXD_GPIO				PIO_PA9_IDX
#define UART0_RXD_FLAGS				IOPORT_MODE_MUX_A
#define UART0_TXD_GPIO				PIO_PA10_IDX
#define UART0_TXD_FLAGS				IOPORT_MODE_MUX_A

/* LEDs */
#define NRLED_PIN					PIO_PA8_IDX
#define NGLED_PIN					PIO_PD30_IDX

/* Mode detection */
#define CTRL_NDEV					PIO_PA17_IDX

/* Incoming voltage detection */
#define ANA_VBAT_PIN                PIO_PA21_IDX        

/* LT8619 sync */
#define SYNC_LT8619                 PIO_PA28_IDX        

/* Start signal for PoEM */
#define NSTART                      PIO_PD15_IDX

/* Shutdown signal from PoEM */
#define NSHDN                       PIO_PA10_IDX

#endif // USER_BOARD_H
