/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef ATMEL_START_PINS_H_INCLUDED
#define ATMEL_START_PINS_H_INCLUDED

#include <hal_gpio.h>

// SAMD21 has 8 pin functions

#define GPIO_PIN_FUNCTION_A 0
#define GPIO_PIN_FUNCTION_B 1
#define GPIO_PIN_FUNCTION_C 2
#define GPIO_PIN_FUNCTION_D 3
#define GPIO_PIN_FUNCTION_E 4
#define GPIO_PIN_FUNCTION_F 5
#define GPIO_PIN_FUNCTION_G 6
#define GPIO_PIN_FUNCTION_H 7

#define IR_READING GPIO(GPIO_PORTA, 2)
#define AUX GPIO(GPIO_PORTA, 3)
#define PA06 GPIO(GPIO_PORTA, 6)
#define SDA1 GPIO(GPIO_PORTA, 8)
#define SCL1 GPIO(GPIO_PORTA, 9)
#define SDA0 GPIO(GPIO_PORTA, 16)
#define SCL0 GPIO(GPIO_PORTA, 17)
#define SDA2 GPIO(GPIO_PORTA, 22)
#define SCL2 GPIO(GPIO_PORTA, 23)
#define PB00 GPIO(GPIO_PORTB, 0)
#define PB01 GPIO(GPIO_PORTB, 1)
#define GLED_B GPIO(GPIO_PORTB, 2)
#define RLED_B GPIO(GPIO_PORTB, 3)
#define IR_CTRL GPIO(GPIO_PORTB, 4)
#define MISO GPIO(GPIO_PORTB, 8)
#define CS_B GPIO(GPIO_PORTB, 9)
#define MOSI GPIO(GPIO_PORTB, 10)
#define SCK GPIO(GPIO_PORTB, 11)

#endif // ATMEL_START_PINS_H_INCLUDED
