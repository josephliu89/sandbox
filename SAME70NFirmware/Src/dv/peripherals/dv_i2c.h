/*
 * dv_i2c.h
 *
 * Created: 11/13/2019 4:30:40 PM
 *  Author: liu
 */ 

#ifndef DV_I2C_H_
#define DV_I2C_H_

/* TWI/I2C definitions */
#define TWI_CLK						(100000U)	// TWI bus clock 100 kHz
#define RTC_MCP						(0x6F)		// TODO: Remove, used to test interface; I2C slave address for MCP79410

void init_i2c(void);
void test_i2c(void);

#endif /* DV_I2C_H_ */