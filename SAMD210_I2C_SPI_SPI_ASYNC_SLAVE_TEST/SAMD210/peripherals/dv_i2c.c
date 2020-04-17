#include <atmel_start.h>
#include "dv_i2c.h"

void initI2c(void) {
	struct io_descriptor *I2C_0_io;
	struct io_descriptor *I2C_1_io;
	struct io_descriptor *I2C_2_io;

	i2c_m_sync_get_io_descriptor(&I2C_0, &I2C_0_io);
	i2c_m_sync_get_io_descriptor(&I2C_1, &I2C_1_io);
	i2c_m_sync_get_io_descriptor(&I2C_2, &I2C_2_io);
	i2c_m_sync_enable(&I2C_0);
	i2c_m_sync_enable(&I2C_1);
	i2c_m_sync_enable(&I2C_2);
}
