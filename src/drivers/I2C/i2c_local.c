#include "i2c_local.h"

void i2c_test_write()
{
	int ret;
	uint8_t config[2] = {0x03, 0x8C};
	ret = i2c_write_dt(&dev_i2c, config, sizeof(config));
	if (ret != 0)
	{
		printk("Failed to write to I2C device address %x at reg. %x n", dev_i2c.addr, config[0]);
	}
}
