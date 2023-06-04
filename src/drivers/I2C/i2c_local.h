#include "../../includes.h"

/*
 * Initialize the device driver for I2C_0
 */
#define I2C0_NODE DT_NODELABEL(mysensor)
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C0_NODE);

void i2c_test_write();