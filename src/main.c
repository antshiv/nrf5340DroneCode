/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Sample app to demonstrate PWM.
 */
#include "includes.h"
#include "drivers/PWM/pwm_local.h"
#include "drivers/SPIM/spim_local.h"
#include "drivers/SPIS/spis_local.h"
#include "drivers/I2C/i2c_local.h"
#include "drivers/GPIO_INPUT/gpio_input_local.h"
#include "drivers/BLE_NUS/ble_nus_local.h"

#define SLEEP_TIME_MS 1


/*
 * Initialize the device driver for SPI_4
 */
/* 1000 msec = 1 sec */
// #define SLEEP_TIME_MS 1000
static const struct gpio_dt_spec spi4_cs = GPIO_DT_SPEC_GET(DT_ALIAS(spi4_cs), gpios);
// static const struct spi_cs_control spi4 = SPI_CS_CONTROL_PTR_DT(SPI_NODE, cs_gpios);
/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
// static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/*************OLD SPI *************************************/
/* static const struct spi_config spi_cfg = {
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
			 SPI_MODE_CPOL | SPI_MODE_CPHA,
	.frequency = 4000000,
	.slave = 0,
};

const struct device * spi_dev;

static void spi_init(void)
{
	const char* const spiName = "SPI_4";
	int err, ret;

	printk("SPI4 example application\n");

	spi_dev = DEVICE_DT_GET(DT_NODELABEL(arduino_spi));
	if (!device_is_ready(spi4_cs.port)) {
		printk("spi4 cs is not ready\n");
		return;
	}

	ret = gpio_pin_configure_dt(&spi4_cs, GPIO_OUTPUT_INIT_LOW);
	if (ret < 0) {
		printk("spi4 cs config failed\n");
		return;
	}

	if (spi_dev == NULL) {
		printk("Could not get %s device\n", spiName);
		return;
	}
}

void spi_test_send(void)
{
	int err;
	static uint8_t tx_buffer[1];
	static uint8_t rx_buffer[1];

	const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = sizeof(tx_buffer)
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};

	struct spi_buf rx_buf = {
		.buf = rx_buffer,
		.len = sizeof(rx_buffer),
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1
	};

	err = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);
	if (err) {
		printk("SPI error: %d\n", err);
	} else {
		 Connect MISO to MOSI for loopback
		printk("TX sent: %x\n", tx_buffer[0]);
		printk("RX recv: %x\n", rx_buffer[0]);
		tx_buffer[0]++;
	}
} */
/*******************END OLD SPI ******************************/


/***************Peri BLE   UART   ****************/

/***************************************************/
/****************** Peri BLE UART ***********************/
/****************************************************/

void main(void)
{
	uint32_t max_period;
	uint32_t period;
	uint8_t dir = 0U;
	int ret;
	int blink_status = 0;
	int err = 0;

	err = uart_init();
	if (err)
	{
		error();
	}

	if (!gpio_is_ready_dt(&button))
	{
		printk("Error: button device %s is not ready\n",
			   button.port->name);
		return 0;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0)
	{
		printk("Error %d: failed to configure %s pin %d\n",
			   ret, button.port->name, button.pin);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&button,
										  GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0)
	{
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			   ret, button.port->name, button.pin);
		return 0;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);

	if (led.port && !device_is_ready(led.port))
	{
		printk("Error %d: LED device %s is not ready; ignoring it\n",
			   ret, led.port->name);
		led.port = NULL;
	}
	if (led.port)
	{
		ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT);
		if (ret != 0)
		{
			printk("Error %d: failed to configure LED device %s pin %d\n",
				   ret, led.port->name, led.pin);
			led.port = NULL;
		}
		else
		{
			printk("Set up LED at %s pin %d\n", led.port->name, led.pin);
		}
	}

	printk("Press the button\n");

	ble_nus_init();

	printk("PWM-based blinky\n");

	printk("SPIM Example\n");
	spi_init();
	spi_slave_init();
	printk("SPI master/slave example started\n");
	spi_slave_write_test_msg();
	pwm_check_ready();

	if (!device_is_ready(dev_i2c.bus))
	{
		printk("I2C bus %s is not ready!\n\r", dev_i2c.bus->name);
		return;
	}

	calibratePWM(&period);

	while (1)
	{
		setDuty(pwm_led0, period, period / 2U);
		setDuty(pwm_led1, period, period / 2U);
		setDuty(pwm_led2, period, period / 10U);
		setDuty(pwm_led3, period, period / 10U);

		// gpio_pin_set_dt(&spi4_cs, 0);
		ret = gpio_pin_toggle_dt(&spi4_cs);
		/*  SPI  */
		spi_write_test_msg();
		// ret = gpio_pin_toggle_dt(&led);
		if (ret < 0)
		{
			return;
		}
		k_msleep(SLEEP_TIME_MS);

		spi_slave_process_message();

		/* if sensor input */

		/*  END */
		// spi_test_send();
		// i2c_test_write();
		ret = gpio_pin_toggle_dt(&spi4_cs);

		// gpio_pin_set_dt(&spi4_cs, 1);
		k_sleep(K_SECONDS(4U));
	}
}
