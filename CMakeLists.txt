# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(blinky_pwm)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/drivers/PWM/pwm_local.c)
target_sources(app PRIVATE src/drivers/SPIM/spim_local.c)
target_sources(app PRIVATE src/drivers/SPIS/spis_local.c)
target_sources(app PRIVATE src/drivers/I2C/i2c_local.c)
target_sources(app PRIVATE src/drivers/GPIO_INPUT/gpio_input_local.c)
target_sources(app PRIVATE src/drivers/BLE_NUS/ble_nus_local.c)

# Include UART ASYNC API adapter
target_sources_ifdef(CONFIG_BT_NUS_UART_ASYNC_ADAPTER app PRIVATE
  src/uart_async_adapter.c
)

# NORDIC SDK APP END

zephyr_library_include_directories(.)
