# SPDX-License-Identifier: MIT
"""Each sysfs attribute sits on the HID interface that owns the function.

i2c_enable belongs to the interface that owns the gpiochip, not to the
one that carries the bus: its store calls ft260_gpio_en_update(), which
acts on hid_get_drvdata(hdev), and ft260_gpio_set() enforces that same
mask. In chip modes 0x00, 0x02 and 0x03 the gpiochip is on the UART
interface.
"""

import os

import pytest

UART_HAS = ["chip_mode", "gpio2_func", "gpioa_func", "gpiog_func",
	    "uart_mode", "uart_dcd_ri", "clock_ctl", "i2c_enable"]
I2C_HAS = ["hid_over_i2c_en", "i2c_reset", "clock"]
I2C_LACKS = ["gpio2_func", "gpioa_func", "gpiog_func", "uart_mode",
	     "uart_dcd_ri", "chip_mode", "i2c_enable"]
UART_LACKS = ["i2c_reset", "clock", "hid_over_i2c_en"]


@pytest.mark.parametrize("attr", UART_HAS)
def test_uart_has(chip, attr):
	assert os.path.exists(os.path.join(chip.uart, attr))


@pytest.mark.parametrize("attr", I2C_HAS)
def test_i2c_has(chip, attr):
	assert os.path.exists(os.path.join(chip.i2c, attr))


@pytest.mark.parametrize("attr", I2C_LACKS)
def test_i2c_lacks(chip, attr):
	assert not os.path.exists(os.path.join(chip.i2c, attr))


@pytest.mark.parametrize("attr", UART_LACKS)
def test_uart_lacks(chip, attr):
	assert not os.path.exists(os.path.join(chip.uart, attr))
