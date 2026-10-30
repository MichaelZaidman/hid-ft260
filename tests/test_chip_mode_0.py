# SPDX-License-Identifier: MIT
"""A chip strapped to chip mode 0x00, FT260_MODE_ALL.

DCNF1 and DCNF0 are both 0. The datasheet table 5.1 gives the same two
interfaces as mode 0x03, and the driver says so twice: the switch in
ft260_get_interface_type() puts FT260_MODE_ALL and FT260_MODE_BOTH in
one case, and ft260_gpio_init() rewrites ALL to BOTH before it builds
the mask. This file is the check that both folds happen, because the
value the chip reports is 0 and everything downstream expects 3.

What it does not repeat: the rest of the suite already covers a mode
0x03 chip in depth. Point --chip at this board to run all of that
against mode 0x00 as well.

Skipped unless a chip reports chip_mode 0.
"""

import os

import pytest

import ftlib

GPIO_TOTAL = 14
I2C_DEFAULT = 0x0003		# FT260_GPIO_I2C_DEFAULT

# dev->gpio_uart_mode[], the same table the driver fills at probe.
UART_MODE_SET = {
	0: 0x2FB0,
	1: 0x2800,
	2: 0x0480,
	3: 0x2CB0,
	4: 0x2CB0,
}


def test_the_chip_has_two_interfaces(mode_all_chip):
	nodes = ftlib.find_nodes(mode_all_chip.usb)
	assert len(nodes) == 2, nodes
	assert set(nodes) == {mode_all_chip.uart, mode_all_chip.i2c}


def test_the_interfaces_split_the_same_way_as_mode_3(mode_all_chip):
	"""iface_id 1 is the UART one, anything else the I2C one."""
	assert ftlib.read_attr(mode_all_chip.uart, "chip_mode") == "0"
	assert mode_all_chip.tty is not None
	assert mode_all_chip.bus is not None


def test_the_gpiochip_belongs_to_the_uart_interface(mode_all_chip):
	owner = ftlib.find_gpio_owner(mode_all_chip.usb)
	assert owner == mode_all_chip.uart
	base = ftlib.find_gpio_base(owner)
	assert base is not None
	path = os.path.join(ftlib.GPIO_SYSFS, f"gpiochip{base}")
	assert ftlib.read_attr(path, "ngpio") == str(GPIO_TOTAL)


def test_the_mask_is_built_as_mode_3(mode_all_chip):
	"""ALL is rewritten to BOTH, so the I2C pins stay out of the mask
	and the UART pins follow the flow control mode the chip holds."""
	mode = int(ftlib.read_attr(mode_all_chip.uart, "uart_mode"))
	want = UART_MODE_SET[mode] | ftlib.gpio2_bit(mode_all_chip.uart)

	mask = ftlib.gpio_en_now(mode_all_chip.uart)
	assert mask is not None, "no mask in the log for this interface"
	assert int(mask, 16) == want
	assert int(mask, 16) & I2C_DEFAULT == 0
