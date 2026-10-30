# SPDX-License-Identifier: MIT
"""A chip strapped to chip mode 0x02, UART only.

DCNF1 is 1 and DCNF0 is 0, so the FT260 creates one HID interface and
it is the UART one, datasheet table 5.1. The driver takes the
FT260_MODE_UART case of ft260_get_interface_type() and builds a tty, a
gpiochip and no I2C adapter. Nothing else on the bench reaches that
path: modes 0x00 and 0x03 create two interfaces, and mode 0x01 creates
one I2C interface.

The probe mask is the other thing only this mode shows. With no I2C
interface, ft260_gpio_init() adds FT260_GPIO_I2C_DEFAULT, so GPIO0 and
GPIO1 belong to the gpiochip from the start instead of carrying SCL
and SDA.

Skipped unless a chip reports chip_mode 2, so a bench with both boards
at 0x03 just passes it by.
"""

import errno
import os

import pytest

import ftlib

GPIO_TOTAL = 14
MODE_4_SET = 0x2CB0		# FT260_GPIO_UART_MODE_4_SET
I2C_DEFAULT = 0x0003		# FT260_GPIO_I2C_DEFAULT

# ft260_uart_chip_mode_0_2_3_attr_group, in full.
UART_GROUP = {
	"chip_mode", "pwren_status", "suspend_status", "power_saving_en",
	"i2c_enable", "gpio2_func", "gpioa_func", "gpiog_func",
	"uart_mode", "uart_dcd_ri", "clock_ctl",
}

# Attributes of the two I2C groups that must not appear here.
I2C_ONLY = {"hid_over_i2c_en", "i2c_reset", "clock"}


def test_the_chip_has_one_hid_interface(uart_only_chip):
	nodes = ftlib.find_nodes(uart_only_chip.usb)
	assert len(nodes) == 1, nodes
	assert nodes[0] == uart_only_chip.uart


def test_the_uart_group_is_the_only_group(uart_only_chip):
	node = uart_only_chip.uart
	have = {a for a in os.listdir(node) if a in UART_GROUP | I2C_ONLY}
	assert have == UART_GROUP


def test_a_tty_but_no_i2c_adapter(uart_only_chip):
	assert uart_only_chip.tty is not None
	assert uart_only_chip.bus is None


def test_the_gpiochip_belongs_to_this_interface(uart_only_chip):
	owner = ftlib.find_gpio_owner(uart_only_chip.usb)
	assert owner == uart_only_chip.uart
	base = ftlib.find_gpio_base(owner)
	assert base is not None
	path = os.path.join(ftlib.GPIO_SYSFS, f"gpiochip{base}")
	assert ftlib.read_attr(path, "ngpio") == str(GPIO_TOTAL)


def test_the_mask_carries_the_i2c_pins(uart_only_chip):
	"""No I2C interface, so GPIO0 and GPIO1 start as GPIO."""
	mode = int(ftlib.read_attr(uart_only_chip.uart, "uart_mode"))
	assert mode == 4, "this test reads the mask for flow control mode 4"

	want = MODE_4_SET | I2C_DEFAULT | ftlib.gpio2_bit(uart_only_chip.uart)
	mask = ftlib.gpio_en_now(uart_only_chip.uart)
	assert mask is not None, "no mask in the log for this interface"
	assert int(mask, 16) == want


def test_every_line_matches_the_mask(uart_only_chip):
	mask = int(ftlib.gpio_en_now(uart_only_chip.uart), 16)
	base = uart_only_chip.base
	for offset in range(GPIO_TOTAL):
		err = ftlib.line_errno(base + offset, "in")
		if mask & (1 << offset):
			assert err is None, f"offset {offset} refused with {err}"
		else:
			assert err == errno.EIO, f"offset {offset} gave {err}"
