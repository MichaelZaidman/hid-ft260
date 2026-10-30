# SPDX-License-Identifier: MIT
"""A chip strapped to chip mode 0x01, I2C only.

DCNF1 is 0 and DCNF0 is 1, so the FT260 creates one HID interface and
it is the I2C one, datasheet table 5.1. This is the only mode that
reaches two pieces of the driver: the FT260_MODE_I2C branch of
ft260_gpio_init(), which builds the gpiochip on the I2C interface, and
ft260_i2c_chip_mode_1_attr_group, the only group with all 12
attributes on one interface.

The probe mask is FT260_GPIO_UART_MODE_0_SET. The chip reports
uart_mode 0x00 in this mode, so every UART pin is a GPIO, while GPIO0
and GPIO1 stay out of the mask because this interface owns the I2C
bus. Writing i2c_enable 0 hands those two over as well, and here the
gpiochip that gains them belongs to the same interface that took the
write. In mode 0x03 it belongs to the other one.

Skipped unless a chip reports chip_mode 1.
"""

import errno
import os

import pytest

import ftlib

GPIO_TOTAL = 14
MODE_0_SET = 0x2FB0		# FT260_GPIO_UART_MODE_0_SET
I2C_DEFAULT = 0x0003		# FT260_GPIO_I2C_DEFAULT
I2C_LINES = (0, 1)

# ft260_i2c_chip_mode_1_attr_group, in full.
I2C_GROUP = {
	"chip_mode", "pwren_status", "suspend_status", "hid_over_i2c_en",
	"power_saving_en", "i2c_enable", "gpio2_func", "gpioa_func",
	"gpiog_func", "clock_ctl", "i2c_reset", "clock",
}

# Attributes of the UART group that must not appear here.
UART_ONLY = {"uart_mode", "uart_dcd_ri"}


def test_the_chip_has_one_hid_interface(i2c_only_chip):
	nodes = ftlib.find_nodes(i2c_only_chip.usb)
	assert len(nodes) == 1, nodes
	assert nodes[0] == i2c_only_chip.i2c


def test_the_mode_1_group_is_the_only_group(i2c_only_chip):
	node = i2c_only_chip.i2c
	have = {a for a in os.listdir(node) if a in I2C_GROUP | UART_ONLY}
	assert have == I2C_GROUP


def test_an_i2c_adapter_but_no_tty(i2c_only_chip):
	assert i2c_only_chip.bus is not None
	assert i2c_only_chip.uart is None
	assert i2c_only_chip.tty is None


def test_the_gpiochip_belongs_to_the_i2c_interface(i2c_only_chip):
	owner = ftlib.find_gpio_owner(i2c_only_chip.usb)
	assert owner == i2c_only_chip.i2c
	base = ftlib.find_gpio_base(owner)
	assert base is not None
	path = os.path.join(ftlib.GPIO_SYSFS, f"gpiochip{base}")
	assert ftlib.read_attr(path, "ngpio") == str(GPIO_TOTAL)


def test_the_mask_is_the_uart_mode_0_set(i2c_only_chip):
	"""No UART interface, so all its pins are GPIO. The I2C pins are
	not, because this interface drives the bus."""
	want = MODE_0_SET | ftlib.gpio2_bit(i2c_only_chip.i2c)
	mask = ftlib.gpio_en_now(i2c_only_chip.i2c)
	assert mask is not None, "no mask in the log for this interface"
	assert int(mask, 16) == want
	assert int(mask, 16) & I2C_DEFAULT == 0


def test_every_line_matches_the_mask(i2c_only_chip):
	mask = int(ftlib.gpio_en_now(i2c_only_chip.i2c), 16)
	base = i2c_only_chip.base
	for offset in range(GPIO_TOTAL):
		err = ftlib.line_errno(base + offset, "in")
		if mask & (1 << offset):
			assert err is None, f"offset {offset} refused with {err}"
		else:
			assert err == errno.EIO, f"offset {offset} gave {err}"


def test_i2c_enable_moves_the_mask_of_the_same_interface(i2c_only_chip):
	"""The write and the gpiochip are on one interface in this mode."""
	node = i2c_only_chip.i2c
	base = i2c_only_chip.base
	orig = ftlib.read_attr(node, "i2c_enable")
	assert orig == "1", f"i2c_enable starts at {orig}, nothing to prove"

	marker = ftlib.mark("i2c-enable-0")
	ftlib.write_attr(node, "i2c_enable", 0)
	try:
		mask = ftlib.mask_of(os.path.basename(node), marker)
		assert mask is not None, "the driver logged no mask"
		assert int(mask, 16) & I2C_DEFAULT == I2C_DEFAULT
		for offset in I2C_LINES:
			assert ftlib.line_errno(base + offset, "in") is None
	finally:
		marker = ftlib.mark("i2c-enable-1")
		ftlib.write_attr(node, "i2c_enable", orig)

	mask = ftlib.mask_of(os.path.basename(node), marker)
	assert int(mask, 16) & I2C_DEFAULT == 0
	for offset in I2C_LINES:
		assert ftlib.line_errno(base + offset, "in") == errno.EIO
