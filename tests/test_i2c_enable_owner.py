# SPDX-License-Identifier: MIT
"""Which instance owns gpio_en when i2c_enable is written, chip mode 0x03.

In dual mode the chip presents two HID interfaces. i2c_enable_store()
calls ft260_gpio_en_update(), which picks its target with
hid_get_drvdata(hdev), so the mask it changes belongs to the interface
that carries the attribute. ft260_gpio_set() enforces the mask of the
interface that owns the gpiochip. The attribute has to sit on that same
interface, or writing i2c_enable 0 cannot release GPIO0 and GPIO1
(FT260_GPIO_I2C_DEFAULT, 0x0003) for GPIO use.

While i2c_enable was in ft260_i2c_chip_mode_0_3_attr_group, every test
here failed in chip mode 0x03.

The tests run in file order: the baseline is taken before the write.
"""

import os

import pytest

import ftlib

I2C_LINES = [0, 1]		# FT260_GPIO_I2C_DEFAULT


@pytest.fixture(scope="module")
def released(chip):
	"""Record i2c_enable, let the tests change it, put it back."""
	if chip.chip_mode != "3":
		pytest.skip(f"chip mode {chip.chip_mode}, this test wants 3")
	node = ftlib.find_node("i2c_enable", chip.usb)
	orig = ftlib.read_attr(node, "i2c_enable")
	if orig != "1":
		pytest.skip(f"i2c_enable starts at {orig}, the run assumes 1")

	state = {"node": node, "mask": None}
	yield state

	ftlib.write_attr(ftlib.find_node("i2c_enable", chip.usb),
			 "i2c_enable", orig)
	ftlib.unexport_all(chip.base)


def test_attribute_sits_on_the_gpiochip_owner(chip, released):
	owner = ftlib.find_gpio_owner(chip.usb)
	assert os.path.basename(released["node"]) == os.path.basename(owner)


@pytest.mark.parametrize("offset", I2C_LINES)
def test_line_refused_while_i2c_owns_the_pins(chip, released, offset):
	assert ftlib.line_state(chip.base + offset) != "usable"


def test_write_zero_logs_a_new_mask(chip, released):
	marker = ftlib.mark("i2c-enable-0")
	ftlib.write_attr(released["node"], "i2c_enable", 0)
	owner = os.path.basename(ftlib.find_gpio_owner(chip.usb))
	released["mask"] = ftlib.mask_of(owner, marker)
	assert released["mask"] is not None, f"{owner} logged no new mask"


@pytest.mark.parametrize("offset", I2C_LINES)
def test_line_usable_after_the_release(chip, released, offset):
	assert released["mask"] is not None, "the i2c_enable write did not run"
	assert ftlib.line_state(chip.base + offset) == "usable"
