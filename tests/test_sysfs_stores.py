# SPDX-License-Identifier: MIT
"""Writing the sysfs attributes, not just finding them.

Three things are checked. A pin function select changes gpio_en and
with it what the gpiochip accepts: writing 0 hands the pin to GPIO,
writing the old value back takes it away, and then
ft260_gpio_direction_set() refuses the line with -EIO.

A 16-bit store takes a four digit value. FT260_ATTR_STORE used to
return the feature report length instead of the write size, so the VFS
retried the remainder and userspace saw "Invalid argument" although the
value had been applied.

clock_ctl takes the three documented system clock settings, 12, 24
and 48 MHz. Nothing reads the clock back from the wire, so the test
asks the two interfaces to keep working at each setting: the I2C bus
answers and the UART carries a byte to the peer.

A value that does not parse is rejected: kstrtou8 and kstrtou16 fail
and the store returns -EINVAL without touching the chip.

A clock outside 60-3400 kHz is rejected too. The chip would take such
a value, report it back in its I2C status report and still run the bus
at the 100 kHz default, so the driver refuses it in
ft260_i2c_clock_parse() instead of passing it on.
"""

import errno
import os

import pytest

import ftlib

GPIOA = 6			# FT260_GPIO_A, BIT(6)
GPIOG = 12			# FT260_GPIO_G, BIT(12)
FUNC_GPIO = 0			# FT260_MFPIN_GPIO
FAST_CLOCK = 1000		# kHz, four digits on purpose
CLOCK_MIN = 60			# FT260_I2C_CLOCK_MIN
CLOCK_MAX = 3400		# FT260_I2C_CLOCK_MAX


@pytest.fixture(scope="module")
def saved(chip):
	"""Put the three attributes back whatever the tests do."""
	uart, i2c = chip.uart, chip.i2c
	orig = {
		"gpioa_func": ftlib.read_attr(uart, "gpioa_func"),
		"gpiog_func": ftlib.read_attr(uart, "gpiog_func"),
		"clock": ftlib.read_attr(i2c, "clock"),
		"clock_ctl": ftlib.read_attr(uart, "clock_ctl"),
	}
	if orig["gpioa_func"] == "0" or orig["gpiog_func"] == "0":
		pytest.skip(f"a pin is already GPIO: {orig}")

	yield orig

	ftlib.write_attr(chip.uart, "gpioa_func", orig["gpioa_func"])
	ftlib.write_attr(chip.uart, "gpiog_func", orig["gpiog_func"])
	ftlib.write_attr(chip.i2c, "clock", orig["clock"])
	ftlib.write_attr(chip.uart, "clock_ctl", orig["clock_ctl"])


@pytest.mark.parametrize("attr,offset,bit",
			 [("gpioa_func", GPIOA, 1 << GPIOA),
			  ("gpiog_func", GPIOG, 1 << GPIOG)],
			 ids=["gpioa", "gpiog"])
def test_func_select_round_trip(chip, saved, attr, offset, bit):
	marker = ftlib.mark(f"{attr}-0")
	ftlib.write_attr(chip.uart, attr, FUNC_GPIO)
	mask = ftlib.mask_of(os.path.basename(chip.uart), marker)
	assert mask is not None, "the driver logged no mask"
	assert int(mask, 16) & bit == bit
	assert ftlib.line_errno(chip.base + offset) is None

	marker = ftlib.mark(f"{attr}-back")
	ftlib.write_attr(chip.uart, attr, saved[attr])
	mask = ftlib.mask_of(os.path.basename(chip.uart), marker)
	assert mask is not None, "the driver logged no mask"
	assert int(mask, 16) & bit == 0
	assert ftlib.line_errno(chip.base + offset) == errno.EIO


def test_clock_takes_four_digits(chip, saved):
	ftlib.write_attr(chip.i2c, "clock", FAST_CLOCK)
	assert ftlib.read_attr(chip.i2c, "clock") == str(FAST_CLOCK)


def test_clock_restored_and_the_bus_still_answers(chip, saved, eeprom):
	ftlib.write_attr(chip.i2c, "clock", saved["clock"])
	assert ftlib.read_attr(chip.i2c, "clock") == saved["clock"]
	assert ftlib.i2c_read(chip.bus, eeprom, 16).returncode == 0


@pytest.mark.parametrize("value", [CLOCK_MIN, CLOCK_MAX],
			 ids=["min", "max"])
def test_clock_takes_the_range_ends(chip, saved, value):
	ftlib.write_attr(chip.i2c, "clock", value)
	assert ftlib.read_attr(chip.i2c, "clock") == str(value)
	# Leave no fast bus behind: the 24c02 at 0x50 does not answer at
	# 3400 kHz once the system clock is 48 MHz.
	ftlib.write_attr(chip.i2c, "clock", saved["clock"])


@pytest.mark.parametrize("value", [CLOCK_MIN - 1, CLOCK_MAX + 1, 65535],
			 ids=["below_min", "above_max", "u16_max"])
def test_clock_outside_the_range_refused(chip, saved, value):
	with pytest.raises(OSError) as err:
		ftlib.write_attr(chip.i2c, "clock", value)
	assert err.value.errno == errno.EINVAL
	assert ftlib.read_attr(chip.i2c, "clock") != str(value)


@pytest.mark.parametrize("node_attr,value",
			 [("uart/gpioa_func", "abc"),
			  ("uart/gpioa_func", "300"),
			  ("i2c/clock", "abc"),
			  ("i2c/clock", "70000")],
			 ids=["gpioa_text", "gpioa_over_u8",
			      "clock_text", "clock_over_u16"])
def test_unparsable_value_refused(chip, saved, node_attr, value):
	which, attr = node_attr.split("/")
	node = chip.uart if which == "uart" else chip.i2c
	with pytest.raises(OSError) as err:
		ftlib.write_attr(node, attr, value)
	assert err.value.errno == errno.EINVAL


@pytest.mark.parametrize("value", [0, 1, 2],
			 ids=["12MHz", "24MHz", "48MHz"])
def test_clock_ctl_round_trip(chip, saved, eeprom, value):
	ftlib.write_attr(chip.uart, "clock_ctl", value)
	assert ftlib.read_attr(chip.uart, "clock_ctl") == str(value)
	assert ftlib.i2c_read(chip.bus, eeprom, 16).returncode == 0


@pytest.mark.parametrize("value", [0, 1, 2],
			 ids=["12MHz", "24MHz", "48MHz"])
def test_uart_carries_a_byte_at_each_system_clock(chip, saved, peer_link,
						  value):
	"""The baud generator runs off the system clock, so 9600 has to
	survive all three settings."""
	import serial

	ftlib.write_attr(chip.uart, "clock_ctl", value)
	msg = b"ft260-clock-ctl\n"
	with serial.Serial(peer_link, 9600, timeout=2) as tx, \
	     serial.Serial(chip.tty, 9600, timeout=2) as rx:
		tx.write(msg)
		tx.flush()
		assert rx.read(len(msg)) == msg
