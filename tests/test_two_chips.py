# SPDX-License-Identifier: MIT
"""Two FT260s on one host must stay independent.

Each chip registers its own gpiochip, I2C adapter and tty. The driver
keys all of them off the HID device, so a mistake shows up as one chip
answering for the other: a shared gpiochip label, overlapping line
numbers, a reused tty index, or an i2c_enable write that moves the
wrong gpio_en mask.

The peer is wired to one chip only. The chip fixture finds which one,
so the transfer test can check that the traffic does not reach the
other.
"""

import os

import pytest
import serial

import ftlib

BAUD = 9600
I2C_LINES = (0, 1)		# FT260_GPIO_I2C_DEFAULT


@pytest.fixture(scope="module")
def two(chips):
	"""Two chips with both interfaces, so chip mode 0x00 or 0x03.

	A chip in mode 0x01 or 0x02 has one interface and no second
	gpio_en to move, so it cannot show the cross-talk this file
	looks for.
	"""
	both = [c for c in chips if c.uart and c.i2c]
	if len(both) < 2:
		pytest.skip(f"{len(both)} chip(s) with two interfaces, "
			    "this file wants 2")
	return both[0], both[1]


def test_both_chips_probed(two):
	for chip in two:
		assert chip.uart, f"{chip.usb} has no uart interface"
		assert chip.i2c, f"{chip.usb} has no i2c interface"
		assert chip.base is not None, f"{chip.usb} has no gpiochip"
		assert chip.bus is not None, f"{chip.usb} has no i2c bus"
		assert chip.tty, f"{chip.usb} has no tty"


def test_gpiochips_do_not_overlap(two):
	first, second = sorted(chip.base for chip in two)
	assert second - first >= ftlib.GPIO_TOTAL


def test_gpiochip_labels_differ(two):
	labels = ["ft260_" + os.path.basename(ftlib.find_gpio_owner(chip.usb))
		  for chip in two]
	assert labels[0] != labels[1]
	for chip, label in zip(two, labels):
		path = f"{ftlib.GPIO_SYSFS}/gpiochip{chip.base}"
		assert ftlib.read_attr(path, "label") == label
		assert ftlib.read_attr(path, "ngpio") == str(ftlib.GPIO_TOTAL)


def test_i2c_buses_and_ttys_differ(two):
	assert two[0].bus != two[1].bus
	assert two[0].tty != two[1].tty


def test_peer_traffic_reaches_only_the_wired_chip(two, wired, peer):
	if wired is None:
		pytest.skip("no chip answered on the peer")
	other = [c for c in two if c.usb != wired.usb][0]
	msg = f"ft260-two-{os.getpid()}\n".encode()
	with serial.Serial(other.tty, BAUD, timeout=2) as rx_other, \
	     serial.Serial(wired.tty, BAUD, timeout=5) as rx_wired, \
	     serial.Serial(peer, BAUD, timeout=5) as line:
		rx_wired.reset_input_buffer()
		rx_other.reset_input_buffer()
		line.write(msg)
		line.flush()
		got = rx_wired.read_until(msg)
		stray = rx_other.read(len(msg))
	assert msg in got, f"wired chip got {got!r}"
	assert stray == b"", f"second chip got {stray!r}"


def test_i2c_enable_is_per_chip(chip, two):
	wired = chip
	other = [c for c in two if c.usb != wired.usb][0]
	owner = ftlib.find_gpio_owner(wired.usb)
	if ftlib.read_attr(owner, "i2c_enable") != "1":
		pytest.skip("the first chip does not hold the i2c pins")
	before = [ftlib.line_state(other.base + o) for o in I2C_LINES]
	assert "usable" not in before, (
		f"second chip starts with lines {before}, nothing to prove")

	ftlib.write_attr(owner, "i2c_enable", 0)
	try:
		mine = [ftlib.line_state(wired.base + o) for o in I2C_LINES]
		theirs = [ftlib.line_state(other.base + o) for o in I2C_LINES]
	finally:
		ftlib.write_attr(owner, "i2c_enable", 1)
	assert mine == ["usable"] * len(I2C_LINES)
	assert theirs == before


def test_rebind_reuses_the_tty_index(two):
	other = two[1]
	name = other.tty
	ftlib.rebind(other.uart)
	assert other.tty == name
