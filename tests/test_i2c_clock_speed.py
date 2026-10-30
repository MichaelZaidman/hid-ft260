# SPDX-License-Identifier: MIT
"""Writing clock changes the bus speed, not just the status report.

The chip takes the speed through request 0x22 and reports it back in
the I2C status report, so reading the attribute proves nothing on its
own: a chip that ignored the request could still echo the value. The
check is the time a fixed read takes.

A 2048 byte read at 100 kHz spends about 184 ms on the wire. At
3400 kHz that falls to 5 ms and what remains is the USB interrupt
endpoint, 60 bytes per 1 ms frame, so 35 ms for the same read. The
margin is wide, so the test asks only for a factor of two.

Both the attribute and the bus number must come from the same HID
device. With two boards attached, writing one board's clock and timing
the other board's bus shows no change at all.
"""

import time

import pytest

import ftlib

LENGTH = 2048
SLOW = 100			# kHz
FAST = 3400
WANT_RATIO = 2.0


def timed_read(bus, addr, length):
	start = time.monotonic()
	done = ftlib.i2c_read_at(bus, addr, 0, length)
	return time.monotonic() - start, done


@pytest.fixture
def clock(chip):
	orig = ftlib.read_attr(chip.i2c, "clock")
	yield lambda khz: ftlib.write_attr(chip.i2c, "clock", khz)
	ftlib.write_attr(chip.i2c, "clock", orig)


def test_faster_clock_shortens_a_read(chip, big_eeprom, clock):
	clock(SLOW)
	slow, done = timed_read(chip.bus, big_eeprom, LENGTH)
	assert done.returncode == 0, done.stderr.strip()

	clock(FAST)
	fast, done = timed_read(chip.bus, big_eeprom, LENGTH)
	assert done.returncode == 0, done.stderr.strip()

	assert slow / fast >= WANT_RATIO, (
		f"{LENGTH} bytes took {slow:.3f} s at {SLOW} kHz and "
		f"{fast:.3f} s at {FAST} kHz")
