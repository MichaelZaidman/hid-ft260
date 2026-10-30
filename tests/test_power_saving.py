# SPDX-License-Identifier: MIT
"""The driver wakes the chip before a transfer that follows an idle.

The FT260 falls back to a 30 kHz oscillator after 5 s without
traffic. ft260_xfer_status() handles that. When more than
FT260_WAKEUP_NEEDED_AFTER_MS, 4800 ms, passed since the last
transfer, it reads the I2C status report once before the real one
and logs:

    ft260_xfer_status: bus_status 0x20, wakeup

The other line of the same function carries a clock and a bus_busy
field, so a test can tell the two apart. One transfer logs one
wake-up at most. The first pass of the polling loop moves
need_wakeup_at 4800 ms forward.

The three tests sleep 22 s in total. They need debug=1, which
ftlib.insmod() always passes.
"""

import time

import ftlib

IDLE_S = 5.5			# past the 4800 ms window and the chip's 5 s
READ_LEN = 32


def _wakeups(log):
	return [line for line in log.splitlines() if line.endswith(", wakeup")]


def test_transfer_after_idle_wakes_the_chip(chip, eeprom):
	time.sleep(IDLE_S)
	marker = ftlib.mark("wakeup-idle")

	done = ftlib.i2c_read(chip.bus, eeprom, READ_LEN)

	assert done.returncode == 0, done.stderr.strip()
	log = ftlib.dmesg_since(marker)
	assert len(_wakeups(log)) == 1, log
	assert len(ftlib.hex_bytes(done.stdout)) == READ_LEN


def test_transfer_inside_the_window_does_not(chip, eeprom):
	time.sleep(IDLE_S)
	done = ftlib.i2c_read(chip.bus, eeprom, READ_LEN)
	assert done.returncode == 0, done.stderr.strip()
	marker = ftlib.mark("wakeup-busy")

	done = ftlib.i2c_read(chip.bus, eeprom, READ_LEN)

	assert done.returncode == 0, done.stderr.strip()
	log = ftlib.dmesg_since(marker)
	assert _wakeups(log) == [], log


def test_the_window_rearms(chip, eeprom):
	"""A second idle wakes the chip again. The window re-arms."""
	time.sleep(IDLE_S)
	done = ftlib.i2c_read(chip.bus, eeprom, READ_LEN)
	assert done.returncode == 0, done.stderr.strip()

	time.sleep(IDLE_S)
	marker = ftlib.mark("wakeup-rearm")

	done = ftlib.i2c_read(chip.bus, eeprom, READ_LEN)

	assert done.returncode == 0, done.stderr.strip()
	log = ftlib.dmesg_since(marker)
	assert len(_wakeups(log)) == 1, log
