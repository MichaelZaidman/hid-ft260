# SPDX-License-Identifier: MIT
"""The i2c_reset attribute restarts the controller and leaves it usable.

Writing i2c_reset makes ft260_i2c_reset() send FT260_SET_I2C_RESET,
AN_394 section 4.4.15. The attribute is write only, so the value
written does not matter and reading it must fail.

Not reachable from here: ft260_i2c_abort(), which sends a STOP before
the reset, runs on a read timeout, on a status check failure, and on
an SMBus block read with a bad count. A working EEPROM produces none
of these; the commit that added it shortened the completion timeout in
the source to get one. ft260_schedule_reset() runs from the TN_189
endpoint STALL path at probe and needs a chip that shows the errata.
"""

import errno
import os

import pytest

import ftlib

DONE = "ft260_i2c_reset: done"
READ_LEN = 16


def test_reset_logs_done(chip):
	marker = ftlib.mark("i2c-reset")
	ftlib.write_attr(chip.i2c, "i2c_reset", 1)
	log = ftlib.dmesg_since(marker)
	assert log.count(DONE) == 1, log


@pytest.mark.parametrize("count", [1, 5], ids=["once", "five_times"])
def test_bus_answers_after_resets(chip, eeprom, count):
	for _ in range(count):
		ftlib.write_attr(chip.i2c, "i2c_reset", 1)
	done = ftlib.i2c_read(chip.bus, eeprom, READ_LEN)
	assert done.returncode == 0, done.stderr.strip()


def test_reset_is_write_only(chip):
	with pytest.raises(OSError) as err:
		ftlib.read_attr(chip.i2c, "i2c_reset")
	assert err.value.errno == errno.EACCES
