# SPDX-License-Identifier: MIT
"""The functionality mask and the adapter quirks.

ft260_functionality() returns six I2C_FUNC_ names. ft260_i2c_quirks
sets max_comb_1st_msg_len to 2. No other test reads either value.
The other I2C tests send transfers that the mask and the quirk
allow, so they pass without reading them.

The mask comes from the I2C_FUNCS ioctl on /dev/i2c-N. That ioctl
returns i2c_get_functionality(), which is the driver value with
nothing added. The flag values below are from
include/uapi/linux/i2c.h.
"""

import fcntl
import os
import re
import struct
import subprocess

import pytest

import ftlib

I2C_FUNCS = 0x0705		# takes a pointer to an unsigned long

FUNC_I2C = 0x00000001
FUNC_10BIT_ADDR = 0x00000002
FUNC_PROTOCOL_MANGLING = 0x00000004
FUNC_SMBUS_PEC = 0x00000008
FUNC_NOSTART = 0x00000010
FUNC_SLAVE = 0x00000020
FUNC_SMBUS_BLOCK_PROC_CALL = 0x00008000
FUNC_SMBUS_QUICK = 0x00010000
FUNC_SMBUS_READ_BYTE = 0x00020000
FUNC_SMBUS_WRITE_BYTE = 0x00040000
FUNC_SMBUS_READ_BYTE_DATA = 0x00080000
FUNC_SMBUS_WRITE_BYTE_DATA = 0x00100000
FUNC_SMBUS_READ_WORD_DATA = 0x00200000
FUNC_SMBUS_WRITE_WORD_DATA = 0x00400000
FUNC_SMBUS_PROC_CALL = 0x00800000
FUNC_SMBUS_READ_BLOCK_DATA = 0x01000000
FUNC_SMBUS_WRITE_BLOCK_DATA = 0x02000000
FUNC_SMBUS_READ_I2C_BLOCK = 0x04000000
FUNC_SMBUS_WRITE_I2C_BLOCK = 0x08000000
FUNC_SMBUS_HOST_NOTIFY = 0x10000000

FUNC_SMBUS_BYTE = FUNC_SMBUS_READ_BYTE | FUNC_SMBUS_WRITE_BYTE
FUNC_SMBUS_BYTE_DATA = FUNC_SMBUS_READ_BYTE_DATA | FUNC_SMBUS_WRITE_BYTE_DATA
FUNC_SMBUS_WORD_DATA = FUNC_SMBUS_READ_WORD_DATA | FUNC_SMBUS_WRITE_WORD_DATA
FUNC_SMBUS_BLOCK_DATA = (FUNC_SMBUS_READ_BLOCK_DATA |
			 FUNC_SMBUS_WRITE_BLOCK_DATA)
FUNC_SMBUS_I2C_BLOCK = FUNC_SMBUS_READ_I2C_BLOCK | FUNC_SMBUS_WRITE_I2C_BLOCK

# The six names of ft260_functionality(), in that order.
WANT = (FUNC_I2C | FUNC_SMBUS_BYTE | FUNC_SMBUS_BYTE_DATA |
	FUNC_SMBUS_WORD_DATA | FUNC_SMBUS_BLOCK_DATA | FUNC_SMBUS_I2C_BLOCK)

# Flags the driver must not claim.
NOT_CLAIMED = {
	"10BIT_ADDR":		FUNC_10BIT_ADDR,
	"PROTOCOL_MANGLING":	FUNC_PROTOCOL_MANGLING,
	"SMBUS_PEC":		FUNC_SMBUS_PEC,
	"NOSTART":		FUNC_NOSTART,
	"SLAVE":		FUNC_SLAVE,
	"SMBUS_BLOCK_PROC_CALL": FUNC_SMBUS_BLOCK_PROC_CALL,
	"SMBUS_QUICK":		FUNC_SMBUS_QUICK,
	"SMBUS_PROC_CALL":	FUNC_SMBUS_PROC_CALL,
	"SMBUS_HOST_NOTIFY":	FUNC_SMBUS_HOST_NOTIFY,
}

# What i2cdetect -F prints, line by line, and the flag behind each.
DETECT_LINES = {
	"I2C":				FUNC_I2C,
	"SMBus Quick Command":		FUNC_SMBUS_QUICK,
	"SMBus Send Byte":		FUNC_SMBUS_WRITE_BYTE,
	"SMBus Receive Byte":		FUNC_SMBUS_READ_BYTE,
	"SMBus Write Byte":		FUNC_SMBUS_WRITE_BYTE_DATA,
	"SMBus Read Byte":		FUNC_SMBUS_READ_BYTE_DATA,
	"SMBus Write Word":		FUNC_SMBUS_WRITE_WORD_DATA,
	"SMBus Read Word":		FUNC_SMBUS_READ_WORD_DATA,
	"SMBus Process Call":		FUNC_SMBUS_PROC_CALL,
	"SMBus Block Write":		FUNC_SMBUS_WRITE_BLOCK_DATA,
	"SMBus Block Read":		FUNC_SMBUS_READ_BLOCK_DATA,
	"SMBus Block Process Call":	FUNC_SMBUS_BLOCK_PROC_CALL,
	"SMBus PEC":			FUNC_SMBUS_PEC,
	"I2C Block Write":		FUNC_SMBUS_WRITE_I2C_BLOCK,
	"I2C Block Read":		FUNC_SMBUS_READ_I2C_BLOCK,
}


@pytest.fixture
def funcs(chip):
	"""The mask the adapter reports, through the I2C_FUNCS ioctl."""
	if chip.bus is None:
		pytest.skip("no ft260 i2c bus")
	path = f"/dev/i2c-{chip.bus}"
	if not os.path.exists(path):
		pytest.skip(f"no {path}")
	fd = os.open(path, os.O_RDWR)
	try:
		buf = fcntl.ioctl(fd, I2C_FUNCS, struct.pack("L", 0))
	finally:
		os.close(fd)
	return struct.unpack("L", buf)[0]


def test_funcs_is_the_expected_mask(funcs):
	assert funcs == WANT, f"{funcs:#010x} against {WANT:#010x}"


@pytest.mark.parametrize("name", sorted(NOT_CLAIMED))
def test_funcs_claims_nothing_else(funcs, name):
	"""One case per flag, so a failure names the flag."""
	assert not funcs & NOT_CLAIMED[name], f"{funcs:#010x}"


def test_i2cdetect_agrees_with_the_mask(chip, funcs):
	"""The same mask as users see it."""
	done = subprocess.run(["i2cdetect", "-F", str(chip.bus)],
			      capture_output=True, text=True)
	assert done.returncode == 0, done.stderr

	# Two spaces or more separate the name from the answer. Matching
	# by prefix would take "I2C Block Write" for "I2C".
	printed = dict(re.findall(r"^(.+?)\s{2,}(yes|no)$", done.stdout,
				  re.MULTILINE))

	assert sorted(printed) == sorted(DETECT_LINES), printed
	for name, flag in DETECT_LINES.items():
		want = "yes" if funcs & flag else "no"
		assert printed[name] == want, name


def test_combined_write_over_two_bytes_is_rejected(chip):
	"""ft260_i2c_quirks sets max_comb_1st_msg_len to 2.

	The i2c core rejects the transfer before it reaches the driver,
	so the address does not matter. 0x60 answers nothing on either
	board: if the check ever stops working, the result is a NAK, not
	a write into an EEPROM.
	"""
	done = subprocess.run(["i2ctransfer", "-y", str(chip.bus),
			       "w3@0x60", "0x00", "0x00", "0x00", "r1"],
			      capture_output=True, text=True)

	assert done.returncode != 0
	assert "not supported" in done.stderr.lower(), done.stderr


def test_two_byte_first_message_is_accepted(chip, big_eeprom):
	"""The boundary value, and the transfer a 24LC512 needs."""
	done = ftlib.i2c_read_at(chip.bus, ftlib.BIG_EEPROM, 0x0000, 1)

	assert done.returncode == 0, done.stderr
	assert len(ftlib.hex_bytes(done.stdout)) == 1
