# SPDX-License-Identifier: MIT
"""Writes that span one to five HID output reports must all complete.

FT260_WR_I2C_DATA_MAX is 60, so a longer write is split. Waiting for
bus idle after every chunk, instead of only after the one carrying
STOP, made ft260_xfer_status() spin on -EAGAIN from the second chunk
on and large writes failed.

The two offset bytes count towards the transaction length, so a 59
byte payload already takes two reports. The 24LC512 has 128 byte
pages and a write that crosses a page boundary wraps inside the page,
so the content is only read back for the writes that stay in one
page; the longer ones are checked for completing.

Only runs against 0x51, the EEPROM that holds no FT260 configuration.
"""

import time

import pytest

import ftlib

BASE = 0x1000			# page aligned on a 128 byte page
PAGE = 128
REPORT = 60			# FT260_WR_I2C_DATA_MAX
WRITE_MS = 0.02			# EEPROM write cycle before a read back
LENGTHS = (58, 59, 126, 190, 254)
ERRORS = ("timeout", "failed", "-110", "0x72", "eagain")


def pattern(length):
	return bytes((i * 7 + 13) & 0xFF for i in range(length))


@pytest.fixture(scope="module")
def target(chip, eeprom):
	if eeprom != "0x51":
		pytest.skip(f"writes only run against 0x51, found {eeprom}")
	return chip.bus, eeprom


@pytest.mark.parametrize(
	"length", LENGTHS,
	ids=[f"{n}B_{-(-(n + 2) // REPORT)}" for n in LENGTHS])
def test_write_completes(target, length):
	bus, addr = target
	data = pattern(length)
	marker = ftlib.mark(f"write-{length}")

	done = ftlib.i2c_write(bus, addr, BASE, data)
	assert done.returncode == 0, done.stderr.strip()
	noise = [line for line in ftlib.dmesg_since(marker).splitlines()
		 if any(word in line.lower() for word in ERRORS)]
	assert noise == []

	if length > PAGE - (BASE % PAGE):
		return
	time.sleep(WRITE_MS)
	back = ftlib.i2c_read_at(bus, addr, BASE, length)
	assert back.returncode == 0, back.stderr.strip()
	assert bytes(int(b, 16) for b in back.stdout.split()) == data
