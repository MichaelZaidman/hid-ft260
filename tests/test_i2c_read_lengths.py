# SPDX-License-Identifier: MIT
"""Reads that span one to four HID input reports must not time out.

The driver scales the read timeout with the length:

    timeout = FT260_RD_ONE_REPORT_TO * DIV_ROUND_UP(rd_len,
                                                    FT260_RD_ONE_REPORT_MAX)

One fixed timeout for every length was the defect, so the lengths that
cross a 60 byte report boundary are the ones worth reading. Each length
is read three times from the device on the bus and the kernel log
must stay clean.
"""

import pytest

import ftlib

LENGTHS = (60, 61, 120, 180, 240)
REPEATS = 3
REPORT = 60			# FT260_RD_ONE_REPORT_MAX
ERRORS = ("timeout", "failed", "-110", "0x72")


@pytest.mark.parametrize(
	"length", LENGTHS,
	ids=[f"{n}B_{-(-n // REPORT)}" for n in LENGTHS])
def test_read_completes(chip, eeprom, length):
	marker = ftlib.mark(f"read-{length}")
	for _ in range(REPEATS):
		done = ftlib.i2c_read(chip.bus, eeprom, length)
		assert done.returncode == 0, done.stderr.strip()
		assert len(done.stdout.split()) == length
	noise = [line for line in ftlib.dmesg_since(marker).splitlines()
		 if any(word in line.lower() for word in ERRORS)]
	assert noise == []
