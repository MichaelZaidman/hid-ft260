# SPDX-License-Identifier: MIT
"""Data rate on the I2C bus, to catch a performance regression.

i2cperf moves a fixed number of bytes and prints the payload rate, the
efficiency, the transfer count and the chunk size. The tests compare
the rate with a floor measured on this bench. The numbers behind the
floors are in TESTS.md, section "I2C write throughput".

The chunk sizes are the large ones on purpose. The driver prints one
debug line per transfer, so debug=1 costs per transfer, not per byte.
At chunk 240 the 4096 bytes are 18 transfers and the measured rate at
debug=1 sits inside the debug=0 spread, within one percent. The suite
loads the module with debug=1, and these tests run that way.

The write chunk is one EEPROM page, 128 bytes on the 24LC512, and the
range starts at address 0. A larger chunk wraps inside the page: a
240-byte write puts its bytes 128 to 239 back on addresses 0 to 111.
The rate is still the bus rate, but the content is then garbage, so
the write test uses a page and checks what was stored.

A new EEPROM moves the write floor. The page size decides whether a
chunk wraps and the write cycle time decides how long the part NAKs,
and both are per part. The read floor is bus clock and driver cost,
so it stays. Re-measure the write floor when the part changes.

i2cperf prints a stats table even when every transfer failed, see
TESTS.md, so the tests also check the transfer count.
"""

import re
import subprocess

import pytest

import ftlib

SIZE = 4096			# bytes per run
RANGE = "0x0-0xfff"		# the same 4096 bytes, as i2cperf wants them
CLOCK_KHZ = "100"		# what the floors were measured at
# A payload rate can never reach the clock, because every transfer
# also carries address and control bytes. A run where no transfer
# happened prints a rate above it, see TESTS.md.
CLOCK_BPS = 100 * 1000

READ_CHUNK = 240
READ_IOS = 18			# ceil(4096 / 240)
# Measured 2026-10-20, 24LC512 at 0x51, debug=1: 51408 to 51926 bps.
READ_FLOOR = 41000		# 80% of the lowest of three runs

WRITE_CHUNK = 128		# one page of the 24LC512
WRITE_IOS = 32			# 4096 / 128
# Measured 2026-10-20, same conditions: 55151 to 55312 bps.
WRITE_FLOOR = 44000		# 80% of the lowest of three runs

STATS = re.compile(r"^\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s*$", re.M)


def _perf(tool, bus, addr, mode, chunk):
	"""One i2cperf run, as (rate, efficiency, size, ios, chunk).

	The exit code matters. core_stat() computes the transfer count
	and the rate from the requested range, not from the transfers
	that ran, and run_with_stat() adds to total_time only for the
	ones that worked, so a run that stops halfway prints the full
	size at a higher rate. The exit code is the only way to tell
	that from a complete run.
	"""
	done = subprocess.run([tool, "-S", mode[0], mode[1], "-o", "2",
			       "-r", RANGE, "-s", str(chunk), str(bus), addr],
			      capture_output=True, text=True)
	assert done.returncode == 0, (
		f"i2cperf exited {done.returncode}: {done.stderr.strip()}\n"
		"Copies older than the exit code fix return 1 after every "
		"fill, successful or not. Update i2cperf.")
	found = STATS.search(done.stdout)
	assert found, done.stdout[-400:]
	return [int(value) for value in found.groups()]


@pytest.fixture
def perf(chip, big_eeprom, i2cperf):
	"""Run i2cperf on the part the tests may write, at a known clock."""
	clock = ftlib.read_attr(chip.i2c, "clock")
	if clock != CLOCK_KHZ:
		pytest.skip(f"clock is {clock} kHz, the floors are for {CLOCK_KHZ}")

	def _run(mode, chunk):
		return _perf(i2cperf, chip.bus, big_eeprom, mode, chunk)

	return _run


def test_read_rate(perf):
	rate, eff, size, ios, chunk = perf(("-d", "2"), READ_CHUNK)

	assert (size, ios, chunk) == (SIZE, READ_IOS, READ_CHUNK)
	assert rate < CLOCK_BPS, f"{rate} bps is above the bus clock"
	assert rate >= READ_FLOOR, f"{rate} bps, floor {READ_FLOOR}, {eff}% efficient"


def test_write_rate_and_content(chip, big_eeprom, perf):
	"""One page per transfer, so the data can be checked as well."""
	rate, eff, size, ios, chunk = perf(("-f", "2"), WRITE_CHUNK)

	assert (size, ios, chunk) == (SIZE, WRITE_IOS, WRITE_CHUNK)
	assert rate < CLOCK_BPS, f"{rate} bps is above the bus clock"
	assert rate >= WRITE_FLOOR, f"{rate} bps, floor {WRITE_FLOOR}, {eff}% efficient"

	# i2cperf -f 2 fills with an increment, so the byte at address a
	# is a & 0xff. Read across a page boundary to catch a wrap.
	done = ftlib.i2c_read_at(chip.bus, big_eeprom, 0x0078, 16)
	assert done.returncode == 0, done.stderr.strip()
	assert ftlib.hex_bytes(done.stdout) == list(range(0x78, 0x88))
