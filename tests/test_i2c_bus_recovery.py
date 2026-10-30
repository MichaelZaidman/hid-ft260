# SPDX-License-Identifier: MIT
"""The driver frees a bus that a slave is holding.

An SMBus block read takes the first byte the slave sends as the length
of the block. An EEPROM has no length byte, it answers with stored
data, and a value above I2C_SMBUS_BLOCK_MAX makes ft260_smbus_xfer()
stop with -EPROTO. The count phase carries no STOP, so the EEPROM
keeps SDA low while it waits for the next clock. FT260_SET_I2C_RESET
restarts the controller without clocking the bus, the chip cannot put
a STOP on the line, and every later START ends in arbitration lost,
bus_status 0x72. Neither the sysfs i2c_reset, nor unbind and bind, nor
a module reload clears that state.

ft260_i2c_abort() reads the status after the reset, finds
FT260_I2C_STATUS_BUS_BUSY and calls i2c_recover_bus(). The callbacks
turn the I2C controller off, which gives GPIO0 and GPIO1 back their
GPIO function, and i2c_generic_scl_recovery() clocks SCL nine times
and makes a STOP.

The tests read the EEPROM at 0x50, the one holding the FT260
configuration. It takes a one byte word address, which is what an
SMBus command byte is. Both boards here carry it. Nothing is written.
"""

import pytest

import ftlib

CONFIG_EEPROM = ftlib.CONFIG_EEPROM
BLOCK_MAX = 32			# I2C_SMBUS_BLOCK_MAX
FREE_ADDRS = ("0x30", "0x31", "0x32")


def _offset_holding(data, low, high):
	"""Offset of the first byte in low..high, with its block in range."""
	for offset, value in enumerate(data):
		if low <= value <= high and offset + 1 + value <= len(data):
			return offset
	return None


def _bad_count_offset(data):
	offset = _offset_holding(data, BLOCK_MAX + 1, 0xff)
	if offset is None:
		pytest.skip("no byte above the block maximum in the dump")
	return offset


def test_block_read_with_invalid_count_fails(chip, config_eeprom):
	offset = _bad_count_offset(config_eeprom)
	marker = ftlib.mark("block-bad-count")

	done = ftlib.i2cget(chip.bus, CONFIG_EEPROM, offset, "s")

	assert done.returncode != 0, done.stdout.strip()
	log = ftlib.dmesg_since(marker)
	assert f"invalid count {config_eeprom[offset]}" in log, log


def test_bus_works_after_an_invalid_count(chip, config_eeprom):
	offset = _bad_count_offset(config_eeprom)
	marker = ftlib.mark("block-recovery")

	ftlib.i2cget(chip.bus, CONFIG_EEPROM, offset, "s")

	log = ftlib.dmesg_since(marker)
	assert "recovering" in log, log
	done = ftlib.i2c_read(chip.bus, CONFIG_EEPROM, 4)
	assert done.returncode == 0, done.stderr.strip()


def test_address_nak_does_not_recover(chip, config_eeprom):
	free = next((addr for addr in FREE_ADDRS
		     if ftlib.i2c_read(chip.bus, addr, 1).returncode != 0), None)
	if free is None:
		pytest.skip(f"all of {FREE_ADDRS} answer on i2c-{chip.bus}")
	marker = ftlib.mark("address-nak")

	done = ftlib.i2c_read(chip.bus, free, 1)

	assert done.returncode != 0, done.stdout.strip()
	log = ftlib.dmesg_since(marker)
	assert "recovering" not in log, log


def test_block_read_with_valid_count(chip, config_eeprom):
	offset = _offset_holding(config_eeprom, 1, BLOCK_MAX)
	if offset is None:
		pytest.skip("no byte between 1 and the block maximum")
	count = config_eeprom[offset]

	done = ftlib.i2cget(chip.bus, CONFIG_EEPROM, offset, "s")

	assert done.returncode == 0, done.stderr.strip()
	assert ftlib.hex_bytes(done.stdout) == \
		config_eeprom[offset + 1:offset + 1 + count]
