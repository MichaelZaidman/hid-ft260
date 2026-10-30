# SPDX-License-Identifier: MIT
"""The SMBus transfer sizes ft260_smbus_xfer() implements.

Every other I2C test in the suite calls i2ctransfer, which goes
through master_xfer. i2cget and i2cset use the SMBus entry point, so
these tests are the only ones that reach ft260_smbus_xfer(). Each
read with a command byte is a combined transfer there: a write with
FT260_FLAG_START, then a read with
FT260_FLAG_START_STOP_REPEATED. BLOCK_DATA has its own file,
test_i2c_bus_recovery.py.

The reads run against the EEPROM at 0x50. Its word address is one
byte, exactly what an SMBus command byte carries, so every answer has
a counterpart in a dump taken with i2ctransfer.

The writes run against the 24LC512 at 0x51, which holds no FT260
configuration. That part takes a two byte word address, so the SMBus
command byte is only the high half of it:

    byte data   command and one byte: both are the address, nothing
                is stored, the slave's pointer moves
    word data   command and two bytes: one byte stored at the address
                made of the command and the first byte
    i2c block   command and n bytes: n-1 bytes stored at that address

Send byte is left out: one byte leaves that EEPROM waiting for the
second half of its address. Every write is read back with i2ctransfer
and the original bytes are put back.
"""

import time

import ftlib

OFFSET = 0x10			# where the reads look, inside the dump
BLOCK_LEN = 16			# bytes of an i2c block read
ADDR = 0x1000			# two byte word address the writes use
PAYLOAD = [0xa5, 0x5a, 0x00, 0xff, 0x0f, 0xf0, 0x33, 0xcc]
WRITE_CYCLE_S = 0.01		# the EEPROM NAKs while it stores a page


def _restore(bus, addr, offset, data):
	ftlib.i2c_write(bus, addr, offset, data)
	time.sleep(WRITE_CYCLE_S)


def test_byte_data_read(chip, config_eeprom):
	done = ftlib.i2cget(chip.bus, ftlib.CONFIG_EEPROM, OFFSET, "b")

	assert done.returncode == 0, done.stderr.strip()
	assert ftlib.hex_bytes(done.stdout) == [config_eeprom[OFFSET]]


def test_word_data_read(chip, config_eeprom):
	done = ftlib.i2cget(chip.bus, ftlib.CONFIG_EEPROM, OFFSET, "w")

	assert done.returncode == 0, done.stderr.strip()
	want = config_eeprom[OFFSET] | config_eeprom[OFFSET + 1] << 8
	assert ftlib.hex_bytes(done.stdout) == [want]


def test_i2c_block_read(chip, config_eeprom):
	done = ftlib.i2cget(chip.bus, ftlib.CONFIG_EEPROM, OFFSET, "i",
			    BLOCK_LEN)

	assert done.returncode == 0, done.stderr.strip()
	assert ftlib.hex_bytes(done.stdout) == \
		config_eeprom[OFFSET:OFFSET + BLOCK_LEN]


def test_send_byte_then_receive_byte(chip, config_eeprom):
	"""Send byte leaves the word address, receive byte reads there."""
	done = ftlib.i2cset(chip.bus, ftlib.CONFIG_EEPROM, OFFSET, mode="c")
	assert done.returncode == 0, done.stderr.strip()

	done = ftlib.i2cget(chip.bus, ftlib.CONFIG_EEPROM)

	assert done.returncode == 0, done.stderr.strip()
	assert ftlib.hex_bytes(done.stdout) == [config_eeprom[OFFSET]]


def test_byte_data_write_moves_the_pointer(chip, big_eeprom):
	"""Nothing is stored: the EEPROM takes both bytes as its address.

	PAYLOAD is written first so that the byte at ADDR differs from
	the one the pointer stands on before the transfer. Without that
	the receive byte could answer from the old position and the test
	would pass on stored content.
	"""
	length = len(PAYLOAD)
	orig = ftlib.hex_bytes(
		ftlib.i2c_read_at(chip.bus, big_eeprom, ADDR, length).stdout)
	_restore(chip.bus, big_eeprom, ADDR, PAYLOAD)

	try:
		ftlib.i2c_read_at(chip.bus, big_eeprom, ADDR + 4, 1)

		done = ftlib.i2cset(chip.bus, big_eeprom, ADDR >> 8,
				    ADDR & 0xff, mode="b")
		assert done.returncode == 0, done.stderr.strip()

		done = ftlib.i2cget(chip.bus, big_eeprom)
		assert done.returncode == 0, done.stderr.strip()
		assert ftlib.hex_bytes(done.stdout) == [PAYLOAD[0]]

		done = ftlib.i2c_read_at(chip.bus, big_eeprom, ADDR, length)
		assert ftlib.hex_bytes(done.stdout) == PAYLOAD
	finally:
		_restore(chip.bus, big_eeprom, ADDR, orig)


def test_word_data_write(chip, big_eeprom):
	orig = ftlib.hex_bytes(
		ftlib.i2c_read_at(chip.bus, big_eeprom, ADDR, 1).stdout)
	value = orig[0] ^ 0xff

	# Low byte first on the wire: the address, then the data.
	done = ftlib.i2cset(chip.bus, big_eeprom, ADDR >> 8,
			    (ADDR & 0xff) | value << 8, mode="w")
	assert done.returncode == 0, done.stderr.strip()
	time.sleep(WRITE_CYCLE_S)

	try:
		done = ftlib.i2c_read_at(chip.bus, big_eeprom, ADDR, 1)
		assert done.returncode == 0, done.stderr.strip()
		assert ftlib.hex_bytes(done.stdout) == [value]
	finally:
		_restore(chip.bus, big_eeprom, ADDR, orig)


def test_i2c_block_write(chip, big_eeprom):
	length = len(PAYLOAD)
	orig = ftlib.hex_bytes(
		ftlib.i2c_read_at(chip.bus, big_eeprom, ADDR, length).stdout)

	done = ftlib.i2cset(chip.bus, big_eeprom, ADDR >> 8, ADDR & 0xff,
			    *PAYLOAD, mode="i")
	assert done.returncode == 0, done.stderr.strip()
	time.sleep(WRITE_CYCLE_S)

	try:
		done = ftlib.i2c_read_at(chip.bus, big_eeprom, ADDR, length)
		assert done.returncode == 0, done.stderr.strip()
		assert ftlib.hex_bytes(done.stdout) == PAYLOAD
	finally:
		_restore(chip.bus, big_eeprom, ADDR, orig)
