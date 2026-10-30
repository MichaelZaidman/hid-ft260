# SPDX-License-Identifier: MIT
"""One I2C read and one UART transfer, to prove both functions still work.

The I2C side reads the EEPROM on the FT260 bus. The UART side sends a
line from the ft260 tty and expects it on the peer. Run after the GPIO tests, which take pins away from both.
"""

import os

import pytest
import serial

import ftlib

BAUD = 9600


def test_eeprom_read_16_bytes(chip, eeprom):
	done = ftlib.i2c_read(chip.bus, eeprom, 16)
	assert done.returncode == 0, done.stderr.strip()
	assert len(done.stdout.split()) == 16


def test_uart_to_peer(chip, peer_link):
	if not chip.tty:
		pytest.skip("the chip has no tty")
	msg = f"ft260-regr-{os.getpid()}\n".encode()
	with serial.Serial(peer_link, BAUD, timeout=5) as rx, \
	     serial.Serial(chip.tty, BAUD, timeout=5) as tx:
		rx.reset_input_buffer()
		tx.write(msg)
		tx.flush()
		got = rx.read_until(msg)
	assert msg in got, f"peer got {got!r}"
