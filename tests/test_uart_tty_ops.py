# SPDX-License-Identifier: MIT
"""The tty operations no other UART test reaches.

test_uart_flow.py and test_uart_modem_lines.py open the tty, so they
cover open, close, write, set_termios and tiocmget. This file covers
the rest of ft260_uart_ops: proc_show, get_icount, chars_in_buffer
and hangup.

proc_show writes /proc/tty/driver/ft260_ser. The name comes from
tty_driver->driver_name. Only root can read the file. It holds one
line per registered port:

    ft260 info:1.0 driver revision:
    0: uart:FT260 tx:0 rx:38

The tx and rx counters live in struct ft260_device, not in the
tty_struct, so a close does not reset them. Every test here reads
them twice and compares the two values.

ft260_uart_write_room() has no test. Userspace sees the value only
as a short return from a non-blocking write larger than
XMIT_FIFO_SIZE. How short that return is depends on how fast the
fifo drains into the chip.
"""

import errno
import fcntl
import os
import struct
import termios
import time

import pytest
import serial

import ftlib

TIOCGICOUNT = 0x545D
TIOCOUTQ = 0x5411
TIOCVHANGUP = 0x5437
BAUD = 9600
PATTERN = b"ft260-tty-ops-pattern\n"
# struct serial_icounter_struct: 11 counters then int reserved[9].
ICOUNT = "20i"
COUNT_S = 2.0			# how long a counter may take to settle


@pytest.fixture
def tty(chip):
	if chip.tty is None:
		pytest.skip("no tty on this chip")
	if ftlib.proc_counts(ftlib.tty_index(chip.tty)) is None:
		pytest.skip(f"no port line in {ftlib.PROC_TTY}")
	return chip.tty


def _icount(fd):
	"""tx and rx as ft260_uart_get_icount() reports them."""
	buf = fcntl.ioctl(fd, TIOCGICOUNT, struct.pack(ICOUNT, *([0] * 20)))
	fields = struct.unpack(ICOUNT, buf)
	return fields[5], fields[4]	# tx is the sixth, rx the fifth


def _wait_counts(index, want, timeout=COUNT_S):
	"""Poll the proc counters until they reach want, or time out."""
	deadline = time.monotonic() + timeout
	while True:
		got = ftlib.proc_counts(index)
		if got == want or time.monotonic() > deadline:
			return got
		time.sleep(0.05)


def test_proc_lists_the_port(tty):
	with open(ftlib.PROC_TTY) as f:
		lines = f.read().splitlines()

	assert lines[0] == "ft260 info:1.0 driver revision:"
	index = ftlib.tty_index(tty)
	assert any(line.startswith(f"{index}: uart:FT260 tx:") for line in lines)


def test_proc_tx_counts_the_write(tty):
	index = ftlib.tty_index(tty)

	with serial.Serial(tty, BAUD, timeout=1) as port:
		before = ftlib.proc_counts(index)
		port.write(PATTERN)
		port.flush()
		after = _wait_counts(index, (before[0] + len(PATTERN), before[1]))

	assert after[0] - before[0] == len(PATTERN)


def test_proc_rx_counts_the_peer(tty, peer_link):
	index = ftlib.tty_index(tty)

	with serial.Serial(tty, BAUD, timeout=2) as port, \
	     serial.Serial(peer_link, BAUD, timeout=2) as other:
		port.reset_input_buffer()
		before = ftlib.proc_counts(index)
		other.write(PATTERN)
		other.flush()
		assert port.read(len(PATTERN)) == PATTERN
		after = _wait_counts(index, (before[0], before[1] + len(PATTERN)))

	assert after[1] - before[1] == len(PATTERN)


def test_get_icount_matches_proc(tty):
	index = ftlib.tty_index(tty)

	with serial.Serial(tty, BAUD, timeout=1) as port:
		before = ftlib.proc_counts(index)
		port.write(PATTERN)
		port.flush()
		want = _wait_counts(index, (before[0] + len(PATTERN), before[1]))
		got = _icount(port.fileno())

	assert got == want


def test_chars_in_buffer_empty_when_idle(tty):
	with serial.Serial(tty, BAUD, timeout=1) as port:
		port.write(PATTERN)
		port.flush()
		termios.tcdrain(port.fileno())
		queued = struct.unpack("i", fcntl.ioctl(port.fileno(), TIOCOUTQ,
						       struct.pack("i", 0)))[0]

	assert queued == 0


def test_hangup_breaks_the_open_fd(tty):
	"""TIOCVHANGUP is the only way into ft260_uart_hangup().

	A close ends the session too, and it closes the fd as well.
	After TIOCVHANGUP the fd stays open. The tty core swaps it to
	hung_up_tty_fops, where hung_up_tty_read() returns 0, so a read
	gives end of file, and hung_up_tty_write() returns -EIO.
	"""
	fd = os.open(tty, os.O_RDWR | os.O_NOCTTY)
	try:
		fcntl.ioctl(fd, TIOCVHANGUP)
		assert os.read(fd, 1) == b""
		with pytest.raises(OSError) as err:
			os.write(fd, PATTERN)
	finally:
		os.close(fd)

	assert err.value.errno == errno.EIO

	with serial.Serial(tty, BAUD, timeout=1) as port:
		assert port.write(PATTERN) == len(PATTERN)
