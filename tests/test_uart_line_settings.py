# SPDX-License-Identifier: MIT
"""What ft260_uart_change_speed() puts in the UART config report.

The other UART files set 9600 or 1.5 Mbaud, 8N1, and look at flow
control. This file covers the rest of the termios conversion: baud
rate, data bits, parity and stop bits. It also covers the three
inputs the driver rejects: 300 baud, 12000001 baud and CS5.

With debug=1 the driver prints the whole request, in two records:

    ft260_uart_change_speed: configured termios: flow control: 4, baudrate: 1200,
    ft260_uart_change_speed: data_bit: 8, parity: 0, stop_bit: 0, breaking: 0

One open prints that pair twice. ft260_uart_port_activate()
configures the port at the baud rate it already holds. Then the
termios from userspace runs ft260_uart_change_speed() again. So the
tests read the last pair in the window, not the first.

The log says what the driver asked the chip for. It does not say
what the chip did, and it would read the same if two fields of
ft260_configure_uart_request_report were swapped. The last test
sends data under 7E2 to cover that.

Break has no test because the driver does not implement it.
ft260_uart_ops has no break_ctl, and req.breaking is always
FT260_UART_CFG_BREAKING_NO.
"""

import re
import termios
import time

import pytest
import serial

import ftlib

BAUD_MIN = 1200			# FT260_UART_CFG_BAUD_MIN
BAUD_MAX = 12000000		# FT260_UART_CFG_BAUD_MAX
DEFAULT_BAUD = 9600		# what the driver falls back to
PATTERN = b"ft260-line-settings\n"	# 7 bit clean, so 7E2 carries it
LOG_S = 2.0			# how long to wait for the log


def _config(marker, timeout=LOG_S):
	"""The last request pair the driver logged, as a dict of ints.

	Keys are baudrate, data_bit, parity and stop_bit. Writes a marker
	on every pass, because the driver's lines can stay out of dmesg
	until something else is printed.
	"""
	deadline = time.monotonic() + timeout
	while True:
		ftlib.mark("flush")
		log = ftlib.dmesg_since(marker)
		found = {}
		for line in log.splitlines():
			if "ft260_uart_change_speed" not in line:
				continue
			for name, value in re.findall(r"(\w+): (\d+)", line):
				found[name] = int(value)
		if len(found) >= 4 or time.monotonic() > deadline:
			return found, log
		time.sleep(0.1)


@pytest.fixture
def tty(chip):
	if chip.tty is None:
		pytest.skip("no tty on this chip")
	return chip.tty


def _open(tty, marker, **kwargs):
	"""Open the tty with these termios settings and return the request."""
	with serial.Serial(tty, timeout=1, **kwargs):
		return _config(marker)


@pytest.mark.parametrize("baud", [BAUD_MIN, 115200])
def test_baud_rate_reaches_the_report(tty, baud):
	marker = ftlib.mark(f"baud-{baud}")

	got, log = _open(tty, marker, baudrate=baud)

	assert got.get("baudrate") == baud, log


@pytest.mark.parametrize("baud", [300, BAUD_MAX + 1])
def test_baud_rate_out_of_range_falls_back(tty, baud):
	"""Below FT260_UART_CFG_BAUD_MIN or above the maximum.

	ft260_uart_change_speed() keeps 9600 for the report and calls
	tty_encode_baud_rate(), so userspace sees 9600 too.
	"""
	marker = ftlib.mark(f"baud-bad-{baud}")

	try:
		port = serial.Serial(tty, baudrate=baud, timeout=1)
	except (ValueError, OSError) as err:
		pytest.skip(f"pyserial cannot set {baud} baud: {err}")
	with port:
		got, log = _config(marker)
		cflag = termios.tcgetattr(port.fileno())[2]

	assert f"invalid baud rate {baud}" in log
	assert got.get("baudrate") == DEFAULT_BAUD, log
	assert cflag & termios.CBAUD == termios.B9600


@pytest.mark.parametrize("bits, want", [(serial.SEVENBITS, 7),
					(serial.EIGHTBITS, 8)])
def test_data_bits(tty, bits, want):
	marker = ftlib.mark(f"bits-{want}")

	got, log = _open(tty, marker, baudrate=DEFAULT_BAUD, bytesize=bits)

	assert got.get("data_bit") == want, log


def test_five_data_bits_falls_back_to_eight(tty):
	"""CS5 and CS6 are not supported.

	The driver logs an error, sets FT260_UART_CFG_DATA_BITS_8 and
	rewrites termios->c_cflag, so userspace reads CS8 back.
	"""
	marker = ftlib.mark("bits-5")

	with serial.Serial(tty, baudrate=DEFAULT_BAUD, timeout=1,
			   bytesize=serial.FIVEBITS) as port:
		got, log = _config(marker)
		cflag = termios.tcgetattr(port.fileno())[2]

	assert "invalid data bit size" in log
	assert got.get("data_bit") == 8, log
	assert cflag & termios.CSIZE == termios.CS8


@pytest.mark.parametrize("parity, want", [(serial.PARITY_NONE, 0),
					  (serial.PARITY_ODD, 1),
					  (serial.PARITY_EVEN, 2)])
def test_parity(tty, parity, want):
	marker = ftlib.mark(f"parity-{want}")

	got, log = _open(tty, marker, baudrate=DEFAULT_BAUD, parity=parity)

	assert got.get("parity") == want, log


@pytest.mark.parametrize("stop, want", [(serial.STOPBITS_ONE, 0),
					(serial.STOPBITS_TWO, 2)])
def test_stop_bits(tty, stop, want):
	marker = ftlib.mark(f"stop-{want}")

	got, log = _open(tty, marker, baudrate=DEFAULT_BAUD, stopbits=stop)

	assert got.get("stop_bit") == want, log


def test_seven_bits_even_parity_on_the_wire(tty, peer_link):
	"""7E2 end to end, the only check that the fields are in order.

	The log would look the same if two fields of
	ft260_configure_uart_request_report were swapped. Here the peer
	has to agree with the FT260 for the bytes to arrive.
	"""
	settings = dict(baudrate=DEFAULT_BAUD, bytesize=serial.SEVENBITS,
			parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_TWO,
			timeout=2)

	with serial.Serial(tty, **settings) as ft, \
	     serial.Serial(peer_link, **settings) as other:
		ft.reset_input_buffer()
		other.write(PATTERN)
		other.flush()

		assert ft.read(len(PATTERN)) == PATTERN
