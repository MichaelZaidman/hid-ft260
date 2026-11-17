# SPDX-License-Identifier: MIT
"""Modem line control over the FT260 UART, through TIOCMGET and TIOCMSET.

A pin that gpio_en gives to the gpiochip is read and written through
the GPIO report. tiocmget inverts RTS, CTS, DTR and DSR, and leaves DCD
and RI as read. tiocmset acts on RTS and DTR only, writing 0 to assert.

A pin in UART function is not read. tiocmget reports CTS, DSR and DCD
active and RI not active, as the get_mctrl() comment in serial_core.h
asks. It reports RTS or DTR under flow control active, because the
chip holds it active while it can take data. tiocmset ignores RTS while
RTS/CTS flow control owns it, and DTR while DTR/DSR flow control owns
it.

RTS and CTS are cabled both ways between the board and the peer. Board
DTR goes to peer DSR, and peer DTR goes to board DSR through 1 kOhm. So
asserting one side's output must show on the other side's input. The
driver's inversion and the cable's active-low sense cancel, so the two
booleans match. DCD and RI are not cabled.
"""

import contextlib
import fcntl
import os
import struct
import termios
import types

import pytest
import serial

import ftlib

WANT = 0x2CB0			# FT260_GPIO_UART_MODE_4_SET
DCD_RI = 0x0030			# FT260_GPIO_UART_DCD_RI
BAUD = 9600
DATA = b"0123456789abcdef"
DTR_DSR = 2			# FT260_UART_CFG_FLOW_CTRL_DTR_DSR

DCD = 4				# FT260_GPIO_4, read as TIOCM_CAR
RI = 5				# FT260_GPIO_5, read as TIOCM_RNG

# Flow control -> owned pair, free pair, gpio_en bits of the owned pair.
# The board output of a pair goes to the peer input of the same name,
# and the peer output to the board input.
FLOW = {
	"rtscts": (("rts", "cts"), ("dtr", "dsr"), 0x0480),
	"dtrdsr": (("dtr", "dsr"), ("rts", "cts"), 0x2800),
}
TIOCM = {"rts": termios.TIOCM_RTS, "cts": termios.TIOCM_CTS,
	 "dtr": termios.TIOCM_DTR, "dsr": termios.TIOCM_DSR}


def tiocmget(port):
	"""Modem bits from the driver. pyserial's rts and dtr are a cache."""
	buf = fcntl.ioctl(port.fd, termios.TIOCMGET, struct.pack("I", 0))
	return struct.unpack("I", buf)[0]


@contextlib.contextmanager
def driven(line, value):
	"""Hold one line at value, as an output, then give it back."""
	ftlib.export(line)
	try:
		ftlib.set_direction(line, "out")
		ftlib.set_value(line, value)
		yield
	finally:
		with contextlib.suppress(OSError):
			ftlib.set_direction(line, "in")
		ftlib.unexport(line)


@pytest.fixture
def pair(chip, peer_link):
	"""Both ends open with no flow control, so the pins stay GPIO."""
	mask = ftlib.gpio_en_now(chip.uart)
	if mask and int(mask, 16) & WANT != WANT:
		pytest.skip(f"gpio_en {mask} does not cover the modem pins")
	with serial.Serial(chip.tty, BAUD, timeout=2) as ft, \
	     serial.Serial(peer_link, BAUD, timeout=2) as other:
		yield ft, other


@pytest.fixture(params=sorted(FLOW))
def flow(request, chip, peer_link):
	"""Both ends open, the board with one hardware flow control mode.

	CRTSCTS selects RTS/CTS. DTR/DSR has no termios flag, so it is
	written to uart_mode with the port open. Opening the port calls
	ft260_uart_change_speed(), which puts the termios mode back.
	"""
	owned, free, bits = FLOW[request.param]
	rtscts = request.param == "rtscts"
	marker = ftlib.mark(f"flow-{request.param}")
	with serial.Serial(peer_link, BAUD, timeout=2) as other, \
	     serial.Serial(chip.tty, BAUD, rtscts=rtscts, timeout=2) as ft:
		before = None
		if not rtscts:
			before = ftlib.read_attr(chip.uart, "uart_mode")
			ftlib.write_attr(chip.uart, "uart_mode", DTR_DSR)
		try:
			mask = ftlib.mask_of(os.path.basename(chip.uart), marker)
			assert mask is not None, "the driver logged no mask"
			mask = int(mask, 16)
			assert mask & bits == 0
			assert mask & (WANT & ~bits) == WANT & ~bits

			yield types.SimpleNamespace(
				ft=ft, other=other, out=owned[0], inp=owned[1],
				free_out=free[0], free_in=free[1],
				owned=TIOCM[owned[0]] | TIOCM[owned[1]])
		finally:
			if before is not None:
				ftlib.write_attr(chip.uart, "uart_mode", before)


@pytest.mark.parametrize("line", ["cts", "dsr", "cd", "ri"])
def test_get_reads_every_input_line(pair, line):
	ft, _ = pair
	assert getattr(ft, line) in (True, False)


@pytest.mark.parametrize("state", [True, False, True, False])
def test_rts_reaches_the_peer_cts(pair, state):
	ft, other = pair
	ft.rts = state
	assert other.cts is state


@pytest.mark.parametrize("state", [True, False])
def test_peer_rts_reaches_the_chip_cts(pair, state):
	ft, other = pair
	other.rts = state
	assert ft.cts is state


@pytest.mark.parametrize("state", [True, False, True, False])
def test_dtr_reaches_the_peer_dsr(pair, state):
	ft, other = pair
	ft.dtr = state
	assert other.dsr is state


@pytest.mark.parametrize("state", [True, False])
def test_peer_dtr_reaches_the_chip_dsr(pair, state):
	ft, other = pair
	other.dtr = state
	assert ft.dsr is state


@pytest.mark.parametrize("value", [1, 0], ids=["high", "low"])
def test_dcd_follows_the_pin(chip, pair, value):
	"""GPIO4 is read as TIOCM_CAR with no inversion.

	Nothing on the bench drives DCD, so the gpiochip can. The tty
	reads the same pin through ft260_gpio_get_all().
	"""
	ft, _ = pair
	with driven(chip.base + DCD, value):
		assert ft.cd is bool(value)


@pytest.mark.parametrize("value", [1, 0], ids=["high", "low"])
def test_ri_follows_the_pin(chip, pair, value):
	"""GPIO5 is read as TIOCM_RNG with no inversion."""
	ft, _ = pair
	with driven(chip.base + RI, value):
		assert ft.ri is bool(value)


def test_dcd_ri_attribute_reports_dcd_on_ri_off(chip, pair):
	"""uart_dcd_ri 1 takes GPIO4 and GPIO5 from the gpiochip.

	The driver does not read the two pins then, so tiocmget reports
	DCD active and RI not active. The attribute is write only, so the
	old value cannot be read; 0 is what the mask shows now.

	The write has to happen with the port already open. Opening it
	calls ft260_uart_change_speed(), which ends in
	ft260_gpio_en_update(FT260_SET_UART_MODE, flow_ctrl) and puts the
	whole mode 4 mask back, undoing the attribute.
	"""
	ft, _ = pair
	assert ft.cts in (True, False)

	marker = ftlib.mark("dcd-ri-1")
	ftlib.write_attr(chip.uart, "uart_dcd_ri", 1)
	try:
		mask = ftlib.mask_of(os.path.basename(chip.uart), marker)
		assert mask is not None, "the driver logged no mask"
		assert int(mask, 16) & DCD_RI == 0

		assert ft.cd is True
		assert ft.ri is False
		assert ft.cts in (True, False)
	finally:
		marker = ftlib.mark("dcd-ri-0")
		ftlib.write_attr(chip.uart, "uart_dcd_ri", 0)

	mask = ftlib.mask_of(os.path.basename(chip.uart), marker)
	assert int(mask, 16) & DCD_RI == DCD_RI
	assert ft.cts in (True, False)


def test_flow_reports_owned_lines_active(flow):
	"""The owned pair reads active while the peer drives its line off."""
	setattr(flow.other, flow.out, False)
	assert tiocmget(flow.ft) & flow.owned == flow.owned


def test_flow_free_pair_follows_the_peer(flow):
	for state in (True, False, True, False):
		setattr(flow.other, flow.free_out, state)
		assert getattr(flow.ft, flow.free_in) is state


def test_flow_tiocmset_ignores_the_owned_line(flow):
	"""Setting the owned output off is not an error and does nothing.

	With no data flowing the chip holds the line active.
	"""
	setattr(flow.ft, flow.out, False)
	assert getattr(flow.other, flow.inp) is True
	assert tiocmget(flow.ft) & flow.owned == flow.owned


def test_flow_tiocmset_sets_the_free_line(flow):
	"""The free output follows tiocmset, and the flow control still works.

	tiocmset writes the whole GPIO report, which also carries the pins
	in UART function.
	"""
	for state in (True, False, True, False):
		setattr(flow.ft, flow.free_out, state)
		assert getattr(flow.other, flow.free_in) is state
		assert getattr(flow.other, flow.inp) is True

	flow.other.write(DATA)
	assert flow.ft.read(len(DATA)) == DATA
	flow.ft.write(DATA)
	assert flow.other.read(len(DATA)) == DATA
