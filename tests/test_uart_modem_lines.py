# SPDX-License-Identifier: MIT
"""Modem line control over the FT260 UART, through TIOCMGET and TIOCMSET.

Both handlers refuse unless gpio_en covers FT260_GPIO_UART_MODE_4_SET,
because the pins must be muxed as GPIO. tiocmget inverts RTS, CTS, DTR
and DSR, and leaves DCD and RI as read. tiocmset acts on RTS and DTR
only, writing 0 to assert.

RTS and CTS are cabled between the board and the peer, so asserting one
side's RTS must show on the other side's CTS. The driver's inversion and
the cable's active-low sense cancel, so the two booleans match. DTR and
DSR are not cabled here, so they are only checked for not failing.
"""

import contextlib
import errno
import os

import pytest
import serial

import ftlib

WANT = 0x2CB0			# FT260_GPIO_UART_MODE_4_SET
DCD_RI = 0x0030			# FT260_GPIO_UART_DCD_RI
BAUD = 9600

DCD = 4				# FT260_GPIO_4, read as TIOCM_CAR
RI = 5				# FT260_GPIO_5, read as TIOCM_RNG
DSR = 13			# FT260_GPIO_H, read as TIOCM_DSR


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


def test_dtr_toggles_without_error(pair):
	ft, _ = pair
	for state in (True, False):
		ft.dtr = state


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


@pytest.mark.parametrize("value", [1, 0], ids=["high", "low"])
def test_dsr_is_inverted_on_the_pin(chip, pair, value):
	"""GPIOH is read as TIOCM_DSR inverted, unlike DCD and RI.

	DSR is not cabled here, so driving it is safe. GPIOE is left
	alone: it is CTS and the peer's RTS drives it.
	"""
	ft, _ = pair
	with driven(chip.base + DSR, value):
		assert ft.dsr is not bool(value)


def test_dcd_ri_attribute_gates_the_ioctl(chip, pair):
	"""uart_dcd_ri 1 takes GPIO4 and GPIO5 back, so both ioctls fail.

	The guard wants the whole FT260_GPIO_UART_MODE_4_SET, and that
	includes the two DCD/RI bits. The attribute is write only, so the
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

		with pytest.raises(OSError) as err:
			ft.cts
		assert err.value.errno == errno.EINVAL
	finally:
		marker = ftlib.mark("dcd-ri-0")
		ftlib.write_attr(chip.uart, "uart_dcd_ri", 0)

	mask = ftlib.mask_of(os.path.basename(chip.uart), marker)
	assert int(mask, 16) & DCD_RI == DCD_RI
	assert ft.cts in (True, False)


def test_refused_while_rts_cts_owns_the_pins(chip, peer_link):
	"""With CRTSCTS the mask drops bits 7 and 10, so both ioctls fail."""
	with pytest.raises(OSError) as err:
		with serial.Serial(chip.tty, BAUD, rtscts=True, timeout=2) as ft:
			ft.cts
	assert err.value.errno == errno.EINVAL
