# SPDX-License-Identifier: MIT
"""The UART side of the power saving workaround.

In power saving mode the chip loses data on Rx above 4800 baud after
5 s of silence. The driver keeps the chip awake with a timer. The
timer runs while the tty is open and the baud rate is above
FT260_UART_EN_PW_SAVE_BAUD, 4800. ft260_uart_start_wakeup() re-arms
it every FT260_WAKEUP_NEEDED_AFTER_MS, 4800 ms, and the work it
schedules reads the chip version report.

ft260_uart_wakeup_workaraund_enable() arms and disarms the timer. It
logs one line per call:

    ft260_uart_wakeup_workaraund_enable: activate wakeup workaround

It prints nothing when the chip has power saving disabled, so the
whole file skips in that case. The log tests need debug=1, which
ftlib.insmod() always passes.

One open logs that line twice. ft260_uart_port_activate() arms the
workaround at the baud rate the port already holds. Then the termios
from userspace runs ft260_uart_change_speed(), which arms or disarms
it again:

    activate wakeup workaround
    ft260_uart_port_activate: configured baudrate = 9600
    deactivate wakeup workaround
    ft260_uart_change_speed: ... baudrate: 4800

So every test reads the last line in the window, not the first.

These lines also reach the log late. Measured here: 0.5 s after the
close, dmesg still had no shutdown line. The line appeared as soon
as the test wrote one record of its own to /dev/kmsg. So every read
below writes a marker first, then polls.
"""

import time

import pytest
import serial

import ftlib

FAST_BAUD = 9600		# above FT260_UART_EN_PW_SAVE_BAUD
SLOW_BAUD = 4800		# equal to it, so the timer stays off
IDLE_S = 6.0			# past the chip's 5 s
PATTERN = b"ft260-wakeup-pattern\n"
ACTIVATE = "activate wakeup workaround"
DEACTIVATE = "deactivate wakeup workaround"
STATE_S = 2.0			# how long to wait for a state to show up


def _last_state(log):
	"""activate or deactivate, whichever the driver logged last.

	DEACTIVATE holds ACTIVATE as a substring, so it is tested first.
	"""
	seen = [line for line in log.splitlines()
		if ACTIVATE in line or DEACTIVATE in line]
	if not seen:
		return None
	return DEACTIVATE if DEACTIVATE in seen[-1] else ACTIVATE


def _state_since(marker, want, timeout=STATE_S):
	"""The last state the driver logged, with the log for the message.

	Writes a marker of its own on every pass, because the driver's
	lines can stay out of dmesg until something else is printed.
	"""
	deadline = time.monotonic() + timeout
	while True:
		ftlib.mark("flush")
		log = ftlib.dmesg_since(marker)
		state = _last_state(log)
		if state == want or time.monotonic() > deadline:
			return state, log
		time.sleep(0.1)


@pytest.fixture
def power_saving(chip):
	if ftlib.read_attr(chip.uart, "power_saving_en") != "1":
		pytest.skip("the chip has power saving disabled")
	if chip.tty is None:
		pytest.skip("no tty on this chip")
	return chip.tty


def test_fast_baud_arms_the_timer(power_saving):
	marker = ftlib.mark("wakeup-fast")

	with serial.Serial(power_saving, FAST_BAUD, timeout=1):
		state, log = _state_since(marker, ACTIVATE)

	assert state == ACTIVATE, log


def test_slow_baud_leaves_it_off(power_saving):
	marker = ftlib.mark("wakeup-slow")

	with serial.Serial(power_saving, SLOW_BAUD, timeout=1):
		state, log = _state_since(marker, DEACTIVATE)

	assert state == DEACTIVATE, log


def test_close_disarms_the_timer(power_saving):
	with serial.Serial(power_saving, FAST_BAUD, timeout=1):
		marker = ftlib.mark("wakeup-close")

	state, log = _state_since(marker, DEACTIVATE)

	assert state == DEACTIVATE, log


def test_rx_survives_an_idle(chip, peer_link, power_saving):
	"""Data arrives after an idle, which is what the timer is for.

	There is no negative case. Below 4800 baud the chip keeps the
	data without the workaround, and above it the driver always
	arms the timer, so the data loss cannot be provoked from
	userspace. This test shows the path works. It does not
	reproduce the defect.
	"""
	with serial.Serial(power_saving, FAST_BAUD, timeout=2) as ft, \
	     serial.Serial(peer_link, FAST_BAUD, timeout=2) as other:
		ft.reset_input_buffer()
		time.sleep(IDLE_S)

		other.write(PATTERN)
		other.flush()

		assert ft.read(len(PATTERN)) == PATTERN
