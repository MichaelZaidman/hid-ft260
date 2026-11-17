# SPDX-License-Identifier: MIT
"""Open the UART tty across an unbind of its HID interface.

ft260_remove() hangs up the tty, but a file that stays open keeps the
tty_struct, and the port with it, until the last close. Two things
must not reach the removed port:

1. A reopen of the hung-up tty through /proc/self/fd. tty_reopen()
   checks only tty->count, so the driver must refuse it in
   ft260_uart_port_activate().
2. An open of the tty that a new probe registers. The new port must
   not get the index of the old one while the old tty is still in
   the driver's ttys[] table, or the open finds the old tty.

An unbind leaves the hid_device in place, so these tests see the
wrong open itself, with or without KASAN.
"""

import errno
import os
import time

import pytest
import serial

import ftlib

BAUD = 9600
PATTERN = b"ft260-unbind-open-pattern\n"
COMPLAINTS = ("WARNING", "BUG:", "Call Trace", "refcount", "use-after",
	      "Oops", "KASAN")
RELEASE_S = 1.0			# release_one_tty() runs from a work item


def _complaints(marker):
	return [line for line in ftlib.dmesg_since(marker).splitlines()
		if any(word in line for word in COMPLAINTS)]


@pytest.fixture
def uart(chip):
	"""The UART interface, its name and its tty, bound again after."""
	node = chip.uart
	tty = ftlib.find_tty(node)
	if tty is None:
		pytest.skip("no tty on this chip")
	name = os.path.basename(node)

	yield node, name, tty

	if not os.path.exists(os.path.join(ftlib.DRV, name)):
		ftlib.write_attr(ftlib.DRV, "bind", name)
		time.sleep(2.0)


def test_reopen_after_unbind_fails(uart):
	node, name, tty = uart
	marker = ftlib.mark("uart-reopen")
	fd = os.open(tty, os.O_RDWR | os.O_NOCTTY)
	try:
		ftlib.write_attr(ftlib.DRV, "unbind", name)
		with pytest.raises(OSError) as err:
			again = os.open(f"/proc/self/fd/{fd}",
					os.O_RDWR | os.O_NOCTTY)
			os.close(again)
	finally:
		os.close(fd)

	assert err.value.errno == errno.ENODEV
	assert _complaints(marker) == []


def test_rebind_with_open_fd_takes_a_new_index(uart):
	node, name, tty = uart
	old = ftlib.tty_index(tty)
	marker = ftlib.mark("uart-rebind-index")
	fd = os.open(tty, os.O_RDWR | os.O_NOCTTY)
	try:
		ftlib.rebind(node)
		new_tty = ftlib.find_tty(node)
		assert new_tty is not None, "no tty after the rebind"
		assert ftlib.tty_index(new_tty) != old
		with serial.Serial(new_tty, BAUD, timeout=1) as port:
			assert port.write(PATTERN) == len(PATTERN)
	finally:
		os.close(fd)

	time.sleep(RELEASE_S)
	ftlib.rebind(node)
	assert ftlib.find_tty(node) == tty, "old index not free after close"
	assert _complaints(marker) == []
