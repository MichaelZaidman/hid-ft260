# SPDX-License-Identifier: MIT
"""A failed UART output report must not leave bytes queued for ever.

ft260_uart_transmit_chars() takes up to 60 bytes from xmit_fifo per
output report. When a report fails, the bytes still in the fifo are
not sent, so chars_in_buffer() must not keep counting them:

1. tcdrain() after the error must return. It waits in
   tty_wait_until_sent() with no timeout while chars_in_buffer() is
   not zero.
2. An unbind while another process waits in TCSETSW must finish.
   That process holds a line discipline reference, and the hangup in
   ft260_remove() waits in tty_ldisc_lock() until the reference goes.

A real output report fails only when the USB device goes away. This
needs a module built with ft260_harness.py: its fake_tx_fail parameter
fails that many reports with -EIO. The tests are skipped when modinfo
of the --ko module has no fake_tx_fail parameter.

One write of PAYLOAD bytes, below XMIT_FIFO_SIZE, in raw mode reaches
the driver as one write(), so the tx work runs once and its first
report is the one that fails.
"""

import fcntl
import os
import signal
import struct
import subprocess
import sys
import termios
import time
import tty as ttymode

import pytest

import ftlib

PARAMS = "/sys/module/hid_ft260/parameters"
PAYLOAD = 1024
TIOCOUTQ = 0x5411
DRAIN_S = 5			# bound on tcdrain() after the error
UNBIND_S = 10			# bound on the unbind during TCSETSW
WAIT_S = 0.5			# for the child to enter TCSETSW
FAKED = "harness: fake TX failure"
COMPLAINTS = ("WARNING", "BUG:", "Call Trace", "refcount", "use-after",
	      "Oops", "KASAN", "blocked for more than")

CHILD = """
import os, sys, termios, tty
fd = os.open(sys.argv[1], os.O_RDWR | os.O_NOCTTY)
tty.setraw(fd)
os.write(fd, b"x" * int(sys.argv[2]))
attrs = termios.tcgetattr(fd)
print("draining", flush=True)
try:
	termios.tcsetattr(fd, termios.TCSADRAIN, attrs)
	print("drained", flush=True)
except OSError as err:
	print("tcsetattr", err.errno, flush=True)
"""


class Timeout(Exception):
	pass


def _alarm(signum, frame):
	raise Timeout()


def _outq(fd):
	return struct.unpack("i", fcntl.ioctl(fd, TIOCOUTQ,
					      struct.pack("i", 0)))[0]


def _complaints(marker):
	return [line for line in ftlib.dmesg_since(marker).splitlines()
		if any(word in line for word in COMPLAINTS)]


@pytest.fixture
def harness(request, chip):
	parms = subprocess.run(["modinfo", "-F", "parm",
				request.config.getoption("--ko")],
			       capture_output=True, text=True).stdout
	if "fake_tx_fail" not in parms:
		pytest.skip("module built without fake_tx_fail")
	if chip.tty is None:
		pytest.skip("no tty on this chip")

	yield

	ftlib.write_attr(PARAMS, "fake_tx_fail", 0)


@pytest.fixture
def uart(chip, harness):
	"""The UART interface name and its tty, bound again after."""
	node = chip.uart
	name = os.path.basename(node)

	yield name, ftlib.find_tty(node)

	if not os.path.exists(os.path.join(ftlib.DRV, name)):
		ftlib.write_attr(ftlib.DRV, "bind", name)
		time.sleep(2.0)


def test_tcdrain_returns_after_tx_error(uart):
	name, tty = uart
	marker = ftlib.mark("uart-tx-error-drain")
	fd = os.open(tty, os.O_RDWR | os.O_NOCTTY)
	old = signal.signal(signal.SIGALRM, _alarm)
	try:
		ttymode.setraw(fd)
		ftlib.write_attr(PARAMS, "fake_tx_fail", 1)
		assert os.write(fd, b"x" * PAYLOAD) == PAYLOAD
		signal.alarm(DRAIN_S)
		try:
			termios.tcdrain(fd)
			timed_out = False
		except Timeout:
			timed_out = True
		finally:
			signal.alarm(0)
		queued = _outq(fd)
	finally:
		signal.signal(signal.SIGALRM, old)
		os.close(fd)

	log = ftlib.dmesg_since(marker)
	assert log.count(FAKED) == 1
	assert not timed_out, f"tcdrain() still waits, {queued} bytes queued"
	assert queued == 0
	assert _complaints(marker) == []


def test_unbind_during_tcsetsw_after_tx_error(uart):
	name, tty = uart
	marker = ftlib.mark("uart-tx-error-unbind")
	ftlib.write_attr(PARAMS, "fake_tx_fail", 1)
	child = subprocess.Popen([sys.executable, "-c", CHILD, tty,
				  str(PAYLOAD)], stdout=subprocess.PIPE,
				 text=True)
	try:
		assert child.stdout.readline().strip() == "draining"
		time.sleep(WAIT_S)
		unbind = subprocess.Popen(["sh", "-c",
					   f"echo {name} > {ftlib.DRV}/unbind"])
		try:
			unbind.wait(UNBIND_S)
			hung = False
		except subprocess.TimeoutExpired:
			hung = True
			child.kill()
			unbind.wait()
	finally:
		child.kill()
		child.wait()

	log = ftlib.dmesg_since(marker)
	assert log.count(FAKED) == 1
	assert not hung, "unbind waited for the TCSETSW process"
	assert unbind.returncode == 0
	assert _complaints(marker) == []
