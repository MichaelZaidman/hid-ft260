# SPDX-License-Identifier: MIT
"""Unbind the gpiochip owner while a thread writes a GPIO line.

A devm gpiochip stays registered until after ft260_remove() returns,
and ft260_remove() frees the driver data first. A write that lands in
that window uses freed memory. The window is short, so each cycle races
a writer against the unbind and the test repeats it. With KASAN the
use-after-free is reported in the kernel log; without KASAN a cycle can
pass on a broken driver.
"""

import os
import threading
import time

import pytest

import ftlib

CYCLES = 20
GPIO2 = 2			# DIO7, pin 14
COMPLAINTS = ("WARNING", "BUG:", "Call Trace", "refcount", "use-after",
	      "Oops", "KASAN")


class Writer(threading.Thread):
	"""Toggle one line through a single open value file until stopped.

	The file stays open across the unbind, so writes keep reaching
	gpiolib after the line is gone. Errors are expected then and only
	counted.
	"""

	def __init__(self, line):
		super().__init__(daemon=True)
		self.fd = os.open(ftlib.line_path(line, "value"), os.O_WRONLY)
		self.stop = threading.Event()
		self.started = threading.Event()
		self.ok = 0
		self.failed = 0

	def run(self):
		value = 0
		try:
			while not self.stop.is_set():
				value ^= 1
				try:
					os.pwrite(self.fd, str(value).encode(), 0)
					self.ok += 1
					self.started.set()
				except OSError:
					self.failed += 1
		finally:
			os.close(self.fd)


def drive_gpio2(chip):
	ftlib.write_attr(chip.uart, "gpio2_func", 0)
	line = chip.base + GPIO2
	ftlib.export(line)
	ftlib.set_direction(line, "out")
	return line


@pytest.fixture(scope="module")
def owner(chip):
	"""The interface the gpiochip hangs off, with gpio2_func restored."""
	orig = ftlib.read_attr(chip.uart, "gpio2_func")
	node = ftlib.find_gpio_owner(chip.usb)
	if node is None:
		pytest.skip("no ft260 gpiochip")

	yield node

	name = os.path.basename(node)
	if not os.path.exists(os.path.join(ftlib.DRV, name)):
		ftlib.write_attr(ftlib.DRV, "bind", name)
		time.sleep(2.0)
	ftlib.unexport_all(chip.base)
	ftlib.write_attr(chip.uart, "gpio2_func", orig)


def test_unbind_during_gpio_writes_is_quiet(chip, owner):
	name = os.path.basename(owner)
	for cycle in range(CYCLES):
		writer = Writer(drive_gpio2(chip))
		writer.start()
		assert writer.started.wait(5.0), \
			f"cycle {cycle}: no write succeeded before the unbind"

		marker = ftlib.mark(f"gpio-unbind-{cycle}")
		ftlib.write_attr(ftlib.DRV, "unbind", name)
		writer.stop.set()
		writer.join(5.0)
		assert not writer.is_alive(), f"cycle {cycle}: writer hung"

		ftlib.write_attr(ftlib.DRV, "bind", name)
		time.sleep(2.0)

		log = ftlib.dmesg_since(marker)
		hits = [line for line in log.splitlines()
			if any(word in line for word in COMPLAINTS)]
		assert hits == [], f"cycle {cycle}, {writer.ok} writes ok"
		assert chip.base is not None, f"cycle {cycle}: gpiochip not back"
