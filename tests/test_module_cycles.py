# SPDX-License-Identifier: MIT
"""Unload and reload the driver, with and without a line in use.

Exporting GPIO11 (DTR) takes a reference on the module, so rmmod has to
be refused rather than warn. After that, five clean cycles must leave no
kernel complaint and must bring the device back.
"""

import subprocess

import pytest

import ftlib

CYCLES = 5
COMPLAINTS = ("WARNING", "BUG:", "Call Trace", "refcount", "use-after", "Oops")
GPIO_DTR = 11


def test_rmmod_refused_while_a_line_is_exported(chip, exported):
	exported(GPIO_DTR)
	done = subprocess.run(["rmmod", "hid-ft260"],
			      capture_output=True, text=True)
	assert done.returncode != 0, "rmmod succeeded with a line exported"
	assert "in use" in done.stderr


def test_five_cycles_are_quiet(chip, ko):
	marker = ftlib.mark("cycles")
	for _ in range(CYCLES):
		ftlib.rmmod()
		ftlib.insmod(ko)
	log = ftlib.dmesg_since(marker)
	hits = [line for line in log.splitlines()
		if any(word in line for word in COMPLAINTS)]
	assert hits == []


def test_device_is_back_after_the_cycles(chip):
	assert chip.uart
	assert chip.base is not None
