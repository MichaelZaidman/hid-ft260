# SPDX-License-Identifier: MIT
"""TN_189 reset work against a probe of the same chip.

When a probe fails the TN_189 checks, it queues a work item that resets
the USB device and rebinds its interfaces. A probe that runs outside
hub_event, as on insmod with the board attached, holds only the HID
device lock. The reset then runs usbhid's pre_reset while the failing
probe, or the probe of the sibling interface, starts or stops the HID
hardware.

This needs a module built with the TN_189 test harness: the parameters
fake_bus, fake_dev, fake_ifmask and fake_delay_ms. The next probe of an
interface whose bit is set in fake_ifmask fails as if the chip did not
answer, queues the reset and sleeps fake_delay_ms before it stops the
HID hardware. The reset work logs "harness: USB reset, N probes run"
and, when N is not 0, "harness: USB reset while N probes run". The
real STALL cannot be caused on purpose, so this is the only way to run
the recovery path on a healthy chip.

The first test loads the module with the board attached. The second
deauthorizes and authorizes the board, so the probes run under the USB
device lock, as in hub_event. Both fail on an overlap line or on a
kernel complaint, and both need the board back afterwards.

The second test also runs with both interfaces faked, as on a real
STALL. Both probes then queue a reset, and the driver must reset the
chip only once: a second reset tears down the chip that the first one
has recovered. Every case expects exactly one "harness: USB reset"
line. The first test fakes only one interface: on insmod the reset work
unbinds the sibling before its first probe, so a faked failure of the
sibling comes in the next enumeration and correctly queues a new reset.

Setup: a chip with both interfaces, chip mode 0x00 or 0x03, as the chip
fixture picks it. The test reads busnum and devnum of that chip's USB
device from sysfs, so only this chip is faked and counted; a second
board on the bus is left alone. The faked interface is the first HID
interface of the chip in sorted order, so its sibling probes after it.
The test is skipped when modinfo of the --ko module has no fake_ifmask
parameter.

Build the harness module from this repo, in a directory of its own,
for the running kernel. tn189_harness.py adds the harness to a copy of
hid-ft260.c; the driver in the repo stays as it is. Run make inside
that directory: the Makefile builds M=$(PWD).

    mkdir /tmp/ft260-harness
    python3 tn189_harness.py ../hid-ft260.c /tmp/ft260-harness/hid-ft260.c
    cp ../hid-ids.h ../Makefile /tmp/ft260-harness/
    (cd /tmp/ft260-harness && make)
    sudo .venv/bin/pytest -v --ko=/tmp/ft260-harness/hid-ft260.ko \\
        test_tn189_reset_race.py

tn189_harness.py also takes drivers/hid/hid-ft260.c of a kernel tree,
for a module of that tree.
"""

import os
import subprocess
import time

import pytest

import ftlib

DELAY_MS = 500
RECOVER_S = 20.0
# After recovery, time for a second reset to log its line
SETTLE_S = {"mask": 2.0, "both": 5.0}
PARAMS = "/sys/module/hid_ft260/parameters"
COMPLAINTS = ("WARNING", "BUG:", "Call Trace", "refcount", "use-after",
	      "Oops", "KASAN")
FAKED = "harness: fake TN_189 failure"
RESET = "harness: USB reset, "
OVERLAP = "harness: USB reset while"


def usb_attr(usb, name):
	return int(ftlib.read_attr(usb, name))


def iface_number(node):
	"""bInterfaceNumber of the USB interface a HID node hangs off."""
	usbif = os.path.dirname(os.path.realpath(node))
	return int(ftlib.read_attr(usbif, "bInterfaceNumber"), 16)


@pytest.fixture(scope="module")
def harness(chip, ko):
	"""Bus, device and two interface masks.

	"mask" is the interface that probes first. The HID devices of a chip
	probe in the order they were added, and their names end in an
	increasing id, so the first one in sorted order is the one whose
	sibling probes after it. "both" is every HID interface of the chip.
	"""
	parms = subprocess.run(["modinfo", "-F", "parm", ko],
			       capture_output=True, text=True).stdout
	if "fake_ifmask" not in parms:
		pytest.skip("module built without the TN_189 test harness")

	nodes = ftlib.find_nodes(chip.usb)
	found = {
		"bus": usb_attr(chip.usb, "busnum"),
		"dev": usb_attr(chip.usb, "devnum"),
		"mask": 1 << iface_number(nodes[0]),
		"both": sum(1 << iface_number(node) for node in nodes),
	}

	yield found

	if os.path.isdir(PARAMS):
		ftlib.write_attr(PARAMS, "fake_ifmask", 0)
		ftlib.write_attr(PARAMS, "fake_dev", 0)


def wait_recovered(chip, marker, settle):
	"""Wait for the reset line and for both interfaces of the chip."""
	deadline = time.monotonic() + RECOVER_S
	while time.monotonic() < deadline:
		log = ftlib.dmesg_since(marker)
		if RESET in log and chip.uart and chip.i2c and chip.tty:
			time.sleep(settle)
			return ftlib.dmesg_since(marker)
		time.sleep(0.5)
	return ftlib.dmesg_since(marker)


def count(log, text):
	return sum(1 for line in log.splitlines() if text in line)


def check(chip, log, faked):
	assert count(log, FAKED) == faked, "the harness did not fail the probes"
	assert count(log, RESET) == 1, "the chip was not reset exactly once"
	overlap = [line for line in log.splitlines() if OVERLAP in line]
	assert overlap == [], "USB reset while a probe of the chip ran"
	hits = [line for line in log.splitlines()
		if any(word in line for word in COMPLAINTS)]
	assert hits == []
	assert chip.uart, "UART interface not back"
	assert chip.i2c, "I2C interface not back"
	assert chip.tty and os.path.exists(chip.tty), "tty not back"
	assert chip.bus is not None, "I2C adapter not back"


def test_reset_during_insmod_probe(chip, ko, harness):
	mask = harness["mask"]
	for base in ftlib.find_gpio_bases():
		ftlib.unexport_all(base)
	marker = ftlib.mark("tn189-insmod")
	ftlib.rmmod()
	subprocess.run(["insmod", ko, "debug=1",
			f"fake_bus={harness['bus']}",
			f"fake_dev={harness['dev']}",
			f"fake_ifmask={mask}",
			f"fake_delay_ms={DELAY_MS}"], check=True)
	log = wait_recovered(chip, marker, SETTLE_S["mask"])
	check(chip, log, 1)


@pytest.mark.parametrize("which", ["mask", "both"])
def test_reset_during_locked_probe(chip, harness, which):
	mask = harness[which]
	ftlib.write_attr(PARAMS, "fake_bus", harness["bus"])
	ftlib.write_attr(PARAMS, "fake_dev", harness["dev"])
	ftlib.write_attr(PARAMS, "fake_delay_ms", DELAY_MS)
	ftlib.write_attr(PARAMS, "fake_ifmask", mask)
	for base in ftlib.find_gpio_bases():
		ftlib.unexport_all(base)
	marker = ftlib.mark("tn189-authorize")
	ftlib.write_attr(chip.usb, "authorized", 0)
	time.sleep(1.0)
	ftlib.write_attr(chip.usb, "authorized", 1)
	log = wait_recovered(chip, marker, SETTLE_S[which])
	check(chip, log, bin(mask).count("1"))
