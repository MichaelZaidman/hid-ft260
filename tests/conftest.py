# SPDX-License-Identifier: MIT
"""Session setup for the hid-ft260 hardware tests.

Run as root, with the module built in the parent directory:

    sudo .venv/bin/pytest -v
"""

import hashlib
import os
import shutil
import time

import pytest

import ftlib

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_KO = os.path.join(os.path.dirname(HERE), "hid-ft260.ko")
# Under sudo, ~ is /root, so ask for the home of the user who invoked it.
HOME = os.path.expanduser("~" + os.environ.get("SUDO_USER", ""))
DEFAULT_PATTERN = os.path.join(HOME, "sw/uart-test/pattern")
DEFAULT_PEER = "/dev/ttyUSB0"		# FT2232 channel wired to one ft260
DEFAULT_I2CPERF = os.path.join(HOME, "sw/i2cperf/i2cperf")


def pytest_addoption(parser):
	parser.addoption("--ko", default=DEFAULT_KO,
			 help=f"module to test, default {DEFAULT_KO}")
	parser.addoption("--pattern", default=DEFAULT_PATTERN,
			 help=f"file to send over the UART, default {DEFAULT_PATTERN}")
	parser.addoption("--bytes", type=int, default=512 * 1024,
			 help="how much of it to send, 0 for all")
	parser.addoption("--peer", default=DEFAULT_PEER,
			 help=f"tty wired to an ft260 UART, default {DEFAULT_PEER}")
	parser.addoption("--chip", default=None,
			 help="tty of the chip to test, default: ask the peer")
	parser.addoption("--eeprom", default=None,
			 help="i2c address to read, default: probe 0x51 then 0x50")
	parser.addoption("--i2cperf", default=DEFAULT_I2CPERF,
			 help=f"throughput tool, default {DEFAULT_I2CPERF}")


def pytest_report_header(config):
	"""Name the module build and the kernel at the top of every run.

	srcversion is a hash of the source only, so two builds of one tree
	share it. The sha256 of the file tells them apart.
	"""
	ko = os.path.abspath(config.getoption("--ko"))
	lines = []
	if os.path.isfile(ko):
		with open(ko, "rb") as f:
			digest = hashlib.sha256(f.read()).hexdigest()
		st = os.stat(ko)
		mtime = time.strftime("%Y-%m-%d %H:%M:%S %z",
				      time.localtime(st.st_mtime))
		lines.append(f"module: {ko}, sha256: {digest}, "
			     f"size: {st.st_size}, mtime: {mtime}")
		lines.append(f"module srcversion: {ftlib.modinfo(ko, 'srcversion')}, "
			     f"vermagic: {ftlib.modinfo(ko, 'vermagic')}")
	else:
		lines.append(f"module: {ko} not found")
	with open("/proc/cmdline") as f:
		cmdline = f.read().strip()
	lines.append(f"kernel: {os.uname().release}, cmdline: {cmdline}")
	return lines


class Chip:
	"""Where one device is, looked up on every access.

	Keyed by the USB device directory, so two chips never answer for
	each other. The paths and the gpiochip base are read live, because
	a test that reloads the module invalidates whatever was cached
	before it.
	"""

	def __init__(self, usb):
		self.usb = usb

	@property
	def uart(self):
		"""HID interface carrying uart_mode."""
		return ftlib.find_node("uart_mode", self.usb)

	@property
	def i2c(self):
		"""HID interface carrying hid_over_i2c_en."""
		return ftlib.find_node("hid_over_i2c_en", self.usb)

	@property
	def base(self):
		return ftlib.find_gpio_base(ftlib.find_gpio_owner(self.usb))

	@property
	def bus(self):
		return ftlib.find_i2c_bus(self.usb)

	@property
	def tty(self):
		return ftlib.find_tty(self.uart)

	@property
	def node(self):
		"""Any interface of this chip. Mode 1 and 2 have only one."""
		return self.uart or self.i2c

	@property
	def chip_mode(self):
		return ftlib.read_attr(self.node, "chip_mode")


@pytest.fixture(scope="session")
def ko(request):
	path = request.config.getoption("--ko")
	if not os.path.isfile(path):
		pytest.skip(f"no module at {path}")
	return path


@pytest.fixture(scope="session")
def chips(ko):
	"""Load the module under test and describe every device found.

	Leaves the module loaded after the run, so a failed run still
	leaves the board usable.
	"""
	if os.geteuid() != 0:
		pytest.skip("needs root")
	if ftlib.module_loaded():
		for base in ftlib.find_gpio_bases():
			ftlib.unexport_all(base)
		ftlib.rmmod()
	ftlib.insmod(ko)

	want = ftlib.modinfo(ko, "srcversion")
	got = ftlib.loaded_srcversion()
	if got != want:
		pytest.exit(f"loaded hid_ft260 has srcversion {got}, "
			    f"{ko} has {want}", returncode=1)

	found = [Chip(usb) for usb in ftlib.find_devices()]
	if not found:
		pytest.skip("no ft260 bound")

	yield found

	for base in ftlib.find_gpio_bases():
		ftlib.unexport_all(base)


@pytest.fixture(scope="session")
def peer(request):
	"""The tty on the other end of the UART cable, or None."""
	path = request.config.getoption("--peer")
	return path if path and os.path.exists(path) else None


@pytest.fixture(scope="session")
def wired(chips, peer):
	"""The chip the peer is cabled to, or None.

	Neither the USB path nor the tty index is pinned to a board: both
	follow probe order and both changed across a reboot here. Sending
	a marker from the peer is the only way to tell.
	"""
	if peer is None:
		return None
	return ftlib.find_peer_chip(chips, peer)


@pytest.fixture(scope="session")
def chip(request, chips, wired):
	"""The device under test, named by --chip or the one on the cable."""
	named = request.config.getoption("--chip")
	if named:
		found = [c for c in chips if c.tty == named]
		if not found:
			pytest.skip(f"no chip on {named}")
		picked = found[0]
	else:
		picked = wired or chips[0]

	if not picked.uart or not picked.i2c or picked.base is None:
		pytest.skip("ft260 sysfs nodes are missing")
	return picked


@pytest.fixture(scope="session")
def uart_only_chip(chips):
	"""A chip strapped to chip mode 0x02: one interface, UART, no I2C."""
	for one in chips:
		if one.uart and ftlib.read_attr(one.uart, "chip_mode") == "2":
			return one
	pytest.skip("no chip strapped to chip mode 0x02")


@pytest.fixture(scope="session")
def mode_all_chip(chips):
	"""A chip strapped to chip mode 0x00, FT260_MODE_ALL."""
	for one in chips:
		if one.uart and ftlib.read_attr(one.uart, "chip_mode") == "0":
			return one
	pytest.skip("no chip strapped to chip mode 0x00")


@pytest.fixture(scope="session")
def i2c_only_chip(chips):
	"""A chip strapped to chip mode 0x01: one interface, I2C, no tty."""
	for one in chips:
		# In mode 0x00 and 0x03 the I2C interface has no chip_mode:
		# it is in the mode 1 group only.
		if one.i2c and os.path.exists(os.path.join(one.i2c, "chip_mode")) \
		   and ftlib.read_attr(one.i2c, "chip_mode") == "1":
			return one
	pytest.skip("no chip strapped to chip mode 0x01")


@pytest.fixture(scope="session")
def eeprom(request, chip):
	"""Address of an i2c device that answers on this chip's bus."""
	if chip.bus is None or not os.path.exists(f"/dev/i2c-{chip.bus}"):
		pytest.skip("no ft260 i2c bus")
	if not shutil.which("i2ctransfer"):
		pytest.skip("i2ctransfer not installed")
	named = request.config.getoption("--eeprom")
	addr = named or ftlib.find_eeprom(chip.bus)
	if addr is None:
		pytest.skip(f"nothing answers on i2c-{chip.bus}")
	return addr


@pytest.fixture(scope="module")
def config_eeprom(chip):
	"""First 64 bytes of the EEPROM at 0x50, to pick offsets from.

	Its word address is one byte, which is what an SMBus command
	byte carries. What is stored differs per board, so no test may
	name a value.
	"""
	if chip.bus is None:
		pytest.skip("no ft260 i2c bus")
	# A bus left stuck by an earlier run eats the first transfer:
	# that is the one which reaches ft260_i2c_abort() and recovers.
	done = ftlib.i2c_read(chip.bus, ftlib.CONFIG_EEPROM, 64)
	if done.returncode != 0:
		done = ftlib.i2c_read(chip.bus, ftlib.CONFIG_EEPROM, 64)
	if done.returncode != 0:
		pytest.skip(f"nothing at {ftlib.CONFIG_EEPROM} on i2c-{chip.bus}")
	return ftlib.hex_bytes(done.stdout)


@pytest.fixture(scope="session")
def i2cperf(request):
	"""The throughput tool, or a skip when it is not installed."""
	path = request.config.getoption("--i2cperf")
	if not os.access(path, os.X_OK):
		pytest.skip(f"no i2cperf at {path}")
	return path


@pytest.fixture
def big_eeprom(eeprom):
	"""The 24LC512 at 0x51, the part tests may write.

	It takes the two-byte offset i2c_read_at() sends, answers at
	3400 kHz and holds no FT260 configuration. The 24c02 at 0x50
	does none of the three.
	"""
	if eeprom != ftlib.BIG_EEPROM:
		pytest.skip(f"no 24LC512 on this bus, found {eeprom}")
	return eeprom


@pytest.fixture
def peer_link(chip, wired, peer):
	"""The peer path, for tests that need the cable on this chip."""
	if peer is None:
		pytest.skip("no peer")
	if wired is None or wired.usb != chip.usb:
		pytest.skip(f"the peer is not cabled to {chip.tty}")
	return peer


@pytest.fixture
def exported(chip):
	"""Export a line by offset and release it afterwards."""
	taken = []

	def _export(offset):
		line = chip.base + offset
		ftlib.export(line)
		taken.append(line)
		return line

	yield _export

	for line in taken:
		try:
			ftlib.unexport(line)
		except OSError:
			pass
