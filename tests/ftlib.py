# SPDX-License-Identifier: MIT
"""Helpers shared by the hid-ft260 hardware tests.

Everything here needs root: the tests write sysfs attributes, load and
unload the module, and read the kernel log.
"""

import os
import re
import subprocess
import time

DRV = "/sys/bus/hid/drivers/ft260"
GPIO_SYSFS = "/sys/class/gpio"
# What proc_show writes, named after tty_driver->driver_name.
PROC_TTY = "/proc/tty/driver/ft260_ser"
# 24c02, one byte word address, holds the FT260 configuration. On both
# boards here.
CONFIG_EEPROM = "0x50"
# 24LC512, two byte word address, no FT260 configuration. One board.
BIG_EEPROM = "0x51"
GPIO_TOTAL = 14
GPIO_2_BIT = 0x0004		# FT260_GPIO_2, DIO7, pin 14


def read_attr(node, name):
	with open(os.path.join(node, name)) as f:
		return f.read().strip()


def write_attr(node, name, value):
	with open(os.path.join(node, name), "w") as f:
		f.write(str(value))


def _usb_path(node):
	"""USB device directory of a HID interface, the key for one chip."""
	return os.path.dirname(os.path.dirname(os.path.realpath(node)))


def _hid_nodes():
	if not os.path.isdir(DRV):
		return []
	nodes = [os.path.join(DRV, n) for n in sorted(os.listdir(DRV))]
	return [n for n in nodes
		if os.path.exists(os.path.join(n, "report_descriptor"))]


def find_devices():
	"""USB device paths of the bound FT260s, in a stable order.

	Every other finder takes one of these as its usb argument. Without
	it they answer for the first device they meet.
	"""
	return sorted({_usb_path(n) for n in _hid_nodes()})


def find_nodes(usb):
	"""Every HID interface of one chip. Mode 1 and 2 have only one."""
	return sorted(n for n in _hid_nodes() if _usb_path(n) == usb)


def find_node(attr, usb=None):
	"""The HID interface that carries attr, or None.

	The UART interface has uart_mode, the I2C one hid_over_i2c_en.
	"""
	for node in _hid_nodes():
		if not os.path.exists(os.path.join(node, attr)):
			continue
		if usb is None or _usb_path(node) == usb:
			return node
	return None


def _gpiochips():
	"""(base, label) of every gpiochip, lowest line number first."""
	out = []
	for chip in os.listdir(GPIO_SYSFS):
		if not chip.startswith("gpiochip"):
			continue
		try:
			with open(os.path.join(GPIO_SYSFS, chip, "label")) as f:
				out.append((int(chip[len("gpiochip"):]),
					    f.read().strip()))
		except OSError:
			continue
	return sorted(out)


def find_gpio_base(node=None):
	"""First line number of an ft260 gpiochip, or None.

	With a HID interface, the one it registered: the label is
	"ft260_" followed by that interface's name.
	"""
	want = "ft260" if node is None else "ft260_" + os.path.basename(node)
	for base, label in _gpiochips():
		if label.startswith(want):
			return base
	return None


def find_gpio_bases():
	"""Line numbers of every ft260 gpiochip, for cleanup."""
	return [base for base, label in _gpiochips() if label.startswith("ft260")]


def find_gpio_owner(usb=None):
	"""The HID interface the gpiochip hangs off, or None.

	That interface is the one whose gpio_en mask ft260_gpio_set()
	enforces.
	"""
	for node in _hid_nodes():
		if usb is not None and _usb_path(node) != usb:
			continue
		if any(e.startswith("gpiochip") for e in os.listdir(node)):
			return node
	return None


def find_i2c_bus(usb=None):
	"""Bus number of the adapter the driver registered, or None."""
	node = find_node("hid_over_i2c_en", usb)
	if not node:
		return None
	for name in os.listdir(node):
		if name.startswith("i2c-"):
			return int(name[len("i2c-"):])
	return None


def i2c_read(bus, addr, length):
	"""One i2ctransfer read from register 0 of addr."""
	return subprocess.run(
		["i2ctransfer", "-y", "-f", str(bus),
		 f"w1@{addr}", "0x00", f"r{length}"],
		capture_output=True, text=True)


def i2c_read_at(bus, addr, offset, length):
	"""One i2ctransfer read from a two-byte offset."""
	return subprocess.run(
		["i2ctransfer", "-y", "-f", str(bus), f"w2@{addr}",
		 f"0x{offset >> 8:02x}", f"0x{offset & 0xff:02x}",
		 f"r{length}"],
		capture_output=True, text=True)


def i2c_write(bus, addr, offset, data):
	"""One i2ctransfer write of data at a two-byte offset."""
	payload = [f"0x{offset >> 8:02x}", f"0x{offset & 0xff:02x}"]
	payload += [f"0x{b:02x}" for b in data]
	return subprocess.run(
		["i2ctransfer", "-y", "-f", str(bus),
		 f"w{len(payload)}@{addr}"] + payload,
		capture_output=True, text=True)


def i2cget(bus, addr, offset=None, mode=None, length=None):
	"""One i2cget call, which goes through ft260_smbus_xfer().

	mode names the SMBus size: b byte data, w word data, s block
	data, i i2c block data. Without an offset i2cget does a receive
	byte, which reads from wherever the slave's pointer stands.
	length is the byte count of an i2c block read.
	"""
	cmd = ["i2cget", "-y", "-f", str(bus), str(addr)]
	if offset is not None:
		cmd.append(f"0x{offset:02x}")
	if mode:
		cmd.append(mode)
	if length is not None:
		cmd.append(str(length))
	return subprocess.run(cmd, capture_output=True, text=True)


def i2cset(bus, addr, offset, *values, mode=None):
	"""One i2cset call, the write side of ft260_smbus_xfer().

	mode c sends the offset alone, b one byte, w one word low byte
	first, i a block of bytes.
	"""
	cmd = ["i2cset", "-y", "-f", str(bus), str(addr), f"0x{offset:02x}"]
	cmd += [f"0x{value:x}" for value in values]
	if mode:
		cmd.append(mode)
	return subprocess.run(cmd, capture_output=True, text=True)


def hex_bytes(text):
	"""The 0xNN words of i2ctransfer or i2cget output, as ints."""
	return [int(word, 16) for word in text.split()]


def find_eeprom(bus, candidates=("0x51", "0x50")):
	"""First address on bus that answers a read, or None.

	The two boards here carry the EEPROM at different addresses, so
	no test may hardcode one. Reads only.
	"""
	for addr in candidates:
		if i2c_read(bus, addr, 1).returncode == 0:
			return addr
	return None


def gpio2_bit(node):
	"""GPIO_2_BIT when the chip hands pin 14 to the gpiochip, else 0.

	Every mask this chip logs carries that bit, because probe adds it
	from the stored gpio2_func. The two boards here differ in it, so
	no test may hardcode a mask without asking.
	"""
	return GPIO_2_BIT if read_attr(node, "gpio2_func") == "0" else 0


def find_tty(node):
	"""Device path of the tty this interface registered, or None.

	node is None on a chip with no UART interface, chip mode 0x01.
	"""
	if node is None:
		return None
	try:
		names = os.listdir(os.path.join(node, "tty"))
	except OSError:
		return None
	return os.path.join("/dev", names[0]) if names else None


def tty_index(path):
	"""Minor index of /dev/ttyFTn, the number proc_show prints."""
	return int(re.search(r"(\d+)$", path).group(1))


def proc_counts(index):
	"""tx and rx of one port, from /proc/tty/driver/ft260_ser.

	The counters live in struct ft260_device and survive a close, so
	a test has to compare two reads, not one absolute value.
	"""
	with open(PROC_TTY) as f:
		for line in f:
			if line.startswith(f"{index}: "):
				found = dict(re.findall(r"(\w+):(\d+)", line))
				return int(found["tx"]), int(found["rx"])
	return None


def mark(tag):
	"""Write a unique marker into the kernel log and return it.

	Reading from the marker on survives ring buffer drops; an absolute
	dmesg line offset does not, and with debug=1 the buffer does drop
	records mid-run.
	"""
	marker = f"ft260-test-{os.getpid()}-{tag}"
	with open("/dev/kmsg", "w") as f:
		f.write(marker)
	return marker


def dmesg():
	return subprocess.run(["dmesg"], capture_output=True, text=True,
			      check=True).stdout


def dmesg_since(marker):
	"""The kernel log from marker to the end, marker line included."""
	log = dmesg()
	pos = log.rfind(marker)
	return log[pos:] if pos >= 0 else ""


def gpio_en(text):
	"""Last mask the driver logged in text, without the bitmap tail."""
	found = re.findall(r"enabled GPIOs: ([0-9a-f]+)", text)
	return found[-1] if found else None


def gpio_en_since(marker):
	return gpio_en(dmesg_since(marker))


def mask_of(dev, marker):
	"""Last mask logged by one device since marker, None if it logged none.

	dev is the HID device name, for example 0003:0403:6030.0007. The
	line reads "ft260 <dev>: enabled GPIOs: 2cb0, bitmap 0003".
	"""
	lines = [l for l in dmesg_since(marker).splitlines()
		 if f"{dev}: enabled GPIOs" in l]
	return gpio_en(lines[-1]) if lines else None


def module_loaded():
	with open("/proc/modules") as f:
		return any(line.startswith("hid_ft260 ") for line in f)


def export(line):
	with open(os.path.join(GPIO_SYSFS, "export"), "w") as f:
		f.write(str(line))


def unexport(line):
	with open(os.path.join(GPIO_SYSFS, "unexport"), "w") as f:
		f.write(str(line))


def unexport_all(base):
	"""Release every line of the chip. A line left exported pins the
	module and makes the next run fail to unload it."""
	if base is None:
		return
	for offset in range(GPIO_TOTAL):
		path = os.path.join(GPIO_SYSFS, f"gpio{base + offset}")
		if os.path.isdir(path):
			try:
				unexport(base + offset)
			except OSError:
				pass


def line_path(line, name):
	return os.path.join(GPIO_SYSFS, f"gpio{line}", name)


def set_direction(line, direction):
	with open(line_path(line, "direction"), "w") as f:
		f.write(direction)


def set_value(line, value):
	with open(line_path(line, "value"), "w") as f:
		f.write(str(value))


def get_value(line):
	with open(line_path(line, "value")) as f:
		return f.read().strip()


def line_state(line):
	"""Whether the gpiochip lets the line be claimed and driven.

	Returns "usable", "direction refused" or "export refused". The line
	is given back before returning.
	"""
	try:
		export(line)
	except OSError:
		return "export refused"
	try:
		set_direction(line, "out")
		state = "usable"
	except OSError:
		state = "direction refused"
	try:
		set_direction(line, "in")
	except OSError:
		pass
	unexport(line)
	return state


def line_errno(line, direction="out"):
	"""errno from setting the direction, or None when it works.

	ft260_gpio_direction_set() returns -EIO for a line outside
	gpio_en, ft260_gpio_set() returns -EINVAL. It tests gpio_en before
	it looks at the direction, so "in" gives the same answer without
	driving the pin. The line is given back before returning.
	"""
	export(line)
	err = None
	try:
		set_direction(line, direction)
	except OSError as exc:
		err = exc.errno
	try:
		set_direction(line, "in")
	except OSError:
		pass
	unexport(line)
	return err


def insmod(ko, settle=2.0):
	subprocess.run(["insmod", ko, "debug=1"], check=True)
	time.sleep(settle)


def rmmod():
	subprocess.run(["rmmod", "hid-ft260"], check=True)


def reload(ko):
	"""Unload and load the module, return the log of the new probe."""
	marker = mark("reload")
	if module_loaded():
		for base in find_gpio_bases():
			unexport_all(base)
		rmmod()
	insmod(ko)
	return dmesg_since(marker)


def rebind(node, settle=2.0):
	"""Re-probe one HID interface, return the log of that probe.

	Only this interface is touched, so the log window holds no lines
	from a second chip. ft260_get_interface_type() prints no device
	name, so that is the only way to tell the two apart.
	"""
	name = os.path.basename(node)
	marker = mark("rebind")
	write_attr(DRV, "unbind", name)
	write_attr(DRV, "bind", name)
	time.sleep(settle)
	return dmesg_since(marker)


def gpio_en_now(node):
	"""Last mask the driver logged for this interface, or None.

	The driver logs the mask on every change, so the last line is the
	current value as long as the log still holds it.
	"""
	dev = os.path.basename(node)
	lines = [l for l in dmesg().splitlines()
		 if f"{dev}: enabled GPIOs" in l]
	return gpio_en(lines[-1]) if lines else None


def probed_uart_mode(log):
	"""Flow control mode the probe read out of the chip, as an int.

	The driver logs "get_interface_type: uart_mode:  0x03" with
	debug=1, before ft260_uart_probe() overwrites the chip setting.
	"""
	found = re.findall(r"get_interface_type: uart_mode: *0x([0-9a-f]+)", log)
	return int(found[-1], 16) if found else None


def find_peer_chip(chips, peer, baud=9600, timeout=2.0):
	"""The chip whose tty receives a byte sent from peer, or None.

	Needs pyserial. Imported here so the rest of the helpers stay
	usable without it.
	"""
	import serial

	msg = b"ft260-which-chip\n"
	ports = {}
	try:
		for chip in chips:
			if chip.tty:
				ports[chip] = serial.Serial(chip.tty, baud,
							    timeout=timeout)
				ports[chip].reset_input_buffer()
		with serial.Serial(peer, baud, timeout=timeout) as line:
			line.write(msg)
			line.flush()
		for chip, port in ports.items():
			if port.read(len(msg)) == msg:
				return chip
	finally:
		for port in ports.values():
			port.close()
	return None
