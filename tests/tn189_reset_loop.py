#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Reset the FT260 by its RESETN pin in a loop and check that it recovers.

TN_189: the FT260 interrupt endpoint can STALL after enumeration. The
driver detects it in probe and resets the USB device. A healthy chip
cannot be made to STALL, but a chip reset by its RESETN pin enumerates
again, and some of those enumerations STALL. Each cycle here does what
FTDI's loop does: RESETN high 200 ms, low 1500 ms, high, wait 1500 ms.
Then it waits for both HID interfaces, checks the UART against a peer
and reads the EEPROM.

A HW setup that can reset the FT260 drives RESETN, controlled by 'L'
and 'H' on its serial port (tn189_reset/tn189_reset.ino).

The module must be loaded before the run, or given with --ko. Run as
root, from this directory:

    sudo .venv/bin/python3 tn189_reset_loop.py \\
        --reset /dev/serial/by-path/<controller tty> --usb 2-3.4 \\
        --peer /dev/ttyUSB0 --cycles 1000

Every cycle prints one line, to the console and to --log. A cycle with
a STALL, a failure or a kernel complaint also saves its kernel log
next to --log. The run stops at the first failed cycle, unless
--keep-going is given.
"""

import argparse
import os
import subprocess
import sys
import time

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ftlib  # noqa: E402

STALL = "TN_189 errata"
COMPLAINTS = ("WARNING", "BUG:", "Call Trace", "refcount", "use-after",
	      "Oops", "KASAN", "circular locking", "blocked for more than")
SETTLE_S = 2.0


class Log:
	def __init__(self, path):
		self.path = path
		self.f = open(path, "a", buffering=1)

	def __call__(self, text):
		print(text, flush=True)
		self.f.write(text + "\n")


def command(port, letter):
	port.reset_input_buffer()
	port.write(letter.encode())
	port.flush()
	got = port.readline()
	if got != (letter + "\n").encode():
		raise RuntimeError(f"controller answered {got!r} to {letter!r}")


def pulse(port, args):
	command(port, "H")
	time.sleep(args.high_ms / 1000)
	command(port, "L")
	time.sleep(args.low_ms / 1000)
	command(port, "H")
	time.sleep(args.after_ms / 1000)


def present(usb):
	uart = ftlib.find_node("uart_mode", usb)
	i2c = ftlib.find_node("hid_over_i2c_en", usb)
	tty = ftlib.find_tty(uart) if uart else None
	bus = ftlib.find_i2c_bus(usb)
	return uart and i2c and tty and os.path.exists(tty) and bus is not None


def wait_back(usb, recover_s):
	deadline = time.monotonic() + recover_s
	while time.monotonic() < deadline:
		if present(usb):
			time.sleep(SETTLE_S)
			return present(usb)
		time.sleep(0.25)
	return False


def uart_ok(usb, peer, baud, tag):
	tty = ftlib.find_tty(ftlib.find_node("uart_mode", usb))
	msg = f"ft260-reset-{tag}\n".encode()
	try:
		with serial.Serial(peer, baud, timeout=2) as rx, \
		     serial.Serial(tty, baud, timeout=2) as tx:
			rx.reset_input_buffer()
			tx.write(msg)
			tx.flush()
			got = rx.read_until(msg)
	except serial.SerialException:
		return False
	return msg in got


def i2c_ok(usb, addr):
	bus = ftlib.find_i2c_bus(usb)
	if bus is None:
		return False
	done = ftlib.i2c_read(bus, addr, 16)
	return done.returncode == 0 and len(done.stdout.split()) == 16


def main():
	ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
	ap.add_argument("--reset", required=True,
			help="serial port of the RESETN controller")
	ap.add_argument("--usb", required=True,
			help="USB port of the FT260, for example 2-3.4")
	ap.add_argument("--peer", required=True,
			help="tty wired to the FT260 UART")
	ap.add_argument("--eeprom", default=ftlib.BIG_EEPROM)
	ap.add_argument("--cycles", type=int, default=100)
	ap.add_argument("--keep-going", action="store_true")
	ap.add_argument("--ko", help="rmmod, then insmod this module")
	ap.add_argument("--debug", action="store_true",
			help="load --ko with debug=1")
	ap.add_argument("--baud", type=int, default=9600)
	ap.add_argument("--high-ms", type=int, default=200)
	ap.add_argument("--low-ms", type=int, default=1500)
	ap.add_argument("--after-ms", type=int, default=1500)
	ap.add_argument("--recover-s", type=float, default=20.0)
	ap.add_argument("--log", default=time.strftime(
		"tn189-reset-loop-%Y%m%d-%H%M%S.log"))
	args = ap.parse_args()

	log = Log(args.log)
	stem = os.path.splitext(args.log)[0]
	usb = os.path.realpath(os.path.join("/sys/bus/usb/devices", args.usb))

	if args.ko:
		if ftlib.module_loaded():
			ftlib.rmmod()
		subprocess.run(["insmod", args.ko] +
			       (["debug=1"] if args.debug else []), check=True)
		wait_back(usb, args.recover_s)
	if not ftlib.module_loaded():
		sys.exit("hid_ft260 is not loaded")
	if not present(usb):
		sys.exit(f"no FT260 with both interfaces at {args.usb}")

	log(f"== {time.strftime('%F %T')} {' '.join(sys.argv)}")
	log(f"== kernel {os.uname().release}, hid_ft260 srcversion "
	    f"{ftlib.read_attr('/sys/module/hid_ft260', 'srcversion')}")
	log("cycle  enum stalls resets back uart i2c complaints seconds")

	total = {"cycles": 0, "stalls": 0, "resets": 0, "failed": 0}
	enum_line = f"usb {args.usb}: new "
	reset_line = f"usb {args.usb}: reset "
	port = serial.Serial(args.reset, 115200, timeout=2)
	try:
		for n in range(1, args.cycles + 1):
			start = time.monotonic()
			marker = ftlib.mark(f"reset-loop-{n}")
			pulse(port, args)
			back = wait_back(usb, args.recover_s)
			uart = back and uart_ok(usb, args.peer, args.baud, n)
			i2c = back and i2c_ok(usb, args.eeprom)
			text = ftlib.dmesg_since(marker)
			lines = text.splitlines()
			enum = sum(enum_line in line for line in lines)
			stalls = sum(STALL in line for line in lines)
			resets = sum(reset_line in line for line in lines)
			hits = [line for line in lines
				if any(word in line for word in COMPLAINTS)]
			ok = enum and back and uart and i2c and not hits

			total["cycles"] = n
			total["stalls"] += stalls
			total["resets"] += resets
			total["failed"] += not ok
			log(f"{n:5d}  {enum:4d} {stalls:6d} {resets:6d} "
			    f"{'y' if back else 'N':>4} "
			    f"{'y' if uart else 'N':>4} {'y' if i2c else 'N':>3} "
			    f"{len(hits):10d} {time.monotonic() - start:7.1f}")
			if stalls or not ok:
				with open(f"{stem}.cycle-{n}.dmesg", "w") as f:
					f.write(text)
			if not ok and not args.keep_going:
				log(f"== stop: cycle {n} failed")
				break
	except KeyboardInterrupt:
		log("== stop: interrupted")
	finally:
		port.close()
		log(f"== {time.strftime('%F %T')} cycles {total['cycles']}, "
		    f"stalls {total['stalls']}, resets {total['resets']}, "
		    f"failed {total['failed']}")


if __name__ == "__main__":
	main()
