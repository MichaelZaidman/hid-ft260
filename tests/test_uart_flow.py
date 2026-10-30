# SPDX-License-Identifier: MIT
"""Flow control over the FT260 UART, against an FT2232 on /dev/ttyUSB0.

The peer is an FT2232 cabled to one FT260 UART; --peer names it and
--chip names the board, or the chip fixture asks the peer which board
answers.

The FT260 is a full speed device with a 1 ms interrupt pipe and a
60-byte payload, so it tops out near 60 kB/s and never makes the FT2232
wait. Only USB0 -> FT0 exercises flow control. The first case sends with
no flow control and must lose data; that is what makes the two paced
cases mean something. The last case sends the other way, where the
FT260 paces itself, and is a transmit path check.

Each case also reads uart_mode and the gpio_en the driver logs:
ft260_uart_set_termios() maps CRTSCTS to 1, IXON|IXOFF to 3 and neither
to 4, then calls ft260_gpio_en_update(). With RTS/CTS the mask drops
bits 7 and 10, the RTS and CTS lines.
"""

import os
import threading
import time

import pytest
import serial

import ftlib

BAUD = 1500000
IDLE = 2.0			# receiver stops after this long with no byte

# flow name -> (xonxoff, rtscts, uart_mode, gpio_en without GPIO2)
FLOWS = {
	"none":    (False, False, "4", 0x2CB0),
	"xonxoff": (True,  False, "3", 0x2CB0),
	"rtscts":  (False, True,  "1", 0x2830),
}


def want_mask(chip, flow):
	"""Mask the driver must log, with this chip's gpio2_func folded in."""
	return "%04x" % (FLOWS[flow][3] | ftlib.gpio2_bit(chip.uart))


@pytest.fixture(scope="module")
def payload(request, chip):
	if not chip.tty:
		pytest.skip("the chip has no tty")
	path = request.config.getoption("--pattern")
	if not os.path.exists(path):
		pytest.skip(f"no pattern at {path}")
	count = request.config.getoption("--bytes")
	with open(path, "rb") as f:
		return f.read() if count == 0 else f.read(count)


def drain(port, out):
	last = time.monotonic()
	while time.monotonic() - last < IDLE:
		chunk = port.read(port.in_waiting or 1)
		if chunk:
			out += chunk
			last = time.monotonic()


def transfer(chip, tx_path, rx_path, flow, data):
	"""Send data one way and return (received, uart_mode, gpio_en)."""
	xon, rts, _, _ = FLOWS[flow]
	marker = ftlib.mark(f"flow-{flow}")
	got = bytearray()
	with serial.Serial(rx_path, BAUD, xonxoff=xon, rtscts=rts,
			   timeout=0.5) as rx, \
	     serial.Serial(tx_path, BAUD, xonxoff=xon, rtscts=rts,
			   timeout=60) as tx:
		rx.reset_input_buffer()
		reader = threading.Thread(target=drain, args=(rx, got))
		reader.start()
		tx.write(data)
		tx.flush()
		reader.join()
	mode = ftlib.read_attr(chip.uart, "uart_mode")
	return bytes(got), mode, ftlib.mask_of(chip.uart.split("/")[-1], marker)


@pytest.mark.parametrize("flow", ["none", "xonxoff", "rtscts"])
def test_peer_to_ft260(chip, peer_link, payload, flow):
	got, mode, mask = transfer(chip, peer_link, chip.tty, flow, payload)
	assert mode == FLOWS[flow][2]
	assert mask == want_mask(chip, flow)
	if flow == "none":
		assert len(got) < len(payload), "no flow control, yet nothing lost"
	else:
		assert got == payload, f"got {len(got)} of {len(payload)} bytes"


def test_ft260_to_peer(chip, peer_link, payload):
	got, mode, mask = transfer(chip, chip.tty, peer_link, "none", payload)
	assert mode == FLOWS["none"][2]
	assert mask == want_mask(chip, "none")
	assert got == payload, f"got {len(got)} of {len(payload)} bytes"
