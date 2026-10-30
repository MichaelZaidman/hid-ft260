# SPDX-License-Identifier: MIT
"""gpio_en follows uart_mode, and the gpiochip follows gpio_en.

Writing uart_mode runs ft260_gpio_en_update(), which clears one group
of pins and sets another: gpio_en = (gpio_en & ~CLR) | SET. The two
tables are in the driver, CLR in the switch on the written value and
SET in dev->gpio_uart_mode[], filled at probe. The expected mask is
computed from the mask logged before the write, so a board whose
gpio2_func, gpioa_func or gpiog_func differs needs no special case.

Then every one of the 14 offsets is claimed. The gpiochip must take
the line when its bit is in gpio_en and refuse it with -EIO when it is
not. The direction asked for is "in": ft260_gpio_direction_set()
checks gpio_en before it looks at the direction, so the answer is the
same and GPIOC and GPIOD, the UART RX and TX pins wired to the peer,
are not driven.

uart_mode 0 is FT260_UART_CFG_FLOW_CTRL_OFF, which turns the UART off
in the chip. The fixture puts the old mode back and the last test
checks that the port still carries a byte.
"""

import errno
import os

import pytest

import ftlib

GPIO_TOTAL = 14

# Pin groups, FT260_GPIO_* in the driver. A is bit 6, H is bit 13.
RX_TX = (1 << 8) | (1 << 9)		# GPIOC, GPIOD
DCD_RI = (1 << 4) | (1 << 5)		# GPIO4, GPIO5
RTS_CTS = (1 << 7) | (1 << 10)		# GPIOB, GPIOE
DTR_DSR = (1 << 11) | (1 << 13)		# GPIOF, GPIOH

# dev->gpio_uart_mode[], set at probe.
SET = {
	0: RX_TX | DCD_RI | RTS_CTS | DTR_DSR,
	1: DTR_DSR,
	2: RTS_CTS,
	3: RTS_CTS | DTR_DSR | DCD_RI,
	4: RTS_CTS | DTR_DSR | DCD_RI,
}

# The bitmap cleared first, from the switch on the written value.
CLR = {
	0: SET[0],
	1: RX_TX | RTS_CTS,
	2: RX_TX | DTR_DSR,
	3: RX_TX,
	4: RX_TX,
}


@pytest.fixture(scope="module")
def saved_mode(chip):
	"""Put uart_mode back whatever the tests leave behind."""
	orig = ftlib.read_attr(chip.uart, "uart_mode")
	yield orig
	ftlib.write_attr(chip.uart, "uart_mode", orig)


def set_mode(chip, mode):
	"""Write the mode, return the mask the driver logged."""
	marker = ftlib.mark(f"uart-mode-{mode}")
	ftlib.write_attr(chip.uart, "uart_mode", mode)
	mask = ftlib.mask_of(os.path.basename(chip.uart), marker)
	assert mask is not None, "the driver logged no mask"
	return int(mask, 16)


@pytest.mark.parametrize("mode", sorted(SET), ids=lambda m: f"mode{m}")
def test_mask_and_lines_follow_the_mode(chip, saved_mode, mode):
	now = ftlib.gpio_en_now(chip.uart)
	assert now is not None, "no mask in the log for this interface"
	before = int(now, 16)
	want = (before & ~CLR[mode]) | SET[mode]

	assert set_mode(chip, mode) == want

	for offset in range(GPIO_TOTAL):
		err = ftlib.line_errno(chip.base + offset, "in")
		if want & (1 << offset):
			assert err is None, f"offset {offset} refused with {err}"
		else:
			assert err == errno.EIO, f"offset {offset} gave {err}"


def test_the_port_still_works_after_the_sweep(chip, saved_mode, peer_link):
	import serial

	ftlib.write_attr(chip.uart, "uart_mode", saved_mode)
	msg = b"ft260-uart-mode-sweep\n"
	with serial.Serial(peer_link, 9600, timeout=2) as tx, \
	     serial.Serial(chip.tty, 9600, timeout=2) as rx:
		tx.write(msg)
		tx.flush()
		assert rx.read(len(msg)) == msg
