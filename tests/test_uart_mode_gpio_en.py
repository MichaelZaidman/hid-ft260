# SPDX-License-Identifier: MIT
"""gpio_en comes from the flow control mode the probe writes.

ft260_uart_probe() forces the chip to FLOW_CTRL_NONE, but
ft260_gpio_init() used to derive gpio_en from the mode read before that
write. With a chip reporting RTS_CTS at probe, gpio_en came out 0x2800
instead of 0x2cb0 and the DCD, RI, RTS and CTS lines were refused; only
DTR and DSR survived.

The test arms that trap: it stores RTS_CTS in the chip, reloads, and
requires FT260_GPIO_UART_MODE_4_SET and all six lines.
"""

import pytest

import ftlib

WANT = 0x2CB0			# FT260_GPIO_UART_MODE_4_SET
RTS_CTS = 1			# FT260_UART_CFG_FLOW_CTRL_RTS_CTS
NAMES = {4: "DCD", 5: "RI", 7: "RTS", 10: "CTS", 11: "DTR", 13: "DSR"}


@pytest.fixture(scope="module")
def armed(chip):
	"""Store RTS_CTS in the chip, re-probe, put the old mode back."""
	before = ftlib.read_attr(chip.uart, "uart_mode")
	ftlib.write_attr(chip.uart, "uart_mode", RTS_CTS)
	log = ftlib.rebind(chip.uart)

	yield {"log": log, "probed": ftlib.probed_uart_mode(log),
	       "mask": ftlib.gpio_en(log)}

	ftlib.write_attr(chip.uart, "uart_mode", before)
	ftlib.rebind(chip.uart)


def test_probe_read_rts_cts(armed):
	"""Without this the rest proves nothing: the trap was not armed."""
	assert armed["probed"] == RTS_CTS


def test_mask_covers_mode_4_set(armed):
	assert armed["mask"] is not None, "the driver logged no mask"
	assert int(armed["mask"], 16) & WANT == WANT


@pytest.mark.parametrize("offset", sorted(NAMES),
			 ids=[NAMES[o] for o in sorted(NAMES)])
def test_modem_line_usable(chip, armed, offset):
	assert ftlib.line_state(chip.base + offset) == "usable"
