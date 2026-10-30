# SPDX-License-Identifier: MIT
"""Writing gpioN/value must not report an error to userspace.

ft260_gpio_set() used to return hid_hw_raw_request()'s byte count, which
gpiolib treats as a failure, so a successful write surfaced as an error
on the value attribute. The write is what matters here, so the failure
message names the errno; test_gpio_func.py covers the same pin from the
function-select side.
"""

import pytest

import ftlib

GPIO2 = 2			# DIO7, pin 14


@pytest.fixture(scope="module")
def driven(chip):
	"""Hand GPIO2 to the gpiochip and drive it as an output."""
	orig = ftlib.read_attr(chip.uart, "gpio2_func")
	ftlib.write_attr(chip.uart, "gpio2_func", 0)
	line = chip.base + GPIO2
	ftlib.export(line)
	ftlib.set_direction(line, "out")

	yield line

	try:
		ftlib.unexport(line)
	except OSError:
		pass
	ftlib.write_attr(chip.uart, "gpio2_func", orig)


@pytest.mark.parametrize("value", [1, 0, 1, 0],
			 ids=["set", "clear", "set_again", "clear_again"])
def test_value_write_is_silent(driven, value):
	try:
		ftlib.set_value(driven, value)
	except OSError as err:
		pytest.fail(f"write {value} returned {err.strerror}")
	assert ftlib.get_value(driven) == str(value)
