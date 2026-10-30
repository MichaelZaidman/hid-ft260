# SPDX-License-Identifier: MIT
"""GPIO2 function-select round trip, chip mode 0x03.

GPIO2 is DIO7, pin 14. Writing gpio2_func 0 hands the pin to the
gpiochip, which the driver reports as bit 2 joining the enabled mask.
Writing 1 gives it back to the UART. The tests run in file order: the
pin is taken in the first one and given back in the last.
"""

import pytest

import ftlib

GPIO2 = 2
EN_WITH_GPIO2 = "2cb4"
EN_WITHOUT_GPIO2 = "2cb0"


@pytest.fixture(scope="module")
def gpio2(chip):
	"""Hand GPIO2 to the gpiochip, export it as an output, give it back."""
	if chip.chip_mode != "3":
		pytest.skip(f"chip mode {chip.chip_mode}, this step wants 3")

	orig = ftlib.read_attr(chip.uart, "gpio2_func")
	marker = ftlib.mark("gpio2-func-0")
	ftlib.write_attr(chip.uart, "gpio2_func", 0)
	line = chip.base + GPIO2
	ftlib.export(line)
	ftlib.set_direction(line, "out")

	yield {"line": line, "orig": orig, "mask": ftlib.gpio_en_since(marker)}

	try:
		ftlib.unexport(line)
	except OSError:
		pass
	if ftlib.read_attr(chip.uart, "gpio2_func") != orig:
		ftlib.write_attr(chip.uart, "gpio2_func", orig)


def test_func_0_enables_the_line(gpio2):
	assert gpio2["mask"] == EN_WITH_GPIO2


@pytest.mark.parametrize("value", [1, 0, 1, 0])
def test_value_round_trip(gpio2, value):
	ftlib.set_value(gpio2["line"], value)
	assert ftlib.get_value(gpio2["line"]) == str(value)


def test_input_direction_reads_a_level(gpio2):
	ftlib.set_direction(gpio2["line"], "in")
	assert ftlib.get_value(gpio2["line"]) in ("0", "1")


def test_restore_releases_the_line(chip, gpio2):
	if gpio2["orig"] == "0":
		pytest.skip("gpio2_func starts at 0, the UART never had the pin")
	marker = ftlib.mark("gpio2-func-restore")
	ftlib.write_attr(chip.uart, "gpio2_func", gpio2["orig"])
	assert ftlib.gpio_en_since(marker) == EN_WITHOUT_GPIO2
