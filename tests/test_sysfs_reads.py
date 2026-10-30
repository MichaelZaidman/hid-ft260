# SPDX-License-Identifier: MIT
"""The read-only attributes: permissions and values.

test_sysfs_groups.py checks where each attribute sits and
test_sysfs_stores.py checks the writable ones. The five read-only
attributes are chip_mode, pwren_status, suspend_status,
power_saving_en and hid_over_i2c_en. Until now nothing read their
values.

All five come from one feature report, parsed as
struct ft260_get_system_status_report. The tests here cannot prove
the field offsets in that struct. There is no second way to read the
report from userspace: the driver calls hid_hw_start(hdev, 0), so no
hidraw node appears, and a read-only attribute has no round trip.
Two offsets are proved elsewhere. chip_mode decides which attribute
group is attached, which test_sysfs_groups.py and the three chip mode
files check. power_saving_en reaches dev->power_saving_en and gates
the wake-up line test_power_saving.py reads.

What is left to check here is the value range and the permissions.
kernfs_fop_open() returns -EACCES for a write open when S_IWUGO is
clear, so a DEVICE_ATTR_RO file rejects a write even for root.
"""

import errno
import os

import pytest

import ftlib

# Attribute name -> which HID interface carries it.
READ_ONLY = {
	"chip_mode":		"uart",
	"pwren_status":		"uart",
	"suspend_status":	"uart",
	"power_saving_en":	"uart",
	"hid_over_i2c_en":	"i2c",
}

# The fields documented as 0 or 1 in ft260_get_system_status_report.
BOOLEAN = ["pwren_status", "suspend_status", "power_saving_en",
	   "hid_over_i2c_en"]

READ_LEN = 8


def _node(chip, which):
	node = chip.uart if which == "uart" else chip.i2c
	if node is None:
		pytest.skip(f"no {which} interface on this chip")
	return node


def _path(chip, attr):
	node = _node(chip, READ_ONLY[attr])
	path = os.path.join(node, attr)
	if not os.path.exists(path):
		pytest.skip(f"no {attr} in this chip mode")
	return path


@pytest.mark.parametrize("attr", sorted(READ_ONLY))
def test_read_only_attr_rejects_a_write(chip, attr):
	path = _path(chip, attr)

	assert os.stat(path).st_mode & 0o777 == 0o444

	with pytest.raises(OSError) as err:
		open(path, "w").close()

	assert err.value.errno == errno.EACCES


@pytest.mark.parametrize("attr", BOOLEAN)
def test_boolean_attr_reads_zero_or_one(chip, attr):
	value = open(_path(chip, attr)).read().strip()

	assert value in ("0", "1")


def test_chip_mode_is_in_range(chip):
	"""DCNF0 and DCNF1, two bits, so 0 to 3."""
	value = int(open(_path(chip, "chip_mode")).read())

	assert 0 <= value <= 3


def test_suspend_status_is_zero_after_a_transfer(chip, eeprom):
	"""0 means not suspended, and the chip just answered."""
	done = ftlib.i2c_read(chip.bus, eeprom, READ_LEN)
	assert done.returncode == 0, done.stderr.strip()

	assert open(_path(chip, "suspend_status")).read().strip() == "0"


def test_pwren_status_is_one_after_a_transfer(chip, eeprom):
	"""1 means the chip is ready, and the chip just answered."""
	done = ftlib.i2c_read(chip.bus, eeprom, READ_LEN)
	assert done.returncode == 0, done.stderr.strip()

	assert open(_path(chip, "pwren_status")).read().strip() == "1"
