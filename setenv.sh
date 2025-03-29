#!/bin/bash
# SPDX-License-Identifier: MIT
#
# Copyright (C) 2024 Michael Zaidman <michael.zaidman@gmail.com>
#
# The script sets the environment variables pointing to the sysfs attribute
# directories of all active ft260 devices.
#
# Usage:
#   . ./setenv.sh

# Example with 1 ft260 device:
: '
$ . setenv.sh
sysfs_i2c_14
sysfs_ttyFT0

$ echo $sysfs_i2c_14
/sys/bus/hid/drivers/ft260/0003:0403:6030.0011
'
# Example with 2 ft260 devices:
: '
$ . setenv.sh
sysfs_i2c_15
sysfs_ttyFT1
sysfs_i2c_14
sysfs_ttyFT0

$ echo $sysfs_i2c_15
/sys/bus/hid/drivers/ft260/0003:0403:6030.0013
'

PVID='*0403:6030*'
SYSFS_FT260=/sys/bus/hid/drivers/ft260/

a=$(find $SYSFS_FT260 -name $PVID | xargs -I % sh -c 'find %/ -maxdepth 1 -name i2c-* -o -name tty*')
for w in $a; do
	b=$(basename "$w")
	[ "$b" = "tty" ] && e=$(find $w/ -name ttyF*) && b=$(basename "$e")
	c=$(dirname "$w")
	d=sysfs_${b/-/_}
	echo $d
	declare $d=$c
done