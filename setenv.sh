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

# Example with 2 ft260 devices:
#   michael@m2:~/sw/hid-ft260$ . setenv.sh
#   sysfs_i2c_15
#   sysfs_i2c_0

#   michael@m2:~/sw/hid-ft260$ echo $sysfs_i2c_0
#   /sys/bus/hid/drivers/ft260/0003:0403:6030.0007
#
#   michael@m2:~/sw/hid-ft260$ echo $sysfs_i2c_15
#   /sys/bus/hid/drivers/ft260/0003:0403:6030.000C

PVID='*0403:6030*'
SYSFS_FT260=/sys/bus/hid/drivers/ft260/

a=$(find $SYSFS_FT260 -name $PVID | xargs -I % sh -c 'find %/ -maxdepth 1 -name i2c-*')
a+=" "
a+=$(find $SYSFS_FT260 -name $PVID | xargs -I % sh -c 'find %/ -maxdepth 2 -name ttyFT*');
for w in $a; do
	b=$(basename "$w"); c=$(dirname "$w"); d=sysfs_${b/-/_}; echo $d; declare $d=$c
done
