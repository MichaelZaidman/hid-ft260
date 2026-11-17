# hid-ft260

## Overview

### FTDI FT260 Linux kernel driver

The FTDI FT260 chip implements USB to I2C/UART bridges through two
USB HID class interfaces. The first is for I2C and the second
for UART. Each interface is a separate USB HID device. With this
driver bound, the kernel creates no hidraw node for them. In addition,
the chip implements 14 GPIOs via multifunctional pins.

## Current status

The **main branch** of this repository holds the most comprehensive version
of the FT260 Linux kernel driver:

1. I2C support - upstreamed into the kernel 5.13 mainline.
2. UART support - two implementations ([1](https://patches.linaro.org/project/linux-serial/patch/20220928192421.11908-1-contact@christina-quast.de/))
   and ([2](https://lore.kernel.org/lkml/638c51a2.170a0220.3af16.18f8@mx.google.com/))
   of the initial UART support were around for some time, and now they are
   unified, fixed several issues, and merged into the **main branch** of this repo.
   It is submitted upstream via this
   [commit](https://patchwork.kernel.org/project/linux-input/patch/20240216-ft260_review5-v5-1-36db44673ac7@christina-quast.de/).
   Since then, more functionalities have been added to this repository,
   such as flow control and modem control via ioctl support.
3. GPIO support - developed in the [gpio](https://github.com/MichaelZaidman/hid-ft260/tree/gpio)
   branch, reapplied on top of the latest UART support, and is merged into the **main branch** of this repo.
   The earlier GPIO version (before UART support) was submitted upstream via this
   [commit](https://lore.kernel.org/lkml/20230211115752.26276-2-michael.zaidman@gmail.com/T/).
   Since then, several issues have been fixed, and the code was improved and evolved in this repo.
4. Upstream series - in August 2026, the UART and GPIO support and the
   I2C fixes from this repo were posted upstream as one series,
   [HID: ft260: add UART and GPIO support, plus I2C fixes](https://lore.kernel.org/all/20260822213941.98882-1-michael.zaidman@gmail.com/T/#t).
   It replaces the earlier UART and GPIO submissions above. Later
   versions:
   - [v2](https://lore.kernel.org/all/20261030223637.86907-1-michael.zaidman@gmail.com/T/#t), Oct 2026
   - [v3](https://lore.kernel.org/all/20261114230800.1344401-1-michael.zaidman@gmail.com/T/#t), Nov 2026
   - [v4](https://lore.kernel.org/all/20261117200423.19561-1-michael.zaidman@gmail.com/T/#t), Nov 2026

   Note: v2, v3 and v4 were tested with the suite in `tests/`.

### I2C Interface
FTDI suggests using hidraw and libusb userspace libraries to operate the
FT260 I2C host controller via hidraw Linux kernel driver. But this
approach makes the standard Linux I2C tools useless, and it does not
allow the I2C sysfs tree instantiation required by I2C multiplexers
and switches.

This driver enables the FT260 to be seen and handled by the Linux
kernel i2c-core code as a regular I2C bus adapter, like when the I2C
controller is a part of the MCU or CPU vendor chipset. It provides the
following benefits:

1.  It enables usage of the standard Linux userspace I2C tools like
    [i2c-tools](https://i2c.wiki.kernel.org/index.php/I2C_Tools) and a wide
    range of userspace applications relying on the `/dev/i2c-x` kernel API.
2.  The driver exposes the FT260 I2C controller device via `/sys/bus/i2c`
    bus to other I2C devices relying on it. It allows the sysfs I2C tree
    instantiation, essential for complex I2C topologies, built with I2C
    multiplexers and/or I2C switches.
3.  It guarantees atomic access at the kernel level to the I2C devices,
    residing on the I2C controller's bus, implicitly enabling concurrent
    access to the I2C bus from multiple userspace processes instead of
    explicit context synchronization when such access is done via hidraw
    and libusb userspace libraries.

### UART Interface
This driver adds a serial interface /dev/ttyFTx, which implements tty serial
driver ops, making it easier to configure the baud rate, transmit and receive data,
and termios settings.

### References
1. [DS_FT260.pdf](https://ftdichip.com/wp-content/uploads/2023/08/DS_FT260.pdf)
2. [AN_394_User_Guide_for_FT260.pdf](https://ftdichip.com/wp-content/uploads/2025/04/AN_394_User_Guide_for_FT260.pdf)

## HOWTO

### Getting started
```sh
git clone https://github.com/MichaelZaidman/hid-ft260.git
cd hid-ft260
make
sudo insmod hid-ft260.ko
```

Kernels before 5.16 do not have `hid_is_usb`, which was introduced in this
[commit](https://lkml.org/lkml/2021/12/13/526). On such a kernel the build
fails with this error:
```console
.../hid-ft260.c: In function ‘ft260_probe’:
.../hid-ft260.c: error: implicit declaration of function ‘hid_is_usb’ [-Werror=implicit-function-declaration]
  if (!hid_is_usb(hdev))
```

As a workaround, you can comment out the following two lines in the ft260_probe routine:
```c
        if (!hid_is_usb(hdev))
                return -EINVAL;
```

### Configure ft260 device via sysfs
The driver exposes multiple attributes to user space via sysfs.

Since the order and number of the USB devices vary between systems,
the sysfs path to the USB device may change even on the same system.

You can figure out the sysfs path yourself or use the `setenv` script
to set the environment variable per USB interface of every FT260 device.

```sh
. ./setenv.sh
```
#### Example with 2 ft260 devices
```console
$ . setenv.sh
sysfs_i2c_15
sysfs_ttyFT1
sysfs_i2c_14
sysfs_ttyFT0
```

Now `ls $sysfs_i2c_14` and `ls $sysfs_ttyFT0` list the attributes of each
interface.

### sysfs device attributes location
The driver groups the device sysfs attributes per I2C and/or UART HID
interfaces depending on the configured chip mode via the DCNF0 and DCNF1
pins.

The GPIO-related sysfs attributes are grouped under the UART interface
in the 0, 2, and 3 chip modes and the I2C interface in chip mode 1.

### Change I2C bus clock

Figure out the sysfs ft260 device node path, as explained earlier.
The clock is set in kHz, in the range 60 to 3400. The driver rejects a
value outside that range. To set the i2c clock to 400 kHz, run this command:

```sh
sudo bash -c "echo 400 > $sysfs_i2c_14/clock"
```

### Enable debug logging

To enable verbose debug output from the driver at load time:

```sh
sudo insmod hid-ft260.ko debug=1
```

Or at runtime:

```sh
echo 1 | sudo tee /sys/module/hid_ft260/parameters/debug
```

The debug messages are written to the kernel log and can be viewed with `dmesg`.
To disable debug logging:

```sh
echo 0 | sudo tee /sys/module/hid_ft260/parameters/debug
```

### Set a multifunctional pin as a GPIO

The FT260 has three pins that have more than two functions: DIO7 (pin 14),
DIO0 (pin 7), and DIO12 (pin 27). The function of these pins can be
configured by eFUSE, EEPROM, or via the sysfs interface.

For the sysfs method, you need to figure out the sysfs ft260 device node
path, as explained earlier in **Configure ft260 device via sysfs** and
**sysfs device attributes location** sections.

In this example, I configure pin 14 to act as GPIO2. The `gpioget` and
`gpioset` commands are from libgpiod v1:

```console
$ cat $sysfs_ttyFT0/gpio2_func
1
$ sudo bash -c "echo 0 > $sysfs_ttyFT0/gpio2_func"
$ cat $sysfs_ttyFT0/gpio2_func
0
$ sudo gpioget 0 2
0
$ sudo gpioset 0 2=1
$ sudo gpioget 0 2
1
$ sudo gpioset 0 2=0
$ sudo gpioget 0 2
0
```

### Modem pins control via ioctl (TIOCMBIS, TIOCMBIC, TIOCMGET and TIOCMSET)

TIOCMGET reports RTS, CTS, DTR, DSR, DCD (TIOCM_CAR) and RI (TIOCM_RNG).
TIOCMSET, TIOCMBIS and TIOCMBIC change RTS and DTR only.

Controlling the DTR and RTS UART signals through user-space applications
via ioctl is helpful for programming embedded devices, such as the ESP32,
using esptool as described in [issue #27](https://github.com/MichaelZaidman/hid-ft260/issues/27).

### Run the test suite

The tests in `tests/` drive real FT260 hardware. They need root, pytest,
pyserial and i2c-tools; without `i2ctransfer` the I2C tests skip:
```sh
cd tests
python3 -m venv .venv
.venv/bin/pip install pytest pyserial
sudo .venv/bin/pytest -v
```

They load `../hid-ft260.ko`; use `--ko=PATH` to test another build, for
example the in-tree module. A test skips when its hardware is missing: an
i2c device on the FT260 bus (a 24LC512 at 0x51 for the write tests), a
second UART wired to the FT260 (`--peer`, default /dev/ttyUSB0), a chip
strapped to another chip mode, a UART pattern file (`--pattern`), or the
i2cperf tool (`--i2cperf`). `pytest --help` lists all options.

The tests were written for the author's bench, two FT260 boards and an
FT2232 wired to one of them. That setup is not documented here.

`tn189_reset_loop.py` is not part of the pytest run. It tests the TN_189
workaround and needs a HW setup that holds the FT260 RESETN pin low on
command. `tn189_reset/tn189_reset.ino` is the sketch for that setup and
describes its serial commands.
