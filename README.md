# hid-ft260

## Overview

### FTDI FT260 Linux kernel driver

The FTDI FT260 chip implements USB to I2C/UART bridges through two
USB HID class interfaces. The first - is for I2C and the second
for UART. Each interface is independent, and the kernel detects it
as a separate USB hidraw device. In addition, the chip implements
14 GPIOs via multifunctional pins.

## Current status

The **main branch** of this repository holds the most comprehensive version
of the FT260 Linux kernel driver:

1. I2C support - upstreamed into the kernel 5.13 mainline.
2. UART support - two implementations ([1](https://patches.linaro.org/project/linux-serial/patch/20220928192421.11908-1-contact@christina-quast.de/))
   and ([2](https://lore.kernel.org/lkml/638c51a2.170a0220.3af16.18f8@mx.google.com/))
   of the initial UART support were around for some time, and now they are
   unified, fixed several issues and merged into the **main branch** of this repo.
   It is submitted upstream via this
   [commit](https://patchwork.kernel.org/project/linux-input/patch/20240216-ft260_review5-v5-1-36db44673ac7@christina-quast.de/).
3. GPIO support - developed in the [gpio](https://github.com/MichaelZaidman/hid-ft260/tree/gpio)
   branch, reapplied on top of the latest UART support, and is merged into the **main branch** of this repo.
   The earlier GPIO version (before UART support) was submitted upstream via this
   [commit](https://lore.kernel.org/lkml/20230211115752.26276-2-michael.zaidman@gmail.com/T/).


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

1.	It enables usage of the standard Linux userspace I2C tools like
    [i2c-tools](https://i2c.wiki.kernel.org/index.php/I2C_Tools) and a wide
    range of userspace applications relying on the `/dev/i2c-x` kernel API.
2.	The driver exposes the FT260 I2C controller device via `sys/bus/i2c`
    bus to other I2C devices relied on it. It allows the sysfs I2C tree
    instantiation, essential for complex I2C topologies, built with I2C
    multiplexers and/or I2C switches.
3.	It guarantees atomic access at the kernel level to the I2C devices,
    resided on the I2C controller's bus, implicitly enabling concurrent
    access to the I2C bus from multiple userspace processes instead of
    explicit contexts synchronization when such access is done via hidraw
    and libusb userspace libraries.

### UART Interface
This driver adds a serial interface /dev/ttyFTx, which implements tty serial
driver ops, making it easier to configure the baud rate, transmit and receive data,
and termios settings.

### References
1. [DS_FT260.pdf](https://ftdichip.com/wp-content/uploads/2020/07/DS_FT260.pdf)
2. [AN_394_User_Guide_for_FT260.pdf](https://www.ftdichip.com/Support/Documents/AppNotes/AN_394_User_Guide_for_FT260.pdf)

## HOWTO

### Getting started
```
git clone https://github.com/MichaelZaidman/hid-ft260.git
cd hid-ft260
make
sudo insmod hid-ft260.ko
```

If you got the below failure, your kernel does not support the `hid_is_usb`,
which was added by [https://lkml.org/lkml/2021/12/13/526](https://lkml.org/lkml/2021/12/13/526) commit.
```
/home/swuser/sw/hid-ft260/hid-ft260.c: In function ‘ft260_probe’:
/home/swuser/sw/hid-ft260/hid-ft260.c:928:7: error: implicit declaration of function ‘hid_is_usb’ [-Werror=implicit-function-declaration]
  if (!hid_is_usb(hdev))
```

As workaround you can comment two lines in the ft260_probe routine:
```
        if (!hid_is_usb(hdev))
                return -EINVAL;
```

### Configure ft260 device via sysfs
The driver exposes multiple attributes to user space via sysfs.

Since the order and number of the USB devices vary between systems,
the sysfs path to the USB device may change even on the same system.

You can figure out the sysfs path yourself or use the setenv script
to set the environment variable per USB interface of every FT260 device.

```
. ./setenv.sh
```
#### Example with 2 ft260 devices:
```
$ . setenv.sh
sysfs_i2c_15
sysfs_i2c_0

$ echo $sysfs_i2c_0
/sys/bus/hid/drivers/ft260/0003:0403:6030.0007

$ echo $sysfs_i2c_15
/sys/bus/hid/drivers/ft260/0003:0403:6030.000C
```

Now we can see all availiable attributes:
```
$ ls $sysfs_i2c_0
```

### Change I2C bus clock

Figure out the sysfs ft260 device node path, as explained earlier.
To set the i2c clock to 400KHz on my system, I run this command:

```
sudo bash -c 'echo 400 > $sysfs_i2c_0/clock'
```

### Set a multifunctional pin as GPIO

The FT260 has three pins that have more than two functions: DIO7 (pin 14),
DIO0 (pin 7), and DIO12 (pin 27). The function of these pins can be configured
by eFUSE, EEPROM, or via the sysfs interface.

For the sysfs method, you need to figure out the sysfs ft260 device node path,
as explained earlier.

In this example, I configure pin 14 to act as GPIO2:

```
$ cat $sysfs_i2c_0/gpio2_func
1
$ sudo bash -c "echo 0 > $sysfs_i2c_0/gpio2_func"
$ cat $sysfs_i2c_0/gpio2_func
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
