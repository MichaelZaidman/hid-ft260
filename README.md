# hid-ft260

## Overview

### FTDI FT260 Linux kernel driver

The FTDI FT260 chip implements USB to I2C/UART bridges through two
USB HID class interfaces. The first - is for I2C and the second
for UART. Each interface is independent, and the kernel detects it
as a separate USB hidraw device. In addition, the chip implements
14 GPIOs via multifunctional pins.

Current status:
1. The initial version of the USB HID to I2C bridge is merged into the kernel 5.13 mainline.
2. The "HID: ft260: fixes and performance improvements" patch set is queued into the kernel 6.2 release.
3. The initial GPIO support is developed in the gpio branch of this repo.
4. The initial UART support is discussed in the kernel mailing lists.
5. I continue developing the driver in this repo, upstreaming the changes once they are mature enough.

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
    [i2c-tools](https://i2c.wiki.kernel.org/index.php/I2C_Tools) and a wide range of userspace applications relying on the
    `/dev/i2c-x` kernel API.  
2.	The driver exposes the FT260 I2C controller device via `sys/bus/i2c`
    bus to other I2C devices relied on it. It allows the sysfs I2C tree
    instantiation, essential for complex I2C topologies, built with I2C
    multiplexers and/or I2C switches.
3.	It guarantees atomic access at the kernel level to the I2C devices,
    resided on the I2C controller's bus, implicitly enabling concurrent
    access to the I2C bus from multiple userspace processes instead of
    explicit contexts synchronization when such access is done via hidraw
    and libusb userspace libraries.

Specs:
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

If you got the below failure, your kernel does not support the hid_is_usb which was added by [https://lkml.org/lkml/2021/12/13/526](https://lkml.org/lkml/2021/12/13/526) commit.
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

### Change the I2C bus clock

You will need to figure out the ft260 device on the USB bus under sysfs.
Once you find it, you will see all available attributes.
For example, to set the i2c clock to 400KHz on my system, I run this command:

```
sudo bash -c 'echo 400 > /sys/bus/usb/devices/2-2\:1.0/0003\:0403\:6030.0002/clock'
```

### Set the multifunctional pin as GPIO

The FT260 has three pins that have more than two functions: DIO7 (pin 14),
DIO0 (pin 7), and DIO12 (pin 27). The function of these pins can be configured
by eFUSE, EEPROM, or via the sysfs interface:

In this example, I configure pin 14 to act as GPIO2:

```
$ cat /sys/bus/usb/devices/2-2\:1.0/0003\:0403\:6030.0002/gpio2_func
1
$ sudo bash -c "echo 0 > /sys/bus/usb/devices/2-2\:1.0/0003\:0403\:6030.0002/gpio2_func"
$ cat /sys/bus/usb/devices/2-2\:1.0/0003\:0403\:6030.0002/gpio2_func
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