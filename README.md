# hid-ft260

### FTDI FT260 USB HID to I2C host bridge Linux kernel driver

The FTDI FT260 chip implements USB to I2C/UART bridges through two
USB HID class interfaces. The first - for I2C, and the second
for UART. Each interface is independent, and the kernel detects it
as a separate USB hidraw device.

There is no Linux I2C controller driver for this chip to date, and FTDI
suggests using hidraw and libusb userspace libraries to operate the
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
    access to the I2C bus from multiple userspace processes, in contrast
    to when such access performs via hidraw and libusb userspace libraries
    requiring explicit processes synchronization.

Specs:
1. [DS_FT260.pdf](https://ftdichip.com/wp-content/uploads/2020/07/DS_FT260.pdf)
2. [AN_394_User_Guide_for_FT260.pdf](https://www.ftdichip.com/Support/Documents/AppNotes/AN_394_User_Guide_for_FT260.pdf)

The driver is merged into the Linux kernel 5.13 mainline.

1. Kernel - [https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/hid/hid-ft260.c?h=v5.13](https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/hid/hid-ft260.c?h=v5.13)
2. Commit - [https://lkml.org/lkml/2021/2/19/484](https://lkml.org/lkml/2021/2/19/484)
