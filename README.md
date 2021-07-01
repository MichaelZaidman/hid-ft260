# hid-ft260

### FTDI FT260 USB HID to I2C host bridge Linux kernel driver

The FTDI FT260 chip implements USB to I2C/UART bridges through two
USB HID class interfaces. The first - for I2C, and the second
for UART. Each interface is independent, and the kernel detects it
as a separate USB hidraw device.

There is no I2C master/host driver for this chip to date, and FTDI
suggests using hidraw and libusb userspace libraries to operate the
FT260 I2C host controller via hidraw Linux kernel driver. But this
approach makes the standard Linux I2C tools useless, and it does not
allow the I2C sysfs tree instantiation required by I2C multiplexers
and switches.

This driver enables the FT260 to be seen and handled by the Linux
kernel i2c-core code as a regular I2C bus adapter like it is done
when the I2C controller is a part of the MCU or CPU vendor chipset.
1.	It enables the standard Linux user-space I2C tools like [i2c-tools](https://i2c.wiki.kernel.org/index.php/I2C_Tools)
    and a wide range of user-space applications relying on the
    /dev/i2c-x kernel API.  
2.	The I2C master device is seen via sys/bus/i2c/devices sysfs
    interface that other I2C slave devices are relied on. This
    allows the sysfs I2C tree instantiation, essential for complex
    I2C topologies, built with I2C multiplexers or switches.
3.	It automatically handles I2C tree branch locking to guarantee
    atomic access to the slave device in the topologies involving
    I2C multiplexers or switches. That allows concurrent access to
    the I2C bus from multiple user-space processes. When such access
    is managed via hidraw and libusb user-space libraries, it requires
    an explicit synchronization between the processes.

Specs:
1. [DS_FT260.pdf](https://ftdichip.com/wp-content/uploads/2020/07/DS_FT260.pdf)
2. [AN_394_User_Guide_for_FT260.pdf](https://www.ftdichip.com/Support/Documents/AppNotes/AN_394_User_Guide_for_FT260.pdf)

The driver is merged into the Linux kernel 5.13 mainline.

1. Kernel - [https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/hid/hid-ft260.c?h=v5.13](https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/hid/hid-ft260.c?h=v5.13)
2. Commit - [https://lkml.org/lkml/2021/2/19/484](https://lkml.org/lkml/2021/2/19/484)
