#!/usr/bin/env python3
# Test-only: add fault injection to a copy of hid-ft260.c: the one of
# this repo, or drivers/hid/hid-ft260.c of a kernel tree. Never sent
# upstream. One module built from the result serves both groups below.
#
# Usage: ft260_harness.py IN.c OUT.c
#
# TN_189 probe failure and reset/probe overlap detector, used by
# tests/test_tn189_reset_race.py:
#   fake_bus, fake_dev  USB bus and device number of the board under test
#   fake_ifmask         bit N set: the next probe of interface N acts as if
#                       ft260_check_dev_responsive() returned -ENODEV; the
#                       bit is cleared, so the probes after the reset pass
#   fake_delay_ms       after a faked failure, sleep this long between
#                       ft260_schedule_reset() and hid_hw_stop()
# The reset work logs "harness: USB reset while N probes run" when it
# calls usb_reset_device() while a probe of that board is in ft260_probe().
#
# UART TX failure, used by tests/test_uart_tx_error.py:
#   fake_tx_fail        fail this many UART data output reports with
#                       -EIO, on any board, and log "harness: fake TX
#                       failure" for each
import sys

src, dst = sys.argv[1], sys.argv[2]
s = open(src).read()


def rep(old, new):
    global s
    assert s.count(old) == 1, old
    s = s.replace(old, new)


rep('MODULE_PARM_DESC(debug, "Toggle FT260 debugging messages");\n',
    '''MODULE_PARM_DESC(debug, "Toggle FT260 debugging messages");

/* hid-ft260 test harness, test-only */
#include <linux/delay.h>
static uint fake_bus;
module_param(fake_bus, uint, 0600);
static uint fake_dev;
module_param(fake_dev, uint, 0600);
static ulong fake_ifmask;
module_param(fake_ifmask, ulong, 0600);
static uint fake_delay_ms;
module_param(fake_delay_ms, uint, 0600);
static uint fake_tx_fail;
module_param(fake_tx_fail, uint, 0600);
static atomic_t ft260_fake_probes = ATOMIC_INIT(0);

static bool ft260_fake_match(struct usb_device *udev)
{
	return fake_dev && udev->bus->busnum == fake_bus &&
	       udev->devnum == fake_dev;
}
''')

rep('''	ret = ft260_check_dev_responsive(hdev);
	if (ret == -ENODEV) {
		ft260_schedule_reset(usbif);
		goto err_hid_stop;
	}
''', '''	if (ft260_fake_match(interface_to_usbdev(usbif)) &&
	    test_and_clear_bit(usbif->cur_altsetting->desc.bInterfaceNumber,
			       &fake_ifmask)) {
		dev_info(&usbif->dev, "harness: fake TN_189 failure\\n");
		ft260_schedule_reset(usbif);
		if (fake_delay_ms)
			msleep(fake_delay_ms);
		dev_info(&usbif->dev, "harness: probe calls hid_hw_stop\\n");
		ret = -ENODEV;
		goto err_hid_stop;
	}

	ret = ft260_check_dev_responsive(hdev);
	if (ret == -ENODEV) {
		ft260_schedule_reset(usbif);
		goto err_hid_stop;
	}
''')

rep('static int ft260_probe(struct hid_device *hdev, const struct hid_device_id *id)\n',
    'static int __ft260_probe(struct hid_device *hdev, const struct hid_device_id *id)\n')

rep('static void ft260_remove(struct hid_device *hdev)\n',
    '''static int ft260_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct usb_device *udev;
	bool match;
	int ret;

	if (!hid_is_usb(hdev))
		return -EINVAL;

	udev = interface_to_usbdev(to_usb_interface(hdev->dev.parent));
	match = ft260_fake_match(udev);
	if (match)
		atomic_inc(&ft260_fake_probes);
	ret = __ft260_probe(hdev, id);
	if (match)
		atomic_dec(&ft260_fake_probes);
	return ret;
}

static void ft260_remove(struct hid_device *hdev)
''')

rep('\tret = usb_reset_device(usbdev);\n',
    '''	if (ft260_fake_match(usbdev))
		dev_info(&usbif->dev, "harness: USB reset, %d probes run\\n",
			 atomic_read(&ft260_fake_probes));
	if (ft260_fake_match(usbdev) && atomic_read(&ft260_fake_probes))
		dev_warn(&usbif->dev, "harness: USB reset while %d probes run\\n",
			 atomic_read(&ft260_fake_probes));
	ret = usb_reset_device(usbdev);
''')

rep('\t\tret = ft260_hid_output_report(hdev, (u8 *)rep, len + 2);\n',
    '''		if (READ_ONCE(fake_tx_fail)) {
			WRITE_ONCE(fake_tx_fail, fake_tx_fail - 1);
			hid_info(hdev, "harness: fake TX failure\\n");
			ret = -EIO;
		} else {
			ret = ft260_hid_output_report(hdev, (u8 *)rep, len + 2);
		}
''')

open(dst, 'w').write(s)
