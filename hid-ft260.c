// SPDX-License-Identifier: GPL-2.0-only
/*
 * FTDI FT260 USB HID to I2C host bridge
 *
 * Copyright (c) 2021, Michael Zaidman <michaelz@xsightlabs.com>
 *
 * Data Sheet:
 *   https://www.ftdichip.com/Support/Documents/DataSheets/ICs/DS_FT260.pdf
 */

#include "hid-ids.h"
#include <linux/hidraw.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/gpio/driver.h>
#include <linux/uio_driver.h>

#ifdef DEBUG
static int ft260_debug = 1;
#else
static int ft260_debug;
#endif

module_param_named(debug, ft260_debug, int, 0600);
MODULE_PARM_DESC(debug, "Toggle FT260 debugging messages");

#define ft260_dbg(format, arg...)					  \
	do {								  \
		if (ft260_debug)					  \
			pr_info("%s: " format, __func__, ##arg);	  \
	} while (0)

#define FT260_REPORT_MAX_LENGTH (64)
#define FT260_I2C_DATA_REPORT_ID(len) (FT260_I2C_REPORT_MIN + ((len) - 1) / 4)

#define FT260_WAKEUP_NEEDED_AFTER_MS (4800) /* 5s minus 200ms margin */

/*
 * The ft260 input report format defines 62 bytes for the data payload, but
 * when requested 62 bytes, the controller returns 60 and 2 in separate input
 * reports. To achieve better performance with the multi-report read data
 * transfers, we set the maximum read payload length to a multiple of 60.
 * With a 100 kHz I2C clock, one 240 bytes read takes about 1/27 second,
 * which is excessive; On the other hand, some higher layer drivers like at24
 * or optoe limit the i2c reads to 128 bytes. To not block other drivers out
 * of I2C for potentially troublesome amounts of time, we select the maximum
 * read payload length to be 180 bytes.
 */
#define FT260_RD_DATA_MAX (180)
#define FT260_WR_DATA_MAX (60)

#define FT260_GPIOCHIP "ft260_gpio"
#define FT260_GPIO_MAX (6)
#define FT260_GPIO_EX_MAX (8)
#define FT260_GPIO_TOTAL (FT260_GPIO_MAX + FT260_GPIO_EX_MAX)
#define FT260_GPIO_MASK (~(0xffff << FT260_GPIO_TOTAL))

/*
 * Device interface configuration.
 * The FT260 has 2 interfaces that are controlled by DCNF0 and DCNF1 pins.
 * First implementes USB HID to I2C bridge function and
 * second - USB HID to UART bridge function.
 */
enum {
	FT260_MODE_ALL			= 0x00,
	FT260_MODE_I2C			= 0x01,
	FT260_MODE_UART			= 0x02,
	FT260_MODE_BOTH			= 0x03,
};

/* Control pipe */
enum {
	FT260_GET_RQST_TYPE		= 0xA1,
	FT260_GET_REPORT		= 0x01,
	FT260_SET_RQST_TYPE		= 0x21,
	FT260_SET_REPORT		= 0x09,
	FT260_FEATURE			= 0x03,
};

/* Report IDs / Feature In */
enum {
	FT260_CHIP_VERSION		= 0xA0,
	FT260_SYSTEM_SETTINGS		= 0xA1,
	FT260_I2C_STATUS		= 0xC0,
	FT260_I2C_READ_REQ		= 0xC2,
	FT260_I2C_REPORT_MIN		= 0xD0,
	FT260_I2C_REPORT_MAX		= 0xDE,
	FT260_GPIO			= 0xB0,
	FT260_UART_INTERRUPT_STATUS	= 0xB1,
	FT260_UART_STATUS		= 0xE0,
	FT260_UART_RI_DCD_STATUS	= 0xE1,
	FT260_UART_REPORT		= 0xF0,
};

/* Feature Out */
enum {
	FT260_SET_CLOCK			= 0x01,
	FT260_SET_I2C_MODE		= 0x02,
	FT260_SET_UART_MODE		= 0x03,
	FT260_ENABLE_INTERRUPT		= 0x05,
	FT260_SELECT_GPIO2_FUNC		= 0x06,
	FT260_ENABLE_UART_DCD_RI	= 0x07,
	FT260_SELECT_GPIOA_FUNC		= 0x08,
	FT260_SELECT_GPIOG_FUNC		= 0x09,
	FT260_SET_INTERRUPT_TRIGGER	= 0x0A,
	FT260_SET_SUSPEND_OUT_POLAR	= 0x0B,
	FT260_ENABLE_UART_RI_WAKEUP	= 0x0C,
	FT260_SET_UART_RI_WAKEUP_CFG	= 0x0D,
	FT260_SET_I2C_RESET		= 0x20,
	FT260_SET_I2C_CLOCK_SPEED	= 0x22,
	FT260_SET_UART_RESET		= 0x40,
	FT260_SET_UART_CONFIG		= 0x41,
	FT260_SET_UART_BAUD_RATE	= 0x42,
	FT260_SET_UART_DATA_BIT		= 0x43,
	FT260_SET_UART_PARITY		= 0x44,
	FT260_SET_UART_STOP_BIT		= 0x45,
	FT260_SET_UART_BREAKING		= 0x46,
	FT260_SET_UART_XON_XOFF		= 0x49,
};

/* Response codes in I2C status report */
enum {
	FT260_I2C_STATUS_SUCCESS	= 0x00,
	FT260_I2C_STATUS_CTRL_BUSY	= 0x01,
	FT260_I2C_STATUS_ERROR		= 0x02,
	FT260_I2C_STATUS_ADDR_NO_ACK	= 0x04,
	FT260_I2C_STATUS_DATA_NO_ACK	= 0x08,
	FT260_I2C_STATUS_ARBITR_LOST	= 0x10,
	FT260_I2C_STATUS_CTRL_IDLE	= 0x20,
	FT260_I2C_STATUS_BUS_BUSY	= 0x40,
};

/* I2C Conditions flags */
enum {
	FT260_FLAG_NONE			= 0x00,
	FT260_FLAG_START		= 0x02,
	FT260_FLAG_START_REPEATED	= 0x03,
	FT260_FLAG_STOP			= 0x04,
	FT260_FLAG_START_STOP		= 0x06,
	FT260_FLAG_START_STOP_REPEATED	= 0x07,
};

/* Multi-function pin functions */
enum {
	FT260_MFPIN_GPIO		= 0x00,
	FT260_MFPIN_SUSPOUT		= 0x01,
	FT260_MFPIN_PWREN		= 0x02,
	FT260_MFPIN_TX_ACTIVE		= 0x03,
	FT260_MFPIN_TX_LED		= 0x04,
	FT260_MFPIN_RX_LED		= 0x05,
	FT260_MFPIN_BCD_DET		= 0x06,
};

enum {
	FT260_GPIO_VALUE		= 0x00,
	FT260_GPIO_DIRECTION		= 0x01,
	FT260_GPIO_DIR_INPUT		= 0x00,
	FT260_GPIO_DIR_OUTPUT		= 0x01,
};

/* GPIO offsets */
enum {
	FT260_GPIO_0			= (1 << 0),
	FT260_GPIO_1			= (1 << 1),
	FT260_GPIO_2			= (1 << 2),
	FT260_GPIO_3			= (1 << 3),
	FT260_GPIO_4			= (1 << 4),
	FT260_GPIO_5			= (1 << 5),
	FT260_GPIO_A			= (1 << (FT260_GPIO_MAX + 0)),
	FT260_GPIO_B			= (1 << (FT260_GPIO_MAX + 1)),
	FT260_GPIO_C			= (1 << (FT260_GPIO_MAX + 2)),
	FT260_GPIO_D			= (1 << (FT260_GPIO_MAX + 3)),
	FT260_GPIO_E			= (1 << (FT260_GPIO_MAX + 4)),
	FT260_GPIO_F			= (1 << (FT260_GPIO_MAX + 5)),
	FT260_GPIO_G			= (1 << (FT260_GPIO_MAX + 6)),
	FT260_GPIO_H			= (1 << (FT260_GPIO_MAX + 7)),
};

/* GPIO groups */
enum {
	FT260_GPIO_WAKEUP		= (FT260_GPIO_3),
	FT260_GPIO_I2C_DEFAULT		= (FT260_GPIO_0 | FT260_GPIO_1),
	FT260_GPIO_UART_DCD_RI		= (FT260_GPIO_4 | FT260_GPIO_5),
	FT260_GPIO_UART			= (FT260_GPIO_B | FT260_GPIO_C |
					   FT260_GPIO_D | FT260_GPIO_E |
					   FT260_GPIO_F | FT260_GPIO_H),
	FT260_GPIO_UART_DEFAULT		= (FT260_GPIO_UART |
					   FT260_GPIO_UART_DCD_RI),
};

enum {
	FT260_I2C_DISABLE = 0,
	FT260_I2C_ENABLE = 1,
};

enum {
	FT260_UART_MODE_OFF = 0,
	FT260_UART_MODE_RTS_CTS = 1,
	FT260_UART_MODE_DTR_DSR = 2,
	FT260_UART_MODE_XON_XOFF = 3,
};

enum {
	FT260_WAKEUP_INTERUP_DISABLE = 0,
	FT260_WAKEUP_INTERUP_ENABLE = 1,
};

/* Interrupt trigger conditions */
enum {
	FT260_INTR_COND_RISING_EDGE = 0,
	FT260_INTR_COND_LEVEL_HIGH = 1,
	FT260_INTR_COND_FALLING_EDGE = 2,
	FT260_INTR_COND_LEVEL_LOW = 3,
};

/* Interupt level duration, when trigger cond is level high/low */
enum {
	FT260_INTR_COND_DELAY_1MS = 1,
	FT260_INTR_COND_DELAY_5MS = 2,
	FT260_INTR_COND_DELAY_30MS = 3,
};


enum {
	FT260_INTERFACE_I2C = 0,
	FT260_INTERFACE_UART = 1,
};

#define FT260_SET_REQUEST_VALUE(report_id) ((FT260_FEATURE << 8) | (report_id))

/* Feature In reports */

struct ft260_get_chip_version_report {
	u8 report;		/* FT260_CHIP_VERSION */
	u8 chip_code[4];	/* FTDI chip identification code */
	u8 reserved[8];
} __packed;

struct ft260_get_system_status_report {
	u8 report;		/* FT260_SYSTEM_SETTINGS */
	u8 chip_mode;		/* DCNF0 and DCNF1 status, bits 0-1 */
	u8 clock_ctl;		/* 0 - 12MHz, 1 - 24MHz, 2 - 48MHz */
	u8 suspend_status;	/* 0 - not suspended, 1 - suspended */
	u8 pwren_status;	/* 0 - FT260 is not ready, 1 - ready */
	u8 i2c_enable;		/* 0 - disabled, 1 - enabled */
	u8 uart_mode;		/* 0 - OFF; 1 - RTS_CTS, 2 - DTR_DSR, */
				/* 3 - XON_XOFF, 4 - No flow control */
	u8 hid_over_i2c_en;	/* 0 - disabled, 1 - enabled */
	u8 gpio2_func;		/* 0 - GPIO,  1 - SUSPOUT, */
				/* 2 - PWREN, 4 - TX_LED */
	u8 gpioa_func;		/* 0 - GPIO, 3 - TX_ACTIVE, 4 - TX_LED */
	u8 gpiog_func;		/* 0 - GPIO, 2 - PWREN, */
				/* 5 - RX_LED, 6 - BCD_DET */
	u8 suspend_out_pol;	/* 0 - active-high, 1 - active-low */
	u8 enable_wakeup_int;	/* 0 - disabled, 1 - enabled */
	u8 intr_cond;		/* Interrupt trigger conditions */
	u8 power_saving_en;	/* 0 - disabled, 1 - enabled */
	u8 reserved[10];
} __packed;

struct ft260_get_i2c_status_report {
	u8 report;		/* FT260_I2C_STATUS */
	u8 bus_status;		/* I2C bus status */
	__le16 clock;		/* I2C bus clock in range 60-3400 KHz */
	u8 reserved;
} __packed;

struct ft260_gpio_state {
	u8 vals;		/* GPIO[0-5] values in bits 0 - 5 */
	u8 dirs;		/* GPIO[0-5] directions, 0 - in, 1 - out */
	u8 ex_vals;		/* GPIO[A-H] values in bits 0 - 7 */
	u8 ex_dirs;		/* GPIO[A-H] directions, 0 - in, 1 - out */
} __packed;

struct ft260_gpio_read_request_report {
	u8 report;		/* FT260_GPIO */
	struct ft260_gpio_state	gpio;
} __packed;

/* Feature Out reports */

struct ft260_set_system_clock_report {
	u8 report;		/* FT260_SYSTEM_SETTINGS */
	u8 request;		/* FT260_SET_CLOCK */
	u8 clock_ctl;		/* 0 - 12MHz, 1 - 24MHz, 2 - 48MHz */
} __packed;

struct ft260_set_i2c_mode_report {
	u8 report;		/* FT260_SYSTEM_SETTINGS */
	u8 request;		/* FT260_SET_I2C_MODE */
	u8 i2c_enable;		/* 0 - disabled, 1 - enabled */
} __packed;

struct ft260_set_uart_mode_report {
	u8 report;		/* FT260_SYSTEM_SETTINGS */
	u8 request;		/* FT260_SET_UART_MODE */
	u8 uart_mode;		/* 0 - OFF; 1 - RTS_CTS, 2 - DTR_DSR, */
				/* 3 - XON_XOFF, 4 - No flow control */
} __packed;

struct ft260_set_enable_interrupt_report {
	u8 report;		/* FT260_SYSTEM_SETTINGS */
	u8 request;		/* Enable Interrupt/Wake up */
	u8 enable_wakeup_int;		/* 0 GPIO/1 Wakeup/Interupt	*/
} __packed;


struct ft260_set_interrupt_trigger_cond_report {
	u8 report;		/* FT260_SYSTEM_SETTINGS */
	u8 request;		/* Set Interrupt Trigger Condition */
	u16 intr_cond;		/* First byte = trigger condition */
				/* 0 - rising e, 1 - lvl high, 2 - falling e, 3 - lvl low */
				/* Second byte = duration for level high/low */
				/* 1 - 1ms, 2 - 5ms, 3 - 30ms */
} __packed;

struct ft260_set_i2c_reset_report {
	u8 report;		/* FT260_SYSTEM_SETTINGS */
	u8 request;		/* FT260_SET_I2C_RESET */
} __packed;

struct ft260_set_i2c_speed_report {
	u8 report;		/* FT260_SYSTEM_SETTINGS */
	u8 request;		/* FT260_SET_I2C_CLOCK_SPEED */
	__le16 clock;		/* I2C bus clock in range 60-3400 KHz */
} __packed;

struct ft260_set_gpio2_func_report {
	u8 report;		/* FT260_SYSTEM_SETTINGS */
	u8 request;		/* FT260_SELECT_GPIO2_FUNC */
	u8 gpio2_func;		/* Pin func: 0 - GPIO, 1 - SUSPOUT, */
				/* 2 - PWREN# (active-low), 4 - TX_LED */
} __packed;

struct ft260_set_gpioa_func_report {
	u8 report;		/* FT260_SYSTEM_SETTINGS */
	u8 request;		/* FT260_SELECT_GPIOA_FUNC */
	u8 gpioa_func;		/* Pin func: 0 - GPIO, */
				/* 3 - TX_ACTIVE, 4 - TX_LED */
} __packed;

struct ft260_set_gpiog_func_report {
	u8 report;		/* FT260_SYSTEM_SETTINGS */
	u8 request;		/* FT260_SELECT_GPIOG_FUNC */
	u8 gpiog_func;		/* Pin func: 0 - GPIO, */
				/* 2 - PWREN# (active-low), */
				/* 5 - RX_LED, 6 - BCD_DET */
} __packed;

struct ft260_gpio_write_request_report {
	u8 report;		/* FT260_GPIO */
	struct ft260_gpio_state	gpio;
} __packed;

/* Data transfer reports */

struct ft260_i2c_write_request_report {
	u8 report;		/* FT260_I2C_REPORT */
	u8 address;		/* 7-bit I2C address */
	u8 flag;		/* I2C transaction condition */
	u8 length;		/* data payload length */
	u8 data[FT260_WR_DATA_MAX]; /* data payload */
} __packed;

struct ft260_i2c_read_request_report {
	u8 report;		/* FT260_I2C_READ_REQ */
	u8 address;		/* 7-bit I2C address */
	u8 flag;		/* I2C transaction condition */
	__le16 length;		/* data payload length */
} __packed;

struct ft260_i2c_input_report {
	u8 report;		/* FT260_I2C_REPORT */
	u8 length;		/* data payload length */
	u8 data[2];		/* data payload */
} __packed;

static const struct hid_device_id ft260_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_FUTURE_TECHNOLOGY,
			 USB_DEVICE_ID_FT260) },
	{ /* END OF LIST */ }
};
MODULE_DEVICE_TABLE(hid, ft260_devices);

struct ft260_device {
	struct i2c_adapter adap;
	struct hid_device *hdev;
	u8 interface;
	struct completion wait;
	struct gpio_chip *gc;
	struct mutex lock;
	u8 write_buf[FT260_REPORT_MAX_LENGTH];
	u8 feature_buf[FT260_REPORT_MAX_LENGTH];
	unsigned long need_wakeup_at;
	u8 *read_buf;
	u16 read_idx;
	u16 read_len;
	u16 clock;
	struct ft260_gpio_state gpio;
	u16 gpio_en;
	struct uio_info uio;
};

static int ft260_hid_feature_report_get(struct hid_device *hdev,
					unsigned char report_id, u8 *data,
					size_t len)
{
	u8 *buf;
	int ret;

	buf = kmalloc(len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = hid_hw_raw_request(hdev, report_id, buf, len, HID_FEATURE_REPORT,
				 HID_REQ_GET_REPORT);
	if (likely(ret == len))
		memcpy(data, buf, len);
	else if (ret >= 0)
		ret = -EIO;
	kfree(buf);
	return ret;
}

static int ft260_hid_feature_report_set(struct hid_device *hdev, u8 *data,
					size_t len)
{
	u8 *buf;
	int ret;

	buf = kmemdup(data, len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = hid_hw_raw_request(hdev, buf[0], buf, len, HID_FEATURE_REPORT,
				 HID_REQ_SET_REPORT);

	kfree(buf);
	return ret;
}

static int ft260_i2c_reset(struct hid_device *hdev)
{
	struct ft260_set_i2c_reset_report report;
	int ret;

	report.report = FT260_SYSTEM_SETTINGS;
	report.request = FT260_SET_I2C_RESET;

	ret = ft260_hid_feature_report_set(hdev, (u8 *)&report, sizeof(report));
	if (ret < 0) {
		hid_err(hdev, "failed to reset I2C controller: %d\n", ret);
		return ret;
	}

	ft260_dbg("done\n");
	return ret;
}

static int ft260_xfer_status(struct ft260_device *dev, u8 bus_busy)
{
	struct hid_device *hdev = dev->hdev;
	struct ft260_get_i2c_status_report report;
	int ret;

	if (time_is_before_jiffies(dev->need_wakeup_at)) {
		ret = ft260_hid_feature_report_get(hdev, FT260_I2C_STATUS,
						(u8 *)&report, sizeof(report));
		if (unlikely(ret < 0)) {
			hid_err(hdev, "failed to retrieve status: %d, no wakeup\n",
				ret);
		} else {
			dev->need_wakeup_at = jiffies +
				msecs_to_jiffies(FT260_WAKEUP_NEEDED_AFTER_MS);
			ft260_dbg("bus_status %#02x, wakeup\n",
				  report.bus_status);
		}
	}

	ret = ft260_hid_feature_report_get(hdev, FT260_I2C_STATUS,
					   (u8 *)&report, sizeof(report));
	if (unlikely(ret < 0)) {
		hid_err(hdev, "failed to retrieve status: %d\n", ret);
		return ret;
	}

	dev->clock = le16_to_cpu(report.clock);
	ft260_dbg("bus_status %#02x, clock %u\n", report.bus_status,
		  dev->clock);

	if (report.bus_status & (FT260_I2C_STATUS_CTRL_BUSY | bus_busy))
		return -EAGAIN;

	/*
	 * The error condition (bit 1) is a status bit reflecting any
	 * error conditions. When any of the bits 2, 3, or 4 are raised
	 * to 1, bit 1 is also set to 1.
	 */
	if (report.bus_status & FT260_I2C_STATUS_ERROR) {
		hid_err(hdev, "i2c bus error: %#02x\n", report.bus_status);
		return -EIO;
	}

	return 0;
}

static int ft260_hid_output_report(struct hid_device *hdev, u8 *data,
				   size_t len)
{
	u8 *buf;
	int ret;

	buf = kmemdup(data, len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = hid_hw_output_report(hdev, buf, len);

	kfree(buf);
	return ret;
}

static int ft260_hid_output_report_check_status(struct ft260_device *dev,
						u8 *data, int len)
{
	u8 bus_busy;
	int ret, usec, try = 100;
	struct hid_device *hdev = dev->hdev;
	struct ft260_i2c_write_request_report *rep =
		(struct ft260_i2c_write_request_report *)data;

	ret = ft260_hid_output_report(hdev, data, len);
	if (ret < 0) {
		hid_err(hdev, "%s: failed to start transfer, ret %d\n",
			__func__, ret);
		ft260_i2c_reset(hdev);
		return ret;
	}

	/* transfer time = 1 / clock(KHz) * 9 bits * bytes */
	usec = len * 9000 / dev->clock;
	if (usec > 2000) {
		usec -= 1500;
		usleep_range(usec, usec + 100);
		ft260_dbg("wait %d usec, len %d\n", usec, len);
	}

	/*
	 * Do not check the busy bit for combined transactions
	 * since the controller keeps the bus busy between writing
	 * and reading IOs to ensure an atomic operation.
	 */
	if (rep->flag == FT260_FLAG_START)
		bus_busy = 0;
	else
		bus_busy = FT260_I2C_STATUS_BUS_BUSY;

	do {
		ret = ft260_xfer_status(dev, bus_busy);
		if (ret != -EAGAIN)
			break;
	} while (--try);

	if (ret == 0)
		return 0;

	ft260_i2c_reset(hdev);
	return -EIO;
}

static int ft260_i2c_write(struct ft260_device *dev, u8 addr, u8 *data,
			   int len, u8 flag)
{
	int ret, wr_len, idx = 0;
	struct hid_device *hdev = dev->hdev;
	struct ft260_i2c_write_request_report *rep =
		(struct ft260_i2c_write_request_report *)dev->write_buf;

	if (len < 1)
		return -EINVAL;

	rep->flag = FT260_FLAG_START;

	do {
		if (len <= FT260_WR_DATA_MAX) {
			wr_len = len;
			if (flag == FT260_FLAG_START_STOP)
				rep->flag |= FT260_FLAG_STOP;
		} else {
			wr_len = FT260_WR_DATA_MAX;
		}

		rep->report = FT260_I2C_DATA_REPORT_ID(wr_len);
		rep->address = addr;
		rep->length = wr_len;

		memcpy(rep->data, &data[idx], wr_len);

		ft260_dbg("rep %#02x addr %#02x off %d len %d wlen %d flag %#x d[0] %#02x\n",
			  rep->report, addr, idx, len, wr_len,
			  rep->flag, data[0]);

		ret = ft260_hid_output_report_check_status(dev, (u8 *)rep,
							   wr_len + 4);
		if (ret < 0) {
			hid_err(hdev, "%s: failed with %d\n", __func__, ret);
			return ret;
		}

		len -= wr_len;
		idx += wr_len;
		rep->flag = 0;

	} while (len > 0);

	return 0;
}

static int ft260_smbus_write(struct ft260_device *dev, u8 addr, u8 cmd,
				 u8 *data, u8 data_len, u8 flag)
{
	int ret = 0;
	int len = 4;

	struct ft260_i2c_write_request_report *rep =
		(struct ft260_i2c_write_request_report *)dev->write_buf;

	if (data_len >= sizeof(rep->data))
		return -EINVAL;

	rep->address = addr;
	rep->data[0] = cmd;
	rep->length = data_len + 1;
	rep->flag = flag;
	len += rep->length;

	rep->report = FT260_I2C_DATA_REPORT_ID(len);

	if (data_len > 0)
		memcpy(&rep->data[1], data, data_len);

	ft260_dbg("rep %#02x addr %#02x cmd %#02x datlen %d replen %d\n",
		  rep->report, addr, cmd, rep->length, len);

	ret = ft260_hid_output_report_check_status(dev, (u8 *)rep, len);

	return ret;
}

static int ft260_i2c_read(struct ft260_device *dev, u8 addr, u8 *data,
			  u16 len, u8 flag)
{
	u16 rd_len;
	u16 rd_data_max = 60;
	int timeout, ret = 0;
	struct ft260_i2c_read_request_report rep;
	struct hid_device *hdev = dev->hdev;
	u8 bus_busy = 0;

	if ((flag & FT260_FLAG_START_REPEATED) == FT260_FLAG_START_REPEATED)
		flag = FT260_FLAG_START_REPEATED;
	else
		flag = FT260_FLAG_START;
	do {
		if (len <= rd_data_max) {
			rd_len = len;
			flag |= FT260_FLAG_STOP;
		} else {
			rd_len = rd_data_max;
		}
		rd_data_max = FT260_RD_DATA_MAX;

		rep.report = FT260_I2C_READ_REQ;
		rep.length = cpu_to_le16(rd_len);
		rep.address = addr;
		rep.flag = flag;

		ft260_dbg("rep %#02x addr %#02x len %d rlen %d flag %#x\n",
			  rep.report, rep.address, len, rd_len, flag);

		reinit_completion(&dev->wait);

		dev->read_idx = 0;
		dev->read_buf = data;
		dev->read_len = rd_len;

		ret = ft260_hid_output_report(hdev, (u8 *)&rep, sizeof(rep));
		if (ret < 0) {
			hid_err(hdev, "%s: failed with %d\n", __func__, ret);
			goto ft260_i2c_read_exit;
		}

		timeout = msecs_to_jiffies(5000);
		if (!wait_for_completion_timeout(&dev->wait, timeout)) {
			ret = -ETIMEDOUT;
			ft260_i2c_reset(hdev);
			goto ft260_i2c_read_exit;
		}

		dev->read_buf = NULL;

		if (flag & FT260_FLAG_STOP)
			bus_busy = FT260_I2C_STATUS_BUS_BUSY;

		ret = ft260_xfer_status(dev, bus_busy);
		if (ret < 0) {
			ret = -EIO;
			ft260_i2c_reset(hdev);
			goto ft260_i2c_read_exit;
		}

		len -= rd_len;
		data += rd_len;
		flag = 0;

	} while (len > 0);

ft260_i2c_read_exit:
	dev->read_buf = NULL;
	return ret;
}

/*
 * A random read operation is implemented as a dummy write operation, followed
 * by a current address read operation. The dummy write operation is used to
 * load the target byte address into the current byte address counter, from
 * which the subsequent current address read operation then reads.
 */
static int ft260_i2c_write_read(struct ft260_device *dev, struct i2c_msg *msgs)
{
	int ret;
	int wr_len = msgs[0].len;
	int rd_len = msgs[1].len;
	struct hid_device *hdev = dev->hdev;
	u8 addr = msgs[0].addr;
	u16 read_off = 0;

	if (wr_len > 2) {
		hid_err(hdev, "%s: invalid wr_len: %d\n", __func__, wr_len);
		return -EOPNOTSUPP;
	}

	if (ft260_debug) {
		if (wr_len == 2)
			read_off = be16_to_cpu(*(__be16 *)msgs[0].buf);
		else
			read_off = *msgs[0].buf;

		pr_info("%s: off %#x rlen %d wlen %d\n", __func__,
			read_off, rd_len, wr_len);
	}

	ret = ft260_i2c_write(dev, addr, msgs[0].buf, wr_len,
				  FT260_FLAG_START);
	if (ret < 0)
		return ret;

	ret = ft260_i2c_read(dev, addr, msgs[1].buf, rd_len,
				 FT260_FLAG_START_STOP_REPEATED);
	if (ret < 0)
		return ret;

	return 0;
}

static int ft260_i2c_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs,
			  int num)
{
	int ret;
	struct ft260_device *dev = i2c_get_adapdata(adapter);
	struct hid_device *hdev = dev->hdev;

	mutex_lock(&dev->lock);

	ret = hid_hw_power(hdev, PM_HINT_FULLON);
	if (ret < 0) {
		hid_err(hdev, "failed to enter FULLON power mode: %d\n", ret);
		mutex_unlock(&dev->lock);
		return ret;
	}

	if (num == 1) {
		if (msgs->flags & I2C_M_RD)
			ret = ft260_i2c_read(dev, msgs->addr, msgs->buf,
						 msgs->len, FT260_FLAG_START_STOP);
		else
			ret = ft260_i2c_write(dev, msgs->addr, msgs->buf,
						  msgs->len, FT260_FLAG_START_STOP);
		if (ret < 0)
			goto i2c_exit;

	} else {
		/* Combined write then read message */
		ret = ft260_i2c_write_read(dev, msgs);
		if (ret < 0)
			goto i2c_exit;
	}

	ret = num;
i2c_exit:
	hid_hw_power(hdev, PM_HINT_NORMAL);
	mutex_unlock(&dev->lock);
	return ret;
}

static int ft260_smbus_xfer(struct i2c_adapter *adapter, u16 addr, u16 flags,
				char read_write, u8 cmd, int size,
				union i2c_smbus_data *data)
{
	int ret;
	struct ft260_device *dev = i2c_get_adapdata(adapter);
	struct hid_device *hdev = dev->hdev;

	ft260_dbg("smbus size %d\n", size);

	mutex_lock(&dev->lock);

	ret = hid_hw_power(hdev, PM_HINT_FULLON);
	if (ret < 0) {
		hid_err(hdev, "power management error: %d\n", ret);
		mutex_unlock(&dev->lock);
		return ret;
	}

	switch (size) {
	case I2C_SMBUS_BYTE:
		if (read_write == I2C_SMBUS_READ)
			ret = ft260_i2c_read(dev, addr, &data->byte, 1,
						 FT260_FLAG_START_STOP);
		else
			ret = ft260_smbus_write(dev, addr, cmd, NULL, 0,
						FT260_FLAG_START_STOP);
		break;
	case I2C_SMBUS_BYTE_DATA:
		if (read_write == I2C_SMBUS_READ) {
			ret = ft260_smbus_write(dev, addr, cmd, NULL, 0,
						FT260_FLAG_START);
			if (ret)
				goto smbus_exit;

			ret = ft260_i2c_read(dev, addr, &data->byte, 1,
						 FT260_FLAG_START_STOP_REPEATED);
		} else {
			ret = ft260_smbus_write(dev, addr, cmd, &data->byte, 1,
						FT260_FLAG_START_STOP);
		}
		break;
	case I2C_SMBUS_WORD_DATA:
		if (read_write == I2C_SMBUS_READ) {
			ret = ft260_smbus_write(dev, addr, cmd, NULL, 0,
						FT260_FLAG_START);
			if (ret)
				goto smbus_exit;

			ret = ft260_i2c_read(dev, addr, (u8 *)&data->word, 2,
						 FT260_FLAG_START_STOP_REPEATED);
		} else {
			ret = ft260_smbus_write(dev, addr, cmd,
						(u8 *)&data->word, 2,
						FT260_FLAG_START_STOP);
		}
		break;
	case I2C_SMBUS_BLOCK_DATA:
		if (read_write == I2C_SMBUS_READ) {
			ret = ft260_smbus_write(dev, addr, cmd, NULL, 0,
						FT260_FLAG_START);
			if (ret)
				goto smbus_exit;

			ret = ft260_i2c_read(dev, addr, data->block,
						 data->block[0] + 1,
						 FT260_FLAG_START_STOP_REPEATED);
		} else {
			ret = ft260_smbus_write(dev, addr, cmd, data->block,
						data->block[0] + 1,
						FT260_FLAG_START_STOP);
		}
		break;
	case I2C_SMBUS_I2C_BLOCK_DATA:
		if (read_write == I2C_SMBUS_READ) {
			ret = ft260_smbus_write(dev, addr, cmd, NULL, 0,
						FT260_FLAG_START);
			if (ret)
				goto smbus_exit;

			ret = ft260_i2c_read(dev, addr, data->block + 1,
						 data->block[0],
						 FT260_FLAG_START_STOP_REPEATED);
		} else {
			ret = ft260_smbus_write(dev, addr, cmd, data->block + 1,
						data->block[0],
						FT260_FLAG_START_STOP);
		}
		break;
	default:
		hid_err(hdev, "unsupported smbus transaction size %d\n", size);
		ret = -EOPNOTSUPP;
	}

smbus_exit:
	hid_hw_power(hdev, PM_HINT_NORMAL);
	mutex_unlock(&dev->lock);
	return ret;
}

static u32 ft260_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE |
		   I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA |
		   I2C_FUNC_SMBUS_BLOCK_DATA | I2C_FUNC_SMBUS_I2C_BLOCK;
}

static const struct i2c_adapter_quirks ft260_i2c_quirks = {
	.flags = I2C_AQ_COMB_WRITE_THEN_READ,
	.max_comb_1st_msg_len = 2,
};

static const struct i2c_algorithm ft260_i2c_algo = {
	.master_xfer = ft260_i2c_xfer,
	.smbus_xfer = ft260_smbus_xfer,
	.functionality = ft260_functionality,
};


static int ft260_gpio_chip_match_name(struct gpio_chip *chip, void *data)
{
	return !strcmp(chip->label, data);
}

static void ft260_gpio_en_set(struct ft260_device *dev, u16 bitmap)
{
	dev->gpio_en |= bitmap & FT260_GPIO_MASK;
}

static void ft260_gpio_en_clr(struct ft260_device *dev, u16 bitmap)
{
	dev->gpio_en &= ~bitmap & FT260_GPIO_MASK;
}

static void ft260_gpio_en_update(struct hid_device *hdev, u8 req, u8 value)
{
	u16 bitmap;
	struct ft260_device *dev = hid_get_drvdata(hdev);

	switch (req) {
	case FT260_SELECT_GPIO2_FUNC:
		bitmap = FT260_GPIO_2;
		break;
	case FT260_SELECT_GPIOA_FUNC:
		bitmap = FT260_GPIO_A;
		break;
	case FT260_SELECT_GPIOG_FUNC:
		bitmap = FT260_GPIO_G;
		break;
	default:
		return;
	}

	if (value == FT260_MFPIN_GPIO)
		ft260_gpio_en_set(dev, bitmap);
	else
		ft260_gpio_en_clr(dev, bitmap);

	hid_info(hdev, "enabled GPIOs: %04x\n", dev->gpio_en);
}

/*
 * For GPIO, we use hid_hw_raw_request directly with preallocated buffer to not
 * interfere with i2c operation.
 */
static void ft260_gpio_set(struct gpio_chip *gc, u32 offset, int value)
{
	int ret;
	struct ft260_gpio_write_request_report *rep;
	int len = sizeof(struct ft260_gpio_read_request_report);
	struct ft260_device *dev = gpiochip_get_data(gc);
	struct hid_device *hdev = dev->hdev;

	if (offset >= FT260_GPIO_TOTAL) {
		hid_err(hdev, "%s: invalid offset %d\n", __func__, offset);
		return;
	}

	ft260_dbg("offset %d val %d\n", offset, value);

	mutex_lock(&dev->lock);

	if (!(dev->gpio_en & (1 << offset))) {
		hid_err(hdev, "%s: wrong pin function %d\n", __func__, offset);
		goto exit;
	}

	rep = (struct ft260_gpio_write_request_report *)&dev->feature_buf;

	rep->gpio = dev->gpio;

	if (offset < FT260_GPIO_MAX) {
		if (value)
			rep->gpio.vals |= !!value << offset;
		else
			rep->gpio.vals &= ~(1 << offset);
	} else {
		offset = offset - FT260_GPIO_MAX;
		if (value)
			rep->gpio.ex_vals |= !!value << offset;
		else
			rep->gpio.ex_vals &= ~(1 << offset);
	}

	ft260_dbg("dirs %#02x vals %#02x ex_dir %#02x ex_vals %#02x\n",
		  rep->gpio.dirs, rep->gpio.vals,
		  rep->gpio.ex_dirs, rep->gpio.ex_vals);

	ret = hid_hw_raw_request(hdev, FT260_GPIO, (u8 *)rep, len,
				 HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	if (ret < 0) {
		hid_err(hdev, "%s: error setting GPIO: %d\n", __func__, ret);
		goto exit;
	}

	dev->gpio = rep->gpio;
exit:
	mutex_unlock(&dev->lock);
}

static int ft260_gpio_direction_set(struct gpio_chip *gc, u32 offset,
					int value, int direction)
{
	int ret;
	u8 *buf;
	struct ft260_gpio_write_request_report *rep;
	int len = sizeof(struct ft260_gpio_read_request_report);
	struct ft260_device *dev = gpiochip_get_data(gc);
	struct hid_device *hdev = dev->hdev;

	if (offset >= FT260_GPIO_TOTAL) {
		hid_err(hdev, "%s: invalid offset %d\n", __func__, offset);
		return -EINVAL;
	}

	ft260_dbg("offset %d val %d direction %d\n", offset, value, direction);

	mutex_lock(&dev->lock);

	if (!(dev->gpio_en & (1 << offset))) {
		hid_err(hdev, "%s: wrong pin function %d\n", __func__, offset);
		ret = -EIO;
		goto exit;
	}

	buf = (u8 *)&dev->feature_buf;

	ret = hid_hw_raw_request(hdev, FT260_GPIO, buf, len,
				 HID_FEATURE_REPORT, HID_REQ_GET_REPORT);
	if (ret != len) {
		ret = ret < 0 ? ret : -EIO;
		hid_err(hdev, "%s: error getting GPIO: %d\n", __func__, ret);
		goto exit;
	}

	rep = (struct ft260_gpio_write_request_report *)buf;
	len = sizeof(struct ft260_gpio_write_request_report);

	if (direction == FT260_GPIO_DIR_OUTPUT)
		if (offset < FT260_GPIO_MAX)
			rep->gpio.dirs |= 1 << offset;
		else
			rep->gpio.ex_dirs |= 1 << (offset - FT260_GPIO_MAX);
	else
		if (offset < FT260_GPIO_MAX)
			rep->gpio.dirs &= ~(1 << offset);
		else
			rep->gpio.ex_dirs &= ~(1 << (offset - FT260_GPIO_MAX));

	ft260_dbg("dirs %#02x val %#02x ex_dirs %#02x ex_vals %#02x\n",
		  rep->gpio.dirs, rep->gpio.vals,
		  rep->gpio.ex_dirs, rep->gpio.ex_vals);

	ret = hid_hw_raw_request(hdev, FT260_GPIO, buf, len,
				 HID_FEATURE_REPORT, HID_REQ_SET_REPORT);

	if (ret < 0) {
		hid_err(hdev, "%s: error setting GPIO: %d\n", __func__, ret);
		goto exit;
	}

	dev->gpio = rep->gpio;
	mutex_unlock(&dev->lock);

	if (direction == FT260_GPIO_DIR_OUTPUT)
		ft260_gpio_set(gc, offset, value);

	return 0;
exit:
	mutex_unlock(&dev->lock);
	return ret;
}

static int ft260_gpio_direction_output(struct gpio_chip *gc,
					   u32 offset, int value)
{
	return ft260_gpio_direction_set(gc, offset, value,
					FT260_GPIO_DIR_OUTPUT);
}

static int ft260_gpio_direction_input(struct gpio_chip *gc, u32 offset)
{
	return ft260_gpio_direction_set(gc, offset, 0,
					FT260_GPIO_DIR_INPUT);
}

static int ft260_gpio_get_all(struct gpio_chip *gc, int item)
{
	int ret;
	u8 *buf;
	struct ft260_gpio_read_request_report *rep;
	int len = sizeof(struct ft260_gpio_read_request_report);
	struct ft260_device *dev = gpiochip_get_data(gc);
	struct hid_device *hdev = dev->hdev;

	buf = (u8 *)&dev->feature_buf;

	mutex_lock(&dev->lock);
	ret = hid_hw_raw_request(hdev, FT260_GPIO, buf, len,
				 HID_FEATURE_REPORT, HID_REQ_GET_REPORT);

	if (ret != len) {
		ret = ret < 0 ? ret : -EIO;
		hid_err(hdev, "%s: error getting GPIO: %d\n", __func__, ret);
		goto exit;
	}

	rep = (struct ft260_gpio_read_request_report *)buf;
	if (item == FT260_GPIO_VALUE)
		ret = (rep->gpio.ex_vals << FT260_GPIO_MAX) |
			   rep->gpio.vals;
	else
		ret = (rep->gpio.ex_dirs << FT260_GPIO_MAX) |
			   rep->gpio.dirs;
exit:
	mutex_unlock(&dev->lock);
	return ret;
}

static int ft260_gpio_get_direction(struct gpio_chip *gc, u32 offset)
{
	int ret = ft260_gpio_get_all(gc, FT260_GPIO_DIRECTION);

	if (ret < 0)
		return ret;
	return ~((ret >> offset) & 1);
}

static int ft260_gpio_get(struct gpio_chip *gc, u32 offset)
{
	int ret = ft260_gpio_get_all(gc, FT260_GPIO_VALUE);

	if (ret < 0)
		return ret;
	return (ret >> offset) & 1;
}

static int ft260_get_system_config(struct hid_device *hdev,
				   struct ft260_get_system_status_report *cfg)
{
	int ret;
	int len = sizeof(struct ft260_get_system_status_report);

	ret = ft260_hid_feature_report_get(hdev, FT260_SYSTEM_SETTINGS,
					   (u8 *)cfg, len);
	if (ret < 0) {
		hid_err(hdev, "failed to retrieve system status\n");
		return ret;
	}
	return 0;
}

static int ft260_is_interface_enabled(struct hid_device *hdev,
				struct ft260_get_system_status_report *cfg)
{
	struct usb_interface *usbif = to_usb_interface(hdev->dev.parent);
	int interface = usbif->cur_altsetting->desc.bInterfaceNumber;
	int ret;

	ret = ft260_get_system_config(hdev, cfg);
	if (ret < 0)
		return ret;

	ft260_dbg("interface:  0x%02x\n", interface);
	ft260_dbg("chip mode:  0x%02x\n", cfg->chip_mode);
	ft260_dbg("clock_ctl:  0x%02x\n", cfg->clock_ctl);
	ft260_dbg("i2c_enable: 0x%02x\n", cfg->i2c_enable);
	ft260_dbg("uart_mode:  0x%02x\n", cfg->uart_mode);
	ft260_dbg("gpio2_func: 0x%02x\n", cfg->gpio2_func);
	ft260_dbg("gpioA_func: 0x%02x\n", cfg->gpioa_func);
	ft260_dbg("gpioG_func: 0x%02x\n", cfg->gpiog_func);
	ft260_dbg("wakeup_int: 0x%02x\n", cfg->enable_wakeup_int);
	ft260_dbg("intr_cond: 0x%02x\n", cfg->intr_cond);

	ret = interface;
	return ret;
}

static int ft260_byte_show(struct hid_device *hdev, int id, u8 *cfg, int len,
			   u8 *field, u8 *buf)
{
	int ret;

	ret = ft260_hid_feature_report_get(hdev, id, cfg, len);
	if (ret < 0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%d\n", *field);
}

static int ft260_word_show(struct hid_device *hdev, int id, u8 *cfg, int len,
			   __le16 *field, u8 *buf)
{
	int ret;

	ret = ft260_hid_feature_report_get(hdev, id, cfg, len);
	if (ret < 0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%d\n", le16_to_cpu(*field));
}

static void ft260_attr_dummy_func(struct hid_device *hdev, u8 req, u16 value)
{
}

static void ft260_attr_uart_mode(struct hid_device *hdev, u8 req, u16 value)
{
	struct ft260_device *dev = hid_get_drvdata(hdev);

	if (!dev) {
		hid_err(hdev, "Failed to get device data\n");
		return;
	}

	if (value == FT260_UART_MODE_OFF) {
		dev->gpio_en |= FT260_GPIO_UART_DEFAULT;
		ft260_dbg(
			"UART disable, GPIOs: %04x\n",
			dev->gpio_en
		);
	} else  {
		dev->gpio_en &= ~FT260_GPIO_UART_DEFAULT;
		ft260_dbg(
			"UART enable, GPIOs: %04x\n",
			dev->gpio_en
		);
	}
}

static void ft260_attr_i2c_enable(struct hid_device *hdev, u8 req, u16 value)
{
	struct ft260_device *dev = hid_get_drvdata(hdev);

	if (!dev) {
		hid_err(hdev, "Failed to get device data\n");
		return;
	}

	if (value == FT260_I2C_DISABLE) {
		dev->gpio_en |= FT260_GPIO_I2C_DEFAULT;
		ft260_dbg(
			"I2C disable, GPIOs: %04x\n",
			dev->gpio_en
		);
	} else  {
		dev->gpio_en &= ~FT260_GPIO_I2C_DEFAULT;
		ft260_dbg(
			"I2C enable, GPIOs: %04x\n",
			dev->gpio_en
		);
	}
}

static void ft260_attr_enable_wakeup_int(struct hid_device *hdev, u8 req, u16 value)
{
	struct ft260_device *dev = hid_get_drvdata(hdev);

	if (!dev) {
		hid_err(hdev, "Failed to get device data\n");
		return;
	}

	if (value == FT260_WAKEUP_INTERUP_DISABLE) {
		dev->gpio_en |= FT260_GPIO_WAKEUP;
		ft260_dbg(
			"Wakeup/Interupt disable, GPIOs: %04x\n",
			dev->gpio_en
		);
	} else  {
		dev->gpio_en &= ~FT260_GPIO_WAKEUP;
		ft260_dbg(
			"Wakeup/Interupt enable, GPIOs: %04x\n",
			dev->gpio_en
		);
	}
}

#define FT260_ATTR_SHOW(name, reptype, id, type, func)			       \
	static ssize_t name##_show(struct device *kdev,			       \
				   struct device_attribute *attr, char *buf)   \
	{								       \
		struct reptype rep;					       \
		struct hid_device *hdev = to_hid_device(kdev);		       \
		type *field = &rep.name;				       \
		int len = sizeof(rep);					       \
										   \
		return func(hdev, id, (u8 *)&rep, len, field, buf);	       \
	}

#define FT260_SSTAT_ATTR_SHOW(name)					       \
		FT260_ATTR_SHOW(name, ft260_get_system_status_report,	       \
				FT260_SYSTEM_SETTINGS, u8, ft260_byte_show)

#define FT260_I2CST_ATTR_SHOW(name)					       \
		FT260_ATTR_SHOW(name, ft260_get_i2c_status_report,	       \
				FT260_I2C_STATUS, __le16, ft260_word_show)

#define FT260_ATTR_STORE(name, reptype, id, req, type, ctype, strtou, func)    \
	static ssize_t name##_store(struct device *kdev,		       \
					struct device_attribute *attr,	       \
					const char *buf, size_t count)	       \
	{								       \
		struct reptype rep;					       \
		struct hid_device *hdev = to_hid_device(kdev);		       \
		struct ft260_device *dev = hid_get_drvdata(hdev);	       \
		type name;						       \
		int ret;						       \
										   \
		if (!strtou(buf, 10, (ctype *)&name)) {			       \
			rep.name = name;				       \
			rep.report = id;				       \
			rep.request = req;				       \
			mutex_lock(&dev->lock);				       \
			ret = ft260_hid_feature_report_set(hdev, (u8 *)&rep,   \
							   sizeof(rep));       \
			if (ret < 0)					       \
				hid_err(hdev, "%s: failed!\n", __func__);      \
			else						       \
				func(hdev, req, name);			       \
			mutex_unlock(&dev->lock);			       \
		} else {						       \
			ret = -EINVAL;					       \
		}							       \
		return ret;						       \
	}

#define FT260_BYTE_ATTR_STORE(name, reptype, req, func)			       \
		FT260_ATTR_STORE(name, reptype, FT260_SYSTEM_SETTINGS, req,    \
				 u8, u8, kstrtou8, func)

#define FT260_WORD_ATTR_STORE(name, reptype, req, func)			       \
		FT260_ATTR_STORE(name, reptype, FT260_SYSTEM_SETTINGS, req,    \
				 __le16, u16, kstrtou16, func)

FT260_SSTAT_ATTR_SHOW(chip_mode);
static DEVICE_ATTR_RO(chip_mode);

FT260_SSTAT_ATTR_SHOW(pwren_status);
static DEVICE_ATTR_RO(pwren_status);

FT260_SSTAT_ATTR_SHOW(suspend_status);
static DEVICE_ATTR_RO(suspend_status);

FT260_SSTAT_ATTR_SHOW(hid_over_i2c_en);
static DEVICE_ATTR_RO(hid_over_i2c_en);

FT260_SSTAT_ATTR_SHOW(gpio2_func);
FT260_BYTE_ATTR_STORE(gpio2_func, ft260_set_gpio2_func_report,
			  FT260_SELECT_GPIO2_FUNC, ft260_gpio_en_update);
static DEVICE_ATTR_RW(gpio2_func);

FT260_SSTAT_ATTR_SHOW(gpioa_func);
FT260_BYTE_ATTR_STORE(gpioa_func, ft260_set_gpioa_func_report,
			  FT260_SELECT_GPIOA_FUNC, ft260_gpio_en_update);
static DEVICE_ATTR_RW(gpioa_func);

FT260_SSTAT_ATTR_SHOW(gpiog_func);
FT260_BYTE_ATTR_STORE(gpiog_func, ft260_set_gpiog_func_report,
			  FT260_SELECT_GPIOG_FUNC, ft260_gpio_en_update);
static DEVICE_ATTR_RW(gpiog_func);

FT260_SSTAT_ATTR_SHOW(enable_wakeup_int);
FT260_BYTE_ATTR_STORE(enable_wakeup_int, ft260_set_enable_interrupt_report,
			  FT260_ENABLE_INTERRUPT, ft260_attr_enable_wakeup_int);
static DEVICE_ATTR_RW(enable_wakeup_int);

FT260_SSTAT_ATTR_SHOW(intr_cond);
FT260_WORD_ATTR_STORE(intr_cond, ft260_set_interrupt_trigger_cond_report,
			  FT260_SET_INTERRUPT_TRIGGER, ft260_attr_dummy_func);
static DEVICE_ATTR_RW(intr_cond);

FT260_SSTAT_ATTR_SHOW(power_saving_en);
static DEVICE_ATTR_RO(power_saving_en);

FT260_SSTAT_ATTR_SHOW(i2c_enable);
FT260_BYTE_ATTR_STORE(i2c_enable, ft260_set_i2c_mode_report,
			  FT260_SET_I2C_MODE, ft260_attr_i2c_enable);
static DEVICE_ATTR_RW(i2c_enable);

FT260_SSTAT_ATTR_SHOW(uart_mode);
FT260_BYTE_ATTR_STORE(uart_mode, ft260_set_uart_mode_report,
			  FT260_SET_UART_MODE, ft260_attr_uart_mode);
static DEVICE_ATTR_RW(uart_mode);

FT260_SSTAT_ATTR_SHOW(clock_ctl);
FT260_BYTE_ATTR_STORE(clock_ctl, ft260_set_system_clock_report,
			  FT260_SET_CLOCK, ft260_attr_dummy_func);
static DEVICE_ATTR_RW(clock_ctl);

FT260_I2CST_ATTR_SHOW(clock);
FT260_WORD_ATTR_STORE(clock, ft260_set_i2c_speed_report,
			  FT260_SET_I2C_CLOCK_SPEED, ft260_attr_dummy_func);
static DEVICE_ATTR_RW(clock);

static ssize_t i2c_reset_store(struct device *kdev,
				   struct device_attribute *attr, const char *buf,
				   size_t count)
{
	struct hid_device *hdev = to_hid_device(kdev);
	int ret = ft260_i2c_reset(hdev);

	if (ret)
		return ret;
	return count;
}
static DEVICE_ATTR_WO(i2c_reset);

static ssize_t hid_phys_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct hid_device *hdev = to_hid_device(dev);
    return scnprintf(buf, PAGE_SIZE, "%s\n", hdev->phys);
}
static DEVICE_ATTR_RO(hid_phys);

static const struct attribute_group ft260_attr_group = {
	.attrs = (struct attribute *[]) {
		  &dev_attr_chip_mode.attr,
		  &dev_attr_pwren_status.attr,
		  &dev_attr_suspend_status.attr,
		  &dev_attr_hid_over_i2c_en.attr,
		  &dev_attr_power_saving_en.attr,
		  &dev_attr_i2c_enable.attr,
		  &dev_attr_enable_wakeup_int.attr,
		  &dev_attr_intr_cond.attr,
		  &dev_attr_gpio2_func.attr,
		  &dev_attr_gpioa_func.attr,
		  &dev_attr_gpiog_func.attr,
		  &dev_attr_uart_mode.attr,
		  &dev_attr_clock_ctl.attr,
		  &dev_attr_i2c_reset.attr,
		  &dev_attr_clock.attr,
		  &dev_attr_hid_phys.attr,
		  NULL
	}
};

static int ft260_i2c_probe(struct hid_device *hdev,
	struct ft260_device *dev,
	struct ft260_get_system_status_report const *cfg)
{

	int ret;
	struct gpio_chip *chip;

	dev->adap.class = I2C_CLASS_HWMON;
	dev->adap.algo = &ft260_i2c_algo;
	dev->adap.quirks = &ft260_i2c_quirks;
	dev->adap.dev.parent = &hdev->dev;
	snprintf(dev->adap.name, sizeof(dev->adap.name),
		 "FT260 usb-i2c bridge");

	mutex_init(&dev->lock);
	init_completion(&dev->wait);

	ret = ft260_xfer_status(dev, FT260_I2C_STATUS_BUS_BUSY);

	if (ret)
		ft260_i2c_reset(hdev);

	i2c_set_adapdata(&dev->adap, dev);
	ret = i2c_add_adapter(&dev->adap);
	if (ret) {
		hid_err(hdev, "failed to add i2c adapter\n");
		return -1;
	}

	chip = gpiochip_find(hdev->phys, ft260_gpio_chip_match_name);
	if (chip)
		return 0;

	hid_info(hdev, "initialize gpio chip\n");

	if (cfg->chip_mode) {
		if (!(cfg->chip_mode & FT260_MODE_UART))
			dev->gpio_en |= FT260_GPIO_UART_DEFAULT;

		if (!(cfg->chip_mode & FT260_MODE_I2C))
			dev->gpio_en |= FT260_GPIO_I2C_DEFAULT;
	}

	if (cfg->i2c_enable == FT260_I2C_DISABLE)
		dev->gpio_en |= FT260_GPIO_I2C_DEFAULT;
	else
		dev->gpio_en &= ~FT260_GPIO_I2C_DEFAULT;

	if (cfg->uart_mode == FT260_UART_MODE_OFF)
		dev->gpio_en |= FT260_GPIO_UART_DEFAULT;
	else
		dev->gpio_en &= ~FT260_GPIO_UART_DEFAULT;

	if (cfg->gpio2_func == FT260_MFPIN_GPIO)
		dev->gpio_en |= FT260_GPIO_2;

	if (cfg->enable_wakeup_int == FT260_WAKEUP_INTERUP_DISABLE)
		dev->gpio_en |= FT260_GPIO_3;
	else
		dev->gpio_en &= ~FT260_GPIO_3;

	if (cfg->gpioa_func == FT260_MFPIN_GPIO)
		dev->gpio_en |= FT260_GPIO_A;

	if (cfg->gpiog_func == FT260_MFPIN_GPIO)
		dev->gpio_en |= FT260_GPIO_G;

	hid_info(hdev, "enabled GPIOs: %04x\n", dev->gpio_en);

	dev->gc = devm_kzalloc(&hdev->dev, sizeof(*dev->gc), GFP_KERNEL);
	if (!dev->gc) {
		ret = -ENOMEM;
		goto err_i2c_free;
	}

	hid_set_drvdata(hdev, dev);

	dev->gc->label			= hdev->phys;
	dev->gc->direction_input	= ft260_gpio_direction_input;
	dev->gc->direction_output	= ft260_gpio_direction_output;
	dev->gc->get_direction		= ft260_gpio_get_direction;
	dev->gc->set			= ft260_gpio_set;
	dev->gc->get			= ft260_gpio_get;
	dev->gc->base			= -1;
	dev->gc->ngpio			= FT260_GPIO_TOTAL;
	dev->gc->can_sleep		= 1;
	dev->gc->parent			= &hdev->dev;

	ret = devm_gpiochip_add_data(&hdev->dev, dev->gc, dev);
	if (ret)
		goto err_i2c_free;

	return 0;

err_i2c_free:
	i2c_del_adapter(&dev->adap);
	return -1;
}

static int ft260_uart_probe(struct hid_device *hdev, struct ft260_device *dev)
{

	int ret;

	hid_info(hdev, "uart interface is not supported\n");
	// Only manage interrupt

	dev->uio.name = hdev->phys;
	dev->uio.version = "0.0.1";
	dev->uio.irq = UIO_IRQ_CUSTOM;

	ret = uio_register_device(&hdev->dev, &dev->uio);

	if (ret) {
		hid_err(hdev, "failed to register uio\n");
		return -1;
	}

	return 0;
}

static int ft260_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct ft260_device *dev;
	struct ft260_get_chip_version_report version;
	struct ft260_get_system_status_report cfg;
	int ret;

	if (!hid_is_usb(hdev))
		return -EINVAL;

	dev = devm_kzalloc(&hdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "failed to parse HID\n");
		return ret;
	}

	ret = hid_hw_start(hdev, 0);
	if (ret) {
		hid_err(hdev, "failed to start HID HW\n");
		return ret;
	}

	ret = hid_hw_open(hdev);
	if (ret) {
		hid_err(hdev, "failed to open HID HW\n");
		goto err_hid_stop;
	}

	hid_info(hdev, "USB HID v%x.%02x Device [%s] on %s\n",
		hdev->version >> 8, hdev->version & 0xff, hdev->name,
		hdev->phys);

	ret = ft260_hid_feature_report_get(hdev, FT260_CHIP_VERSION,
					   (u8 *)&version, sizeof(version));
	if (ret < 0) {
		hid_err(hdev, "failed to retrieve chip version\n");
		goto err_hid_close;
	}

	hid_info(hdev, "chip code: %02x%02x %02x%02x\n",
		 version.chip_code[0], version.chip_code[1],
		 version.chip_code[2], version.chip_code[3]);


	ret = ft260_is_interface_enabled(hdev, &cfg);
	if (ret < 0)
		goto err_hid_close;

	hid_set_drvdata(hdev, dev);

	dev->interface = ret;
	dev->hdev = hdev;
	dev->adap.owner = THIS_MODULE;

	switch (ret) {
	case FT260_INTERFACE_I2C:
		ret = ft260_i2c_probe(hdev, dev, &cfg);
		if (ret)
			goto err_hid_close;
		break;
	case FT260_INTERFACE_UART:
		ret = ft260_uart_probe(hdev, dev);
		if (ret)
			goto err_hid_close;
		break;
	default:
		goto err_hid_close;
	}

	ret = sysfs_create_group(&hdev->dev.kobj, &ft260_attr_group);
	if (ret < 0) {
		hid_err(hdev, "failed to create sysfs attrs\n");
		goto err_hid_close;
	}


	return 0;

err_hid_close:
	hid_hw_close(hdev);
err_hid_stop:
	hid_hw_stop(hdev);
	return ret;
}

static void ft260_remove(struct hid_device *hdev)
{
	struct ft260_device *dev = hid_get_drvdata(hdev);

	if (!dev) {
		hid_err(hdev, "failed to retrieve dev\n");
		return;
	}

	sysfs_remove_group(&hdev->dev.kobj, &ft260_attr_group);
	i2c_del_adapter(&dev->adap);
	uio_unregister_device(&dev->uio);

	hid_hw_close(hdev);
	hid_hw_stop(hdev);
}

static int ft260_raw_event(struct hid_device *hdev, struct hid_report *report,
			   u8 *data, int size)
{
	struct ft260_device *dev = hid_get_drvdata(hdev);
	struct ft260_i2c_input_report *xfer = (void *)data;

	if (xfer->report >= FT260_I2C_REPORT_MIN &&
		xfer->report <= FT260_I2C_REPORT_MAX) {
		ft260_dbg("i2c resp: rep %#02x len %d\n", xfer->report,
			  xfer->length);

		if ((dev->read_buf == NULL) ||
			(xfer->length > dev->read_len - dev->read_idx)) {
			hid_err(hdev, "unexpected report %#02x, length %d\n",
				xfer->report, xfer->length);
			return -1;
		}

		memcpy(&dev->read_buf[dev->read_idx], &xfer->data,
			   xfer->length);
		dev->read_idx += xfer->length;

		if (dev->read_idx == dev->read_len)
			complete(&dev->wait);

	} else if (xfer->report >= FT260_UART_INTERRUPT_STATUS) {
		ft260_dbg("Interrupt !\n");
		uio_event_notify(&dev->uio);


	} else {
		hid_err(hdev, "unhandled report %#02x\n", xfer->report);

	}
	return 0;
}

static struct hid_driver ft260_driver = {
	.name		= "ft260",
	.id_table	= ft260_devices,
	.probe		= ft260_probe,
	.remove		= ft260_remove,
	.raw_event	= ft260_raw_event,
};

module_hid_driver(ft260_driver);
MODULE_DESCRIPTION("FTDI FT260 USB HID to I2C host bridge");
MODULE_AUTHOR("Michael Zaidman <michael.zaidman@gmail.com>");
MODULE_LICENSE("GPL v2");
