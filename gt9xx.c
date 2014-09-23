/* drivers/input/touchscreen/gt9xx.c
 *
 * 2010 - 2012 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Version:1.2
 * Author:andrew@goodix.com
 * Release Date:2012/10/15
 * Revision record:
 *	  V1.0:2012/08/31,first Release
 *	  V1.2:2012/10/15,modify gtp_reset_guitar,slot report,tracking_id & 0x0F
 *
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>

struct goodix_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	u16 abs_x_max;
	u16 abs_y_max;
	u8 max_touch_num;
	u8 int_trigger_type;
};

#define GTP_DEBUG_ON		0

#define GTP_IRQ_TAB		{IRQ_TYPE_EDGE_RISING, \
				 IRQ_TYPE_EDGE_FALLING, \
				 IRQ_TYPE_LEVEL_LOW, \
				 IRQ_TYPE_LEVEL_HIGH}

#define GTP_MAX_HEIGHT		4096
#define GTP_MAX_WIDTH		4096
#define GTP_INT_TRIGGER		1
#define GTP_MAX_TOUCH		10

#define GTP_DRIVER_VERSION	"V1.3<2014/09/18>"
#define GTP_I2C_NAME		"Goodix9110-TS"
#define GTP_CONFIG_MAX_LENGTH	240

/* Register defineS */
#define GTP_READ_COOR_ADDR	0x814E
#define GTP_REG_SLEEP		0x8040
#define GTP_REG_SENSOR_ID	0x814A
#define GTP_REG_CONFIG_DATA	0x8047
#define GTP_REG_VERSION		0x8140

#define RESOLUTION_LOC		1
#define TRIGGER_LOC		6

#define GTP_INFO(fmt,arg...)		pr_info("<<-GTP-INFO->> "fmt"\n",##arg)
#define GTP_ERROR(fmt,arg...)		pr_err("<<-GTP-ERROR->> "fmt"\n",##arg)
#define GTP_DEBUG(fmt,arg...)		do{\
						if (GTP_DEBUG_ON)\
							pr_debug("<<-GTP-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
					}while(0)
static const char *goodix_ts_name = "Goodix Capacitive TouchScreen";

/**
 * gtp_i2c_read - read data from a register of the i2c slave device.
 *
 * @client: i2c device.
 * @reg: the register to read from.
 * @buf: raw write data buffer.
 * @len: lenght of the buffer to write
 */
static int gtp_i2c_read(struct i2c_client *client, u16 reg, u8 *buf, s32 len)
{
	struct i2c_msg msgs[2];
	u8 wbuf[2] = { reg >> 8, reg & 0xff };

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = client->addr;
	msgs[0].len   = 2;
	msgs[0].buf   = wbuf;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].len   = len;
	msgs[1].buf   = buf;

	return i2c_transfer(client->adapter, msgs, 2);
}

/**
 * gtp_i2c_write - write data to the i2c slave device.
 *
 * @client: i2c device.
 * @buf: raw write data buffer.
 * @len: lenght of the buffer to write
 *
 * The function does not take the destination register as a parameter.
 * The caller is in charge of setting the register properly in the first bytes
 * of buf.
 */
static int gtp_i2c_write(struct i2c_client *client, u8 *buf, s32 len)
{
	struct i2c_msg msg;

	msg.flags = !I2C_M_RD;
	msg.addr  = client->addr;
	msg.len   = len;
	msg.buf   = buf;

	return i2c_transfer(client->adapter, &msg, 1);
}

/**
 * goodix_ts_work_func - Process incoming IRQ
 *
 * @ts: our goodix_ts_data pointer
 *
 * Called when the IRQ is triggered. Read the current device state, and push
 * the input events to the user space.
 */
static void goodix_ts_work_func(struct goodix_ts_data *ts)
{
	u8  end_cmd[3] = {
			GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0};
	u8  point_data[1 + 8 * GTP_MAX_TOUCH + 1];
	bool touch_state;
	u8 touch_num;
	u8 finger;
	static u16 pre_touch = 0;
	u8* coor_data = NULL;
	int input_x;
	int input_y;
	int input_w;
	int id = 0;
	int i;
	int ret;

	ret = gtp_i2c_read(ts->client, GTP_READ_COOR_ADDR, point_data, 10);
	if (ret < 0) {
		/* If touchscreen is reset for any reason, the i2c address maybe changed */
		if (ts->client->addr == 0x14)
			ts->client->addr = 0x5d;
		else
			ts->client->addr = 0x14;

		GTP_ERROR("I2C transfer error. errno:%d , change i2c address as %d\n ", ret,ts->client->addr);
		goto exit_work_func;
	}

	finger = point_data[0];
	if ((finger & 0x80) == 0)
		goto exit_work_func;

	touch_num = finger & 0x0f;
	if (touch_num > GTP_MAX_TOUCH)
		goto exit_work_func;

	if (touch_num > 1)
		ret = gtp_i2c_read(ts->client, GTP_READ_COOR_ADDR + 10,
				   &point_data[10], 8 * (touch_num - 1));

	GTP_DEBUG("pre_touch:%02x, finger:%02x.", pre_touch, finger);

	if (pre_touch || touch_num) {
		s32 pos = 0;
		u16 touch_index = 0;

		coor_data = &point_data[1];
		if (touch_num) {
			id = coor_data[pos] & 0x0F;
			touch_index |= (0x01<<id);
		}

		GTP_DEBUG("id=%d,touch_index=0x%x,pre_touch=0x%x\n",id, touch_index,pre_touch);
		for (i = 0; i < GTP_MAX_TOUCH; i++) {
			touch_state = touch_index & (0x01 << i);
			if (touch_state) {
				input_x  = coor_data[pos + 1] | coor_data[pos + 2] << 8;
				input_y  = coor_data[pos + 3] | coor_data[pos + 4] << 8;
				input_w  = coor_data[pos + 5] | coor_data[pos + 6] << 8;

				input_mt_slot(ts->input_dev, id);
				input_mt_report_slot_state(ts->input_dev,
						MT_TOOL_FINGER, touch_state);
				input_report_abs(ts->input_dev,
						ABS_MT_POSITION_X, input_x);
				input_report_abs(ts->input_dev,
						ABS_MT_POSITION_Y, input_y);
				input_report_abs(ts->input_dev,
						ABS_MT_TOUCH_MAJOR, input_w);
				input_report_abs(ts->input_dev,
						ABS_MT_WIDTH_MAJOR, input_w);
				pre_touch |= 0x01 << i;

				pos += 8;
				id = coor_data[pos] & 0x0F;
				touch_index |= (0x01 << id);
			} else {
				input_mt_slot(ts->input_dev, i);
				input_mt_report_slot_state(ts->input_dev,
						MT_TOOL_FINGER, touch_state);
				pre_touch &= ~(0x01 << i);
			}
		}
	}

	input_mt_sync_frame(ts->input_dev);
	input_sync(ts->input_dev);

exit_work_func:

	ret = gtp_i2c_write(ts->client, end_cmd, 3);
	if (ret < 0)
		GTP_INFO("I2C write end_cmd  error!");
}

/**
 * goodix_ts_irq_handler - The IRQ handler
 *
 * @irq: interrupt number.
 * @dev_id: private data pointer.
 */
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
	struct goodix_ts_data *ts = dev_id;

	goodix_ts_work_func(ts);

	return IRQ_HANDLED;
}

/**
 * gtp_read_version - Initialize the panel
 *
 * @ts: our goodix_ts_data pointer
 *
 * Must be called during probe
 */
static void gtp_init_panel(struct goodix_ts_data *ts)
{
	s32 ret;
	u8 config[GTP_CONFIG_MAX_LENGTH];

	ret = gtp_i2c_read(ts->client, GTP_REG_CONFIG_DATA, config,
			   GTP_CONFIG_MAX_LENGTH);
	if (ret < 0) {
		GTP_ERROR("GTP read resolution & max_touch_num failed, use default value!");
		ts->abs_x_max = GTP_MAX_WIDTH;
		ts->abs_y_max = GTP_MAX_HEIGHT;
		ts->int_trigger_type = GTP_INT_TRIGGER;
		return;
	}

	ts->abs_x_max = (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
	ts->abs_y_max = (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
	ts->int_trigger_type = (config[TRIGGER_LOC]) & 0x03;
	if ((!ts->abs_x_max)||(!ts->abs_y_max)) {
		GTP_ERROR("GTP resolution & max_touch_num invalid, use default value!");
		ts->abs_x_max = GTP_MAX_WIDTH;
		ts->abs_y_max = GTP_MAX_HEIGHT;
	}

	msleep(500);

	GTP_DEBUG("X_MAX = %d,Y_MAX = %d,TRIGGER = 0x%02x",
			 ts->abs_x_max,ts->abs_y_max,ts->int_trigger_type);
}


/**
 * gtp_read_version - Read goodix touchscreen version
 *
 * @client: the i2c client
 * @version: output buffer containing the version on success
 */
static s32 gtp_read_version(struct i2c_client *client, u16* version)
{
	s32 ret = -1;
	s32 i = 0;
	u8 buf[6];

	ret = gtp_i2c_read(client, GTP_REG_VERSION, buf, sizeof(buf));
	if (ret < 0) {
		GTP_ERROR("GTP read version failed");
		return ret;
	}

	if (version)
		*version = (buf[5] << 8) | buf[4];

	for(i=0; i<4; i++) {
		if (!buf[i])
			buf[i] = 0x30;
	}
	GTP_INFO("IC VERSION:%c%c%c%c_%02x%02x",
			  buf[0], buf[1], buf[2], buf[3], buf[5], buf[4]);

	return ret;
}

/**
 * gtp_i2c_test - I2C test function to check if the device answers.
 *
 * @client: the i2c client
 */
static s8 gtp_i2c_test(struct i2c_client *client)
{
	u8 test;
	u8 retry = 0;
	s8 ret = -1;

	while (retry++ < 2) {
		ret = gtp_i2c_read(client, GTP_REG_CONFIG_DATA, &test, 1);
		if (ret > 0)
			return ret;

		GTP_ERROR("GTP i2c test failed time %d.",retry);
		msleep(10);
	}
	return ret;
}

/**
 * gtp_request_irq - Request the IRQ handler
 *
 * @ts: our goodix_ts_data pointer
 *
 * Must be called during probe
 */
static s8 gtp_request_irq(struct goodix_ts_data *ts)
{
	s32 ret = -1;
	const u8 irq_table[] = GTP_IRQ_TAB;

	GTP_DEBUG("INT trigger type:%x", ts->int_trigger_type);

	ret  = devm_request_threaded_irq(&ts->client->dev,
					 ts->client->irq, NULL,
					 goodix_ts_irq_handler,
					 irq_table[ts->int_trigger_type] | IRQF_ONESHOT,
					 ts->client->name,
					 ts);
	if (ret) {
		GTP_ERROR("Request IRQ failed!ERRNO:%d.", ret);
		return -1;
	}

	disable_irq_nosync(ts->client->irq);
	return 0;
}

/**
 * gtp_request_input_dev - Allocate, populate and register the input device
 *
 * @ts: our goodix_ts_data pointer
 *
 * Must be called during probe
 */
static s8 gtp_request_input_dev(struct goodix_ts_data *ts)
{
	s8 ret = -1;
	s8 phys[32];

	ts->input_dev = devm_input_allocate_device(&ts->client->dev);
	if (ts->input_dev == NULL) {
		GTP_ERROR("Failed to allocate input device.");
		return -ENOMEM;
	}

	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	input_mt_init_slots(ts->input_dev, GTP_MAX_TOUCH, INPUT_MT_DIRECT);

	sprintf(phys, "input/ts");
	ts->input_dev->name = goodix_ts_name;
	ts->input_dev->phys = phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = 10427;

	ret = input_register_device(ts->input_dev);
	if (ret) {
		GTP_ERROR("Register %s input device failed", ts->input_dev->name);
		return -ENODEV;
	}

	return 0;
}

static int goodix_ts_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	s32 ret = -1;
	struct goodix_ts_data *ts;
	u16 version_info;

	GTP_INFO("GTP Driver Version:%s", GTP_DRIVER_VERSION);
	GTP_INFO("GTP I2C Address:0x%02x", client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		GTP_ERROR("I2C check functionality failed.");
		return -ENODEV;
	}
	ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		GTP_ERROR("Alloc GFP_KERNEL memory failed.");
		return -ENOMEM;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);

	ret = gtp_i2c_test(client);
	if (ret < 0) {
		client->addr = 0x5d;
		ret = gtp_i2c_test(client);
		if (ret < 0) {
			GTP_ERROR("I2C communication ERROR!");
			return -ENODEV;
		}
		GTP_INFO("GTP I2C new Address:0x%02x", client->addr);
	}

	gtp_init_panel(ts);

	ret = gtp_request_input_dev(ts);
	if (ret < 0) {
		GTP_ERROR("GTP request input dev failed");
		return ret;
	}

	ret = gtp_request_irq(ts);
	if (ret < 0)
		return ret;

	ret = gtp_read_version(client, &version_info);
	if (ret < 0) {
		GTP_ERROR("Read version failed.");
		return ret;
	}

	enable_irq(ts->client->irq);

	return 0;
}

static const struct i2c_device_id goodix_ts_id[] = {
	{ GTP_I2C_NAME, 0 },
	{ "GDIX1001:00", 0 },
	{ }
};

static const struct acpi_device_id goodix_acpi_match[] = {
	{ "GDIX1001", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, goodix_acpi_match);

static struct i2c_driver goodix_ts_driver = {
	.probe = goodix_ts_probe,
	.id_table = goodix_ts_id,
	.driver = {
		.name = GTP_I2C_NAME,
		.owner = THIS_MODULE,
		.acpi_match_table = goodix_acpi_match,
	},
};

module_i2c_driver(goodix_ts_driver);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
