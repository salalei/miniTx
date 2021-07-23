/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-01-11     Salalei      the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "touch.h"
#include "drv_gt9147.h"

#define DBG_TAG "GT9147"
#define DBG_LVL DBG_ERROR
#include <rtdbg.h>

#ifdef RT_USING_TOUCH

#define GT9147_DEFAULT_CFG_PARAM \
{\
	.cfg_version = GT9147_CURRTN_VERSION,\
	.x_max_lsb = 0xe0,\
	.x_max_msb = 0x01,\
	.y_max_lsb = 0x20,\
	.y_max_msb = 0x03,\
	.touch_numb = 0x05,\
	.module_sw1 = 0x3c,\
	.module_sw2 = 0x00,\
	.shake_count = 0x44,\
	.filter = 0x08,\
	.large_touch = 0x1e,\
	.noise_reduction = 0x08,\
	.screen_touch_level = 0x50,\
	.screen_leave_level = 0x3c,\
	.low_power_ctrl = 0x0f,\
	.refresh_rate = 0x05,\
	.x_threshold = 0x00,\
	.y_threshold = 0x00,\
	.gesture_sw1 = 0x00,\
	.gesture_sw2 = 0x00,\
	.space_up_down = 0x00,\
	.space_left_right = 0x00,\
	.mini_filter = 0x00,\
	.stretch_r0 = 0x18,\
	.stretch_r1 = 0x1a,\
	.stretch_r2 = 0x1e,\
	.stretch_rm = 0x14,\
	.drv_group_a_numb = 0x89,\
	.drv_group_b_numb = 0x28,\
	.sensor_numb = 0x0a,\
	.freq_a_factor = 0x30,\
	.freq_b_factor = 0x2e,\
	.pannel_bit_freq_lsb = 0xbb,\
	.pannel_bit_freq_msb = 0x0a,\
	.module_sw4 = 0x03,\
	.gesture_long_press_time = 0x00,\
	.pannel_tx_gain = 0x00,\
	.pannel_rx_gain = 0x02,\
	.pannel_dump_shift = 0x33,\
	.drv_frame_ctrl = 0x1d,\
	.s_feedback = 0x00,\
	.module_sw3 = 0x00,\
	.screen_neg_thres = 0x00,\
	.shape_ctrl = 0x00,\
	.edge_complem_thres = 0x00,\
	.edge_complement_x = 0x00,\
	.edge_complement_y = 0x00,\
	.water_frame_time = 0x32,\
	.water_update_time = 0x10,\
	.charging_level_up = 0x00,\
	.gesture_ctrl = 0x2a,\
	.freq_hopping_start = 0x1c,\
	.freq_hopping_end = 0x5a,\
	.noise_detect_time = 0x94,\
	.hopping_flag = 0xc5,\
	.hopping_threshold = 0x02,\
	.hot_knot_noise_map = 0x07,\
	.noise_min_threshold = 0x00,\
	.reserved1 = 0x00,\
	.hopping_sensor_group = 0x04,\
	.hopping_seg1_normalize = 0xb5,\
	.hopping_seg1_factor = 0x1f,\
	.main_clock_adjust = 0x00,\
	.hopping_seg2_normalize = 0x90,\
	.hopping_seg2_factor = 0x28,\
	.reserved2 = 0x00,\
	.hopping_seg3_normalize = 0x77,\
	.hopping_seg3_factor = 0x32,\
	.reserved3 = 0x00,\
	.hopping_seg4_normalize = 0x62,\
	.hopping_seg4_factor = 0x3f,\
	.reserved4 = 0x00,\
	.hopping_seg5_normalize = 0x52,\
	.hopping_seg5_factor = 0x50,\
	.reserved5 = 0x00,\
	.hopping_seg6_normalize = 0x52,\
	.key1 = 0x00,\
	.key2 = 0x00,\
	.key3 = 0x00,\
	.key4 = 0x00,\
	.key_area = 0x00,\
	.key_touch_level = 0x00,\
	.key_leave_level = 0x00,\
	.key1_2_sens = 0x00,\
	.key3_4_sens = 0x00,\
	.key_restrain = 0x00,\
	.key_down_edge_filter = 0x00,\
	.proximity_valid_time = 0x00,\
	.proximity_press_time1 = 0x00,\
	.proximity_press_time2 = 0x00,\
	.proximity_large_touch = 0x00,\
	.proximity_drv_select = 0x00,\
	.proximity_sens_select = 0x00,\
	.proximity_touch_level = 0x00,\
	.proximity_leave_level = 0x00,\
	.proximity_sample_add_time = 0x00,\
	.proximity_sample_dec_lsb = 0x00,\
	.proximity_sample_dec_msb = 0x00,\
	.proximity_leave_shake_count = 0x00,\
	.data_threshold = 0x0f,\
	.pxy_threshold = 0x0f,\
	.dump_shift = 0x03,\
	.rx_gain = 0x06,\
	.freq_gain0 = 0x10,\
	.freq_gain1 = 0x42,\
	.freq_gain2 = 0xf8,\
	.freq_gain3 = 0x0f,\
	.gesture_refresh_rate = 0x14,\
	.combine_dis = 0x00,\
	.split_set = 0x00,\
	.gesture_touch_level = 0x00,\
	.new_green_wake_up_level = 0x00,\
	.sensor_ch = {0x1a, 0x18, 0x16, 0x14, 0x12, 0x10, 0x0e, 0x0c, 0x0a, 0x08, 0x00, 0x00, 0x00, 0x00},\
	.reserved6 = {0x00},\
	.driver_ch = {0x29, 0x28, 0x24, 0x22, 0x20, 0x1f, 0x1e, 0x1d, 0x0e, 0x0c, 0x0a, 0x08, 0x06, 0x05, \
		0x04, 0x02, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},\
	.reserved7 = {0x00},\
	.driver_gain_0_1 = 0xff,\
	.driver_gain_2_3 = 0xff,\
	.driver_gain_4_5 = 0xff,\
	.driver_gain_6_7 = 0xff,\
	.driver_gain_8_9 = 0xff,\
	.driver_gain_10_11 = 0xff,\
	.driver_gain_12_13 = 0xff,\
	.driver_gain_14_15 = 0xff,\
	.driver_gain_16_17 = 0xff,\
	.driver_gain_18_19 = 0xff,\
	.driver_gain_20_21 = 0xff,\
	.driver_gain_22_23 = 0xff,\
	.driver_gain_24_25 = 0xff,\
	.config_chksum = 0x00,\
	.config_fresh = 0x01\
}

#define GET_CFG_REG_ADDR(member)		((rt_uint16_t)((rt_uint32_t)(&((gt9147_cfg_reg_t)GT9147_CFG_REG_ADDR)->member)))
#define GET_COORD_REG_ADDR(member)		((rt_uint16_t)((rt_uint32_t)(&((gt9147_coord_reg_t)GT9147_COORD_REG_ADDR)->member)))

struct touch_config_user_data
{
	struct rt_i2c_client i2c_client;
};

/**
 * @brief Write an amount of data to a specific register address
 * 
 * @param dev Pointer to the touch device structure
 * @param reg_addr Register address to be written
 * @param buf Pointer to the data buffer
 * @param len Amount of data to be sent
 * @return The error code 
 */
static rt_err_t gt9147_mem_write(rt_touch_t dev, rt_uint16_t reg_addr, rt_uint8_t *buf, rt_uint16_t len)
{
	struct rt_i2c_client *client = &((struct touch_config_user_data *)(dev->config.user_data))->i2c_client;
    struct rt_i2c_msg msgs[2];

	//Send high 8 bits first
	reg_addr = reg_addr >> 8 | reg_addr << 8;
    msgs[0].addr = client->client_addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = (rt_uint8_t *)&reg_addr;
    msgs[0].len = 2;

	msgs[1].addr = client->client_addr;
    msgs[1].flags = RT_I2C_WR | RT_I2C_NO_START;
    msgs[1].buf = buf;
    msgs[1].len = len;

	client->bus->retries = 5;
    if (rt_i2c_transfer(client->bus, msgs, 2) == 2)
        return RT_EOK;
    else
        return RT_ERROR;
}

/**
 * @brief Read an amount of data from a specific register address
 * 
 * @param dev Pointer to the touch device structure
 * @param reg_addr Register address to be read
 * @param buf Pointer to the buffer data
 * @param len Amount of data to be received
 * @return The error code 
 */
static rt_err_t gt9147_mem_read(rt_touch_t dev, rt_uint16_t reg_addr, rt_uint8_t *buf, rt_uint16_t len)
{
	struct rt_i2c_client *client = &((struct touch_config_user_data *)(dev->config.user_data))->i2c_client;
    struct rt_i2c_msg msgs[2];

	//Send high 8 bits first
	reg_addr = reg_addr >> 8 | reg_addr << 8;
    msgs[0].addr = client->client_addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = (rt_uint8_t *)&reg_addr;
    msgs[0].len = 2;

	msgs[1].addr = client->client_addr;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf = buf;
    msgs[1].len = len;

	client->bus->retries = 5;
    if (rt_i2c_transfer(client->bus, msgs, 2) == 2)
        return RT_EOK;
    else
        return RT_ERROR;
}

/**
 * @brief Get device id
 * 
 * @param dev Pointer to the touch device structure
 * @param id GT9147 id
 * @return The error code 
 */
static rt_err_t touch_get_id(rt_touch_t dev, rt_uint32_t *id)
{
	rt_uint32_t read_id;
	rt_err_t res;

	res = gt9147_mem_read(dev, GET_COORD_REG_ADDR(product_id_1), (rt_uint8_t *)&read_id, 4);
	if(res == RT_EOK)
		*id = read_id;

	return res;
}

/**
 * @brief Get device information
 * 
 * @param dev Pointer to the touch device structure
 * @return The error code 
 */
static rt_err_t touch_get_info(rt_touch_t dev)
{
	rt_err_t res;
	rt_uint8_t point_num;
	rt_uint16_t range;

	dev->info.type = RT_TOUCH_TYPE_CAPACITANCE;
	LOG_I("Touch type:CAPACITANCE");
	dev->info.vendor = RT_TOUCH_VENDOR_GT;
	LOG_I("Touch vendor:GTxx series");

	res = gt9147_mem_read(dev, GET_CFG_REG_ADDR(touch_numb), (rt_uint8_t *)&point_num, 1);
	if(res == RT_EOK)
		dev->info.point_num = point_num & 0x0f;
	else
		goto error;
	LOG_I("Touch numb:%d", dev->info.point_num);

	res = gt9147_mem_read(dev, GET_CFG_REG_ADDR(x_max_lsb), (rt_uint8_t *)&range, 2);
	if(res == RT_EOK)
		dev->info.range_x = range;
	else
		goto error;
	LOG_I("Touch x range:%d", dev->info.range_x);

	res = gt9147_mem_read(dev, GET_CFG_REG_ADDR(y_max_lsb), (rt_uint8_t *)&range, 2);
	if(res == RT_EOK)
		dev->info.range_y = range;
	else
		goto error;
	LOG_I("Touch y range:%d", dev->info.range_y);

	return RT_EOK;

error:

	LOG_E("Get touch information failed");

	return res;
}

/**
 * @brief Set the range of x or y coordinate
 * 
 * @param dev Pointer to the touch device structure
 * @param coord
 *    @arg 0 X coordinate
 *    @arg 1 Y coordinate
 * @param range The range of x or y coordinate
 * @return The error code 
 */
static rt_err_t touch_set_x_y_range(rt_touch_t dev, rt_int32_t coord, rt_int32_t range)
{
	rt_err_t res;
	rt_uint16_t write_range = (rt_uint16_t) range;
	rt_uint16_t reg_addr = coord ? GET_CFG_REG_ADDR(y_max_lsb) : GET_CFG_REG_ADDR(x_max_lsb);
	
	res = gt9147_mem_write(dev, reg_addr, (rt_uint8_t *)&write_range, 2);
	if(res != RT_EOK)
		LOG_E("Set touch %s range failed", coord ? "y" : "x");

	return res;
}

/**
 * @brief Set x y coordinate exchange
 * 
 * @param dev Pointer to the touch device structure
 * @return The error code 
 */
static rt_err_t touch_set_x_to_y(rt_touch_t dev)
{
	rt_err_t res;
	rt_uint8_t reg_data;

	res = gt9147_mem_read(dev, GET_CFG_REG_ADDR(module_sw1), (rt_uint8_t *)&reg_data, 1);
	if(res != RT_EOK)
		goto error;

	reg_data ^= 1 << 3;

	res = gt9147_mem_write(dev, GET_CFG_REG_ADDR(module_sw1), (rt_uint8_t *)&reg_data, 1);
	if(res != RT_EOK)
		goto error;

	return RT_EOK;

error:

	LOG_E("Set X Y coordinate exchange failed");

	return res;
}

/**
 * @brief Enable or disable touch interrupt
 * 
 * @param dev Pointer to the touch device structure
 * @param enable
 *    @arg RT_TRUE Enable touch interrupt
 *    @arg RT_FALSE Disable touch interrupt
 * @return The error code 
 */
static rt_err_t touch_enable_irq(rt_touch_t dev, rt_bool_t enable)
{
	return rt_pin_irq_enable(dev->config.irq_pin.pin, enable ? PIN_IRQ_ENABLE : PIN_IRQ_DISABLE);
}

static rt_size_t touch_readpoint(struct rt_touch_device *dev, void *buf, rt_size_t touch_num)
{
	rt_uint8_t status;
	rt_size_t cur_touch_num;
	rt_uint8_t *touch_reg_data = RT_NULL;
	rt_uint8_t clear = 0;
	rt_bool_t cur_track_id[5] = {RT_FALSE};
	rt_uint8_t id_to_index[5] = {0};
	struct rt_touch_data *touch_data = (struct rt_touch_data *)buf;
	int index;
	static rt_bool_t last_track_id[5] = {RT_FALSE};

	if(gt9147_mem_read(dev, GET_COORD_REG_ADDR(status), (rt_uint8_t *)&status, 1) != RT_EOK)
		return 0;

	if(gt9147_mem_write(dev, GET_COORD_REG_ADDR(status), (rt_uint8_t *)&clear, 1) != RT_EOK)
		return 0;

	//bit 7 is buffer status
	if(!(status & 1 << 7))
		return 0;

	cur_touch_num = status & 0x0f;
	
	if(cur_touch_num)
	{
		touch_reg_data = (rt_uint8_t *)rt_malloc(8 * cur_touch_num);
		if(touch_reg_data == RT_NULL)
		{
			LOG_E("Request failed");
			return 0;
		}

		if(gt9147_mem_read(dev, GET_COORD_REG_ADDR(track_id1), touch_reg_data, 8 * cur_touch_num) != RT_EOK)
		{
			LOG_E("Read touch point coordinates failed");
			rt_free(touch_reg_data);
			return 0;
		}
	}

	for(int i = 0; i < cur_touch_num; i++)
	{
		if(touch_reg_data[8 * i] < 5)
		{
			cur_track_id[touch_reg_data[8 * i]] = RT_TRUE;
			id_to_index[touch_reg_data[8 * i]] = i;
		}
	}

	touch_num = touch_num < cur_touch_num ? touch_num : cur_touch_num;

	for(int i = 0; i < touch_num; i++)
	{
		if(cur_track_id[i])
		{
			if(last_track_id[i] == RT_FALSE)
				touch_data[i].event = RT_TOUCH_EVENT_DOWN;
			else
				touch_data[i].event = RT_TOUCH_EVENT_MOVE;
			touch_data[i].timestamp = rt_touch_get_ts();
			touch_data[i].track_id = i;
			index = id_to_index[i];
			touch_data[i].width = (rt_uint16_t)touch_reg_data[8 * index + 6] << 8 | touch_reg_data[8 * index + 5];
			touch_data[i].x_coordinate = (rt_uint16_t)touch_reg_data[8 * index + 2] << 8 | touch_reg_data[8 * index + 1];
			touch_data[i].y_coordinate = (rt_uint16_t)touch_reg_data[8 * index + 4] << 8 | touch_reg_data[8 * index + 3];
		}
		else if(last_track_id[i])
			touch_data[i].event = RT_TOUCH_EVENT_UP;
		else
			touch_data[i].event = RT_TOUCH_EVENT_NONE;
	}

	rt_free(touch_reg_data);

	return touch_num;
}

static rt_err_t touch_control(rt_touch_t dev, int cmd, void *arg)
{
	rt_uint32_t id;
	rt_err_t res;

	switch(cmd)
	{
	case RT_TOUCH_CTRL_GET_ID:
		res = touch_get_id(dev, &id);
		if(res == RT_EOK)
			*(rt_uint32_t *)arg = id;
		return RT_EOK;

	case RT_TOUCH_CTRL_GET_INFO:
		rt_memcpy(arg, &dev->info, sizeof(struct rt_touch_info));
		return RT_EOK;

	case RT_TOUCH_CTRL_SET_MODE:
		if((rt_uint32_t)arg == RT_DEVICE_FLAG_INT_RX || (rt_uint32_t)arg == RT_DEVICE_FLAG_RDONLY)
			return RT_EOK;
		else
			return RT_EINVAL;
		
	case RT_TOUCH_CTRL_SET_X_RANGE:
		res = touch_set_x_y_range(dev, 0, *(rt_int32_t *)arg);
		return 0;
		
	case RT_TOUCH_CTRL_SET_Y_RANGE:
		res = touch_set_x_y_range(dev, 1, *(rt_int32_t *)arg);
		return res;

	case RT_TOUCH_CTRL_SET_X_TO_Y:
		res = touch_set_x_to_y(dev);
		return res;

	case RT_TOUCH_CTRL_DISABLE_INT:
		res = touch_enable_irq(dev, RT_FALSE);
		return res;

	case RT_TOUCH_CTRL_ENABLE_INT:
		res = touch_enable_irq(dev, RT_TRUE);
		return res;

	case RT_TOUCH_CTRL_POWER_ON:
		LOG_E("The command is not supported in the current version");
		return RT_ERROR;

	case RT_TOUCH_CTRL_POWER_OFF:
		LOG_E("The command is not supported in the current version");
		return RT_ERROR;

	case RT_TOUCH_CTRL_GET_STATUS:
		LOG_E("The command is not supported in the current version");
		return RT_ERROR;

	default:
		return RT_EINVAL;
	}
}

static struct rt_touch_ops touch_ops = 
{
	.touch_readpoint = touch_readpoint,
	.touch_control = touch_control
};

static rt_err_t drv_gt9147_init(rt_touch_t dev)
{
	rt_err_t res;
	rt_uint8_t cmd;
	rt_uint8_t current_version;

	//Reset
	cmd = 0x02;
	res = gt9147_mem_write(dev, GT9147_CMD_ADDR, (rt_uint8_t *)&cmd, 1);
	if(res != RT_EOK)
	{
		LOG_E("Cannot get gt9147 version");
		return res;
	}

	//Get current version
	res = gt9147_mem_read(dev, GET_CFG_REG_ADDR(cfg_version), (rt_uint8_t *)&current_version, 1);
	if(res != RT_EOK)
	{
		LOG_E("Cannot get gt9147 version");
		return res;
	}

	//Check whether to update
	if(current_version < GT9147_CURRTN_VERSION)
	{
		struct gt9147_cfg_reg cfg_reg_param = GT9147_DEFAULT_CFG_PARAM;
		rt_uint8_t check_sum = 0, *p = (rt_uint8_t *)&cfg_reg_param;
		
		LOG_I("Update gt9147 configuration, current version=%d, new version=%d", current_version, GT9147_CURRTN_VERSION);

		for(int i = 0; i < sizeof(struct gt9147_cfg_reg) - 2; i++)
			check_sum += *p++;

		check_sum = ~check_sum + 1;
		LOG_D("Check sum=%d", check_sum);
		cfg_reg_param.config_chksum = check_sum;

		res = gt9147_mem_write(dev, GET_CFG_REG_ADDR(cfg_version), (rt_uint8_t *)&cfg_reg_param, sizeof(struct gt9147_cfg_reg));

		if(res != RT_EOK)
		{
			LOG_E("Cannot set gt9147 configuration");
			return res;
		}
		LOG_I("Done");
	}

	//Read coordinate
	cmd = 0x00;
	res = gt9147_mem_write(dev, GT9147_CMD_ADDR, (rt_uint8_t *)&cmd, 1);
	if(res != RT_EOK)
	{
		LOG_E("Cannot set command");
		return res;
	}

	return RT_EOK;
}

/**
 * @brief Regiter GT9147 device
 * 
 * @param dev_name The name of the device
 * @param bus_name The name of the i2c bus
 * @param irq_pin Interrupt pin
 * @param rst_pin Reset pin
 * @return The error code 
 */
rt_err_t rt_hw_gt9147_init(char *dev_name, char *bus_name, rt_uint16_t irq_pin, rt_uint16_t rst_pin)
{
	struct touch_config_user_data *user_data = RT_NULL;
	rt_touch_t touch_dev = RT_NULL;

	user_data = (struct touch_config_user_data *)rt_malloc(sizeof(struct touch_config_user_data));
	if(user_data == RT_NULL)
	{
		LOG_E("Request failed");
		goto error;
	}
	rt_memset(user_data, 0, sizeof(struct touch_config_user_data));

	user_data->i2c_client.bus = (struct rt_i2c_bus_device *)rt_device_find(bus_name);
	if(user_data->i2c_client.bus == RT_NULL)
	{
		LOG_E("Cannot find bus, name: %s", bus_name);
		goto error;
	}
	user_data->i2c_client.client_addr = GT9147_DEVICE_ADDR;

	touch_dev = (rt_touch_t)rt_malloc(sizeof(struct rt_touch_device));
	if(touch_dev == RT_NULL)
	{
		LOG_E("Request failed");
		goto error;
	}
	rt_memset(touch_dev, 0, sizeof(struct rt_touch_device));

	touch_dev->config.user_data = user_data;
	touch_dev->config.dev_name = bus_name;
	touch_dev->config.irq_pin.pin = irq_pin;
	touch_dev->config.irq_pin.mode = PIN_MODE_INPUT_PULLDOWN;
	touch_dev->irq_handle = RT_NULL;
	touch_dev->ops = &touch_ops;
	if(touch_get_info(touch_dev) != RT_EOK)
	{
		LOG_E("Cannot get gt9147 information");
		goto error;
	}

	rt_pin_write(rst_pin, PIN_LOW);
	rt_pin_mode(rst_pin, PIN_MODE_OUTPUT);
	//Write irq pin low to select address 0xBA
	rt_pin_write(irq_pin, PIN_LOW);
	rt_pin_mode(irq_pin, PIN_MODE_OUTPUT);
	rt_thread_mdelay(15);//> 10ms + 100us
	rt_pin_write(rst_pin, PIN_HIGH);
	rt_thread_mdelay(100);//> 5ms + 50ms
	rt_pin_mode(irq_pin, PIN_MODE_INPUT);

	if(drv_gt9147_init(touch_dev) != RT_EOK)
	{
		LOG_E("GT9147 init failed");
		goto error;
	}

	if(rt_hw_touch_register(touch_dev, dev_name, RT_DEVICE_FLAG_INT_RX, RT_NULL) != RT_EOK)
	{
		LOG_E("Register touch device failed,name=%s", dev_name);
		goto error;
	}

	LOG_I("Touch device register successful");

	return RT_EOK;

error:

	if(user_data != RT_NULL)
		rt_free(user_data);
	if(touch_dev != RT_NULL)
		rt_free(touch_dev);
	
	return RT_ERROR;
}

int rt_hw_touch_init(void)
{
	return rt_hw_gt9147_init("touch", "i2c0", 40, 39);
}

// INIT_ENV_EXPORT(rt_hw_touch_init);

#endif