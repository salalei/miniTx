#include <rtthread.h>
#include <rthw.h>
#include "rtdevice.h"

#include "drv_bw3532mib.h"
#include "drv_display.h"
#include "drv_gpio.h"

#define DBG_TAG "BW3532"
#define DBG_LVL DBF_LOG
#include <rtdbg.h>

#define BW3532MIB_RST(value) gpio_set_value(IF_BW3532MIB_RST_PORT, IF_BW3232MIB_RST_PIN, value)
#define BW3532MIB_CS(value) gpio_set_value(IF_BW3532MIB_CS_PORT, IF_BW3232MIB_CS_PIN, value)
#define BW3532MIB_SCK(value) gpio_set_value(IF_BW3532MIB_SCK_PORT, IF_BW3232MIB_SCK_PIN, value)
#define BW3532MIB_MOSI(value) gpio_set_value(IF_BW3532MIB_MOSI_PORT, IF_BW3232MIB_MOSI_PIN, value)

static void drv_bw3532mib_reset(void)
{
	BW3532MIB_RST(0);
	rt_thread_mdelay(20);
	BW3532MIB_RST(1);
	rt_thread_mdelay(20);
}

static void drv_bw3532mib_soft_spi_9bit_write(rt_uint16_t data)
{
	for (int i = 0; i < 9; i++)
	{
		BW3532MIB_SCK(0);
		if (data & 0x100)
			BW3532MIB_MOSI(1);
		else
			BW3532MIB_MOSI(0);
		data <<= 1;
		BW3532MIB_SCK(1);
	}
}

static void drv_bw3532mib_write_cmd(rt_uint8_t cmd)
{
	BW3532MIB_CS(0);
	drv_bw3532mib_soft_spi_9bit_write((rt_uint16_t)cmd);
	BW3532MIB_CS(1);
}

static void drv_bw3532mib_write_data(rt_uint8_t data)
{
	BW3532MIB_CS(0);
	drv_bw3532mib_soft_spi_9bit_write((rt_uint16_t)data | 0x100);
	BW3532MIB_CS(1);
}

static void drv_bw3532mib_init(void)
{
	gpio_set_func(IF_BW3532MIB_RST_PORT, IF_BW3232MIB_RST_PIN, IO_OUTPUT);
	gpio_set_func(IF_BW3532MIB_CS_PORT, IF_BW3232MIB_CS_PIN, IO_OUTPUT);
	gpio_set_func(IF_BW3532MIB_SCK_PORT, IF_BW3232MIB_SCK_PIN, IO_OUTPUT);
	gpio_set_func(IF_BW3532MIB_MOSI_PORT, IF_BW3232MIB_MOSI_PIN, IO_OUTPUT);

	BW3532MIB_RST(1);
	BW3532MIB_CS(1);
	BW3532MIB_SCK(1);
	drv_bw3532mib_reset();

	drv_bw3532mib_write_cmd(0x11);
	rt_thread_mdelay(20);

	drv_bw3532mib_write_cmd(0xD0);
	drv_bw3532mib_write_data(0x07);
	drv_bw3532mib_write_data(0x42);
	drv_bw3532mib_write_data(0x19);

	drv_bw3532mib_write_cmd(0xD1);
	drv_bw3532mib_write_data(0x00);
	drv_bw3532mib_write_data(0x14);
	drv_bw3532mib_write_data(0x1B);

	drv_bw3532mib_write_cmd(0xD2);
	drv_bw3532mib_write_data(0x01);
	drv_bw3532mib_write_data(0x12);

	drv_bw3532mib_write_cmd(0xC0);
	drv_bw3532mib_write_data(0x00);
	drv_bw3532mib_write_data(0x3B);
	drv_bw3532mib_write_data(0x00);
	drv_bw3532mib_write_data(0x02);
	drv_bw3532mib_write_data(0x01);

	drv_bw3532mib_write_cmd(0xC5);
	drv_bw3532mib_write_data(0x00);

	drv_bw3532mib_write_cmd(0xC8);
	drv_bw3532mib_write_data(0x00);
	drv_bw3532mib_write_data(0x46);
	drv_bw3532mib_write_data(0x44);
	drv_bw3532mib_write_data(0x50);
	drv_bw3532mib_write_data(0x04);
	drv_bw3532mib_write_data(0x16);
	drv_bw3532mib_write_data(0x33);
	drv_bw3532mib_write_data(0x13);
	drv_bw3532mib_write_data(0x77);
	drv_bw3532mib_write_data(0x05);
	drv_bw3532mib_write_data(0x0F);
	drv_bw3532mib_write_data(0x00);

	drv_bw3532mib_write_cmd(0xB4);
	drv_bw3532mib_write_data(0x10); //RGB

	drv_bw3532mib_write_cmd(0x36);
	drv_bw3532mib_write_data(0xf8);
	drv_bw3532mib_write_cmd(0x3A);
	drv_bw3532mib_write_data(0x66);

	drv_bw3532mib_write_cmd(0x2A);
	drv_bw3532mib_write_data(0x00);
	drv_bw3532mib_write_data(0x00);
	drv_bw3532mib_write_data(0x01);
	drv_bw3532mib_write_data(0xdf);

	drv_bw3532mib_write_cmd(0x2B);
	drv_bw3532mib_write_data(0x00);
	drv_bw3532mib_write_data(0x00);
	drv_bw3532mib_write_data(0x01);
	drv_bw3532mib_write_data(0x3f);
	rt_thread_mdelay(50);
	drv_bw3532mib_write_cmd(0x29);
	drv_bw3532mib_write_cmd(0x2C);

	rt_thread_mdelay(10);
}

static rt_err_t lcd_init(rt_device_t dev)
{
	return RT_EOK;
}

static rt_err_t lcd_open(rt_device_t dev, rt_uint16_t oflag)
{
	return RT_EOK;
}

static rt_err_t lcd_close(rt_device_t dev)
{
	return RT_EOK;
}

static rt_err_t lcd_control(rt_device_t dev, int cmd, void *arg)
{
	switch (cmd)
	{
	case RTGRAPHIC_CTRL_GET_INFO:
		rt_memcpy(arg, dev->user_data, sizeof(struct rt_device_graphic_info));
		return RT_EOK;

	case RTGRAPHIC_CTRL_GET_EXT:
		return RT_EOK;

	case RTGRAPHIC_CTRL_RECT_UPDATE:
		return RT_EOK;
	}
	return RT_ERROR;
}

#ifdef RT_USING_DEVICE_OPS
static const struct rt_device_ops ops =
	{
		.init = lcd_init,
		.open = lcd_open,
		.close = lcd_close,
		.control = lcd_control};
#endif

static int rt_hw_lcd_init(void)
{
	static rt_uint32_t framebuffer[BW3532MIB_WIDTH * BW3532MIB_HEIGHT] __attribute__((aligned(4))) = {0};

	struct display_debe_layer_param debe_param = {
		.enable = RT_TRUE,
		.width = BW3532MIB_WIDTH,
		.height = BW3532MIB_HEIGHT,
		.x = 0,
		.y = 0,
		.layer_index = 0,
		.priority = 3,
		.framebuffer = framebuffer};

	struct display_param lcd_param = {
		.width = BW3532MIB_WIDTH,
		.hspw = 2,
		.hbp = 3,
		.hfp = 3,
		.height = BW3532MIB_HEIGHT,
		.vspw = 2,
		.vbp = 2,
		.vfp = 4,
		.clk = 8 * 1000 * 1000,
		.depth = TCON_16_DEPTH,
		.auto_reload = RT_TRUE,
		.io_pol = {
			.data_pol = 0,
			.clk_pol = 0,
			.de_pol = 1,
			.hsync = 1,
			.vsync = 0}};

	static struct rt_device dev;
	static struct rt_device_graphic_info lcd_info;

	rt_err_t res;

	drv_bw3532mib_init();

	dev.type = RT_Device_Class_Graphic;
#ifdef RT_USING_DEVICE_OPS
	dev.ops = &ops;
#else
	dev.init = lcd_init;
	dev.open = lcd_open;
	dev.close = lcd_close;
	dev.control = lcd_control;
#endif

	lcd_info.pixel_format = RTGRAPHIC_PIXEL_FORMAT_ARGB888,
	lcd_info.bits_per_pixel = 24;
	lcd_info.width = BW3532MIB_WIDTH;
	lcd_info.height = BW3532MIB_HEIGHT;
	lcd_info.framebuffer = (rt_uint8_t *)debe_param.framebuffer;

	dev.user_data = &lcd_info;
	res = drv_lcd_init(&lcd_param, &debe_param);
	if (res != RT_EOK)
	{
		LOG_E("LCD initialization failed");
		return RT_ERROR;
	}

	return rt_device_register(&dev, "lcd", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);
}

INIT_DEVICE_EXPORT(rt_hw_lcd_init);
