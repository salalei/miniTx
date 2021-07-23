#include "rtthread.h"
#include "rtdevice.h"
#include <drivers/watchdog.h>

#define DBG_TAG "APP WDG"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static rt_device_t wdg_dev = RT_NULL;

static void idle_hook(void)
{
	rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_KEEPALIVE, RT_NULL);
}

static int app_wdg_init(void)
{
	rt_err_t res;

	wdg_dev = rt_device_find("wdg");
	if(wdg_dev == RT_NULL)
	{
		LOG_E("Cannot find device wdg");
		return RT_ERROR;
	}

	rt_uint32_t timeout = 10;
	res = rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_SET_TIMEOUT, (void *)&timeout);
	if(res != RT_EOK)
	{
		LOG_E("Cannot set watchdog timeout");
		return res;
	}

	res = rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_START, RT_NULL);
	if(res != RT_EOK)
	{
		LOG_E("Cannot start watchdog");
		return res;
	}

	rt_thread_idle_sethook(idle_hook);

	return RT_EOK;
}

INIT_APP_EXPORT(app_wdg_init);