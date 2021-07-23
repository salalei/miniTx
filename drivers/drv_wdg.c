#include "drv_wdg.h"
#include "rtthread.h"

#define DBG_TAG "WDG"
#define DBG_LVL DBG_ERROR
#include <rtdbg.h>

/**
 * @brief Set watchdog feeding period
 */
void drv_wdg_set_period(enum wdg_period period)
{
	rt_uint32_t value = WDG->mode;

	RT_ASSERT(period <= wdg_period_16s);

	value &= ~(0xf << 4);
	value |= period << 4;
	WDG->mode = value;
}

/**
 * @brief Read watchdog feeding period
 */
enum wdg_period drv_wdg_read_period(void)
{
	return (WDG->mode & 0xf << 4) >> 4;
}

/**
 * @brief Enable or disable watchdog
 *
 * @param enable 
 *   @arg RT_TRUE enable watchdog
 *   @arg RT_FALSE disable watchdog
 */
void drv_wdg_enable(rt_bool_t enable)
{
	if(enable)
	{
		WDG->cfg |= 1;
		WDG->mode |= 1;
	}
	else
		WDG->mode &= ~1;
}

/**
 * @brief Reset watchdog counter
 */
void drv_wdg_feed(void)
{
	WDG->ctrl = 0xa57 << 1 | 1;
}


#ifdef RT_USING_WDT

#include <drivers/watchdog.h>

static rt_err_t wdg_set_timeout(rt_uint32_t timeout)
{
	if(timeout == 0)
	{
		LOG_E("Timeout is 0");
		return RT_EINVAL;
	}
	if(timeout <= 6)
		drv_wdg_set_period(timeout);
	else if(timeout <= 16)
		drv_wdg_set_period((timeout - 6) / 2 + 6);
	else
		drv_wdg_set_period(wdg_period_16s);
	
	return RT_EOK;
}

static rt_uint32_t wdg_get_timeout(void)
{
	enum wdg_period period = drv_wdg_read_period();

	switch (period)
	{
	case wdg_period_0_5s:
		return 0;
	case wdg_period_1s:
		return 1;
	case wdg_period_2s:
		return 2;
	case wdg_period_3s:
		return 3;
	case wdg_period_4s:
		return 4;
	case wdg_period_5s:
		return 5;
	case wdg_period_6s:
		return 6;
	case wdg_period_8s:
		return 8;
	case wdg_period_10s:
		return 10;
	case wdg_period_12s:
		return 12;
	case wdg_period_14s:
		return 14;
	case wdg_period_16s:
		return 16;
	}

	return 0;
}

rt_err_t wdg_init(rt_watchdog_t *wdt)
{
	return RT_EOK;
}

rt_err_t wdg_control(rt_watchdog_t *wdt, int cmd, void *arg)
{
	switch(cmd)
	{
	case RT_DEVICE_CTRL_WDT_GET_TIMEOUT:
		*(rt_uint32_t *)arg = wdg_get_timeout();
		break;
	
	case RT_DEVICE_CTRL_WDT_SET_TIMEOUT:
		return wdg_set_timeout(*(rt_uint32_t *)arg);
	
	case RT_DEVICE_CTRL_WDT_GET_TIMELEFT:
		LOG_E("The command is not supported in this chip");
		return RT_EINVAL;

	case RT_DEVICE_CTRL_WDT_KEEPALIVE:
		drv_wdg_feed();
		break;

	case RT_DEVICE_CTRL_WDT_START:
		drv_wdg_enable(RT_TRUE);
		drv_wdg_feed();
		break;

	case RT_DEVICE_CTRL_WDT_STOP:
		drv_wdg_enable(RT_FALSE);
		break;
	}

	return RT_EOK;
}

struct rt_watchdog_ops ops =
{
	.init = wdg_init,
	.control = wdg_control
};

int rt_hw_wdg_init(void)
{
	static struct rt_watchdog_device wdg_dev =
	{
		.ops = &ops,
	};	

	return rt_hw_watchdog_register(&wdg_dev, "wdg", RT_DEVICE_FLAG_STANDALONE, RT_NULL);
}

INIT_DEVICE_EXPORT(rt_hw_wdg_init);

#endif