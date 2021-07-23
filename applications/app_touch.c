#include "rtthread.h"
#include "rtdevice.h"
#include "touch.h"
#include "littlevgl2rtt.h"

#define DBG_TAG "APP TOUCH"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static rt_device_t touch_dev = RT_NULL;
static rt_sem_t touch_sem;
static rt_thread_t touch_read_thread;

static void touch_read_entry(void *param)
{
	struct rt_touch_data point[5] = {0};
	rt_size_t point_num = 0;
	rt_err_t res;

	while(1)
	{
		res = rt_sem_take(touch_sem, 20);

		if(res == RT_EOK)
		{
			point_num = rt_device_read(touch_dev, 0, point, 5);
			if(point_num)
			{
				littlevgl2rtt_send_input_event(point[0].x_coordinate, 319 - point[0].y_coordinate, point[0].event);
			}

			rt_device_control(touch_dev, RT_TOUCH_CTRL_ENABLE_INT, RT_NULL);
		}
		else
			littlevgl2rtt_send_input_event(point[0].x_coordinate, 319 - point[0].y_coordinate, RT_TOUCH_EVENT_UP);
	}
}

static rt_err_t touch_callback(rt_device_t dev, rt_size_t size)
{
    rt_device_control(touch_dev, RT_TOUCH_CTRL_DISABLE_INT, RT_NULL);
    rt_sem_release(touch_sem);
	
    return RT_EOK;
}

static int app_touch_init(void)
{
	char id[4];

	touch_dev = rt_device_find("touch");
    if (touch_dev == RT_NULL)
    {
        LOG_E("Cannot find touch device");
        return RT_ERROR;
    }
	if (rt_device_open(touch_dev, RT_DEVICE_FLAG_INT_RX) != RT_EOK)
	{
		LOG_E("Open touch device failed!");
		return RT_ERROR;
	}
	
	if(rt_device_control(touch_dev, RT_TOUCH_CTRL_GET_ID, id) == RT_EOK)
		LOG_I("Touch device ID：%c%c%c%c", id[0], id[1], id[2], id[3]);

	rt_int32_t x = 480, y = 320;
    rt_device_control(touch_dev, RT_TOUCH_CTRL_SET_X_RANGE, &x);
    rt_device_control(touch_dev, RT_TOUCH_CTRL_SET_Y_RANGE, &y);

	touch_sem = rt_sem_create("touch sem", 0, RT_IPC_FLAG_FIFO);
	if(touch_sem == RT_NULL)
	{
		LOG_E("Create touch semaphore failed");
	}

	touch_read_thread = rt_thread_create("read touch",
										touch_read_entry,
										RT_NULL,
										10 * 1024,
										10,
										5);
	
	if(touch_read_thread == RT_NULL)
	{
		LOG_E("Create touch read thread failed");
		return RT_ERROR;
	}

	rt_thread_startup(touch_read_thread);

	rt_device_set_rx_indicate(touch_dev, touch_callback);

	LOG_D("touch thread startup");

	return RT_EOK;
}

INIT_APP_EXPORT(app_touch_init);
