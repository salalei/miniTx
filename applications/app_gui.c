#include "littlevgl2rtt.h" 
#include "lvgl.h" 

#include "rtthread.h"
#include "rtdevice.h" 

// #include "lv_demo_music.h"

#define DBG_TAG "APP GUI"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

// static void btn_cb(lv_obj_t *btn, lv_event_t event)
// {
// 	static rt_uint32_t cnt = 0;

// 	if(event == LV_EVENT_CLICKED)
// 	{
// 		cnt++;
// 		lv_obj_t *child = lv_obj_get_child(btn, RT_NULL);
// 		lv_label_set_text_fmt(child, "%d", cnt);
// 	}
// }

static void lvgl_entry(void *p)
{
	lv_demo_widgets();
	// lv_demo_music();
	// lv_obj_t *btn = lv_btn_create(lv_scr_act(), NULL);
	// lv_btn_set_fit(btn, LV_FIT_TIGHT);
	// lv_obj_set_x(btn, 100);
	// lv_obj_set_y(btn, 100);
	// lv_obj_t* label = lv_label_create(btn, NULL);
	// lv_label_set_text_fmt(label, "BUTTON1");
	// lv_obj_t* btn1 = lv_btn_create(lv_scr_act(), btn);
	// lv_obj_set_x(btn1, 190);
	// lv_obj_set_y(btn1, 120);
	// lv_obj_t* label1 = lv_label_create(btn1, NULL);
	// lv_label_set_text_fmt(label1, "BUTTON2");
	// lv_obj_del(label1);
} 

int app_gui_init(void)
{
    rt_err_t ret;
    rt_thread_t thread;

    ret = littlevgl2rtt_init("lcd"); 
    if(ret != RT_EOK)
        return ret;

    thread = rt_thread_create("lv_demo", lvgl_entry, RT_NULL, 10 * 1024, 6, 5); 
    if(thread == RT_NULL)
        return RT_ERROR;

    rt_thread_startup(thread);

    return RT_EOK; 
}
INIT_APP_EXPORT(app_gui_init); 