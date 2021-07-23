#ifndef __DRV_WDG_H__
#define __DRV_WDG_H__

#include "rtdef.h"

#define WDG_BASE_ADDR              (0x01C20C00 + 0xa0)

struct tina_wdg
{
	volatile rt_uint32_t irq_en;/* 0x00 */
	volatile rt_uint32_t irq_sta;/* 0x04 */
	volatile rt_uint8_t res1[8];
	volatile rt_uint32_t ctrl;/* 0x10 */
	volatile rt_uint32_t cfg;/* 0x14 */
	volatile rt_uint32_t mode;/* 0x18 */
};

typedef struct tina_wdg *tina_wdg_t;
#define WDG ((tina_wdg_t) WDG_BASE_ADDR)

/**
 * @brief Watchdog feeding period
 */
enum wdg_period
{
	wdg_period_0_5s = 0,//0.5s
	wdg_period_1s,
	wdg_period_2s,
	wdg_period_3s,
	wdg_period_4s,
	wdg_period_5s,
	wdg_period_6s,
	wdg_period_8s,
	wdg_period_10s,
	wdg_period_12s,
	wdg_period_14s,
	wdg_period_16s,
};

void drv_wdg_set_period(enum wdg_period period);
enum wdg_period drv_wdg_read_period(void);
void drv_wdg_enable(rt_bool_t enable);
void drv_wdg_feed(void);

#endif // !__DRV_WDG_H__#define __DRV_WDG_H__

