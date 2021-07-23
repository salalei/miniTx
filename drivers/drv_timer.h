#ifndef __DRV_TIMER_H__
#define __DRV_TIMER_H__

#include "rtdef.h"

#define TIMER_BASE_ADDR              0x01C20C00

struct child_timer
{
	volatile rt_uint32_t ctrl;
	volatile rt_uint32_t inter;
	volatile rt_uint32_t val;
	volatile rt_uint8_t res[4];
};

struct child_avs
{
	volatile rt_uint32_t ctrl;
	volatile rt_uint32_t cnt0;
	volatile rt_uint32_t cnt1;
	volatile rt_uint32_t div;
};

struct tina_timer
{
	volatile rt_uint32_t irq_en;/* 0x00 */
	volatile rt_uint32_t irq_pend;/* 0x04 */
	volatile rt_uint8_t res1[8];/* 0x08 */
	struct child_timer child_timer[3];/* 0x10 */
	volatile rt_uint8_t res2[64];/* 0x40 */
	volatile rt_uint32_t avs_ctl;/* 0x80 */
	volatile rt_uint32_t avs_cnt0;/* 0x84 */
	volatile rt_uint32_t avs_cnt1;/* 0x88 */
	volatile rt_uint32_t avs_div;/* 0x8c */
};

typedef struct tina_timer *tina_timer_t;
#define TIMER ((tina_timer_t) TIMER_BASE_ADDR)

/**
 * @brief Allwinner TIMER peripheral
 */
enum tina_timerx
{
	TINA_TIMER0 = 0,
	TINA_TIMER1,
	TINA_TIMER2
};

/**
 * @brief Check whether the specified interrupt is set or not
 *
 * @param timerx TINA_TIMER0~2
 * 
 * @return Interrupt status of the timer
 */
static inline rt_bool_t drv_timer_get_irq_status(enum tina_timerx timerx)
{
	RT_ASSERT(timerx < 3);

	if(TIMER->irq_pend & (1 << timerx))
		return RT_TRUE;
	else
		return RT_FALSE;
}

/**
 * @brief Clear the specified interrupt status
 *
 * @param timerx TINA_TIMER0~2
 */
static inline void drv_timer_clear_irq(rt_uint8_t timerx)
{
	RT_ASSERT(timerx < 3);

	TIMER->irq_pend |= 1 << timerx;
}

rt_uint32_t drv_timer_read_counter(enum tina_timerx timerx);
rt_uint32_t drv_timer_read_loader(enum tina_timerx timerx);
void drv_timer_enable(enum tina_timerx timerx, rt_bool_t enable);
void drv_timer_irq_enable(enum tina_timerx timerx, rt_bool_t enable);
void drv_timer_init(enum tina_timerx timerx, rt_uint8_t div, rt_uint32_t count);

#endif // !__DRV_TIMER_H__