#include <rtthread.h>
#include <rthw.h>

#include "interrupt.h"
#include "drv_clock.h"
#include "drv_timer.h"

#define DBG_TAG  "TIMER"
#define DBG_LVL  DBG_ERROR
#include <rtdbg.h>

/**
 * @brief Read the current counter value of the timer
 *
 * @param timerx TINA_TIMER0~2
 *
 * @return The counter value of the timer
 */
rt_uint32_t drv_timer_read_counter(enum tina_timerx timerx)
{
	RT_ASSERT(timerx < 3);

	return TIMER->child_timer[timerx].val;
}

/**
 * @brief Read the loading value of the timer
 *
 * @param timerx TINA_TIMER0~2
 *
 * @return The loading value of the timer
 */
rt_uint32_t drv_timer_read_loader(enum tina_timerx timerx)
{
	RT_ASSERT(timerx < 3);

	return TIMER->child_timer[timerx].inter;
}

/**
 * @brief Enable or disable timer
 *
 * @param timerx TINA_TIMER0~2
 * @param enable 
 *   @arg RT_TRUE enable timer
 *   @arg RT_FALSE disable timer
 */
void drv_timer_enable(enum tina_timerx timerx, rt_bool_t enable)
{
	RT_ASSERT(timerx < 3);

	if(enable)
		TIMER->child_timer[timerx].ctrl |= 1;
	else
		TIMER->child_timer[timerx].ctrl &= ~1;
}

/**
 * @brief Enable or disable timer interrupt
 *
 * @param timerx TINA_TIMER0~2
 * @param enable 
 *   @arg RT_TRUE enable timer interrupt
 *   @arg RT_FALSE disable timer interrupt
 */
void drv_timer_irq_enable(enum tina_timerx timerx, rt_bool_t enable)
{
	RT_ASSERT(timerx < 3);

	if(enable)
		TIMER->irq_en |= 1 << timerx;
	else
		TIMER->irq_en &= ~(1 << timerx);
}

/**
 * @brief Initializes the timer
 *
 * @param timerx TINA_TIMER0~2
 * @param div Select the pre-scale of timer clock source,the timer frequency = 24000000 / 2^div
 * @param count The loading value of the timer
 */
void drv_timer_init(enum tina_timerx timerx, rt_uint8_t div, rt_uint32_t count)
{
	RT_ASSERT(timerx < 3 && div < 8);
	
	TIMER->child_timer[timerx].inter = count;
	TIMER->child_timer[timerx].ctrl = 0 << 7 | div << 4 | 1 << 2 | 1 << 1;
	TIMER->child_timer[timerx].val = 0;
}