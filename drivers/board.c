/*
 * File      : board.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2017, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-5-30      Bernard      the first version
 */

#include <rthw.h>
#include <rtthread.h>

#include "board.h"
#include "drv_uart.h"
#include "drv_clock.h"
#include "drv_timer.h"
#include "drv_dma.h"
#include "interrupt.h"
#include "mmu.h"

static void os_clock_irq_handle(int irqno, void *param)
{
    if(drv_timer_get_irq_status(TINA_TIMER0))
	{
		drv_timer_clear_irq(TINA_TIMER0);
		rt_tick_increase();
	}
}

static void mmu_init(void)
{
    struct mem_desc r6_mem_desc[] =
    {
        { 0x00000000, 0xFFFFFFFF, 0x00000000, RW_NCNB },     /* None cached for 4G memory */
        { 0x80000000, 0x82000000 - 1, 0x80000000, RW_CB },   /* 32M cached SDRAM memory */
        //{ 0x00000000, 0x00001000-1, 0x80000000, RW_CB },         /* isr vector table */
        //here not set mmu
        //start_gcc.S Copy vector to the correct address
    };

    rt_hw_mmu_init(r6_mem_desc, sizeof(r6_mem_desc) / sizeof(r6_mem_desc[0]));
}

static void os_clock_init(void)
{
	drv_timer_init(TINA_TIMER0, 0, 24000000 / RT_TICK_PER_SECOND);
	drv_timer_irq_enable(TINA_TIMER0, RT_TRUE);
	drv_timer_enable(TINA_TIMER0, RT_TRUE);

    rt_hw_interrupt_install(TIMER0_INTERRUPT, os_clock_irq_handle, RT_NULL, "os");
    rt_hw_interrupt_umask(TIMER0_INTERRUPT);

}

void rt_hw_board_init(void)
{
	cpu_set_clk(800000000);

    mmu_init();

    rt_hw_interrupt_init();
	
#ifdef RT_USING_HEAP
    /* init memory system */
    rt_system_heap_init(RT_HW_HEAP_BEGIN, RT_HW_HEAP_END);
#endif
	rt_hw_dma_init();
    rt_hw_uart_init();

#ifdef RT_USING_CONSOLE
    /* set console device */
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif /* RT_USING_CONSOLE */

    os_clock_init();

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
}
