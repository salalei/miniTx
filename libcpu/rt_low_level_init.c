/*
 * File      : rt_low_level_init.c
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
 * 2018-02-08     RT-Thread    the first version
 */

#include "rtthread.h"
#include "drv_clock.h"
#include "drv_gpio.h"

void rt_low_level_init(void)
{
    volatile unsigned int *addr;
    volatile unsigned int time;
    int i;

    //change cpu clk source to 24M
    CCU->cpu_clk_src = 0x10000;
    //init cpu pll clk 408M
    CCU->pll_cpu_ctrl = 0x80001000;
    time = 0xffff;
    while ((!(CCU->pll_cpu_ctrl & (0x1 << 28))) && (time--));
    //change cpu clk source to pll
    if (time > 0)
    {
        CCU->cpu_clk_src = 0x20000;
    }

    //init periph pll clk:600M
    //init ahb    pll clk:200M
    //init apb    pll clk:100M
	if (CCU->pll_periph_ctrl & (0x1 << 31))
        return;

    CCU->pll_stable_time0 = 0x1ff;
	CCU->pll_stable_time1 = 0x1ff;

    CCU->pll_periph_ctrl |= (0x1 << 31);
    while (!(CCU->pll_periph_ctrl & (0x1 << 28)));

    CCU->ahb_apb_hclkc_cfg = (0x0 << 16) | (0x3 << 12) | (0x0 << 8) | (0x2 << 6) | (0x0 << 4);

    //init gpio config
    for (i = 0; i < 6; i++)
    {
        if (i == 1)
            continue;// not config gpio B

        addr = (unsigned int *)(GPIO_BASE_ADDR + i * 0x24 + 0x00);
        *addr = 0x77777777;
        addr = (unsigned int *)(GPIO_BASE_ADDR + i * 0x24 + 0x04);
        *addr = 0x77777777;
        addr = (unsigned int *)(GPIO_BASE_ADDR + i * 0x24 + 0x08);
        *addr = 0x77777777;
        addr = (unsigned int *)(GPIO_BASE_ADDR + i * 0x24 + 0x0C);
        *addr = 0x77777777;
    }
}

