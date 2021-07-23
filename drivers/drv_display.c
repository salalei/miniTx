#include <rtthread.h>

#include "drv_display.h"
#include "drv_clock.h"
#include "drv_gpio.h"

#define DBG_TAG  "DISPLAY"
#define DBG_LVL  DBF_ERROR
#include <rtdbg.h>

static const rt_uint32_t tina_vert_coef[32] = {
	0x00004000, 0x000140ff, 0x00033ffe, 0x00043ffd,
	0x00063efc, 0xff083dfc, 0x000a3bfb, 0xff0d39fb,
	0xff0f37fb, 0xff1136fa, 0xfe1433fb, 0xfe1631fb,
	0xfd192ffb, 0xfd1c2cfb, 0xfd1f29fb, 0xfc2127fc,
	0xfc2424fc, 0xfc2721fc, 0xfb291ffd, 0xfb2c1cfd,
	0xfb2f19fd, 0xfb3116fe, 0xfb3314fe, 0xfa3611ff,
	0xfb370fff, 0xfb390dff, 0xfb3b0a00, 0xfc3d08ff,
	0xfc3e0600, 0xfd3f0400, 0xfe3f0300, 0xff400100,
};

static const rt_uint32_t tina_horz_coef[64] = {
	0x40000000, 0x00000000, 0x40fe0000, 0x0000ff03,
	0x3ffd0000, 0x0000ff05, 0x3ffc0000, 0x0000ff06,
	0x3efb0000, 0x0000ff08, 0x3dfb0000, 0x0000ff09,
	0x3bfa0000, 0x0000fe0d, 0x39fa0000, 0x0000fe0f,
	0x38fa0000, 0x0000fe10, 0x36fa0000, 0x0000fe12,
	0x33fa0000, 0x0000fd16, 0x31fa0000, 0x0000fd18,
	0x2ffa0000, 0x0000fd1a, 0x2cfa0000, 0x0000fc1e,
	0x29fa0000, 0x0000fc21, 0x27fb0000, 0x0000fb23,
	0x24fb0000, 0x0000fb26, 0x21fb0000, 0x0000fb29,
	0x1ffc0000, 0x0000fa2b, 0x1cfc0000, 0x0000fa2e,
	0x19fd0000, 0x0000fa30, 0x16fd0000, 0x0000fa33,
	0x14fd0000, 0x0000fa35, 0x11fe0000, 0x0000fa37,
	0x0ffe0000, 0x0000fa39, 0x0dfe0000, 0x0000fa3b,
	0x0afe0000, 0x0000fa3e, 0x08ff0000, 0x0000fb3e,
	0x06ff0000, 0x0000fb40, 0x05ff0000, 0x0000fc40,
	0x03ff0000, 0x0000fd41, 0x01ff0000, 0x0000fe42,
};

/**
 * @brief Pin D0 to D21 used as LCD function
 */
void drv_lcd_cfg_pinmux(enum tcon_depth depth)
{
    for(int i = 0; i < 22 ; i++)
    {
		if(depth == TCON_18_DEPTH || (i != 0 && i != 12))
        	gpio_set_func(GPIO_PORT_D, i, IO_FUN_1);
    }
}

/**
 * @brief Set LCD clock frequency
 * 
 * @param clk LCD clock frequency
 * @return The error code
 */
rt_err_t drv_lcd_set_clk(int clk)
{
	rt_int64_t n, m, diff;
	rt_int32_t best_n = 0, best_m = 0, best_diff = 0x0FFFFFFF, pll;
	rt_bool_t best_double = RT_FALSE;

	for (m = 5; m <= 96; m++)
    {
		n = (m * clk) / 3000000;
		if ((n >= 9) && (n <= 127))
        {
			diff = clk - (3000000 * n) / m;
			if (diff < best_diff) {
				best_diff = diff;
				best_m = m;
				best_n = n;
				best_double = RT_FALSE;
			}
		}

		if (!(m & 1))
			continue;

		n = (m * clk) / 6000000;
		if ((n >= 9) && (n <= 127))
        {
			diff = clk - (6000000 * n) / m;
			if (diff < best_diff)
            {
				best_diff = diff;
				best_m = m;
				best_n = n;
				best_double = RT_TRUE;
			}
		}
	}
    LOG_D("Video frequency:%d", best_n * 3000000);
    if(video_set_pll_clk(best_n * 3000000) != RT_EOK)
        return RT_ERROR;

    if (best_double)
        pll = 2 << 24;
    else
        pll = 0;

    CCU->tcon_clk = 1 << 31 | pll;
    TCON->tcon0_dclk = 0xf << 28 | best_m;

    return RT_EOK;
}

static rt_err_t drv_de_set_clk(volatile rt_uint32_t *reg_addr, int clk)
{
	int pll = periph_get_pll_clk();
	int div = 1;

	while ((pll / div) > clk)
		div++;

    if(div > 16)
        return RT_ERROR;
    *reg_addr = 1 << 31 | 2 << 24 | (div - 1);

    return RT_EOK;
}

/**
 * @brief Set DEBE module clock frequency
 * 
 * @param clk DEBE module clock frequency
 * @return The error code
 */
rt_err_t drv_debe_set_clk(int clk)
{
    return drv_de_set_clk(&CCU->be_clk, clk);
}

/**
 * @brief Set DEFE module clock frequency
 * 
 * @param clk DEFE module clock frequency
 * @return The error code
 */
rt_err_t drv_defe_set_clk(int clk)
{
    return drv_de_set_clk(&CCU->fe_clk, clk);
}

/**
 * @brief Initializes the tcon0, tcon0 control LCD timing,output LCD signal and data to io
 * 
 * @param param Pointer to display parameters structure
 */
void drv_tcon0_init(const struct display_param *param)
{
	int bp, total;
    int delay = param->vfp + param->vspw + param->vbp;
	rt_uint32_t *io_pol = (rt_uint32_t *)&param->io_pol;

    if(delay > 30)
        delay = 30;
    TCON->tcon0_ctrl = 1 << 31 | delay << 4;
    TCON->tcon0_timing_active = (param->width - 1) << 16 | (param->height - 1);

    bp = param->hbp + param->hspw;
    total = param->hfp + param->width + bp;
    TCON->tcon0_timing_h = (total - 1) << 16 | (bp - 1);

    bp = param->vbp + param->vspw;
    total = param->vfp + param->height + bp;
    TCON->tcon0_timing_v = (total * 2) << 16 | (bp - 1);

    TCON->tcon0_timing_sync = (param->hspw - 1) << 16 | (param->vspw - 1);

    TCON->tcon0_hv_intf = 0;
    TCON->tcon0_cpu_intf = 0;

    TCON->tcon0_frm_seed[0] = 0x11111111;
    TCON->tcon0_frm_seed[1] = 0x11111111;
    TCON->tcon0_frm_seed[2] = 0x11111111;
    TCON->tcon0_frm_seed[3] = 0x11111111;
    TCON->tcon0_frm_seed[4] = 0x11111111;
    TCON->tcon0_frm_seed[5] = 0x11111111;

    TCON->tcon0_frm_table[0] = 0x01010000;
    TCON->tcon0_frm_table[1] = 0x15151111;
    TCON->tcon0_frm_table[2] = 0x57575555;
    TCON->tcon0_frm_table[3] = 0x7F7F7777;
    if(param->depth == TCON_18_DEPTH)
        TCON->tcon0_frm_ctrl = 1 << 31;
    else
        TCON->tcon0_frm_ctrl = 1 << 31 | 5 << 4;

	TCON->tcon0_io_polarity = *io_pol;
    TCON->tcon0_io_tristate = 0;

    TCON->ctrl = 1 << 31;
}

/**
 * @brief Reset LCD
 */
void drv_lcd_reset(void)
{
	bus_software_reset_disable(LCD_GATING);
	bus_software_reset_enable(LCD_GATING);
}

/**
 * @brief Initializes DEFE module
 */
void drv_defe_init(void)
{
	DEFE->enable = 1;

	for (int i = 0; i < 32; i++)
	{
		DEFE->ch0_horzcoef0[i] = tina_horz_coef[2 * i];
		DEFE->ch0_horzcoef1[i] = tina_horz_coef[2 * i + 1];
		DEFE->ch0_vertcoef[i] = tina_vert_coef[i];
		DEFE->ch1_horzcoef0[i] = tina_horz_coef[2 * i];
		DEFE->ch1_horzcoef1[i] = tina_horz_coef[2 * i + 1];
		DEFE->ch1_vertcoef[i] = tina_vert_coef[i];
	}
}

/**
 * @brief Reset DEFE module
 */
void drv_defe_reset(void)
{
	bus_software_reset_disable(DEFE_GATING);
	bus_software_reset_enable(DEFE_GATING);
}

/**
 * @brief Set DEFE module channel parameters
 * 
 * @param param Pointer to DEFE channel parameter structure
 * @return The error code
 */
rt_err_t drv_defe_set_ch_param(struct display_defe_ch_param *param)
{
	DEFE->bypass = 1 << 1 | 1;
	DEFE->ch0_addr = (rt_uint32_t)param->framebuffer;
	DEFE->ch0_stride = param->in_width * 4;
	DEFE->input_fmt = 1 << 8 | 5 << 4 | 1;
	DEFE->output_fmt = 2;
	DEFE->ch0_insize = (param->in_height - 1) << 16 | (param->in_width - 1);
	DEFE->ch0_outsize = (param->out_height - 1) << 16 | (param->out_width - 1);
	DEFE->ch0_horzfact = 1 << 16 | 1;
	DEFE->ch0_vertfact = 1 << 16 | 1;
	DEFE->frame_ctrl = 1 << 16 | 1;

	return RT_EOK;
}

/**
 * @brief Initializes DEBE module
 * 
 * @param param Pointer to display parameters structure
 */
void drv_debe_init(const struct display_param *param)
{
    for (int i = 0x0800; i < 0x1000; i += 4)
        *(rt_uint32_t *)(DEBE_BASE_ADDR + i) = 0;

    DEBE->disp_size = (param->height - 1) << 16 | (param->width - 1);
	DEBE->backcolor = 0xffffffff;
	if(param->auto_reload)
		DEBE->reg_ctrl = 1;
	else
		DEBE->reg_ctrl = 1 << 1 | 1;
    DEBE->mode = 1 << 1 | 1;
}

/**
 * @brief Reset DEBE module
 */
void drv_debe_reset(void)
{
	bus_software_reset_disable(DEBE_GATING);
	bus_software_reset_enable(DEBE_GATING);
}

/**
 * @brief LCD framebuffer will be reloaded
 */
void drv_debe_reload_buffer(void)
{
	DEBE->reg_ctrl = 1;
}

/**
 * @brief Set DEBE module layer parameters
 * 
 * @param param Pointer to struct DEBE layer parameters structure
 * @return The error code
 */
rt_err_t drv_debe_set_layer_param(struct display_debe_layer_param *param)
{
	if(param->layer_index > 3)
	{
		LOG_E("layer_index > 3");
		return RT_EINVAL;
	}

	if(param->enable == RT_FALSE)
	{
		DEBE->mode &= ~(1 << (param->layer_index + 8));
		return RT_EOK;
	}

	if(param->layer_index == 0)
	{
		DEBE->layer0_size = (param->height - 1) << 16 | (param->width - 1);
		DEBE->layer0_stride = param->width << 5;
		DEBE->layer0_addr_low32b = (rt_uint32_t)param->framebuffer << 3;
		DEBE->layer0_addr_high4b = (rt_uint32_t)param->framebuffer >> 29;
		DEBE->layer0_pos = param->y << 16 | param->x;
		DEBE->layer0_attr1_ctrl = param->priority << 10 | 1 << 8;
	}
	else if(param->layer_index == 1)
	{
		DEBE->layer1_size = (param->height - 1) << 16 | (param->width - 1);
		DEBE->layer1_stride = param->width << 5;
		DEBE->layer1_addr_low32b = (rt_uint32_t)param->framebuffer << 3;
		DEBE->layer1_addr_high4b = (rt_uint32_t)param->framebuffer >> 29;
		DEBE->layer1_pos = param->y << 16 | param->x;
		DEBE->layer1_attr1_ctrl = param->priority << 10 | 1 << 8;
	}
	else if(param->layer_index == 2)
	{
		DEBE->layer2_size = (param->height - 1) << 16 | (param->width - 1);
		DEBE->layer2_stride = param->width << 5;
		DEBE->layer2_addr_low32b = (rt_uint32_t)param->framebuffer << 3;
		DEBE->layer2_addr_high4b = (rt_uint32_t)param->framebuffer >> 29;
		DEBE->layer2_pos = param->y << 16 | param->x;
		DEBE->layer2_attr1_ctrl = param->priority << 10 | 1 << 8;
	}
	else if(param->layer_index == 3)
	{
		DEBE->layer3_size = (param->height - 1) << 16 | (param->width - 1);
		DEBE->layer3_stride = param->width << 5;
		DEBE->layer3_addr_low32b = (rt_uint32_t)param->framebuffer << 3;
		DEBE->layer3_addr_high4b = (rt_uint32_t)param->framebuffer >> 29;
		DEBE->layer3_pos = param->y << 16 | param->x;
		DEBE->layer3_attr1_ctrl = param->priority << 10 | 1 << 8;
	}

	DEBE->mode |= 1 << (param->layer_index + 8);

	return RT_EOK;
}

/**
 * @brief Initializes LCD
 * 
 * @param lcd_param Pointer to struct display parameters structure
 * @param debe_param Pointer to struct DEBE layer parameters structure
 * @return The error code 
 */
rt_err_t drv_lcd_init(struct display_param *lcd_param, struct display_debe_layer_param *debe_param)
{
	//Enable the bus gate
	if(bus_gate_clk_enable(LCD_GATING) != RT_EOK)
    {
        LOG_E("Enable LCD clock failed");
        return RT_ERROR;
    }
	if(bus_gate_clk_enable(DEBE_GATING) != RT_EOK)
    {
        LOG_E("Enable DEBE clock failed");
        return RT_ERROR;
    }
    if(dram_gate_clk_enable(BE_GATING_DRAM) != RT_EOK)
    {
        LOG_E("Enable DEBE DRAM GATE failed");
        return RT_ERROR;
    }

	//Reset registers
	drv_lcd_reset();
	drv_debe_reset();

	//Set clock frequency
    if(drv_debe_set_clk(300000000) != RT_EOK)
    {
        LOG_E("DEBE clock cannot set");
        return RT_ERROR;
    }
    if(drv_lcd_set_clk(lcd_param->clk) != RT_EOK)
    {
        LOG_E("LCD clock cannot set");
        return RT_ERROR;
    }

    drv_lcd_cfg_pinmux(lcd_param->depth);
    drv_debe_init(lcd_param);
	drv_debe_set_layer_param(debe_param);
    drv_tcon0_init(lcd_param);

	return RT_EOK;
}


