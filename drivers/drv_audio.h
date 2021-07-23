#ifndef __DRV_AUDIO_H__
#define __DRV_AUDIO_H__

#include <rtdef.h>

#define AUDIO_BASE_ADDR				0x01C23C00	

struct tina_audio
{
	volatile rt_uint32_t dac_dpc;/* 0x00 */
	volatile rt_uint32_t dac_fifo_ctrl;/* 0x04 */
	volatile rt_uint32_t dac_fifo_status;/* 0x08 */
	volatile rt_uint32_t dac_txdata;/* 0x0c */
	volatile rt_uint32_t adc_fifo_ctrl;/* 0x10 */
	volatile rt_uint32_t adc_fifo_status;/* 0x14 */
	volatile rt_uint32_t adc_rxdata;/* 0x18 */
	volatile rt_uint32_t res0;/* 0x1C */
	volatile rt_uint32_t dac_mixer_ctrl;/* 0x20 */
	volatile rt_uint32_t adc_mixer_ctrl;/* 0x24 */
	volatile rt_uint32_t adda_tune;/* 0x28 */
	volatile rt_uint32_t bias_da16_cal_ctrl0;/* 0x2C */
	volatile rt_uint32_t res1;/* 0x30 */
	volatile rt_uint32_t bias_da16_cal_ctrl1;/* 0x34 */
	volatile rt_uint8_t res2[8];/* 0x38 */
	volatile rt_uint32_t dac_cnt;/* 0x40 */
	volatile rt_uint32_t adc_cnt;/* 0x44 */
	volatile rt_uint32_t dac_debug;/* 0x48 */
	volatile rt_uint32_t adc_debug;/* 0x4c */
	volatile rt_uint8_t res3[32];/* 0x50 */
	volatile rt_uint32_t adc_dap_ctrl;/* 0x70 */
	volatile rt_uint32_t adc_dap_l_ctrl;/* 0x74 */
	volatile rt_uint32_t adc_dap_r_ctrl;/* 0x78 */
	volatile rt_uint32_t adc_dap_param;/* 0x7c */
	volatile rt_uint32_t adc_dap_l_ac;/* 0x80 */
	volatile rt_uint32_t adc_dap_l_dat;/* 0x84 */
	volatile rt_uint32_t adc_dap_r_ac;/* 0x88 */
	volatile rt_uint32_t adc_dap_r_dat;/* 0x8c */
	volatile rt_uint32_t adc_dap_hpfc;/* 0x90 */
	volatile rt_uint32_t adc_dap_l_inac;/* 0x94 */
	volatile rt_uint32_t adc_dap_r_inac;/* 0x98 */
	volatile rt_uint32_t adc_dap_ort;/* 0x9c */
};

typedef struct tina_audio *tina_audio_t;
#define AUDIO ((tina_audio_t) AUDIO_BASE_ADDR)

enum audio_irq_type
{
	AUDIO_IRQ_FIFO_EMPTY = 1 << 3,
	AUDIO_IRQ_FIFO_UNDERRUN = 1 << 2,
	AUDIO_IRQ_FIFO_OVERRUN = 1 << 1
};


static inline rt_bool_t drv_audio_dac_irq_status(enum audio_irq_type type)
{
	RT_ASSERT(type == AUDIO_IRQ_FIFO_EMPTY || type == AUDIO_IRQ_FIFO_UNDERRUN || type == AUDIO_IRQ_FIFO_OVERRUN);

	if(AUDIO->dac_fifo_status & type)
		return RT_TRUE;
	else
		return RT_FALSE;
}

static inline void drv_audio_dac_clear_irq(enum audio_irq_type type)
{
	RT_ASSERT((type & (~(AUDIO_IRQ_FIFO_EMPTY | AUDIO_IRQ_FIFO_UNDERRUN | AUDIO_IRQ_FIFO_OVERRUN))) == 0 && type != 0);

	AUDIO->dac_fifo_status |= type;
}


#endif