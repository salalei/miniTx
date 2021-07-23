#ifndef __DRV_TP_ADC_H__
#define __DRV_TP_ADC_H__

#include "rtdef.h"

#define TP_ADC_BASE_ADDR              0x01C24800

struct tina_tp_adc
{
	rt_uint32_t ctrl0;/* 0x000 */
	rt_uint32_t ctrl1;/* 0x004 */
	rt_uint32_t ctrl2;/* 0x008 */
	rt_uint32_t ctrl3;/* 0x00c */
	rt_uint32_t int_fifo_ctrl;/* 0x010 */
	rt_uint32_t int_fifo_status;/* 0x014 */
	rt_uint32_t reserved0;
	rt_uint32_t com_data;/* 0x01c */
	rt_uint32_t reserved1;
	rt_uint32_t data;/* 0x024 */
};

typedef struct tina_tp_adc *tina_tp_adc_t;
#define TP_ADC ((tina_tp_adc_t) TP_ADC_BASE_ADDR)

/**
 * @brief TP ADC channel
 */
enum tp_adc_channel
{
	TP_ADC_CHANNEL_0 = 0,
	TP_ADC_CHANNEL_1,
	TP_ADC_CHANNEL_2,
	TP_ADC_CHANNEL_3
};

/**
 * @brief Averaging and median filter size
 */
enum tp_adc_filter
{
	TP_ADC_FILTER_DISABLE = 0,
	TP_ADC_FILTER_4_2,//Averaging filter size = 4,median filter size = 2
	TP_ADC_FILTER_5_3,//Averaging filter size = 5,median filter size = 3
	TP_ADC_FILTER_8_4,//Averaging filter size = 8,median filter size = 4
	TP_ADC_FILTER_16_8,//Averaging filter size = 16,median filter size = 8
};

/**
 * @brief TP interrupt type
 */
enum tp_irq_type
{
	TP_IRQ_OVERRUN = 1 << 17,//TP fifo over run irq
	TP_IRQ_DATA_READY = 1 << 16,//TP data ready irq
	TP_IRQ_UP = 1 << 1,//TP last touch (stylus up) irq
	TP_IRQ_DOWN = 1 << 0//TP last touch (stylus down) irq
};

/**
 * @brief Check whether the specified interrupt is set or not
 * @param type TP interrupt type
 * 
 * @return The status of the specified interrupt
 */
static inline rt_bool_t drv_tp_get_irq_status(enum tp_irq_type type)
{
	RT_ASSERT(type == TP_IRQ_OVERRUN || type == TP_IRQ_DATA_READY || type == TP_IRQ_UP || type == TP_IRQ_DOWN);

	if(TP_ADC->int_fifo_status & type)
		return RT_TRUE;
	else
		return RT_FALSE;
}

/**
 * @brief Clear the specified interrupt status
 * 
 * @param type TP ADC interrupt type,the parameter can be combination of the following values
 *    @arg TP_IRQ_OVERRUN
 *    @arg TP_IRQ_DATA_READY
 *    @arg TP_IRQ_UP
 *    @arg TP_IRQ_DOWN
 */
static inline void drv_tp_clear_irq(enum tp_irq_type type)
{
	RT_ASSERT((type & (~(TP_IRQ_OVERRUN | TP_IRQ_DATA_READY | TP_IRQ_UP | TP_IRQ_DOWN))) == 0 && type != 0);

	TP_ADC->int_fifo_status |= type;
}

void drv_tp_adc_init(rt_uint8_t fs_div, rt_uint16_t tacq, enum tp_adc_filter filter);
void drv_tp_drq_enable(rt_bool_t enable);
void drv_tp_irq_enable(enum tp_irq_type type, rt_bool_t enable);
void drv_tp_enable(rt_bool_t enable);
void drv_tp_adc_chanel_enable(enum tp_adc_channel channel, rt_bool_t enable);
rt_uint8_t drv_tp_read_fifo_counter(void);
void drv_tp_fifo_flush(void);
rt_uint16_t drv_tp_read_data(void);

#endif // ! __DRV_TPADC_H__