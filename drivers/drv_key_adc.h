#ifndef __DRV_KEY_ADC_H__
#define __DRV_KEY_ADC_H__

#include "rtdef.h"

#define KEY_ADC_BASE_ADDR              0x01C23400

struct tina_key_adc
{
	rt_uint32_t ctrl;/* 0x000 */
	rt_uint32_t irq_en;/* 0x004 */
	rt_uint32_t irq_pend;/* 0x008 */
	rt_uint32_t data;/* 0x00c */
};

typedef struct tina_key_adc *tina_key_adc_t;
#define KEY_ADC ((tina_key_adc_t) KEY_ADC_BASE_ADDR)

/**
 * @brief KEY ADC sampling rate
 */
enum key_adc_sample_rate
{
	KEY_ADC_SAMPLE_RATE_250Hz = 0,//250Hz
	KEY_ADC_SAMPLE_RATE_125Hz,//125Hz
	KEY_ADC_SAMPLE_RATE_62_5Hz,//62.5Hz
	KEY_ADC_SAMPLE_RATE_32_25Hz//32.25Hz
};

/**
 * @brief KEY ADC interrupt type
 */
enum key_adc_irq_type
{
	KEY_ADC_IRQ_KEYUP = 1 << 4,
	KEY_ADC_IRQ_ALREADY_HOLD = 1 << 3,
	KEY_ADC_IRQ_HOLD = 1 << 2,
	KEY_ADC_IRQ_KEYDOWN = 1 << 1,
	KEY_ADC_IRQ_DATA = 1 << 0
};

/**
 * @brief Check whether the specified interrupt is set or not
 * 
 * @param type KEY ADC interrupt type
 * @return The status of the specified interrupt 
 */
static inline rt_bool_t drv_key_adc_get_irq_status(enum key_adc_irq_type type)
{
	RT_ASSERT(type == KEY_ADC_IRQ_KEYUP || type == KEY_ADC_IRQ_ALREADY_HOLD || type == KEY_ADC_IRQ_HOLD || \
		type == KEY_ADC_IRQ_KEYDOWN || type == KEY_ADC_IRQ_DATA);

	if(KEY_ADC->irq_pend & type)
		return RT_TRUE;
	else
		return RT_FALSE;
}

/**
 * @brief Clear the specified interrupt status
 * 
 * @param type KEY ADC interrupt type,the parameter can be combination of the following values
 *    @arg KEY_ADC_IRQ_KEYUP
 *    @arg KEY_ADC_IRQ_ALREADY_HOLD
 *    @arg KEY_ADC_IRQ_HOLD
 *    @arg KEY_ADC_IRQ_KEYDOWN
 *    @arg KEY_ADC_IRQ_DATA
 */
static inline void drv_key_adc_clear_irq(enum key_adc_irq_type type)
{
	RT_ASSERT((type & (~(KEY_ADC_IRQ_KEYUP | KEY_ADC_IRQ_ALREADY_HOLD | KEY_ADC_IRQ_HOLD | KEY_ADC_IRQ_KEYDOWN | \
		KEY_ADC_IRQ_DATA))) == 0 && type != 0);

	KEY_ADC->irq_pend |= type;
}

void drv_key_adc_init(enum key_adc_sample_rate rate);
void drv_key_adc_enable(rt_bool_t enable);
void drv_key_adc_irq_enable(enum key_adc_irq_type type, rt_bool_t enable);
rt_uint8_t drv_key_adc_read_data(void);

#endif
