#include <rtthread.h>

#include "drv_key_adc.h"
#include "drv_clock.h"

#define DBG_TAG  "KEY ADC"
#define DBG_LVL  DBG_LOG
#include <rtdbg.h>

/**
 * @brief Initialize KEY ADC peripheral
 * 
 * @param rate KEY ADC sampling rate
 */
void drv_key_adc_init(enum key_adc_sample_rate rate)
{
	RT_ASSERT(rate < 4);
	//Delay 1 sample,normal mode
	KEY_ADC->ctrl = 1 << 24 | rate << 2;
}

/**
 * @brief Enable or disable KEY ADC
 * 
 * @param enable
 *    @arg RT_TRUE Enable KEY ADC
 * 	  @arg RT_FALSE Disable KEY ADC
 */
void drv_key_adc_enable(rt_bool_t enable)
{
	if(enable)
		KEY_ADC->ctrl |= 1 << 0;
	else
		KEY_ADC->ctrl &= ~1 << 0;
}

/**
 * @brief Enable or disable KEY ADC interrupt
 * 
 * @param type KEY ADC interrupt type,the parameter can be combination of the following values
 *    @arg KEY_ADC_IRQ_KEYUP
 *    @arg KEY_ADC_IRQ_ALREADY_HOLD
 *    @arg KEY_ADC_IRQ_HOLD
 *    @arg KEY_ADC_IRQ_KEYDOWN
 *    @arg KEY_ADC_IRQ_DATA
 * @param enable
 *    @arg RT_TRUE Enable KEY ADC interrupt
 * 	  @arg RT_FALSE Disable KEY ADC interrupt
 */
void drv_key_adc_irq_enable(enum key_adc_irq_type type, rt_bool_t enable)
{
	RT_ASSERT((type & (~(KEY_ADC_IRQ_KEYUP | KEY_ADC_IRQ_ALREADY_HOLD | KEY_ADC_IRQ_HOLD | KEY_ADC_IRQ_KEYDOWN | \
		KEY_ADC_IRQ_DATA))) == 0 && type != 0);
	
	if(enable)
		KEY_ADC->irq_en |= type;
	else
		KEY_ADC->irq_en &= ~type;
}

/**
 * @brief Read KEY ADC data
 * 
 * @return KEY ADC data,6-bit resolution 
 */
rt_uint8_t drv_key_adc_read_data(void)
{
	return KEY_ADC->data;
}


#if defined TINA_USING_KEY_ADC && defined RT_USING_ADC

#include <rtdevice.h>
#include "drivers/adc.h"

static rt_err_t adc_enabled(struct rt_adc_device *device, rt_uint32_t channel, rt_bool_t enabled)
{
	drv_key_adc_enable(enabled);

	return RT_EOK;
}

static rt_err_t adc_convert(struct rt_adc_device *device, rt_uint32_t channel, rt_uint32_t *value)
{
	if(channel > 1)
	{
		LOG_E("KEY ADC has only 1 channel");
		return RT_EINVAL;
	}
	*value = drv_key_adc_read_data();

	return RT_EOK;
}

static const struct rt_adc_ops ops =
{
	.enabled = adc_enabled,
	.convert = adc_convert
};

static int rt_hw_key_adc_init(void)
{
	static struct rt_adc_device dev;

	drv_key_adc_init(KEY_ADC_SAMPLE_RATE_250Hz);

	return rt_hw_adc_register(&dev, "key_adc", &ops, RT_NULL);
}

INIT_DEVICE_EXPORT(rt_hw_key_adc_init);

#endif


