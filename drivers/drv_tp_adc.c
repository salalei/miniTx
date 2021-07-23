#include <rtthread.h>

#include "drv_tp_adc.h"
#include "drv_clock.h"
#include "drv_dma.h"

#define DBG_TAG  "TP ADC"
#define DBG_LVL  DBG_LOG
#include <rtdbg.h>

/**
 * @brief Initializes TP ADC,make the TP module work in the Auxiliary ADC mode
 * 
 * @param fs_div ADC sampling frequency divider,ADC sampling frequency = 4MHz / 2 ^ (20 - fs_div)
 * @param tacq Touch panel ADC acquire time,TACQ = 4MHz / (16 * (tacq + 1))
 * @param filter Select the averaging and median filter size 
 * @note ADC sampling time must great or equal M * (TACQ + 3.25us),where M is the number of ADC channels
 */
void drv_tp_adc_init(rt_uint8_t fs_div, rt_uint16_t tacq, enum tp_adc_filter filter)
{
	RT_ASSERT(fs_div < 16 && filter < 5);

	TP_ADC->ctrl0 = 0xf << 24 | 1 << 23 | 2 << 20 | fs_div << 16 | tacq;//TP ADC clock frequency = 24MHz / 6 = 4MHz
	TP_ADC->ctrl1 = 1 << 4;//Disable TP ADC and select Auxiliary ADC mode
	TP_ADC->ctrl2 = 1 << 26;

	if(filter != TP_ADC_FILTER_DISABLE)
		TP_ADC->ctrl3 = 1 << 2 | (filter - 1);
	else
		TP_ADC->ctrl3 = 0;
}

/**
 * @brief Enable or disable the TP DMA interface
 * 
 * @param enable
 *    @arg RT_TRUE Enable the TP DMA interface
 *    @arg RT_FALSE Disable the TP DMA interface
 */
void drv_tp_drq_enable(rt_bool_t enable)
{
	if(enable)
		TP_ADC->int_fifo_ctrl |= 1 << 7;
	else
		TP_ADC->int_fifo_ctrl &= ~(1 << 7);
}

/**
 * @brief Enable or disable TP interrupt
 * 
 * @param type TP ADC interrupt type,the parameter can be combination of the following values
 *    @arg TP_IRQ_OVERRUN
 *    @arg TP_IRQ_DATA_READY
 *    @arg TP_IRQ_UP
 *    @arg TP_IRQ_DOWN
 * @param enable
 *    @arg RT_TRUE Enable the TP ADC interrupt
 *    @arg RT_FALSE Disable the TP ADC interrupt
 */
void drv_tp_irq_enable(enum tp_irq_type type, rt_bool_t enable)
{
	RT_ASSERT((type & (~(TP_IRQ_OVERRUN | TP_IRQ_DATA_READY | TP_IRQ_UP | TP_IRQ_DOWN))) == 0 && type != 0);

	if(enable)
		TP_ADC->int_fifo_ctrl |= type;
	else
		TP_ADC->int_fifo_ctrl &= ~type;
}

/**
 * @brief Enable or disable the TP peripheral
 * 
 * @param enable
 *    @arg RT_TRUE Enable the TP peripheral
 *    @arg RT_FALSE Disable the TP peripheral
 */
void drv_tp_enable(rt_bool_t enable)
{
	if(enable)
		TP_ADC->ctrl1 |= 1 << 5;
	else
		TP_ADC->ctrl1 &= ~(1 << 5);
}

/**
 * @brief Enable or disable the specified TP ADC channel
 * 
 * @param channel The TP ADC channel
 * @param enable
 *    @arg RT_TRUE Enable the specified TP ADC channel
 *    @arg RT_FALSE Disable the specified TP ADC channel
 */
void drv_tp_adc_chanel_enable(enum tp_adc_channel channel, rt_bool_t enable)
{
	RT_ASSERT(channel < 4);

	if(enable)
		TP_ADC->ctrl1 |= 1 << channel;
	else
		TP_ADC->ctrl1 &= ~(1 << channel);
}

/**
 * @brief Read the counter of the TP fifo
 * 
 * @return The counter of the TP fifo
 */
rt_uint8_t drv_tp_read_fifo_counter(void)
{
	rt_uint32_t reg_data = TP_ADC->int_fifo_status;

	return (reg_data >> 8) & 0x1f;
}

/**
 * @brief Flush fifo
 */
void drv_tp_fifo_flush(void)
{
	TP_ADC->int_fifo_ctrl |= 1 << 4;
}

/**
 * @brief Read TP data
 * 
 * @return TP data,12-bit resolution
 */
rt_uint16_t drv_tp_read_data(void)
{
	return TP_ADC->data;
}


#if defined TINA_USING_TP_ADC && defined RT_USING_ADC

#include <rtdevice.h>
#include "drivers/adc.h"

#define TP_ADC_DMA			NDMA_CH3

static rt_err_t adc_enabled(struct rt_adc_device *device, rt_uint32_t channel, rt_bool_t enabled)
{
	return RT_EOK;
}

static rt_err_t adc_convert(struct rt_adc_device *device, rt_uint32_t channel, rt_uint32_t *value)
{
	if(channel > 3)
	{
		LOG_E("TP ADC has only 4 channels");
		return RT_EINVAL;
	}
	*value = ((rt_uint16_t*)device->parent.user_data)[channel];

	return RT_EOK;
}

static const struct rt_adc_ops ops =
{
	.enabled = adc_enabled,
	.convert = adc_convert
};

static int rt_hw_tp_adc_init(void)
{
	static struct rt_adc_device dev;
	static rt_uint16_t adc_res[4] __attribute__((aligned(4)));//The address of AD result must be aligned
	struct dma_param param;

	drv_tp_adc_init(12, 15, TP_ADC_FILTER_16_8);//1KHz sampling
	drv_tp_drq_enable(RT_TRUE);//Enable DMA request
	drv_tp_fifo_flush();

	drv_tp_adc_chanel_enable(TP_ADC_CHANNEL_0, RT_TRUE);
	drv_tp_adc_chanel_enable(TP_ADC_CHANNEL_1, RT_TRUE);
	drv_tp_adc_chanel_enable(TP_ADC_CHANNEL_2, RT_TRUE);
	drv_tp_adc_chanel_enable(TP_ADC_CHANNEL_3, RT_TRUE);

	param.byte_num = 4 * sizeof(rt_uint16_t);
	param.continuous_enable = RT_TRUE;
	param.dst_addr = (rt_uint32_t *)adc_res;
	param.dst_addr_increase = RT_TRUE;
	param.dst_data_width = DMA_DATA_WIDTH_16BIT;
	param.dst_drq_type = NDMA_DST_DRQ_SDRAM;
	param.src_addr = (rt_uint32_t *)&TP_ADC->data;
	param.src_addr_increase = RT_FALSE;
	param.src_data_width = DMA_DATA_WIDTH_16BIT;
	param.src_drq_type = NDMA_SRC_DRQ_TP_ADC;
	drv_dma_init(TP_ADC_DMA, &param);
	drv_dma_enable(TP_ADC_DMA, RT_TRUE);

	drv_tp_enable(RT_TRUE);//Start AD conversion

	return rt_hw_adc_register(&dev, "tp_adc", &ops, adc_res);
}

INIT_DEVICE_EXPORT(rt_hw_tp_adc_init);

#endif