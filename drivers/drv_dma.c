#include <rtthread.h>

#include "drv_dma.h"
#include "drv_clock.h"
#include "interrupt.h"

#define DBG_TAG  "DMA"
#define DBG_LVL  DBG_ERROR
#include <rtdbg.h>

static void (*dma_irq_callback[8])(void *param) = {RT_NULL};
static void *dma_irq_param[8] = {RT_NULL};

/**
 * @brief Enable or disable or disable DMA interrupt
 *
 * @param dmax xDMA_CHy:where x can be D or N to select the DMA and y can be 0 to 3 to select the channel
 * @param type DMA interrupt type,the parameter can be combination of the following values
 *    @arg DMA_IRQ_HT Half transfer complete flag
 *    @arg DMA_IRQ_FT Transfer complete flag
 * @param enable
 *   @arg RT_TRUE Enable DMA interrupt
 *   @arg RT_FALSE Disable DMA interrupt
 */
void drv_dma_irq_enable(enum tina_dmax dmax, enum dma_irq_type type, rt_bool_t enable)
{
	rt_uint32_t data;

	RT_ASSERT(dmax < 8 && (type & ~(DMA_IRQ_HT | DMA_IRQ_FT)) == 0 && type != 0);

	data = type << (((dmax & 0x4) ? 16 : 0) + 2 * (dmax & 0x3));
	if(enable)
		DMA->irq_en |= data;
	else
		DMA->irq_en &= ~data;
}

/**
 * @brief Initializes the DMA
 *
 * @param dmax xDMA_CHy:where x can be D or N to select the DMA and y can be 0 to 3 to select the channel
 * @param param Pointer to struct DMA parameters structure
 */
void drv_dma_init(enum tina_dmax dmax, struct dma_param *param)
{
	struct tina_dma_cfg *dma_cfg;
	rt_uint32_t reg_data = 0;

	RT_ASSERT(dmax < 8 && param);

	DMA->auto_gate = 1 << 16;
	if(dmax & 0x4)
		dma_cfg = &DMA->ddma[dmax & 0x3];
	else
		dma_cfg = &DMA->ndma[dmax & 0x3];
	
	dma_cfg->src_addr = (rt_uint32_t)param->src_addr;
	dma_cfg->dst_addr = (rt_uint32_t)param->dst_addr;
	dma_cfg->bc = param->byte_num;

	if(param->continuous_enable)
		reg_data |= 1 << 29;
	if(param->dst_addr_increase == RT_FALSE)
		reg_data |= 1 << 21;
	if(param->src_addr_increase == RT_FALSE)
		reg_data |= 1 << 5;
	reg_data |= param->dst_data_width << 24 | param->dst_drq_type << 16 | 1 << 15 \
		| param->src_data_width << 8 | param->src_drq_type | 1 << 23;

	dma_cfg->ctrl = reg_data;
}

/**
 * @brief Enable or disable DMA
 *
 * @param dmax xDMA_CHy:where x can be D or N to select the DMA and y can be 0 to 3 to select the channel
 * @param enable 
 *   @arg RT_TRUE enable DMA
 *   @arg RT_FALSE disable DMA
 */
void drv_dma_enable(enum tina_dmax dmax, rt_bool_t enable)
{
	struct tina_dma_cfg *dma_cfg;

	RT_ASSERT(dmax < 8);

	if(dmax & 0x4)
		dma_cfg = &DMA->ddma[dmax & 0x3];
	else
		dma_cfg = &DMA->ndma[dmax & 0x3];

	if(enable)
		dma_cfg->ctrl |= 1 << 31;
	else
		dma_cfg->ctrl &= ~(1 << 31);
}

/**
 * @brief Check whether the DMA is busy or not
 *
 * @param dmax xDMA_CHy:where x can be D or N to select the DMA and y can be 0 to 3 to select the channel
 * @return 
 *   @arg RT_TRUE DMA busy
 *   @arg RT_FALSE DMA idle
 */
rt_bool_t drv_dma_check_busy(enum tina_dmax dmax)
{
	struct tina_dma_cfg *dma_cfg;

	RT_ASSERT(dmax < 8);

	if(dmax & 0x4)
		dma_cfg = &DMA->ddma[dmax & 0x3];
	else
		dma_cfg = &DMA->ndma[dmax & 0x3];
	
	return dma_cfg->ctrl & 1 << 30 ? RT_TRUE : RT_FALSE;
}

/**
 * @brief Reset DMA
 */
void drv_dma_reset(void)
{
	bus_software_reset_disable(DMA_GATING);
	bus_software_reset_enable(DMA_GATING);
}

void drv_dma_register_irq_callback(enum tina_dmax dmax, void (*callback)(void *param), void *param)
{
	RT_ASSERT(dmax < 8);

	dma_irq_callback[dmax] = callback;
	dma_irq_param[dmax] = param;
}

static void drv_dma_irq_handler(int vector, void *param)
{
	for(int i = 0; i < 8; i++)
	{
		if(drv_dma_get_irq_status((enum tina_dmax)i, DMA_IRQ_HT | DMA_IRQ_FT))
		{
			if(dma_irq_callback[i] != RT_NULL)
				dma_irq_callback[i](dma_irq_param[i]);
		}
	}
}

void rt_hw_dma_init(void)
{
	drv_dma_reset();
	bus_gate_clk_enable(DMA_GATING);

	rt_hw_interrupt_install(DMA_INTERRUPT, drv_dma_irq_handler, RT_NULL, "DMA");
	rt_hw_interrupt_umask(DMA_INTERRUPT);
}