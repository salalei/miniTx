#ifndef __DRV_DMA_H__
#define __DRV_DMA_H__

#include "rtdef.h"

#define DMA_BASE_ADDR              0x01C02000

struct tina_dma_cfg
{
	rt_uint32_t ctrl;/* 0x00 */
	rt_uint32_t src_addr;/* 0x04 */
	rt_uint32_t dst_addr;/* 0x08 */
	rt_uint32_t bc;/* 0x0C */
	rt_uint32_t reserved0[2];
	rt_uint32_t ddma_para;/* 0x18 */
	rt_uint32_t reserved1;
};

struct tina_dma
{
	rt_uint32_t irq_en;/* 0x000 */
	rt_uint32_t irq_pend;/* 0x004  */
	rt_uint32_t auto_gate;/* 0x008 */
	rt_uint32_t reserved0[61];
	struct tina_dma_cfg ndma[8];/* 0x100 */
	rt_uint32_t reserved1[64];
	struct tina_dma_cfg ddma[8];/* 0x300 */
};

typedef struct tina_dma *tina_dma_t;
#define DMA ((tina_dma_t) DMA_BASE_ADDR)

/**
 * @brief Allwinner DMA
 */
enum tina_dmax
{
	NDMA_CH0 = 0,
	NDMA_CH1,
	NDMA_CH2,
	NDMA_CH3,
	DDMA_CH0,
	DDMA_CH1,
	DDMA_CH2,
	DDMA_CH3
};

/**
 * @brief DMA interrupt type
 */
enum dma_irq_type
{
	DMA_IRQ_HT = 1 << 0,//Half transfer
	DMA_IRQ_FT = 1 << 1,//Full transfer
};

/**
 * @brief DMA DRQ type
 */
enum dma_drq_type
{
	NDMA_DST_DRQ_OWA_TX = 0X01,
	NDMA_DST_DRQ_SPI0_TX = 0X04,
	NDMA_DST_DRQ_SPI1_TX = 0X05,
	NDMA_DST_DRQ_UART0_TX = 0X08,
	NDMA_DST_DRQ_UART1_TX = 0X09,
	NDMA_DST_DRQ_UART2_TX = 0X0A,
	NDMA_DST_DRQ_AUDIO_CODEC_DAC = 0X0C,
	NDMA_DST_DRQ_DAUDIO = 0X0E,
	NDMA_DST_DRQ_SRAM = 0X10,
	NDMA_DST_DRQ_SDRAM = 0X11,
	NDMA_DST_DRQ_USB = 0X14,
	NDMA_DST_DRQ_USB_EP1 = 0X15,
	NDMA_DST_DRQ_USB_EP2 = 0X16,
	NDMA_DST_DRQ_USB_EP3 = 0X17,

	NDMA_SRC_DRQ_IR_RX = 0X00,
	NDMA_SRC_DRQ_SPI0_RX = 0X04,
	NDMA_SRC_DRQ_SPI1_RX = 0X05,
	NDMA_SRC_DRQ_UART0_RX = 0X08,
	NDMA_SRC_DRQ_UART1_RX = 0X09,
	NDMA_SRC_DRQ_UART2_RX = 0X0A,
	NDMA_SRC_DRQ_AUDIO_CODEC = 0X0C,
	NDMA_SRC_DRQ_TP_ADC = 0X0D,
	NDMA_SRC_DRQ_DAUDIO = 0X0E,
	NDMA_SRC_DRQ_SRAM = 0X10,
	NDMA_SRC_DRQ_SDRAM = 0X11,
	NDMA_SRC_DRQ_USB = 0X14,
	NDMA_SRC_DRQ_USB_EP1 = 0X15,
	NDMA_SRC_DRQ_USB_EP2 = 0X16,
	NDMA_SRC_DRQ_USB_EP3 = 0X17,

	DDMA_DST_DRQ_SRAM = 0X00,
	DDMA_DST_DRQ_SDRAM = 0X01,
	DDMA_DST_DRQ_LCDC = 0X02,
	DDMA_DST_DRQ_USB = 0X04,
	DDMA_DST_AHB_MEM = 0X09,

	DDMA_SRC_DRQ_SRAM = 0X00,
	DDMA_SRC_DRQ_SDRAM = 0X01,
	DDMA_SRC_DRQ_USB = 0X04,
	DDMA_SRC_AHB_MEM = 0X09,
};

/**
 * @brief DMA data width
 */
enum dma_data_width
{
	DMA_DATA_WIDTH_8BIT = 0,
	DMA_DATA_WIDTH_16BIT,
	DMA_DATA_WIDTH_32BIT
};

/**
 * @brief DMA parameters structure
 */
struct dma_param
{
	rt_uint32_t *src_addr;//Source data address,the address should be aligned
	enum dma_data_width src_data_width;//Source data width
	rt_bool_t src_addr_increase;//Whether the source data address incremented or not
	enum dma_drq_type src_drq_type;//Source data DRQ type

	rt_uint32_t *dst_addr;//Destination data address,the address should be aligned
	enum dma_data_width dst_data_width;//Destination data width
	rt_bool_t dst_addr_increase;//Whether the destination_address incremented or not
	enum dma_drq_type dst_drq_type;//Destination data DRQ type

	rt_uint32_t byte_num;//NDMA cannot exceed 128K,DDMA cannot exceed 16K
	rt_bool_t continuous_enable;//Whether continout mode is enabled or not
	
};

/**
 * @brief Check whether the specified interrupt is set or not
 * 
 * @param dmax xDMA_CHy:where x can be D or N to select the DMA and y can be 0 to 3 to select the channel
 * @param type DMA interrupt type,the parameter can be combination of the following values
 *    @arg DMA_IRQ_HT Half transfer complete flag
 *    @arg DMA_IRQ_FT Transfer complete flag
 * @return The status of the specified interrupt 
 */
static inline rt_bool_t drv_dma_get_irq_status(enum tina_dmax dmax, enum dma_irq_type type)
{
	RT_ASSERT(dmax < 8 && (type & ~(DMA_IRQ_HT | DMA_IRQ_FT)) == 0 && type != 0);

	if(DMA->irq_pend & (type << (((dmax & 0x4) ? 16 : 0) + 2 * (dmax & 0x3))))
		return RT_TRUE;
	else
		return RT_FALSE;
}

/**
 * @brief Clear the specified interrupt status
 * 
 * @param dmax xDMA_CHy:where x can be D or N to select the DMA and y can be 0 to 3 to select the channel
 * @param type DMA interrupt type,the parameter can be combination of the following values
 *    @arg DMA_IRQ_HT Half transfer complete flag
 *    @arg DMA_IRQ_FT Transfer complete flag 
 */
static inline void drv_dma_clear_irq(enum tina_dmax dmax, enum dma_irq_type type)
{
	RT_ASSERT(dmax < 8 && (type & ~(DMA_IRQ_HT | DMA_IRQ_FT)) == 0 && type != 0);

	DMA->irq_pend |= type << (((dmax & 0x4) ? 16 : 0) + 2 * (dmax & 0x3));
}

void drv_dma_irq_enable(enum tina_dmax dmax, enum dma_irq_type type, rt_bool_t enable);
void drv_dma_init(enum tina_dmax dmax, struct dma_param *param);
void drv_dma_enable(enum tina_dmax dmax, rt_bool_t enable);
rt_bool_t drv_dma_check_busy(enum tina_dmax dmax);
void drv_dma_reset(void);
void rt_hw_dma_init(void);
void drv_dma_register_irq_callback(enum tina_dmax dmax, void (*callback)(void *param), void *param);

#endif