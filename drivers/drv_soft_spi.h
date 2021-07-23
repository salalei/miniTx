#ifndef __DRV_SOFT_SPI_H__
#define __DRV_SOFT_SPI_H__

#include "rtdef.h"

struct soft_spi_handle;

/** 
  * @brief SPI通信模式定义 
  */   
enum soft_spi_mode
{
	SOFT_SPI_MODE0 = 0,		//模式0  低电平空闲,一边沿采样
	SOFT_SPI_MODE1,			//模式1  低电平空闲,第二边沿采样
	SOFT_SPI_MODE2,			//模式2  高电平空闲,第一边沿采样
	SOFT_SPI_MODE3,			//模式3  高电平空闲,第二边沿采样
};

/** 
  * @brief 软件SPI句柄  
  */ 
struct soft_spi_handle
{
	void (*set_scl)(rt_int32_t value);
	void (*set_mosi)(rt_int32_t value);
	rt_int32_t (*get_miso)(void);

	rt_uint32_t delay_count;

	rt_uint8_t (*rw_byte)(struct soft_spi_handle*, rt_uint8_t);
};

void drv_soft_spi_master_transmit(struct soft_spi_handle *handle, rt_uint8_t *buf, rt_uint16_t len);
void drv_soft_spi_master_receive(struct soft_spi_handle *handle, rt_uint8_t *buf, rt_uint16_t len);
void drv_soft_spi_init(	struct soft_spi_handle *handle,
						enum soft_spi_mode mode,
						void (*set_scl)(rt_int32_t value),
						void (*set_mosi)(rt_int32_t value),
						rt_int32_t (*get_miso)(void),
						rt_uint32_t delay_count);

#endif