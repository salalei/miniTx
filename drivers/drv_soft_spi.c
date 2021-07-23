#include <rtthread.h>
#include "drv_soft_spi.h"

/**
 * @brief SPI通信延时
 * 
 * @param handle 指向模拟SPI句柄的指针
 */
static void spi_delay(struct soft_spi_handle *handle)
{
	volatile rt_uint32_t i = handle->delay_count;

	while(i--);
}

/**
  * @brief  模式0读写一个字节
  * @param  handle: 指向模拟SPI句柄的指针
  * @param  data: 需要发送的数据
  * @retval 读取到的数据
  */
static rt_uint8_t rw_byte_0(struct soft_spi_handle *handle, rt_uint8_t data)
{
	rt_uint8_t rx_data = 0;
	
	handle->set_scl(0);
	for(int i = 0; i < 8; i++)
	{
		handle->set_mosi(data & 0x80 ? 1 : 0);
		data <<= 1;
		spi_delay(handle);
		handle->set_scl(1);
		spi_delay(handle);
		rx_data <<= 1;
		if(handle->get_miso && handle->get_miso())
			rx_data |= 0x01;
		handle->set_scl(0);
	}
	return rx_data;
}

/**
  * @brief  模式1读写一个字节
  * @param  handle: 指向模拟SPI句柄的指针
  * @param  data: 需要发送的数据
  * @retval 读取到的数据
  */
static rt_uint8_t rw_byte_1(struct soft_spi_handle *handle, rt_uint8_t data)
{
	rt_uint8_t rx_data = 0;
	
	for(int i = 0; i < 8; i++)
	{
		handle->set_scl(1);
		handle->set_mosi(data & 0x80 ? 1 : 0);
		data <<= 1;
		spi_delay(handle);
		handle->set_scl(0);
		spi_delay(handle);
		rx_data <<= 1;
		if(handle->get_miso && handle->get_miso())
			rx_data |= 0x01;
	}
	return rx_data;
}

/**
  * @brief  模式2读写一个字节
  * @param  handle: 指向模拟SPI句柄的指针
  * @param  data: 需要发送的数据
  * @retval 读取到的数据
  */
static rt_uint8_t rw_byte_2(struct soft_spi_handle *handle, rt_uint8_t data)
{
	rt_uint8_t rx_data = 0;
	
	handle->set_scl(1);
	for(int i = 0; i < 8; i++)
	{
		handle->set_mosi(data & 0x80 ? 1 : 0);
		data <<= 1;
		spi_delay(handle);
		handle->set_scl(0);
		spi_delay(handle);
		rx_data <<= 1;
		if(handle->get_miso && handle->get_miso())
			rx_data |= 0x01;
		handle->set_scl(1);
	}
	return rx_data;
}

/**
  * @brief  模式3读写一个字节
  * @param  handle: 指向模拟SPI句柄的指针
  * @param  data: 需要发送的数据
  * @retval 读取到的数据
  */
static rt_uint8_t rw_byte_3(struct soft_spi_handle *handle, rt_uint8_t data)
{
	rt_uint8_t rx_data = 0;
	
	for(int i = 0; i < 8; i++)
	{
		handle->set_scl(0);
		handle->set_mosi(data & 0x80 ? 1 : 0);
		data <<= 1;
		spi_delay(handle);
		handle->set_scl(1);
		spi_delay(handle);
		rx_data <<= 1;
		if(handle->get_miso && handle->get_miso())
			rx_data |= 0x01;
	}
	return rx_data;
}

/**
  * @brief  主机模式传输多个字节
  * @param  handle: 指向模拟SPI句柄的指针
  * @param  buf: 指向用户需要传输的数据的指针
  * @param  len: 需要传输的数据长度
  * @retval NULL
  */
void drv_soft_spi_master_transmit(struct soft_spi_handle *handle, rt_uint8_t *buf, rt_uint16_t len)
{	
	while(len--)
	{
		handle->rw_byte(handle, *buf++);
	}
}
/**
  * @brief  主机模式接受多个字节
  * @param  handle: 指向模拟SPI句柄的指针
  * @param  buf: 指向用户接受缓存区的指针
  * @param  len: 需要读取的数据长度
  * @retval NULL
  */
void drv_soft_spi_master_receive(struct soft_spi_handle *handle, rt_uint8_t *buf, rt_uint16_t len)
{
	while(len--)
	{
		*buf++ = handle->rw_byte(handle, 0xff);
	}
}

/**
 * @brief 初始化指定模拟spi的句柄
 * 
 * @param handle: 指向模拟SPI句柄的指针
 * @param mode: SPI的传输模式
 *    @arg SOFT_SPI_MODE0 低电平空闲,第一边沿采样
 * 	  @arg SOFT_SPI_MODE1 低电平空闲,第二边沿采样
 *    @arg SOFT_SPI_MODE2 高电平空闲,第一边沿采样
 *    @arg SOFT_SPI_MODE3 高电平空闲,第二边沿采样
 * @param set_scl: 设置SCL引脚电平的函数
 * @param set_mosi: 设置MOSI引脚电平的函数
 * @param get_miso: 获取MISO引脚电平的函数,如果不需要接受数据，可以填RT_NULL
 * @param delay_count: 每个SPI时钟的延时次数,决定SPI通信的频率,具体频率需要自己仿真测试
 */
void drv_soft_spi_init(	struct soft_spi_handle *handle,
						enum soft_spi_mode mode,
						void (*set_scl)(rt_int32_t value),
						void (*set_mosi)(rt_int32_t value),
						rt_int32_t (*get_miso)(void),
						rt_uint32_t delay_count)
{
	RT_ASSERT(handle != RT_NULL && set_scl != RT_NULL && set_mosi != RT_NULL && mode < 4);

	handle->set_scl = set_scl;
	handle->set_mosi = set_mosi;
	handle->get_miso = get_miso;
	handle->delay_count = delay_count;
	if(mode == SOFT_SPI_MODE0)
		handle->rw_byte = rw_byte_0;
	else if(mode == SOFT_SPI_MODE1)
		handle->rw_byte = rw_byte_1;
	else if (mode == SOFT_SPI_MODE2)
		handle->rw_byte = rw_byte_2;
	else
		handle->rw_byte = rw_byte_3;
}