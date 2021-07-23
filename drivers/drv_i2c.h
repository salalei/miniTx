#ifndef __DRV_I2C_H__
#define __DRV_I2C_H__

#include "rtdef.h"
#include "drv_gpio.h"

#define TWI0_BASE_ADDR              0x01C27000
#define TWI1_BASE_ADDR              0x01C27400
#define TWI2_BASE_ADDR              0x01C27800

struct tina_twi
{
	rt_uint32_t slave_address;
	rt_uint32_t xtnd_slave_addr;
	rt_uint32_t data;
	rt_uint32_t control;
	rt_uint32_t status;
	rt_uint32_t baudrate;
	rt_uint32_t soft_reset;
};

typedef struct tina_twi *tina_twi_t;
#define TWI0 ((tina_twi_t) TWI0_BASE_ADDR)
#define TWI1 ((tina_twi_t) TWI1_BASE_ADDR)
#define TWI2 ((tina_twi_t) TWI2_BASE_ADDR)

/**
 * @brief TWI status
 */
enum twi_status
{
	STATUS_START = 0x08,
	STATUS_REPEATED_START = 0x10,
	STATUS_ADDR_W_ACK = 0x18,
	STATUS_DATA_W_ACK = 0x28,
	STATUS_ADDR_R_ACK = 0x40,
	STATUS_ADDR_R_NAK = 0x48,
	STATUS_DATA_R_ACK = 0x50,
	STATUS_DATA_R_NAK = 0x58,
	STATUS_IDLE	= 0xF8,
};

/**
 * @brief TWI parameters structure
 */
struct twi_param
{
	rt_uint32_t clk;//I2C clock frequency
	rt_uint8_t own_addr;//The device own address
	enum gpio_port i2c_sda_port;
	enum gpio_pin i2c_sda_pin;
	rt_uint8_t i2c_sda_fun;
	enum gpio_port i2c_scl_port;
	enum gpio_pin i2c_scl_pin;
	rt_uint8_t i2c_scl_fun;
};

/**
 * @brief Disable TWI
 * 
 * @param twix Where x can be 0 to 2
 */
static inline void drv_i2c_disable(tina_twi_t twix)
{
    twix->control = 0;
}

/**
 * @brief Generate start condition
 * 
 * @param twix Where x can be 0 to 2
 */
static inline void drv_i2c_start(tina_twi_t twix)
{
    twix->control = 1 << 6 | 1 << 5;
}

/**
 * @brief Send a byte through the i2c bus
 * 
 * @param twix Where x can be 0 to 2
 * @param data Byte to be transmitted
 */
static inline void drv_i2c_send(tina_twi_t twix, rt_uint8_t data)
{
    twix->data = data;
    twix->control = 1 << 6;
}

/**
 * @brief Receive a byte by the i2c bus
 * 
 * @param twix Where x can be 0 to 2
 * @param data Pointer to the byte to be received
 */
static inline void drv_i2c_recv(tina_twi_t twix, rt_uint8_t *data)
{
    *data = twix->data;
}

/**
 * @brief Generate stop condition
 * 
 * @param twix Where x can be 0 to 2
 */
static inline void drv_i2c_stop(tina_twi_t twix)
{
    twix->control = 1 << 6 | 1 << 4;
}

/**
 * @brief Enable or disable the specified i2c acknowledge features
 * 
 * @param twix Where x can be 0 to 2
 * @param enable
 *    @arg RT_TRUE Enable acknowledge
 *    @arg RT_FALSE Disable acknowledge
 */
static inline void drv_i2c_enable_ack(tina_twi_t twix, rt_bool_t enable)
{
    if(enable)
        twix->control = 1 << 6 | 1 << 2;
    else
        twix->control = 1 << 6;
}

/**
 * @brief Enable or disable the specified i2c interrupt
 * 
 * @param twix Where x can be 0 to 2
 * @param enable
 *    @arg RT_TRUE Enable interrupt
 *    @arg RT_FALSE Disable interrupt
 */
static inline void drv_i2c_enable_irq(tina_twi_t twix, rt_bool_t enable)
{
    if(enable)
        twix->control |= 1 << 7;
    else
        twix->control &= ~(1 << 7);
}

rt_err_t drv_i2c_wait(tina_twi_t twix, enum twi_status except_status, volatile rt_uint32_t timeout);
rt_err_t drv_i2c_init(tina_twi_t twix, struct twi_param *param);

#endif