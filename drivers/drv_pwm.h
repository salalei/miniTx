#ifndef __DRV_PWM_H__
#define __DRV_PWM_H__

#include "rtdef.h"
#include "drv_gpio.h"

#define PWM_BASE_ADDR              0x01C21000

struct PWM
{
	volatile rt_uint32_t ctrl;/* 0x00 */
	volatile rt_uint32_t ch0_period;/* 0x04 */
	volatile rt_uint32_t ch1_period;/* 0x08 */
};

typedef struct PWM *PWM_t;
#define PWM ((PWM_t) PWM_BASE_ADDR)

/**
 * @brief PWM channel
 * 
 */
enum pwm_channel
{
	PWM_CHANNEL_0 = 0,
	PWM_CHANNEL_1
};

/**
 * @brief PWM channel pre-scale
 */
enum pwm_clk_div
{
	PWM_CLK_DIV_120 = 0,
	PWM_CLK_DIV_180 = 1,
	PWM_CLK_DIV_240 = 2,
	PWM_CLK_DIV_360 = 3,
	PWM_CLK_DIV_480 = 4,
	PWM_CLK_DIV_12K = 8,
	PWM_CLK_DIV_24K = 9,
	PWM_CLK_DIV_36K = 10,
	PWM_CLK_DIV_48K = 11,
	PWM_CLK_DIV_72K = 12,
	PWM_CLK_DIV_1 = 15,
};

/**
 * @brief PWM parmeters structure
 * 
 */
struct pwm_param
{
	rt_uint8_t active_state;//0 is low level active
	enum pwm_clk_div div;//PWM channel pre-scale,the PWM frequency = 24000000 / div
	enum gpio_port pwm_port;
	enum gpio_pin pwm_pin;
	rt_uint8_t pwm_fun;
};

void drv_pwm_init(enum pwm_channel channel, struct pwm_param *param);
void drv_pwm_enable(enum pwm_channel channel, rt_bool_t enable);
rt_err_t drv_pwm_set_period_duty(enum pwm_channel channel, rt_uint16_t period, rt_uint16_t duty);
void drv_pwm_get_period_duty(enum pwm_channel channel, rt_uint16_t *period, rt_uint16_t *duty);

#endif