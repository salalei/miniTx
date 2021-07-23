#include <rtthread.h>
#include <rtdevice.h>
#include "drv_pwm.h"

#define DBG_TAG  "PWM"
#define DBG_LVL  DBG_ERROR
#include <rtdbg.h>

/**
 * @brief Initializes the PWM
 * 
 * @param channel Specify the PWM channel
 * @param param Pointer to PWM parameters structure
 */
void drv_pwm_init(enum pwm_channel channel, struct pwm_param *param)
{
	RT_ASSERT(channel < 2 && param);

	gpio_set_func(param->pwm_port, param->pwm_pin, param->pwm_fun);

	PWM->ctrl |= 1 << (6 + channel * 15) | (param->div & 0xf) << (channel * 15);
	if(param->active_state)
		PWM->ctrl |= 1 << (5 + channel * 15);
	else
		PWM->ctrl &= ~(1 << (5 + channel * 15));
}

/**
 * @brief Enable or disable PWM
 * 
 * @param channel Specify the PWM channel
 * @param enable
 *    @arg RT_TRUE Enable PWM
 *    @arg RT_FALSE Disable PWM
 */
void drv_pwm_enable(enum pwm_channel channel, rt_bool_t enable)
{
	RT_ASSERT(channel < 2);

	if(enable)
		PWM->ctrl |= 1 << (4 + channel * 15);
	else
		PWM->ctrl &= ~(1 << (4 + channel * 15));
}

/**
 * @brief Set the period and duty cycle of the PWM
 * 
 * @param channel Specify the PWM channel
 * @param period Number of the entire cycle of the PWM clock
 * @param duty Number of the active cycle of the PWM clock
 * @return The error code 
 */
rt_err_t drv_pwm_set_period_duty(enum pwm_channel channel, rt_uint16_t period, rt_uint16_t duty)
{
	RT_ASSERT(channel < 2);

	if(PWM->ctrl & 1 << (28 + channel))
	{
		LOG_E("PWM period register is busy");
		return RT_EBUSY;
	}

	if(channel == PWM_CHANNEL_0)
		PWM->ch0_period = period << 16 | duty;
	else
		PWM->ch1_period = period << 16 | duty;
	
	return RT_EOK;
}

/**
 * @brief Get the current period and duty cycle of the PWM
 * 
 * @param channel Specify the PWM channel
 * @param period Current number of the entire cycle of the PWM clock
 * @param duty Current number of the active cycle of the PWM clock
 */
void drv_pwm_get_period_duty(enum pwm_channel channel, rt_uint16_t *period, rt_uint16_t *duty)
{
	rt_uint32_t reg_data;

	RT_ASSERT(channel < 2 && period && duty);

	if(channel == PWM_CHANNEL_0)
		reg_data = PWM->ch0_period;
	else
		reg_data = PWM->ch1_period;
	
	*period = (rt_uint16_t)(reg_data >> 16);
	*duty = (rt_uint16_t)reg_data;
}


#ifdef RT_USING_PWM

//125 / 3 ns per cycle
#define PWM_NS_TO_CYCLE(ns)         ((ns) * 3 / 125)
#define PWM_CYCLE_TO_NS(cycle)      ((cycle) * 125 / 3)

enum
{
    PWM1_INDEX = 1,
    PWM2_INDEX,
};

static rt_err_t pwm_enable(struct rt_pwm_configuration *configuration, rt_bool_t enable)
{
    if(configuration->channel == PWM1_INDEX)
    {
#ifdef TINA_USING_PWM0
		drv_pwm_enable(PWM_CHANNEL_0, enable);
#else
        LOG_E("PWM channel 0 not used");
        return RT_EINVAL;
#endif
    }
    else if(configuration->channel == PWM2_INDEX)
    {
#ifdef TINA_USING_PWM1
		drv_pwm_enable(PWM_CHANNEL_1, enable);
#else
        LOG_E("PWM channel 1 not used");
        return RT_EINVAL;
#endif
    }
    else
        return RT_EINVAL;

    return RT_EOK;
}

static rt_err_t pwm_get(struct rt_pwm_configuration *configuration)
{
    rt_uint16_t period, duty;

    if(configuration->channel == PWM1_INDEX)
    {
#ifdef TINA_USING_PWM0
        drv_pwm_get_period_duty(PWM_CHANNEL_0, &period, &duty);
#else
        LOG_E("PWM channel 0 not used");
        return RT_EINVAL;
#endif
    }
    else if(configuration->channel == PWM2_INDEX)
    {
#ifdef TINA_USING_PWM1
        drv_pwm_get_period_duty(PWM_CHANNEL_1, &period, &duty);
#else
        LOG_E("PWM channel 1 not used");
        return RT_EINVAL;
#endif
    }
    else
        return RT_EINVAL;

    configuration->period = period;
    configuration->pulse = duty;

    return RT_EOK;
}

static rt_err_t pwm_set(struct rt_pwm_configuration *configuration)
{
    if(configuration->channel == PWM1_INDEX)
    {
#ifdef TINA_USING_PWM0
		drv_pwm_set_period_duty(PWM_CHANNEL_0, PWM_NS_TO_CYCLE(configuration->period), PWM_NS_TO_CYCLE(configuration->pulse));
#else
        LOG_E("PWM channel 0 not used");
        return RT_EINVAL;
#endif
    }
    else if(configuration->channel == PWM2_INDEX)
    {
#ifdef TINA_USING_PWM1
		drv_pwm_set_period_duty(PWM_CHANNEL_1, PWM_NS_TO_CYCLE(configuration->period), PWM_NS_TO_CYCLE(configuration->pulse));
#else
        LOG_E("PWM channel 1 not used");
        return RT_EINVAL;
#endif
    }
    
    return RT_EOK;
}

rt_err_t pwm_control(struct rt_device_pwm *device, int cmd, void *arg)
{
    struct rt_pwm_configuration *configuration = (struct rt_pwm_configuration *)arg;

    switch (cmd)
    {
    case PWM_CMD_ENABLE:
        return pwm_enable(configuration, RT_TRUE);
    case PWM_CMD_DISABLE:
        return pwm_enable(configuration, RT_FALSE);
    case PWM_CMD_SET:
        return pwm_set(configuration);
    case PWM_CMD_GET:
        return pwm_get(configuration);
    default:
        return RT_EINVAL;
    }
}

static const struct rt_pwm_ops ops = 
{
    .control = pwm_control
};

static int rt_hw_pwm_init(void)
{
#ifdef TINA_USING_PWM0
	{
		struct pwm_param param =
		{
			.active_state = 1,
			.div = PWM_CLK_DIV_1,
			.pwm_port = GPIO_PORT_E,
			.pwm_pin = GPIO_PIN_12,
			.pwm_fun = IO_FUN_3
		};
		
		drv_pwm_enable(PWM_CHANNEL_0, RT_FALSE);
		drv_pwm_init(PWM_CHANNEL_0, &param);
	}
#endif

#ifdef TINA_USING_PWM1
	{
		struct pwm_param param =
		{
			.active_state = 1,
			.div = PWM_CLK_DIV_1,
			.pwm_port = GPIO_PORT_E,
			.pwm_pin = GPIO_PIN_6,
			.pwm_fun = IO_FUN_2
		};
		
		drv_pwm_enable(PWM_CHANNEL_1, RT_FALSE);
		drv_pwm_init(PWM_CHANNEL_1, &param);
		drv_pwm_enable(PWM_CHANNEL_1, RT_TRUE);
		drv_pwm_set_period_duty(PWM_CHANNEL_1, PWM_NS_TO_CYCLE(1000000), PWM_NS_TO_CYCLE(500000));
	}
#endif

#if defined TINA_USING_PWM0 || defined TINA_USING_PWM1
	static struct rt_device_pwm pwm_dev;
	return rt_device_pwm_register(&pwm_dev, "pwm", &ops, RT_NULL);
#else
	return 0;
#endif
}

INIT_DEVICE_EXPORT(rt_hw_pwm_init);

#endif