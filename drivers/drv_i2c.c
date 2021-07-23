#include <rtthread.h>
#include "rthw.h"
#include "interrupt.h"
#include "drv_clock.h"
#include "drv_i2c.h"

#define DBG_TAG  "I2C"
#define DBG_LVL  DBG_ERROR
#include <rtdbg.h>

/**
 * @brief Wait for specified TWI statrus
 * 
 * @param twix Where x can be 0 to 2
 * @param except_status Specify TWI status
 * @param timeout Specify the timeout
 * @return The error code 
 */
rt_err_t drv_i2c_wait(tina_twi_t twix, enum twi_status except_status, volatile rt_uint32_t timeout)
{
    do
    {
        if(except_status == STATUS_IDLE)
        {
            if(twix->status == except_status)
                return RT_EOK;
        }
        else if(twix->control & 1 << 3)
        {
            if(twix->status == except_status)
                return RT_EOK;
            else
            {
                LOG_E("Except status:%#X, actual status:%#X", except_status, twix->status);
                return RT_ERROR;
            }
        }
    }
    while(timeout--);

	if(except_status == STATUS_IDLE)
		LOG_E("Except status:%#X, actual status:%#X", except_status, twix->status);

    return RT_ETIMEOUT;
}

/**
 * @brief Reset TWI
 * 
 * @param twix Where x can be 0 to 2
 */
static void i2c_reset(tina_twi_t twix)
{
    if(twix == TWI0)
	{
		bus_software_reset_disable(TWI0_GATING);
		bus_software_reset_enable(TWI0_GATING);
	}
    else if(twix == TWI1)
    {
		bus_software_reset_disable(TWI1_GATING);
		bus_software_reset_enable(TWI1_GATING);
	}
    else if(twix == TWI2)
    {
		bus_software_reset_disable(TWI2_GATING);
		bus_software_reset_enable(TWI2_GATING);
	}

    twix->soft_reset = 1 << 0;
    rt_thread_mdelay(2);
    twix->soft_reset = 0;
}

/**
 * @brief Set the TWI clock frequency
 * 
 * @param twix 
 * @param clk 
 * @return rt_uint32_t 
 */
static rt_uint32_t i2c_set_rate(tina_twi_t twix, rt_uint32_t clk)
{
    rt_uint32_t max = 0, temp, baud = 0x44;

	for (int n = 0; n < 8; n++)
    {
		for (int m = 0; m < 16; m++)
        { 
			temp = apb_get_clk() / (10 * (m + 1) * (1 << n));
			if ((temp <= clk) && (temp > max)) {
				max = temp;
				baud = (m << 3) | n;
			}
		}
	}
    twix->baudrate = baud;
    rt_thread_mdelay(1);
    LOG_D("Except rate:%d, real rate:%d", clk, max);
    
    return max;
}

/**
 * @brief Initialize the TWI
 * 
 * @param twix Where x can be 0 to 2
 * @param param Pointer to TWI parameters structure
 * @return The error code
 */
rt_err_t drv_i2c_init(tina_twi_t twix, struct twi_param *param)
{
	//Reset TWI
    i2c_reset(twix);
	//Enable the bus gate
    if(twix == TWI0)
        bus_gate_clk_enable(TWI0_GATING);
    else if(twix == TWI1)
        bus_gate_clk_enable(TWI1_GATING);
    else if(twix == TWI2)
        bus_gate_clk_enable(TWI2_GATING);
    
    gpio_set_func(param->i2c_sda_port, param->i2c_sda_pin, param->i2c_sda_fun);
    gpio_set_func(param->i2c_scl_port, param->i2c_scl_pin, param->i2c_scl_fun);

    i2c_set_rate(twix, param->clk);
    twix->slave_address = param->own_addr;
    twix->xtnd_slave_addr = 0;
	//Generate i2c stop condition
    drv_i2c_stop(twix);
    if(drv_i2c_wait(twix, STATUS_IDLE, 10000) != RT_EOK)
    {
        LOG_E("i2c init failed");
        return RT_ERROR;
    }

    return RT_EOK;
}


#ifdef RT_USING_I2C

#include <rtdevice.h>

#define I2C_WAIT_STATUS_TIMEOUT 10000

#define I2C_GO_ON				0
#define I2C_BACK				1

#define I2C_MODE_BLOCK			0//BLOCK mode
#define I2C_MODE_INT			1//INT mode

enum i2c_event
{
	TINA_I2C_START_EVENT = 0,
	TINA_I2C_SEND_ADDRESS_EVENT,
	TINA_I2C_WRITE_EVENT,
	TINA_I2C_READ_EVENT,
};

struct i2c_handle
{
    tina_twi_t twix;
	rt_uint32_t except_status;
	enum i2c_event event;
	rt_sem_t sem;
	struct rt_i2c_msg *msgs;
	rt_uint32_t msgs_num;
	rt_uint32_t msgs_index;
	rt_uint32_t rw_index;
	rt_err_t res;

	rt_base_t mode;
};

static int i2c_irq_start(struct i2c_handle *handle)
{
	struct rt_i2c_msg *msg = &handle->msgs[handle->msgs_index];

	if(handle->event == TINA_I2C_START_EVENT)
	{
		if(msg->flags & RT_I2C_NO_START)
		{
			if(msg->flags & RT_I2C_RD)
				handle->event = TINA_I2C_READ_EVENT;
			else
				handle->event = TINA_I2C_WRITE_EVENT;
			handle->rw_index = 0;
		}
		else
		{
			//If its not the first message, then we send the STATUS_REPEATED_START
			if(handle->msgs_index)
				handle->except_status = STATUS_REPEATED_START;
			else
				handle->except_status = STATUS_START;

			drv_i2c_start(handle->twix);
			handle->event = TINA_I2C_SEND_ADDRESS_EVENT;
			drv_i2c_enable_irq(handle->twix, RT_TRUE);

			return I2C_BACK;
		}
	}

	return I2C_GO_ON;
}

static int i2c_irq_send_address(struct i2c_handle *handle)
{
	struct rt_i2c_msg *msg = &handle->msgs[handle->msgs_index];

	if(handle->event == TINA_I2C_SEND_ADDRESS_EVENT)
	{
		rt_uint8_t addr = msg->addr << 1;
		if(msg->flags & RT_I2C_RD)
		{
			addr |= 1;
			handle->except_status = STATUS_ADDR_R_ACK;
			handle->event = TINA_I2C_READ_EVENT;
		}
		else
		{
			handle->except_status = STATUS_ADDR_W_ACK;
			handle->event = TINA_I2C_WRITE_EVENT;
		}
		handle->rw_index = 0;

		drv_i2c_send(handle->twix, addr);
		drv_i2c_enable_irq(handle->twix, RT_TRUE);

		return I2C_BACK;
	}

	return I2C_GO_ON;
}

static int i2c_irq_write(struct i2c_handle *handle)
{
	struct rt_i2c_msg *msg = &handle->msgs[handle->msgs_index];

	if(handle->event == TINA_I2C_WRITE_EVENT)
	{
		if(handle->rw_index < msg->len)
		{
			handle->except_status = STATUS_DATA_W_ACK;
			drv_i2c_send(handle->twix, msg->buf[handle->rw_index++]);
			drv_i2c_enable_irq(handle->twix, RT_TRUE);
			
			return I2C_BACK;
		}
		else
			handle->event = TINA_I2C_START_EVENT;
	}
	
	return I2C_GO_ON;
}

static int i2c_irq_read(struct i2c_handle *handle)
{
	struct rt_i2c_msg *msg = &handle->msgs[handle->msgs_index];

	if(handle->event == TINA_I2C_READ_EVENT)
	{
		if(handle->rw_index > 0 && handle->rw_index <= msg->len)
			drv_i2c_recv(handle->twix, &msg->buf[handle->rw_index - 1]);

		if(handle->rw_index < msg->len)
		{
			if(handle->rw_index == msg->len - 1)
			{
				//If its the last transmission, then we send the NACK
				drv_i2c_enable_ack(handle->twix, RT_FALSE);
				handle->except_status =  STATUS_DATA_R_NAK;
			}
			else
			{
				drv_i2c_enable_ack(handle->twix, RT_TRUE);
				handle->except_status =  STATUS_DATA_R_ACK;
			}
			handle->rw_index++;
			drv_i2c_enable_irq(handle->twix, RT_TRUE);

			return I2C_BACK;
		}
		else
			handle->event = TINA_I2C_START_EVENT;
	}

	return I2C_GO_ON;
}

static void i2c_irq_handler(int vector, void *param)
{
	struct i2c_handle *handle = (struct i2c_handle *)param;
	
	if(handle->twix->status != handle->except_status)
	{
		LOG_E("Except status:%#X, actual status:%#X, event=%d, msgs index=%d, msgs flag=%#X", \
			handle->except_status, handle->twix->status, handle->event, handle->msgs_index, handle->msgs[handle->msgs_index].flags);
		goto out;
	}

	while(handle->msgs_index < handle->msgs_num)
	{
		if(i2c_irq_start(handle) != I2C_GO_ON)
			return;
		
		if(i2c_irq_send_address(handle) != I2C_GO_ON)
			return;

		if(i2c_irq_write(handle) != I2C_GO_ON)
			return;

		if(i2c_irq_read(handle) != I2C_GO_ON)
			return;
		
		handle->msgs_index++;
	}
	if (!(handle->msgs[handle->msgs_index].flags & RT_I2C_NO_STOP))
		drv_i2c_stop(handle->twix);
	handle->res = RT_EOK;

out:

	if(handle->res != RT_EOK)
		drv_i2c_disable(handle->twix);

	rt_sem_release(handle->sem);
}

static rt_size_t master_int_xfer(struct i2c_handle *handle, struct rt_i2c_msg msgs[], rt_uint32_t num)
{
	struct rt_i2c_msg *msg = &msgs[0];
	rt_err_t ret = RT_EOK;

    if(num == 0)
        return 0;
    
	handle->msgs = &msgs[0];
	handle->msgs_num = num;
	handle->rw_index = 0;
	handle->msgs_index = 0;
	handle->res = RT_ERROR;
	handle->event = TINA_I2C_START_EVENT;

	while(handle->msgs_index < handle->msgs_num)
	{
		if(i2c_irq_start(handle) != I2C_GO_ON)
		{
			LOG_D("First start");
			goto wait;
		}

		msg = &handle->msgs[handle->msgs_index];

		if(msg->len == 0)
		{
			handle->event = TINA_I2C_START_EVENT;
			LOG_D("Invalid length");
		}
		else
		{
			if(i2c_irq_write(handle) != I2C_GO_ON)
			{
				LOG_D("First write");
				goto wait;
			}

			if(i2c_irq_read(handle) != I2C_GO_ON)
			{
				LOG_D("First read");
				goto wait;
			}
		}

		handle->msgs_index++;
	}

	LOG_D("Invalid message");
	return num;

wait:

	ret = rt_sem_take(handle->sem, 100);
	
	drv_i2c_enable_irq(handle->twix, RT_FALSE);

    if (!(msg->flags & RT_I2C_NO_STOP) || handle->res != RT_EOK || ret != RT_EOK)
    {
        drv_i2c_stop(handle->twix);
        if(drv_i2c_wait(handle->twix, STATUS_IDLE, I2C_WAIT_STATUS_TIMEOUT) != RT_EOK)
        {
			i2c_reset(handle->twix);
			drv_i2c_stop(handle->twix);
            LOG_E("i2c stop failed");
        }
    }

	if(ret == RT_EOK && handle->res == RT_EOK)
	{
		LOG_D("Message sent successfully");
		return handle->msgs_index;
		
	}
	else
	{
		LOG_E("Message sent failed");
		return handle->msgs_index - 1 < 0 ? 0 : handle->msgs_index - 1;
	}
}

static rt_err_t i2c_block_begin(struct i2c_handle *handle, struct rt_i2c_msg *msg, rt_uint8_t count)
{
    rt_uint32_t except_status;
	rt_uint8_t addr;
    
	if (count)
    {
        drv_i2c_start(handle->twix);
        if(drv_i2c_wait(handle->twix, STATUS_REPEATED_START, I2C_WAIT_STATUS_TIMEOUT) != RT_EOK)
        {
            LOG_E("i2c restart failed");
            return RT_EIO;
        }
    }
    else
    {
        drv_i2c_start(handle->twix);
        if(drv_i2c_wait(handle->twix, STATUS_START, I2C_WAIT_STATUS_TIMEOUT) != RT_EOK)
        {
            LOG_E("i2c start failed");
            return RT_EIO;
        }
    }

    if(msg->flags & RT_I2C_ADDR_10BIT)
    {
        LOG_E("10-bit address is not supported");
        return RT_EIO;
    }
    else
    {
        addr = msg->addr << 1;
        if (msg->flags & RT_I2C_RD)
        {
            addr |= 1;
            except_status = STATUS_ADDR_R_ACK;
        }
        else
            except_status = STATUS_ADDR_W_ACK;
        
        drv_i2c_send(handle->twix, addr);
        if(drv_i2c_wait(handle->twix, except_status, I2C_WAIT_STATUS_TIMEOUT) != RT_EOK)
        {
            LOG_E("i2c send address failed, address=%#X", addr);
            return RT_EIO;
        }
    }

    return RT_EOK;
}

static rt_err_t i2c_block_read(struct i2c_handle *handle, struct rt_i2c_msg *msg)
{
    rt_uint8_t *p = msg->buf;
    rt_int32_t count = msg->len;

    while (count-- > 0)
    {
        drv_i2c_enable_ack(handle->twix, count > 0 ? RT_TRUE : RT_FALSE);
        if(drv_i2c_wait(handle->twix, count > 0 ? STATUS_DATA_R_ACK : STATUS_DATA_R_NAK, I2C_WAIT_STATUS_TIMEOUT) != RT_EOK)
        {
            LOG_E("i2c receive failed");
            return RT_EIO;
        }
        drv_i2c_recv(handle->twix, p++);
    }

    return RT_EOK;
}

static rt_err_t i2c_block_write(struct i2c_handle *handle, struct rt_i2c_msg *msg)
{
    rt_uint8_t *p = msg->buf;
    rt_int32_t count = msg->len;

    while (count-- > 0)
    {
        drv_i2c_send(handle->twix, *p++);
        if(drv_i2c_wait(handle->twix, STATUS_DATA_W_ACK, I2C_WAIT_STATUS_TIMEOUT) != RT_EOK)
        {
            LOG_E("i2c write failed");
            return RT_EIO;
        }
    }

    return RT_EOK;
}

rt_size_t master_block_xfer(struct i2c_handle *handle, struct rt_i2c_msg msgs[], rt_uint32_t num)
{
    struct rt_i2c_msg *msg;
    rt_int32_t i;

    if(num == 0)
        return 0;
        
    for (i = 0; i < num; i++)
    {
        msg = &msgs[i];

        if (!(msg->flags & RT_I2C_NO_START))
        {
            if ((i2c_block_begin(handle, msg, i) != RT_EOK))
                goto out;
        }
        if (msg->flags & RT_I2C_RD)
        {
            if ((i2c_block_read(handle, msg) != RT_EOK))
                goto out;
        }
        else
        {
            if(i2c_block_write(handle, msg) != RT_EOK)
                goto out;
        }
    }

	if ((msg->flags & RT_I2C_NO_STOP))
		return i;
		
out:
    
	drv_i2c_stop(handle->twix);
	if(drv_i2c_wait(handle->twix, STATUS_IDLE, I2C_WAIT_STATUS_TIMEOUT) != RT_EOK)
	{
		i2c_reset(handle->twix);
		drv_i2c_stop(handle->twix);
		LOG_E("i2c stop failed");
	}

    return i;
}

rt_size_t master_xfer(struct rt_i2c_bus_device *bus, struct rt_i2c_msg msgs[], rt_uint32_t num)
{
	struct i2c_handle *handle = (struct i2c_handle *)bus->priv;
	rt_uint32_t retries = bus->retries;
	rt_size_t send_num;

	do
	{
		if(handle->mode == I2C_MODE_INT)
			send_num = master_int_xfer(handle, msgs, num);
		else
			send_num = master_block_xfer(handle, msgs, num);
		
		if(send_num == num)
			break;
	}
	while (retries--);

	return send_num;
}

static const struct rt_i2c_bus_device_ops ops =
{
    master_xfer,
    RT_NULL,
    RT_NULL
};

static rt_err_t drv_i2c_register(char *name, struct i2c_handle *handle, struct rt_i2c_bus_device *bus, struct twi_param *param)
{
	rt_err_t res;

	if(handle->mode == I2C_MODE_INT)
	{
		int vector;
		char str[10];

		if(handle->twix == TWI0)
			vector = TWI0_INTERRUPT;
		else if(handle->twix == TWI1)
			vector = TWI1_INTERRUPT;
		else
			vector = TWI2_INTERRUPT;
		
		rt_sprintf(str, "%s irq", name);
		rt_hw_interrupt_install(vector, i2c_irq_handler, handle, str);
		rt_hw_interrupt_umask(vector);

		rt_sprintf(str, "%s sem", name);
		handle->sem = rt_sem_create(str, 0, RT_IPC_FLAG_FIFO);
		if(handle->sem == RT_NULL)
		{
			LOG_E("Create %s semaphore failed", name);
			res = RT_ERROR;
			goto error;
		}
	}

	res = drv_i2c_init(handle->twix, param);
	if(res != RT_EOK)
		goto error;

	bus->priv = (void *)handle;
	bus->ops = &ops;
	if(rt_i2c_bus_device_register(bus, name) != RT_EOK)
	{
		LOG_E("Register %s bus device failed", name);
		res = RT_ERROR;
		goto error;
	}

	return RT_EOK;

error:

	rt_sem_delete(handle->sem);

	return res;
}
int rt_hw_i2c_init(void)
{
	rt_err_t res = RT_EOK;
#ifdef TINA_USING_I2C0
	{
		static struct i2c_handle i2c0_handle =
		{
			TWI0, 0, 0, RT_NULL, RT_NULL, 0, 0, 0, RT_ERROR, I2C_MODE_INT
		};
		
		static struct rt_i2c_bus_device i2c0_bus;

		struct twi_param param =
		{
			.clk = 400000,
			.own_addr = 0x00,
			.i2c_sda_port = GPIO_PORT_E,
			.i2c_sda_pin = GPIO_PIN_12,
			.i2c_sda_fun = IO_FUN_2,
			.i2c_scl_port = GPIO_PORT_E,
			.i2c_scl_pin = GPIO_PIN_11,
			.i2c_scl_fun = IO_FUN_2,
		};
		
		res = drv_i2c_register("i2c0", &i2c0_handle, &i2c0_bus, &param);
	}
#endif

#ifdef TINA_USING_I2C1
	{
		static struct i2c_handle i2c1_handle =
		{
			TWI1, 0, 0, RT_NULL, RT_NULL, 0, 0, 0, RT_ERROR, I2C_MODE_INT
		};
		
		static struct rt_i2c_bus_device i2c1_bus;

		struct twi_param param =
		{
			.clk = 400000,
			.own_addr = 0x00,
			.i2c_sda_port = GPIO_PORT_D,
			.i2c_sda_pin = GPIO_PIN_6,
			.i2c_sda_fun = IO_FUN_2,
			.i2c_scl_port = GPIO_PORT_D,
			.i2c_scl_pin = GPIO_PIN_5,
			.i2c_scl_fun = IO_FUN_2,
		};

		res = drv_i2c_register("i2c1", &i2c1_handle, &i2c1_bus, &param);
	}
#endif

#ifdef TINA_USING_I2C2
	{
		static struct i2c_handle i2c2_handle =
		{
			TWI2, 0, 0, RT_NULL, RT_NULL, 0, 0, 0, RT_ERROR, I2C_MODE_INT
		};
		
		static struct rt_i2c_bus_device i2c2_bus;

		struct twi_param param =
		{
			.clk = 400000,
			.own_addr = 0x00,
			.i2c_sda_port = GPIO_PORT_E,
			.i2c_sda_pin = GPIO_PIN_1,
			.i2c_sda_fun = IO_FUN_3,
			.i2c_scl_port = GPIO_PORT_E,
			.i2c_scl_pin = GPIO_PIN_0,
			.i2c_scl_fun = IO_FUN_3,
		};

		res = drv_i2c_register("i2c2", &i2c2_handle, &i2c2_bus, &param);
	}
#endif

	return res;
}

INIT_DEVICE_EXPORT(rt_hw_i2c_init);

#endif