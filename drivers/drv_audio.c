#include <rtthread.h>

#include "drv_audio.h"
#include "drv_clock.h"
#include "drv_dma.h"
#include "interrupt.h"

#define DBG_TAG  "AUDIO"
#define DBG_LVL  DBG_DBG
#include <rtdbg.h>

void drv_audio_set_dac_volume(rt_int32_t volume)
{
	RT_ASSERT(volume < 64);

	AUDIO->dac_mixer_ctrl = (AUDIO->dac_mixer_ctrl & ~0x3f) | volume;
}

rt_err_t drv_audio_set_dac_sample_rate(rt_uint32_t rate)
{
	rt_uint32_t audio_clk;
	rt_uint32_t adfs;

	switch(rate)
	{
	case 8000:
		audio_clk = 196571429;
		adfs = 5;
		break;
	
	case 11025:
		audio_clk = 180600000;
		adfs = 4;
		break;

	case 12000:
		audio_clk = 196571429;
		adfs = 4;
		break;

	case 16000:
		audio_clk = 196571429;
		adfs = 3;
		break;

	case 22050:
		audio_clk = 180395238;
		adfs = 2;
		break;

	case 24000:
		audio_clk = 196571429;
		adfs = 2;
		break;

	case 32000:
		audio_clk = 196571429;
		adfs = 1;
		break;

	case 44100:
		audio_clk = 180600000;
		adfs = 0;
		break;

	case 48000:
		audio_clk = 196571429;
		adfs = 0;
		break;

	case 96000:
		audio_clk = 196571429;
		adfs = 7;
		break;

	case 192000:
		audio_clk = 196571429;
		adfs = 6;
		break;

	default:
		return RT_EINVAL;
	}

	if(audio_set_pll_clk(audio_clk) != RT_EOK)
	{
		LOG_E("AUDIO clock cannot set");
		return RT_ERROR;
	}

	AUDIO->dac_fifo_ctrl = (AUDIO->dac_fifo_ctrl & ~(0x7 << 29)) | adfs << 29;

	return RT_EOK;
}

void drv_audio_set_dac_sample_bits(rt_int32_t bits)
{
	RT_ASSERT(bits == 16 || bits == 24);

	if(bits == 24)
	{
		AUDIO->dac_fifo_ctrl |= 1 << 5;
		AUDIO->dac_fifo_ctrl &= ~(0x3 << 24);
	}
	else
	{
		AUDIO->dac_fifo_ctrl &= ~(1 << 5);
		AUDIO->dac_fifo_ctrl |= 0x3 << 24;
	}
}

void drv_audio_dac_init(void)
{
	bus_software_reset_disable(AUDIO_CODEC_GATING);
	bus_software_reset_enable(AUDIO_CODEC_GATING);

	bus_gate_clk_enable(AUDIO_CODEC_GATING);

	AUDIO->adda_tune = 0x44555556;
	AUDIO->bias_da16_cal_ctrl0 = 0x00000004;
	AUDIO->bias_da16_cal_ctrl1 = 0x10000000;

	rt_thread_mdelay(10);

	AUDIO->dac_fifo_ctrl = 1 << 26 | 0x3 << 21 | 0xf << 8 | 1;
	AUDIO->dac_mixer_ctrl = 1 << 31 | 1 << 30 | 1 << 29 | 1 << 28 | 1 << 27 | 1 << 26 | 1 << 25 | 1 << 24 | \
		0x3 << 22 | 1 << 21 | 0x2 << 16 | 1 << 15 | 0x2 << 8;
	AUDIO->dac_dpc = 1 << 18;

	CCU->audio_codec_clk |= 1 << 31;
}

void drv_audio_dac_drq_enable(rt_bool_t enable)
{
	if(enable)
		AUDIO->dac_fifo_ctrl |= 1 << 4;
	else
		AUDIO->dac_fifo_ctrl &= ~(1 << 4);
}

void drv_audio_dac_irq_enable(enum audio_irq_type type, rt_bool_t enable)
{
	RT_ASSERT((type & (~(AUDIO_IRQ_FIFO_EMPTY | AUDIO_IRQ_FIFO_UNDERRUN | AUDIO_IRQ_FIFO_OVERRUN))) == 0 && type != 0);

	if(enable)
		AUDIO->dac_fifo_ctrl |= type;
	else
		AUDIO->dac_fifo_ctrl &= ~type;
}

void drv_audio_dac_mono_enable(rt_bool_t enable)
{
	if(enable)
		AUDIO->dac_fifo_ctrl |= 1 << 6;
	else
		AUDIO->dac_fifo_ctrl &= ~(1 << 6);
}

void drv_audio_dac_enable(rt_bool_t enable)
{
	if(enable)
		AUDIO->dac_dpc |= 1 << 31;
	else
		AUDIO->dac_dpc &= ~(1 << 31);
}

void drv_audio_dac_fifo_flush(void)
{
	AUDIO->dac_fifo_ctrl |= 1;
}

void drv_audio_dac_debug_mode_enable(rt_bool_t enable)
{
	if(enable)
		AUDIO->dac_debug = 1 << 11 | 0x1 << 9 | 1 << 8;
	else
		AUDIO->dac_debug = 0;
}


#ifdef RT_USING_AUDIO

#include <rtdevice.h>

#define AUDIO_TX_BLOCK_SIZE		4096
#define AUDIO_TX_BLOCK_COUNT	2
#define AUDIO_TX_DMA			NDMA_CH0

struct audio_handle
{
	struct rt_audio_configure config;
	rt_int32_t volume;
	rt_uint8_t dac_tx_buffer[AUDIO_TX_BLOCK_COUNT * AUDIO_TX_BLOCK_SIZE] __attribute__((aligned (4)));
	rt_sem_t sem;
	rt_bool_t audio_status;
};

static void set_audio_dma(rt_uint16_t samplebits, const rt_uint8_t *buffer, rt_size_t size)
{
	struct dma_param param;
	enum dma_data_width data_width;

	if(samplebits == 16)
		data_width = DMA_DATA_WIDTH_16BIT;
	else
		data_width = DMA_DATA_WIDTH_32BIT;

	param.byte_num = size;
	param.continuous_enable = RT_FALSE;
	param.dst_addr = (rt_uint32_t *)&AUDIO->dac_txdata;
	param.dst_addr_increase = RT_FALSE;
	param.dst_data_width = data_width;
	param.dst_drq_type = NDMA_DST_DRQ_AUDIO_CODEC_DAC;
	param.src_addr = (rt_uint32_t *)buffer;
	param.src_addr_increase = RT_TRUE;
	param.src_data_width = data_width;
	param.src_drq_type = NDMA_SRC_DRQ_SDRAM;
	drv_dma_init(AUDIO_TX_DMA, &param);
}

static void audio_dma_irq_callback(void *param)
{
	struct rt_audio_device *audio = (struct rt_audio_device *)param;
	struct audio_handle *handle = (struct audio_handle *)audio->parent.user_data;

	if(drv_dma_get_irq_status(AUDIO_TX_DMA, DMA_IRQ_FT))
	{
		drv_dma_clear_irq(AUDIO_TX_DMA, DMA_IRQ_FT);

		rt_sem_release(handle->sem);
	}
}

static void audio_dac_dma_entry(void *param)
{
	struct rt_audio_device *audio = (struct rt_audio_device *)param;
	struct audio_handle *handle = (struct audio_handle *)audio->parent.user_data;

	while(1)
	{
		if(handle->audio_status)
			rt_audio_tx_complete(audio);
		else
			rt_thread_mdelay(10);
	}
}

static rt_err_t getcaps(struct rt_audio_device *audio, struct rt_audio_caps *caps)
{
	struct audio_handle *handle = (struct audio_handle *)audio->parent.user_data;

	switch(caps->main_type)
	{
	case AUDIO_TYPE_QUERY:
		switch(caps->sub_type)
		{
		case AUDIO_TYPE_QUERY:
			caps->udata.mask = AUDIO_TYPE_OUTPUT | AUDIO_TYPE_MIXER;
			break;
		
		default:
			return RT_EINVAL;
		}
		break;
	
	case AUDIO_TYPE_OUTPUT:
		switch(caps->sub_type)
		{
		case AUDIO_DSP_PARAM:
			rt_memcpy(&caps->udata.config, &handle->config, sizeof(struct rt_audio_configure));
			break;
		
		default:
			return RT_EINVAL;
		}
		break;
	
	case AUDIO_TYPE_MIXER:
		switch(caps->sub_type)
		{
		case AUDIO_MIXER_QUERY:
			caps->udata.mask = AUDIO_MIXER_VOLUME | AUDIO_MIXER_MUTE;
			break;
		
		case AUDIO_MIXER_VOLUME:
			caps->udata.value = handle->volume;
			break;
		
		default:
			return RT_EINVAL;
		}
		break;

	default:
		return RT_EINVAL;
	}

	return RT_EOK;
}
static rt_err_t configure(struct rt_audio_device *audio, struct rt_audio_caps *caps)
{
	struct audio_handle *handle = (struct audio_handle *)audio->parent.user_data;
	rt_err_t res;

	switch(caps->main_type)
	{
	case AUDIO_TYPE_MIXER:
		switch(caps->sub_type)
		{
		case AUDIO_MIXER_VOLUME:
			if(caps->udata.value > 99)
			{
				LOG_E("Volume cannot exceed 99");
				return RT_EINVAL;
			}
			else
				drv_audio_set_dac_volume(caps->udata.value * 63 / 99);
			break;
		
		case AUDIO_MIXER_MUTE:
			drv_audio_set_dac_volume(0);
			break;
		}
		break;

	case AUDIO_TYPE_OUTPUT:
		switch(caps->sub_type)
		{
		case AUDIO_DSP_PARAM:
			rt_memcpy(&handle->config, &caps->udata.config, sizeof(struct rt_audio_configure));

			res = drv_audio_set_dac_sample_rate(handle->config.samplerate);
			if(res != RT_EOK)
				return res;

			if(caps->udata.config.channels == 1)
				drv_audio_dac_mono_enable(RT_TRUE);
			else if(caps->udata.config.channels == 2)
				drv_audio_dac_mono_enable(RT_FALSE);
			else
				return RT_EINVAL;

			if(handle->config.samplebits == 16 || handle->config.samplebits == 24)
				drv_audio_set_dac_sample_bits(handle->config.samplebits);
			else
				return RT_EINVAL;
			break;
		
		case AUDIO_DSP_SAMPLERATE:
			handle->config.samplerate = caps->udata.config.samplerate;

			res = drv_audio_set_dac_sample_rate(handle->config.samplerate);
			if(res != RT_EOK)
				return res;
			break;
		
		case AUDIO_DSP_CHANNELS:
			handle->config.channels = caps->udata.config.channels;

			if(caps->udata.config.channels == 1)
				drv_audio_dac_mono_enable(RT_TRUE);
			else if(caps->udata.config.channels == 2)
				drv_audio_dac_mono_enable(RT_FALSE);
			else
				return RT_EINVAL;
			break;
		
		case AUDIO_DSP_SAMPLEBITS:
			handle->config.samplebits = caps->udata.config.samplebits;

			if(handle->config.samplebits == 16 || handle->config.samplebits == 24)
				drv_audio_set_dac_sample_bits(handle->config.samplebits);
			else
				return RT_EINVAL;
			break;
		
		default:
			return RT_EINVAL;
		}
		break;
	}

	return RT_EOK;
}
static rt_err_t init(struct rt_audio_device *audio)
{
	return RT_EOK;
}
static rt_err_t start(struct rt_audio_device *audio, int stream)
{
	struct audio_handle *handle = (struct audio_handle *)audio->parent.user_data;

	handle->audio_status = RT_TRUE;

	return RT_EOK;
}
static rt_err_t stop(struct rt_audio_device *audio, int stream)
{
	struct audio_handle *handle = (struct audio_handle *)audio->parent.user_data;

	handle->audio_status = RT_FALSE;

	return RT_EOK;
}
static rt_size_t transmit(struct rt_audio_device *audio, const void *write_buffer, void *read_buffer, rt_size_t size)
{
	struct audio_handle *handle = (struct audio_handle *)audio->parent.user_data;
	
	rt_sem_take(handle->sem, RT_WAITING_FOREVER);

	drv_dma_irq_enable(AUDIO_TX_DMA, DMA_IRQ_FT, RT_FALSE);
	drv_dma_enable(AUDIO_TX_DMA, RT_FALSE);
	set_audio_dma(handle->config.samplebits, write_buffer, size);
	drv_dma_irq_enable(AUDIO_TX_DMA, DMA_IRQ_FT, RT_TRUE);
	drv_dma_enable(AUDIO_TX_DMA, RT_TRUE);

	return size;
}
static void buffer_info(struct rt_audio_device *audio, struct rt_audio_buf_info *info)
{
	struct audio_handle *handle = (struct audio_handle *)audio->parent.user_data;

	info->block_count = AUDIO_TX_BLOCK_COUNT;
	info->block_size = AUDIO_TX_BLOCK_SIZE;
	info->buffer = handle->dac_tx_buffer;
	info->total_size = AUDIO_TX_BLOCK_SIZE * AUDIO_TX_BLOCK_COUNT;
}

static struct rt_audio_ops ops =
{
	.getcaps = getcaps,
	.configure = configure,
	.init = init,
	.start = start,
	.stop = stop,
	.transmit = transmit,
	.buffer_info = buffer_info
};

static int rt_hw_audio_init(void)
{
	static struct rt_audio_device dev;
	static struct audio_handle handle = {0};
	rt_thread_t thread;
	rt_err_t res;

	drv_audio_dac_init();
	drv_audio_dac_drq_enable(RT_TRUE);
	drv_audio_dac_enable(RT_TRUE);
	
	drv_dma_register_irq_callback(AUDIO_TX_DMA, audio_dma_irq_callback, &dev);

	dev.ops = &ops;
	res = rt_audio_register(&dev, "sound0", RT_DEVICE_FLAG_WRONLY, &handle);
	if(res != RT_EOK)
		return res;

	handle.sem = rt_sem_create("dac dma irq", AUDIO_TX_BLOCK_COUNT, RT_IPC_FLAG_FIFO);
	if(handle.sem == RT_NULL)
		return RT_ERROR;
		
	thread = rt_thread_create("dac dma", audio_dac_dma_entry, &dev, 1024, 4, 5);
	if(thread == RT_NULL)
		return RT_ERROR;
	
	rt_thread_startup(thread);

	return RT_EOK;
}

INIT_DEVICE_EXPORT(rt_hw_audio_init);

#endif
