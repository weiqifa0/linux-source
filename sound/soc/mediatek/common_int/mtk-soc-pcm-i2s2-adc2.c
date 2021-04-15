/*
* Copyright (C) 2015 MediaTek Inc.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*/

/*******************************************************************************
 *
 * Filename:
 * ---------
 *   mtk_osc_pcm_i2s2_adc2.c
 *
 * Project:
 * --------
 *   Audio Driver Kernel Function
 *
 * Description:
 * ------------
 *   Audio i2s2 to adc2 capture
 *
 * Author:
 * -------
 *
 *
 *------------------------------------------------------------------------------
 *
 *
 *******************************************************************************/


/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/

#include <linux/dma-mapping.h>
#include <sound/pcm_params.h>

#include "mtk-auddrv-common.h"
#include "mtk-soc-pcm-common.h"
#include "mtk-auddrv-def.h"
#include "mtk-auddrv-afe.h"
#include "mtk-auddrv-ana.h"
#include "mtk-auddrv-clk.h"
#include "mtk-auddrv-kernel.h"
#include "mtk-soc-afe-control.h"
#include "mtk-soc-pcm-platform.h"
#include "mtk-auddrv-gpio.h"

static struct afe_mem_control_t  *I2S2_ADC2_Control_context;
static struct snd_dma_buffer *Adc2_Capture_dma_buf;
static unsigned int mPlaybackDramState;
static struct device *mDev;

/*
 *    function implementation
 */

static void StartAudioI2S2ADC2Hardware(struct snd_pcm_substream *substream);
static void StopAudioI2S2adc2Hardware(struct snd_pcm_substream *substream);
static int mtk_i2s2_adc2_probe(struct platform_device *pdev);
static int mtk_i2s2_adc2_pcm_close(struct snd_pcm_substream *substream);
static int mtk_i2s2_adc2_data_probe(struct snd_soc_platform *platform);

static struct snd_pcm_hardware mtk_I2S2_adc2_hardware = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED),
	.formats =      SND_SOC_ADV_MT_FMTS,
	.rates =        SOC_HIGH_USE_RATE,
	.rate_min =     SOC_HIGH_USE_RATE_MIN,
	.rate_max =     SOC_HIGH_USE_RATE_MAX,
	.channels_min =     SOC_NORMAL_USE_CHANNELS_MIN,
	.channels_max =     SOC_NORMAL_USE_CHANNELS_MAX,
	.buffer_bytes_max = AWB_MAX_BUFFER_SIZE,
	.period_bytes_max = AWB_MAX_BUFFER_SIZE,
	.periods_min =     AWB_MIN_PERIOD_SIZE,
	.periods_max =      AWB_MAX_PERIOD_SIZE,
	.fifo_size =        0,
};

static void StopAudioI2S2adc2Hardware(struct snd_pcm_substream *substream)
{
	pr_warn("StopAudioI2S2adc2Hardware\n");

	SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_VUL_DATA2, false);

	irq_remove_user(substream, irq_request_number(Soc_Aud_Digital_Block_MEM_VUL_DATA2));

	/* 4-mic setting*/
	if (substream->runtime->channels > 2) {
		SetIntfConnection(Soc_Aud_InterCon_DisConnect,
			Soc_Aud_AFE_IO_Block_ADDA_UL, Soc_Aud_AFE_IO_Block_MEM_VUL);
		SetIntfConnection(Soc_Aud_InterCon_DisConnect,
			Soc_Aud_AFE_IO_Block_ADDA_UL2, Soc_Aud_AFE_IO_Block_MEM_VUL_DATA2);
	} else {
		SetIntfConnection(Soc_Aud_InterCon_DisConnect,
			Soc_Aud_AFE_IO_Block_ADDA_UL2, Soc_Aud_AFE_IO_Block_MEM_VUL_DATA2);
	}

	/* here to close APLL */
	DisableAPLLTunerbySampleRate(substream->runtime->rate);
	DisableALLbySampleRate(substream->runtime->rate);

	/* disable I2S2 pin */
	AudDrv_GPIO_EXTADC_Select(0);

}

static void StartAudioI2S2ADC2Hardware(struct snd_pcm_substream *substream)
{
	pr_warn("+StartAudioI2S2ADC2Hardware\n");

	/* enable I2S2 pin */
	AudDrv_GPIO_EXTADC_Select(1);

	if (substream->runtime->format == SNDRV_PCM_FORMAT_S32_LE ||
		substream->runtime->format == SNDRV_PCM_FORMAT_U32_LE) {
		pr_warn("+StartAudioI2S2ADC2Hardware SNDRV_PCM_FORMAT_U32_LE\n");
		SetConnectionFormat(OUTPUT_DATA_FORMAT_24BIT, Soc_Aud_AFE_IO_Block_MEM_VUL_DATA2);
		SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_VUL_DATA2,
			AFE_WLEN_32_BIT_ALIGN_8BIT_0_24BIT_DATA);

	} else {
		SetConnectionFormat(OUTPUT_DATA_FORMAT_16BIT, Soc_Aud_AFE_IO_Block_MEM_VUL_DATA2);
		SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_VUL_DATA2, AFE_WLEN_16_BIT);
	}

	/* 4-mic setting*/
	if (substream->runtime->channels > 2) {
		SetIntfConnection(Soc_Aud_InterCon_Connection,
			Soc_Aud_AFE_IO_Block_ADDA_UL, Soc_Aud_AFE_IO_Block_MEM_VUL);
		SetIntfConnection(Soc_Aud_InterCon_Connection,
			Soc_Aud_AFE_IO_Block_ADDA_UL2, Soc_Aud_AFE_IO_Block_MEM_VUL_DATA2);

		if (substream->runtime->format == SNDRV_PCM_FORMAT_S32_LE ||
			substream->runtime->format == SNDRV_PCM_FORMAT_U32_LE)
			SetConnectionFormat(OUTPUT_DATA_FORMAT_24BIT, Soc_Aud_AFE_IO_Block_MEM_VUL);
		else
			SetConnectionFormat(OUTPUT_DATA_FORMAT_16BIT, Soc_Aud_AFE_IO_Block_MEM_VUL);

		Afe_Set_Reg(AFE_MEMIF_PBUF_SIZE, 0x1 << 17, 0x1 << 17);

		/* I_03/I_04 I_17/I_18 source from external ADC */
		Afe_Set_Reg(AFE_ADDA_TOP_CON0, 0x1, 0x1);
		Afe_Set_Reg(AFE_ADDA6_TOP_CON0, 0x1, 0x1);
	} else {
		/* I_17/I_18 source from external ADC */
		Afe_Set_Reg(AFE_ADDA6_TOP_CON0, 0x1, 0x1);
		SetIntfConnection(Soc_Aud_InterCon_Connection,
			Soc_Aud_AFE_IO_Block_ADDA_UL2, Soc_Aud_AFE_IO_Block_MEM_VUL_DATA2);
	}

	/* external adc here to open APLL */
	EnableALLbySampleRate(substream->runtime->rate);
	EnableAPLLTunerbySampleRate(substream->runtime->rate);

	SetSampleRate(Soc_Aud_Digital_Block_MEM_VUL_DATA2, substream->runtime->rate);
	SetChannels(Soc_Aud_Digital_Block_MEM_VUL_DATA2, substream->runtime->channels);
	SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_VUL_DATA2, true);

	/* here to set interrupt */
	irq_add_user(substream,
		     irq_request_number(Soc_Aud_Digital_Block_MEM_VUL_DATA2),
		     substream->runtime->rate,
		     substream->runtime->period_size);
}

static int mtk_i2s2_adc2_alsa_stop(struct snd_pcm_substream *substream)
{
	pr_warn("mtk_i2s2_adc2_alsa_stop\n");
	StopAudioI2S2adc2Hardware(substream);
	RemoveMemifSubStream(Soc_Aud_Digital_Block_MEM_VUL_DATA2, substream);
	return 0;
}

static snd_pcm_uframes_t mtk_i2s2_adc2_pcm_pointer(struct snd_pcm_substream *substream)
{
	return get_mem_frame_index(substream,
		I2S2_ADC2_Control_context, Soc_Aud_Digital_Block_MEM_VUL_DATA2);
}

static int mtk_i2s2_adc2_pcm_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *hw_params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret = 0;

	substream->runtime->dma_bytes = params_buffer_bytes(hw_params);
	if (AllocateAudioSram(&substream->runtime->dma_addr,
			      &substream->runtime->dma_area,
			      substream->runtime->dma_bytes, substream,
			      params_format(hw_params), false) == 0) {
		mPlaybackDramState = false;
		/* pr_warn("mtk_pcm_hw_params dma_bytes = %d\n",substream->runtime->dma_bytes); */
	} else {
		substream->runtime->dma_area = Adc2_Capture_dma_buf->area;
		substream->runtime->dma_addr = Adc2_Capture_dma_buf->addr;
		mPlaybackDramState = true;
		AudDrv_Emi_Clk_On();
	}
	pr_warn("mtk_i2s2_adc2_pcm_hw_params dma_bytes = %zu dma_area = %p dma_addr = 0x%lx\n",
	       runtime->dma_bytes, runtime->dma_area, (long)runtime->dma_addr);

	set_mem_block(substream, hw_params, I2S2_ADC2_Control_context, Soc_Aud_Digital_Block_MEM_VUL_DATA2);
	return ret;
}

static int mtk_i2s2_adc2_capture_pcm_hw_free(struct snd_pcm_substream *substream)
{
	pr_warn("mtk_i2s2_adc2_capture_pcm_hw_free\n");
	if (mPlaybackDramState == true) {
		AudDrv_Emi_Clk_Off();
		mPlaybackDramState = false;
	} else
		freeAudioSram((void *)substream);
	return 0;
}

static int mtk_i2s2_adc2_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret = 0;

	mPlaybackDramState = false;

	pr_warn("mtk_i2s2_adc2_pcm_open\n");
	I2S2_ADC2_Control_context = Get_Mem_ControlT(Soc_Aud_Digital_Block_MEM_VUL_DATA2);
	runtime->hw = mtk_I2S2_adc2_hardware;
	memcpy((void *)(&(runtime->hw)), (void *)&mtk_I2S2_adc2_hardware, sizeof(struct snd_pcm_hardware));

	ret = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);

	if (ret < 0)
		pr_err("snd_pcm_hw_constraint_integer failed\n");

	runtime->hw.info |= SNDRV_PCM_INFO_INTERLEAVED;
	runtime->hw.info |= SNDRV_PCM_INFO_NONINTERLEAVED;
	runtime->hw.info |= SNDRV_PCM_INFO_MMAP_VALID;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		pr_debug("SNDRV_PCM_STREAM_CAPTURE\n");

	if (ret < 0) {
		pr_err("mtk_i2s2_adc2_pcm_close\n");
		mtk_i2s2_adc2_pcm_close(substream);
		return ret;
	}
	AudDrv_Clk_On();
	pr_debug("mtk_i2s2_adc2_pcm_open return\n");
	return 0;
}

static int mtk_i2s2_adc2_pcm_close(struct snd_pcm_substream *substream)
{
	AudDrv_Clk_Off();
	return 0;
}

static int mtk_i2s2_adc2_alsa_start(struct snd_pcm_substream *substream)
{
	pr_debug("mtk_i2s2_adc2_alsa_start\n");
	SetMemifSubStream(Soc_Aud_Digital_Block_MEM_VUL_DATA2, substream);
	StartAudioI2S2ADC2Hardware(substream);
	return 0;
}

static int mtk_i2s2_adc2_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	pr_debug("mtk_i2s2_adc2_pcm_trigger cmd = %d\n", cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		return mtk_i2s2_adc2_alsa_start(substream);
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		return mtk_i2s2_adc2_alsa_stop(substream);
	}
	return -EINVAL;
}

static int mtk_i2s2_adc2_pcm_copy(struct snd_pcm_substream *substream,
				  int channel, snd_pcm_uframes_t pos,
				  void __user *dst, snd_pcm_uframes_t count)
{
	return mtk_memblk_copy(substream, channel, pos, dst, count,
		I2S2_ADC2_Control_context, Soc_Aud_Digital_Block_MEM_VUL_DATA2);
}

static int mtk_i2s_adc2_pcm_silence(struct snd_pcm_substream *substream,
				    int channel, snd_pcm_uframes_t pos,
				    snd_pcm_uframes_t count)
{
	pr_debug("dummy_pcm_silence\n");
	return 0; /* do nothing */
}


static void *dummy_page[2];

static struct page *mtk_i2s2_adc2_pcm_page(struct snd_pcm_substream *substream,
					   unsigned long offset)
{
	pr_debug("dummy_pcm_page\n");
	return virt_to_page(dummy_page[substream->stream]); /* the same page */
}


static struct snd_pcm_ops mtk_i2s2_adc2_ops = {
	.open =     mtk_i2s2_adc2_pcm_open,
	.close =    mtk_i2s2_adc2_pcm_close,
	.ioctl =    snd_pcm_lib_ioctl,
	.hw_params =    mtk_i2s2_adc2_pcm_hw_params,
	.hw_free =  mtk_i2s2_adc2_capture_pcm_hw_free,
	.trigger =  mtk_i2s2_adc2_pcm_trigger,
	.pointer =  mtk_i2s2_adc2_pcm_pointer,
	.copy =     mtk_i2s2_adc2_pcm_copy,
	.silence =  mtk_i2s_adc2_pcm_silence,
	.page =     mtk_i2s2_adc2_pcm_page,
};

static struct snd_soc_platform_driver mtk_soc_platform = {
	.ops        = &mtk_i2s2_adc2_ops,
	.probe      = mtk_i2s2_adc2_data_probe,
};

static int mtk_i2s2_adc2_probe(struct platform_device *pdev)
{
	pr_debug("mtk_i2s2_adc2_probe\n");

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(64);
	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

	if (pdev->dev.of_node)
		dev_set_name(&pdev->dev, "%s", MT_SOC_I2S2_ADC2_PCM);

	pr_err("%s: dev name %s\n", __func__, dev_name(&pdev->dev));
	mDev = &pdev->dev;
	return snd_soc_register_platform(&pdev->dev,
					 &mtk_soc_platform);
}

static int mtk_i2s2_adc2_data_probe(struct snd_soc_platform *platform)
{
	pr_warn("mtk_i2s2_adc2_data_probe\n");
	AudDrv_Allocate_mem_Buffer(platform->dev, Soc_Aud_Digital_Block_MEM_VUL_DATA2, UL2_MAX_BUFFER_SIZE);
	Adc2_Capture_dma_buf =  Get_Mem_Buffer(Soc_Aud_Digital_Block_MEM_VUL_DATA2);
	return 0;
}

static int mtk_i2s2_adc2_remove(struct platform_device *pdev)
{
	pr_warn("%s\n", __func__);
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mt_soc_pcm_i2s2_adc2_of_ids[] = {
	{ .compatible = "mediatek,mt_soc_pcm_i2s2_adc2", },
	{}
};
#endif

static struct platform_driver mtk_i2s2_adc2_capture_driver = {
	.driver = {
		.name = MT_SOC_I2S2_ADC2_PCM,
		.owner = THIS_MODULE,
	},
	.probe = mtk_i2s2_adc2_probe,
	.remove = mtk_i2s2_adc2_remove,
};

static struct platform_device *soc_i2s2_adc2_capture_dev;

static int __init mtk_soc_i2s2_adc2_platform_init(void)
{
	int ret = 0;

	pr_debug("%s\n", __func__);
	soc_i2s2_adc2_capture_dev = platform_device_alloc(MT_SOC_I2S2_ADC2_PCM, -1);
	if (!soc_i2s2_adc2_capture_dev)
		return -ENOMEM;

	ret = platform_device_add(soc_i2s2_adc2_capture_dev);
	if (ret != 0) {
		platform_device_put(soc_i2s2_adc2_capture_dev);
		return ret;
	}

	ret = platform_driver_register(&mtk_i2s2_adc2_capture_driver);
	return ret;
}

static void __exit mtk_soc_i2s2_adc2_platform_exit(void)
{
	pr_debug("%s\n", __func__);
	platform_driver_unregister(&mtk_i2s2_adc2_capture_driver);
}

module_init(mtk_soc_i2s2_adc2_platform_init);
module_exit(mtk_soc_i2s2_adc2_platform_exit);

MODULE_DESCRIPTION("I2S2 ADC2 module platform driver");
MODULE_LICENSE("GPL");


