/*
 * Remote processor machine-specific module for Davinci
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#define pr_fmt(fmt)    "%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/printk.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>

#include <mach/da8xx.h>
#include <mach/cputype.h>
#include <mach/psc.h>
#include <mach/remoteproc.h>

#include "remoteproc_internal.h"

/*
 * Technical Reference:
 * OMAP-L138 Applications Processor System Reference Guide
 * http://www.ti.com/litv/pdf/sprugm7d
 */

/* local reset bit (0 is asserted) in MDCTL15 register (section 9.6.18) */
#define LRST                BIT(8)

/* next state bits in MDCTL15 register (section 9.6.18) */
#define NEXT_ENABLED        0x3

/* register for DSP boot address in SYSCFG0 module (section 11.5.6) */
#define HOST1CFG        0x44

#define SYSCFG_CHIPSIG_OFFSET 0x174
#define SYSCFG_CHIPSIG_CLR_OFFSET 0x178
#define SYSCFG_CHIPINT0 (1 << 0)
#define SYSCFG_CHIPINT1 (1 << 1)
#define SYSCFG_CHIPINT2 (1 << 2)
#define SYSCFG_CHIPINT3 (1 << 3)

/**
 * struct davinci_rproc - davinci remote processor state
 * @rproc: rproc handle
 */
struct davinci_rproc {
	struct rproc *rproc;
	struct clk *dsp_clk;
};

void __iomem *syscfg0_base;

struct platform_device *remoteprocdev;
static struct work_struct workqueue;

static void handle_event(struct work_struct *work)
{
	struct rproc *rproc = platform_get_drvdata(remoteprocdev);

	/* Process incoming buffers on our vring */
	while (IRQ_HANDLED == rproc_vq_interrupt(rproc, 0));

	/* Must allow wakeup of potenitally blocking senders: */
	rproc_vq_interrupt(rproc, 1);
}

/**
 * davinci_rproc_callback() - inbound virtqueue message handler
 *
 * This handler is invoked directly by the kernel whenever the remote
 * core (DSP) has modified the state of a virtqueue.  There is no
 * "payload" message indicating the virtqueue index as is the case with
 * mailbox-based implementations on OMAP4.  As such, this handler "polls"
 * each known virtqueue index for every invocation.
 */
static irqreturn_t davinci_rproc_callback(int irq, void *p)
{
	if (__raw_readl(syscfg0_base + SYSCFG_CHIPSIG_OFFSET) &
	    SYSCFG_CHIPINT0) {
		/*
		 * Following can fail if work is pending; but it's OK since the
		 * work function will loop to process all incoming messages.
		 * schedule_work() calls handle_event with pending bit off.
		 */
		(void)schedule_work(&workqueue);

		/* Clear interrupt: */
		__raw_writel(SYSCFG_CHIPINT0, 
			syscfg0_base + SYSCFG_CHIPSIG_CLR_OFFSET);
	}

	return IRQ_HANDLED;
}

static int davinci_rproc_start(struct rproc *rproc)
{
	struct platform_device *pdev = to_platform_device(rproc->dev);
	struct device *dev = rproc->dev;
	struct davinci_rproc_pdata *pdata = dev->platform_data;
	struct davinci_soc_info *soc_info = &davinci_soc_info;
	void __iomem *psc_base;
	struct davinci_rproc *drproc = rproc->priv;
	struct clk *dsp_clk;
	int ret;

	INIT_WORK(&workqueue, handle_event);
	remoteprocdev = pdev;

	ret = request_irq(28, davinci_rproc_callback, IRQF_SHARED,
			  "davinci-remoteproc", drproc);
	if (ret) {
		dev_err(dev, "request_irq error: %d\n", ret);
		return ret;
	}

	/* hw requires the start (boot) address be on 1KB boundary */
	if (rproc->bootaddr & 0x3ff) {
		dev_err(dev, "invalid boot address: must be aligned to 1KB\n");
		return -EINVAL;
	}

	pr_info("getting clock in start\n");
	dsp_clk = clk_get(dev, pdata->clk_name);
	if (IS_ERR_OR_NULL(dsp_clk)) {
		dev_err(dev, "clk_get error: %ld\n", PTR_ERR(dsp_clk));
		return PTR_ERR(dsp_clk);
	}
	pr_info("got clk\n");

#if 0
	psc_base = ioremap(soc_info->psc_bases[0], SZ_4K);
	syscfg0_base = ioremap(DA8XX_SYSCFG0_BASE, SZ_4K);
	pr_info("MDCTL before: 0x%x\n", __raw_readl(psc_base + MDCTL + 4 * DA8XX_LPSC0_GEM));

	clk_enable(dsp_clk);
	drproc->dsp_clk = dsp_clk;

	/* insure local reset is asserted before writing start address */
	__raw_writel(NEXT_ENABLED, psc_base + MDCTL + 4 * DA8XX_LPSC0_GEM);

	__raw_writel(rproc->bootaddr, syscfg0_base + HOST1CFG);

	/* de-assert local reset to start the dsp running */
	__raw_writel(LRST | NEXT_ENABLED, psc_base + MDCTL + 4 * DA8XX_LPSC0_GEM);

	iounmap(syscfg0_base);
	iounmap(psc_base);
#else
	pr_info("start: clk = %p\n", dsp_clk);

	syscfg0_base = ioremap(DA8XX_SYSCFG0_BASE, SZ_4K);
	__raw_writel(rproc->bootaddr, syscfg0_base + HOST1CFG);

	clk_enable(dsp_clk);
	drproc->dsp_clk = dsp_clk;
#endif

	return 0;
}

static int davinci_rproc_stop(struct rproc *rproc)
{
	struct davinci_soc_info *soc_info = &davinci_soc_info;
	void __iomem *psc_base;
	struct davinci_rproc *drproc = rproc->priv;
	struct clk *dsp_clk = drproc->dsp_clk;

#if 1
	psc_base = ioremap(soc_info->psc_bases[0], SZ_4K);

	/* halt the dsp by asserting local reset */
	__raw_writel(NEXT_ENABLED, psc_base + MDCTL + 4 * DA8XX_LPSC0_GEM);

	pr_info("calling clk_disable: %p\n", dsp_clk);
	clk_disable(dsp_clk);
	pr_info("calling clk_put: %p\n", dsp_clk);
	clk_put(dsp_clk);

	iounmap(psc_base);

	free_irq(28, drproc);

	/* Flush any pending work: */
	(void)flush_work_sync(&workqueue);

#else
	pr_info("calling clk_disable: 0x%x\n", dsp_clk);
	clk_disable(dsp_clk);
	clk_put(dsp_clk);
#endif

	return 0;
}

/* kick a virtqueue */
static void davinci_rproc_kick(struct rproc *rproc, int vqid)
{
	/* Poll for ack from other side first: */
	while(__raw_readl(syscfg0_base + SYSCFG_CHIPSIG_OFFSET) &
		SYSCFG_CHIPINT2);

	/* Interupt remote proc: */
	__raw_writel(SYSCFG_CHIPINT2, syscfg0_base + SYSCFG_CHIPSIG_OFFSET);
}

static struct rproc_ops davinci_rproc_ops = {
	.start = davinci_rproc_start,
	.stop = davinci_rproc_stop,
	.kick = davinci_rproc_kick,
};

static int davinci_rproc_probe(struct platform_device *pdev)
{
	struct davinci_soc_info *soc_info = &davinci_soc_info;
	struct davinci_rproc_pdata *pdata = pdev->dev.platform_data;
	struct davinci_rproc *drproc;
	struct rproc *rproc;
	void __iomem *psc_base;
	int ret;

	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(pdev->dev.parent, "dma_set_coherent_mask: %d\n", ret);
		return ret;
	}

	rproc = rproc_alloc(&pdev->dev, pdata->name, &davinci_rproc_ops,
				pdata->firmware, sizeof(*drproc));
	if (!rproc)
		return -ENOMEM;

	drproc = rproc->priv;
	drproc->rproc = rproc;

	platform_set_drvdata(pdev, rproc);

	ret = rproc_register(rproc);
	if (ret)
		goto free_rproc;

	/* insure the dsp is halted by asserting local reset */
	psc_base = ioremap(soc_info->psc_bases[0], SZ_4K);
	__raw_writel(NEXT_ENABLED, psc_base + MDCTL + 4 * DA8XX_LPSC0_GEM);
	iounmap(psc_base);

	return 0;

free_rproc:
	rproc_free(rproc);
	return ret;
}

static int __devexit davinci_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);

	return rproc_unregister(rproc);
}

static struct platform_driver davinci_rproc_driver = {
	.probe = davinci_rproc_probe,
	.remove = __devexit_p(davinci_rproc_remove),
	.driver = {
		.name = "davinci-rproc",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(davinci_rproc_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Davinci Remote Processor control driver");
