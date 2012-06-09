/*
 * NXP LPC32xx SoC Key Scan Interface
 *
 * Authors:
 *    Kevin Wells <kevin.wells@nxp.com>
 *    Roland Stigge <stigge@antcom.de>
 *
 * Copyright (C) 2010 NXP Semiconductors
 * Copyright (C) 2012 Roland Stigge
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * This controller supports square key matrices from 1x1 up to 8x8
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/input/matrix_keypad.h>

#define DRV_NAME				"lpc32xx_keys"

/*
 * Key scanner register offsets
 */
#define LPC32XX_KS_DEB(x)			((x) + 0x00)
#define LPC32XX_KS_STATE_COND(x)		((x) + 0x04)
#define LPC32XX_KS_IRQ(x)			((x) + 0x08)
#define LPC32XX_KS_SCAN_CTL(x)			((x) + 0x0C)
#define LPC32XX_KS_FAST_TST(x)			((x) + 0x10)
#define LPC32XX_KS_MATRIX_DIM(x)		((x) + 0x14) /* 1..8 */
#define LPC32XX_KS_DATA(x, y)			((x) + 0x40 + ((y) << 2))

#define LPC32XX_KSCAN_DEB_NUM_DEB_PASS(n)	((n) & 0xFF)

#define LPC32XX_KSCAN_SCOND_IN_IDLE		0x0
#define LPC32XX_KSCAN_SCOND_IN_SCANONCE		0x1
#define LPC32XX_KSCAN_SCOND_IN_IRQGEN		0x2
#define LPC32XX_KSCAN_SCOND_IN_SCAN_MATRIX	0x3

#define LPC32XX_KSCAN_IRQ_PENDING_CLR		0x1

#define LPC32XX_KSCAN_SCTRL_SCAN_DELAY(n)	((n) & 0xFF)

#define LPC32XX_KSCAN_FTST_FORCESCANONCE	0x1
#define LPC32XX_KSCAN_FTST_USE32K_CLK		0x2

#define LPC32XX_KSCAN_MSEL_SELECT(n)		((n) & 0xF)

/*
 * Key scanner configuration structure
 */
struct lpc32xx_kscan_cfg {
	u32 matrix_sz;		/* Size of matrix in XxY, ie. 3 = 3x3 */
	unsigned short *keymap;	/* Pointer to key map for the scan matrix */
	u32 deb_clks;		/* Debounce clocks (based on 32KHz clock) */
	u32 scan_delay;		/* Scan delay (based on 32KHz clock) */
	int row_shift;
};

struct lpc32xx_kscan_drv {
	struct input_dev *input;
	struct lpc32xx_kscan_cfg *kscancfg;
	struct clk *clk;
	void __iomem *kscan_base;
	int irq;
	u8 lastkeystates[8];
	u32 io_p_start;
	u32 io_p_size;
};

static void lpc32xx_mod_states(struct lpc32xx_kscan_drv *kscandat, int col)
{
	u8 key;
	int row;
	unsigned changed, scancode, keycode;

	key = readl(LPC32XX_KS_DATA(kscandat->kscan_base, col));
	changed = key ^ kscandat->lastkeystates[col];
	if (changed) {
		for (row = 0; row < kscandat->kscancfg->matrix_sz; row++) {
			if (changed & (1 << row)) {
				/* Key state changed, signal an event */
				scancode = MATRIX_SCAN_CODE(
				    row, col, kscandat->kscancfg->row_shift);
				keycode = kscandat->kscancfg->keymap[scancode];
				input_event(kscandat->input, EV_MSC, MSC_SCAN,
					    scancode);
				input_report_key(kscandat->input, keycode,
						 key & (1 << row));
			}
		}

		kscandat->lastkeystates[col] = key;
	}
}

static irqreturn_t lpc32xx_kscan_irq(int irq, void *dev_id)
{
	int i;
	struct lpc32xx_kscan_drv *kscandat = dev_id;

	for (i = 0; i < kscandat->kscancfg->matrix_sz; i++)
		lpc32xx_mod_states(kscandat, i);

	writel(1, LPC32XX_KS_IRQ(kscandat->kscan_base));

	input_sync(kscandat->input);

	return IRQ_HANDLED;
}

static int lpc32xx_kscan_open(struct input_dev *dev)
{
	struct lpc32xx_kscan_drv *kscandat = input_get_drvdata(dev);

	return clk_prepare_enable(kscandat->clk);
}

static void lpc32xx_kscan_close(struct input_dev *dev)
{
	struct lpc32xx_kscan_drv *kscandat = input_get_drvdata(dev);

	clk_disable_unprepare(kscandat->clk);
}

static struct lpc32xx_kscan_cfg *lpc32xx_parse_dt(struct device *dev)
{
	struct lpc32xx_kscan_cfg *pdata;
	struct device_node *np = dev->of_node;
	u32 rows, columns;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "could not allocate memory for platform data\n");
		return NULL;
	}

	of_property_read_u32(np, "keypad,num-rows", &rows);
	of_property_read_u32(np, "keypad,num-columns", &columns);
	if (!rows || rows != columns) {
		dev_err(dev, "rows and columns must be specified and be equal!\n");
		goto out1;
	}

	pdata->matrix_sz = rows;
	pdata->row_shift = get_count_order(columns);

	of_property_read_u32(np, "nxp,debounce-delay-ms", &pdata->deb_clks);
	of_property_read_u32(np, "nxp,scan-delay-ms", &pdata->scan_delay);
	if (!pdata->deb_clks || !pdata->scan_delay) {
		dev_err(dev, "debounce or scan delay not specified\n");
		goto out1;
	}

	pdata->keymap = kzalloc(sizeof(pdata->keymap[0]) *
				(rows << pdata->row_shift), GFP_KERNEL);
	if (!pdata->keymap) {
		dev_err(dev, "could not allocate memory for keymap\n");
		goto out1;
	}

	return pdata;
out1:
	kfree(pdata);
	return NULL;
}

static int __devinit lpc32xx_kscan_probe(struct platform_device *pdev)
{
	struct lpc32xx_kscan_drv *kscandat;
	struct resource *res;
	int error;

	kscandat = kzalloc(sizeof(struct lpc32xx_kscan_drv), GFP_KERNEL);
	if (!kscandat) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get platform I/O memory\n");
		error = -EBUSY;
		goto out1;
	}

	res = request_mem_region(res->start, resource_size(res), pdev->name);
	if (!res) {
		dev_err(&pdev->dev, "failed to request I/O memory\n");
		error = -EBUSY;
		goto out1;
	}
	kscandat->io_p_start = res->start;
	kscandat->io_p_size = resource_size(res);

	kscandat->kscan_base = ioremap(res->start, resource_size(res));
	if (!kscandat->kscan_base) {
		dev_err(&pdev->dev, "failed to remap I/O memory\n");
		error = -EBUSY;
		goto out2;
	}

	/* Get the key scanner clock */
	kscandat->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(kscandat->clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		error = PTR_ERR(kscandat->clk);
		goto out3;
	}
	clk_enable(kscandat->clk);

	kscandat->irq = platform_get_irq(pdev, 0);
	if ((kscandat->irq < 0) || (kscandat->irq >= NR_IRQS)) {
		dev_err(&pdev->dev, "failed to get platform irq\n");
		error = -EINVAL;
		goto out4;
	}
	error = request_irq(kscandat->irq, lpc32xx_kscan_irq,
			    0, pdev->name, kscandat);
	if (error) {
		dev_err(&pdev->dev, "failed to request irq\n");
		goto out4;
	}

	kscandat->input = input_allocate_device();
	if (kscandat->input == NULL) {
		dev_err(&pdev->dev, "failed to allocate device\n");
		error = -ENOMEM;
		goto out5;
	}

	kscandat->kscancfg = lpc32xx_parse_dt(&pdev->dev);
	if (!kscandat->kscancfg) {
		dev_err(&pdev->dev, "failed to get platform data\n");
		error = -EINVAL;
		goto out6;
	}

	/* Setup key input */
	kscandat->input->evbit[0]	= BIT_MASK(EV_KEY);
	kscandat->input->name		= pdev->name;
	kscandat->input->phys		= "matrix-keys/input0";
	kscandat->input->id.vendor	= 0x0001;
	kscandat->input->id.product	= 0x0001;
	kscandat->input->id.version	= 0x0100;
	kscandat->input->open		= lpc32xx_kscan_open;
	kscandat->input->close		= lpc32xx_kscan_close;
	kscandat->input->dev.parent	= &pdev->dev;

	matrix_keypad_build_keymap(NULL, NULL, kscandat->kscancfg->matrix_sz,
				   kscandat->kscancfg->matrix_sz,
				   kscandat->kscancfg->keymap, kscandat->input);

	platform_set_drvdata(pdev, kscandat);

	input_set_drvdata(kscandat->input, kscandat);
	input_set_capability(kscandat->input, EV_MSC, MSC_SCAN);

	error = input_register_device(kscandat->input);
	if (error) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto out6;
	}

	/* Configure the key scanner */
	writel(kscandat->kscancfg->deb_clks,
	       LPC32XX_KS_DEB(kscandat->kscan_base));
	writel(kscandat->kscancfg->scan_delay,
	       LPC32XX_KS_SCAN_CTL(kscandat->kscan_base));
	writel(LPC32XX_KSCAN_FTST_USE32K_CLK,
	       LPC32XX_KS_FAST_TST(kscandat->kscan_base));
	writel(kscandat->kscancfg->matrix_sz,
	       LPC32XX_KS_MATRIX_DIM(kscandat->kscan_base));
	writel(1, LPC32XX_KS_IRQ(kscandat->kscan_base));
	clk_disable(kscandat->clk);
	return 0;

out6:
	input_free_device(kscandat->input);
out5:
	free_irq(kscandat->irq, pdev);
out4:
	clk_disable(kscandat->clk);
	clk_put(kscandat->clk);
out3:
	iounmap(kscandat->kscan_base);
out2:
	release_mem_region(kscandat->io_p_start, kscandat->io_p_size);
out1:
	kfree(kscandat);

	return error;
}

static int __devexit lpc32xx_kscan_remove(struct platform_device *pdev)
{
	struct lpc32xx_kscan_drv *kscandat = platform_get_drvdata(pdev);

	input_unregister_device(kscandat->input);
	free_irq(kscandat->irq, pdev);
	clk_put(kscandat->clk);
	iounmap(kscandat->kscan_base);
	release_mem_region(kscandat->io_p_start, kscandat->io_p_size);
	kfree(kscandat);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int lpc32xx_kscan_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lpc32xx_kscan_drv *kscandat = platform_get_drvdata(pdev);

	/* Clear IRQ and disable clock */
	writel(1, LPC32XX_KS_IRQ(kscandat->kscan_base));
	clk_disable_unprepare(kscandat->clk);

	return 0;
}

static int lpc32xx_kscan_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lpc32xx_kscan_drv *kscandat = platform_get_drvdata(pdev);

	/* Enable clock and clear IRQ */
	clk_prepare_enable(kscandat->clk);
	writel(1, LPC32XX_KS_IRQ(kscandat->kscan_base));

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(lpc32xx_kscan_pm_ops, lpc32xx_kscan_suspend,
			 lpc32xx_kscan_resume);

static const struct of_device_id lpc32xx_kscan_match[] = {
	{ .compatible = "nxp,lpc3220-key" },
	{},
};
MODULE_DEVICE_TABLE(of, lpc32xx_kscan_match);

static struct platform_driver lpc32xx_kscan_driver = {
	.probe		= lpc32xx_kscan_probe,
	.remove		= __devexit_p(lpc32xx_kscan_remove),
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.pm	= &lpc32xx_kscan_pm_ops,
		.of_match_table = of_match_ptr(lpc32xx_kscan_match),
	}
};

module_platform_driver(lpc32xx_kscan_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kevin Wells <kevin.wells@nxp.com>");
MODULE_AUTHOR("Roland Stigge <stigge@antcom.de>");
MODULE_DESCRIPTION("Key scanner driver for LPC32XX devices");
