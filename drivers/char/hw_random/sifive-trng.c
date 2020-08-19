// SPDX-License-Identifier: GPL-2.0-only
/*
 * SiFive True Random Number Generator Driver
 *
 * Author: Yash Shah: <yash.shah@sifive.com>
 *
 * Copyright (C) 2020 SIFIVE INC
 */

#include <linux/hw_random.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/of_irq.h>

/* Registers */
#define SIFIVE_TRNG_CR_REG	0xE0
#define SIFIVE_TRNG_SR_REG	0xE4
#define SIFIVE_TRNG_DATA_REG	0xE8

/* Registers fields */
#define SIFIVE_TRNG_CR_HTSTART	BIT(0)
#define SIFIVE_TRNG_CR_RNDIRQEN	BIT(1)
#define SIFIVE_TRNG_CR_HTIRQEN	BIT(2)
#define SIFIVE_TRNG_CR_BURSTEN	BIT(3)

#define SIFIVE_TRNG_SR_RNDRDY	BIT(0)
#define SIFIVE_TRNG_SR_SRCS	BIT(1)
#define SIFIVE_TRNG_SR_HTR	BIT(2)
#define SIFIVE_TRNG_SR_HTS	BIT(3)

struct sifive_trng_data {
	int irq;
	void __iomem *reg;
	struct device *dev;
	struct hwrng ops;
	struct completion data_ready;
};

static int sifive_trng_read(struct hwrng *rng, void *buf, size_t max, bool wait)
{
	struct sifive_trng_data *ddata = (struct sifive_trng_data *)rng->priv;
	size_t read = 0;
	u32 val, ret;
	u32 *data = buf;

	while (read < max) {
		/* enable interrupt */
		val = readl(ddata->reg + SIFIVE_TRNG_CR_REG);
		val |= SIFIVE_TRNG_CR_RNDIRQEN;
		writel(val, ddata->reg + SIFIVE_TRNG_CR_REG);

		ret = wait_for_completion_timeout(&ddata->data_ready,
						  msecs_to_jiffies(3000));
		if (!ret)
			return -ETIMEDOUT;

		*data++ = readl(ddata->reg + SIFIVE_TRNG_DATA_REG);
		read += 4;
	}

	return read;	/* No of bytes read */
}

static irqreturn_t sifive_trng_irq_handler(int irq, void *id)
{
	struct sifive_trng_data *ddata = (struct sifive_trng_data *)id;
	u32 val;

	val = readl(ddata->reg + SIFIVE_TRNG_SR_REG);
	if (val & SIFIVE_TRNG_SR_RNDRDY) {
		complete(&ddata->data_ready);
		/* disable interrupt */
		val = readl(ddata->reg + SIFIVE_TRNG_CR_REG);
		val &= ~SIFIVE_TRNG_CR_RNDIRQEN;
		writel(val, ddata->reg + SIFIVE_TRNG_CR_REG);
	}
	if (val & SIFIVE_TRNG_SR_SRCS) {
		dev_err(ddata->dev, "Entropy source has failed\n");
		writel(SIFIVE_TRNG_SR_SRCS, ddata->reg + SIFIVE_TRNG_SR_REG);
	}

	return IRQ_HANDLED;
}

static int sifive_trng_probe(struct platform_device *pdev)
{
	struct sifive_trng_data *ddata;
	struct device_node *np = pdev->dev.of_node;
	bool burst_en;
	int ret;

	ddata = devm_kzalloc(&pdev->dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	ddata->reg = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ddata->reg))
		return PTR_ERR(ddata->reg);

	ddata->irq = platform_get_irq(pdev, 1);
	if (ddata->irq < 0)
		return ddata->irq;

	ret = devm_request_irq(&pdev->dev, ddata->irq, sifive_trng_irq_handler,
			       0, dev_name(&pdev->dev), ddata);
	if (ret) {
		dev_err(&pdev->dev, "Could not request TRNG IRQ\n");
		return ret;
	}

	ddata->ops.priv	= (unsigned long)ddata;
	ddata->ops.read	= sifive_trng_read;
	ddata->ops.name	= pdev->name;
	ddata->dev = &pdev->dev;

	dev_set_drvdata(&pdev->dev, ddata);

	init_completion(&ddata->data_ready);

	/* Run and wait for health test to finish */
	writel(SIFIVE_TRNG_CR_HTSTART, ddata->reg + SIFIVE_TRNG_CR_REG);

	do {
		ret = readl(ddata->reg + SIFIVE_TRNG_SR_REG);
	} while (ret & SIFIVE_TRNG_SR_HTR);

	if (readl(ddata->reg + SIFIVE_TRNG_SR_REG) & SIFIVE_TRNG_SR_HTS) {
		dev_err(&pdev->dev, "Health test failed\n");
		return -EIO;
	}

	/* Clear health test */
	writel(0, ddata->reg + SIFIVE_TRNG_CR_REG);

	/* Optional Parameter */
	burst_en = of_property_read_bool(np, "burst-mode");
	if (burst_en) {
		writel(SIFIVE_TRNG_CR_BURSTEN, ddata->reg + SIFIVE_TRNG_CR_REG);
		dev_info(&pdev->dev, "Burst enabled\n");
	}

	/* Dummy read to clear RNDRDY bit */
	readl(ddata->reg + SIFIVE_TRNG_DATA_REG);

	ret = devm_hwrng_register(&pdev->dev, &ddata->ops);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register HW TRNG\n");
		return ret;
	}

	dev_info(&pdev->dev, "Successfully registered HW TRNG\n");

	return 0;
}

static int sifive_trng_remove(struct platform_device *pdev)
{
	struct sifive_trng_data *ddata = dev_get_drvdata(&pdev->dev);

	/* clear all the config settings */
	writel(0, ddata->reg + SIFIVE_TRNG_CR_REG);
	return 0;
}

static const struct of_device_id sifive_trng_match[] = {
	{ .compatible = "sifive,hca-0.5" },
	{},
};
MODULE_DEVICE_TABLE(of, sifive_trng_match);

static struct platform_driver sifive_trng_driver = {
	.driver = {
		.name = "sifive-hwrandom",
		.of_match_table = of_match_ptr(sifive_trng_match),
	},
	.probe = sifive_trng_probe,
	.remove = sifive_trng_remove
};

module_platform_driver(sifive_trng_driver);

MODULE_AUTHOR("Yash Shah <yash.shah@sifive.com>");
MODULE_LICENSE("GPL v2");
