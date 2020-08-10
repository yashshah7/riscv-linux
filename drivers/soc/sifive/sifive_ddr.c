// SPDX-License-Identifier: GPL-2.0
/*
 * SiFive specific cadence DDR controller Driver
 *
 * Copyright (C) 2019-2020 SiFive, Inc.
 *
 */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <soc/sifive/sifive_ddr.h>

static ATOMIC_NOTIFIER_HEAD(ddr_err_chain);

int register_sifive_ddr_error_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&ddr_err_chain, nb);
}
EXPORT_SYMBOL_GPL(register_sifive_ddr_error_notifier);

int unregister_sifive_ddr_error_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&ddr_err_chain, nb);
}
EXPORT_SYMBOL_GPL(unregister_sifive_ddr_error_notifier);

static void handle_ce(struct sifive_ddr_priv *pv)
{
	u64 err_c_addr = 0x0;
	u64 err_c_data = 0x0;
	u32 err_c_synd, err_c_id;
	u32 sig_val_l, sig_val_h;

	sig_val_l = readl(pv->reg + ECC_C_ADDR_L_REG);
	sig_val_h = (readl(pv->reg + ECC_C_ADDR_H_REG) &
		     ECC_ADDR_H_MASK);
	err_c_addr = (((ulong)sig_val_h << CTL_REG_WIDTH_SHIFT) | sig_val_l);

	sig_val_l = readl(pv->reg + ECC_C_DATA_L_REG);
	sig_val_h = readl(pv->reg + ECC_C_DATA_H_REG);
	err_c_data = (((ulong)sig_val_h << CTL_REG_WIDTH_SHIFT) | sig_val_l);

	err_c_id = ((readl(pv->reg + ECC_U_C_ID_REG) &
		     ECC_C_ID_MASK) >> ECC_C_ID_SHIFT);

	err_c_synd = ((readl(pv->reg + ECC_C_SYND_REG) &
		      ECC_SYND_MASK) >> ECC_SYND_SHIFT);

	pv->error_count = 1;
	pv->page_frame_number = err_c_addr >> PAGE_SHIFT;
	pv->offset_in_page = err_c_addr & ~PAGE_MASK;
	pv->syndrome = err_c_synd;
	pv->top_layer = 0;
	pv->mid_layer = 0;
	pv->low_layer = -1;

	atomic_notifier_call_chain(&ddr_err_chain, SIFIVE_DDR_ERR_TYPE_CE, pv);
}

static void handle_ue(struct sifive_ddr_priv *pv)
{
	u64 err_u_addr = 0x0;
	u64 err_u_data = 0x0;
	u32 err_u_synd, err_u_id;
	u32 sig_val_l, sig_val_h;

	sig_val_l = readl(pv->reg + ECC_U_ADDR_L_REG);
	sig_val_h = (readl(pv->reg + ECC_U_ADDR_H_REG) &
		     ECC_ADDR_H_MASK);
	err_u_addr = (((ulong)sig_val_h << CTL_REG_WIDTH_SHIFT) | sig_val_l);

	sig_val_l = readl(pv->reg + ECC_U_DATA_L_REG);
	sig_val_h = readl(pv->reg + ECC_U_DATA_H_REG);
	err_u_data = (((ulong)sig_val_h << CTL_REG_WIDTH_SHIFT) | sig_val_l);

	err_u_id = ((readl(pv->reg + ECC_U_C_ID_REG) &
		    ECC_U_ID_MASK) >> ECC_U_ID_SHIFT);

	err_u_synd = ((readl(pv->reg + ECC_U_SYND_REG) &
		      ECC_SYND_MASK) >> ECC_SYND_SHIFT);

	pv->error_count = 1;
	pv->page_frame_number = err_u_addr >> PAGE_SHIFT;
	pv->offset_in_page = err_u_addr & ~PAGE_MASK;
	pv->syndrome = err_u_synd;
	pv->top_layer = 0;
	pv->mid_layer = 0;
	pv->low_layer = -1;

	atomic_notifier_call_chain(&ddr_err_chain, SIFIVE_DDR_ERR_TYPE_UE, pv);
}

static irqreturn_t ecc_isr(int irq, void *ptr)
{
	struct sifive_ddr_priv *pv = ptr;
	u32 intr_status;
	u32 val;

	/* Check the intr status and confirm ECC error intr */
	intr_status = readl(pv->reg + ECC_CTL_INT_STATUS_REG);

	dev_dbg(pv->dev, "InterruptStatus : 0x%x\n", intr_status);
	val = intr_status & (ECC_INT_CE_UE_MASK);
	if (!((val & ECC_CE_INTR_MASK) || (val & ECC_UE_INTR_MASK)))
		return IRQ_NONE;

	if (val & ECC_CE_INTR_MASK) {
		handle_ce(pv);

		/* Clear the interrupt source */
		if (val & ECC_INT_CE_EVENT)
			writel(ECC_INT_CE_EVENT,
			       pv->reg + ECC_CTL_INT_ACK_REG);
		else if (val & ECC_INT_SECOND_CE_EVENT)
			writel(ECC_INT_SECOND_CE_EVENT,
			       pv->reg + ECC_CTL_INT_ACK_REG);
		else
			dev_err(pv->dev, "Failed to clear IRQ\n");
	}

	if (val & ECC_UE_INTR_MASK) {
		handle_ue(pv);

		/* Clear the interrupt source */
		if (val & ECC_INT_UE_EVENT)
			writel(ECC_INT_UE_EVENT,
			       pv->reg + ECC_CTL_INT_ACK_REG);
		else if (val & ECC_INT_SECOND_UE_EVENT)
			writel(ECC_INT_SECOND_UE_EVENT,
			       pv->reg + ECC_CTL_INT_ACK_REG);
		else
			dev_err(pv->dev, "Failed to clear IRQ\n");
	}

	return IRQ_HANDLED;
}

static int sifive_ddr_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sifive_ddr_priv *priv;
	struct resource *res;
	int ret, irq;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->reg = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->reg))
		return PTR_ERR(priv->reg);

	irq = platform_get_irq(pdev, 0);
	ret = devm_request_irq(dev, irq, ecc_isr, 0, "sifive-fu540-ddr", priv);
	if (ret) {
		dev_err(dev, "request_irq failed\n");
		return ret;
	}

	/* Enable & set CE/UE Interrupts for DDR4 Controller */
	writel(~(ECC_INT_CE_UE_MASK), priv->reg + ECC_CTL_INT_MASK_REG);

	platform_set_drvdata(pdev, priv);
	dev_info(dev, "SiFive DDR probe successful\n");

	return 0;
}

static int sifive_ddr_remove(struct platform_device *pdev)
{
	struct sifive_ddr_priv *priv = platform_get_drvdata(pdev);

	/* Disable All ECC Interrupts for DDR4 Controller */
	writel(ECC_INT_CE_UE_MASK, priv->reg + ECC_CTL_INT_MASK_REG);

	return 0;
}

static const struct of_device_id sifive_ddr_of_match[] = {
	{ .compatible = "sifive,fu540-c000-ddr"},
	{},
};

MODULE_DEVICE_TABLE(of, sifive_ddr_of_match);

static struct platform_driver sifive_ddr_driver = {
	.driver = {
		.name = "sifive-ddr",
		.of_match_table = sifive_ddr_of_match,
	},
	.probe = sifive_ddr_probe,
	.remove = sifive_ddr_remove,
};

module_platform_driver(sifive_ddr_driver);

MODULE_AUTHOR("SiFive");
MODULE_DESCRIPTION("SiFive DDR Controller driver");
MODULE_LICENSE("GPL v2");
