// SPDX-License-Identifier: GPL-2.0
/*
 * SiFive Platform EDAC Driver
 *
 * Copyright (C) 2018-2019 SiFive, Inc.
 *
 * This driver is partially based on octeon_edac-pc.c
 *
 */
#include <linux/edac.h>
#include <linux/platform_device.h>
#include "edac_module.h"
#include <soc/sifive/sifive_l2_cache.h>
#include <soc/sifive/sifive_ddr.h>

#define DRVNAME "sifive_edac"
#define EDAC_MOD_NAME "Sifive ECC Manager"

struct sifive_edac_priv {
	struct notifier_block notifier;
	struct edac_device_ctl_info *dci;
};

struct sifive_edac_mc_priv {
	struct notifier_block notifier;
	struct mem_ctl_info *mci;
};

/**
 * EDAC MC error callback
 *
 * @event: non-zero if unrecoverable.
 */
static
int ecc_mc_err_event(struct notifier_block *this, unsigned long event, void *ptr)
{
	struct sifive_ddr_priv *priv = ptr;
	struct sifive_edac_mc_priv *p;

	p = container_of(this, struct sifive_edac_mc_priv, notifier);
	if (event == SIFIVE_DDR_ERR_TYPE_UE) {
		edac_mc_handle_error(HW_EVENT_ERR_UNCORRECTED, p->mci,
				     priv->error_count, priv->page_frame_number,
				     priv->offset_in_page, priv->syndrome,
				     priv->top_layer, priv->mid_layer,
				     priv->low_layer, p->mci->ctl_name, "");
	} else if (event == SIFIVE_DDR_ERR_TYPE_CE) {
		edac_mc_handle_error(HW_EVENT_ERR_CORRECTED, p->mci,
				     priv->error_count, priv->page_frame_number,
				     priv->offset_in_page, priv->syndrome,
				     priv->top_layer, priv->mid_layer,
				     priv->low_layer, p->mci->ctl_name, "");
	}

	return NOTIFY_OK;
}

static int ecc_mc_register(struct platform_device *pdev)
{
	struct sifive_edac_mc_priv *p;
	struct edac_mc_layer layers[1];
	int ret;

	p = devm_kzalloc(&pdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->notifier.notifier_call = ecc_mc_err_event;
	platform_set_drvdata(pdev, p);

	layers[0].type = EDAC_MC_LAYER_CHIP_SELECT;
	layers[0].size = 1;
	layers[0].is_virt_csrow = true;

	p->mci = edac_mc_alloc(0, ARRAY_SIZE(layers), layers, 0);
	if (!p->mci) {
		dev_err(&pdev->dev, "Failed mem allocation for mc instance\n");
		return -ENOMEM;
	}

	p->mci->pdev = &pdev->dev;
	/* Initialize controller capabilities */
	p->mci->mtype_cap = MEM_FLAG_DDR4;
	p->mci->edac_ctl_cap = EDAC_FLAG_NONE | EDAC_FLAG_SECDED;
	p->mci->edac_cap = EDAC_FLAG_SECDED;
	p->mci->scrub_cap = SCRUB_UNKNOWN;
	p->mci->scrub_mode = SCRUB_HW_PROG;
	p->mci->ctl_name = dev_name(&pdev->dev);
	p->mci->dev_name = dev_name(&pdev->dev);
	p->mci->mod_name = EDAC_MOD_NAME;
	p->mci->ctl_page_to_phys = NULL;

	/* Interrupt feature is supported by cadence mc */
	edac_op_state = EDAC_OPSTATE_INT;

	ret = edac_mc_add_mc(p->mci);
	if (ret) {
		edac_printk(KERN_ERR, EDAC_MOD_NAME,
			    "Failed to register with EDAC core\n");
		goto err;
	}

	if (IS_ENABLED(CONFIG_SIFIVE_DDR))
		register_sifive_ddr_error_notifier(&p->notifier);

	return 0;

err:
	edac_mc_free(p->mci);

	return -ENXIO;
}

static int ecc_mc_unregister(struct platform_device *pdev)
{
	struct sifive_edac_mc_priv *p = platform_get_drvdata(pdev);

	if (IS_ENABLED(CONFIG_SIFIVE_DDR))
		unregister_sifive_ddr_error_notifier(&p->notifier);

	edac_mc_del_mc(&pdev->dev);
	edac_mc_free(p->mci);

	return 0;
}

/**
 * EDAC error callback
 *
 * @event: non-zero if unrecoverable.
 */
static
int ecc_err_event(struct notifier_block *this, unsigned long event, void *ptr)
{
	const char *msg = (char *)ptr;
	struct sifive_edac_priv *p;

	p = container_of(this, struct sifive_edac_priv, notifier);

	if (event == SIFIVE_L2_ERR_TYPE_UE)
		edac_device_handle_ue(p->dci, 0, 0, msg);
	else if (event == SIFIVE_L2_ERR_TYPE_CE)
		edac_device_handle_ce(p->dci, 0, 0, msg);

	return NOTIFY_OK;
}

static int ecc_register(struct platform_device *pdev)
{
	struct sifive_edac_priv *p;

	p = devm_kzalloc(&pdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->notifier.notifier_call = ecc_err_event;
	platform_set_drvdata(pdev, p);

	p->dci = edac_device_alloc_ctl_info(0, "sifive_ecc", 1, "sifive_ecc",
					    1, 1, NULL, 0,
					    edac_device_alloc_index());
	if (!p->dci)
		return -ENOMEM;

	p->dci->dev = &pdev->dev;
	p->dci->mod_name = "Sifive ECC Manager";
	p->dci->ctl_name = dev_name(&pdev->dev);
	p->dci->dev_name = dev_name(&pdev->dev);

	if (edac_device_add_device(p->dci)) {
		dev_err(p->dci->dev, "failed to register with EDAC core\n");
		goto err;
	}

	if (IS_ENABLED(CONFIG_SIFIVE_L2))
		register_sifive_l2_error_notifier(&p->notifier);

	return 0;

err:
	edac_device_free_ctl_info(p->dci);

	return -ENXIO;
}

static int ecc_unregister(struct platform_device *pdev)
{
	struct sifive_edac_priv *p = platform_get_drvdata(pdev);

	if (IS_ENABLED(CONFIG_SIFIVE_L2))
		unregister_sifive_l2_error_notifier(&p->notifier);

	edac_device_del_device(&pdev->dev);
	edac_device_free_ctl_info(p->dci);

	return 0;
}

static struct platform_device *sifive_pdev;

static int __init sifive_edac_init(void)
{
	int ret;

	sifive_pdev = platform_device_register_simple(DRVNAME, 0, NULL, 0);
	if (IS_ERR(sifive_pdev))
		return PTR_ERR(sifive_pdev);

	ret = ecc_register(sifive_pdev);
	if (ret)
		platform_device_unregister(sifive_pdev);

	ret = ecc_mc_register(sifive_pdev);
	if (ret) {
		ecc_unregister(sifive_pdev);
		platform_device_unregister(sifive_pdev);
	}

	return ret;
}

static void __exit sifive_edac_exit(void)
{
	ecc_unregister(sifive_pdev);
	ecc_mc_unregister(sifive_pdev);
	platform_device_unregister(sifive_pdev);
}

module_init(sifive_edac_init);
module_exit(sifive_edac_exit);

MODULE_AUTHOR("SiFive Inc.");
MODULE_DESCRIPTION("SiFive platform EDAC driver");
MODULE_LICENSE("GPL v2");
