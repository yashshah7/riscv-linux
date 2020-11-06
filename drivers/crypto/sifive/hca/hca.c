// SPDX-License-Identifier: GPL-2.0-only
/*
 * SiFive Hardware Cryptographic Accelerator (HCA)
 *
 * Copyright (C) 2020 SiFive Inc
 */

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include "hca.h"

struct sifive_hca_dev *hca_dev;

/*
 * The cryptophic algorithms
 */
static struct skcipher_alg *general_hca_cipher_algs[] = {
/*
	&sifive_hca_ecb_aes_alg,
	&sifive_hca_cbc_aes_alg,
	&sifive_hca_cfb_aes_alg,
	&sifive_hca_ofb_aes_alg,
	&sifive_hca_ctr_aes_alg,
	&sifive_hca_gcm_aes_alg,
	&sifive_hca_ccm_aes_alg,
*/
};

static struct ahash_alg *general_hca_ahash_algs[] = {
//	&sifive_hca_sha224_alg,
//	&sifive_hca_sha256_alg,
//	&sifive_hca_sha384_alg,
	&sifive_hca_sha512_alg,
};

static const struct sifive_hca_algs general_hca_algs = {
	.cipher_algs = general_hca_cipher_algs,
	.ncipher_algs = ARRAY_SIZE(general_hca_cipher_algs),
	.ahash_algs = general_hca_ahash_algs,
	.nahash_algs = ARRAY_SIZE(general_hca_ahash_algs),
	.has_dma = true,
};

static void sifive_hca_dequeue_req(void)
{
	struct crypto_async_request *req = NULL, *backlog = NULL;
	struct sifive_hca_ctx *ctx;

	spin_lock_bh(&hca_dev->lock);
	backlog = crypto_get_backlog(&hca_dev->queue);
	req = crypto_dequeue_request(&hca_dev->queue);
	spin_unlock_bh(&hca_dev->lock);

	if (!req)
		return;

	if (backlog)
		backlog->complete(backlog, -EINPROGRESS);

	ctx = crypto_tfm_ctx(req->tfm);
	ctx->ops->crypt(req);
}

int sifive_hca_queue_req(struct crypto_async_request *req)
{
	int ret;

	spin_lock_bh(&hca_dev->lock);
	ret = crypto_enqueue_request(&hca_dev->queue, req);
	spin_unlock_bh(&hca_dev->lock);

	if (ret != -EINPROGRESS)
		return ret;

	sifive_hca_dequeue_req();

	return -EINPROGRESS;
}

/* HCA driver interrupt handler */
static irqreturn_t sifive_hca_irq_handler(int irq, void *dev_id)
{
	struct sifive_hca_dev *hca = dev_id;

	if (sifive_hca_dma_int_status(hca))
		sifive_hca_dma_int_handle(hca);

	switch (sifive_hca_get_fifo_target(hca)) {
		case HCA_CR_IFIFOTGT_AES:
			/* AES handler */
			if (sifive_hca_crypto_int_status(hca)) {
				complete(&hca->aes_completion);
				sifive_hca_crypto_int_ack(hca);
			}
			break;
		case HCA_CR_IFIFOTGT_SHA:
			/* SHA handler */
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

/* Initialize hardware crypto accelerator */
static int sifive_hca_init(const struct device *dev, struct sifive_hca_dev *hca)
{
	uint32_t ret;

	ret = sifive_hca_get_rev(hca, 0);
	if (!ret) {
		dev_err(dev, "HCA revision error\n");
		return -ENODEV;
	}

	if (hca->algs->ahash_algs) {
		ret = sifive_hca_get_sha_rev(hca, 0);
		if (!ret) {
			dev_err(dev, "SHA revision error\n");
			return -ENODEV;
		}
	}

	sifive_hca_crypto_int_disable(hca);
	sifive_hca_ofifo_int_disable(hca);
	sifive_hca_dma_int_disable(hca);
	init_completion(&hca->aes_completion);

	return 0;
}

/* Register algs to crypto framework */
static int sifive_hca_add_algs(struct sifive_hca_dev *hca)
{
	int ret;

	//ret = sifive_aes_algs_register(hca);
	//if (ret)
	//	goto err_aes_algs;

	ret = sifive_ahash_algs_register(hca);
	if (ret)
		goto err_ahash_algs;

	return 0;

err_ahash_algs:
	sifive_ahash_algs_unregister(hca);
//err_aes_algs:
//	sifive_aes_algs_unregister(hca);

	return ret;
}

static void sifive_hca_remove_algs(struct sifive_hca_dev *hca)
{
	//sifive_aes_algs_unregister(hca);
	sifive_ahash_algs_unregister(hca);
}

static int sifive_hca_probe(struct platform_device *pdev)
{
	struct sifive_hca_dev *hca;
	int ret;

	/* Ensure there is only one HCA */
	if (hca_dev) {
		dev_err(&pdev->dev, "Only one HCA device authorized\n");
		return -EEXIST;
	}

	hca = devm_kzalloc(&pdev->dev, sizeof(*hca), GFP_KERNEL);
	if (!hca)
		return -ENOMEM;

	/* Get HCA capability of this device */
	hca->algs = of_device_get_match_data(&pdev->dev);
	if (IS_ERR(hca->algs))
		return PTR_ERR(hca->algs);

	/* GET MMIO mapping for HCA */
	hca->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(hca->regs))
		return PTR_ERR(hca->regs);

	/*
 	 * There are two interrupts sinks, one for the AES/SHA/DMA and
 	 * one for the TRNG.
 	 */
	hca->irq = platform_get_irq(pdev, 0);
	if (hca->irq < 0)
		return hca->irq;

	ret = devm_request_irq(&pdev->dev, hca->irq, sifive_hca_irq_handler,
			       0, dev_name(&pdev->dev), hca);
	if (ret) {
		dev_err(&pdev->dev, "Unable to attach HCA interrupt\n");
		return ret;
	}

	hca->dev = &pdev->dev;

	spin_lock_init(&hca->lock);

	/* Initialize the embedded DMA */
	ret = sifive_hca_dma_init(hca);
	if (ret) {
		dev_err(&pdev->dev, "Unable to initialize embedded DMA of HCA\n");
		goto err_exit;
	}

	/* Initialize hardware crypto accelerator */
	ret = sifive_hca_init(&pdev->dev, hca);
	if (ret) {
		dev_err(&pdev->dev, "Unable to initialize crypto accelerator\n");
		goto err_dma;
	}

	/* Add algs to crypto framework */
	ret = sifive_hca_add_algs(hca);
	if (ret) {
		dev_err(&pdev->dev, "Unable to register algorithms\n");
		goto err_alg;
	}

	platform_set_drvdata(pdev, hca);

	hca_dev = hca;

	return 0;

err_alg:
	sifive_hca_remove_algs(hca);
err_dma:
	sifive_hca_dma_free(hca);
err_exit:

	return ret;
}

static int sifive_hca_remove(struct platform_device *pdev)
{
	struct sifive_hca_dev *hca = platform_get_drvdata(pdev);

	sifive_hca_remove_algs(hca);
	sifive_hca_dma_free(hca);
	sifive_hca_crypto_int_disable(hca);
	sifive_hca_ofifo_int_disable(hca);
	sifive_hca_dma_int_disable(hca);
	free_irq(hca->irq, hca);

	return 0;
}

/*
 * Compatible rule:
 * sifive,hca-<version> for the general HCA IP block programming model.
 * sifive,<chip>-hca-<version> for the HCA as integrated on a particular chip
 */
static const struct of_device_id sifive_hca_of_match[] = {
	{ .compatible = "sifive,hca-0.5", .data = &general_hca_algs },
	{}
};
MODULE_DEVICE_TABLE(of, sifive_hca_of_match);

static struct platform_driver sifive_hca_driver = {
	.probe		= sifive_hca_probe,
	.remove		= sifive_hca_remove,
	.driver		= {
		.name	= "sifive-hca",
		.of_match_table = of_match_ptr(sifive_hca_of_match),
	},
};
module_platform_driver(sifive_hca_driver);

MODULE_AUTHOR("Zong Li <zong.li@sifive.com>");
MODULE_DESCRIPTION("Support for SiFive HCA");
MODULE_LICENSE("GPL v2");
