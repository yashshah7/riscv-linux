// SPDX-License-Identifier: GPL-2.0-only
/*
 * Author: Yash Shah <yash.shah@sifive.com>
 *
 * This work is based on Marvell's CESA driver
 *
 */

#include <crypto/aes.h>

#include "hca.h"

struct sifive_hca_aes_ctx {
	struct sifive_hca_ctx base;
	struct crypto_aes_ctx aes;
};

struct sifive_hca_op_ctx {
	u8 aes_mode;
	u8 aes_key_size;
	u8 aes_process_type;
	u32 aes_key[8];
	u32 aes_initv[4];
	int src_nents;
	int dst_nents;
};

static inline void
sifive_hca_skcipher_dma_cleanup(struct skcipher_request *req)
{
	struct sifive_hca_op_ctx *tmpl = skcipher_request_ctx(req);

	if (req->dst != req->src) {
		dma_unmap_sg(hca_dev->dev, req->dst, tmpl->dst_nents,
			     DMA_FROM_DEVICE);
		dma_unmap_sg(hca_dev->dev, req->src, tmpl->src_nents,
			     DMA_TO_DEVICE);
	} else {
		dma_unmap_sg(hca_dev->dev, req->src, tmpl->src_nents,
			     DMA_BIDIRECTIONAL);
	}
}

static inline void sifive_hca_skcipher_cleanup(struct skcipher_request *req)
{
	sifive_hca_skcipher_dma_cleanup(req);
}

static void sifive_hca_skcipher_std_crypt(struct skcipher_request *skreq)
{
	struct sifive_hca_op_ctx *tmpl = skcipher_request_ctx(skreq);
	unsigned int offset = 0;
	uint8_t buf[32];
	size_t len;

	while (offset < skreq->cryptlen)
	{
		len = min_t(size_t, skreq->cryptlen - offset,
			    HCA_AES_FIFO_SIZE);

		len = sg_pcopy_to_buffer(skreq->src, tmpl->src_nents, buf,
					  len, offset);

		reinit_completion(&hca_dev->aes_completion);

		if (sifive_hca_ififo_is_full(hca_dev)) {
			dev_err(hca_dev->dev, "IFIFO is full\n");
			break;
		} else {
			sifive_hca_fifo_push(hca_dev, buf, len);
		}

		if (sifive_hca_crypto_is_int_enable(hca_dev))
			wait_for_completion(&hca_dev->aes_completion);
		else
			sifive_hca_poll_aes_timeout(hca_dev,
						    HCA_AES_TIMEOUT);
		if (sifive_hca_ofifo_is_empty(hca_dev)) {
			dev_err(hca_dev->dev, "OFIFO is empty\n");
			break;
		} else {
			sifive_hca_fifo_pop(hca_dev, buf, len);
		}

		len = sg_pcopy_from_buffer(skreq->dst, tmpl->dst_nents, buf,
					   len, offset);
		offset += len;
	}
}

static void sifive_hca_skcipher_dma_crypt(struct skcipher_request *skreq)
{
	struct scatterlist *sg_src = skreq->src, *sg_dst = skreq->dst;
	dma_addr_t src, dst;
	u32 len;
	int ret;

	while (sg_src != NULL && sg_dst != NULL)
	{
		src = sg_dma_address(sg_src);
		dst = sg_dma_address(sg_dst);
		len = sg_dma_len(sg_src);

		reinit_completion(&hca_dev->dma_completion);
		ret = sifive_hca_dma_transfer(hca_dev, src, dst, len);
		if (ret < 0)
			break;

		if (sifive_hca_dma_is_int_enable(hca_dev))
			wait_for_completion(&hca_dev->dma_completion);
		else
			sifive_hca_poll_dma_timeout(hca_dev,
						    HCA_AES_TIMEOUT);

		sg_src = sg_next(sg_src);
		sg_dst = sg_next(sg_dst);
	}
}

static void sifive_hca_skcipher_crypt(struct crypto_async_request *req)
{
	struct skcipher_request *skreq = skcipher_request_cast(req);
	struct sifive_hca_op_ctx *tmpl = skcipher_request_ctx(skreq);
	u8 mode = tmpl->aes_mode;

	sifive_hca_fifo_target_aes(hca_dev);
	sifive_hca_fifo_set_be(hca_dev);

	sifive_hca_aes_set_mode(hca_dev, tmpl->aes_mode);
	sifive_hca_aes_set_keysize(hca_dev, tmpl->aes_key_size);
	sifive_hca_aes_set_process_type(hca_dev, tmpl->aes_process_type);

	sifive_hca_aes_set_key(hca_dev, tmpl->aes_key_size, &tmpl->aes_key[0]);
	sifive_hca_aes_set_initv(hca_dev, HCA_AES_INITV_SIZE,
				 &tmpl->aes_initv[0]);

	if (mode != HCA_AES_CR_MODE_ECB)
		sifive_hca_aes_set_init(hca_dev);

	if (hca_dev->algs->has_dma) {
		if (sifive_hca_dma_crypt_check(skreq->src, skreq->dst,
					       NULL, 0)) {
			sifive_hca_skcipher_dma_crypt(skreq);
			return;
		}
		sifive_hca_skcipher_dma_cleanup(skreq);
	}

	sifive_hca_skcipher_std_crypt(skreq);
}

static inline void
sifive_hca_skcipher_req_cleanup(struct crypto_async_request *req)
{
	struct skcipher_request *skreq = skcipher_request_cast(req);

	sifive_hca_skcipher_cleanup(skreq);
}

static const struct sifive_hca_req_ops sifive_hca_skcipher_req_ops = {
	.crypt = sifive_hca_skcipher_crypt,
	.cleanup = sifive_hca_skcipher_req_cleanup,
};

static void sifive_hca_skcipher_cra_exit(struct crypto_tfm *tfm)
{
	void *ctx = crypto_tfm_ctx(tfm);

	memzero_explicit(ctx, tfm->__crt_alg->cra_ctxsize);
}

static int sifive_hca_skcipher_cra_init(struct crypto_tfm *tfm)
{
	struct sifive_hca_ctx *ctx = crypto_tfm_ctx(tfm);

	ctx->ops = &sifive_hca_skcipher_req_ops;

	crypto_skcipher_set_reqsize(__crypto_skcipher_cast(tfm),
				    sizeof(struct sifive_hca_op_ctx));

	return 0;
}

static int sifive_hca_aes_setkey(struct crypto_skcipher *cipher, const u8 *key,
			      unsigned int len)
{
	struct crypto_tfm *tfm = crypto_skcipher_tfm(cipher);
	struct sifive_hca_aes_ctx *ctx = crypto_tfm_ctx(tfm);

	memcpy(ctx->aes.key_enc, key, len);
	ctx->aes.key_length = len;

	return 0;
}

static int sifive_hca_skcipher_dma_req_init(struct skcipher_request *req,
					 const struct sifive_hca_op_ctx *tmpl)
{
	int ret;

	if (req->src != req->dst) {
		ret = dma_map_sg(hca_dev->dev, req->src, tmpl->src_nents,
				 DMA_TO_DEVICE);
		if (!ret)
			return -ENOMEM;

		ret = dma_map_sg(hca_dev->dev, req->dst, tmpl->dst_nents,
				 DMA_FROM_DEVICE);
		if (!ret) {
			ret = -ENOMEM;
			goto err_unmap_src;
		}
	} else {
		ret = dma_map_sg(hca_dev->dev, req->src, tmpl->src_nents,
				 DMA_BIDIRECTIONAL);
		if (!ret)
			return -ENOMEM;
	}
	return 0;

err_unmap_src:
	dma_unmap_sg(hca_dev->dev, req->src, tmpl->src_nents,
		     req->dst != req->src ? DMA_TO_DEVICE : DMA_BIDIRECTIONAL);

	return ret;
}

static int sifive_hca_skcipher_req_init(struct skcipher_request *req,
				     struct sifive_hca_op_ctx *tmpl)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	unsigned int blksize = crypto_skcipher_blocksize(tfm);
	int ret = 0;

	if (!IS_ALIGNED(req->cryptlen, blksize))
		return -EINVAL;

	tmpl->src_nents = sg_nents_for_len(req->src, req->cryptlen);
	if (tmpl->src_nents < 0) {
		dev_err(hca_dev->dev, "Invalid number of src SG");
		return tmpl->src_nents;
	}
	tmpl->dst_nents = sg_nents_for_len(req->dst, req->cryptlen);
	if (tmpl->dst_nents < 0) {
		dev_err(hca_dev->dev, "Invalid number of dst SG");
		return tmpl->dst_nents;
	}

	if (hca_dev->algs->has_dma)
		ret = sifive_hca_skcipher_dma_req_init(req, tmpl);

	return ret;
}

static int sifive_hca_skcipher_queue_req(struct skcipher_request *req,
				      struct sifive_hca_op_ctx *tmpl)
{
	int ret;

	ret = sifive_hca_skcipher_req_init(req, tmpl);
	if (ret)
		return ret;

	ret = sifive_hca_queue_req(&req->base);

	if (sifive_hca_req_needs_cleanup(&req->base, ret))
		sifive_hca_skcipher_cleanup(req);

	return ret;
}

static int sifive_hca_aes_op(struct skcipher_request *req,
			  struct sifive_hca_op_ctx *tmpl)
{
	struct sifive_hca_aes_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	int i;
	u32 *key;

	key = ctx->aes.key_enc;

	for (i = 0; i < ctx->aes.key_length / sizeof(u32); i++)
		tmpl->aes_key[i] = cpu_to_be32(key[i]);

	tmpl->aes_key_size = ctx->aes.key_length;

	return sifive_hca_skcipher_queue_req(req, tmpl);
}

static int sifive_hca_ecb_aes_encrypt(struct skcipher_request *req)
{
	struct sifive_hca_op_ctx *tmpl = skcipher_request_ctx(req);

	tmpl->aes_process_type = HCA_AES_DIR_ENC;
	tmpl->aes_mode = HCA_AES_CR_MODE_ECB;

	return sifive_hca_aes_op(req, tmpl);
}

static int sifive_hca_ecb_aes_decrypt(struct skcipher_request *req)
{
	struct sifive_hca_op_ctx *tmpl = skcipher_request_ctx(req);

	tmpl->aes_process_type = HCA_AES_DIR_DEC;
	tmpl->aes_mode = HCA_AES_CR_MODE_ECB;

	return sifive_hca_aes_op(req, tmpl);
}

struct skcipher_alg sifive_hca_ecb_aes_alg = {
	.setkey = sifive_hca_aes_setkey,
	.encrypt = sifive_hca_ecb_aes_encrypt,
	.decrypt = sifive_hca_ecb_aes_decrypt,
	.min_keysize = AES_MIN_KEY_SIZE,
	.max_keysize = AES_MAX_KEY_SIZE,
	.base = {
		.cra_name = "ecb(aes)",
		.cra_driver_name = "sifive-ecb-aes",
		.cra_priority = 300,
		.cra_flags = CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC |
			     CRYPTO_ALG_ALLOCATES_MEMORY,
		.cra_blocksize = AES_BLOCK_SIZE,
		.cra_ctxsize = sizeof(struct sifive_hca_aes_ctx),
		.cra_alignmask = 0x1f,
		.cra_module = THIS_MODULE,
		.cra_init = sifive_hca_skcipher_cra_init,
		.cra_exit = sifive_hca_skcipher_cra_exit,
	},
};

static int sifive_hca_ctr_aes_encrypt(struct skcipher_request *req)
{
	struct sifive_hca_op_ctx *tmpl = skcipher_request_ctx(req);

	tmpl->aes_process_type = HCA_AES_DIR_ENC;
	tmpl->aes_mode = HCA_AES_CR_MODE_CTR;

	memcpy(tmpl->aes_initv, req->iv, HCA_AES_INITV_SIZE);

	return sifive_hca_aes_op(req, tmpl);
}

static int sifive_hca_ctr_aes_decrypt(struct skcipher_request *req)
{
	struct sifive_hca_op_ctx *tmpl = skcipher_request_ctx(req);

	tmpl->aes_process_type = HCA_AES_DIR_DEC;
	tmpl->aes_mode = HCA_AES_CR_MODE_CTR;

	memcpy(tmpl->aes_initv, req->iv, HCA_AES_INITV_SIZE);

	return sifive_hca_aes_op(req, tmpl);
}

struct skcipher_alg sifive_hca_ctr_aes_alg = {
	.setkey = sifive_hca_aes_setkey,
	.encrypt = sifive_hca_ctr_aes_encrypt,
	.decrypt = sifive_hca_ctr_aes_decrypt,
	.min_keysize = AES_MIN_KEY_SIZE,
	.max_keysize = AES_MAX_KEY_SIZE,
	.ivsize = HCA_AES_INITV_SIZE,
	.base = {
		.cra_name = "ctr(aes)",
		.cra_driver_name = "sifive-ctr-aes",
		.cra_priority = 300,
		.cra_flags = CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC |
			     CRYPTO_ALG_ALLOCATES_MEMORY,
		.cra_blocksize = AES_BLOCK_SIZE,
		.cra_ctxsize = sizeof(struct sifive_hca_aes_ctx),
		.cra_alignmask = 0x1f,
		.cra_module = THIS_MODULE,
		.cra_init = sifive_hca_skcipher_cra_init,
		.cra_exit = sifive_hca_skcipher_cra_exit,
	},
};

static int sifive_hca_ofb_aes_encrypt(struct skcipher_request *req)
{
	struct sifive_hca_op_ctx *tmpl = skcipher_request_ctx(req);

	tmpl->aes_process_type = HCA_AES_DIR_ENC;
	tmpl->aes_mode = HCA_AES_CR_MODE_OFB;

	memcpy(tmpl->aes_initv, req->iv, HCA_AES_INITV_SIZE);

	return sifive_hca_aes_op(req, tmpl);
}

static int sifive_hca_ofb_aes_decrypt(struct skcipher_request *req)
{
	struct sifive_hca_op_ctx *tmpl = skcipher_request_ctx(req);

	tmpl->aes_process_type = HCA_AES_DIR_DEC;
	tmpl->aes_mode = HCA_AES_CR_MODE_OFB;

	memcpy(tmpl->aes_initv, req->iv, HCA_AES_INITV_SIZE);

	return sifive_hca_aes_op(req, tmpl);
}

struct skcipher_alg sifive_hca_ofb_aes_alg = {
	.setkey = sifive_hca_aes_setkey,
	.encrypt = sifive_hca_ofb_aes_encrypt,
	.decrypt = sifive_hca_ofb_aes_decrypt,
	.min_keysize = AES_MIN_KEY_SIZE,
	.max_keysize = AES_MAX_KEY_SIZE,
	.ivsize = HCA_AES_INITV_SIZE,
	.base = {
		.cra_name = "ofb(aes)",
		.cra_driver_name = "sifive-ofb-aes",
		.cra_priority = 300,
		.cra_flags = CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC |
			     CRYPTO_ALG_ALLOCATES_MEMORY,
		.cra_blocksize = 1,
		.cra_ctxsize = sizeof(struct sifive_hca_aes_ctx),
		.cra_alignmask = 0x1f,
		.cra_module = THIS_MODULE,
		.cra_init = sifive_hca_skcipher_cra_init,
		.cra_exit = sifive_hca_skcipher_cra_exit,
	},
};

static int sifive_hca_cfb_aes_encrypt(struct skcipher_request *req)
{
	struct sifive_hca_op_ctx *tmpl = skcipher_request_ctx(req);

	tmpl->aes_process_type = HCA_AES_DIR_ENC;
	tmpl->aes_mode = HCA_AES_CR_MODE_CFB;

	memcpy(tmpl->aes_initv, req->iv, HCA_AES_INITV_SIZE);

	return sifive_hca_aes_op(req, tmpl);
}

static int sifive_hca_cfb_aes_decrypt(struct skcipher_request *req)
{
	struct sifive_hca_op_ctx *tmpl = skcipher_request_ctx(req);

	tmpl->aes_process_type = HCA_AES_DIR_DEC;
	tmpl->aes_mode = HCA_AES_CR_MODE_CFB;

	memcpy(tmpl->aes_initv, req->iv, HCA_AES_INITV_SIZE);

	return sifive_hca_aes_op(req, tmpl);
}

struct skcipher_alg sifive_hca_cfb_aes_alg = {
	.setkey = sifive_hca_aes_setkey,
	.encrypt = sifive_hca_cfb_aes_encrypt,
	.decrypt = sifive_hca_cfb_aes_decrypt,
	.min_keysize = AES_MIN_KEY_SIZE,
	.max_keysize = AES_MAX_KEY_SIZE,
	.ivsize = HCA_AES_INITV_SIZE,
	.base = {
		.cra_name = "cfb(aes)",
		.cra_driver_name = "sifive-cfb-aes",
		.cra_priority = 300,
		.cra_flags = CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC |
			     CRYPTO_ALG_ALLOCATES_MEMORY,
		.cra_blocksize = 1,
		.cra_ctxsize = sizeof(struct sifive_hca_aes_ctx),
		.cra_alignmask = 0x1f,
		.cra_module = THIS_MODULE,
		.cra_init = sifive_hca_skcipher_cra_init,
		.cra_exit = sifive_hca_skcipher_cra_exit,
	},
};

static int sifive_hca_cbc_aes_encrypt(struct skcipher_request *req)
{
	struct sifive_hca_op_ctx *tmpl = skcipher_request_ctx(req);

	tmpl->aes_process_type = HCA_AES_DIR_ENC;
	tmpl->aes_mode = HCA_AES_CR_MODE_CBC;

	memcpy(tmpl->aes_initv, req->iv, HCA_AES_INITV_SIZE);

	return sifive_hca_aes_op(req, tmpl);
}

static int sifive_hca_cbc_aes_decrypt(struct skcipher_request *req)
{
	struct sifive_hca_op_ctx *tmpl = skcipher_request_ctx(req);

	tmpl->aes_process_type = HCA_AES_DIR_DEC;
	tmpl->aes_mode = HCA_AES_CR_MODE_CBC;

	memcpy(tmpl->aes_initv, req->iv, HCA_AES_INITV_SIZE);

	return sifive_hca_aes_op(req, tmpl);
}

struct skcipher_alg sifive_hca_cbc_aes_alg = {
	.setkey = sifive_hca_aes_setkey,
	.encrypt = sifive_hca_cbc_aes_encrypt,
	.decrypt = sifive_hca_cbc_aes_decrypt,
	.min_keysize = AES_MIN_KEY_SIZE,
	.max_keysize = AES_MAX_KEY_SIZE,
	.ivsize = HCA_AES_INITV_SIZE,
	.base = {
		.cra_name = "cbc(aes)",
		.cra_driver_name = "sifive-cbc-aes",
		.cra_priority = 300,
		.cra_flags = CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC |
			     CRYPTO_ALG_ALLOCATES_MEMORY,
		.cra_blocksize = AES_BLOCK_SIZE,
		.cra_ctxsize = sizeof(struct sifive_hca_aes_ctx),
		.cra_alignmask = 0x1f,
		.cra_module = THIS_MODULE,
		.cra_init = sifive_hca_skcipher_cra_init,
		.cra_exit = sifive_hca_skcipher_cra_exit,
	},
};

static struct skcipher_alg *general_hca_cipher_algs[] = {
	&sifive_hca_cbc_aes_alg,
	&sifive_hca_cfb_aes_alg,
	&sifive_hca_ofb_aes_alg,
	&sifive_hca_ctr_aes_alg,
	&sifive_hca_ecb_aes_alg,
};

/* Register algs to crypto framework */
int sifive_aes_algs_register(struct sifive_hca_dev *hca)
{
	return crypto_register_skciphers(general_hca_cipher_algs[0],
					 ARRAY_SIZE(general_hca_cipher_algs));
}

void sifive_aes_algs_unregister(struct sifive_hca_dev *hca)
{
	crypto_unregister_skciphers(general_hca_cipher_algs[0],
				    ARRAY_SIZE(general_hca_cipher_algs));
}
