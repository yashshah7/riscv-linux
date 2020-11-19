// SPDX-License-Identifier: GPL-2.0-only
/*
 * Author: Yash Shah <yash.shah@sifive.com>
 *
 */

#include <crypto/aead.h>
#include <crypto/internal/aead.h>

#include "aead_util.h"
#include "hca.h"

#define AEAD_INITV_SIZE 12

struct sifive_hca_aes_ctx {
	struct sifive_hca_ctx base;
	struct crypto_aes_ctx aes;
};

struct sifive_hca_op_ctx {
	struct scatterlist *src_sg;
	struct scatterlist *dst_sg;
	struct scatterlist *aad_sg;
	int src_nents;
	int dst_nents;
	int aad_nents;
	int dlen;
	int ccmt;
	int ccmq;
	u32 aes_key[8];
	u32 aes_initv[4];
	u8 aes_mode;
	u8 aes_key_size;
	u8 aes_process_type;
};

static inline void
sifive_hca_aead_dma_cleanup(struct aead_request *areq)
{
	struct sifive_hca_op_ctx *tmpl = aead_request_ctx(areq);

	dma_unmap_sg(hca_dev->dev, tmpl->aad_sg, tmpl->aad_nents,
		     DMA_TO_DEVICE);
	if (areq->dst != areq->src) {
		dma_unmap_sg(hca_dev->dev, tmpl->dst_sg, tmpl->dst_nents,
				DMA_FROM_DEVICE);
		dma_unmap_sg(hca_dev->dev, tmpl->src_sg, tmpl->src_nents,
				DMA_TO_DEVICE);
	} else {
		dma_unmap_sg(hca_dev->dev, tmpl->src_sg, tmpl->src_nents,
				DMA_BIDIRECTIONAL);
	}
}

static inline void sifive_hca_aead_cleanup(struct aead_request *areq)
{
        sifive_hca_aead_dma_cleanup(areq);
}

static void sifive_hca_aead_process_crypt(struct crypto_async_request *req)
{
	struct aead_request *areq = aead_request_cast(req);
	unsigned int skip = areq->cryptlen + areq->assoclen;
	unsigned int nents = sg_nents(areq->dst);
	uint32_t buf[4], i;

	sifive_hca_aes_get_auth(hca_dev, buf);

	for (i = 0; i < 4; i++)
                buf[i] = cpu_to_be32(buf[i]);

	sg_pcopy_from_buffer(areq->dst, nents, (u8 *)buf, 16, skip);
}

static void sifive_hca_aead_std_crypt(struct crypto_async_request *req)
{
	struct aead_request *areq = aead_request_cast(req);
        struct sifive_hca_op_ctx *tmpl = aead_request_ctx(areq);
        unsigned int offset = 0;
        uint8_t buf[64];
        size_t len;

	sifive_hca_aes_set_dtype(hca_dev, HCA_AES_DTYPE_AAD);
	while (offset < areq->assoclen)
	{
		len = min_t(size_t, areq->assoclen - offset,
			    HCA_AES_FIFO_SIZE);

		len = sg_pcopy_to_buffer(tmpl->aad_sg, tmpl->aad_nents, buf,
					 len, offset);

		if (sifive_hca_ififo_is_full(hca_dev)) {
			dev_err(hca_dev->dev, "IFIFO is full\n");
			break;
		} else {
			sifive_hca_aes_fifo_push(hca_dev, buf, len);
		}

		offset += len;
		while (!(sifvie_hca_ififo_is_empty(hca_dev)));
	}

	offset = 0;
	sifive_hca_aes_set_dtype(hca_dev, HCA_AES_DTYPE_PLD);
	reinit_completion(&hca_dev->aes_completion);
	while (offset < tmpl->dlen)
	{
		len = min_t(size_t, tmpl->dlen - offset, 32);

		len = sg_pcopy_to_buffer(tmpl->src_sg, tmpl->src_nents, buf,
				len, offset);


		if (sifive_hca_ififo_is_full(hca_dev)) {
			dev_err(hca_dev->dev, "IFIFO is full\n");
			break;
		} else {
			sifive_hca_aes_fifo_push(hca_dev, buf, len);
		}

		while (!(sifvie_hca_ififo_is_empty(hca_dev)));

		if (sifive_hca_ofifo_is_empty(hca_dev)) {
			dev_err(hca_dev->dev, "OFIFO is empty\n");
			break;
		} else {
			sifive_hca_aes_fifo_pop(hca_dev, buf, len);
		}

		len = sg_pcopy_from_buffer(tmpl->dst_sg, tmpl->dst_nents, buf,
				len, offset);
		offset += len;
	}

	if (sifive_hca_crypto_is_int_enable(hca_dev))
		wait_for_completion(&hca_dev->aes_completion);
	else
		sifive_hca_poll_aes_timeout(hca_dev, HCA_AES_TIMEOUT);

	if (tmpl->aes_process_type == HCA_AES_DIR_ENC)
		sifive_hca_aead_process_crypt(req);
}

static void sifive_hca_aead_dma_crypt(struct crypto_async_request *req)
{
	struct aead_request *areq = aead_request_cast(req);
	struct sifive_hca_op_ctx *tmpl = aead_request_ctx(areq);
	struct scatterlist *sg_src = tmpl->src_sg, *sg_dst = tmpl->dst_sg;
	struct scatterlist *sg_aad = tmpl->aad_sg;
	dma_addr_t src, dst;
	u32 len, copied = 0;

	sifive_hca_aes_set_dtype(hca_dev, HCA_AES_DTYPE_AAD);
	while (sg_aad != NULL && copied < areq->assoclen) {
		src = sg_dma_address(sg_aad);
		len = sg_dma_len(sg_aad);

		reinit_completion(&hca_dev->dma_completion);
		sifive_hca_dma_transfer(hca_dev, src, 0, len);

		if (sifive_hca_dma_is_int_enable(hca_dev))
			wait_for_completion(&hca_dev->dma_completion);
		else
			sifive_hca_poll_dma_timeout(hca_dev, HCA_AES_TIMEOUT);

		sg_aad = sg_next(sg_aad);
		copied += len;
	}

	reinit_completion(&hca_dev->aes_completion);
	sifive_hca_aes_set_dtype(hca_dev, HCA_AES_DTYPE_PLD);
	while (sg_src != NULL && sg_dst != NULL) {
		src = sg_dma_address(sg_src);
		dst = sg_dma_address(sg_dst);
		len = sg_dma_len(sg_src);

		reinit_completion(&hca_dev->dma_completion);
		sifive_hca_dma_transfer(hca_dev, src, dst, len);

		if (sifive_hca_dma_is_int_enable(hca_dev))
			wait_for_completion(&hca_dev->dma_completion);
		else
			sifive_hca_poll_dma_timeout(hca_dev, HCA_AES_TIMEOUT);

		sg_src = sg_next(sg_src);
		sg_dst = sg_next(sg_dst);
	}

	if (sifive_hca_crypto_is_int_enable(hca_dev))
		wait_for_completion(&hca_dev->aes_completion);
	else
		sifive_hca_poll_aes_timeout(hca_dev, HCA_AES_TIMEOUT);

	if (tmpl->aes_process_type == HCA_AES_DIR_ENC)
		sifive_hca_aead_process_crypt(req);
}

static void sifive_hca_aead_crypt(struct crypto_async_request *req)
{
	struct aead_request *areq = aead_request_cast(req);
	struct sifive_hca_op_ctx *tmpl = aead_request_ctx(areq);
	int val, i;

	sifive_hca_fifo_target_aes(hca_dev);
	sifive_hca_fifo_set_be(hca_dev);

	sifive_hca_aes_set_mode(hca_dev, tmpl->aes_mode);
	sifive_hca_aes_set_keysize(hca_dev, tmpl->aes_key_size);
	sifive_hca_aes_set_process_type(hca_dev, tmpl->aes_process_type);

	sifive_hca_aes_set_key(hca_dev, tmpl->aes_key_size, &tmpl->aes_key[0]);

	for (i = 0; i < 4; i++)
                tmpl->aes_initv[i] = cpu_to_be32(tmpl->aes_initv[i]);

	sifive_hca_aes_set_initv(hca_dev, AEAD_INITV_SIZE,
				 &tmpl->aes_initv[0]);

	sifive_hca_aes_set_alen(hca_dev, areq->assoclen);
	sifive_hca_aes_set_pldlen(hca_dev, tmpl->dlen);

	if (tmpl->aes_mode == HCA_AES_CR_MODE_CCM) {
		sifive_hca_aes_set_ccmq(hca_dev, tmpl->dlen);
		val = crypto_aead_authsize(crypto_aead_reqtfm(areq));
		sifive_hca_aes_set_ccmt(hca_dev, val);
	}

	if (hca_dev->algs->has_dma) {
		if (sifive_hca_dma_crypt_check(tmpl->src_sg, tmpl->dst_sg,
					       tmpl->aad_sg, areq->assoclen)) {
			sifive_hca_aead_dma_crypt(req);
			return;
		}
		sifive_hca_aead_dma_cleanup(areq);
	}

	sifive_hca_aead_std_crypt(req);
}

static inline void
sifive_hca_aead_req_cleanup(struct crypto_async_request *req)
{
	struct aead_request *areq = aead_request_cast(req);

	sifive_hca_aead_cleanup(areq);
}

static const struct sifive_hca_req_ops sifive_hca_aead_req_ops = {
	.crypt = sifive_hca_aead_crypt,
	.cleanup = sifive_hca_aead_req_cleanup,
};

static void sifive_hca_aead_cra_exit(struct crypto_tfm *tfm)
{
	void *ctx = crypto_tfm_ctx(tfm);

	memzero_explicit(ctx, tfm->__crt_alg->cra_ctxsize);
}

static int sifive_hca_aead_cra_init(struct crypto_tfm *tfm)
{
	struct sifive_hca_ctx *ctx = crypto_tfm_ctx(tfm);

	ctx->ops = &sifive_hca_aead_req_ops;

	crypto_aead_set_reqsize(__crypto_aead_cast(tfm),
			sizeof(struct sifive_hca_op_ctx));

	return 0;
}

static int sifive_hca_aes_gcm_setauthsize(struct crypto_aead *tfm,
                                          unsigned int authsize)
{
        return authsize == AES_BLOCK_SIZE ? 0 : -EINVAL;
}

static int sifive_hca_aes_aead_setkey(struct crypto_aead *cipher, const u8 *key,
		unsigned int len)
{
	struct crypto_tfm *tfm = crypto_aead_tfm(cipher);
	struct sifive_hca_aes_ctx *ctx = crypto_tfm_ctx(tfm);

	memcpy(ctx->aes.key_enc, key, len);
	ctx->aes.key_length = len;

	return 0;
}

static int sifive_hca_aead_dma_req_init(struct aead_request *areq,
		const struct sifive_hca_op_ctx *tmpl)
{
	int ret;

	if (tmpl->aad_sg != NULL) {
		ret = dma_map_sg(hca_dev->dev, tmpl->aad_sg, tmpl->aad_nents,
				 DMA_TO_DEVICE);
		if (!ret)
			return -ENOMEM;
	}

	if (areq->src != areq->dst) {
		ret = dma_map_sg(hca_dev->dev, tmpl->src_sg, tmpl->src_nents,
				DMA_TO_DEVICE);
		if (!ret) {
			ret = -ENOMEM;
			goto err_unmap_aad;
		}

		ret = dma_map_sg(hca_dev->dev, tmpl->dst_sg, tmpl->dst_nents,
				DMA_FROM_DEVICE);
		if (!ret) {
			ret = -ENOMEM;
			goto err_unmap_src;
		}
	} else {
		ret = dma_map_sg(hca_dev->dev, tmpl->src_sg, tmpl->src_nents,
				DMA_BIDIRECTIONAL);
		if (!ret) {
			ret = -ENOMEM;
			goto err_unmap_aad;
		}
	}
	return 0;

err_unmap_src:
	dma_unmap_sg(hca_dev->dev, tmpl->src_sg, tmpl->src_nents,
			areq->dst != areq->src ? DMA_TO_DEVICE : DMA_BIDIRECTIONAL);
err_unmap_aad:
	dma_unmap_sg(hca_dev->dev, tmpl->aad_sg, tmpl->aad_nents,
			DMA_TO_DEVICE);

	return ret;
}

static int sifive_hca_aead_req_init(struct aead_request *areq,
				    struct sifive_hca_op_ctx *tmpl)
{
	struct crypto_aead *tfm = crypto_aead_reqtfm(areq);
	unsigned int blksize = crypto_aead_blocksize(tfm);
	unsigned int src_skip, dst_skip;
	int src_nents, dst_nents, authsize, ret = 0;

	if (!IS_ALIGNED(areq->cryptlen, blksize))
		return -EINVAL;

	authsize = crypto_aead_authsize(crypto_aead_reqtfm(areq));

	if (tmpl->aes_process_type == HCA_AES_DIR_DEC)
		tmpl->dlen = areq->cryptlen - authsize;
	else
		tmpl->dlen = areq->cryptlen;

	if (areq->assoclen != 0)
		tmpl->aad_sg = areq->src;
	else
		tmpl->aad_sg = NULL;

	tmpl->aad_nents = sg_nents_for_len(tmpl->aad_sg, areq->assoclen);

	sifive_hca_sg_at_offset(areq->src, areq->assoclen, &tmpl->src_sg, &src_skip);
	src_nents = sifive_hca_sg_count(tmpl->src_sg, src_skip, tmpl->dlen);

	tmpl->src_nents = src_nents;
	if (tmpl->src_nents < 0) {
		dev_err(hca_dev->dev, "Invalid number of src SG");
		return tmpl->src_nents;
	}

	sifive_hca_sg_at_offset(areq->dst, areq->assoclen, &tmpl->dst_sg, &dst_skip);
	dst_nents = sifive_hca_sg_count(tmpl->dst_sg, dst_skip, tmpl->dlen);

	tmpl->dst_nents = dst_nents;
	if (tmpl->dst_nents < 0) {
		dev_err(hca_dev->dev, "Invalid number of dst SG");
		return tmpl->dst_nents;
	}

	if (hca_dev->algs->has_dma)
		ret = sifive_hca_aead_dma_req_init(areq, tmpl);

	return ret;
}

static int sifive_hca_aead_queue_req(struct aead_request *areq,
		struct sifive_hca_op_ctx *tmpl)
{
	int ret;

	ret = sifive_hca_aead_req_init(areq, tmpl);
	if (ret)
		return ret;

	ret = sifive_hca_queue_req(&areq->base);

	if (sifive_hca_req_needs_cleanup(&areq->base, ret))
		sifive_hca_aead_cleanup(areq);

	return ret;
}

static int sifive_hca_aes_op(struct aead_request *areq, struct sifive_hca_op_ctx *tmpl)
{
	struct sifive_hca_aes_ctx *ctx = crypto_tfm_ctx(areq->base.tfm);
	int i;
	u32 *key;

	key = ctx->aes.key_enc;

	for (i = 0; i < ctx->aes.key_length / sizeof(u32); i++)
		tmpl->aes_key[i] = cpu_to_be32(key[i]);

	tmpl->aes_key_size = ctx->aes.key_length;

	return sifive_hca_aead_queue_req(areq, tmpl);
}

static int sifive_hca_aes_gcm_encrypt(struct aead_request *areq)
{
	struct sifive_hca_op_ctx *tmpl = aead_request_ctx(areq);

	tmpl->aes_process_type = HCA_AES_DIR_ENC;
	tmpl->aes_mode = HCA_AES_CR_MODE_GCM;

	memcpy(tmpl->aes_initv, areq->iv, AEAD_INITV_SIZE);

	return sifive_hca_aes_op(areq, tmpl);
}

static int sifive_hca_aes_gcm_decrypt(struct aead_request *areq)
{
	struct sifive_hca_op_ctx *tmpl = aead_request_ctx(areq);

	tmpl->aes_process_type = HCA_AES_DIR_DEC;
	tmpl->aes_mode = HCA_AES_CR_MODE_GCM;

	memcpy(tmpl->aes_initv, areq->iv, AEAD_INITV_SIZE);

	return sifive_hca_aes_op(areq, tmpl);
}

struct aead_alg sifive_hca_gcm_aes_alg = {
	.setkey         = sifive_hca_aes_aead_setkey,
	.setauthsize    = sifive_hca_aes_gcm_setauthsize,
	.encrypt        = sifive_hca_aes_gcm_encrypt,
	.decrypt        = sifive_hca_aes_gcm_decrypt,
	.ivsize         = AEAD_INITV_SIZE,
	.maxauthsize    = AES_BLOCK_SIZE,
	.base = {
		.cra_name               = "gcm(aes)",
		.cra_driver_name        = "sifive-gcm-aes",
		.cra_priority           = 400,
		.cra_flags              = CRYPTO_ALG_ASYNC,
		.cra_blocksize          = 1,
		.cra_ctxsize            = sizeof(struct sifive_hca_aes_ctx),
		.cra_alignmask          = 0x1f,
		.cra_module             = THIS_MODULE,
		.cra_init = sifive_hca_aead_cra_init,
		.cra_exit = sifive_hca_aead_cra_exit,
	},
};

static int sifive_hca_aes_ccm_encrypt(struct aead_request *areq)
{
	struct sifive_hca_op_ctx *tmpl = aead_request_ctx(areq);

	tmpl->aes_process_type = HCA_AES_DIR_ENC;
	tmpl->aes_mode = HCA_AES_CR_MODE_CCM;

	memcpy(tmpl->aes_initv, areq->iv, AEAD_INITV_SIZE);

	return sifive_hca_aes_op(areq, tmpl);
}

static int sifive_hca_aes_ccm_decrypt(struct aead_request *areq)
{
	struct sifive_hca_op_ctx *tmpl = aead_request_ctx(areq);

	tmpl->aes_process_type = HCA_AES_DIR_DEC;
	tmpl->aes_mode = HCA_AES_CR_MODE_CCM;

	memcpy(tmpl->aes_initv, areq->iv, AEAD_INITV_SIZE);

	return sifive_hca_aes_op(areq, tmpl);
}

static int sifive_hca_aes_ccm_setauthsize(struct crypto_aead *tfm,
                                          unsigned int authsize)
{
	switch (authsize) {
		case 4:
		case 6:
		case 8:
		case 10:
		case 12:
		case 14:
		case 16:
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

struct aead_alg sifive_hca_ccm_aes_alg = {
	.setkey         = sifive_hca_aes_aead_setkey,
	.setauthsize    = sifive_hca_aes_ccm_setauthsize,
	.encrypt        = sifive_hca_aes_ccm_encrypt,
	.decrypt        = sifive_hca_aes_ccm_decrypt,
	.ivsize         = AEAD_INITV_SIZE,
	.maxauthsize    = AES_BLOCK_SIZE,
	.base = {
		.cra_name               = "ccm(aes)",
		.cra_driver_name        = "sifive-ccm-aes",
		.cra_priority           = 400,
		.cra_flags              = CRYPTO_ALG_ASYNC,
		.cra_blocksize          = 1,
		.cra_ctxsize            = sizeof(struct sifive_hca_aes_ctx),
		.cra_alignmask          = 0x1f,
		.cra_module             = THIS_MODULE,
		.cra_init = sifive_hca_aead_cra_init,
		.cra_exit = sifive_hca_aead_cra_exit,
	},
};

static struct aead_alg *general_hca_aead_algs[] = {
        &sifive_hca_gcm_aes_alg,
        &sifive_hca_ccm_aes_alg,
};

int sifive_aead_algs_register(struct sifive_hca_dev *hca)
{
        return crypto_register_aeads(general_hca_aead_algs[0],
				     ARRAY_SIZE(general_hca_aead_algs));
}

void sifive_aead_algs_unregister(struct sifive_hca_dev *hca)
{
        crypto_unregister_aeads(general_hca_aead_algs[0],
				ARRAY_SIZE(general_hca_aead_algs));
}
