// SPDX-License-Identifier: GPL-2.0-only
/*
 * SiFive Hardware Cryptographic Accelerator (HCA)
 *
 * Copyright (C) 2020 SiFive Inc
 */

#include <crypto/sha.h>

#include "hca.h"

struct sifive_sha_ctx {
	int i;
};

static int sifive_hca_ahash_cra_init(struct crypto_tfm *tfm)
{
	return 0;
}

static void sifive_hca_ahash_cra_exit(struct crypto_tfm *tfm)
{
	__asm__ ("");
}

static int sifive_hca_sha2_init(struct ahash_request *req)
{
	return 0;
}

static int sifive_hca_ahash_update(struct ahash_request *req)
{
	return 0;
}

static int sifive_hca_ahash_final(struct ahash_request *req)
{
	return 0;
}

static int sifive_hca_ahash_finup(struct ahash_request *req)
{
	return 0;
}

static int sifive_hca_sha2_disgest(struct ahash_request *req)
{
	return 0;
}

static int sifive_hca_sha2_export(struct ahash_request *req, void *out)
{
	return 0;
}

static int sifive_hca_sha2_import(struct ahash_request *req, const void *in)
{
	return 0;
}
#if 0
struct ahash_alg sifive_hca_sha224_alg = {

};

struct ahash_alg sifive_hca_sha256_alg = {

};

struct ahash_alg sifive_hca_sha384_alg = {

};
#endif
struct ahash_alg sifive_hca_sha512_alg = {
	.init = sifive_hca_sha2_init,
	.update = sifive_hca_ahash_update,
	.final = sifive_hca_ahash_final,
	.finup = sifive_hca_ahash_finup,
	.digest = sifive_hca_sha2_disgest,
	.export = sifive_hca_sha2_export,
	.import = sifive_hca_sha2_import,
	.halg = {
		.digestsize = SHA512_DIGEST_SIZE,
		.statesize = sizeof(struct sha512_state),
		.base = {
			.cra_name = "sha512",
			.cra_driver_name = "sifive-sha512",
			.cra_priority = 400,
			.cra_flags = CRYPTO_ALG_ASYNC |
				     CRYPTO_ALG_ALLOCATES_MEMORY |
				     CRYPTO_ALG_KERN_DRIVER_ONLY,
			.cra_blocksize = SHA512_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct sifive_sha_ctx),
//			.cra_alignmask = SHA_ALIGN_MSK,
			.cra_module = THIS_MODULE,
			.cra_init = sifive_hca_ahash_cra_init,
			.cra_exit = sifive_hca_ahash_cra_exit,
		}
	}
};

struct ahash_alg *hca_ahash_algs[] = {
//	&sifive_hca_sha224_alg,
//	&sifive_hca_sha256_alg,
//	&sifive_hca_sha384_alg,
	&sifive_hca_sha512_alg,
};

/* Register algs to crypto framework */
int sifive_ahash_algs_register(struct sifive_hca_dev *hca)
{
	return crypto_register_ahashes(hca->algs->ahash_algs,
				       ARRAY_SIZE(hca->algs->ahash_algs));
}

void sifive_ahash_algs_unregister(struct sifive_hca_dev *hca)
{
	crypto_unregister_ahashes(hca->algs->ahash_algs,
				  ARRAY_SIZE(hca->algs->ahash_algs));
}
