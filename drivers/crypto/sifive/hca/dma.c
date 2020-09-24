// SPDX-License-Identifier: GPL-2.0-only
/*
 * SiFive Hardware Cryptographic Accelerator (HCA)
 *
 * Copyright (C) 2020 SiFive Inc
 */

#include "hca.h"

int sifive_hca_dma_int_handle(struct sifive_hca_dev *hca)
{
	sifive_hca_dma_int_ack(hca);

	return 0;
}

int sifive_hca_dma_is_complete(struct sifive_hca_dev *hca)
{
	while (sifive_hca_dma_is_busy(hca));

	return 0;
}

void sifive_hca_dma_transfer(struct sifive_hca_dev *hca, uint32_t src,
			     uint32_t dest, uint32_t len)
{
	/* SHA digest will be stored in HASH register, not DMA_DEST */
	if (!dest)
		sifive_hca_dma_set_dest(hca, dest);

	sifive_hca_dma_set_src(hca, src);
	sifive_hca_dma_set_length(hca, len);

	/* Check input FIFO is not full before wrting more data */
	while (sifive_hca_ififo_is_full(hca));

	sifive_hca_dma_start(hca);
}

/* HCA embedded DMA initialization */
int sifive_hca_dma_init(struct sifive_hca_dev *hca)
{
	sifive_hca_dma_int_enable(hca);

	return 0;
}

int sifive_hca_dma_free(struct sifive_hca_dev *hca)
{
	sifive_hca_dma_int_disable(hca);

	return 0;
}

