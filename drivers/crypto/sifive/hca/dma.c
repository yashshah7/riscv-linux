// SPDX-License-Identifier: GPL-2.0-only
/*
 * SiFive Hardware Cryptographic Accelerator (HCA)
 *
 * Copyright (C) 2020 SiFive Inc
 */

#include "hca.h"

#define DMA_ALIGNMENT	32u	/* FIFO size 32 bytes */
#define DMA_BLOCK_SIZE	16u	/* DMA granularity is 128-bits block */

bool sifive_hca_dma_crypt_check(struct scatterlist *sg_src,
				struct scatterlist *sg_dst,
				struct scatterlist *sg_aad,
				u32 assoclen)
{
	dma_addr_t src, dst;
	u32 s_len, d_len, total = 0;

	while (sg_aad != NULL && total < assoclen)
	{
		src = sg_dma_address(sg_aad);
		s_len = sg_dma_len(sg_aad);

		if (((uintptr_t)src) & ((DMA_ALIGNMENT) - 1u))
			return false;
		if (((uintptr_t)s_len) & ((DMA_BLOCK_SIZE) - 1u))
			return -EINVAL;

		total += s_len;
		sg_aad = sg_next(sg_aad);
	}

	while (sg_src != NULL && sg_dst != NULL)
	{
		src = sg_dma_address(sg_src);
		dst = sg_dma_address(sg_dst);
		s_len = sg_dma_len(sg_src);
		d_len = sg_dma_len(sg_dst);

		if (s_len != d_len)
			return false;
		if (((uintptr_t)src) & ((DMA_ALIGNMENT) - 1u))
			return false;
		if (((uintptr_t)dst) & ((DMA_ALIGNMENT) - 1u))
			return false;
		if (((uintptr_t)s_len) & ((DMA_BLOCK_SIZE) - 1u))
			return -EINVAL;

		sg_src = sg_next(sg_src);
		sg_dst = sg_next(sg_dst);
	}

	return true;
}


int sifive_hca_dma_int_handle(struct sifive_hca_dev *hca)
{
	complete(&hca->dma_completion);
	sifive_hca_dma_int_ack(hca);

	return 0;
}

int sifive_hca_dma_poll_for_complete(struct sifive_hca_dev *hca)
{
	while (sifive_hca_dma_is_busy(hca));

	return 0;
}

int sifive_hca_dma_transfer(struct sifive_hca_dev *hca, uint32_t src,
			    uint32_t dest, uint32_t len)
{
	struct device *dev = hca->dev;

	if (sifive_hca_dma_is_busy(hca)) {
		dev_err(dev, "DMA HW is busy\n");
		return -EBUSY;
	}

	/*
	 * FIXME: In the current release, the input FIFO doesn't block the bus
	 * when full. It is the user responsability to check if the Input FIFO
	 * is not full before writing more data with the help of CR.IFIFOFULL
	 * and CR.IFIFOCNT.
	 */

	/* SHA digest will be stored in HASH register, not DMA_DEST */
	if (dest)
		sifive_hca_dma_set_dest(hca, dest);
	sifive_hca_dma_set_src(hca, src);
	sifive_hca_dma_set_length(hca, (len/DMA_BLOCK_SIZE));

	sifive_hca_dma_start(hca);

	if (sifive_hca_dma_is_bus_error(hca)) {
		dev_err(dev, "Bus error occured during DMA operation\n");
		return -EIO;
	}
	if (sifive_hca_dma_is_rw_illegal(hca)) {
		dev_err(dev, "Illegal access at DMA address\n");
		return -EACCES;
	}
	if (sifive_hca_dma_is_src_unaligned(hca)) {
		dev_err(dev, "Source is not aligned on a DMA boundary\n");
		return -EINVAL;
	}
	if (sifive_hca_dma_is_dest_unaligned(hca)) {
		dev_err(dev, "Destination is not aligned on a DMA boundary\n");
		return -EINVAL;
	}

	return 0;
}

/* HCA embedded DMA initialization */
int sifive_hca_dma_init(struct sifive_hca_dev *hca)
{
	sifive_hca_dma_int_enable(hca);
	sifive_hca_dma_set_src(hca, 0);
	sifive_hca_dma_set_dest(hca, 0);
	sifive_hca_dma_set_length(hca, 0);
	init_completion(&hca->dma_completion);

	return 0;
}

int sifive_hca_dma_free(struct sifive_hca_dev *hca)
{
	sifive_hca_dma_int_disable(hca);
	sifive_hca_dma_set_src(hca, 0);
	sifive_hca_dma_set_dest(hca, 0);
	sifive_hca_dma_set_length(hca, 0);

	return 0;
}

