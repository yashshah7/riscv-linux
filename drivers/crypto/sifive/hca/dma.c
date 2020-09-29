// SPDX-License-Identifier: GPL-2.0-only
/*
 * SiFive Hardware Cryptographic Accelerator (HCA)
 *
 * Copyright (C) 2020 SiFive Inc
 */

#include "hca.h"

#define DMA_ALIGNMENT	32u	/* FIFO size 32 bytes */
#define DMA_BLOCK_SIZE	16u	/* DMA granularity is 128-bits block */

int sifive_hca_dma_int_handle(struct sifive_hca_dev *hca)
{
	struct *dev = hca->dev;

	complete(&dev->dma_completion);
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

	/* FIXME: May be we dont need these priliminary checks since it is
	 * indicated below by reading the regs after starting the dma operation
	 */
	if (((uintptr_t)src) & ((DMA_ALIGNMENT) - 1u)) {
		dev_err(dev, "Source is not aligned on a DMA boundary\n");
		return -EINVAL;
	}
	if (((uintptr_t)dest) & ((DMA_ALIGNMENT) - 1u)) {
		dev_err(dev, "Destination is not aligned on a DMA boundary\n");
		return -EINVAL;
	}
	if (((uintptr_t)len) & ((DMA_BLOCK_SIZE) - 1u)) {
		dev_err(dev, "Length is not aligned on a DMA block size\n");
		return -EINVAL;
	}

	if (sifive_hca_dma_is_busy(hca)) {
		dev_err(dev, "DMA HW is busy\n");
		return -EBUSY;
	}

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
	struct *dev = hca->dev;

	sifive_hca_dma_int_enable(hca);
	init_completion(&dev->dma_completion);

	return 0;
}

int sifive_hca_dma_free(struct sifive_hca_dev *hca)
{
	sifive_hca_dma_int_disable(hca);

	return 0;
}

