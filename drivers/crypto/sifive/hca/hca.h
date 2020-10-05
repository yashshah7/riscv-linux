/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _ASM_SIFIVE_HCA_H
#define _ASM_SIFIVE_HCA_H

#include <crypto/hash.h>
#include <crypto/internal/hash.h>
#include <crypto/internal/skcipher.h>

#define HCA_CR			0x00
#define HCA_CR_IFIFOTGT		BIT(0)
#define HCA_CR_IFIFOTGT_AES	0
#define HCA_CR_IFIFOTGT_SHA	1
#define HCA_CR_IFIFOEMPTY	BIT(1)
#define HCA_CR_IFIFOFULL	BIT(2)
#define HCA_CR_OFIFOEMPTY	BIT(3)
#define HCA_CR_OFIFOFULL	BIT(4)
#define HCA_CR_ENDIANNES	BIT(5)
#define HCA_CR_ENDIANNES_LE	0
#define HCA_CR_ENDIANNES_BE	1
#define HCA_CR_CRYPTODIE	BIT(8)
#define HCA_CR_OFIFOIE		BIT(9)
#define HCA_CR_DMADIE		BIT(10)
#define HCA_CR_CRYPTODIS	BIT(12)
#define HCA_CR_OFIFOIS		BIT(13)
#define HCA_CR_DMADIS		BIT(14)
#define HCA_CR_IFIFOCNT		GENMASK(21, 16)
#define HCA_CR_IFIFOCNT_OFFSET	16
#define HCA_CR_OFIFOCNT		GENMASK(29, 24)
#define HCA_CR_OFIFOCNT_OFFSET	24

#define HCA_AES_CR		0x10
#define HCA_AES_CR_MODE		GENMASK(2, 0)
#define HCA_AES_CR_MODE_ECB	0x0
#define HCA_AES_CR_MODE_CBC	0x1
#define HCA_AES_CR_MODE_CFB	0x2
#define HCA_AES_CR_MODE_OFB	0x3
#define HCA_AES_CR_MODE_CTR	0x4
#define HCA_AES_CR_MODE_GCM	0x5
#define HCA_AES_CR_MODE_CCM	0x6
#define HCA_AES_CR_KEYSZ	GENMASK(5, 4)
#define HCA_AES_CR_KEYSZ_128	0x0
#define HCA_AES_CR_KEYSZ_192	0x1
#define HCA_AES_CR_KEYSZ_256	0x2
#define HCA_AES_CR_PROCESS	BIT(6)
#define HCA_AES_CR_INIT		BIT(7)
#define HCA_AES_CR_DTYPE	BIT(8)
#define HCA_AES_CR_CCMT		GENMASK(11, 9)
#define HCA_AES_CR_CCMT_INVALID	0x0
#define HCA_AES_CR_CCMT_4	0x1
#define HCA_AES_CR_CCMT_6	0x2
#define HCA_AES_CR_CCMT_8	0x3
#define HCA_AES_CR_CCMT_10	0x4
#define HCA_AES_CR_CCMT_12	0x5
#define HCA_AES_CR_CCMT_14	0x6
#define HCA_AES_CR_CCMT_16	0x7
#define HCA_AES_CR_CCMQ		BIT(12)
#define HCA_AES_CR_CCMQ_INVALID	0x0
#define HCA_AES_CR_CCMQ_2	0x1
#define HCA_AES_CR_CCMQ_3	0x2
#define HCA_AES_CR_CCMQ_4	0x3
#define HCA_AES_CR_CCMQ_5	0x4
#define HCA_AES_CR_CCMQ_6	0x5
#define HCA_AES_CR_CCMQ_7	0x6
#define HCA_AES_CR_CCMQ_8	0x7
#define HCA_AES_CR_BUSY		BIT(16)

#define HCA_AES_ALEN		0x20
#define HCA_AES_ALEN_MASK	GENMASK(60, 0)

#define HCA_AES_PLDLEN		0x28
#define HCA_AES_KEY		0x30
#define HCA_AES_INITV		0x50

#define HCA_SHA_CR		0x60
#define HCA_SHA_MODE		GENMASK(1, 0)
#define HCA_SHA_MODE_224	0x0
#define HCA_SHA_MODE_256	0x1
#define HCA_SHA_MODE_384	0x2
#define HCA_SHA_MODE_512	0x3
#define HCA_SHA_INIT		BIT(2)
#define HCA_SHA_BUSY		BIT(16)

#define HCA_FIFO_IN		0x70
#define HCA_AES_OUT		0x80
#define HCA_AES_AUTH		0x90
#define HCA_HASH		0xA0

#define HCA_TRNG_CR		0xE0
#define HCA_TRNG_SR		0xE4
#define HCA_TRNG_DATA		0xE8
#define HCA_TRNG_TRIM		0xEC

#define HCA_DMA_CR		0x110
#define HCA_DMA_CR_START	BIT(0)
#define HCA_DMA_CR_BUSY		BIT(8)
#define HCA_DMA_CR_RDALIGNERR	BIT(9)
#define HCA_DMA_CR_WRALIGNERR	BIT(10)
#define HCA_DMA_CR_RESPERR	BIT(11)
#define HCA_DMA_CR_LEGALERR	BIT(12)

#define HCA_DMA_LEN		0x114
#define HCA_DMA_LEN_MASK	GENMASK(27, 0)
#define HCA_DMA_SRC		0x118
#define HCA_DMA_DEST		0x120

#define HCA_REV			0x200
#define HCA_REV_PATCHREV	GENMASK(3, 0)
#define HCA_REV_MINORREV	GENMASK(7, 4)
#define HCA_REV_MAJORREV	GENMASK(11, 8)

#define HCA_AES_REV		0x204
#define HCA_AES_REV_PATCHREV	GENMASK(3, 0)
#define HCA_AES_REV_MINORREV	GENMASK(7, 4)
#define HCA_AES_REV_MAJORREV	GENMASK(11, 8)
#define HCA_AES_REV_TYPE	GENMASK(15, 12)

#define HCA_SHA_REV		0x208
#define HCA_SHA_REV_PATCHREV	GENMASK(3, 0)
#define HCA_SHA_REV_MINORREV	GENMASK(7, 4)
#define HCA_SHA_REV_MAJORREV	GENMASK(11, 8)
#define HCA_SHA_REV_TYPE	GENMASK(15, 12)

#define HCA_TRNG_REV		0x20C
#define HCA_TRNG_REV_PATCHREV	GENMASK(3, 0)
#define HCA_TRNG_REV_MINORREV	GENMASK(7, 4)
#define HCA_TRNG_REV_MAJORREV	GENMASK(11, 8)
#define HCA_TRNG_REV_TYPE	GENMASK(15, 12)

struct sifive_hca_algs {
	struct skcipher_alg **cipher_algs;
	int ncipher_algs;
	struct ahash_alg **ahash_algs;
	int nahash_algs;
	bool has_dma;
};

struct sifive_hca_dev {
	const struct sifive_hca_algs *algs;
	void __iomem *regs;
	struct device *dev;
	struct completion dma_completion;
	unsigned int irq;
	spinlock_t lock;
};

/* HCA controls  */
static inline void _hca_writel(struct sifive_hca_dev *hca, uint32_t offset, uint32_t data, int set)
{
	spin_lock(&hca->lock);
	if (set)
		writel(readl(hca->regs + offset) | data, hca->regs + offset);
	else
		writel(readl(hca->regs + offset) & ~data, hca->regs + offset);
	spin_unlock(&hca->lock);
}

static inline void sifive_hca_fifo_target_aes(struct sifive_hca_dev *hca)
{
	_hca_writel(hca, HCA_CR, HCA_CR_IFIFOTGT, 0);
}

static inline void sifive_hca_fifo_target_sha(struct sifive_hca_dev *hca)
{
	_hca_writel(hca, HCA_CR, HCA_CR_IFIFOTGT, 1);
}

static inline int sifvie_hca_ififo_is_empty(struct sifive_hca_dev *hca)
{
	return readl(hca->regs + HCA_CR) & HCA_CR_IFIFOEMPTY;
}

static inline int sifive_hca_ififo_is_full(struct sifive_hca_dev *hca)
{
	return readl(hca->regs + HCA_CR) & HCA_CR_IFIFOFULL;
}

static inline int sifive_hca_ofifo_is_empty(struct sifive_hca_dev *hca)
{
	return readl(hca->regs + HCA_CR) & HCA_CR_OFIFOEMPTY;
}

static inline int sifive_hca_ofifo_is_full(struct sifive_hca_dev *hca)
{
	return readl(hca->regs + HCA_CR) & HCA_CR_OFIFOFULL;
}

static inline void sifive_hca_fifo_set_be(struct sifive_hca_dev *hca)
{
	_hca_writel(hca, HCA_CR, HCA_CR_ENDIANNES, 1);
}

static inline void sifive_hca_fifo_set_le(struct sifive_hca_dev *hca)
{
	_hca_writel(hca, HCA_CR, HCA_CR_ENDIANNES, 0);
}

static inline void sifive_hca_crypto_int_enable(struct sifive_hca_dev *hca)
{
	_hca_writel(hca, HCA_CR, HCA_CR_CRYPTODIE, 1);
}

static inline void sifive_hca_crypto_int_disable(struct sifive_hca_dev *hca)
{
	_hca_writel(hca, HCA_CR, HCA_CR_CRYPTODIE, 0);
}

static inline void sifive_hca_crypto_int_ack(struct sifive_hca_dev *hca)
{
	_hca_writel(hca, HCA_CR, HCA_CR_CRYPTODIS, 1);
}

static inline void sifive_hca_ofifo_int_enable(struct sifive_hca_dev *hca)
{
	_hca_writel(hca, HCA_CR, HCA_CR_OFIFOIE, 1);
}

static inline void sifive_hca_ofifo_int_disable(struct sifive_hca_dev *hca)
{
	_hca_writel(hca, HCA_CR, HCA_CR_OFIFOIE, 0);
}

static inline void sifive_hca_ofifo_int_ack(struct sifive_hca_dev *hca)
{
	_hca_writel(hca, HCA_CR, HCA_CR_OFIFOIS, 1);
}

static inline void sifive_hca_dma_int_enable(struct sifive_hca_dev *hca)
{
	_hca_writel(hca, HCA_CR, HCA_CR_DMADIE, 1);
}

static inline void sifive_hca_dma_int_disable(struct sifive_hca_dev *hca)
{
	_hca_writel(hca, HCA_CR, HCA_CR_DMADIE, 0);
}

static inline void sifive_hca_dma_int_ack(struct sifive_hca_dev *hca)
{
	_hca_writel(hca, HCA_CR, HCA_CR_DMADIS, 1);
}

static inline uint8_t sifive_hca_num_ififo(struct sifive_hca_dev *hca)
{
	return ((readl(hca->regs + HCA_CR) & HCA_CR_IFIFOCNT) >> HCA_CR_IFIFOCNT_OFFSET);
}

static inline uint8_t sifive_hca_num_ofifo(struct sifive_hca_dev *hca)
{
	return ((readl(hca->regs + HCA_CR) & HCA_CR_OFIFOCNT) >> HCA_CR_OFIFOCNT_OFFSET);
}

static inline void sifive_hca_dma_start(struct sifive_hca_dev *hca)
{
	_hca_writel(hca, HCA_DMA_CR, HCA_DMA_CR_START, 1);
}

static inline int sifive_hca_dma_is_busy(struct sifive_hca_dev *hca)
{
	return (readl(hca->regs + HCA_DMA_CR) & HCA_DMA_CR_BUSY);
}

static inline int sifive_hca_dma_is_src_unaligned(struct sifive_hca_dev *hca)
{
	return (readl(hca->regs + HCA_DMA_CR) & HCA_DMA_CR_RDALIGNERR);
}

static inline int sifive_hca_dma_is_dest_unaligned(struct sifive_hca_dev *hca)
{
	return (readl(hca->regs + HCA_DMA_CR) & HCA_DMA_CR_WRALIGNERR);
}

static inline int sifive_hca_dma_is_bus_error(struct sifive_hca_dev *hca)
{
	return (readl(hca->regs + HCA_DMA_CR) & HCA_DMA_CR_RESPERR);
}

static inline int sifive_hca_dma_is_rw_illegal(struct sifive_hca_dev *hca)
{
	return (readl(hca->regs + HCA_DMA_CR) & HCA_DMA_CR_LEGALERR);
}

static inline void sifive_hca_dma_set_length(struct sifive_hca_dev *hca, uint32_t data)
{
	writel(data & HCA_DMA_LEN_MASK, hca->regs + HCA_DMA_LEN);
}

static inline void sifive_hca_dma_set_src(struct sifive_hca_dev *hca, uint32_t addr)
{
	writel(addr, hca->regs + HCA_DMA_SRC);
}

static inline void sifive_hca_dma_set_dest(struct sifive_hca_dev *hca, uint32_t addr)
{
	writel(addr, hca->regs + HCA_DMA_DEST);
}

static inline uint32_t sifive_hca_get_rev(struct sifive_hca_dev *hca, uint32_t mask)
{
	return (readl(hca->regs + HCA_REV) & mask);
}

static inline uint32_t sifive_hca_get_aes_rev(struct sifive_hca_dev *hca, uint32_t mask)
{
	return (readl(hca->regs + HCA_AES_REV) & mask);
}

static inline uint32_t sifive_hca_get_sha_rev(struct sifive_hca_dev *hca, uint32_t mask)
{
	return (readl(hca->regs + HCA_SHA_REV) & mask);
}

//extern struct skcipher_alg sifive_hca_cbc_aes_alg;
extern struct ahash_alg sifive_hca_sha512_alg;

int sifive_aes_algs_register(struct sifive_hca_dev *hca);
//void sifive_aes_algs_unregister(struct sifive_hca_dev *hca);

int sifive_ahash_algs_register(struct sifive_hca_dev *hca);
void sifive_ahash_algs_unregister(struct sifive_hca_dev *hca);

int sifive_hca_dma_init(struct sifive_hca_dev *hca);
int sifive_hca_dma_free(struct sifive_hca_dev *hca);

#endif /* _ASM_SIFIVE_HCA_H */
