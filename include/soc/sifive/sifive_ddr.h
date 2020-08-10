/* SPDX-License-Identifier: GPL-2.0 */
/*
 * SiFive DDR Controller header file
 *
 */

#ifndef __SOC_SIFIVE_DDR_H
#define __SOC_SIFIVE_DDR_H

int register_sifive_ddr_error_notifier(struct notifier_block *nb);
int unregister_sifive_ddr_error_notifier(struct notifier_block *nb);

struct sifive_ddr_priv {
	void __iomem *reg;
	struct device *dev;
	u16 error_count;
	unsigned long page_frame_number;
	unsigned long offset_in_page;
	unsigned long syndrome;
	int top_layer;
	int mid_layer;
	int low_layer;
};

#define SIFIVE_DDR_ERR_TYPE_CE	(0)
#define SIFIVE_DDR_ERR_TYPE_UE	(1)
#define SIFIVE_DDR_EDAC_GRAIN	(1)
#define SIFIVE_MEM_TYPE_DDR4	0xA
#define SIFIVE_DDR_WIDTH_16	(2)
#define SIFIVE_DDR_WIDTH_32	(1)
#define SIFIVE_DDR_WIDTH_64	(0)

#define DDR_CTL_MEM_TYPE_REG	0x000
#define DDR_CTL_MEM_WIDTH_REG	0x004
#define ECC_CTL_CONTROL_REG	0x174
#define ECC_U_ADDR_L_REG	0x180
#define ECC_U_ADDR_H_REG	0x184
#define ECC_U_DATA_L_REG	0x188
#define ECC_U_DATA_H_REG	0x18c

#define ECC_C_ADDR_L_REG	0x190
#define ECC_C_ADDR_H_REG	0x194
#define ECC_C_DATA_L_REG	0x198
#define ECC_C_DATA_H_REG	0x19c
#define ECC_U_C_ID_REG		0x1A0
#define ECC_CTL_INT_STATUS_REG	0x210
#define ECC_CTL_INT_ACK_REG	0x218
#define ECC_CTL_INT_MASK_REG	0x220
#define ECC_C_SYND_REG		ECC_C_ADDR_H_REG
#define ECC_U_SYND_REG		ECC_U_ADDR_H_REG

#define ECC_CTL_MTYPE_MASK	GENMASK(11, 8)
#define CTL_MEM_MAX_WIDTH_MASK	GENMASK(31, 24)
#define ECC_ADDR_H_MASK		GENMASK(3, 0)
#define ECC_INT_CE_UE_MASK	GENMASK(6, 3)
#define ECC_CE_INTR_MASK	GENMASK(4, 3)
#define ECC_UE_INTR_MASK	GENMASK(6, 5)
#define ECC_INT_CE_EVENT	BIT(3)
#define ECC_INT_SECOND_CE_EVENT	BIT(4)
#define ECC_INT_UE_EVENT	BIT(5)
#define ECC_INT_SECOND_UE_EVENT	BIT(6)
#define ECC_CTL_ECC_ENABLE	BIT(16)

#define ECC_C_ID_MASK		GENMASK(28, 16)
#define ECC_U_ID_MASK		GENMASK(12, 0)
#define ECC_C_ID_SHIFT		(16)
#define ECC_U_ID_SHIFT		(0)
#define ECC_SYND_MASK		GENMASK(15, 8)
#define ECC_SYND_SHIFT		(8)

#define CTL_REG_WIDTH_SHIFT	(32)

#endif /* __SOC_SIFIVE_DDR_H */
