/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2020 SiFive
 */

#ifndef _AEAD_UTIL_H
#define _AEAD_UTIL_H

#include <linux/kernel.h>
#include <linux/scatterlist.h>

int sifive_hca_sg_at_offset(struct scatterlist *sg, unsigned int skip,
                     struct scatterlist **sge, unsigned int *sge_offset);

int sifive_hca_sg_count(struct scatterlist *sg_list, unsigned int skip, int nbytes);

#endif
