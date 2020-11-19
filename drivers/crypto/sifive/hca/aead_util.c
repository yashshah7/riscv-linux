// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2020 SiFive
 * This code is taken from drivers/crypto/bcm/util.c
 */

#include "aead_util.h"

/**
 * sifive_hca_sg_at_offset() - Find the scatterlist entry at a given distance from the
 * start of a scatterlist.
 * @sg:         [in]  Start of a scatterlist
 * @skip:       [in]  Distance from the start of the scatterlist, in bytes
 * @sge:        [out] Scatterlist entry at skip bytes from start
 * @sge_offset: [out] Number of bytes from start of sge buffer to get to
 *                    requested distance.
 *
 * Return: 0 if entry found at requested distance
 *         < 0 otherwise
 */
int sifive_hca_sg_at_offset(struct scatterlist *sg, unsigned int skip,
                     struct scatterlist **sge, unsigned int *sge_offset)
{
        /* byte index from start of sg to the end of the previous entry */
        unsigned int index = 0;
        /* byte index from start of sg to the end of the current entry */
        unsigned int next_index;

        next_index = sg->length;
        while (next_index <= skip) {
                sg = sg_next(sg);
                index = next_index;
                if (!sg)
                        return -EINVAL;
                next_index += sg->length;
        }

        *sge_offset = skip - index;
        *sge = sg;
        return 0;
}

/**
 * sifive_hca_sg_count() - Determine number of elements in scatterlist to
 * provide a specified number of bytes.
 * @sg_list:  scatterlist to examine
 * @skip:     index of starting point
 * @nbytes:   consider elements of scatterlist until reaching this number of
 *            bytes
 *
 * Return: the number of sg entries contributing to nbytes of data
 */
int sifive_hca_sg_count(struct scatterlist *sg_list, unsigned int skip, int nbytes)
{
        struct scatterlist *sg;
        int sg_nents = 0;
        unsigned int offset;

        if (!sg_list)
                return 0;

        if (sifive_hca_sg_at_offset(sg_list, skip, &sg, &offset) < 0)
                return 0;

        while (sg && (nbytes > 0)) {
                sg_nents++;
                nbytes -= (sg->length - offset);
                offset = 0;
                sg = sg_next(sg);
        }
        return sg_nents;
}

