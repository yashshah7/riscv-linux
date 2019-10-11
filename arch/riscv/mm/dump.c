// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
 * Debug helper to dump the current kernel pagetables of the system
 * so that we can see what the various memory ranges are set to.
 *
 * Derived from x86 and arm implementation:
 * (C) Copyright 2019 SiFive Inc.
 *
 * Author: Yash Shah <yash.shah@sifive.com>
 */
#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/seq_file.h>

#include <asm/fixmap.h>
#include <asm/pgtable.h>
#include <asm/pgtable-bits.h>
#include <asm/ptdump.h>

static const struct addr_marker address_markers[] = {
	{ FIXADDR_START,		"Fixmap start" },
	{ FIXADDR_TOP,			"Fixmap end" },
	{ PCI_IO_START,			"PCI I/O start" },
	{ PCI_IO_END,			"PCI I/O end" },
	{ VMALLOC_START,		"vmalloc() area" },
	{ VMALLOC_END,			"vmalloc() end" },
	{ PAGE_OFFSET,			"Linear mapping" },
	{ -1,				NULL },
};

#define pt_dump_seq_printf(_m, fmt, args...)	\
({						\
	typeof(_m) (m) = (_m);			\
	if (m)					\
		seq_printf(m, fmt, ##args);	\
})

#define pt_dump_seq_puts(_m, fmt)	\
({					\
	typeof(_m) (m) = (_m);		\
	if (m)				\
		seq_printf(m, fmt);	\
})

#define ENDADDR (PAGE_OFFSET + PAGE_SIZE)

/*
 * The page dumper groups page table entries of the same type into a single
 * description. It uses pg_state to track the range information while
 * iterating over the pte entries. When the continuity is broken it then
 * dumps out a description of the range.
 */
struct pg_state {
	struct seq_file *seq;
	const struct addr_marker *marker;
	unsigned long start_address;
	unsigned int level;
	u64 current_prot;
	bool check_wx;
	unsigned long wx_pages;
	unsigned long uxn_pages;
};

struct prot_bits {
	u64		mask;
	u64		val;
	const char	*set;
	const char	*clear;
};

static const struct prot_bits pte_bits[] = {
	{
		.mask	= _PAGE_PRESENT,
		.val	= _PAGE_PRESENT,
		.set	= " ",
		.clear	= "F",
	}, {
		.mask	= _PAGE_USER,
		.val	= _PAGE_USER,
		.set	= "USR",
		.clear	= "   ",
	}, {
		.mask	= _PAGE_READ,
		.val	= _PAGE_READ,
		.set	= "R",
		.clear	= " ",
	}, {
		.mask	= _PAGE_WRITE,
		.val	= _PAGE_WRITE,
		.set	= "W",
		.clear	= " ",
	}, {
		.mask	= _PAGE_EXEC,
		.val	= _PAGE_EXEC,
		.set	= "X",
		.clear	= " ",
	}, {
		.mask	= _PAGE_GLOBAL,
		.val	= _PAGE_GLOBAL,
		.set	= "G",
		.clear	= " ",
	}, {
		.mask	= _PAGE_ACCESSED,
		.val	= _PAGE_ACCESSED,
		.set	= "A",
		.clear	= " ",
	}, {
		.mask	= _PAGE_DIRTY,
		.val	= _PAGE_DIRTY,
		.set	= "D",
		.clear	= " ",
	}, {
		.mask	= _PAGE_SOFT,
		.val	= _PAGE_SOFT,
		.set	= "SW",
		.clear	= "  ",
	}
};

struct pg_level {
	const struct prot_bits *bits;
	const char *name;
	size_t num;
	u64 mask;
};

static struct pg_level pg_level[] = {
	{
	}, { /* pgd */
		.name	= "PGD",
		.bits	= pte_bits,
		.num	= ARRAY_SIZE(pte_bits),
	}, { /* p4d */
		.name	= (CONFIG_PGTABLE_LEVELS > 4) ? "P4D" : "PGD",
		.bits	= pte_bits,
		.num	= ARRAY_SIZE(pte_bits),
	}, { /* pud */
		.name	= (CONFIG_PGTABLE_LEVELS > 3) ? "PUD" : "PGD",
		.bits	= pte_bits,
		.num	= ARRAY_SIZE(pte_bits),
	}, { /* pmd */
		.name	= (CONFIG_PGTABLE_LEVELS > 2) ? "PMD" : "PGD",
		.bits	= pte_bits,
		.num	= ARRAY_SIZE(pte_bits),
	}, { /* pte */
		.name	= "PTE",
		.bits	= pte_bits,
		.num	= ARRAY_SIZE(pte_bits),
	},
};

static void dump_prot(struct pg_state *st, const struct prot_bits *bits,
		      size_t num)
{
	unsigned int i;

	for (i = 0; i < num; i++, bits++) {
		const char *s;

		if ((st->current_prot & bits->mask) == bits->val)
			s = bits->set;
		else
			s = bits->clear;

		if (s)
			pt_dump_seq_printf(st->seq, " %s", s);
	}
}

static void note_prot_wx(struct pg_state *st, unsigned long addr)
{
	if (!st->check_wx)
		return;
	if (!((st->current_prot & pgprot_val(PAGE_KERNEL)) ==
	      pgprot_val(PAGE_KERNEL)))
		return;

	WARN_ONCE(1, "riscv/mm: Found insecure W+X mapping at address %p/%pS\n",
		  (void *)st->start_address, (void *)st->start_address);

	st->wx_pages += (addr - st->start_address) / PAGE_SIZE;
}

static void note_page(struct pg_state *st, unsigned long addr,
		      unsigned int level, u64 val)
{
	static const char units[] = "KMGTPE";
	u64 prot = val & pg_level[level].mask;

	if (!st->level) {
		st->level = level;
		st->current_prot = prot;
		st->start_address = addr;
		pt_dump_seq_printf(st->seq, "---[ %s ]---\n", st->marker->name);
	} else if (prot != st->current_prot || level != st->level ||
		   addr >= st->marker[1].start_address) {
		const char *unit = units;
		unsigned long delta;

		note_prot_wx(st, addr);
		pt_dump_seq_printf(st->seq, "0x%016lx-0x%016lx   ",
				   st->start_address, addr);

		delta = (addr - st->start_address) >> 10;
		while (!(delta & 1023) && unit[1]) {
			delta >>= 10;
			unit++;
		}
		pt_dump_seq_printf(st->seq, "%9lu%c %s", delta, *unit,
				   pg_level[st->level].name);
		if (pg_level[st->level].bits)
			dump_prot(st, pg_level[st->level].bits,
				  pg_level[st->level].num);
		pt_dump_seq_puts(st->seq, "\n");

		if (addr >= st->marker[1].start_address) {
			st->marker++;
			pt_dump_seq_printf(st->seq, "---[ %s ]---\n",
					   st->marker->name);
		}

		st->start_address = addr;
		st->current_prot = prot;
		st->level = level;
	}

	if (addr >= st->marker[1].start_address) {
		st->marker++;
		pt_dump_seq_printf(st->seq, "---[ %s ]---\n", st->marker->name);
	}
}

static void walk_pte(struct pg_state *st, pmd_t *pmdp, unsigned long start,
		     unsigned long end)
{
	unsigned long addr = start;
	pte_t *ptep = pte_offset_kernel(pmdp, start);

	do {
		note_page(st, addr, 4, READ_ONCE(pte_val(*ptep)));
	} while (ptep++, addr += PAGE_SIZE, addr != end);
}

static void walk_pmd(struct pg_state *st, pud_t *pudp, unsigned long start,
		     unsigned long end)
{
	unsigned long next, addr = start;
	pmd_t *pmdp = pmd_offset(pudp, start);

	do {
		pmd_t pmd = READ_ONCE(*pmdp);

		next = pmd_addr_end(addr, end);

		if (pmd_none(pmd)) {
			note_page(st, addr, 3, pmd_val(pmd));
		} else {
			WARN_ON(pmd_bad(pmd));
			walk_pte(st, pmdp, addr, next);
		}
	} while (pmdp++, addr = next, addr != end);
}

static void walk_pud(struct pg_state *st, p4d_t *p4dp, unsigned long start,
		     unsigned long end)
{
	unsigned long next, addr = start;
	pud_t *pudp = pud_offset(p4dp, start);

	do {
		pud_t pud = READ_ONCE(*pudp);

		next = pud_addr_end(addr, end);

		if (pud_none(pud)) {
			note_page(st, addr, 2, pud_val(pud));
		} else {
			WARN_ON(pud_bad(pud));
			walk_pmd(st, pudp, addr, next);
		}
	} while (pudp++, addr = next, addr != end);
}

static void walk_p4d(struct pg_state *st, pgd_t *pgdp, unsigned long start,
		     unsigned long end)
{
	unsigned long next, addr = start;
	p4d_t *p4dp = p4d_offset(pgdp, start);

	do {
		p4d_t p4d = READ_ONCE(*p4dp);

		next = p4d_addr_end(addr, end);

		if (p4d_none(p4d)) {
			note_page(st, addr, 2, p4d_val(p4d));
		} else {
			WARN_ON(p4d_bad(p4d));
			walk_pud(st, p4dp, addr, next);
		}
	} while (p4dp++, addr = next, addr != end);
}

static void walk_pgd(struct pg_state *st, struct mm_struct *mm,
		     unsigned long start)
{
	unsigned long end = ENDADDR;
	unsigned long next, addr = start;
	pgd_t *pgdp = pgd_offset(mm, start);

	do {
		pgd_t pgd = READ_ONCE(*pgdp);

		next = pgd_addr_end(addr, end);

		if (pgd_none(pgd)) {
			note_page(st, addr, 1, pgd_val(pgd));
		} else {
			WARN_ON(pgd_bad(pgd));
			walk_p4d(st, pgdp, addr, next);
		}
	} while (pgdp++, addr = next, addr != end);
}

void ptdump_walk_pgd(struct seq_file *m, struct ptdump_info *info)
{
	struct pg_state st = {
		.seq = m,
		.marker = info->markers,
	};
	walk_pgd(&st, info->mm, info->base_addr);

	note_page(&st, 0, 0, 0);
}

static void ptdump_initialize(void)
{
	unsigned int i, j;

	for (i = 0; i < ARRAY_SIZE(pg_level); i++)
		if (pg_level[i].bits)
			for (j = 0; j < pg_level[i].num; j++)
				pg_level[i].mask |= pg_level[i].bits[j].mask;
}

static struct ptdump_info kernel_ptdump_info = {
	.mm		= &init_mm,
	.markers	= address_markers,
	.base_addr	= FIXADDR_START,
};

void ptdump_check_wx(void)
{
	struct pg_state st = {
		.seq = NULL,
		.marker = (struct addr_marker[]) {
			{ 0, NULL},
			{ -1, NULL},
		},
		.check_wx = true,
	};

	walk_pgd(&st, &init_mm, FIXADDR_START);
	note_page(&st, 0, 0, 0);
	if (st.wx_pages)
		pr_warn("Checked W+X mappings: FAILED, %lu W+X pages found\n",
			st.wx_pages);
	else
		pr_info("Checked W+X mappings: passed, no W+X pages found\n");
}

static int ptdump_init(void)
{
	ptdump_initialize();
	ptdump_debugfs_register(&kernel_ptdump_info, "kernel_page_tables");
	return 0;
}
device_initcall(ptdump_init);
