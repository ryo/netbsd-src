/* $NetBSD: pmap.h,v 1.1 2014/08/10 05:47:38 matt Exp $ */

/*-
 * Copyright (c) 2014 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Matt Thomas of 3am Software Foundry.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _AARCH64_PMAP_H_
#define _AARCH64_PMAP_H_

#ifdef __aarch64__

#include <sys/types.h>
#include <sys/pool.h>
#include <uvm/uvm_pglist.h>

#include <aarch64/pte.h>

#define PMAP_GROWKERNEL
#define PMAP_STEAL_MEMORY

struct pmap {
	kmutex_t pm_lock;
//	struct pool *pm_pvpool;
//	struct pglist pm_pglist;
	pd_entry_t *pm_l0table;	/* L0 table: 512G*512 */
	pd_entry_t *pm_l1table;	/* L1 table: 2G*512 */
	struct pmap_statistics pm_stats;
	unsigned int pm_refcnt;
//	uint16_t pm_asid;
};

#define __HAVE_VM_PAGE_MD

struct vm_page_md {
	uint32_t mdpg_attrs;
#define VM_PAGE_MD_MODIFIED	0x01
#define VM_PAGE_MD_REFERENCED	0x02
#define VM_PAGE_MD_EXECUTABLE	0x04
//XXXAARCH64
//	vm_page_pv_info_t mdpg_pv;
};

#define	VM_MDPAGE_INIT(pg)			\
	do {					\
		(pg)->mdpage.mdpg_attrs = 0;	\
		VM_MDPAGE_PV_INIT(pg);		\
	} while (/*CONSTCOND*/ 0)

/* XXXAARCH64 */
#define	VM_MDPAGE_PV_INIT(pg)			\
	do {					\
	} while (/*CONSTCOND*/ 0)


#define l0pde_pa(pde)		((pde) & LX_TBL_PA)
#define l0pde_index(v)		(((vaddr_t)(v) & L0_ADDR_BITS) >> L0_SHIFT)
#define l0pde_valid(pde)	(((pde) & LX_VALID) == LX_VALID)
/* l0pte always contains table entries */

#define l1pde_pa(pde)		((pde) & LX_TBL_PA)
#define l1pde_index(v)		(((vaddr_t)(v) & L1_ADDR_BITS) >> L1_SHIFT)
#define l1pde_valid(pde)	(((pde) & LX_VALID) == LX_VALID)
#define l1pde_is_block(pde)	(((pde) & LX_TYPE) == LX_TYPE_BLK)
#define l1pde_is_table(pde)	(((pde) & LX_TYPE) == LX_TYPE_TBL)

#define l2pde_pa(pde)		((pde) & LX_TBL_PA)
#define l2pde_index(v)		(((vaddr_t)(v) & L2_ADDR_BITS) >> L2_SHIFT)
#define l2pde_valid(pde)	(((pde) & LX_VALID) == LX_VALID)
#define l2pde_is_block(pde)	(((pde) & LX_TYPE) == LX_TYPE_BLK)
#define l2pde_is_table(pde)	(((pde) & LX_TYPE) == LX_TYPE_TBL)

#define l3pte_pa(pde)		((pde) & LX_TBL_PA)
#define l3pte_index(v)		(((vaddr_t)(v) & L3_ADDR_BITS) >> L3_SHIFT)
#define l3pte_valid(pde)	(((pde) & LX_VALID) == LX_VALID)
#define l3pte_is_page(pde)	(((pde) & LX_TYPE) == L3_TYPE_PAG)
/* l3pte contains always page entries */

void pmap_bootstrap(vaddr_t, vaddr_t);


/* Hooks for the pool allocator */
paddr_t vtophys(vaddr_t);
#define VTOPHYS_FAILED		((paddr_t)-1L)	/* POOL_PADDR_INVALID */
#define POOL_VTOPHYS(va)	vtophys((vaddr_t) (va))


/* devmap */
struct pmap_devmap {
	vaddr_t pd_va;		/* virtual address */
	paddr_t pd_pa;		/* physical address */
	psize_t pd_size;	/* size of region */
	vm_prot_t pd_prot;	/* protection code */
	u_int pd_flags;		/* flags for pmap_kenter_pa() */
};
void pmap_devmap_register(const struct pmap_devmap *);
void pmap_devmap_bootstrap(const struct pmap_devmap *);
const struct pmap_devmap *pmap_devmap_find_pa(paddr_t, psize_t);
const struct pmap_devmap *pmap_devmap_find_va(vaddr_t, vsize_t);
vaddr_t pmap_devmap_phystov(paddr_t);
paddr_t pmap_devmap_vtophys(paddr_t);

/* devmap use L2 blocks. (2Mbyte) */
#define DEVMAP_TRUNC_ADDR(x)	((x) & ~L2_OFFSET)
#define DEVMAP_ROUND_SIZE(x)	(((x) + L2_SIZE - 1) & ~(L2_SIZE - 1))


/* mmap cookie and flags */
#define AARCH64_MMAP_FLAG_SHIFT		(64 - PGSHIFT)
#define AARCH64_MMAP_FLAG_MASK		0xf
#define AARCH64_MMAP_WRITEBACK		0
#define AARCH64_MMAP_NOCACHE		1
#define AARCH64_MMAP_DEVICE		3
static inline u_int
aarch64_mmap_flags(paddr_t mdpgno)
{
	u_int nflag, pflag;

	/*
	 * aarch64 arch has 4 memory attribute:
	 *
	 *  WriteBack      - write back cache
	 *  WriteThru      - wite through cache
	 *  NoCache        - no cache
	 *  Device(nGnRnE) - no Gathering, no Reordering, no Early write ack
	 *
	 * but pmap has PMAP_{NOCACHE,WRITE_COMBINE,WRITE_BACK} flags.
	 */

	nflag = (mdpgno >> AARCH64_MMAP_FLAG_SHIFT) & AARCH64_MMAP_FLAG_MASK;
	switch (nflag) {
	case AARCH64_MMAP_WRITEBACK:
		pflag = PMAP_WRITE_BACK;
		break;
	default:
	case AARCH64_MMAP_NOCACHE:
	case AARCH64_MMAP_DEVICE:
		pflag = PMAP_NOCACHE;
		break;
	}
	return pflag;
}
#define pmap_phys_address(pa)	aarch64_ptob((pa))
#define pmap_mmap_flags(ppn)	aarch64_mmap_flags((ppn))


#elif defined(__arm__)

#include <arm/pmap.h>

#endif /* __arm__/__aarch64__ */

#endif /* !_AARCH64_PMAP_ */
