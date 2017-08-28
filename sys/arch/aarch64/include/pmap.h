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

//XXXAARCH64
//#include <uvm/pmap/pmap_pv.h>
#include <uvm/uvm_pglist.h>

#define PMAP_GROWKERNEL
#define PMAP_STEAL_MEMORY

struct pmap {
	struct pool *pm_pvpool;
	struct pglist pm_pglist;
	uint16_t pm_asid;
	uint64_t pm_ttbr;
//XXXAARCH64
//	pmap_pv_info_t pm_pvinfo;
	struct pmap_statistics pm_stats;
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

#define l0pte_pa(pde)		((pde) & LX_TBL_PA)
#define l0pte_index(v)		(((vaddr_t)(v) & L0_ADDR_BITS) >> L0_SHIFT)
#define l0pte_valid(pde)	(((pde) & LX_VALID) == LX_VALID)
/* l0pte is always table */

#define l1pte_pa(pde)		((pde) & LX_TBL_PA)
#define l1pte_index(v)		(((vaddr_t)(v) & L1_ADDR_BITS) >> L1_SHIFT)
#define l1pte_valid(pde)	(((pde) & LX_VALID) == LX_VALID)
#define l1pte_is_block(pde)	(((pde) & LX_TYPE) == LX_TYPE_BLK)
#define l1pte_is_table(pde)	(((pde) & LX_TYPE) == LX_TYPE_TBL)

#define l2pte_pa(pde)		((pde) & LX_TBL_PA)
#define l2pte_index(v)		(((vaddr_t)(v) & L2_ADDR_BITS) >> L2_SHIFT)
#define l2pte_valid(pde)	(((pde) & LX_VALID) == LX_VALID)
#define l2pte_is_block(pde)	(((pde) & LX_TYPE) == LX_TYPE_BLK)
#define l2pte_is_table(pde)	(((pde) & LX_TYPE) == LX_TYPE_TBL)

#define l3pte_pa(pde)		((pde) & LX_TBL_PA)
#define l3pte_index(v)		(((vaddr_t)(v) & L3_ADDR_BITS) >> L3_SHIFT)
#define l3pte_valid(pde)	(((pde) & LX_VALID) == LX_VALID)
#define l3pte_is_page(pde)	(((pde) & LX_TYPE) == L3_TYPE_PAG)
/* l3pte is always page */

void pmap_bootstrap(vaddr_t, vaddr_t);

#elif defined(__arm__)

#include <arm/pmap.h>

#endif

#endif /* !_AARCH64_PMAP_H_ */
