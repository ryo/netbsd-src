/*	$NetBSD: pmap.c,v 1.1 2014/08/10 05:47:37 matt Exp $	*/

/*
 * Copyright (c) 2017 Ryo Shimizu <ryo@nerv.org>
 * All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>

__KERNEL_RCSID(1, "$NetBSD: pmap.c,v 1.1 2014/08/10 05:47:37 matt Exp $");

#include <sys/param.h>
#include <sys/types.h>
#include <sys/kmem.h>
#include <sys/vmem.h>
#include <sys/atomic.h>

#include <uvm/uvm.h>

#include <aarch64/pmap.h>
#include <aarch64/pte.h>
#include <aarch64/armreg.h>
#include <aarch64/cpufunc.h>

#include "opt_arm_debug.h"

/* memory attributes are configured MAIR_EL1 in locore */
#define LX_BLKPAG_ATTR_NORMAL_WB	__SHIFTIN(0, LX_BLKPAG_ATTR_INDX)
#define LX_BLKPAG_ATTR_NORMAL_NC	__SHIFTIN(1, LX_BLKPAG_ATTR_INDX)
#define LX_BLKPAG_ATTR_NORMAL_WT	__SHIFTIN(2, LX_BLKPAG_ATTR_INDX)
#define LX_BLKPAG_ATTR_DEVICE_MEM	__SHIFTIN(3, LX_BLKPAG_ATTR_INDX)
#define LX_BLKPAG_ATTR_MASK		LX_BLKPAG_ATTR_INDX

static pt_entry_t *_pmap_pte_lookup(vaddr_t);
static pd_entry_t *_pmap_grow_l2(pd_entry_t *, vaddr_t);
static pt_entry_t *_pmap_grow_l3(pd_entry_t *, vaddr_t);
static pt_entry_t _pmap_pte_update_prot(pt_entry_t, vm_prot_t);
static pt_entry_t _pmap_pte_update_cacheflags(pt_entry_t, u_int);
static struct pmap kernel_pmap;

struct pmap * const kernel_pmap_ptr = &kernel_pmap;
static vaddr_t pmap_maxkvaddr;
vmem_t *pmap_asid_arena;

vaddr_t virtual_avail, virtual_end;
vaddr_t virtual_devmap_addr;

//#define PMAP_DEBUG

#ifdef PMAP_DEBUG
#define DMARK()				printf("%s:%d\n", __func__, __LINE__)
#define DPRINTF(format, args...)	printf("%s:%d: " format , \
						__func__, __LINE__, args)
#else
#define DMARK()
#define DPRINTF(args...)
#endif


static const struct pmap_devmap *pmap_devmap_table;

/* XXX: for now, only support for devmap */
static vsize_t
_pmap_map_chunk(pd_entry_t *l2, vaddr_t va, paddr_t pa, vsize_t size,
    vm_prot_t prot, u_int flags)
{
	pd_entry_t oldpte;
	pt_entry_t attr;
	vsize_t resid;

	oldpte = l2[l2pde_index(va)];
	KDASSERT(!l2pde_valid(oldpte));

	attr = _pmap_pte_update_prot(L2_BLOCK, prot);
	attr = _pmap_pte_update_cacheflags(attr, flags);
#ifdef MULTIPROCESSOR
	attr |= LX_BLKPAG_SH_IS;
#endif

	resid = (size + (L2_SIZE - 1)) & ~(L2_SIZE - 1);
	size = resid;

	while (resid > 0) {
		pt_entry_t pte;

		pte = pa | attr;
		atomic_swap_64(&l2[l2pde_index(va)], pte);

		aarch64_tlb_flushID_SE(va);
		if ((prot & VM_PROT_EXECUTE) != 0)
			cpu_icache_sync_range(va, L2_SIZE);

		va += L2_SIZE;
		pa += L2_SIZE;
		resid -= L2_SIZE;
	}

	return size;
}

void
pmap_devmap_register(const struct pmap_devmap *table)
{
	pmap_devmap_table = table;
}

void
pmap_devmap_bootstrap(const struct pmap_devmap *table)
{
	pd_entry_t *l0, *l1, *l2;
	vaddr_t va;
	int i;

	pmap_devmap_register(table);

	l0 = AARCH64_PA_TO_KVA(reg_ttbr1_el1_read());

#ifdef VERBOSE_INIT_ARM
	printf("%s:\n", __func__);
#endif
	for (i = 0; table[i].pd_size != 0; i++) {
#ifdef VERBOSE_INIT_ARM
		printf(" devmap: pa %08lx-%08lx = va %016lx\n",
		    table[i].pd_pa,
		    table[i].pd_pa + table[i].pd_size - 1,
		    table[i].pd_va);
#endif
		va = table[i].pd_va;

		/* update and check virtual_devmap_addr */
		if ((virtual_devmap_addr == 0) ||
		    (virtual_devmap_addr > va)) {
			virtual_devmap_addr = va;

			/* XXX: only one L2 table is allocated for devmap  */
			if ((VM_MAX_KERNEL_ADDRESS - virtual_devmap_addr) >
			    (L2_SIZE * Ln_ENTRIES)) {
				panic("devmap va:%016lx out of range."
				    " available devmap range is %016lx-%016lx",
				    va,
				    VM_MAX_KERNEL_ADDRESS -
				    (L2_SIZE * Ln_ENTRIES),
				    VM_MAX_KERNEL_ADDRESS);
			}
		}

		l1 = l0pde_pa(l0[l0pde_index(va)]);
		KASSERT(l1 != NULL);
		l1 = AARCH64_PA_TO_KVA((paddr_t)l1);

		l2 = l1pde_pa(l1[l1pde_index(va)]);
		if (l2 == NULL)
			panic("L2 table for devmap is not allocated");

		l2 = AARCH64_PA_TO_KVA((paddr_t)l2);

		_pmap_map_chunk(l2,
		    table[i].pd_va,
		    table[i].pd_pa,
		    table[i].pd_size,
		    table[i].pd_prot,
		    table[i].pd_flags);
	}
}

const struct pmap_devmap *
pmap_devmap_find_va(vaddr_t va, vsize_t size)
{
	paddr_t endva;
	int i;

	if (pmap_devmap_table == NULL)
		return NULL;

	endva = va + size;
	for (i = 0; pmap_devmap_table[i].pd_size != 0; i++) {
		if ((va <= pmap_devmap_table[i].pd_va) &&
		    (endva <= pmap_devmap_table[i].pd_va +
		     pmap_devmap_table[i].pd_size))
			return &pmap_devmap_table[i];
	}
	return NULL;
}

const struct pmap_devmap *
pmap_devmap_find_pa(paddr_t pa, psize_t size)
{
	paddr_t endpa;
	int i;

	if (pmap_devmap_table == NULL)
		return NULL;

	endpa = pa + size;
	for (i = 0; pmap_devmap_table[i].pd_size != 0; i++) {
		if ((pa <= pmap_devmap_table[i].pd_pa) &&
		    (endpa <= pmap_devmap_table[i].pd_pa +
		     pmap_devmap_table[i].pd_size))
			return &pmap_devmap_table[i];
	}
	return NULL;
}

vaddr_t
pmap_devmap_phystov(paddr_t pa)
{
	const struct pmap_devmap *table;
	paddr_t offset;

	table = pmap_devmap_find_pa(pa, 0);
	if (table == NULL)
		return 0;

	offset = pa - table->pd_pa;
	return table->pd_va + offset;
}

vaddr_t
pmap_devmap_vtophys(paddr_t va)
{
	const struct pmap_devmap *table;
	vaddr_t offset;

	table = pmap_devmap_find_va(va, 0);
	if (table == NULL)
		return 0;

	offset = va - table->pd_va;
	return table->pd_pa + offset;
}

void
pmap_bootstrap(vaddr_t vstart, vaddr_t vend)
{
	struct pmap *kpm;
	pd_entry_t *l0, l1;
	vaddr_t va;

	/* devmap already uses last of va? */
	if ((virtual_devmap_addr != 0) && (virtual_devmap_addr < vend))
		vend = virtual_devmap_addr;

	virtual_avail = vstart;
	virtual_end = vend;
	pmap_maxkvaddr = vstart;

	cpu_tlb_flushID();

	va = vstart;
	l0 = AARCH64_PA_TO_KVA(reg_ttbr1_el1_read());
	l1 = AARCH64_PA_TO_KVA(l0pde_pa(l0[l0pde_index(va)]));

	memset(&kernel_pmap, 0, sizeof(kernel_pmap));
	kpm = pmap_kernel();
	kpm->pm_refcnt = 1;
	kpm->pm_l0table = l0;
	kpm->pm_l1table = l1;

	mutex_init(&kpm->pm_lock, MUTEX_DEFAULT, IPL_NONE);
}

void
pmap_init(void)
{
	DMARK();

	pmap_asid_arena = vmem_create("asid", 0, 65536, 1, NULL, NULL,
	    NULL, 2, VM_SLEEP, IPL_VM);
}

void
pmap_virtual_space(vaddr_t *vstartp, vaddr_t *vendp)
{
	*vstartp = virtual_avail;
	*vendp = virtual_end;
}

vaddr_t
pmap_steal_memory(vsize_t size, vaddr_t *vstartp, vaddr_t *vendp)
{
	int npage;
	paddr_t pa;
	vaddr_t va;
	psize_t bank_npage;
	uvm_physseg_t bank;

//	DPRINTF("size=%lu, *vstartp=%lx, *vendp=%lx\n", size, *vstartp, *vendp);

	size = round_page(size);
	npage = atop(size);

	for (bank = uvm_physseg_get_first(); uvm_physseg_valid_p(bank);
	    bank = uvm_physseg_get_next(bank)) {

		bank_npage = uvm_physseg_get_avail_end(bank) -
		    uvm_physseg_get_avail_start(bank);
		if (npage <= bank_npage)
			break;
	}

	KDASSERT(uvm_physseg_valid_p(bank));

	/* Steal pages */
	pa = ptoa(uvm_physseg_get_avail_start(bank));
	va = AARCH64_PA_TO_KVA(pa);
	uvm_physseg_unplug(atop(pa), npage);

//	DPRINTF("pa=0x%016lx-0x%016lx: va=0x%016lx\n", pa, pa + npage * PAGE_SIZE, va);

	for (; npage > 0; npage--, pa += PAGE_SIZE)
		pmap_zero_page(pa);

	return va;
}

void
pmap_reference(struct pmap *pm)
{
	atomic_inc_uint(&pm->pm_refcnt);
}

static pd_entry_t *
_pmap_grow_l2(pd_entry_t *l1, vaddr_t va)
{
	pd_entry_t pde, *l2;
	paddr_t pa;

	KASSERT(!(AARCH64_KSEG_START <= va && va < AARCH64_KSEG_END));

//	DPRINTF("l1=%p, va=%lx\n", l1, va);

	pde = l1[l1pde_index(va)];
	if (l1pde_valid(pde)) {
//		DPRINTF("L2 table exists: L1(%p)[%016llx(idx=%d)] -> %016llx\n",
//		    l1, (va & L1_ADDR_BITS), (int)l1pde_index(va), pde);

		KASSERT(!l1pde_is_block(pde));

		pa = l1pde_pa(pde);
		l2 = AARCH64_PA_TO_KVA(pa);
	} else {
		if (uvm.page_init_done) {
			struct vm_page *pg;

			pg = uvm_pagealloc(NULL, 0, NULL,
			    UVM_PGA_USERESERVE | UVM_PGA_ZERO);
			if (pg == NULL)
				panic("%s: cannot allocate L2 table", __func__);
			pa = VM_PAGE_TO_PHYS(pg);
//			DPRINTF("new l2table(PA)=%lx (pagealloc)\n", pa);
		} else {
			/* XXX: uvm_pageboot_alloc() returns AARCH64 KSEG address */
			pa = AARCH64_KVA_TO_PA(uvm_pageboot_alloc(Ln_TABLE_SIZE));
//			DPRINTF("new l2table(PA)=%lx (pageboot_alloc)\n", pa);
		}

		atomic_swap_64(&l1[l1pde_index(va)], pa | L1_TABLE | LX_VALID);

//		DPRINTF("add L2 table on L1(%p)[%016llx(idx=%d)] = %016llx\n",
//		    l1, (va & L1_ADDR_BITS), (int)l1pde_index(va), pde);

		l2 = AARCH64_PA_TO_KVA(pa);
	}
//	DPRINTF("l2=%p\n", l2);
	return l2;
}

pt_entry_t *
_pmap_grow_l3(pd_entry_t *l2, vaddr_t va)
{
	pd_entry_t pde, *l3;
	paddr_t pa;

	KASSERT(!(AARCH64_KSEG_START <= va && va < AARCH64_KSEG_END));

//	DPRINTF("l2=%p, va=%lx & %llx = %llx\n", l2, va, L2_ADDR_BITS, va & L2_ADDR_BITS);

	pde = l2[l2pde_index(va)];
	if (l2pde_valid(pde)) {
//		DPRINTF("L3 table exists: L2(%p)[%016llx(idx=%d)] -> %016llx\n",
//		    l2, (va & L2_ADDR_BITS), (int)l2pde_index(va), pde);

		KASSERT(l2pde_is_table(pde));

		pa = l2pde_pa(pde);
		l3 = AARCH64_PA_TO_KVA(pa);
	} else {

		if (uvm.page_init_done) {
			struct vm_page *pg;

			pg = uvm_pagealloc(NULL, 0, NULL,
			    UVM_PGA_USERESERVE | UVM_PGA_ZERO);
			if (pg == NULL)
				panic("%s: cannot allocate L3 table", __func__);
			pa = VM_PAGE_TO_PHYS(pg);
//			DPRINTF("new l3table(PA)=%lx (pagealloc)\n", pa);
		} else {
			/* XXX: uvm_pageboot_alloc() returns AARCH64 KSEG address */
			pa = AARCH64_KVA_TO_PA(uvm_pageboot_alloc(Ln_TABLE_SIZE));
//			DPRINTF("new l3table(PA)=%lx (pageboot_alloc)\n", pa);
		}

		atomic_swap_64(&l2[l2pde_index(va)], pa | L2_TABLE | LX_VALID);

//		DPRINTF("add L3 table on L2(%p)[%016llx(idx=%d)] = %016llx\n",
//		    l2, (va & L2_ADDR_BITS), (int)l2pde_index(va), pde);

		l3 = AARCH64_PA_TO_KVA(pa);
	}
//	DPRINTF("l3=%p\n", l3);
	return l3;
}

vaddr_t
pmap_growkernel(vaddr_t maxkvaddr)
{
	struct pmap *kpm = pmap_kernel();
	pd_entry_t *l2;
	pt_entry_t *l3;
	int s;

	DPRINTF("maxkvaddr=%16lx, pmap_maxkvaddrr=%16lx\n",
	    maxkvaddr, pmap_maxkvaddr);

	s = splvm();
	mutex_enter(&kpm->pm_lock);

	if (maxkvaddr <= pmap_maxkvaddr) {
		DPRINTF("no need to expand l1/l2 table%s", "\n");
		goto done;
	}

	KDASSERT(maxkvaddr <= virtual_end);

	for (; pmap_maxkvaddr < maxkvaddr; pmap_maxkvaddr += L2_SIZE) {
//		DPRINTF("growing pmap_maxkvaddr=%16lx\n",  pmap_maxkvaddr);

		l2 = _pmap_grow_l2(pmap_kernel()->pm_l1table, pmap_maxkvaddr);
		KDASSERT(l2 != NULL);

		l3 = _pmap_grow_l3(l2, pmap_maxkvaddr);
		KDASSERT(l3 != NULL);
	}
	cpu_tlb_flushID();

 done:
	mutex_exit(&kpm->pm_lock);
	splx(s);

	DPRINTF("done: pmap_maxkvaddr=%16lx\n", pmap_maxkvaddr);

	return pmap_maxkvaddr;
}

bool
pmap_extract(struct pmap *pm, vaddr_t va, paddr_t *pap)
{
	paddr_t pa;

	mutex_enter(&pm->pm_lock);

	if (VM_MIN_KERNEL_ADDRESS <= va && va < VM_MAX_KERNEL_ADDRESS) {
		pd_entry_t pde, *l2, *l3;
		pt_entry_t pte;

		/*
		 * traverse L0 -> L1 -> L2 -> L3 table
		 */
		pde = pmap_kernel()->pm_l1table[l1pde_index(va)];
		if (!l1pde_valid(pde)) {
			goto notfound;
		}
		if (l1pde_is_block(pde)) {
			pa = l1pde_pa(pde) + (va & L1_OFFSET);
			goto found;
		}

		KASSERT(l1pde_is_table(pde));

		l2 = AARCH64_PA_TO_KVA(l1pde_pa(pde));
		pde = l2[l2pde_index(va)];
		if (!l2pde_valid(pde))
			goto notfound;
		if (l2pde_is_block(pde)) {
			pa = l2pde_pa(pde) + (va & L2_OFFSET);
			goto found;
		}

		KASSERT(l2pde_is_table(pde));

		l3 = AARCH64_PA_TO_KVA(l2pde_pa(pde));
		pte = l3[l3pte_index(va)];
		if (!l3pte_valid(pte))
			goto notfound;

		KASSERT(l3pte_is_page(pte));

		pa = l3pte_pa(pte) + (va & L3_OFFSET);

	} else if (AARCH64_KSEG_START <= va && va < AARCH64_KSEG_END) {
		pa = AARCH64_KVA_TO_PA(va);

	} else {
 notfound:
		mutex_exit(&pm->pm_lock);
		return false;
	}

 found:
	mutex_exit(&pm->pm_lock);
	if (pap != NULL)
		*pap = pa;

	return true;
}

static pt_entry_t *
_pmap_pte_lookup(vaddr_t va)
{
	pd_entry_t pde, *l2;
	pt_entry_t *l3;
	paddr_t pa;

	if (VM_MIN_KERNEL_ADDRESS <= va && va < VM_MAX_KERNEL_ADDRESS) {
		pde = pmap_kernel()->pm_l1table[l1pde_index(va)];

		if (!l1pde_valid(pde)) {
			panic("L1 table is not exists for VA=0x%016lx", va);
			return NULL;
		}
		if (l1pde_is_block(pde)) {
			panic("L1 table is exists, but VA=0x%016lx is mapped as L1 block", va);
			return NULL;
		}
		if (!l1pde_is_table(pde))
			panic("%s: l1pde is not block nor table", __func__);

		pa = l1pde_pa(pde);
		l2 = AARCH64_PA_TO_KVA(pa);

		pde = l2[l2pde_index(va)];
		if (!l2pde_valid(pde)) {
			panic("L2 table is not exists for VA=0x%016lx", va);
			return NULL;
		}
		if (l2pde_is_block(pde)) {
			panic("L2 table is exists, but VA=0x%016lx is mapped as L2 block", va);
			return NULL;
		}
		if (!l2pde_is_table(pde))
			panic("%s: l2pde is not block nor table", __func__);

		pa = l2pde_pa(pde);
		l3 = AARCH64_PA_TO_KVA(pa);

		return &l3[l3pte_index(va)];

	} else if (AARCH64_KSEG_START <= va && va < AARCH64_KSEG_END) {
		panic("page entry is mapped in KSEG");
	} else {
		panic("Non kernel segment: va=0x%016lx", va);
	}

	return NULL;
}

static pt_entry_t
_pmap_pte_update_prot(pt_entry_t pte, vm_prot_t prot)
{
	pte &= ~LX_BLKPAG_AF;
	pte &= ~LX_BLKPAG_AP;

	switch (prot & (VM_PROT_READ|VM_PROT_WRITE)) {
	case 0:
	default:
		break;
	case VM_PROT_READ:
		pte |= LX_BLKPAG_AF;
		pte |= LX_BLKPAG_AP_RO_RO;
		break;
	case VM_PROT_WRITE:
	case VM_PROT_READ|VM_PROT_WRITE:
		pte |= LX_BLKPAG_AF;
		pte |= LX_BLKPAG_AP_RW_RW;
		break;
	}

	if ((prot & VM_PROT_EXECUTE) == 0)
		pte |= (LX_BLKPAG_UXN|LX_BLKPAG_PXN);
	else
		pte &= ~(LX_BLKPAG_UXN|LX_BLKPAG_PXN);

	return pte;
}

static pt_entry_t
_pmap_pte_update_cacheflags(pt_entry_t pte, u_int flags)
{
	pte &= ~LX_BLKPAG_ATTR_MASK;

	switch (flags & PMAP_CACHE_MASK) {
	case PMAP_NOCACHE:
	case PMAP_NOCACHE_OVR:
#if 0
		pte |= LX_BLKPAG_ATTR_NORMAL_NC;	/* only no-cache */
#else
		pte |= LX_BLKPAG_ATTR_DEVICE_MEM;	/* nGnRnE */
#endif
		break;
	case PMAP_WRITE_COMBINE:
	case PMAP_WRITE_BACK:
	default:
	case 0:
		pte |= LX_BLKPAG_ATTR_NORMAL_WB;
		break;
	}
	return pte;
}

void
pmap_kenter_pa(vaddr_t va, paddr_t pa, vm_prot_t prot, u_int flags)
{
	struct pmap *kpm = pmap_kernel();
	pt_entry_t *ptep, pte, attr;

	DPRINTF("va=%016lx, pa=%016lx, prot=%08x, flags=%08x\n",
	    va, pa, prot, flags);

	mutex_enter(&kpm->pm_lock);

	ptep = _pmap_pte_lookup(va);
	if (ptep == NULL) {
		panic("%s: cannot lookup L3 PTE: 0x%016lx\n", __func__, va);
	}

	attr = _pmap_pte_update_prot(LX_VALID | L3_TYPE_PAG, prot);
	attr = _pmap_pte_update_cacheflags(attr, flags);
#ifdef MULTIPROCESSOR
	attr |= LX_BLKPAG_SH_IS;
#endif

	pte = pa | attr;

	DPRINTF("ptep=%p, pte=0x%016llx\n", ptep, pte);

	atomic_swap_64(ptep, pte);

	cpu_tlb_flushID_SE(va);
	if ((prot & VM_PROT_EXECUTE) != 0)
		cpu_icache_sync_range(va, PAGE_SIZE);

	mutex_exit(&kpm->pm_lock);
}

void
pmap_update(struct pmap *pm)
{
//	DMARK();
	/* nothing to do */
}

void
pmap_kremove(vaddr_t va, vsize_t size)
{
	struct pmap *kpm = pmap_kernel();
	pt_entry_t *ptep;
	vaddr_t eva;

	DPRINTF("va=%016lx, size=%016lx\n", va, size);

	KDASSERT((va & PGOFSET) == 0);
	KDASSERT((size & PGOFSET) == 0);

	if (AARCH64_KSEG_START <= va && va < AARCH64_KSEG_END)
		return;

	eva = va + size;
	KDASSERT(VM_MIN_KERNEL_ADDRESS <= va && eva < VM_MAX_KERNEL_ADDRESS);

	mutex_enter(&kpm->pm_lock);
	for (; va < eva; va += PAGE_SIZE) {
		ptep = _pmap_pte_lookup(va);
		KASSERT(ptep != NULL);
		if (ptep == NULL)
			continue;

		atomic_swap_64(ptep, 0);
		cpu_tlb_flushID_SE(va);
	}
	mutex_exit(&kpm->pm_lock);
}

void
pmap_protect(struct pmap *pm, vaddr_t sva, vaddr_t eva, vm_prot_t prot)
{
	pt_entry_t *ptep, pte;
	vaddr_t va;
	bool is_kva;

	DPRINTF("sva=%016lx, eva=%016lx, prot=%08x\n", sva, eva, prot);

	if ((prot & VM_PROT_READ) == VM_PROT_NONE) {
		pmap_remove(pm, sva, eva);
		return;
	}

	/* check W^X */
	if ((prot & (VM_PROT_WRITE|VM_PROT_EXECUTE)) ==
	    (VM_PROT_WRITE|VM_PROT_EXECUTE)) {
		panic("%s:%d: W^X: sva=%016lx, eva=%016lx, prot=%08x\n", __func__, __LINE__, sva, eva, prot);
		return;
	}

	if (prot == VM_PROT_NONE) {
		pmap_remove(pm, sva, eva);
		return;
	}

	KDASSERT((sva & PAGE_MASK) == 0);
	KDASSERT((eva & PAGE_MASK) == 0);

#if 1
	//XXXAARCH64
	is_kva = (pm == pmap_kernel());
	if (!is_kva)
		panic("%s:%d: not support user process pmap\n", __func__, __LINE__);
#endif

	mutex_enter(&pm->pm_lock);

	for (va = sva; va < eva; va += PAGE_SIZE) {
		ptep = _pmap_pte_lookup(va);
		KASSERT(ptep != NULL);
		if (ptep == NULL)
			continue;

		pte = *ptep;
		pte = _pmap_pte_update_prot(pte, prot);
		atomic_swap_64(ptep, pte);

		cpu_tlb_flushID_SE(va);
	}

	mutex_exit(&pm->pm_lock);
}

struct pmap *
pmap_create(void)
{
	DMARK();
	panic("%s", __func__);
	return NULL;
}

void
pmap_destroy(struct pmap *pm)
{
	unsigned int refcnt;

	DMARK();
	panic("%s", __func__);

	refcnt = atomic_dec_uint_nv(&pm->pm_refcnt);

	if (refcnt > 0) {
		// no need to destroy

		// XXX
		return;
	}

	// destroy
	mutex_destroy(&pm->pm_lock);

	// XXX
}

long
pmap_resident_count(struct pmap *pm)
{
	DMARK();
	panic("%s", __func__);
	return pm->pm_stats.resident_count;
}

long
pmap_wired_count(struct pmap *pm)
{
	DMARK();
	panic("%s", __func__);
	return pm->pm_stats.wired_count;
}

int
pmap_enter(struct pmap *pm, vaddr_t va, paddr_t pa, vm_prot_t prot, u_int flags)
{
	DMARK();
	panic("%s", __func__);
	return 0;
}

void
pmap_remove(struct pmap *pm, vaddr_t sva, vaddr_t eva)
{
	DMARK();
	panic("%s", __func__);
}

void
pmap_remove_all(struct pmap *pm)
{
	DMARK();
	panic("%s", __func__);
}

void
pmap_unwire(struct pmap *pm, vaddr_t va)
{
	DMARK();
	panic("%s", __func__);
}

void
pmap_copy(struct pmap *dst_map, struct pmap *src_map, vaddr_t dst_addr,
    vsize_t len, vaddr_t src_addr)
{
	DMARK();
	panic("%s", __func__);
}

void
pmap_activate(struct lwp *l)
{
	DMARK();
	panic("%s", __func__);
}

void
pmap_deactivate(struct lwp *l)
{
	DMARK();
	panic("%s", __func__);
}

void
pmap_page_protect(struct vm_page *pg, vm_prot_t prot)
{
	DMARK();
	panic("%s", __func__);
}

bool
pmap_clear_modify(struct vm_page *pg)
{
	struct vm_page_md * const mdpg = VM_PAGE_TO_MD(pg);
	const bool rv = (mdpg->mdpg_attrs & VM_PAGE_MD_MODIFIED) != 0;

	DMARK();

	mdpg->mdpg_attrs &= ~VM_PAGE_MD_MODIFIED;
	return rv;
}

bool
pmap_clear_reference(struct vm_page *pg)
{
	struct vm_page_md * const mdpg = VM_PAGE_TO_MD(pg);
	const bool rv = (mdpg->mdpg_attrs & VM_PAGE_MD_REFERENCED) != 0;

	DMARK();

	mdpg->mdpg_attrs &= ~VM_PAGE_MD_REFERENCED;
	return rv;
}

bool
pmap_is_modified(struct vm_page *pg)
{
	struct vm_page_md * const mdpg = VM_PAGE_TO_MD(pg);

	DMARK();

	return (mdpg->mdpg_attrs & VM_PAGE_MD_MODIFIED) != 0;
}

bool
pmap_is_referenced(struct vm_page *pg)
{
	struct vm_page_md * const mdpg = VM_PAGE_TO_MD(pg);

	DMARK();

	return (mdpg->mdpg_attrs & VM_PAGE_MD_REFERENCED) != 0;
}

paddr_t
pmap_phys_address(paddr_t cookie)
{
	DMARK();
	return cookie;
}

#ifdef MORE_DEBUG
/*
 * Test ref/modify handling.  */
void pmap_testout(void);

void
pmap_testout(void)
{
	vaddr_t va;
	volatile int *loc;
	int val = 0;
	paddr_t pa;
	struct vm_page *pg;
	int ref, mod;

	/* Allocate a page */
	va = (vaddr_t)(VM_MAX_KERNEL_ADDRESS - PAGE_SIZE);
	KASSERT(va != 0);
	loc = (int*)va;

	pmap_extract(pmap_kernel(), va, &pa);
	pg = PHYS_TO_VM_PAGE(pa);
	pmap_kenter_pa(va, pa, VM_PROT_ALL, VM_PROT_ALL);
	pmap_update(pmap_kernel());

	/* Now clear reference and modify */
	ref = pmap_clear_reference(pg);
	mod = pmap_clear_modify(pg);
	printf("Clearing page va %p pa %lx: ref %d, mod %d\n",
	       (void *)(u_long)va, (long)pa,
	       ref, mod);

	/* Check it's properly cleared */
	ref = pmap_is_referenced(pg);
	mod = pmap_is_modified(pg);
	printf("Checking cleared page: ref %d, mod %d\n",
	       ref, mod);

	/* Reference page */
	val = *loc;

	ref = pmap_is_referenced(pg);
	mod = pmap_is_modified(pg);
	printf("Referenced page: ref %d, mod %d val %x\n",
	       ref, mod, val);

	/* Now clear reference and modify */
	ref = pmap_clear_reference(pg);
	mod = pmap_clear_modify(pg);
	printf("Clearing page va %p pa %lx: ref %d, mod %d\n",
	       (void *)(u_long)va, (long)pa,
	       ref, mod);

	/* Modify page */
	*loc = 1;

	ref = pmap_is_referenced(pg);
	mod = pmap_is_modified(pg);
	printf("Modified page: ref %d, mod %d\n",
	       ref, mod);

	/* Now clear reference and modify */
	ref = pmap_clear_reference(pg);
	mod = pmap_clear_modify(pg);
	printf("Clearing page va %p pa %lx: ref %d, mod %d\n",
	       (void *)(u_long)va, (long)pa,
	       ref, mod);

	/* Check it's properly cleared */
	ref = pmap_is_referenced(pg);
	mod = pmap_is_modified(pg);
	printf("Checking cleared page: ref %d, mod %d\n",
	       ref, mod);

	/* Modify page */
	*loc = 1;

	ref = pmap_is_referenced(pg);
	mod = pmap_is_modified(pg);
	printf("Modified page: ref %d, mod %d\n",
	       ref, mod);

	/* Check pmap_protect() */
	pmap_protect(pmap_kernel(), va, va+1, VM_PROT_READ);
	pmap_update(pmap_kernel());
	ref = pmap_is_referenced(pg);
	mod = pmap_is_modified(pg);
	printf("pmap_protect(VM_PROT_READ): ref %d, mod %d\n",
	       ref, mod);

	/* Now clear reference and modify */
	ref = pmap_clear_reference(pg);
	mod = pmap_clear_modify(pg);
	printf("Clearing page va %p pa %lx: ref %d, mod %d\n",
	       (void *)(u_long)va, (long)pa,
	       ref, mod);

	/* Modify page */
	pmap_kenter_pa(va, pa, VM_PROT_ALL, VM_PROT_ALL);
	pmap_update(pmap_kernel());
	*loc = 1;

	ref = pmap_is_referenced(pg);
	mod = pmap_is_modified(pg);
	printf("Modified page: ref %d, mod %d\n",
	       ref, mod);

	/* Check pmap_protect() */
	pmap_protect(pmap_kernel(), va, va+1, VM_PROT_NONE);
	pmap_update(pmap_kernel());
	ref = pmap_is_referenced(pg);
	mod = pmap_is_modified(pg);
	printf("pmap_protect(VM_PROT_READ): ref %d, mod %d\n",
	       ref, mod);

	/* Now clear reference and modify */
	ref = pmap_clear_reference(pg);
	mod = pmap_clear_modify(pg);
	printf("Clearing page va %p pa %lx: ref %d, mod %d\n",
	       (void *)(u_long)va, (long)pa,
	       ref, mod);

	/* Modify page */
	pmap_kenter_pa(va, pa, VM_PROT_ALL, VM_PROT_ALL);
	pmap_update(pmap_kernel());
	*loc = 1;

	ref = pmap_is_referenced(pg);
	mod = pmap_is_modified(pg);
	printf("Modified page: ref %d, mod %d\n",
	       ref, mod);

	/* Check pmap_pag_protect() */
	pmap_page_protect(pg, VM_PROT_READ);
	ref = pmap_is_referenced(pg);
	mod = pmap_is_modified(pg);
	printf("pmap_protect(): ref %d, mod %d\n",
	       ref, mod);

	/* Now clear reference and modify */
	ref = pmap_clear_reference(pg);
	mod = pmap_clear_modify(pg);
	printf("Clearing page va %p pa %lx: ref %d, mod %d\n",
	       (void *)(u_long)va, (long)pa,
	       ref, mod);

	/* Modify page */
	pmap_kenter_pa(va, pa, VM_PROT_ALL, VM_PROT_ALL);
	pmap_update(pmap_kernel());
	*loc = 1;

	ref = pmap_is_referenced(pg);
	mod = pmap_is_modified(pg);
	printf("Modified page: ref %d, mod %d\n",
	       ref, mod);

	/* Check pmap_pag_protect() */
	pmap_page_protect(pg, VM_PROT_NONE);
	ref = pmap_is_referenced(pg);
	mod = pmap_is_modified(pg);
	printf("pmap_protect(): ref %d, mod %d\n",
	       ref, mod);

	/* Now clear reference and modify */
	ref = pmap_clear_reference(pg);
	mod = pmap_clear_modify(pg);
	printf("Clearing page va %p pa %lx: ref %d, mod %d\n",
	       (void *)(u_long)va, (long)pa,
	       ref, mod);

	/* Unmap page */
	pmap_kremove(va, va+1);
	pmap_update(pmap_kernel());
	ref = pmap_is_referenced(pg);
	mod = pmap_is_modified(pg);
	printf("Unmapped page: ref %d, mod %d\n", ref, mod);

	/* Now clear reference and modify */
	ref = pmap_clear_reference(pg);
	mod = pmap_clear_modify(pg);
	printf("Clearing page va %p pa %lx: ref %d, mod %d\n",
	       (void *)(u_long)va, (long)pa, ref, mod);

	/* Check it's properly cleared */
	ref = pmap_is_referenced(pg);
	mod = pmap_is_modified(pg);
	printf("Checking cleared page: ref %d, mod %d\n",
	       ref, mod);

	pmap_kremove(va, va+1);
	pmap_update(pmap_kernel());
	/* pmap_free_page(pa, cpus_active); */
}
#endif
