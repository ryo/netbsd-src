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
#include <sys/msgbuf.h>

#include <uvm/uvm.h>

#include <aarch64/pmap.h>
#include <aarch64/pte.h>
#include <aarch64/armreg.h>

/* memory attributes are configured MAIR_EL1 in locore */
#define LX_BLKPAG_ATTR_NORMAL_WB	__SHIFTIN(0, LX_BLKPAG_ATTR_INDX)
#define LX_BLKPAG_ATTR_NORMAL_NC	__SHIFTIN(1, LX_BLKPAG_ATTR_INDX)
#define LX_BLKPAG_ATTR_NORMAL_WT	__SHIFTIN(2, LX_BLKPAG_ATTR_INDX)
#define LX_BLKPAG_ATTR_DEVICE_MEM	__SHIFTIN(3, LX_BLKPAG_ATTR_INDX)

static pt_entry_t *_pmap_pte_lookup(vaddr_t);
static pd_entry_t *_pmap_grow_l2(pd_entry_t *, vaddr_t);
static pt_entry_t *_pmap_grow_l3(pd_entry_t *, vaddr_t);

static struct pmap kernel_pmap;

struct pmap * const kernel_pmap_ptr = &kernel_pmap;
static vaddr_t pmap_maxkvaddr;
vmem_t *pmap_asid_arena;

vaddr_t virtual_avail, virtual_end;


static inline void
_pmap_invalidate_page(vaddr_t va)
{
	__asm __volatile ("dsb ishst; tlbi vaae1is, %0; dsb ish; isb" : :
	    "r" (atop(va) & 0xfffffffffff));
}

static inline void
_pmap_tlb_flush(void)
{
#ifdef MULTIPROCESSOR
	__asm __volatile ("tlbi vmalle1is; dsb ish; isb");
#else
	__asm __volatile ("tlbi vmalle1; dsb ish; isb");
#endif
}

void
pmap_bootstrap(vaddr_t vstart, vaddr_t vend)
{
	struct pmap *kpm = pmap_kernel();
	pd_entry_t *l0;
	void *p;

	virtual_avail = vstart;
	virtual_end = vend;

	_pmap_tlb_flush();

	pmap_maxkvaddr = vstart;

	memset(&kernel_pmap, 0, sizeof(kernel_pmap));
	kpm->pm_refcnt = 1;
	kpm->pm_l0table = l0 = AARCH64_PA_TO_KVA(reg_ttbr1_el1_read());
	kpm->pm_l1table = AARCH64_PA_TO_KVA(l0pde_pa(l0[l0pde_index(vstart)]));
	mutex_init(&kpm->pm_lock, MUTEX_DEFAULT, IPL_NONE);

	p = (void *)uvm_pageboot_alloc(MSGBUFSIZE);

	printf("uvm_pageboot_alloc(%d) returns %p\n", MSGBUFSIZE, p);
	printf("%s:%d: virtual_avail=%lx, virtual_end=%lx\n",
	    __func__, __LINE__, virtual_avail, virtual_end);

	/* Steal msgbuf area */
	initmsgbuf(p, MSGBUFSIZE);
}

void
pmap_init(void)
{
	printf("%s:%d\n", __func__, __LINE__);

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

//	printf("%s:%d: size=%lu, *vstartp=%lx, *vendp=%lx\n", __func__, __LINE__,
//	    size, *vstartp, *vendp);

	KDASSERT(!uvm.page_init_done);

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
//printf("%s:%d: pa=0x%016lx-0x%016lx: va=0x%016lx\n", __func__, __LINE__, pa, pa + npage * PAGE_SIZE, va);

	uvm_physseg_unplug(atop(pa), npage);

	for (; npage > 0; npage--, va += PAGE_SIZE)
		pmap_zero_page(pa);

	return va;
}

void
pmap_reference(struct pmap *pm)
{
	atomic_inc_uint(&pm->pm_refcnt);
}

static pd_entry_t *
_pmap_grow_l2(pd_entry_t *l1, vaddr_t vaddr)
{
	pd_entry_t l2;
	paddr_t pa;

//printf("%s:%d: l1=%p, vaddr=%lx\n", __func__, __LINE__, l1, vaddr);

	if (l1pde_valid(l1[l1pde_index(vaddr)])) {
//printf("%s:%d: l2 table exists: %016llx\n", __func__, __LINE__, l1[l1pde_index(vaddr)]);

		l2 = l1[l1pde_index(vaddr)];
		l2 = l1pde_pa(l2);
		l2 = AARCH64_PA_TO_KVA(l2);
	} else {
		pa = uvm_pageboot_alloc(Ln_TABLE_SIZE);	// XXX: use uvm_pagealloc
//printf("%s:%d: new l2table(PA)=%lx\n", __func__, __LINE__, pa);

		l1[l1pde_index(vaddr)] = pa |
		    LX_BLKPAG_AF |
#ifdef MULTIPROCESSOR
		    LX_BLKPAG_SH_IS |
#endif
		    LX_BLKPAG_ATTR_NORMAL_WB |
		    LX_BLKPAG_PXN |LX_BLKPAG_UXN |
		    L1_TABLE | LX_VALID;

		_pmap_invalidate_page(l1);

		l2 = AARCH64_PA_TO_KVA(pa);
	}
//printf("%s:%d: l2=%16llx\n", __func__, __LINE__, l2);
	return l2;
}

pt_entry_t *
_pmap_grow_l3(pd_entry_t *l2, vaddr_t vaddr)
{
	pd_entry_t l3;
	paddr_t pa;

//printf("%s:%d: l2=%p, vaddr=%lx\n", __func__, __LINE__, l2, vaddr);

	if (l2pde_valid(l2[l2pde_index(vaddr)])) {
//printf("%s:%d: l3 table exists: %016llx\n", __func__, __LINE__, l2[l2pde_index(vaddr)]);

		l3 = l2[l2pde_index(vaddr)];
		l3 = l2pde_pa(l3);
		l3 = AARCH64_PA_TO_KVA(l3);
	} else {

		/* XXX: uvm_pageboot_alloc() returns AARCH64 KSEG address */
		pa = AARCH64_KVA_TO_PA(uvm_pageboot_alloc(Ln_TABLE_SIZE));		// XXX: use uvm_pagealloc
//printf("%s:%d: new l3table(PA)=%lx\n", __func__, __LINE__, pa);

		l2[l2pde_index(vaddr)] = pa |
		    LX_BLKPAG_AF |
#ifdef MULTIPROCESSOR
		    LX_BLKPAG_SH_IS |
#endif
		    LX_BLKPAG_ATTR_NORMAL_WB |
		    LX_BLKPAG_PXN |LX_BLKPAG_UXN |
		    L2_TABLE | LX_VALID;

		_pmap_invalidate_page(l2);

		l3 = AARCH64_PA_TO_KVA(pa);
	}
//printf("%s:%d: l3=%16llx\n", __func__, __LINE__, l3);
	return l3;
}

vaddr_t
pmap_growkernel(vaddr_t maxkvaddr)
{
	pd_entry_t *l2;
	pt_entry_t *l3;
	vaddr_t kvaddr;
	int i, s;

	printf("%s:%d: maxkvaddr=%16lx, pmap_maxkvaddrr=%16lx\n",
	    __func__, __LINE__,
	    maxkvaddr, pmap_maxkvaddr);

	s = splvm();

	if (maxkvaddr <= pmap_maxkvaddr) {
		printf("%s: no need to expand l1/l2 table\n", __func__);
		splx(s);
		return pmap_maxkvaddr;
	}

	KDASSERT(maxkvaddr <= virtual_end);

	for (; pmap_maxkvaddr < maxkvaddr; pmap_maxkvaddr += L1_SIZE) {
		printf("%s: growing pmap_maxkvaddr=%16lx\n", __func__,
		    pmap_maxkvaddr);

		l2 = _pmap_grow_l2(pmap_kernel()->pm_l1table, pmap_maxkvaddr);
		KDASSERT(l1 != NULL);

		for (i = 0, kvaddr = pmap_maxkvaddr; i < Ln_ENTRIES;
		    i++, kvaddr += L2_SIZE) {
			l3 = _pmap_grow_l3(l2, kvaddr);
		}
		KDASSERT(l3 != NULL);
	}
	splx(s);

	printf("%s: done: pmap_maxkvaddr=%16lx\n", __func__, pmap_maxkvaddr);

	return pmap_maxkvaddr;
}

bool
pmap_extract(struct pmap *pm, vaddr_t va, paddr_t *pap)
{
	paddr_t pa;

	if (VM_MIN_KERNEL_ADDRESS <= va && va <= VM_MAX_KERNEL_ADDRESS) {
		pd_entry_t pde, *l2, *l3;

		/*
		 * traverse L0 -> L1 -> L2 -> L3 table
		 */
		pde = pmap_kernel()->pm_l1table[l1pde_index(va)];
		if (!l1pde_valid(pde))
			return false;
		if (l1pde_is_block(pde)) {
			pa = l1pde_pa(pde) + (va & L1_OFFSET);
			goto found;
		}
		if (!l1pde_is_table(pde))
			panic("%s: l1pde is not block nor table", __func__);

		l2 = AARCH64_PA_TO_KVA(l1pde_pa(pde));
		pde = l2[l2pde_index(va)];
		if (!l2pde_valid(pde))
			return false;
		if (l2pde_is_block(pde)) {
			pa = l2pde_pa(pde) + (va & L2_OFFSET);
			goto found;
		}
		if (!l2pde_is_table(pde))
			panic("%s: l2pde is not block nor table", __func__);

		l3 = AARCH64_PA_TO_KVA(l2pde_pa(pde));
		pde = l3[l3pte_index(va)];
		if (!l3pte_valid(pde))
			return false;
		if (!l3pte_is_page(pde))
			panic("%s: l3pde is not a page", __func__);

		pa = l3pte_pa(pde) + (va & L3_OFFSET);

	} else if ((va & AARCH64_KSEG_MASK) == AARCH64_KSEG_MASK) {
		pa = AARCH64_KVA_TO_PA(va);

	} else {
		return false;
	}

 found:
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

	if (VM_MIN_KERNEL_ADDRESS <= va && va <= VM_MAX_KERNEL_ADDRESS) {
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

	} else if ((va & AARCH64_KSEG_MASK) == AARCH64_KSEG_MASK) {
		panic("page entry is mapped in KSEG");
	} else {
		panic("Non kernel segment: va=0x%016lx", va);
	}

	return NULL;
}

void
pmap_kenter_pa(vaddr_t va, paddr_t pa, vm_prot_t prot, u_int flags)
{
	pt_entry_t *ptep, pte, attr;

//	printf("%s:%d: va=%016lx, pa=%016lx, prot=%08x, flags=%08x\n", __func__, __LINE__,
//	    va, pa, prot, flags);

	ptep = _pmap_pte_lookup(va);
	if (ptep == NULL)
		panic("%s: cannot lookup L3 PTE: 0x%016lx\n", __func__, va);

	switch (prot & (VM_PROT_READ|VM_PROT_WRITE)) {
	case 0:
	default:
		attr = 0;
		break;
	case VM_PROT_READ:
		attr = LX_BLKPAG_AF | LX_BLKPAG_AP_RO_NONE;
		break;
	case VM_PROT_WRITE:
	case VM_PROT_READ|VM_PROT_WRITE:
		attr = LX_BLKPAG_AF | LX_BLKPAG_AP_RW_NONE;
		break;
	}

	attr |= (prot & VM_PROT_EXECUTE) ? 0 : (LX_BLKPAG_UXN|LX_BLKPAG_PXN);

	switch (flags & PMAP_CACHE_MASK) {
	case PMAP_NOCACHE:
	case PMAP_NOCACHE_OVR:
#if 0
		attr |= LX_BLKPAG_ATTR_NORMAL_NC;	/* but reordering? */
#else
		attr |= LX_BLKPAG_ATTR_DEVICE_MEM;
#endif
		break;
	case PMAP_WRITE_COMBINE:
	case PMAP_WRITE_BACK:
	case 0:
		attr |= LX_BLKPAG_ATTR_NORMAL_WB;
		break;
	default:
		panic("%s: illegal cache flags=0x%08x(cache=%02x): va=%016lx, pa=0x%016lx",
		    __func__, flags, flags & PMAP_CACHE_MASK, va, pa);
	}

	pte = pa | attr |
#ifdef MULTIPROCESSOR
	    LX_BLKPAG_SH_IS |
#endif
	    LX_VALID | L3_TYPE_PAG;

//	printf("%s: ptep=%p, pte=0x%016llx\n", __func__, ptep, pte);

	*ptep = pte;

	_pmap_tlb_flush();
	_pmap_invalidate_page(ptep);
	_pmap_invalidate_page(va);
}

void
pmap_update(struct pmap *pm)
{
//	printf("%s:%d\n", __func__, __LINE__);
}

void
pmap_kremove(vaddr_t va, vsize_t size)
{
	printf("%s:%d\n", __func__, __LINE__);
	panic("%s", __func__);
}

struct pmap *
pmap_create(void)
{
	printf("%s:%d\n", __func__, __LINE__);
	panic("%s", __func__);
	return NULL;
}

void
pmap_destroy(struct pmap *pm)
{
	unsigned int refcnt;

	printf("%s:%d\n", __func__, __LINE__);
	panic("%s", __func__);

	if (__predict_true(pm->pm_refcnt == 1))
		refcnt = pm->pm_refcnt = 0;
	else
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
	printf("%s:%d\n", __func__, __LINE__);
	panic("%s", __func__);
	return pm->pm_stats.resident_count;
}

long
pmap_wired_count(struct pmap *pm)
{
	printf("%s:%d\n", __func__, __LINE__);
	panic("%s", __func__);
	return pm->pm_stats.wired_count;
}

int
pmap_enter(struct pmap *pm, vaddr_t va, paddr_t pa, vm_prot_t prot, u_int flags)
{
	printf("%s:%d\n", __func__, __LINE__);
	panic("%s", __func__);
	return 0;
}

void
pmap_remove(struct pmap *pm, vaddr_t sva, vaddr_t eva)
{
	printf("%s:%d\n", __func__, __LINE__);
	panic("%s", __func__);
}

void
pmap_remove_all(struct pmap *pm)
{
	printf("%s:%d\n", __func__, __LINE__);
	panic("%s", __func__);
}

void
pmap_protect(struct pmap *pm, vaddr_t sva, vaddr_t eva, vm_prot_t prot)
{
	printf("%s:%d\n", __func__, __LINE__);
	panic("%s", __func__);
}

void
pmap_unwire(struct pmap *pm, vaddr_t va)
{
	printf("%s:%d\n", __func__, __LINE__);
	panic("%s", __func__);
}

void
pmap_copy(struct pmap *dst_map, struct pmap *src_map, vaddr_t dst_addr,
    vsize_t len, vaddr_t src_addr)
{
	printf("%s:%d\n", __func__, __LINE__);
	panic("%s", __func__);
}

void
pmap_activate(struct lwp *l)
{
	printf("%s:%d\n", __func__, __LINE__);
	panic("%s", __func__);
}

void
pmap_deactivate(struct lwp *l)
{
	printf("%s:%d\n", __func__, __LINE__);
	panic("%s", __func__);
}

void
pmap_page_protect(struct vm_page *pg, vm_prot_t prot)
{
	printf("%s:%d\n", __func__, __LINE__);
	panic("%s", __func__);
}

bool
pmap_clear_modify(struct vm_page *pg)
{
	struct vm_page_md * const mdpg = VM_PAGE_TO_MD(pg);
	const bool rv = (mdpg->mdpg_attrs & VM_PAGE_MD_MODIFIED) != 0;

	printf("%s:%d\n", __func__, __LINE__);

	mdpg->mdpg_attrs &= ~VM_PAGE_MD_MODIFIED;
	return rv;
}

bool
pmap_clear_reference(struct vm_page *pg)
{
	struct vm_page_md * const mdpg = VM_PAGE_TO_MD(pg);
	const bool rv = (mdpg->mdpg_attrs & VM_PAGE_MD_REFERENCED) != 0;

	printf("%s:%d\n", __func__, __LINE__);

	mdpg->mdpg_attrs &= ~VM_PAGE_MD_REFERENCED;
	return rv;
}

bool
pmap_is_modified(struct vm_page *pg)
{
	struct vm_page_md * const mdpg = VM_PAGE_TO_MD(pg);

	printf("%s:%d\n", __func__, __LINE__);

	return (mdpg->mdpg_attrs & VM_PAGE_MD_MODIFIED) != 0;
}

bool
pmap_is_referenced(struct vm_page *pg)
{
	struct vm_page_md * const mdpg = VM_PAGE_TO_MD(pg);

	printf("%s:%d\n", __func__, __LINE__);

	return (mdpg->mdpg_attrs & VM_PAGE_MD_REFERENCED) != 0;
}

paddr_t
pmap_phys_address(paddr_t cookie)
{
	printf("%s:%d\n", __func__, __LINE__);
	return cookie;
}
