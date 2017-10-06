/*	$NetBSD$	*/

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
__KERNEL_RCSID(0, "$NetBSD$");

#include "opt_arm_debug.h"
#include "opt_ddb.h"
#include "opt_uvmhist.h"

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

//#define PMAP_DEBUG
//#define PMAP_PV_DEBUG


UVMHIST_DEFINE(pmaphist);
#ifdef UVMHIST

#ifndef UVMHIST_PMAPHIST_SIZE
#define UVMHIST_PMAPHIST_SIZE	8192
#endif

static struct kern_history_ent pmaphistbuf[UVMHIST_PMAPHIST_SIZE];

static void
pmap_hist_init(void)
{
	static bool inited = false;
	if (inited == false) {
		UVMHIST_INIT_STATIC(pmaphist, pmaphistbuf);
		inited = true;
	}
}
#define PMAP_HIST_INIT()	pmap_hist_init()

#else /* UVMHIST */

#define PMAP_HIST_INIT()	((void)0)

#endif /* UVMHIST */


#ifdef PMAP_DEBUG
#define DPRINTF(format, args...)	\
	printf("%s:%d: " format , __func__, __LINE__, args)
#else
#define DPRINTF(args...)
#endif

#define dprintf(format, args...)	\
	printf("%s:%d: " format , __func__, __LINE__, args)


/* saved permission bit for referenced/modified emulation */
#define LX_BLKPAG_OS_READ	LX_BLKPAG_OS_0
#define LX_BLKPAG_OS_WRITE	LX_BLKPAG_OS_1
#define LX_BLKPAG_OS_RWMASK	(LX_BLKPAG_OS_WRITE|LX_BLKPAG_OS_READ)

/* memory attributes are configured MAIR_EL1 in locore */
#define LX_BLKPAG_ATTR_NORMAL_WB	__SHIFTIN(0, LX_BLKPAG_ATTR_INDX)
#define LX_BLKPAG_ATTR_NORMAL_NC	__SHIFTIN(1, LX_BLKPAG_ATTR_INDX)
#define LX_BLKPAG_ATTR_NORMAL_WT	__SHIFTIN(2, LX_BLKPAG_ATTR_INDX)
#define LX_BLKPAG_ATTR_DEVICE_MEM	__SHIFTIN(3, LX_BLKPAG_ATTR_INDX)
#define LX_BLKPAG_ATTR_MASK		LX_BLKPAG_ATTR_INDX

struct pv_entry {
	TAILQ_ENTRY(pv_entry) pv_link;
	struct pmap *pv_pmap;
	vaddr_t pv_va;
	paddr_t pv_pa;
	pt_entry_t *pv_ptep;	/* for fast pte lookup */
};

struct pmap {
	kmutex_t pm_lock;
	int pm_s;
	struct pool *pm_pvpool;
	pd_entry_t *pm_l0table;	/* L0 table: 512G*512 */
	paddr_t pm_l0table_pa;

	pool_cache_t pm_pdp_cache;	/* for userspace L[0123] tables */

	struct pmap_statistics pm_stats;
	unsigned int pm_refcnt;
	int pm_asid;
	bool pm_activated;
};

static pt_entry_t *_pmap_pte_lookup(struct pmap *, vaddr_t);
static pt_entry_t _pmap_pte_adjust_prot(pt_entry_t, vm_prot_t, vm_prot_t);
static pt_entry_t _pmap_pte_adjust_cacheflags(pt_entry_t, u_int);
static void _pmap_remove(struct pmap *, vaddr_t, bool);
static int _pmap_enter(struct pmap *, vaddr_t, paddr_t, vm_prot_t, u_int, bool);

static struct pmap kernel_pmap;

struct pmap * const kernel_pmap_ptr = &kernel_pmap;
static vaddr_t pmap_maxkvaddr;

vaddr_t virtual_avail, virtual_end;
vaddr_t virtual_devmap_addr;

static struct pool_cache _pmap_cache;
static struct pool_cache _pmap_pv_pool;

static inline void
pmap_pv_lock(struct vm_page_md *md __unused)
{
	mutex_enter(&md->mdpg_pvlock);
}
static inline void
pmap_pv_unlock(struct vm_page_md *md __unused)
{
	mutex_exit(&md->mdpg_pvlock);
}

#define PM_LOCK(pm)				\
	do {					\
		mutex_enter(&(pm)->pm_lock);	\
	} while (0 /*CONSTCOND*/)
#define PM_UNLOCK(pm)				\
	do {					\
		mutex_exit(&(pm)->pm_lock);	\
	} while (0 /*CONSTCOND*/)

/* XXX: for debug */
static char * __unused
str_vmflags(uint32_t flags)
{
	static int idx = 0;
	static char buf[4][32];	/* XXX */
	char *p;

	p = buf[idx];
	idx = (idx + 1) & 3;

	p[0] = (flags & VM_PROT_READ) ? 'R' : '-';
	p[1] = (flags & VM_PROT_WRITE) ? 'W' : '-';
	p[2] = (flags & VM_PROT_EXECUTE) ? 'X' : '-';
	if (flags & PMAP_WIRED)
		memcpy(&p[3], ",WIRED\0", 7);
	else
		p[3] = '\0';

	return p;
}

static void __unused
pm_addr_check(struct pmap *pm, vaddr_t va, const char *prefix)
{
	if (pm == pmap_kernel()) {
		if (VM_MIN_KERNEL_ADDRESS <= va && va < VM_MAX_KERNEL_ADDRESS) {
			// ok
		} else {
			printf("%s: kernel pm %p: va=%016lx is not kernel address\n",
			    prefix, pm, va);
			panic("pm_addr_check");
		}
	} else {
		if (VM_MIN_ADDRESS <= va && va <= VM_MAX_ADDRESS) {
			// ok
		} else {
			printf("%s: usef pm %p: va=%016lx is not kernel address\n",
			    prefix, pm, va);
			panic("pm_addr_check");
		}
	}
}
#define PM_ADDR_CHECK(pm, va)		pm_addr_check(pm, va, __func__)

#define IN_KSEG_ADDR(va)	((AARCH64_KSEG_START <= (va)) && ((va) < AARCH64_KSEG_END))



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

	attr = _pmap_pte_adjust_prot(L2_BLOCK, prot, VM_PROT_ALL);
	attr = _pmap_pte_adjust_cacheflags(attr, flags);
#ifdef MULTIPROCESSOR
	attr |= LX_BLKPAG_SH_IS;
#endif

	resid = (size + (L2_SIZE - 1)) & ~(L2_SIZE - 1);
	size = resid;

	while (resid > 0) {
		pt_entry_t pte;

		pte = pa | attr;

		atomic_swap_64(&l2[l2pde_index(va)], pte);
		aarch64_tlbi_by_va(va);

		if (prot & VM_PROT_EXECUTE)
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
	pd_entry_t *l0;
	vaddr_t va;
	paddr_t l0pa;

	PMAP_HIST_INIT();	/* init once */

	UVMHIST_FUNC(__func__);
	UVMHIST_CALLED(pmaphist);

	/* devmap already uses last of va? */
	if ((virtual_devmap_addr != 0) && (virtual_devmap_addr < vend))
		vend = virtual_devmap_addr;

	virtual_avail = vstart;
	virtual_end = vend;
	pmap_maxkvaddr = vstart;

	{
		/* clean cache modified PA=VA */
		void cpucache_clean(void);
		cpucache_clean();
	}
	aarch64_tlbi_all();

	va = vstart;
	l0pa = reg_ttbr1_el1_read();
	l0 = AARCH64_PA_TO_KVA(l0pa);

	memset(&kernel_pmap, 0, sizeof(kernel_pmap));
	kpm = pmap_kernel();
	kpm->pm_refcnt = 1;
	kpm->pm_l0table = l0;
	kpm->pm_l0table_pa = l0pa;
	kpm->pm_activated = true;

	mutex_init(&kpm->pm_lock, MUTEX_DEFAULT, IPL_VM);
}

static int
_pmap_pdp_ctor(void *arg, void *v, int flags)
{
	memset(v, 0, Ln_TABLE_SIZE);
	return 0;
}

static int
_pmap_pmap_ctor(void *arg, void *v, int flags)
{
	memset(v, 0, sizeof(struct pmap));
	return 0;
}

static int
_pmap_pv_ctor(void *arg, void *v, int flags)
{
	memset(v, 0, sizeof(struct pv_entry));
	return 0;
}

void
pmap_init(void)
{
	pool_cache_bootstrap(&_pmap_cache, sizeof(struct pmap),
	    0, 0, 0, "pmappl", NULL, IPL_NONE, _pmap_pmap_ctor, NULL, NULL);
	pool_cache_bootstrap(&_pmap_pv_pool, sizeof(struct pv_entry),
	    0, 0, 0, "pvpl", NULL, IPL_NONE, _pmap_pv_ctor, NULL, NULL);
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

	UVMHIST_FUNC(__func__);
	UVMHIST_CALLED(pmaphist);

	UVMHIST_LOG(pmaphist, "size=%llu, *vstartp=%llx, *vendp=%llx", size, *vstartp, *vendp, 0);

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

	for (; npage > 0; npage--, pa += PAGE_SIZE) {
		pmap_zero_page(pa);
#if 1 /* VIPT */
		/* pmap_zero_page use KSEG address */
		cpu_dcache_wbinv_range(AARCH64_PA_TO_KVA(pa), PAGE_SIZE);
#endif
	}

	return va;
}

void
pmap_reference(struct pmap *pm)
{
	atomic_inc_uint(&pm->pm_refcnt);
}

static pd_entry_t *
_pmap_alloc_pdp_kernel(struct pmap *pm, paddr_t *pap)
{
	paddr_t pa;

	UVMHIST_FUNC(__func__);
	UVMHIST_CALLED(pmaphist);

	KASSERT(pm == pmap_kernel());
	if (uvm.page_init_done) {
		struct vm_page *pg;

		pg = uvm_pagealloc(NULL, 0, NULL,
		    UVM_PGA_USERESERVE | UVM_PGA_ZERO);
		if (pg == NULL)
			panic("%s: cannot allocate L3 table", __func__);
		pa = VM_PAGE_TO_PHYS(pg);
	} else {
		/* uvm_pageboot_alloc() returns AARCH64 KSEG address */
		pa = AARCH64_KVA_TO_PA(
		    uvm_pageboot_alloc(Ln_TABLE_SIZE));
	}
	if (pap != NULL)
		*pap = pa;

	UVMHIST_LOG(pmaphist, "pa=%llx, va=%llx", pa, AARCH64_PA_TO_KVA(pa), 0, 0);

	return AARCH64_PA_TO_KVA(pa);
}

static inline pd_entry_t *
_pmap_alloc_pdp_cache(struct pmap *pm, paddr_t *pap)
{
	void *p;

	UVMHIST_FUNC(__func__);
	UVMHIST_CALLED(pmaphist);

	p = pool_cache_get_paddr(pm->pm_pdp_cache, PR_NOWAIT, pap);

	UVMHIST_LOG(pmaphist, "pa=%llx, va=%llx", *pap, p, 0, 0);

	return p;
}

/* not used. usually, freed by pool_cache_destroy() at pmap_destroy() */
static inline void __unused
_pmap_free_pdp_cache(struct pmap *pm, pd_entry_t *pdp)
{
	pool_cache_put(pm->pm_pdp_cache, pdp);
}

static inline pd_entry_t *
pmap_alloc_pdp(struct pmap *pm, paddr_t *pap)
{
	if (pm == pmap_kernel())
		return _pmap_alloc_pdp_kernel(pm, pap);
	return _pmap_alloc_pdp_cache(pm, pap);
}

vaddr_t
pmap_growkernel(vaddr_t maxkvaddr)
{
	UVMHIST_FUNC(__func__);
	UVMHIST_CALLED(pmaphist);

	UVMHIST_LOG(pmaphist, "maxkvaddr=%llx, pmap_maxkvaddr=%llx", maxkvaddr, pmap_maxkvaddr, 0, 0);

	pmap_maxkvaddr = maxkvaddr;

	return maxkvaddr;
}

bool
pmap_extract(struct pmap *pm, vaddr_t va, paddr_t *pap)
{
	static pt_entry_t *ptep;
	paddr_t pa;

#if 0
	PM_ADDR_CHECK(pm, va);
#else
	if (((pm == pmap_kernel()) &&
	    !(VM_MIN_KERNEL_ADDRESS <= va && va < VM_MAX_KERNEL_ADDRESS) &&
	    !(AARCH64_KSEG_START <= va && va < AARCH64_KSEG_END)) ||
	    ((pm != pmap_kernel()) &&
	    !(VM_MIN_ADDRESS <= va && va <= VM_MAX_ADDRESS)))
		return false;
#endif

	PM_LOCK(pm);

	if (AARCH64_KSEG_START <= va && va < AARCH64_KSEG_END) {
		pa = AARCH64_KVA_TO_PA(va);
	} else {
		pt_entry_t pte;

		ptep = _pmap_pte_lookup(pm, va);
		if (ptep == NULL) {
			pd_entry_t pde, *l1, *l2, *l3;

			/*
			 * traverse L0 -> L1 -> L2 -> L3 table
			 * with considering block
			 */
			pde = pm->pm_l0table[l0pde_index(va)];
			if (!l0pde_valid(pde)) {
				goto notfound;
			}

			l1 = AARCH64_PA_TO_KVA(l0pde_pa(pde));
			pde = l1[l1pde_index(va)];
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

		} else {
			pte = *ptep;
		}
		if (!l3pte_valid(pte))
			goto notfound;

		KASSERT(l3pte_is_page(pte));
		pa = l3pte_pa(pte) + (va & L3_OFFSET);
	}
 found:
	PM_UNLOCK(pm);
	if (pap != NULL)
		*pap = pa;
	return true;

 notfound:
	PM_UNLOCK(pm);
	return false;
}

paddr_t
vtophys(vaddr_t va)
{
	paddr_t pa;

	if (pmap_extract(pmap_kernel(), va, &pa) == false)
		return VTOPHYS_FAILED;

	return pa;
}

static pt_entry_t *
_pmap_pte_lookup(struct pmap *pm, vaddr_t va)
{
	if (AARCH64_KSEG_START <= va && va < AARCH64_KSEG_END) {
		panic("page entry is mapped in KSEG");
	} else {
		pd_entry_t *l0, *l1, *l2, *l3;
		pd_entry_t pde;
		pt_entry_t *ptep;
		unsigned int idx;

		/*
		 * traverse L0 -> L1 -> L2 -> L3 table
		 */
		l0 = pm->pm_l0table;

		idx = l0pde_index(va);
		pde = l0[idx];
		if (!l0pde_valid(pde))
			return NULL;

		l1 = AARCH64_PA_TO_KVA(l0pde_pa(pde));
		idx = l1pde_index(va);
		pde = l1[idx];
		if (!l1pde_valid(pde))
			return NULL;

		if (l1pde_is_block(pde))
			return NULL;

		l2 = AARCH64_PA_TO_KVA(l1pde_pa(pde));
		idx = l2pde_index(va);
		pde = l2[idx];
		if (!l2pde_valid(pde))
			return NULL;
		if (l2pde_is_block(pde))
			return NULL;

		l3 = AARCH64_PA_TO_KVA(l2pde_pa(pde));
		idx = l3pte_index(va);
		ptep = &l3[idx];	/* as PTE */

		return ptep;
	}

	return NULL;
}

static pt_entry_t
_pmap_pte_adjust_prot(pt_entry_t pte, vm_prot_t prot, vm_prot_t protmask)
{
	vm_prot_t masked;

	masked = prot & protmask;
	pte &= ~(LX_BLKPAG_OS_RWMASK|LX_BLKPAG_AF|LX_BLKPAG_AP);

	/* keep prot for ref/mod emulation */
	switch (prot & (VM_PROT_READ|VM_PROT_WRITE)) {
	case 0:
	default:
		break;
	case VM_PROT_READ:
		pte |= LX_BLKPAG_OS_READ;
		break;
	case VM_PROT_WRITE:
	case VM_PROT_READ|VM_PROT_WRITE:
		pte |= (LX_BLKPAG_OS_READ|LX_BLKPAG_OS_WRITE);
		break;
	}

	switch (masked & (VM_PROT_READ|VM_PROT_WRITE)) {
	case 0:
	default:
		/* cannot access due to No LX_BLKPAG_AF */
		pte |= LX_BLKPAG_AP_RO;
		break;
	case VM_PROT_READ:
		/* actual permission of pte */
		pte |= LX_BLKPAG_AF;
		pte |= LX_BLKPAG_AP_RO;
		break;
	case VM_PROT_WRITE:
	case VM_PROT_READ|VM_PROT_WRITE:
		/* actual permission of pte */
		pte |= LX_BLKPAG_AF;
		pte |= LX_BLKPAG_AP_RW;
		break;
	}

	if ((prot & VM_PROT_EXECUTE) == 0)
		pte |= (LX_BLKPAG_UXN|LX_BLKPAG_PXN);
	else
		pte &= ~(LX_BLKPAG_UXN|LX_BLKPAG_PXN);

	return pte;
}

static pt_entry_t
_pmap_pte_adjust_cacheflags(pt_entry_t pte, u_int flags)
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

#define IS_OWNER_PV(pg, pmap, va)					\
	((VM_PAGE_TO_MD((pg))->mdpg_pa_owner != NULL) &&		\
	 (VM_PAGE_TO_MD((pg))->mdpg_pa_owner->pv_pmap == (pmap)) &&	\
	 (VM_PAGE_TO_MD((pg))->mdpg_pa_owner->pv_va == (va)))

static void
_pmap_remove_pv(struct vm_page *pg, struct pmap *pm, vaddr_t va, pt_entry_t pte)
{
	struct vm_page_md *md;
	struct pv_entry *pv, *owner;

	UVMHIST_FUNC(__func__);
	UVMHIST_CALLED(pmaphist);

	UVMHIST_LOG(pmaphist, "pg=%p, pm=%p, va=%llx, pte=%llx", pg, pm, va, pte);

	md = VM_PAGE_TO_MD(pg);

	pmap_pv_lock(md);
#if 1 /* VIPT */
	owner = md->mdpg_pa_owner;
	if ((owner != NULL) &&
	    (owner->pv_pmap == pm) &&
	    (owner->pv_va == va)) {
		/* pa owner is me */
		pv = owner;
		md->mdpg_pa_owner = NULL;

		/* for unmap writable page */
		if ((pte & (LX_BLKPAG_AF|LX_BLKPAG_AP)) ==
		    (LX_BLKPAG_AF|LX_BLKPAG_AP_RW)) {
			KASSERT(owner != NULL);
			cpu_dcache_wbinv_range(va, PAGE_SIZE);
		}
	} else
#endif
	{
		/* find pv */
		TAILQ_FOREACH(pv, &md->mdpg_pvhead, pv_link) {
			if ((pm == pv->pv_pmap) && (va == pv->pv_va)) {
				break;
			}
		}
		KASSERT(pv != NULL);
	}

	TAILQ_REMOVE(&md->mdpg_pvhead, pv, pv_link);
	md->mdpg_pvnum--;

	/* no one map this pa */
	if (md->mdpg_pvnum == 0)
		md->mdpg_flags &= ~PMAP_WIRED;

	pmap_pv_unlock(md);

	pool_cache_put(&_pmap_pv_pool, pv);
}

#if defined(PMAP_PV_DEBUG) || defined(DDB)
static void
pv_dump(struct vm_page_md *md, void (*pr)(const char *, ...))
{
	struct pv_entry *pv;
	int i;

	i = 0;

	pr("md=%p\n", md);
	pr(" md->mdpg_flags=%08x %s\n", md->mdpg_flags,
	    str_vmflags(md->mdpg_flags));
	pr(" md->mdpg_pvnum=%d\n", md->mdpg_pvnum);
	pr(" md->mdpg_pa_owner=%p%s\n", md->mdpg_pa_owner, (md->mdpg_pa_owner != NULL) ? " *" : "");

	TAILQ_FOREACH(pv, &md->mdpg_pvhead, pv_link) {
		pr("%c pv[%d] pv=%p\n",
		    (pv == md->mdpg_pa_owner) ? '*' : ' ',
		    i, pv);
		pr("    pv[%d].pv_pmap =%p (asid=%d)\n", i, pv->pv_pmap, pv->pv_pmap->pm_asid);
		pr("    pv[%d].pv_va   =%016lx\n", i, pv->pv_va);
		i++;
	}
}
#endif /* PMAP_PV_DEBUG & DDB */

static void
_pmap_enter_pv(struct vm_page *pg, struct pmap *pm, vaddr_t va, pt_entry_t *ptep,
    paddr_t pa, u_int flags)
{
	struct vm_page_md *md;
	struct pv_entry *pv;

	UVMHIST_FUNC(__func__);
	UVMHIST_CALLED(pmaphist);

	UVMHIST_LOG(pmaphist, "pg=%p, pm=%p, va=%llx, pa=%llx", pg, pm, va, pa);
	UVMHIST_LOG(pmaphist, "ptep=%p, flags=%08x", ptep, flags, 0, 0);

	md = VM_PAGE_TO_MD(pg);

	/* pv is already registered? */
	pmap_pv_lock(md);
	TAILQ_FOREACH(pv, &md->mdpg_pvhead, pv_link) {
		if ((pm == pv->pv_pmap) && (va == pv->pv_va)) {
			break;
		}
	}
	pmap_pv_unlock(md);

	if (pv == NULL) {
		pv = pool_cache_get(&_pmap_pv_pool, PR_WAITOK);
		pv->pv_pmap = pm;
		pv->pv_va = va;
		pv->pv_pa = pa;
		pv->pv_ptep = ptep;

		pmap_pv_lock(md);
		TAILQ_INSERT_HEAD(&md->mdpg_pvhead, pv, pv_link);
		if (md->mdpg_pvnum++ == 0) {
			/* become a pa owner */
			md->mdpg_pa_owner = pv;
		}
#ifdef PMAP_PV_DEBUG
		if (md->mdpg_pvnum > 1) {
			dprintf("pv %p alias added va=%016lx -> pa=%016lx\n", pv, va, pa);
			pv_dump(md, printf);
		}
#endif
		pmap_pv_unlock(md);
	}
}

void
pmap_kenter_pa(vaddr_t va, paddr_t pa, vm_prot_t prot, u_int flags)
{
	int s;

	KASSERT((prot & VM_PROT_READ) || !(prot & VM_PROT_WRITE));


	s = splvm();
	_pmap_enter(pmap_kernel(), va, pa, prot, flags | PMAP_WIRED, true);
	splx(s);
}

void
pmap_update(struct pmap *pm)
{
	/* nothing to do */
}

void
pmap_kremove(vaddr_t va, vsize_t size)
{
	struct pmap *kpm = pmap_kernel();
	vaddr_t eva;

	UVMHIST_FUNC(__func__);
	UVMHIST_CALLED(pmaphist);

	UVMHIST_LOG(pmaphist, "va=%llx, size=%llx", va, size, 0, 0);

	KDASSERT((va & PGOFSET) == 0);
	KDASSERT((size & PGOFSET) == 0);

	KASSERT(!IN_KSEG_ADDR(va));

	eva = va + size;
	KDASSERT(VM_MIN_KERNEL_ADDRESS <= va && eva < VM_MAX_KERNEL_ADDRESS);

	for (; va < eva; va += PAGE_SIZE) {
		_pmap_remove(kpm, va, true);
	}
}

static void
_pmap_protect_pv(struct vm_page *pg, struct pv_entry *pv, vm_prot_t prot)
{
	pt_entry_t *ptep, pte;
	vm_prot_t pteprot;
	uint32_t mdattr;

	UVMHIST_FUNC(__func__);
	UVMHIST_CALLED(pmaphist);

	UVMHIST_LOG(pmaphist, "pg=%p, pv=%p, prot=%08x", pg, pv, prot, 0);

#if 1 // VIPT
	pmap_pv_lock(VM_PAGE_TO_MD(pg));
	if (IS_OWNER_PV(pg, pv->pv_pmap, pv->pv_va)) {
		mdattr = VM_PAGE_TO_MD(pg)->mdpg_flags &
		    (VM_PROT_READ | VM_PROT_WRITE);
	} else {
		mdattr = 0;
	}
	pmap_pv_unlock(VM_PAGE_TO_MD(pg));
#else
	mdattr = VM_PAGE_TO_MD(pg)->mdpg_flags &
	    (VM_PROT_READ | VM_PROT_WRITE);
#endif

	PM_LOCK(pv->pv_pmap);

	ptep = pv->pv_ptep;
	pte = *ptep;

	pteprot = 0;
	if (pte & LX_BLKPAG_OS_READ)
		pteprot |= VM_PROT_READ;
	if (pte & LX_BLKPAG_OS_WRITE)
		pteprot |= VM_PROT_WRITE;
	if ((pte & (LX_BLKPAG_UXN|LX_BLKPAG_PXN)) == 0)
		pteprot |= VM_PROT_EXECUTE;

	pte = _pmap_pte_adjust_prot(pte, prot & pteprot, mdattr);
	atomic_swap_64(ptep, pte);

	aarch64_tlbi_by_va(pv->pv_va);

	PM_UNLOCK(pv->pv_pmap);
}

void
pmap_protect(struct pmap *pm, vaddr_t sva, vaddr_t eva, vm_prot_t prot)
{
	vaddr_t va;

	KASSERT((prot & VM_PROT_READ) || !(prot & VM_PROT_WRITE));

	UVMHIST_FUNC(__func__);
	UVMHIST_CALLED(pmaphist);

	UVMHIST_LOG(pmaphist, "pm=%p, sva=%016lx, eva=%016lx, prot=%08x", pm, sva, eva, prot);

	PM_ADDR_CHECK(pm, sva);

	KASSERT(!IN_KSEG_ADDR(sva));

	if ((prot & VM_PROT_READ) == VM_PROT_NONE) {
		pmap_remove(pm, sva, eva);
		return;
	}

	KDASSERT((sva & PAGE_MASK) == 0);
	KDASSERT((eva & PAGE_MASK) == 0);

	PM_LOCK(pm);

	for (va = sva; va < eva; va += PAGE_SIZE) {
		pt_entry_t *ptep, pte;
		struct vm_page *pg;
		paddr_t pa;
		uint32_t mdattr;

		ptep = _pmap_pte_lookup(pm, va);
		if (ptep == NULL)
			continue;

		pte = *ptep;

		if (!l3pte_valid(pte))
			continue;

		pa = l3pte_pa(pte);
		pg = PHYS_TO_VM_PAGE(pa);

		if (pg != NULL) {
#if 1 // VIPT
			pmap_pv_lock(VM_PAGE_TO_MD(pg));
			if (IS_OWNER_PV(pg, pm, va)) {
				mdattr = VM_PAGE_TO_MD(pg)->mdpg_flags &
				    (VM_PROT_READ | VM_PROT_WRITE);
			} else {
				mdattr = 0;
			}
			pmap_pv_unlock(VM_PAGE_TO_MD(pg));
#else
			mdattr = VM_PAGE_TO_MD(pg)->mdpg_flags &
				    (VM_PROT_READ | VM_PROT_WRITE);
#endif
		} else {
			mdattr = VM_PROT_ALL;
		}

		pte = *ptep;
		pte = _pmap_pte_adjust_prot(pte, prot, mdattr);
		atomic_swap_64(ptep, pte);

		aarch64_tlbi_by_va(va);
	}

	PM_UNLOCK(pm);
}

void
pmap_activate(struct lwp *l)
{
	struct pmap *pm = l->l_proc->p_vmspace->vm_map.pmap;
	uint64_t ttbr0;

	UVMHIST_FUNC(__func__);
	UVMHIST_CALLED(pmaphist);

	if (pm == pmap_kernel())
		return;

	KASSERT(pm->pm_l0table != NULL);

	UVMHIST_LOG(pmaphist, "lwp=%p (pid=%d)", l, l->l_proc->p_pid, 0, 0);

	/* XXXAARCH64 */
	if (pm->pm_asid == -1)
		pm->pm_asid = l->l_proc->p_pid;

	ttbr0 = ((uint64_t)pm->pm_asid << 48) | pm->pm_l0table_pa;
	aarch64_set_ttbr0(ttbr0);

	pm->pm_activated = true;
}

void
pmap_deactivate(struct lwp *l)
{
	struct pmap *pm = l->l_proc->p_vmspace->vm_map.pmap;

	UVMHIST_FUNC(__func__);
	UVMHIST_CALLED(pmaphist);

	if (pm == pmap_kernel())
		return;

	UVMHIST_LOG(pmaphist, "lwp=%p, asid=%d", l, pm->pm_asid, 0, 0);

	//XXXAARCH64
	pm->pm_activated = false;
}

struct pmap *
pmap_create(void)
{
	struct pmap *pm;

	UVMHIST_FUNC(__func__);
	UVMHIST_CALLED(pmaphist);

	pm = pool_cache_get(&_pmap_cache, PR_WAITOK);
	pm->pm_refcnt = 1;
	pm->pm_asid = -1;

	pm->pm_pdp_cache = pool_cache_init(Ln_TABLE_SIZE, 0, 0, PR_NOALIGN,
	    "pdppl", NULL, IPL_NONE, _pmap_pdp_ctor, NULL, NULL);

	pm->pm_l0table = _pmap_alloc_pdp_cache(pm, &pm->pm_l0table_pa);

	KASSERT(((vaddr_t)pm->pm_l0table & (PAGE_SIZE - 1)) == 0);
	KASSERT(pm->pm_l0table_pa != POOL_PADDR_INVALID);

	mutex_init(&pm->pm_lock, MUTEX_DEFAULT, IPL_NONE);

	UVMHIST_LOG(pmaphist, "pm=%p, pm_l0table=%016lx, pm_l0table_pa=%016lx", pm, pm->pm_l0table, pm->pm_l0table_pa, 0);


	return pm;
}

void
pmap_destroy(struct pmap *pm)
{
	unsigned int refcnt;

	UVMHIST_FUNC(__func__);
	UVMHIST_CALLED(pmaphist);

	UVMHIST_LOG(pmaphist, "pm=%p, pm_l0table=%016lx, pm_l0table_pa=%016lx, refcnt=%d", pm, pm->pm_l0table, pm->pm_l0table_pa, pm->pm_refcnt);

	if (pm == NULL)
		return;

	if (pm == pmap_kernel())
		panic("cannot destroy kernel pmap");

	refcnt = atomic_dec_uint_nv(&pm->pm_refcnt);
	if (refcnt > 0)
		return;

	pool_cache_destroy(pm->pm_pdp_cache);
	mutex_destroy(&pm->pm_lock);
	pool_cache_put(&_pmap_cache, pm);
}

static int
_pmap_enter(struct pmap *pm, vaddr_t va, paddr_t pa, vm_prot_t prot, u_int flags, bool kenter)
{
	struct vm_page *pg;
	pd_entry_t pde;
	pt_entry_t attr, pte, *ptep;
	pd_entry_t *l0, *l1, *l2, *l3;
	paddr_t pdppa;
	uint32_t mdattr;
	unsigned int idx;


	UVMHIST_FUNC(__func__);
	UVMHIST_CALLED(pmaphist);

	UVMHIST_LOG(pmaphist, "pm=%p, kentermode=%d", pm, kenter, 0, 0);
	UVMHIST_LOG(pmaphist, "va=%016lx, pa=%016lx, prot=%08x, flags=%08x", va, pa, prot, flags);


	PM_ADDR_CHECK(pm, va);

	pg = PHYS_TO_VM_PAGE(pa);
	if (pg == NULL) {
		//XXXAARCH64
		panic("%s:%d", __func__, __LINE__);
	}

	PM_LOCK(pm);

	if (VM_PAGE_TO_MD(pg)->mdpg_flags & PMAP_WIRED) {
		panic("%s: physical address %016lx is already wired mapped", __func__, pa);
	}
	if ((VM_PAGE_TO_MD(pg)->mdpg_pvnum >= 1) && (flags & PMAP_WIRED)) {
		panic("%s: physical address %016lx is already mapped. cannot map wired", __func__, pa);
	}


	/*
	 * traverse L0 -> L1 -> L2 -> L3 table with growing pdp if needed.
	 */
	l0 = pm->pm_l0table;

	idx = l0pde_index(va);
	pde = l0[idx];
	if (!l0pde_valid(pde)) {
//		KASSERT(!kenter);

		pmap_alloc_pdp(pm, &pdppa);
		KASSERT(pdppa != POOL_PADDR_INVALID);
		atomic_swap_64(&l0[idx], pdppa | L0_TABLE);
	} else {
		pdppa = l0pde_pa(pde);
	}
	l1 = AARCH64_PA_TO_KVA(pdppa);

	idx = l1pde_index(va);
	pde = l1[idx];
	if (!l1pde_valid(pde)) {
//		KASSERT(!kenter);	// already glowed

		pmap_alloc_pdp(pm, &pdppa);
		KASSERT(pdppa != POOL_PADDR_INVALID);
		atomic_swap_64(&l1[idx], pdppa | L1_TABLE);
	} else {
		pdppa = l1pde_pa(pde);
	}
	l2 = AARCH64_PA_TO_KVA(pdppa);

	idx = l2pde_index(va);
	pde = l2[idx];
	if (!l2pde_valid(pde)) {
//		KASSERT(!kenter);	// already glowed

		pmap_alloc_pdp(pm, &pdppa);
		KASSERT(pdppa != POOL_PADDR_INVALID);
		atomic_swap_64(&l2[idx], pdppa | L2_TABLE);
	} else {
		pdppa = l2pde_pa(pde);
	}
	l3 = AARCH64_PA_TO_KVA(pdppa);

	idx = l3pte_index(va);
	ptep = &l3[idx];	/* as PTE */

	KASSERT(pg != NULL);

	pte = *ptep;

	if (l3pte_valid(pte)) {
		KASSERT(!kenter);	/* pmap_kenter_pa() cannot override */

		/* pte is Already mapped */
		if (l3pte_pa(pte) != pa) {
			struct vm_page *opg;

			DPRINTF("va=%016lx has already mapped. old-pa=%016lx new-pa=%016lx, pte=%016llx\n",
			    va, l3pte_pa(pte), pa, pte);

			opg = PHYS_TO_VM_PAGE(l3pte_pa(pte));
			KASSERT(opg != NULL);

			if (opg == pg) {
				// XXXAARCH64
				panic("%s:%d: XXX: not supported yet", __func__, __LINE__);
			}

			_pmap_remove_pv(opg, pm, va, pte);
		}
	}


	if (kenter) {
		mdattr = (VM_PROT_READ | VM_PROT_WRITE);
		VM_PAGE_TO_MD(pg)->mdpg_flags |= (PMAP_WIRED | mdattr);
	} else {
		_pmap_enter_pv(pg, pm, va, ptep, pa, flags);
#if 1 // VIPT
		pmap_pv_lock(VM_PAGE_TO_MD(pg));
		if (flags & PMAP_WIRED) {
			KASSERT(VM_PAGE_TO_MD(pg)->mdpg_pvnum == 0);
			mdattr = (VM_PROT_READ | VM_PROT_WRITE);
			VM_PAGE_TO_MD(pg)->mdpg_flags |= (PMAP_WIRED | mdattr);
		} else if (IS_OWNER_PV(pg, pm, va)) {
			mdattr = (VM_PROT_READ | VM_PROT_WRITE);
			VM_PAGE_TO_MD(pg)->mdpg_flags |= mdattr;
		} else {
			mdattr = 0;
		}
		pmap_pv_unlock(VM_PAGE_TO_MD(pg));
#else
		if (flags & PMAP_WIRED) {
			mdattr = (VM_PROT_READ | VM_PROT_WRITE);
			VM_PAGE_TO_MD(pg)->mdpg_flags |= (PMAP_WIRED | mdattr);
		} else {
			mdattr = VM_PAGE_TO_MD(pg)->mdpg_flags &
			    (VM_PROT_READ | VM_PROT_WRITE);
		}
#endif
	}

	attr = _pmap_pte_adjust_prot(L3_TABLE, prot, mdattr);
	attr = _pmap_pte_adjust_cacheflags(attr, flags);
	if (VM_MAXUSER_ADDRESS > va)
		attr |= LX_BLKPAG_APUSER;
#ifdef MULTIPROCESSOR
	attr |= LX_BLKPAG_SH_IS;
#endif
	pte = pa | attr;
	atomic_swap_64(ptep, pte);

	aarch64_tlbi_by_va(va);
	if (kenter && (prot & VM_PROT_EXECUTE))
		cpu_icache_sync_range(va, PAGE_SIZE);

#if 1 /* VIPT */
//	if (kenter && (prot & (VM_PROT_READ|VM_PROT_WRITE))) {
	if (kenter) {
		cpu_dcache_inv_range(va, PAGE_SIZE);
	}
#endif

	PM_UNLOCK(pm);
	return 0;
}

int
pmap_enter(struct pmap *pm, vaddr_t va, paddr_t pa, vm_prot_t prot, u_int flags)
{
	KASSERT((prot & VM_PROT_READ) || !(prot & VM_PROT_WRITE));

	return _pmap_enter(pm, va, pa, prot, flags, false);
}

void
pmap_remove_all(struct pmap *pm)
{
	/* nothing to do */
}

static void
_pmap_remove(struct pmap *pm, vaddr_t va, bool kremove)
{
	pt_entry_t pte, *ptep;
	struct vm_page *pg;
	paddr_t pa;


	UVMHIST_FUNC(__func__);
	UVMHIST_CALLED(pmaphist);

	UVMHIST_LOG(pmaphist, "pm=%p, va=%016lx, kremovemode=%d", pm, va, kremove, 0);


	ptep = _pmap_pte_lookup(pm, va);
	if (ptep != NULL) {
		pte = *ptep;
		if (!l3pte_valid(pte))
			return;

		pa = l3pte_pa(pte);
		pg = PHYS_TO_VM_PAGE(pa);
		if (pg != NULL) {
			if (kremove) {
				VM_PAGE_TO_MD(pg)->mdpg_flags &= ~PMAP_WIRED;

				/* for unmap writable wired page */
				if ((pte & (LX_BLKPAG_AF|LX_BLKPAG_AP)) ==
				    (LX_BLKPAG_AF|LX_BLKPAG_AP_RW)) {
					cpu_dcache_wbinv_range(va, PAGE_SIZE);
				}

			} else {
				_pmap_remove_pv(pg, pm, va, pte);
			}
		}
		atomic_swap_64(ptep, 0);

		aarch64_tlbi_by_va(va);
	}
}

void
pmap_remove(struct pmap *pm, vaddr_t sva, vaddr_t eva)
{
	vaddr_t va;

	PM_ADDR_CHECK(pm, sva);

	PM_LOCK(pm);

	KASSERT(!IN_KSEG_ADDR(sva));

	for (va = sva; va < eva; va += PAGE_SIZE)
		_pmap_remove(pm, va, false);

	PM_UNLOCK(pm);
}

void
pmap_page_protect(struct vm_page *pg, vm_prot_t prot)
{
	struct vm_page_md *md = VM_PAGE_TO_MD(pg);
	struct pv_entry *pv;

	KASSERT((prot & VM_PROT_READ) || !(prot & VM_PROT_WRITE));


	UVMHIST_FUNC(__func__);
	UVMHIST_CALLED(pmaphist);

	UVMHIST_LOG(pmaphist, "pg=%p, md=%p, pa=%016lx, prot=%08x", pg, md, VM_PAGE_TO_PHYS(pg), prot);


	/* wired pages are not unaffected */
	if (md->mdpg_flags & PMAP_WIRED)
		return;

	pmap_pv_lock(md);
	TAILQ_FOREACH(pv, &md->mdpg_pvhead, pv_link) {
		_pmap_protect_pv(pg, pv, prot);
	}
	pmap_pv_unlock(md);
}

long
pmap_resident_count(struct pmap *pm)
{
	panic("%s", __func__);
	return pm->pm_stats.resident_count;
}

long
pmap_wired_count(struct pmap *pm)
{
	panic("%s", __func__);
	return pm->pm_stats.wired_count;
}

void
pmap_unwire(struct pmap *pm, vaddr_t va)
{
	panic("%s", __func__);
}

void
pmap_copy(struct pmap *dst_map, struct pmap *src_map, vaddr_t dst_addr,
    vsize_t len, vaddr_t src_addr)
{
	panic("%s", __func__);
}

bool
pmap_fault_fixup(struct pmap *pm, vaddr_t va, vm_prot_t accessprot)
{
	struct vm_page *pg;
	struct vm_page_md *md;
	pt_entry_t *ptep, pte;
	vm_prot_t pmap_prot;
	paddr_t pa;

	UVMHIST_FUNC(__func__);
	UVMHIST_CALLED(pmaphist);

	UVMHIST_LOG(pmaphist, "pm=%p, va=%016lx, accessprot=%08x", pm, va, accessprot, 0);


#if 0
	PM_ADDR_CHECK(pm, va);
#else
	if (((pm == pmap_kernel()) &&
	    !(VM_MIN_KERNEL_ADDRESS <= va && va < VM_MAX_KERNEL_ADDRESS)) ||
	    ((pm != pmap_kernel()) &&
	    !(VM_MIN_ADDRESS <= va && va <= VM_MAX_ADDRESS))) {

		UVMHIST_LOG(pmaphist, "pmap space and va mismatch", 0, 0, 0, 0);
		return false;
	}
#endif

	ptep = _pmap_pte_lookup(pm, va);
	if (ptep == NULL)
		return false;

	pte = *ptep;
	if (!l3pte_valid(pte))
		return false;

	pa = l3pte_pa(*ptep);
	pg = PHYS_TO_VM_PAGE(pa);
	if (pg == NULL)
		return false;
	md = VM_PAGE_TO_MD(pg);

	/* get prot by pmap_enter() (stored in software use bit) */
	switch (pte & (LX_BLKPAG_OS_READ|LX_BLKPAG_OS_WRITE)) {
	case 0:
	default:
		pmap_prot = 0;
		break;
	case LX_BLKPAG_OS_READ:
		pmap_prot = VM_PROT_READ;
		break;
	case LX_BLKPAG_OS_WRITE:
	case LX_BLKPAG_OS_READ|LX_BLKPAG_OS_WRITE:
		pmap_prot = (VM_PROT_READ|VM_PROT_WRITE);
		break;
	}

	UVMHIST_LOG(pmaphist, "va=%016lx, pmapprot=%08x, accessprot=%08x", va, pmap_prot, accessprot, 0);

	/* ignore except read/write */
	accessprot &= (VM_PROT_READ|VM_PROT_WRITE);
	accessprot |= VM_PROT_READ;	/* treat WRITE access as READ/WRITE */

	/* no permission to read/write for this page */
	if ((pmap_prot & accessprot) != accessprot)
		return false;

	KASSERT(((pte & LX_BLKPAG_AF) == 0) ||
	    ((pte & LX_BLKPAG_AP) == LX_BLKPAG_AP_RO));

	pmap_pv_lock(md);
	if ((pte & LX_BLKPAG_AF) == 0) {
#if 1 /* VIPT */
		struct pv_entry *pv, *opv;

		/* find fixup pv */
		TAILQ_FOREACH(pv, &md->mdpg_pvhead, pv_link) {
			if ((pm == pv->pv_pmap) && (va == pv->pv_va)) {
				break;
			}
		}
		KASSERT(pv != NULL);
		opv = md->mdpg_pa_owner;
		if (opv == NULL) {
			UVMHIST_LOG(pmaphist, "no one has pa=%016lx. va=%016lx become owner", pa, va, 0, 0);
			md->mdpg_pa_owner = pv;
		} else if (pv != opv) {
			/* other pv has pa */
			pt_entry_t *opv_ptep, opv_pte;

			UVMHIST_LOG(pmaphist, "va=%016lx -> pa=%016lx has %d aliases (owner: va=%016x)", va, pa, md->mdpg_pvnum, opv->pv_va);
#ifdef PMAP_PV_DEBUG
			dprintf("va %016lx -> pa %016lx has %d alias\n", va, pa, md->mdpg_pvnum);
			pv_dump(md, printf);
#endif

			opv_ptep = opv->pv_ptep;
			KASSERT(opv_ptep != NULL);
			opv_pte = *opv_ptep;

			if ((opv_pte & (LX_BLKPAG_AF|LX_BLKPAG_OS_WRITE)) ==
			    (LX_BLKPAG_AF|LX_BLKPAG_OS_WRITE)) {
				cpu_dcache_wbinv_range(opv->pv_va, PAGE_SIZE);
			}
			/* remove AF from owner pv */
			opv_pte &= ~LX_BLKPAG_AF;
			atomic_swap_64(opv_ptep, opv_pte);

			aarch64_tlbi_by_va(opv->pv_va);

			md->mdpg_pa_owner = pv;
		}
#endif

		UVMHIST_LOG(pmaphist, "REFERENCED: va=%016lx, pa=%016lx, pte_prot=%08x, accessprot=%08x", va, pa, pmap_prot, accessprot);
		DPRINTF("REFERENCED: va=%016lx, pa=%016lx, pte_prot=%s, accessprot=%s\n", va, pa, str_vmflags(pmap_prot), str_vmflags(accessprot));
		md->mdpg_flags |= VM_PROT_READ;
		pte |= LX_BLKPAG_AF;
	}
	if ((accessprot & VM_PROT_WRITE) &&
	    ((pte & LX_BLKPAG_AP) == LX_BLKPAG_AP_RO)) {
		UVMHIST_LOG(pmaphist, "MODIFIED: va=%016lx, pa=%016lx, pte_prot=%08x, accessprot=%08x", va, pa, pmap_prot, accessprot);
		DPRINTF("MODIFIED: va=%016lx, pa=%016lx, pte_prot=%s, accessprot=%s\n", va, pa, str_vmflags(pmap_prot), str_vmflags(accessprot));
		md->mdpg_flags |= VM_PROT_WRITE;
		pte &= ~LX_BLKPAG_AP;
		pte |= LX_BLKPAG_AP_RW;
	}
	pmap_pv_unlock(md);

	atomic_swap_64(ptep, pte);
	aarch64_tlbi_by_va(va);

	return true;
}

bool
pmap_clear_modify(struct vm_page *pg)
{
	struct pmap *pm;
	struct pv_entry *pv;
	struct vm_page_md * const md = VM_PAGE_TO_MD(pg);
	pt_entry_t *ptep, pte, opte;
	vaddr_t va;

	UVMHIST_FUNC(__func__);
	UVMHIST_CALLED(pmaphist);

	UVMHIST_LOG(pmaphist, "pg=%p, mdpg_flags=%08x, mdpg_pvnum=%d", pg, md->mdpg_flags, md->mdpg_pvnum, 0);

	pmap_pv_lock(md);

	if ((md->mdpg_flags & VM_PROT_WRITE) == 0) {
		pmap_pv_unlock(md);
		return false;
	}

	md->mdpg_flags &= ~VM_PROT_WRITE;

	TAILQ_FOREACH(pv, &md->mdpg_pvhead, pv_link) {
		pm = pv->pv_pmap;
		va = pv->pv_va;

		ptep = pv->pv_ptep;
		opte = pte = *ptep;
 tryagain:
		if (!l3pte_valid(pte))
			continue;

		/* clear write permission */
		pte &= ~LX_BLKPAG_AP;
		pte |= LX_BLKPAG_AP_RO;

		/* XXX: possible deadlock if using PM_LOCK(), and racy */
		if ((pte = atomic_cas_64(ptep, opte, pte)) != opte) {
			opte = pte;
			goto tryagain;
		}

		aarch64_tlbi_by_va(va);

		UVMHIST_LOG(pmaphist, "va=%016llx, ptep=%p, pa=%016lx, RW -> RO", va, ptep, l3pte_pa(pte), 0);
	}

	pmap_pv_unlock(md);

	return true;
}

bool
pmap_clear_reference(struct vm_page *pg)
{
	struct pmap *pm;
	struct pv_entry *pv;
	struct vm_page_md * const md = VM_PAGE_TO_MD(pg);
	pt_entry_t *ptep, pte, opte;
	vaddr_t va;

	UVMHIST_FUNC(__func__);
	UVMHIST_CALLED(pmaphist);

	UVMHIST_LOG(pmaphist, "pg=%p, mdpg_flags=%08x, mdpg_pvnum=%d", pg, md->mdpg_flags, md->mdpg_pvnum, 0);

	pmap_pv_lock(md);

	if ((md->mdpg_flags & VM_PROT_READ) == 0) {
		pmap_pv_unlock(md);
		return false;
	}
	md->mdpg_flags &= ~VM_PROT_READ;

	TAILQ_FOREACH(pv, &md->mdpg_pvhead, pv_link) {
		pm = pv->pv_pmap;
		va = pv->pv_va;

		ptep = pv->pv_ptep;
		opte = pte = *ptep;
 tryagain:
		if (!l3pte_valid(pte))
			continue;

		/* clear access permission */
		pte &= ~LX_BLKPAG_AF;

		/* XXX: possible deadlock if using PM_LOCK(), and racy */
		if ((pte = atomic_cas_64(ptep, opte, pte)) != opte) {
			opte = pte;
			goto tryagain;
		}

		aarch64_tlbi_by_va(va);

		UVMHIST_LOG(pmaphist, "va=%016llx, ptep=%p, pa=%016lx, unse AF", va, ptep, l3pte_pa(pte), 0);
	}

	pmap_pv_unlock(md);

	return true;
}

bool
pmap_is_modified(struct vm_page *pg)
{
	struct vm_page_md * const md = VM_PAGE_TO_MD(pg);
	return (md->mdpg_flags & VM_PROT_WRITE);
}

bool
pmap_is_referenced(struct vm_page *pg)
{
	struct vm_page_md * const mdpg = VM_PAGE_TO_MD(pg);
	return (mdpg->mdpg_flags & VM_PROT_READ);
}

#ifdef DDB
static void
pmap_db_pte_print(pt_entry_t pte, int level, void (*pr)(const char *, ...))
{
	if (pte == 0) {
		pr("UNUSED\n");

	} else if (level == 0) {
		/* L0 pde */
		pr(", %s", l1pde_is_table(pte) ? "TABLE" : "***ILLEGAL TYPE***");
		pr(", %s", l0pde_valid(pte) ? "VALID" : "***INVALID***");

		pr(", PA=%016lx", l0pde_pa(pte));

	} else if (((level == 1) && l1pde_is_block(pte)) ||
	    ((level == 2) && l2pde_is_block(pte)) ||
		(level == 3)) {

		if (level == 3) {
			pr(", %s", l3pte_is_page(pte) ? " PAGE" : "***ILLEGAL TYPE***");
			pr(", %s", l3pte_valid(pte) ? "VALID" : "***INVALID***");
		} else {
			pr(", %s", l1pde_is_table(pte) ? "TABLE" : "BLOCK");
			pr(", %s", l1pde_valid(pte) ? "VALID" : "***INVALID***");
		}

		pr(", PA=%016lx", l3pte_pa(pte));

		/* L[12] block, or L3 pte */
		pr(", %s", (pte & LX_BLKPAG_UXN) ? "UXN" : "---");
		pr(", %s", (pte & LX_BLKPAG_PXN) ? "PXN" : "---");

		if (pte & LX_BLKPAG_CONTIG)
			pr(",CONTIG");

		pr(", %s", (pte & LX_BLKPAG_NG) ? "NG" : "--");
		pr(", %s", (pte & LX_BLKPAG_AF) ? "AF" : "--");

		switch (pte & LX_BLKPAG_SH) {
		case LX_BLKPAG_SH_NS:
			pr(", SH_NS");
			break;
		case LX_BLKPAG_SH_OS:
			pr(", SH_OS");
			break;
		case LX_BLKPAG_SH_IS:
			pr(", SH_IS");
			break;
		default:
			pr(", SH_??");
			break;
		}

		pr(", %s", (pte & LX_BLKPAG_AP_RO) ? "RO" : "RW");
		pr(", %s", (pte & LX_BLKPAG_APUSER) ? "EL0" : "EL1");

		switch (pte & LX_BLKPAG_ATTR_MASK) {
		case LX_BLKPAG_ATTR_NORMAL_WB:
			pr(", WRITEBACK");
			break;
		case LX_BLKPAG_ATTR_NORMAL_NC:
			pr(", NOCACHE");
			break;
		case LX_BLKPAG_ATTR_NORMAL_WT:
			pr(", WHITETHRU");
			break;
		case LX_BLKPAG_ATTR_DEVICE_MEM:
			pr(", DEVICE");
			break;
		}

		if (pte & LX_BLKPAG_OS_READ)
			pr(", pmap_read");
		if (pte & LX_BLKPAG_OS_WRITE)
			pr(", pmap_write");
		if ((pte & (LX_BLKPAG_UXN|LX_BLKPAG_PXN)) == 0)
			pr(", executable");

	} else {
		/* L1 and L2 pde */
		pr(", %s", l1pde_is_table(pte) ? "TABLE" : "BLOCK");
		pr(", %s", l1pde_valid(pte) ? "VALID" : "***INVALID***");
		pr(", PA=%016lx", l1pde_pa(pte));
	}
	pr("\n");
}


void
pmap_db_pteinfo(vaddr_t va, void (*pr)(const char *, ...))
{
	struct pmap *pm;
	struct vm_page *pg;
	bool user;


	if (VM_MAXUSER_ADDRESS > va) {
		pm = curlwp->l_proc->p_vmspace->vm_map.pmap;
		user = true;
	} else {
		pm = pmap_kernel();
		user = false;
	}


	{
		pd_entry_t *l0, *l1, *l2, *l3;
		pd_entry_t pde;
		pt_entry_t pte;
		struct vm_page_md *md;
		paddr_t pa;
		unsigned int idx;

		/*
		 * traverse L0 -> L1 -> L2 -> L3 table
		 */
		l0 = pm->pm_l0table;

		pr("TTBR%d=%016llx (%016llx)", user ? 0 : 1, pm->pm_l0table_pa, l0);
		pr(", input-va=%016llx, L0-index=%d, L1-index=%d, L2-index=%d, L3-index=%d\n",
		    va,
		    (va & L0_ADDR_BITS) >> L0_SHIFT,
		    (va & L1_ADDR_BITS) >> L1_SHIFT,
		    (va & L2_ADDR_BITS) >> L2_SHIFT,
		    (va & L3_ADDR_BITS) >> L3_SHIFT);

		idx = l0pde_index(va);
		pde = l0[idx];

		pr("L0[%3d]=%016llx", idx, pde);
		pmap_db_pte_print(pde, 0, pr);

		if (!l0pde_valid(pde))
			return;

		l1 = AARCH64_PA_TO_KVA(l0pde_pa(pde));
		idx = l1pde_index(va);
		pde = l1[idx];

		pr(" L1[%3d]=%016llx", idx, pde);
		pmap_db_pte_print(pde, 1, pr);

		if (!l1pde_valid(pde) || l1pde_is_block(pde))
			return;

		l2 = AARCH64_PA_TO_KVA(l1pde_pa(pde));
		idx = l2pde_index(va);
		pde = l2[idx];

		pr("  L2[%3d]=%016llx", idx, pde);
		pmap_db_pte_print(pde, 2, pr);

		if (!l2pde_valid(pde) || l2pde_is_block(pde))
			return;

		l3 = AARCH64_PA_TO_KVA(l2pde_pa(pde));
		idx = l3pte_index(va);
		pte = l3[idx];

		pr("   L3[%3d]=%016llx", idx, pte);
		pmap_db_pte_print(pte, 3, pr);

		pa = l3pte_pa(pte);
		pg = PHYS_TO_VM_PAGE(pa);
		if (pg != NULL) {
			md = VM_PAGE_TO_MD(pg);
			pv_dump(md, pr);
		}
	}
}
#endif /* DDB */
