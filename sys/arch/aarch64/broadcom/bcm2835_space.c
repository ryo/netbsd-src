/*	$NetBSD: bcm2835_space.c,v 1.10 2016/02/02 13:55:50 skrll Exp $	*/

/*-
 * Copyright (c) 2012 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Nick Hudson
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


#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: bcm2835_space.c,v 1.10 2016/02/02 13:55:50 skrll Exp $");

#include <sys/param.h>
#include <sys/systm.h>

#include <uvm/uvm_extern.h>

#include <sys/bus.h>

#include <arm/locore.h>
#include <arm/broadcom/bcm2835reg.h>

/* Prototypes for all the bus_space structure functions */
bs_protos(bcm2835);
bs_protos(generic_dsb);
bs_protos(generic);
bs_protos(bs_notimpl);

struct bus_space bcm2835_bs_tag = {
	/* stride, flags */
	.bs_stride = 0,
	.bs_flags = 0,

	.bs_map = bcm2835_bs_map,
	.bs_unmap = bcm2835_bs_unmap,
	.bs_subregion = generic_bs_subregion,
	.bs_alloc = generic_bs_alloc,
	.bs_free = generic_bs_free,
	.bs_vaddr = generic_bs_vaddr,
	.bs_mmap = generic_bs_mmap,
	.bs_barrier = generic_bs_barrier,

	/* read */
	.bs_r_1 = generic_bs_r_1,
	.bs_r_2 = generic_bs_r_2,
	.bs_r_4 = generic_bs_r_4,
	.bs_r_8 = generic_bs_r_8,

	/* write */
	.bs_w_1 = generic_bs_w_1,
	.bs_w_2 = generic_bs_w_2,
	.bs_w_4 = generic_bs_w_4,
	.bs_w_8 = generic_bs_w_8,

	/* read region */
	.bs_rr_1 = generic_bs_rr_1,
	.bs_rr_2 = generic_bs_rr_2,
	.bs_rr_4 = generic_bs_rr_4,
	.bs_rr_8 = generic_bs_rr_8,

	/* write region */
	.bs_wr_1 = generic_bs_wr_1,
	.bs_wr_2 = generic_bs_wr_2,
	.bs_wr_4 = generic_bs_wr_4,
	.bs_wr_8 = generic_bs_wr_8,

	/* copy region */
	.bs_c_1 = generic_bs_c_1,
	.bs_c_2 = generic_bs_c_2,
	.bs_c_4 = generic_bs_c_4,
	.bs_c_8 = generic_bs_c_8,

	/* set region */
	.bs_sr_1 = generic_bs_sr_1,
	.bs_sr_2 = generic_bs_sr_2,
	.bs_sr_4 = generic_bs_sr_4,
	.bs_sr_8 = generic_bs_sr_8,

	/* read multi */
	.bs_rm_1 = generic_bs_rm_1,
	.bs_rm_2 = generic_bs_rm_2,
	.bs_rm_4 = generic_bs_rm_4,
	.bs_rm_8 = generic_bs_rm_8,

	/* write multi */
	.bs_wm_1 = generic_bs_wm_1,
	.bs_wm_2 = generic_bs_wm_2,
	.bs_wm_4 = generic_bs_wm_4,
	.bs_wm_8 = generic_bs_wm_8,

	/* set multi */
	.bs_sm_1 = generic_bs_sm_1,
	.bs_sm_2 = generic_bs_sm_2,
	.bs_sm_4 = generic_bs_sm_4,
	.bs_sm_8 = generic_bs_sm_8,

	/* peek */
	.bs_pe_1 = generic_bs_pe_1,
	.bs_pe_2 = generic_bs_pe_2,
	.bs_pe_4 = generic_bs_pe_4,
	.bs_pe_8 = generic_bs_pe_8,

	/* poke */
	.bs_po_1 = generic_bs_po_1,
	.bs_po_2 = generic_bs_po_2,
	.bs_po_4 = generic_bs_po_4,
	.bs_po_8 = generic_bs_po_8,

#ifdef __BUS_SPACE_HAS_STREAM_METHODS
	/* read stream */
	.bs_r_1_s = generic_bs_r_1,
	.bs_r_2_s = generic_bs_r_2,
	.bs_r_4_s = generic_bs_r_4,
	.bs_r_8_s = generic_bs_r_8,

	/* write stream */
	.bs_w_1_s = generic_bs_w_1,
	.bs_w_2_s = generic_bs_w_2,
	.bs_w_4_s = generic_bs_w_4,
	.bs_w_8_s = generic_bs_w_8,

	/* read region stream */
	.bs_rr_1_s = generic_bs_rr_1,
	.bs_rr_2_s = generic_bs_rr_2,
	.bs_rr_4_s = generic_bs_rr_4,
	.bs_rr_8_s = generic_bs_rr_8,

	/* write region stream */
	.bs_wr_1_s = generic_bs_wr_1,
	.bs_wr_2_s = generic_bs_wr_2,
	.bs_wr_4_s = generic_bs_wr_4,
	.bs_wr_8_s = generic_bs_wr_8,

	/* read multi stream */
	.bs_rm_1_s = generic_bs_rm_1,
	.bs_rm_2_s = generic_bs_rm_2,
	.bs_rm_4_s = generic_bs_rm_4,
	.bs_rm_8_s = generic_bs_rm_8,

	/* write multi stream */
	.bs_wm_1_s = generic_bs_wm_1,
	.bs_wm_2_s = generic_bs_wm_2,
	.bs_wm_4_s = generic_bs_wm_4,
	.bs_wm_8_s = generic_bs_wm_8,
#endif
};

struct bus_space bcm2835_a4x_bs_tag = {
	/* stride, flags */
	.bs_stride = 2,
	.bs_flags = 0,

	.bs_map = bcm2835_bs_map,
	.bs_unmap = bcm2835_bs_unmap,
	.bs_subregion = generic_bs_subregion,
	.bs_alloc = generic_bs_alloc,
	.bs_free = generic_bs_free,
	.bs_vaddr = generic_bs_vaddr,
	.bs_mmap = generic_bs_mmap,
	.bs_barrier = generic_bs_barrier,

	/* read */
	.bs_r_1 = generic_bs_r_1,
	.bs_r_2 = generic_bs_r_2,
	.bs_r_4 = generic_bs_r_4,
	.bs_r_8 = generic_bs_r_8,

	/* write */
	.bs_w_1 = generic_bs_w_1,
	.bs_w_2 = generic_bs_w_2,
	.bs_w_4 = generic_bs_w_4,
	.bs_w_8 = generic_bs_w_8,

	/* read region */
	.bs_rr_1 = generic_bs_rr_1,
	.bs_rr_2 = generic_bs_rr_2,
	.bs_rr_4 = generic_bs_rr_4,
	.bs_rr_8 = generic_bs_rr_8,

	/* write region */
	.bs_wr_1 = generic_bs_wr_1,
	.bs_wr_2 = generic_bs_wr_2,
	.bs_wr_4 = generic_bs_wr_4,
	.bs_wr_8 = generic_bs_wr_8,

	/* copy region */
	.bs_c_1 = generic_bs_c_1,
	.bs_c_2 = generic_bs_c_2,
	.bs_c_4 = generic_bs_c_4,
	.bs_c_8 = generic_bs_c_8,

	/* set region */
	.bs_sr_1 = generic_bs_sr_1,
	.bs_sr_2 = generic_bs_sr_2,
	.bs_sr_4 = generic_bs_sr_4,
	.bs_sr_8 = generic_bs_sr_8,

	/* read multi */
	.bs_rm_1 = generic_bs_rm_1,
	.bs_rm_2 = generic_bs_rm_2,
	.bs_rm_4 = generic_bs_rm_4,
	.bs_rm_8 = generic_bs_rm_8,

	/* write multi */
	.bs_wm_1 = generic_bs_wm_1,
	.bs_wm_2 = generic_bs_wm_2,
	.bs_wm_4 = generic_bs_wm_4,
	.bs_wm_8 = generic_bs_wm_8,

	/* set multi */
	.bs_sm_1 = generic_bs_sm_1,
	.bs_sm_2 = generic_bs_sm_2,
	.bs_sm_4 = generic_bs_sm_4,
	.bs_sm_8 = generic_bs_sm_8,

	/* peek */
	.bs_pe_1 = generic_bs_pe_1,
	.bs_pe_2 = generic_bs_pe_2,
	.bs_pe_4 = generic_bs_pe_4,
	.bs_pe_8 = generic_bs_pe_8,

	/* poke */
	.bs_po_1 = generic_bs_po_1,
	.bs_po_2 = generic_bs_po_2,
	.bs_po_4 = generic_bs_po_4,
	.bs_po_8 = generic_bs_po_8,

#ifdef __BUS_SPACE_HAS_STREAM_METHODS
	/* read stream */
	.bs_r_1_s = generic_bs_r_1,
	.bs_r_2_s = generic_bs_r_2,
	.bs_r_4_s = generic_bs_r_4,
	.bs_r_8_s = generic_bs_r_8,

	/* write stream */
	.bs_w_1_s = generic_bs_w_1,
	.bs_w_2_s = generic_bs_w_2,
	.bs_w_4_s = generic_bs_w_4,
	.bs_w_8_s = generic_bs_w_8,

	/* read region stream */
	.bs_rr_1_s = generic_bs_rr_1,
	.bs_rr_2_s = generic_bs_rr_2,
	.bs_rr_4_s = generic_bs_rr_4,
	.bs_rr_8_s = generic_bs_rr_8,

	/* write region stream */
	.bs_wr_1_s = generic_bs_wr_1,
	.bs_wr_2_s = generic_bs_wr_2,
	.bs_wr_4_s = generic_bs_wr_4,
	.bs_wr_8_s = generic_bs_wr_8,

	/* read multi stream */
	.bs_rm_1_s = generic_bs_rm_1,
	.bs_rm_2_s = generic_bs_rm_2,
	.bs_rm_4_s = generic_bs_rm_4,
	.bs_rm_8_s = generic_bs_rm_8,

	/* write multi stream */
	.bs_wm_1_s = generic_bs_wm_1,
	.bs_wm_2_s = generic_bs_wm_2,
	.bs_wm_4_s = generic_bs_wm_4,
	.bs_wm_8_s = generic_bs_wm_8,
#endif
};


int
bcm2835_bs_map(void *t, bus_addr_t ba, bus_size_t size, int flag,
    bus_space_handle_t *bshp)
{
	u_long startpa, endpa, pa;
	vaddr_t va;
	const struct pmap_devmap *pd;
	int pmap_flags;
	bool match = false;

	/* Attempt to find the PA device mapping */
	if (ba >= BCM2835_PERIPHERALS_BASE_BUS &&
	    ba < BCM2835_PERIPHERALS_BASE_BUS + BCM2835_PERIPHERALS_SIZE) {
		match = true;
		pa = BCM2835_PERIPHERALS_BUS_TO_PHYS(ba);
	}
#ifdef BCM2836
	if (ba >= BCM2836_ARM_LOCAL_BASE &&
	    ba < BCM2836_ARM_LOCAL_BASE + BCM2836_ARM_LOCAL_SIZE) {
		match = true;
		pa = ba;
	}
#endif

	if (match && (pd = pmap_devmap_find_pa(pa, size)) != NULL) {
		/* Device was statically mapped. */
		*bshp = pd->pd_va + (pa - pd->pd_pa);
		return 0;
	}

	/* Now assume bus address so convert to PA */
	if (ba >= BCM2835_PERIPHERALS_BASE_BUS &&
	    ba < BCM2835_PERIPHERALS_BASE_BUS + BCM2835_PERIPHERALS_SIZE) {
		pa = ba & ~BCM2835_BUSADDR_CACHE_MASK;
	}

	startpa = trunc_page(pa);
	endpa = round_page(pa + size);

	/* XXX use extent manager to check duplicate mapping */

	va = uvm_km_alloc(kernel_map, endpa - startpa, 0,
	    UVM_KMF_VAONLY | UVM_KMF_NOWAIT);
	if (!va)
		return ENOMEM;

	*bshp = (bus_space_handle_t)(va + (pa - startpa));

	pmap_flags = (flag & BUS_SPACE_MAP_CACHEABLE) ? 0 : PMAP_NOCACHE;
	for (pa = startpa; pa < endpa; pa += PAGE_SIZE, va += PAGE_SIZE) {
		pmap_kenter_pa(va, pa, VM_PROT_READ | VM_PROT_WRITE,
		    pmap_flags);
	}
	pmap_update(pmap_kernel());

	return 0;
}

void
bcm2835_bs_unmap(void *t, bus_space_handle_t bsh, bus_size_t size)
{
	vaddr_t	va;
	vsize_t	sz;

	if (pmap_devmap_find_va(bsh, size) != NULL) {
		/* Device was statically mapped; nothing to do. */
		return;
	}

	va = trunc_page(bsh);
	sz = round_page(bsh + size) - va;

	pmap_kremove(va, sz);
	pmap_update(pmap_kernel());
	uvm_km_free(kernel_map, va, sz, UVM_KMF_VAONLY);
}
