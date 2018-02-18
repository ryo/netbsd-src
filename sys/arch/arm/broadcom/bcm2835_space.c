/*	$NetBSD: bcm2835_space.c,v 1.15 2018/01/22 13:23:56 skrll Exp $	*/

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
__KERNEL_RCSID(0, "$NetBSD: bcm2835_space.c,v 1.15 2018/01/22 13:23:56 skrll Exp $");

#include <sys/param.h>
#include <sys/systm.h>

#include <uvm/uvm_extern.h>

#include <sys/bus.h>

#include <arm/locore.h>
#include <arm/broadcom/bcm2835reg.h>

/* Prototypes for all the bus_space structure functions */
bs_protos(bcm2835);
bs_protos(bcm2835_a4x);
bs_protos(bcm2836);
bs_protos(bcm2836_a4x);

struct bus_space bcm2835_bs_tag;
struct bus_space bcm2835_a4x_bs_tag;
struct bus_space bcm2836_bs_tag;
struct bus_space bcm2836_a4x_bs_tag;

int
bcm283x_bs_map(void *, bus_addr_t, bus_size_t, int, bus_space_handle_t *);

int
bcm283x_bs_map(void *t, bus_addr_t ba, bus_size_t size, int flag,
    bus_space_handle_t *bshp)
{
	u_long startpa, endpa, pa;
	int pmapflags;
	vaddr_t va;

	/* Convert BA to PA */
	pa = ba & ~BCM2835_BUSADDR_CACHE_MASK;

	startpa = trunc_page(pa);
	endpa = round_page(pa + size);

	/* XXX use extent manager to check duplicate mapping */

	va = uvm_km_alloc(kernel_map, endpa - startpa, 0,
	    UVM_KMF_VAONLY | UVM_KMF_NOWAIT | UVM_KMF_COLORMATCH);
	if (!va)
		return ENOMEM;

	*bshp = (bus_space_handle_t)(va + (pa - startpa));

	if (flag & BUS_SPACE_MAP_PREFETCHABLE)
		pmapflags = PMAP_WRITE_COMBINE;
	else if (flag & BUS_SPACE_MAP_CACHEABLE)
		pmapflags = 0;
	else
		pmapflags = PMAP_NOCACHE;

	for (pa = startpa; pa < endpa; pa += PAGE_SIZE, va += PAGE_SIZE) {
		pmap_kenter_pa(va, pa, VM_PROT_READ | VM_PROT_WRITE, pmapflags);
	}
	pmap_update(pmap_kernel());

	return 0;
}

int
bcm2835_bs_map(void *t, bus_addr_t ba, bus_size_t size, int flag,
    bus_space_handle_t *bshp)
{
	const struct pmap_devmap *pd;
	bool match = false;
	u_long pa;

	/* Attempt to find the PA device mapping */
	if (ba >= BCM2835_PERIPHERALS_BASE_BUS &&
	    ba < BCM2835_PERIPHERALS_BASE_BUS + BCM2835_PERIPHERALS_SIZE) {
		match = true;
		pa = BCM2835_PERIPHERALS_BUS_TO_PHYS(ba);
	}

	if (match && (pd = pmap_devmap_find_pa(pa, size)) != NULL) {
		/* Device was statically mapped. */
		*bshp = pd->pd_va + (pa - pd->pd_pa);
		return 0;
	}

	return bcm283x_bs_map(t, ba, size, flag, bshp);
}

int
bcm2836_bs_map(void *t, bus_addr_t ba, bus_size_t size, int flag,
    bus_space_handle_t *bshp)
{
	const struct pmap_devmap *pd;
	bool match = false;
	u_long pa;

	/* Attempt to find the PA device mapping */
	if (ba >= BCM2835_PERIPHERALS_BASE_BUS &&
	    ba < BCM2835_PERIPHERALS_BASE_BUS + BCM2835_PERIPHERALS_SIZE) {
		match = true;
		pa = BCM2836_PERIPHERALS_BUS_TO_PHYS(ba);
	}

	if (ba >= BCM2836_ARM_LOCAL_BASE &&
	    ba < BCM2836_ARM_LOCAL_BASE + BCM2836_ARM_LOCAL_SIZE) {
		match = true;
		pa = ba;
	}

	if (match && (pd = pmap_devmap_find_pa(pa, size)) != NULL) {
		/* Device was statically mapped. */
		*bshp = pd->pd_va + (pa - pd->pd_pa);
		return 0;
	}

	return bcm283x_bs_map(t, ba, size, flag, bshp);
}

