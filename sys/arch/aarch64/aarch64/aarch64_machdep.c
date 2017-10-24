/* $NetBSD: aarch64_machdep.c,v 1.1 2014/08/10 05:47:37 matt Exp $ */

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

#include <sys/cdefs.h>
__KERNEL_RCSID(1, "$NetBSD: aarch64_machdep.c,v 1.1 2014/08/10 05:47:37 matt Exp $");

#include "opt_arm_debug.h"
#include "opt_ddb.h"
#include "opt_kernhist.h"

#include <sys/param.h>
#include <sys/types.h>
#include <sys/bus.h>
#include <sys/kauth.h>
#include <sys/msgbuf.h>

#include <dev/mm.h>

#include <uvm/uvm.h>

#include <aarch64/armreg.h>
#include <aarch64/bootconfig.h>
#include <aarch64/cpufunc.h>
#ifdef DDB
#include <aarch64/db_machdep.h>
#endif
#include <aarch64/frame.h>
#include <aarch64/machdep.h>
#include <aarch64/pmap.h>
#include <aarch64/pte.h>
#include <aarch64/vmparam.h>

char cpu_model[32];
char machine[] = MACHINE;
char machine_arch[] = MACHINE_ARCH;

const pcu_ops_t * const pcu_ops_md_defs[PCU_UNIT_COUNT] = {
	[PCU_FPU] = &pcu_fpu_ops
};

struct vm_map *phys_map;

/* XXX */
vaddr_t physical_start;
vaddr_t physical_end;

struct BootConfig *aarch64_bootconfig;

/*
 * Upper region: 0xffffffffffffffff  Top of virtual memory
 *
 *               0xffffffffffe00000  End of KVA
 *                                   = VM_MAX_KERNEL_ADDRESS
 *
 *               0xffffffc00???????  End of kernel
 *                                   = _end[]
 *               0xffffffc000??????  Start of kernel
 *                                   = __kernel_text[]
 *
 *               0xffffffc000000000  Kernel base address & start of KVA
 *                                   = VM_MIN_KERNEL_ADDRESS
 *
 *               0xffffffbfffffffff  End of direct mapped
 *               0xffff000000000000  Start of direct mapped
 *                                   = AARCH64_KSEG_START
 *                                   = AARCH64_KMEMORY_BASE
 *
 * Hole:         0xfffeffffffffffff
 *               0x0001000000000000
 *
 * Lower region: 0x0000fffffffff000  End of user address space
 *                                   = VM_MAXUSER_ADDRESS
 *
 *               0x000000000???????  End of Loaded kernel image (boot)
 *               0x0000000000??????  Start of Loaded kernel image (boot)
 *                                   = LOADADDRESS
 *
 *               0x0000000000000000  Start of user address space
 */
void
initarm64(struct BootConfig *bootconf)
{
	extern char __kernel_text[];
	extern char _end[];
	extern char lwp0uspace[];

	struct trapframe *tf;
	psize_t memsize_total;
	vaddr_t kernstart, kernend;
	vaddr_t kernstart_l2, kernend_l2;	/* L2 table 2MB aligned */
	paddr_t kernstart_phys, kernend_phys;
	paddr_t kstartp, kendp;			/* physical page of kernel */
	int i;

	aarch64_getcacheinfo();

	cputype = cpu_idnum();	/* for compatible arm */

	kernstart = trunc_page((vaddr_t)__kernel_text);
	kernend = round_page((vaddr_t)_end);
	kernstart_l2 = kernstart & -L2_SIZE;		/* trunk L2_SIZE(2M) */
	kernend_l2 = (kernend + L2_SIZE - 1) & -L2_SIZE;/* round L2_SIZE(2M) */

	kernstart_phys = kernstart - VM_MIN_KERNEL_ADDRESS;
	kernend_phys = kernend - VM_MIN_KERNEL_ADDRESS;

	aarch64_bootconfig = bootconf;

	/* XXX */
	physical_start = bootconf->dram[0].address;
	physical_end = physical_start + ptoa(bootconf->dram[0].pages);

#ifdef VERBOSE_INIT_ARM
	printf(
	    "------------------------------------------\n"
	    "physical_start        = 0x%016lx\n"
	    "kernel_start_phys     = 0x%016lx\n"
	    "kernel_end_phys       = 0x%016lx\n"
	    "physical_end          = 0x%016lx\n"
	    "VM_MIN_KERNEL_ADDRESS = 0x%016lx\n"
	    "kernel_start_l2       = 0x%016lx\n"
	    "kernel_start          = 0x%016lx\n"
	    "kernel_end            = 0x%016lx\n"
	    "kernel_end_l2         = 0x%016lx\n"
	    "(kernel va area)\n"
	    "(devmap va area)\n"
	    "VM_MAX_KERNEL_ADDRESS = 0x%016lx\n"
	    "------------------------------------------\n",
	    physical_start,
	    kernstart_phys,
	    kernend_phys,
	    physical_end,
	    VM_MIN_KERNEL_ADDRESS,
	    kernstart_l2,
	    kernstart,
	    kernend,
	    kernend_l2,
	    VM_MAX_KERNEL_ADDRESS);
#endif

	/*
	 * msgbuf is always allocated from bottom of memory
	 * against corruption by bootloader, or changing kernel layout.
	 */
	physical_end -= round_page(MSGBUFSIZE);
	bootconf->dram[0].pages = atop(physical_end);
	initmsgbuf(AARCH64_PA_TO_KVA(physical_end), MSGBUFSIZE);

#ifdef DDB
	db_machdep_init();
#endif

	uvm_md_init();

	/* register free physical memory blocks */
	memsize_total = 0;
	kstartp = atop(kernstart_phys);
	kendp = atop(kernend_phys);

	KDASSERT(bootconf->dramblocks <= DRAM_BLOCKS);
	for (i = 0; i < bootconf->dramblocks; i++) {
		paddr_t startp, endp;

		/* empty is end */
		if (bootconf->dram[i].address == 0 &&
		    bootconf->dram[i].pages == 0)
			break;

		memsize_total += ptoa(bootconf->dram[i].pages);

		startp = atop(bootconf->dram[i].address);
		endp = startp + bootconf->dram[i].pages;

		/* exclude kernel text/data/bss from given block */
		if (kstartp < startp && endp < kendp) {
			/*
			 *  +-------+ kstartp
			 *  |kernel |
			 *  +- - - -+ startp
			 *  |kernel |
			 *  +- - - -+ endp
			 *  |kernel |
			 *  +-------+ kendp
			 */
			__nothing;
		} else if (startp <= kstartp && kendp <= endp) {
			/*
			 *  +-------+ startp
			 *  |///////|
			 *  +-------+ kstartp
			 *  |       |
			 *  |kernel |
			 *  |       |
			 *  +-------+ kendp
			 *  |///////|
			 *  +-------+ endp
			 */
			uvm_page_physload(
			    startp, kstartp,
			    startp, kstartp,
			    bootconf->dram[i].vmfreelist);
			uvm_page_physload(
			    kendp, endp,
			    kendp, endp,
			    bootconf->dram[i].vmfreelist);
		} else if (startp <= kstartp) {
			/*
			 *  +-------+ startp
			 *  |///////|
			 *  +-------+ kstartp
			 *  |kernel |
			 *  |- - - -+ endp
			 *  |kernel |
			 *  +-------+ kendp
			 */
			uvm_page_physload(
			    startp, kstartp,
			    startp, kstartp,
			    bootconf->dram[i].vmfreelist);
		} else {
			/*
			 *  +-------+ kstartp
			 *  |kernel |
			 *  |- - - -+ startp
			 *  |kernel |
			 *  +-------+ kendp
			 *  |///////|
			 *  +-------+ endp
			 */
			uvm_page_physload(
			    kendp, endp,
			    kendp, endp,
			    bootconf->dram[i].vmfreelist);
		}
	}
	physmem = atop(memsize_total);


	/*
	 * kernel image is mapped on L2 table (2MB*n) by locore.S
	 * virtual space start from 2MB aligned kernend
	 */
	pmap_bootstrap(kernend_l2, VM_MAX_KERNEL_ADDRESS);

	/*
	 * setup lwp0
	 */
	uvm_lwp_setuarea(&lwp0, lwp0uspace);
	memset(&lwp0.l_md, 0, sizeof(lwp0.l_md));
	memset(lwp_getpcb(&lwp0), 0, sizeof(struct pcb));

	tf = (struct trapframe *)(lwp0uspace + USPACE) - 1;
	memset(tf, 0, sizeof(struct trapframe));
	tf->tf_spsr = SPSR_M_EL0T;
	lwp0.l_md.md_utf = lwp0.l_md.md_ktf = tf;
}

void
machdep_init(void)
{
	/* clear cpu reset hook for early boot */
	cpu_reset_address0 = NULL;

#if defined(KERNHIST) && defined(TRAPHIST)
	/* XXXAARCH64 */
	void traphist_init(void);
	traphist_init();
#endif
}

bool
mm_md_direct_mapped_phys(paddr_t pa, vaddr_t *vap)
{
	/* XXX */
	if (physical_start <= pa && pa < physical_end) {
		*vap = AARCH64_PA_TO_KVA(pa);
		return true;
	}

	return false;
}

int
mm_md_physacc(paddr_t pa, vm_prot_t prot)
{
	/* XXX */
	if (physical_start <= pa && pa < physical_end)
		return 0;

	return kauth_authorize_machdep(kauth_cred_get(),
	    KAUTH_MACHDEP_UNMANAGEDMEM, NULL, NULL, NULL, NULL);
}

void
cpu_startup(void)
{
	vaddr_t maxaddr, minaddr;

	consinit();

	/*
	 * Allocate a submap for physio.
	 */
	minaddr = 0;
	phys_map = uvm_km_suballoc(kernel_map, &minaddr, &maxaddr,
	   VM_PHYS_SIZE, 0, FALSE, NULL);

	/* Hello! */
	banner();

	bus_space_mallocok();
}

void
cpu_dumpconf(void)
{
}
