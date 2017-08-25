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
__KERNEL_RCSID(1, "$NetBSD$");

#include "opt_ddb.h"
#include "opt_kgdb.h"
#include "opt_multiprocessor.h"
#include "opt_machdep.h"

#include <sys/param.h>
#include <uvm/uvm.h>

#include <aarch64/vmparam.h>
#include <aarch64/machdep.h>
#include <aarch64/locore.h>
#include <aarch64/pcb.h>

//XXXAARCH64
vaddr_t physical_start;
vaddr_t physical_end;

extern char _start[];
extern char etext[];
extern char __data_start[], _edata[];
extern char __bss_start[], __bss_end__[];
extern char _end[];


/*
 * Upper region: 0xffffffffffffffff  Top of virtual memory
 *
 *               0xffffffffffff0000  End of KVA
 *                                   = VM_MAX_KERNEL_ADDRESS
 *
 *               0xffffffc00???????  End of kernel
 *                                   = _end[]
 *               0xffffffc000??????  Start of kernel
 *                                   = _start[]
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
 *                                   = VM_MAXUSER_ADDRESS)
 *
 *               0x000000000???????  End of Loaded kernel image
 *               0x0000000000??????  Start of Loaded kernel image
 *                                   = LOADADDRESS
 *
 *               0x0000000000000000  Start of user address space
 */
vaddr_t
aarch64_kvminit(vaddr_t ksp)
{
	struct lwp *l;
	struct trapframe *tf;
	psize_t memsize;
	vaddr_t kernstart, kernend;
	paddr_t kernstart_phys, kernend_phys;

	kernstart = trunc_page((vaddr_t)_start);
	kernend = round_page((vaddr_t)_end);
	kernstart_phys = kernstart - VM_MIN_KERNEL_ADDRESS;
	kernend_phys = kernend - VM_MIN_KERNEL_ADDRESS;

	memsize = physical_end - physical_start;
	physmem = memsize / PAGE_SIZE;

#ifdef VERBOSE_INIT_ARM
	printf(
	    "%s: physical_start    = 0x%016lx\n"
	    "%s: physical_end      = 0x%016lx\n"
	    "%s: kernel_start_phys = 0x%016lx\n"
	    "%s: kernel_end_phys   = 0x%016lx\n"
	    "%s: kernel_start      = 0x%016lx\n"
	    "%s: kernel_end        = 0x%016lx\n",
	    __func__, physical_start,
	    __func__, physical_end,
	    __func__, kernstart_phys,
	    __func__, kernend_phys,
	    __func__, kernstart,
	    __func__, kernend);
#endif

	uvm_md_init();
	uvm_page_physload(
	    atop(kernend_phys), atop(physical_end),
	    atop(kernend_phys), atop(physical_end),
	    VM_FREELIST_DEFAULT);
	uvm_page_physload(
	    atop(physical_start), atop(kernend_phys),
	    atop(physical_start), atop(kernend_phys),
	    VM_FREELIST_DEFAULT);



	uvm_lwp_setuarea(&lwp0, ksp);

	l = &lwp0;
	tf = (struct trapframe *)(ksp + USPACE) - 1;
	memset(tf, 0, sizeof(struct trapframe));
	memset(lwp_getpcb(l), 0, sizeof(struct pcb));
	memset(&l->l_md, 0, sizeof(l->l_md));
	l->l_md.md_utf = l->l_md.md_ktf = tf;
	tf->tf_spsr = SPSR_M_EL0T;
	return (vaddr_t)tf;
}
