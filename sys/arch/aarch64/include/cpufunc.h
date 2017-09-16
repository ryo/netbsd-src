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

#ifndef _AARCH64_CPUFUNC_H_
#define _AARCH64_CPUFUNC_H_

#ifdef __aarch64__

#ifdef _KERNEL

struct cpu_functions {
	/* misc */
	uint32_t (*cf_id)(void);
	void (*cf_nullop)(void);

	/* TLB op */
	void (*cf_setttb)(paddr_t);
	void (*cf_tlb_flushID)(void);
	void (*cf_tlb_flushID_SE)(vaddr_t);

	/* cache op */
	int (*cf_dcache_line_size)(void);
	int (*cf_icache_line_size)(void);
	void (*cf_icache_sync_range)(vaddr_t, vsize_t);
	void (*cf_idcache_wbinv_range)(vaddr_t, vsize_t);
	void (*cf_dcache_wbinv_range)(vaddr_t, vsize_t);
	void (*cf_dcache_inv_range)(vaddr_t, vsize_t);
	void (*cf_dcache_wb_range)(vaddr_t, vsize_t);
	void (*cf_sdcache_wbinv_range)(vaddr_t, paddr_t, vsize_t);
	void (*cf_sdcache_inv_range)(vaddr_t, paddr_t, vsize_t);
	void (*cf_sdcache_wb_range)(vaddr_t, paddr_t, vsize_t);

	/* others */
	void (*cf_drain_writebuf)(void);
};

extern struct cpu_functions cpufuncs;
extern u_int cputype;

/* misc */
#define cpufunc_nullop(args...)		cpufuncs.cf_nullop(args)
#define cpu_idnum(args...)		cpufuncs.cf_id(args)
/* TLB op */
#define cpu_setttb(args...)		cpufuncs.cf_setttb(args)
#define cpu_tlb_flushID(args...)	cpufuncs.cf_tlb_flushID(args)
#define cpu_tlb_flushID_SE(args...)	cpufuncs.cf_tlb_flushID_SE(args)
/* cache op */
#define cpu_icache_line_size(args...)	cpufuncs.cf_dcache_line_size(args)
#define cpu_dcache_line_size(args...)	cpufuncs.cf_icache_line_size(args)
#define cpu_dcache_wbinv_range(args...)	cpufuncs.cf_dcache_wbinv_range(args)
#define cpu_dcache_inv_range(args...)	cpufuncs.cf_dcache_inv_range(args)
#define cpu_dcache_wb_range(args...)	cpufuncs.cf_dcache_wb_range(args)
#define cpu_idcache_wbinv_range(args...) cpufuncs.cf_idcache_wbinv_range(args)
#define cpu_icache_sync_range(args...)	cpufuncs.cf_icache_sync_range(args)
#define cpu_sdcache_wbinv_range(args...) cpufuncs.cf_sdcache_wbinv_range(args)
#define cpu_sdcache_inv_range(args...)	cpufuncs.cf_sdcache_inv_range(args)
#define cpu_sdcache_wb_range(args...)	cpufuncs.cf_sdcache_wb_range(args)
/* others */
#define cpu_drain_writebuf(args...)	cpufuncs.cf_drain_writebuf(args)


int set_cpufuncs(void);

/* cpufunc_asm.S */
void aarch64_nullop(void);
uint32_t aarch64_cpuid(void);
void aarch64_setttb(paddr_t);
void aarch64_tlb_flushID(void);
void aarch64_tlb_flushID_SE(vaddr_t);
int aarch64_dcache_line_size(void);
int aarch64_icache_line_size(void);
void aarch64_icache_sync_range(vaddr_t, vsize_t);
void aarch64_idcache_wbinv_range(vaddr_t, vsize_t);
void aarch64_dcache_wbinv_range(vaddr_t, vsize_t);
void aarch64_dcache_inv_range(vaddr_t, vsize_t);
void aarch64_dcache_wb_range(vaddr_t, vsize_t);
void aarch64_sdcache_wbinv_range(vaddr_t, paddr_t, vsize_t);
void aarch64_sdcache_inv_range(vaddr_t, paddr_t, vsize_t);
void aarch64_sdcache_wb_range(vaddr_t, paddr_t, vsize_t);
void aarch64_drain_writebuf(void);

#endif /* _KERNEL */

#elif defined(__arm__)

#include <arm/cpufunc.h>

#endif /* __aarch64__/__arm__ */

#endif /* _AARCH64_CPUFUNC_H_ */
