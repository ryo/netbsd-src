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

#ifdef _KERNEL

struct cpu_functions {
	void (*cf_nullop)(void);
	void (*cf_setttb)(paddr_t);
	void (*cf_tlb_flushID)(void);
	void (*cf_tlb_flushID_SE)(vaddr_t);
	void (*cf_icache_sync_range)(vaddr_t, vsize_t);
	void (*cf_idcache_wbinv_range)(vaddr_t, vsize_t);
	void (*cf_dcache_wbinv_range)(vaddr_t, vsize_t);
	void (*cf_dcache_inv_range)(vaddr_t, vsize_t);
	void (*cf_dcache_wb_range)(vaddr_t, vsize_t);
};

extern struct cpu_functions cpufuncs;
extern u_int cputype;

#define cpufunc_nullop()		cpufuncs.cf_nullop()
#define cpu_setttb(p)			cpufuncs.cf_setttb(p)
#define cpu_tlb_flushID()		cpufuncs.cf_tlb_flushID()
#define cpu_tlb_flushID_SE(v)		cpufuncs.cf_tlb_flushID_SE(v)
#define cpu_dcache_wbinv_range(v, s)	cpufuncs.cf_dcache_wbinv_range(v, s)
#define cpu_dcache_inv_range(v, s)	cpufuncs.cf_dcache_inv_range(v, s)
#define cpu_dcache_wb_range(v, s)	cpufuncs.cf_dcache_wb_range(v, s)
#define cpu_idcache_wbinv_range(v, s)	cpufuncs.cf_idcache_wbinv_range(v, s)
#define cpu_icache_sync_range(v, s)	cpufuncs.cf_icache_sync_range(v, s)

int set_cpufuncs(void);

/* cpufunc_asm.S */
void aarch64_nullop(void);
void aarch64_setttb(paddr_t);
void aarch64_tlb_flushID(void);
void aarch64_tlb_flushID_SE(vaddr_t);
void aarch64_icache_sync_range(vaddr_t, vsize_t);
void aarch64_idcache_wbinv_range(vaddr_t, vsize_t);
void aarch64_dcache_wbinv_range(vaddr_t, vsize_t);
void aarch64_dcache_inv_range(vaddr_t, vsize_t);
void aarch64_dcache_wb_range(vaddr_t, vsize_t);

#endif /* _KERNEL */
#endif /* _AARCH64_CPUFUNC_H_ */
