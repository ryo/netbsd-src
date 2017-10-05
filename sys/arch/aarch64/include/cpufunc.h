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

extern u_int cputype;

/* misc */
#define cpu_idnum(arg...)		aarch64_cpuid(arg)

/* cache op */
#define cpu_icache_line_size(arg...)	aarch64_dcache_line_size(arg)
#define cpu_dcache_line_size(arg...)	aarch64_icache_line_size(arg)
#define cpu_dcache_wbinv_range(arg...)	aarch64_dcache_wbinv_range(arg)
#define cpu_dcache_inv_range(arg...)	aarch64_dcache_inv_range(arg)
#define cpu_dcache_wb_range(arg...)	aarch64_dcache_wb_range(arg)
#define cpu_idcache_wbinv_range(arg...)	aarch64_idcache_wbinv_range(arg)
#define cpu_icache_sync_range(arg...)	aarch64_icache_sync_range(arg)
#define cpu_sdcache_wbinv_range(arg...)	((void)0)
#define cpu_sdcache_inv_range(arg...)	((void)0)
#define cpu_sdcache_wb_range(arg...)	((void)0)
/* others */
#define cpu_drain_writebuf(arg...)	aarch64_drain_writebuf(arg)


/* cpufunc_asm.S */
void aarch64_nullop(void);
uint32_t aarch64_cpuid(void);
int aarch64_dcache_line_size(void);
int aarch64_icache_line_size(void);
void aarch64_icache_sync_range(vaddr_t, vsize_t);
void aarch64_idcache_wbinv_range(vaddr_t, vsize_t);
void aarch64_dcache_wbinv_range(vaddr_t, vsize_t);
void aarch64_dcache_inv_range(vaddr_t, vsize_t);
void aarch64_dcache_wb_range(vaddr_t, vsize_t);
void aarch64_drain_writebuf(void);

void aarch64_tlbi_all(void);			/* all ASID, all VA */
void aarch64_tlbi_by_asid(int);			/*  an ASID, all VA */
void aarch64_tlbi_by_va(vaddr_t);		/* all ASID, a VA */
void aarch64_tlbi_by_va_ll(vaddr_t);		/* all ASID, a VA, lastlevel */
void aarch64_tlbi_by_asid_va(int, vaddr_t);	/*  an ASID, a VA */
void aarch64_tlbi_by_asid_va_ll(int, vaddr_t);	/*  an ASID, a VA, lastlevel */

#endif /* _KERNEL */

#elif defined(__arm__)

#include <arm/cpufunc.h>

#endif /* __aarch64__/__arm__ */

#endif /* _AARCH64_CPUFUNC_H_ */
