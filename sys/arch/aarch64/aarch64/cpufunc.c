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

#include <sys/types.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <aarch64/armreg.h>
#include <aarch64/cpufunc.h>

struct cpu_functions cpufuncs;
extern uint32_t cputype;

int
set_cpufuncs(void)
{
	/* misc */
	cpufuncs.cf_nullop = aarch64_nullop;
	cpufuncs.cf_id = aarch64_cpuid;

	/* TLB op */
	cpufuncs.cf_setttb = aarch64_setttb;
	cpufuncs.cf_tlb_flushID = aarch64_tlb_flushID;
	cpufuncs.cf_tlb_flushID_SE = aarch64_tlb_flushID_SE;

	/* cache op */
	cpufuncs.cf_dcache_line_size = aarch64_dcache_line_size;
	cpufuncs.cf_icache_line_size = aarch64_icache_line_size;
	cpufuncs.cf_icache_sync_range = aarch64_icache_sync_range;
	cpufuncs.cf_idcache_wbinv_range = aarch64_idcache_wbinv_range;
	cpufuncs.cf_dcache_wbinv_range = aarch64_dcache_wbinv_range;
	cpufuncs.cf_dcache_inv_range = aarch64_dcache_inv_range;
	cpufuncs.cf_dcache_wb_range = aarch64_dcache_wb_range;
	cpufuncs.cf_sdcache_wbinv_range = (void *)aarch64_nullop;
	cpufuncs.cf_sdcache_wb_range = (void *)aarch64_nullop;
	cpufuncs.cf_sdcache_inv_range = (void *)aarch64_nullop;

	/* others */
	cpufuncs.cf_drain_writebuf = aarch64_drain_writebuf;


	/* for compatible arm */
	cputype = cpu_idnum();

	return 0;
}
