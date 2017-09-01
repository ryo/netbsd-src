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

#include <machine/asm.h>

#define NOT_IMPL(func, methodstr)					\
	.global _C_LABEL(func)					;	\
_C_LABEL(func):							;	\
	mov	x2, x0						;	\
	adr	x1, 2f						;	\
	adr	x0, 1f						;	\
	b	_C_LABEL(panic)					;	\
1:	.asciz	"bus_space_%s() is not implemented on this tag: %p";	\
2:	.asciz	methodstr					;	\
	.align	2

/* misc */
NOT_IMPL(bs_notimpl_mmap, "mmap")

/* peek */
NOT_IMPL(bs_notimpl_pe_1, "peek_1")
NOT_IMPL(bs_notimpl_pe_2, "peek_2")
NOT_IMPL(bs_notimpl_pe_4, "peek_4")
NOT_IMPL(bs_notimpl_pe_8, "peek_8")

/* poke */
NOT_IMPL(bs_notimpl_po_1, "poke_1")
NOT_IMPL(bs_notimpl_po_2, "poke_2")
NOT_IMPL(bs_notimpl_po_4, "poke_4")
NOT_IMPL(bs_notimpl_po_8, "poke_8")

/* read */
NOT_IMPL(bs_notimpl_r_1, "read_1")
NOT_IMPL(bs_notimpl_r_2, "read_2")
NOT_IMPL(bs_notimpl_r_4, "read_4")
NOT_IMPL(bs_notimpl_r_8, "read_8")

/* read_multi */
NOT_IMPL(bs_notimpl_rm_1, "read_multi_1")
NOT_IMPL(bs_notimpl_rm_2, "read_multi_2")
NOT_IMPL(bs_notimpl_rm_4, "read_multi_4")
NOT_IMPL(bs_notimpl_rm_8, "read_multi_8")

/* read_region */
NOT_IMPL(bs_notimpl_rr_1, "read_region_1")
NOT_IMPL(bs_notimpl_rr_2, "read_region_2")
NOT_IMPL(bs_notimpl_rr_4, "read_region_4")
NOT_IMPL(bs_notimpl_rr_8, "read_region_8")

/* set_multi */
NOT_IMPL(bs_notimpl_sm_1, "set_multi_1")
NOT_IMPL(bs_notimpl_sm_2, "set_multi_2")
NOT_IMPL(bs_notimpl_sm_4, "set_multi_4")
NOT_IMPL(bs_notimpl_sm_8, "set_multi_8")

/* set_region */
NOT_IMPL(bs_notimpl_sr_1, "set_region_1")
NOT_IMPL(bs_notimpl_sr_2, "set_region_2")
NOT_IMPL(bs_notimpl_sr_4, "set_region_4")
NOT_IMPL(bs_notimpl_sr_8, "set_region_8")

/* write */
NOT_IMPL(bs_notimpl_w_1, "write_1")
NOT_IMPL(bs_notimpl_w_2, "write_2")
NOT_IMPL(bs_notimpl_w_4, "write_4")
NOT_IMPL(bs_notimpl_w_8, "write_8")

/* write_multi */
NOT_IMPL(bs_notimpl_wm_1, "write_multi_1")
NOT_IMPL(bs_notimpl_wm_2, "write_multi_2")
NOT_IMPL(bs_notimpl_wm_4, "write_multi_4")
NOT_IMPL(bs_notimpl_wm_8, "write_multi_8")

/* write_region */
NOT_IMPL(bs_notimpl_wr_1, "write_region_1")
NOT_IMPL(bs_notimpl_wr_2, "write_region_2")
NOT_IMPL(bs_notimpl_wr_4, "write_region_4")
NOT_IMPL(bs_notimpl_wr_8, "write_region_8")

/* copy_region */
NOT_IMPL(bs_notimpl_c_1, "copy_region_1")
NOT_IMPL(bs_notimpl_c_2, "copy_region_2")
NOT_IMPL(bs_notimpl_c_4, "copy_region_4")
NOT_IMPL(bs_notimpl_c_8, "copy_region_8")
