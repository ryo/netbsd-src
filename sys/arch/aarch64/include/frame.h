/* $NetBSD: frame.h,v 1.1 2014/08/10 05:47:38 matt Exp $ */

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

#ifndef _AARCH64_FRAME_H_
#define _AARCH64_FRAME_H_

#ifdef __aarch64__

#include <aarch64/reg.h>

struct trapframe {
	struct reg tf_regs __aligned(16);
	uint64_t tf_esr;		// 32-bit register
	uint64_t tf_far;		// 64-bit register
#define tf_reg		tf_regs.r_reg
#define tf_lr		tf_regs.r_reg[30]
#define tf_pc		tf_regs.r_pc
#define tf_sp		tf_regs.r_sp
#define tf_spsr		tf_regs.r_spsr
#define tf_tpidr	tf_regs.r_tpidr
};

#ifdef _KERNEL
/* size of trapframe (stack pointer) must be 16byte aligned */
__CTASSERT((sizeof(struct trapframe) & 15) == 0);
#endif

#define TF_SIZE		sizeof(struct trapframe)

/* label_t in <machine/types.h> for kernel setjmp/longjmp, and onfault buf */
#define LBL_X19	0
#define LBL_X20	1
#define LBL_X21	2
#define LBL_X22	3
#define LBL_X23	4
#define LBL_X24	5
#define LBL_X25	6
#define LBL_X26	7
#define LBL_X27	8
#define LBL_X28	9
#define LBL_X29	10
#define LBL_LR	11
#define LBL_SP	12
#define LBL_MAX	13

#elif defined(__arm__)

#include <arm/frame.h>

#endif /* __aarch64__/__arm__ */

#endif /* _AARCH64_FRAME_H_ */
