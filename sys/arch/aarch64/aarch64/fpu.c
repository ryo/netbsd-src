/* $NetBSD: fpu.c,v 1.1 2014/08/10 05:47:37 matt Exp $ */

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

__KERNEL_RCSID(1, "$NetBSD: fpu.c,v 1.1 2014/08/10 05:47:37 matt Exp $");

#include <sys/param.h>
#include <sys/types.h>
#include <sys/lwp.h>

#include <aarch64/cpu.h>
#include <aarch64/reg.h>
#include <aarch64/pcb.h>
#include <aarch64/armreg.h>

static void fpu_state_load(lwp_t *, unsigned int);
static void fpu_state_save(lwp_t *);
static void fpu_state_release(lwp_t *);

const pcu_ops_t pcu_fpu_ops = {
	.pcu_id = PCU_FPU,
	.pcu_state_load = fpu_state_load,
	.pcu_state_save = fpu_state_save,
	.pcu_state_release = fpu_state_release,
};

static void
fpu_state_load(lwp_t *l, unsigned int flags)
{
	struct pcb *pcb = lwp_getpcb(l);
	struct fpreg *fp = &pcb->pcb_fpregs;

	KASSERT(l->l_pcu_cpu[PCU_FPU] == curcpu());

	if ((flags & PCU_VALID) == 0)
		memset(fp, 0, sizeof(*fp)); /* load full of zeros */

	/* allow user process to use FP */
	l->l_md.md_cpacr = CPACR_FPEN_ALL;

	/* turn on FP to load its state */
	reg_cpacr_el1_write(CPACR_FPEN_EL1);

	__asm __volatile(
	"ldp q0, q1, [%0, #16 * 0];"
	"ldp q2, q3, [%0, #16 * 2];"
	"ldp q4, q5, [%0, #16 * 4];"
	"ldp q6, q7, [%0, #16 * 6];"
	"ldp q8, q9, [%0, #16 * 8];"
	"ldp q10, q11, [%0, #16 * 10];"
	"ldp q12, q13, [%0, #16 * 12];"
	"ldp q14, q15, [%0, #16 * 14];"
	"ldp q16, q17, [%0, #16 * 16];"
	"ldp q18, q19, [%0, #16 * 18];"
	"ldp q20, q21, [%0, #16 * 20];"
	"ldp q22, q23, [%0, #16 * 22];"
	"ldp q24, q25, [%0, #16 * 24];"
	"ldp q26, q27, [%0, #16 * 26];"
	"ldp q28, q29, [%0, #16 * 28];"
	"ldp q30, q31, [%0, #16 * 30]! ;"
	"ldr w8, [%0, #0];"
	"ldr w9, [%0, #4];"
	"msr fpcr, x8;"
	"msr fpsr, x9" :: "r"(fp));

	/* turn off FP again */
	reg_cpacr_el1_write(CPACR_FPEN_NONE);
}

static void
fpu_state_save(lwp_t *l)
{
	struct pcb *pcb = lwp_getpcb(l);
	struct fpreg *fp = &pcb->pcb_fpregs;

	KASSERT(l->l_pcu_cpu[PCU_FPU] == curcpu());

	/* turn on FP to retrieve its state */
	reg_cpacr_el1_write(CPACR_FPEN_EL1);

	__asm __volatile(
	"stp q0, q1, [%0, #16 * 0];"
	"stp q2, q3, [%0, #16 * 2];"
	"stp q4, q5, [%0, #16 * 4];"
	"stp q6, q7, [%0, #16 * 6];"
	"stp q8, q9, [%0, #16 * 8];"
	"stp q10, q11, [%0, #16 * 10];"
	"stp q12, q13, [%0, #16 * 12];"
	"stp q14, q15, [%0, #16 * 14];"
	"stp q16, q17, [%0, #16 * 16];"
	"stp q18, q19, [%0, #16 * 18];"
	"stp q20, q21, [%0, #16 * 20];"
	"stp q22, q23, [%0, #16 * 22];"
	"stp q24, q25, [%0, #16 * 24];"
	"stp q26, q27, [%0, #16 * 26];"
	"stp q28, q29, [%0, #16 * 28];"
	"stp q30, q31, [%0, #16 * 30]! ;"
	"mrs x8, fpcr;"
	"mrs x9, fpsr;"
	"str w8, [%0, #0];"
	"str w9, [%0, #4]" :: "r"(fp));

	/* turn off FP again */
	reg_cpacr_el1_write(CPACR_FPEN_NONE);
}

static void
fpu_state_release(lwp_t *l)
{

	l->l_md.md_cpacr = CPACR_FPEN_NONE;
	if (l == curlwp)
		reg_cpacr_el1_write(l->l_md.md_cpacr);
}
