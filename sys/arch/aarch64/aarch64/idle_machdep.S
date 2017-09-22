/* $NetBSD: idle_machdep.S,v 1.1 2014/08/10 05:47:37 matt Exp $ */

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

#include <aarch64/asm.h>
#include <aarch64/locore.h>
#include "assym.h"
#include "opt_arm_intr_impl.h"
#include "opt_ddb.h"

RCSID("$NetBSD: idle_machdep.S,v 1.1 2014/08/10 05:47:37 matt Exp $");

#ifdef ARM_INTR_IMPL
#include ARM_INTR_IMPL
#else
#error ARM_INTR_IMPL not defined
#endif

#ifndef ARM_IRQ_HANDLER
#error ARM_IRQ_HANDLER not defined
#endif


ENTRY(cpu_idle)
	sub	sp, sp, #TF_SIZE		/* allocate trapfame */

	str	x28, [sp, #TF_X28]
	stp	x29, x30, [sp, #TF_X29]
#ifdef DDB
	add	x29, sp, #TF_X29		/* link frame for backtrace */
#endif

	mrs	x1, tpidr_el1			/* get curcpu() */
	ldr	w28, [x1, #CI_INTR_DEPTH]	/* save intr_depth to w28 */
	add	w2, w28, #1
	str	w2, [x1, #CI_INTR_DEPTH]

	/* fill dummy trapframe */
	mov	x2, #SPSR_M_EL1H		/* what our spsr should be */
	str	x2, [sp, #TF_SPSR]
	mov	x0, sp				/* get pointer to trapframe */

	DISABLE_INTERRUPT
	wfi

	bl	ARM_IRQ_HANDLER			/* irqhandler(trapframe) */

	mrs	x1, tpidr_el1			/* get curcpu() */
	str	w28, [x1, #CI_INTR_DEPTH]	/* restore intr_depth */

	ldr	x28, [sp, #TF_X28]		/* restore registers */
	ldp	x29, x30, [sp, #TF_X29]		/* restore registers */
	add	sp, sp, #TF_SIZE		/* pop trapframe */

	ENABLE_INTERRUPT

	ldr	w3, [x1, #CI_SOFTINTS]		/* Get pending softint mask */
#if 0
	/* CPL should be 0 */
	ldr	w2, [x1, #CI_CPL]		/* Get current priority level */
	lsr	w3, w3, w2			/* shift mask by cpl */
#endif
	cbnz	w3, _C_LABEL(dosoftints)	/* tailcall dosoftints(void) */
	ret
END(cpu_idle)
