/* $NetBSD$ */

#include <aarch64/asm.h>
#include "assym.h"

lr	.req	x30	/* link register */

	 .macro	ventry	label
	.align	7
	b	\label
	.endm

	.pushsection ".entry.text", "ax"
	.p2align 11
ENTRY(vector_start)
#if 0
/*
 * Exception taken from current Exception Level with SP_EL1.
 * (These shouldn't happen)
 */
	ventry	el1_sync_invalid	/* synchronous EL1t */
	ventry	el1_irq_invalid		/* IRQ EL1t */
	ventry	el1_frq_invalid		/* FIQ EL1t */
	ventry	e11_serr_invalid	/* error EL1t */
/*
 * Exception takend form current Exception Level with SP_EL1.
 * There are entries for exceptions caused in EL1 (kernel exceptions).
 */
	ventry	el1_sync		/* synchronous EL1h */
	ventry	el1_irq			/* IRQ EL1h */
	ventry	el1_frq_invalid		/* FRQ EL1h */
	ventry	e1l_serr_invalid	/* error EL1h */
/*
 * Exception taken from lower Exception Level which is using AArch64
 * There are entries for exceptions caused in EL0 (native user exceptions).
 */
	ventry	el0_sync		/* synchrous 64bit EL0 */
	ventry	el0_irq			/* IRQ 64bit EL0 */
	ventry	el0_frq_invalid		/* FRQ 64bit EL0 */
	ventry	e10_serr_invalid	/* error 64bit EL0 */
/*
 * Exception taken from lower Exception Level which is using AArch32
 * There are entries for exceptions caused in EL0 (compat user exceptions).
 */
	ventry	el0_32sync		/* synchronous 32bit EL0 */
	ventry	el0_32irq		/* IRQ 32bit EL0 */
	ventry	el0_32frq_invalid	/* FIQ 32bit EL0 */
	ventry	e10_32serr_invalid	/* error 32bit EL0 */
#endif
ENTRY(vector_end)
