/* $NetBSD$ */

#include <aarch64/asm.h>
#include "assym.h"

lr	.req	x30	/* link register */

	 .macro	VECT_ENTRY label
	.align	7
	b	\label
	.endm

	.pushsection ".entry.text", "ax"
	.p2align 11
ENTRY(el1_vectors)
/*
 * Exception taken from current Exception Level with SP_EL1.
 * (These shouldn't happen)
 */
	VECT_ENTRY	el1_sync_invalid	/* Synchronous EL1t */
	VECT_ENTRY	el1_irq_invalid		/* IRQ EL1t */
	VECT_ENTRY	el1_fiq_invalid		/* FIQ EL1t */
	VECT_ENTRY	el1_serr_invalid	/* Error EL1t */
/*
 * Exception takend form current Exception Level with SP_EL1.
 * There are entries for exceptions caused in EL1 (kernel exceptions).
 */
	VECT_ENTRY	el1_sync		/* Synchronous EL1h */
	VECT_ENTRY	el1_irq			/* IRQ EL1h */
	VECT_ENTRY	el1_fiq_invalid		/* FRQ EL1h */
	VECT_ENTRY	el1_serr_invalid	/* Error EL1h */
/*
 * Exception taken from lower Exception Level which is using AArch64
 * There are entries for exceptions caused in EL0 (native user exceptions).
 */
	VECT_ENTRY	el0_sync		/* Synchronous 64bit EL0 */
	VECT_ENTRY	el0_irq			/* IRQ 64bit EL0 */
	VECT_ENTRY	el0_fiq_invalid		/* FRQ 64bit EL0 */
	VECT_ENTRY	el0_serr_invalid	/* Error 64bit EL0 */
/*
 * Exception taken from lower Exception Level which is using AArch32
 * There are entries for exceptions caused in EL0 (compat user exceptions).
 */
	VECT_ENTRY	el0_32sync		/* Synchronous 32bit EL0 */
	VECT_ENTRY	el0_32irq		/* IRQ 32bit EL0 */
	VECT_ENTRY	el0_fiq_invalid		/* FIQ 32bit EL0 */
	VECT_ENTRY	el0_serr_invalid	/* Error 32bit EL0 */

ENTRY(el1_sync_invalid)
ENTRY(el1_irq_invalid)
ENTRY(el1_fiq_invalid)
ENTRY(el1_serr_invalid)
ENTRY(el0_sync_invalid)
ENTRY(el0_irq_invalid)
ENTRY(el0_fiq_invalid)
ENTRY(el0_serr_invalid)
ENTRY(el1_sync)
ENTRY(el1_irq)
ENTRY(el0_sync)
ENTRY(el0_irq)
ENTRY(el0_32sync)
ENTRY(el0_32irq)
	1:	b	1b
