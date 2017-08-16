/* $NetBSD*/

/*-
 * Copyright (c) 2012-2014 Andrew Turner
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD: head/sys/arm64/arm64/locore.S 316755 2017-04-13 11:56:27Z andrew $
 */

#include "opt_cpuoptions.h"
#include "opt_cputypes.h"
#include "opt_multiprocessor.h"
#include "opt_ddb.h"

#include <aarch64/asm.h>
#include <aarch64/hypervisor.h>
#include "assym.h"

RCSID("$NetBSD$")

//XXXAARCH64
#define AUX_MU_IO_REG	0x3f215040	/* Mini Uart I/O Data (8bit) */
#define AUX_MU_IER_REG	0x3f215044	/* Mini Uart Interrupt Enable (8bit) */
#define AUX_MU_IIR_REG	0x3f215048	/* Mini Uart Interrupt Identify (8bit) */
#define AUX_MU_LCR_REG	0x3f21504C	/* Mini Uart Line Control (8bit) */
#define AUX_MU_MCR_REG	0x3f215050	/* Mini Uart Modem Control (8bit) */
#define AUX_MU_LSR_REG	0x3f215054	/* Mini Uart Line Status (8bit) */
#define AUX_MU_MSR_REG	0x3f215058	/* Mini Uart Modem Status (8bit) */
#define AUX_MU_SCRATCH	0x3f21505C	/* Mini Uart Scratch (8bit) */
#define AUX_MU_CNTL_REG	0x3f215060	/* Mini Uart Extra Control (8bit) */
#define AUX_MU_STAT_REG	0x3f215064	/* Mini Uart Extra Status (32bit) */
#define AUX_MU_BAUD_REG	0x3f215068	/* Mini Uart Baudrate (16bit) */

#define COM_DATA		AUX_MU_IO_REG
#define COM_LSR			AUX_MU_LSR_REG
#define COM_LSR_TXFE_BIT	5	/* Transmit fifo empty */

#define PRINT(string)	stp x0,lr,[sp,#-16]!;bl xprint;.asciz string;.align 2;ldp x0,lr,[sp],#16

	.global _start
_start:
	stp	x0, lr, [sp, #-16]!

	mov	x9, x0

	mrs	x0, SP_EL0
	PRINT("SP_EL0=")
	bl	print_x0
	PRINT("\r\n")

	mrs	x0, SP_EL1
	PRINT("SP_EL1=")
	bl	print_x0
	PRINT("\r\n")

	mov	x0, sp
	msr	SP_EL1, x0

	PRINT("LR=")
	mov	x0, lr
	bl	print_x0
	PRINT("\r\n")

	PRINT("SP=")
	mov	x0, sp
	bl	print_x0
	PRINT("\r\n")

	PRINT("PC=")
	bl	1f
1:	mov	x0, lr
	bl	print_x0
	PRINT("\r\n")

	mov	x0, x9
	PRINT("X0=")
	bl	print_x0
	PRINT("\r\n")

	PRINT("X1=")
	mov	x0, x1
	bl	print_x0
	PRINT("\r\n")

	PRINT("X2=")
	mov	x0, x2
	bl	print_x0
	PRINT("\r\n")

	PRINT("X3=")
	mov	x0, x3
	bl	print_x0
	PRINT("\r\n")

	PRINT("X4=")
	mov	x0, x4
	bl	print_x0
	PRINT("\r\n")

	PRINT("X5=")
	mov	x0, x5
	bl	print_x0
	PRINT("\r\n")


	PRINT("Drop to EL1\r\n")

	/* Drop to EL1 */
	bl	drop_to_el1

	PRINT("Drop to EL1 OK\r\n")

	ldp	x0, lr, [sp], #16
	ret



/*
 * If we are started in EL2, configure the required hypervisor
 * registers and drop to EL1.
 */
drop_to_el1:
	mrs	x1, CurrentEL
	lsr	x1, x1, #2
	cmp	x1, #0x2
	b.eq	1f
	ret
1:
	/* Configure the Hypervisor */
	mov	x2, #(HCR_RW)
	msr	hcr_el2, x2

	/* Load the Virtualization Process ID Register */
	mrs	x2, midr_el1
	msr	vpidr_el2, x2

	/* Load the Virtualization Multiprocess ID Register */
	mrs	x2, mpidr_el1
	msr	vmpidr_el2, x2

	/* Set the bits that need to be 1 in sctlr_el1 */
	ldr	x2, .Lsctlr_res1
	msr	sctlr_el1, x2

	/* Don't trap to EL2 for exceptions */
	mov	x2, #CPTR_RES1
	msr	cptr_el2, x2

	/* Don't trap to EL2 for CP15 traps */
	msr	hstr_el2, xzr

	/* Enable access to the physical timers at EL1 */
	mrs	x2, cnthctl_el2
	orr	x2, x2, #(CNTKCTL_EL0PCTEN | CNTKCTL_EL0VCTEN)
	msr	cnthctl_el2, x2

	/* Set the counter offset to a known value */
	msr	cntvoff_el2, xzr

	/* Hypervisor trap functions */
	adr	x2, hyp_vectors
	msr	vbar_el2, x2

	mov	x2, #(SPSR_F | SPSR_I | SPSR_A | SPSR_A64_D | SPSR_M_EL1H)
	msr	spsr_el2, x2

	/* Configure GICv3 CPU interface */
	mrs	x2, id_aa64pfr0_el1
	/* Extract GIC bits from the register */
	and	x2, x2, ID_AA64PFR0_EL1_GIC
	lsr	x2, x2, ID_AA64PFR0_EL1_GIC_SHIFT
	/* GIC[3:0] == 0001 - GIC CPU interface via special regs. supported */
	cmp	x2, #ID_AA64PFR0_EL1_GIC_CPUIF_EN
	b.ne	2f

	mrs	x2, icc_sre_el2
	orr	x2, x2, #ICC_SRE_EL2_EN	/* Enable access from insecure EL1 */
	orr	x2, x2, #ICC_SRE_EL2_SRE	/* Enable system registers */
	msr	icc_sre_el2, x2
2:

	/* Set the address to return to our return address */
	msr	elr_el2, x30
	isb

	eret

	.align 3
.Lsctlr_res1:
	.quad SCTLR_RES1

#define	VECT_EMPTY	\
	.align 7;	\
	1:	b	1b

	.align 11
hyp_vectors:
	VECT_EMPTY	/* Synchronous EL2t */
	VECT_EMPTY	/* IRQ EL2t */
	VECT_EMPTY	/* FIQ EL2t */
	VECT_EMPTY	/* Error EL2t */

	VECT_EMPTY	/* Synchronous EL2h */
	VECT_EMPTY	/* IRQ EL2h */
	VECT_EMPTY	/* FIQ EL2h */
	VECT_EMPTY	/* Error EL2h */

	VECT_EMPTY	/* Synchronous 64-bit EL1 */
	VECT_EMPTY	/* IRQ 64-bit EL1 */
	VECT_EMPTY	/* FIQ 64-bit EL1 */
	VECT_EMPTY	/* Error 64-bit EL1 */

	VECT_EMPTY	/* Synchronous 32-bit EL1 */
	VECT_EMPTY	/* IRQ 32-bit EL1 */
	VECT_EMPTY	/* FIQ 32-bit EL1 */
	VECT_EMPTY	/* Error 32-bit EL1 */

/*
 * xprint - print strings pointed by $PC(LR)
 *          and return to the end of string.
 *          all registers are saved.
 * e.g.)
 *    bl        xprint        <- call
 *    .ascii    "Hello\r\n\0" <- wouldn't return here
 *    .align    2
 *    nop                     <- return to here
 */
	.global _C_LABEL(xprint)
_C_LABEL(xprint):
	stp	x0, x1, [sp, #-16]!
	mov	x0, lr
	bl	uartputs

1:	ldrb	w1, [x0], #1
	cbnz	w1, 1b

	add	x0, x0, #3
	bic	lr, x0, #3

	ldp	x0, x1, [sp], #16
	ret
END(_C_LABEL(xprint))


	.global _C_LABEL(print_x0)
_C_LABEL(print_x0):
	stp	x0, lr, [sp, #-16]!
	stp	x4, x5, [sp, #-16]!
	stp	x6, x7, [sp, #-16]!

	mov	x7, x0		/* number to display */
	mov	x4, #60		/* num of shift */
	mov	x5, #0xf	/* mask */
1:
	ror	x0, x7, x4
	and	x0, x0, x5
	cmp	x0, #10
	blt	2f
	add	x0, x0, #('a' - 10 - '0')
2:	add	x0, x0, #'0'
	bl	uartputc
	subs	x4, x4, #4
	bge	1b

	ldp	x6, x7, [sp], #16
	ldp	x4, x5, [sp], #16
	ldp	x0, lr, [sp], #16
	ret
END(_C_LABEL(print_x0))


	.global _C_LABEL(uartputs)
_C_LABEL(uartputs):
	stp	x8, lr, [sp, #-16]!
	stp	x0, x1, [sp, #-16]!

	mov	x8, x0
	ldrb	w0, [x8], #1
	cbz	w0, 9f
1:	bl	uartputc
	ldrb	w0, [x8], #1
	cbnz	w0, 1b
9:
	ldp	x0, x1, [sp], #16
	ldp	x8, lr, [sp], #16
	ret
END(_C_LABEL(uartputs))


	.global _C_LABEL(uartputc)
_C_LABEL(uartputc):
	stp	x9, x10, [sp, #-16]!
	stp	x0, x1, [sp, #-16]!

	mov	w9, #(COM_LSR & 0xffff)
	movk	w9, #(COM_LSR >> 16), lsl #16
1:	ldr	w10, [x9]
	tbz	w10, #COM_LSR_TXFE_BIT, 1b

	mov	w9, #(COM_DATA & 0xffff)
	movk	w9, #(COM_DATA >> 16), lsl #16
	and	w0, w0, #0xff
	str	w0, [x9]

	ldp	x0, x1, [sp], #16
	ldp	x9, x10, [sp], #16
	ret
END(_C_LABEL(uartputc))
