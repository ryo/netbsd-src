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

#define PRINT(string)	stp x0,lr,[sp,#-16]!;bl xprint;.ascii string,"\0";.align 2;ldp x0,lr,[sp],#16

	.global _start
_start:
	stp	x0, lr, [sp, #-16]!

	mov	x9, x0

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






	ldp	x0, lr, [sp], #16
	ret





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
