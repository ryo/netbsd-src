/* $NetBSD*/

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

#include <aarch64/asm.h>
#include "assym.h"

#include "opt_ddb.h"

RCSID("$NetBSD$")

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

_C_LABEL(rpi_start):
	str	lr, [sp, #-8]!
	bl	test
	ldr	lr, [sp], #8
	ret
END(_C_LABEL(rpi_start))


Hello:
	.ascii	"HelloWorld\r\n\0"
	.align 2

_C_LABEL(test):
	str	lr, [sp, #-8]!
	adr	x0, Hello
	bl	uartputs
	ldr	lr, [sp], #8
	ret
END(_C_LABEL(test))


	.global _C_LABEL(uartputs)
_C_LABEL(uartputs):
	stp	x8, lr, [sp, #-16]!
	mov	x8, x0
	ldrb	w0, [x8], #1
	cbz	w0, 9f
1:	bl	uartputc
	ldrb	w0, [x8], #1
	cbnz	w0, 1b
9:	ldp	x8, lr, [sp], #16
	ret
END(_C_LABEL(uartputs))


	.global _C_LABEL(uartputc)
_C_LABEL(uartputc):
	stp	x9, x10, [sp, #-16]!

	mov	w9, #(COM_LSR & 0xffff)
	movk	w9, #(COM_LSR >> 16), lsl #16
1:	ldr	w10, [x9]
	tbz	w10, #COM_LSR_TXFE_BIT, 1b

	mov	w9, #(COM_DATA & 0xffff)
	movk	w9, #(COM_DATA >> 16), lsl #16
	and	w0, w0, #0xff
	str	w0, [x9]

	ldp	x9, x10, [sp], #16
	ret
END(_C_LABEL(uartputc))
