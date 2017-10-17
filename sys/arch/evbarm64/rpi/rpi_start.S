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

#include "opt_arm_debug.h"	/* VERBOSE_INIT_ARM */

#include <aarch64/asm.h>
RCSID("$NetBSD$")


#define AUX_MU_IO_REG	0x3f215040	/* Mini Uart I/O Data (8bit) */
#define AUX_MU_LSR_REG	0x3f215054	/* Mini Uart Line Status (8bit) */

#define COM_DATA		AUX_MU_IO_REG
#define COM_LSR			AUX_MU_LSR_REG
#define COM_LSR_TXFE_BIT	5	/* Transmit fifo empty */
#define COM_LSR_RXRDY_BIT	0	/* Byte ready in Receive Buffer */

#define PRINT(string)	\
	stp x0,lr,[sp,#-16]!;bl xprint;.asciz string;.align 2;ldp x0,lr,[sp],#16


/*
 * Kernel start routine for aarch64 boards.
 */
	.global start
start:
	/*
	 * store uboot arguments to uboot_args[4] in rpi_machdep.c
	 */
	ldr	x8, =(uboot_args-start)	/* offset of uboot_args from start */
	adr	x9, start
	add	x8, x8, x9		/* add load address to offset */
	str	x0, [x8, #(8*0)]
	str	x1, [x8, #(8*1)]
	str	x2, [x8, #(8*2)]
	str	x3, [x8, #(8*3)]

#ifdef VERBOSE_INIT_ARM
	PRINT("boot NetBSD/RaspberryPi\r\n")
#endif
	b	aarch64_start		/* aarch64_start() @ aarch64/locore.S */


/*
 * void uartputc(x0:char)
 * char uartgetc(void)
 *   uartputc/uartgetc is a SoC dependent character input/output routine
 *   referenced from locore.S and early console. (before consinit)
 *
 *   Caller (locore.S) expects all registers would be saved.
 *   Don't break any registers.
 *
 * If no uart, please define dummy routine like "uartputc: ret"
 */
ENTRY_NP(uartputc)
#if defined(COM_DATA) && defined(COM_LSR) && defined(COM_LSR_TXFE_BIT)
	stp	x9, x10, [sp, #-16]!
	stp	x0, x1, [sp, #-16]!

	ldr	x9, =COM_LSR
1:	ldr	w10, [x9]
	tbz	w10, #COM_LSR_TXFE_BIT, 1b

	ldr	x9, =COM_DATA
	and	w0, w0, #0xff
	str	w0, [x9]

	ldp	x0, x1, [sp], #16
	ldp	x9, x10, [sp], #16
#endif /* COM_DATA/COM_LSR/COM_LSR_TXFE_BIT */
	ret
END(uartputc)


ENTRY_NP(uartgetc)
#if defined(COM_DATA) && defined(COM_LSR) && defined(COM_LSR_RXRDY_BIT)
	stp	x9, x10, [sp, #-16]!

	ldr	x9, =COM_LSR
1:	ldr	w10, [x9]
	tbz	w10, #COM_LSR_RXRDY_BIT, 1b

	ldr	x9, =COM_DATA
	ldr	w0, [x9]
	and	w0, w0, #0xff

	ldp	x9, x10, [sp], #16
#else /* COM_DATA/COM_LSR/COM_LSR_RXRDY_BIT */
	mov x0, xzr
#endif /* COM_DATA/COM_LSR/COM_LSR_RXRDY_BIT */
	ret
END(uartgetc)
