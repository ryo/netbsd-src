/*	$NetBSD$	*/

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
#include <aarch64/pte.h>
#include "assym.h"

RCSID("$NetBSD$")

#define __LOWEST_SET_BIT(__mask) ((((__mask) - 1) & (__mask)) ^ (__mask))
#define __SHIFTOUT(__x, __mask) (((__x) & (__mask)) / __LOWEST_SET_BIT(__mask))
#define __SHIFTIN(__x, __mask) ((__x) * __LOWEST_SET_BIT(__mask))

//XXXAARCH64
#define AUX_MU_IO_REG	0x3f215040	/* Mini Uart I/O Data (8bit) */
#define AUX_MU_LSR_REG	0x3f215054	/* Mini Uart Line Status (8bit) */

#define COM_DATA		AUX_MU_IO_REG
#define COM_LSR			AUX_MU_LSR_REG
#define COM_LSR_TXFE_BIT	5	/* Transmit fifo empty */

#define PRINT(string)	stp x0,lr,[sp,#-16]!;bl xprint;.asciz string;.align 2;ldp x0,lr,[sp],#16


#define VERBOSE_LOCORE
//#define DEBUG_LOCORE

#ifdef VERBOSE_LOCORE
#define VERBOSE(string)	PRINT(string)
#else
#define VERBOSE(string)
#endif


#define LOCORE_EL2
#define VIRT_BIT	48
#define DEVICE_MEM	0
#define NORMAL_UNCACHED	1
#define NORMAL_MEM	2

/*
 * We assume:
 *  MMU      on with an identity map, or off
 *  D-Cache: off
 *  I-Cache: on or off
 *  We are loaded at a 2MiB aligned address
 */

	.text
	.global _start
_start:
	stp	x0, lr, [sp, #-16]!

#ifdef VERBOSE_LOCORE
	mov	x9, x0		// save x0

	mrs	x0, CurrentEL
	PRINT("CurrentEL=")
	bl	print_x0

	PRINT("SP=")
	mov	x0, sp
	bl	print_x0

	mrs	x0, SP_EL0
	PRINT("SP_EL0=")
	bl	print_x0

	PRINT("LR=")
	mov	x0, lr
	bl	print_x0

	PRINT("SP=")
	mov	x0, sp
	bl	print_x0

	PRINT("PC=")
	bl	1f
1:	mov	x0, lr
	bl	print_x0

	mov	x0, x9		// saved x0
	PRINT("X0=")
	bl	print_x0

	PRINT("X1=")
	mov	x0, x1
	bl	print_x0

	PRINT("X2=")
	mov	x0, x2
	bl	print_x0

	PRINT("X3=")
	mov	x0, x3
	bl	print_x0

	PRINT("X4=")
	mov	x0, x4
	bl	print_x0

	PRINT("X5=")
	mov	x0, x5
	bl	print_x0
#endif


#ifdef LOCORE_EL2
	VERBOSE("Drop to EL1\r\n")
#include "aarch64/aarch64/locore_el2.S"
	VERBOSE("Drop to EL1 DONE\r\n")

#ifdef VERBOSE_LOCORE
	mrs	x0, CurrentEL
	PRINT("CurrentEL=")
	bl	print_x0
#endif

#endif /* LOCORE_EL2 */


	/* disable MMU */
	dsb	sy
	mrs	x0, sctlr_el1
	bic	x0, x2, SCTLR_M		/* clear MMU enable bit */
	msr	sctlr_el1, x0
	isb

	/* Set the context id = 0*/
	msr	contextidr_el1, xzr


	VERBOSE("Create pagetables\r\n")

	/* Create the page tables */
	bl	create_pagetables

	VERBOSE("Create pagetables done\r\n")

	/*
	 * At this point:
	 * x27 = TTBR0 table
	 * x26 = Kernel L1 table
	 * x24 = TTBR1 table
	 */

	VERBOSE("MMU Enable...\r\n")
	bl	start_mmu

	/* change UART address from 0x3f215040 to 0xffff00003f215040 */
	bl	uart_change_to_directmap

	VERBOSE("MMU Enabled!\r\n")


	VERBOSE("Load the exception vectors\r\n")
	ldr	x2, =el1_vectors	# el1_vectors is in kva
	msr	vbar_el1, x2

#if 0
	/* address translation test */
	PRINT("L_COM_DATA(V): ")
	ldr	x0, L_COM_DATA
	bl	print_x0

	at	s1e1rql, x0
	mrs	x0, par_el1
	PRINT("L_COM_DATA(P): ")
	bl	print_x0
#endif

#ifdef DEBUG_LOCORE
	mrs	x0, spsr_el1
	PRINT("SPSR_EL1=")
	bl	print_x0
#endif

#ifdef DEBUG_LOCORE
	mrs	x0, daif
	PRINT("DAIF=")
	bl	print_x0
#endif

	ldr	x0, .Lvstart
	br	x0		# jump to the kernel virtual address

start_done:
	ldp	x0, lr, [sp], #16
	ret

#
# vstart is in kernel virtual address
#
vstart:
	adr	x0, kernelstack_bottom
	mov	sp, x0

#ifdef VERBOSE_LOCORE
	PRINT("SP=")
	mov	x0, sp
	bl	print_x0
#endif

	/* Zero the BSS */
	ldr	x15, .Lbss
	ldr	x14, .Lend
1:	str	xzr, [x15], #8
	cmp	x15, x14
	b.lo	1b

	VERBOSE("set_cpuinfo\r\n")
	bl	set_cpuinfo

	mov	fp, #0		/* trace back starts here */

	VERBOSE("call initarm()\r\n")
	bl	_C_LABEL(initarm)

	bl	_C_LABEL(main)

	/* We should not get here */
	brk	0

	.align 3
.Lvstart:
	.quad	vstart
.Lstart:
	.quad	_start
.Lbss:
	.quad	__bss_start
.Lend:
	.quad	_end

#ifdef MULTIPROCESSOR
#error MULTIPROCESSOR NOT SUPPORTED YET
#endif

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
	.global xprint
xprint:
	stp	x0, x1, [sp, #-16]!
	mov	x0, lr
	bl	uartputs

1:	ldrb	w1, [x0], #1
	cbnz	w1, 1b

	add	x0, x0, #3
	bic	lr, x0, #3

	ldp	x0, x1, [sp], #16
	ret
END(xprint)


	.global _print_x0
_print_x0:
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
END(_print_x0)

	.global _C_LABEL(print_x0)
_C_LABEL(print_x0):
	stp	x0, lr, [sp, #-16]!
	bl	_print_x0
	PRINT("\r\n")
	ldp	x0, lr, [sp], #16
	ret
END(_C_LABEL(print_x0))

print_x11:
	stp	x0, lr, [sp, #-16]!
	PRINT("x11=")
	mov	x0, x11
	bl	_print_x0
	ldp	x0, lr, [sp], #16
	ret

print_x13:
	stp	x0, lr, [sp, #-16]!
	PRINT("x13=")
	mov	x0, x13
	bl	_print_x0
	PRINT("\r\n")
	ldp	x0, lr, [sp], #16
	ret


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

	ldr	x9, L_COM_LSR
1:	ldr	w10, [x9]
	tbz	w10, #COM_LSR_TXFE_BIT, 1b

	ldr	x9, L_COM_DATA
	and	w0, w0, #0xff
	str	w0, [x9]

	ldp	x0, x1, [sp], #16
	ldp	x9, x10, [sp], #16
	ret
END(_C_LABEL(uartputc))

	.global _C_LABEL(uartputA)
_C_LABEL(uartputA):
	stp	x0, x9, [sp, #-16]!

	ldr	x9, L_COM_LSR
1:	ldr	w0, [x9]
	tbz	w0, #COM_LSR_TXFE_BIT, 1b

	ldr	x9, L_COM_DATA
	mov	w0, #'A'
	str	w0, [x9]

	ldp	x0, x9, [sp], #16
	ret
END(_C_LABEL(uartputA))

	.global _C_LABEL(uartputB)
_C_LABEL(uartputB):
	stp	x0, x9, [sp, #-16]!

	ldr	x9, L_COM_LSR
1:	ldr	w0, [x9]
	tbz	w0, #COM_LSR_TXFE_BIT, 1b

	ldr	x9, L_COM_DATA
	mov	w0, #'B'
	str	w0, [x9]

	ldp	x0, x9, [sp], #16
	ret
END(_C_LABEL(uartputB))

	.global _C_LABEL(uartputC)
_C_LABEL(uartputC):
	stp	x0, x9, [sp, #-16]!

	ldr	x9, L_COM_LSR
1:	ldr	w0, [x9]
	tbz	w0, #COM_LSR_TXFE_BIT, 1b

	ldr	x9, L_COM_DATA
	mov	w0, #'C'
	str	w0, [x9]

	ldp	x0, x9, [sp], #16
	ret
END(_C_LABEL(uartputC))

uart_change_to_directmap:
	stp	x0, x1, [sp, #-16]!

	adr	x1, L_COM_DATA
	ldr	x0, [x1]
	movk	x0, #0xffff, lsl #48
	str	x0, [x1]

	adr	x1, L_COM_LSR
	ldr	x0, [x1]
	movk	x0, #0xffff, lsl #48
	str	x0, [x1]

	ldp	x9, x10, [sp], #16
	ret

	.align	3
L_COM_DATA:
	.quad	COM_DATA
L_COM_LSR:
	.quad	COM_LSR


set_cpuinfo:
	mrs	x1, mpidr_el1
	and	x1, x1, #0xff	/* Aff0 = cpu id */
	cmp	x1, #MAXCPUS
	b.ge	arm_cpuinit_too_many_cpu

#ifdef MULTIPROCESSOR
	ldr	x0, .Lcpu_info
	ldr	x0, [x0, x1, lsl #3]	/* x0 = cpu_info[cpuid] */
#else
	ldr	x0, .Lcpu_info_store	/* x0 = &cpu_info_store */
#endif
	msr	tpidr_el1, x0		/* tpidr_el1 = my cpu info */
	ret

arm_cpuinit_too_many_cpu:
	PRINT("Too many CPU: MPIDR_EL1=")
	mrs	x0, mpidr_el1
	bl	print_x0
1:	hlt	#0xffff
	b	1b
	ret

#ifdef MULTIPROCESSOR
.Lcpu_info:
	.quad	_C_LABEL(cpu_info)
#endif
.Lcpu_info_store:
	.quad	_C_LABEL(cpu_info_store)

/*
 * This builds the page tables containing the identity map, and the kernel
 * virtual map.
 *
 * It relys on:
 *  We were loaded to an address that is on a 2MiB boundary
 *  All the memory must not cross a 1GiB boundaty
 *
 * TODO: This is out of date.
 *  There are at least 5 pages before that address for the page tables
 *   The pages used are:
 *    - The Kernel L2 table
 *    - The Kernel L1 table
 *    - The Kernel L0 table             (TTBR1)
 *    - The identity (PA = VA) L1 table
 *    - The identity (PA = VA) L0 table (TTBR0)
 *    - The KSEG L1 tables
 */
create_pagetables:
	/* Save the Link register */
	mov	x5, x30

	/* Clean the page table */
	adr	x6, pagetable
	mov	x26, x6
	adr	x27, pagetable_end
1:
	stp	xzr, xzr, [x6], #16
	stp	xzr, xzr, [x6], #16
	stp	xzr, xzr, [x6], #16
	stp	xzr, xzr, [x6], #16
	cmp	x6, x27
	b.lo	1b

	/*
	 * Build the TTBR1 maps.
	 */
	VERBOSE("Build the TTBR1 maps\r\n")

	/* kernelsize = _end - _start */
	ldr	x10, .Lend
	ldr	x0, .Lstart
	sub	x10, x10, x0

	/* round up kernelsize to L2_SIZE */
	add	x10, x10, #L2_SIZE
	sub	x10, x10, #1
	lsr	x10, x10, #L2_SHIFT

	VERBOSE("Create the kernel space L2 table\r\n")

	# Create the kernel space L2 table
	mov	x6, x26
	mov	x7, #NORMAL_MEM

	# VA start
	mov	x8, #(VM_MIN_KERNEL_ADDRESS & L2_BLOCK_MASK)

	# PA start
	adr	x9, _start
	and	x9, x9, #L2_BLOCK_MASK

	# kernel image load physical address is   0x???00000-
	# kernel virtual address space is 0xffffffc000000000-
	# kernel virtual address start is 0xffffffc0???00000-
	add	x8, x8, x9

	bl	build_l2_block_pagetable

	VERBOSE("Move to the l1 table\r\n")

	/* Move to the l1 table */
	add	x26, x26, #PAGE_SIZE

	/* Link the l1 -> l2 table */
	mov	x9, x6
	mov	x6, x26
	bl	link_l1_pagetable

	VERBOSE("Move to the l0 table\r\n")

	/* Move to the l0 table */
	add	x24, x26, #PAGE_SIZE

	/* Link the l0 -> l1 table */
	mov	x9, x6
	mov	x6, x24
	mov	x10, #1
	bl	link_l0_pagetable



	VERBOSE("Create KSEG maps\r\n")

	/* Link the KSEG tables */
	ldr	x8, =AARCH64_KSEG_START
	adr	x9, pagetable_kseg
#define KSEG_TABLES		1	// XXXAARCH64
	mov	x10, #KSEG_TABLES
	bl	link_l0_pagetable

	VERBOSE("Create KSEG L1 table\r\n")
	adr	x6, pagetable_kseg
	mov	x7, #DEVICE_MEM
	mov	x9, #0
	mov	x8, #0
	mov	x10, #32		// XXXAARCH64
	bl	build_l1_block_pagetable


	VERBOSE("Build the TTBR0 maps\r\n")

	/*
	 * Build the TTBR0 maps.
	 */
	add	x27, x24, #PAGE_SIZE

	mov	x6, x27		/* The initial page table */
#if defined(SOCDEV_PA) && defined(SOCDEV_VA)
	/* Create a table for the UART */
	mov	x7, #DEVICE_MEM
	mov	x8, #(SOCDEV_VA)	/* VA start */
	mov	x9, #(SOCDEV_PA)	/* PA start */
	mov	x10, #1
	bl	build_l1_block_pagetable
#endif

	VERBOSE("Create the VA = PA map\r\n")

	/* Create the VA = PA map */
#if 0
	mov	x7, #NORMAL_UNCACHED /* Uncached as it's only needed early on */
#else
	mov	x7, #DEVICE_MEM		// XXXAARCH64: need because include device address
#endif
	mov	x9, x27
	mov	x8, x9		/* VA start (== PA start) */
	mov	x10, #1
#if 1
	mov	x10, #4		// XXXAARCH64: 0x00000000-0xffffffff
#endif
	bl	build_l1_block_pagetable

	VERBOSE("Move to the l0 table\r\n")

	/* Move to the l0 table */
	add	x27, x27, #PAGE_SIZE

	/* Link the l0 -> l1 table */
	mov	x9, x6
	mov	x6, x27
	mov	x10, #1
	bl	link_l0_pagetable

	/* Restore the Link register */
	mov	x30, x5
	ret

/*
 * Builds an L0 -> L1 table descriptor
 *
 * This is a link for a 512GiB block of memory with up to 1GiB regions mapped
 * within it by build_l1_block_pagetable.
 *
 *  x6  = L0 table
 *  x8  = Virtual Address
 *  x9  = L1 PA (trashed)
 *  x10 = Entry count
 *  x11, x12 and x13 are trashed
 */
link_l0_pagetable:
#ifdef DEBUG_LOCORE
	stp	x0, lr, [sp, #-16]!

	PRINT("========link_l0_pagetable\r\n")

	mov	x0, x6
	PRINT("L0 table       (x6) =")
	bl	print_x0

	mov	x0, x8
	PRINT("Virtual Address(x8) =")
	bl	print_x0

	mov	x0, x9
	PRINT("L1 PA          (x9) =")
	bl	print_x0

	mov	x0, x10
	PRINT("Entry count    (x10)=")
	bl	print_x0
#endif /* DEBUG_LOCORE */

	/*
	 * Link an L0 -> L1 table entry.
	 */
	/* Find the table index */
	lsr	x11, x8, #L0_SHIFT
	and	x11, x11, #L0_ADDR_MASK

	/* Build the L0 block entry */
	mov	x12, #L0_TABLE

	/* Only use the output address bits */
	lsr	x9, x9, #PGSHIFT
1:	orr	x13, x12, x9, lsl #PGSHIFT

#ifdef DEBUG_LOCORE
	PRINT("L0 entry[")
	bl	print_x11
	PRINT("]=")
	bl	print_x13
#endif /* DEBUG_LOCORE */

	/* Store the entry */
	str	x13, [x6, x11, lsl #3]

	sub	x10, x10, #1
	add	x11, x11, #1
	add	x9, x9, #1
	cbnz	x10, 1b

#ifdef DEBUG_LOCORE
	ldp	x0, lr, [sp], #16
#endif /* DEBUG_LOCORE */
	ret

/*
 * Builds an L1 -> L2 table descriptor
 *
 * This is a link for a 1GiB block of memory with up to 2MiB regions mapped
 * within it by build_l2_block_pagetable.
 *
 *  x6  = L1 table
 *  x8  = Virtual Address
 *  x9  = L2 PA (trashed)
 *  x11, x12 and x13 are trashed
 */
link_l1_pagetable:
#ifdef DEBUG_LOCORE
	stp	x0, lr, [sp, #-16]!

	PRINT("========link_l1_pagetable\r\n")

	mov	x0, x6
	PRINT("L1 table       (x6) =")
	bl	print_x0

	mov	x0, x8
	PRINT("VA             (x8) =")
	bl	print_x0

	mov	x0, x9
	PRINT("PA             (x9) =")
	bl	print_x0
#endif /* DEBUG_LOCORE */

	/*
	 * Link an L1 -> L2 table entry.
	 */
	/* Find the table index */
	lsr	x11, x8, #L1_SHIFT
	and	x11, x11, #Ln_ADDR_MASK

	/* Build the L1 block entry */
	mov	x12, #L1_TABLE

	/* Only use the output address bits */
	lsr	x9, x9, #PGSHIFT
	orr	x13, x12, x9, lsl #PGSHIFT

#ifdef DEBUG_LOCORE
	PRINT("L1 entry[")
	bl	print_x11
	PRINT("]=")
	bl	print_x13
#endif /* DEBUG_LOCORE */

	/* Store the entry */
	str	x13, [x6, x11, lsl #3]

#ifdef DEBUG_LOCORE
	ldp	x0, lr, [sp], #16
#endif
	ret

/*
 * Builds count 1 GiB page table entry
 *  x6  = L1 table
 *  x7  = Type (0 = Device, 1 = Normal)
 *  x8  = VA start
 *  x9  = PA start (trashed)
 *  x10 = Entry count
 *  x11, x12 and x13 are trashed
 */
build_l1_block_pagetable:
#ifdef DEBUG_LOCORE
	stp	x0, lr, [sp, #-16]!

	PRINT("========build_l1_block_pagetable (1G)\r\n")

	mov	x0, x6
	PRINT("L1 table       (x6) =")
	bl	print_x0

	mov	x0, x7
	PRINT("Type           (x7) =")
	bl	print_x0

	mov	x0, x8
	PRINT("VA start       (x8) =")
	bl	print_x0

	mov	x0, x9
	PRINT("PA start       (x9) =")
	bl	print_x0

	mov	x0, x10
	PRINT("Entry count    (x10)=")
	bl	print_x0
#endif /* DEBUG_LOCORE */

	/*
	 * Build the L1 table entry.
	 */
	/* Find the table index */
	lsr	x11, x8, #L1_SHIFT
	and	x11, x11, #Ln_ADDR_MASK

	/* Build the L1 block entry */
	lsl	x12, x7, #2
	orr	x12, x12, #L1_BLOCK
	orr	x12, x12, #L1_BLK_AF
#ifdef MULTIPROCESSOR
	orr	x12, x12, ATTR_SH(ATTR_SH_IS)
#endif

	/* Only use the output address bits */
	lsr	x9, x9, #L1_SHIFT

	/* Set the physical address for this virtual address */
1:	orr	x13, x12, x9, lsl #L1_SHIFT

#ifdef DEBUG_LOCORE
	PRINT("L1 entry[")
	bl	print_x11
	PRINT("]=")
	bl	print_x13
#endif /* DEBUG_LOCORE */

	/* Store the entry */
	str	x13, [x6, x11, lsl #3]

	sub	x10, x10, #1
	add	x11, x11, #1
	add	x9, x9, #1
	cbnz	x10, 1b

#ifdef DEBUG_LOCORE
	ldp	x0, lr, [sp], #16
#endif /* DEBUG_LOCORE */
	ret

/*
 * Builds count 2 MiB page table entry
 *  x6  = L2 table
 *  x7  = Type (0 = Device, 1 = Normal)
 *  x8  = VA start
 *  x9  = PA start (trashed)
 *  x10 = Entry count
 *  x11, x12 and x13 are trashed
 */
build_l2_block_pagetable:
#ifdef DEBUG_LOCORE
	stp	x0, lr, [sp, #-16]!

	PRINT("========build_l2_block_pagetable (2M)\r\n")

	mov	x0, x6
	PRINT("L2 table   (x6) =")
	bl	print_x0

	mov	x0, x7
	PRINT("TYPE       (x7) =")
	bl	print_x0

	mov	x0, x8
	PRINT("VA start   (x8) =")
	bl	print_x0

	mov	x0, x9
	PRINT("PA start   (x9) =")
	bl	print_x0

	mov	x0, x10
	PRINT("Entry count(x10)=")
	bl	print_x0
#endif /* DEBUG_LOCORE */

	/*
	 * Build the L2 table entry.
	 */
	/* Find the table index */
	lsr	x11, x8, #L2_SHIFT
	and	x11, x11, #Ln_ADDR_MASK

	/* Build the L2 block entry */
	lsl	x12, x7, #2
	orr	x12, x12, #L2_BLOCK
	orr	x12, x12, #L2_BLKPAG_AF
#ifdef MULTIPROCESSOR
	orr	x12, x12, ATTR_SH(ATTR_SH_IS)
#endif

	/* Only use the output address bits */
	lsr	x9, x9, #L2_SHIFT

	/* Set the physical address for this virtual address */
1:	orr	x13, x12, x9, lsl #L2_SHIFT

#ifdef DEBUG_LOCORE
	PRINT("L2 entry[")
	bl	print_x11
	PRINT("]=")
	bl	print_x13
#endif /* DEBUG_LOCORE */

	/* Store the entry */
	str	x13, [x6, x11, lsl #3]

	sub	x10, x10, #1
	add	x11, x11, #1
	add	x9, x9, #1
	cbnz	x10, 1b

#ifdef DEBUG_LOCORE
	ldp	x0, lr, [sp], #16
#endif
	ret


/*
 * x27 = TTBR0 table
 * x24 = TTBR1 table
*/
start_mmu:
	dsb	sy

	/* Load ttbr0 and ttbr1 */
	msr	ttbr0_el1, x27
	msr	ttbr1_el1, x24
	isb

	/* Clear the Monitor Debug System control register */
	msr	mdscr_el1, xzr

	/* Invalidate the TLB */
	tlbi	vmalle1is

	ldr	x2, mair
	msr	mair_el1, x2

	/*
	 * Setup TCR according to PARange bits from ID_AA64MMFR0_EL1.
	 */
	ldr	x2, tcr
	mrs	x3, id_aa64mmfr0_el1
	bfi	x2, x3, #32, #3
	msr	tcr_el1, x2

	/* Setup SCTLR */
	ldr	x2, sctlr_set
	ldr	x3, sctlr_clear
	mrs	x1, sctlr_el1
	bic	x1, x1, x3	/* Clear the required bits */
	orr	x1, x1, x2	/* Set the required bits */
	msr	sctlr_el1, x1
	isb

	ret


	.align 3
mair:
#define MAIR_ATTR(attr, idx)	((attr) << ((idx) * 8))
	.quad	MAIR_ATTR(MAIR_DEVICE_nGnRnE, 0) |	\
		MAIR_ATTR(MAIR_NORMAL_NC, 1) |		\
		MAIR_ATTR(MAIR_NORMAL_WB, 2) |		\
		MAIR_ATTR(MAIR_NORMAL_WT, 3)

tcr:
#ifdef MULTIPROCESSOR
	.quad (						\
	    __SHIFTIN(64 - VIRT_BIT, TCR_T1SZ) |	\
	    __SHIFTIN(64 - VIRT_BIT, TCR_T0SZ) |	\
	    TCR_AS64K |					\
	    __SHIFTIN(TCR_TG_4KB, TCR_TG1) |		\
	    __SHIFTIN(TCR_XRGN_WB_WA, TCR_ORGN0) |	\
	    __SHIFTIN(TCR_XRGN_WB_WA, TCR_IRGN0) |	\
	    __SHIFTIN(TCR_XRGN_WB_WA, TCR_ORGN1) |	\
	    __SHIFTIN(TCR_XRGN_WB_WA, TCR_IRGN1) |	\
	    /* for MULTIPROCESSOR */			\
	    __SHIFTIN(TCR_SH_INNER, TCR_SH0) |		\
	    __SHIFTIN(TCR_SH_INNER, TCR_SH1))
#else
	.quad (						\
	    __SHIFTIN(64 - VIRT_BIT, TCR_T1SZ) |	\
	    __SHIFTIN(64 - VIRT_BIT, TCR_T0SZ) |	\
	    TCR_AS64K |					\
	    __SHIFTIN(TCR_TG_4KB, TCR_TG1) |		\
	    __SHIFTIN(TCR_XRGN_WB_WA, TCR_ORGN0) |	\
	    __SHIFTIN(TCR_XRGN_WB_WA, TCR_IRGN0) |	\
	    __SHIFTIN(TCR_XRGN_WB_WA, TCR_ORGN1) |	\
	    __SHIFTIN(TCR_XRGN_WB_WA, TCR_IRGN1))
#endif

sctlr_set:
	/* Bits to set */
	.quad (SCTLR_LSMAOE | SCTLR_nTLSMD | SCTLR_UCI | SCTLR_SPAN | \
	    SCTLR_nTWE | SCTLR_nTWI | SCTLR_UCT | SCTLR_DZE | \
	    SCTLR_I | SCTLR_SED | SCTLR_SA0 | SCTLR_SA | SCTLR_C | SCTLR_M)
sctlr_clear:
	/* Bits to clear */
	.quad (SCTLR_EE | SCTLR_EOE | SCTLR_IESB | SCTLR_WXN | SCTLR_UMA | \
	    SCTLR_ITD | SCTLR_THEE | SCTLR_CP15BEN | SCTLR_A)

	/*
	 * Kernel stack overflow, if ever, will destroy lower section
	 * beyond here until writing "_start" ends up with kernel
	 * segfault, hopefully.
	 */
	.align PGSHIFT
.globl _C_LABEL(lwp0uspace)
_C_LABEL(lwp0uspace):
	.space	UPAGES * PAGE_SIZE - TF_SIZE
kernelstack_bottom:

	/*
	 * 3 initial tables (in the following order):
	 *           L2 for kernel (High addresses)
	 *           L1 for kernel
	 *           L1 for user   (Low addresses)
	 */
pagetable:
	.space	PAGE_SIZE
pagetable_l1_ttbr1:
	.space	PAGE_SIZE
pagetable_l0_ttbr1:
	.space	PAGE_SIZE
pagetable_l1_ttbr0:
	.space	PAGE_SIZE
pagetable_l0_ttbr0:
	.space	PAGE_SIZE
pagetable_kseg:
	.space	PAGE_SIZE * KSEG_TABLES
pagetable_end:

	.globl init_pt_va
init_pt_va:
	.quad pagetable		/* XXX: Keep page tables VA */
