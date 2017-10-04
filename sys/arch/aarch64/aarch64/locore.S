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

#include "opt_cpuoptions.h"
#include "opt_cputypes.h"
#include "opt_multiprocessor.h"
#include "opt_ddb.h"

#include <aarch64/asm.h>
#include <aarch64/hypervisor.h>
#include <aarch64/pte.h>
#include "assym.h"

RCSID("$NetBSD$")


//#define DEBUG_LOCORE
//#define DEBUG_MMU


#define LOCORE_EL2

/* attributes are defined in MAIR_EL1 */
#define L2_BLKPAG_ATTR_NORMAL_WB	__SHIFTIN(0, LX_BLKPAG_ATTR_INDX)
#define L2_BLKPAG_ATTR_NORMAL_NC	__SHIFTIN(1, LX_BLKPAG_ATTR_INDX)
#define L2_BLKPAG_ATTR_NORMAL_WT	__SHIFTIN(2, LX_BLKPAG_ATTR_INDX)
#define L2_BLKPAG_ATTR_DEVICE_MEM	__SHIFTIN(3, LX_BLKPAG_ATTR_INDX)

#define PRINT(string)	stp x0,lr,[sp,#-16]!;bl xprint;.asciz string;.align 2;ldp x0,lr,[sp],#16

#ifdef VERBOSE_LOCORE
#define VERBOSE(string)	PRINT(string)
#else
#define VERBOSE(string)
#endif

	.global _ASM_LABEL(aarch64_start)
_ASM_LABEL(aarch64_start):

#ifdef DEBUG_LOCORE
	mrs	x0, CurrentEL
	lsr	x0, x0, #2
	PRINT("CurrentEL=")
	bl	print_x0

	cmp	x0, #2
	bne	1f

	/* EL2 registers can be accessed in EL2 or higher */
	mrs	x0, sctlr_el2
	PRINT("SCTLR_EL2=")
	bl	print_x0

	mrs	x0, hcr_el2
	PRINT("HCR_EL2=")
	bl	print_x0
1:

	mrs	x0, cntfrq_el0
	PRINT("CNTFREQ_EL0=")
	bl	print_x0

	mrs	x0, daif
	PRINT("DAIF=")
	bl	print_x0

	mov	x0, sp
	PRINT("SP=")
	bl	print_x0

	bl	1f
	PRINT("PC=")
1:	mov	x0, lr
	bl	print_x0

	mrs	x0, mpidr_el1
	PRINT("MPIDR_EL1=")
	bl	print_x0

	mrs	x0, s3_1_c11_c0_2
	PRINT("L2CTLR_EL1=")
	bl	print_x0

	PRINT("ID_AA64MPFR0_EL1=")
	mrs	x0, id_aa64pfr0_el1
	bl	print_x0

	PRINT("ID_AA64MPFR1_EL1=")
	mrs	x0, id_aa64pfr1_el1
	bl	print_x0

	PRINT("ID_AA64ISAR0_EL1=")
	mrs	x0, id_aa64isar0_el1
	bl	print_x0

	PRINT("ID_AA64ISAR1_EL1=")
	mrs	x0, id_aa64isar1_el1
	bl	print_x0


	PRINT("ID_AA64MMFR0_EL1=")
	mrs	x0, id_aa64mmfr0_el1
	bl	print_x0

	PRINT("ID_AA64MMFR1_EL1=")
	mrs	x0, id_aa64mmfr1_el1
	bl	print_x0
#endif


#ifdef LOCORE_EL2
	VERBOSE("Drop to EL1...")
# include <aarch64/aarch64/locore_el2.S>
	VERBOSE("OK\r\n")
#ifdef DEBUG_LOCORE
	mrs	x0, CurrentEL
	lsr	x0, x0, #2
	PRINT("CurrentEL=")
	bl	print_x0
#endif /* DEBUG_LOCORE */
#endif /* LOCORE_EL2 */

#ifdef DEBUG_LOCORE
	mrs	x0, daif
	PRINT("DAIF=")
	bl	print_x0
#endif

	bl	mmu_disable

	bl	init_sysregs

	bl	arm_boot_l0pt_init

	VERBOSE("MMU Enable...")
	bl	mmu_enable
	VERBOSE("OK\r\n")

	/* set exception vector */
	ldr	x2, =el1_vectors	# el1_vectors is in kva
	msr	vbar_el1, x2

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

	ldr	x0, .Lvstart	# virtual address of vstart
	br	x0		# jump to the kernel virtual address

/*
 * vstart is in kernel virtual address
 */
vstart:
	adr	x0, lwp0uspace
	add	x0, x0, #(UPAGES * PAGE_SIZE)
	sub	x0, x0, #TF_SIZE	/* lwp0space + USPACE - TF_SIZE */
	mov	sp, x0			/* define lwp0 ksp bottom */

#ifdef DEBUG_LOCORE
	PRINT("SP=")
	mov	x0, sp
	bl	print_x0
#endif

	/* Zero the BSS */
	ldr	x0, .Lbss
	ldr	x1, .Lend
1:	stp	xzr, xzr, [x0], #16
	cmp	x0, x1
	bcc	1b

	bl	set_cpuinfo

	mov	fp, #0			/* trace back starts here */
	bl	_C_LABEL(initarm)	/* Off we go */
	bl	_C_LABEL(main)		/* call main() */

	adr	x0, .Lmainreturned
	b	_C_LABEL(panic)
	/* NOTREACHED */

.Lmainreturned:
	.asciz	"main() returned"

	.align 3
.Lvstart:
	.quad	vstart
.Lkernel_text:
	.quad	__kernel_text
.Lbss:
	.quad	__bss_start
.Lend:
	.quad	_end


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

printn_x1:
	stp	x0, lr, [sp, #-16]!
	mov	x0, x1
	bl	_print_x0
	ldp	x0, lr, [sp], #16
	ret

print_x2:
	stp	x0, lr, [sp, #-16]!
	mov	x0, x2
	bl	_print_x0
	PRINT("\r\n")
	ldp	x0, lr, [sp], #16
	ret


set_cpuinfo:
	mrs	x1, mpidr_el1
	and	x1, x1, #0xff	/* Aff0 = cpu id */
	cmp	x1, #MAXCPUS
	bcs	arm_cpuinit_too_many_cpu

	ldr	x0, .Lcpu_info
	ldr	x0, [x0, x1, lsl #3]	/* x0 = cpu_info[cpuid] */
	msr	tpidr_el1, x0		/* tpidr_el1 = my cpu info */
	str	x1, [x0, #CI_CPUID]	/* ci->ci_cpuid = CPUID */

	ret

arm_cpuinit_too_many_cpu:
	PRINT("Too many CPUs: MPIDR_EL1=")
	mrs	x0, mpidr_el1
	bl	print_x0
1:	wfi
	b	1b
	ret

	.align 3
.Lcpu_info:
	/* struct cpu_info *cpu_info[] in cpu_machdep.c */
	.quad	_C_LABEL(cpu_info)


arm_boot_l0pt_init:
	stp	x0, lr, [sp, #-16]!

	VERBOSE("Creating VA=PA tables\r\n")
	/* VA=PA table for L0 */
	adr	x0, ttbr0_l0table
	mov	x1, #0
	adr	x2, ttbr0_l1table
	bl	l0_settable

	/* VA=PA blocks */
	adr	x0, ttbr0_l1table
	mov	x1, #0			/* PA */
	mov	x2, #0			/* VA */
	mov	x3, #L2_BLKPAG_ATTR_DEVICE_MEM
	mov	x4, #4			/* 4GB = whole 32bit */
	bl	l1_setblocks

	VERBOSE("Creating KSEG tables\r\n")
	/* KSEG table for L0 */
	adr	x0, ttbr1_l0table
	mov	x1, #AARCH64_KSEG_START
	adr	x2, ttbr1_l1table_kseg
	bl	l0_settable

	/* KSEG blocks */
	adr	x0, ttbr1_l1table_kseg
	mov	x1, #AARCH64_KSEG_START
	adr	x2, #0
	mov	x3, #L2_BLKPAG_ATTR_NORMAL_WB
	orr	x3, x3, #(LX_BLKPAG_PXN|LX_BLKPAG_UXN)
	mov	x4, #Ln_ENTRIES		/* whole l1 table */
	bl	l1_setblocks

	VERBOSE("Creating KVA=PA tables\r\n")
	/* KVA=PA table for L0 */
	adr	x0, ttbr1_l0table
	mov	x1, #VM_MIN_KERNEL_ADDRESS
	adr	x2, ttbr1_l1table_kva
	bl	l0_settable

	/* KVA=PA table for L1 */
	adr	x0, ttbr1_l1table_kva
	mov	x1, #VM_MIN_KERNEL_ADDRESS
	adr	x2, ttbr1_l2table_kva
	bl	l1_settable

	/* KVA=PA blocks */
	adr	x0, ttbr1_l2table_kva
	adr	x2, start		/* physical addr. before MMU */
	and	x2, x2, #L2_BLK_OA	/* L2 block size aligned (2MB) */
	mov	x1, #VM_MIN_KERNEL_ADDRESS
	add	x1, x1, x2		/* add kernel load address */
	mov	x3, #L2_BLKPAG_ATTR_NORMAL_WB

	/* kernelsize = _end - _start */
	ldr	x4, .Lend
	ldr	x5, .Lkernel_text
	sub	x4, x4, x5

	/* round up kernelsize to L2_SIZE (2MB) */
	add	x4, x4, #L2_SIZE
	sub	x4, x4, #1
	lsr	x4, x4, #L2_SHIFT
	bl	l2_setblocks

	VERBOSE("Creating devmap tables\r\n")
	/* devmap=PA table for L1 */
	adr	x0, ttbr1_l1table_kva
	ldr	x1, .L_devmap_addr
	adr	x2, ttbr1_l2table_devmap
	bl	l1_settable

	ldp	x0, lr, [sp], #16
	ret

	.align 3
.L_devmap_addr:
	.quad	VM_MAX_KERNEL_ADDRESS - L2_SIZE

/*
 *	x0 = l0table
 *	x1 = vaddr
 *	x2 = l1table
 */
l0_settable:
	stp	x0, lr, [sp, #-16]!

	and	x2, x2, #~PAGE_MASK
	mov	x8, #L0_TABLE
	orr	x2, x2, x8
	and	x1, x1, #L0_ADDR_BITS
	lsr	x1, x1, #L0_SHIFT
	str	x2, [x0, x1, lsl #3]	/* l0table[x1] = x2 */

#ifdef DEBUG_MMU
	PRINT("L0 entry[")
	bl printn_x1
	PRINT("]=")
	bl print_x2
#endif

	ldp	x0, lr, [sp], #16
	ret

/*
 *	x0 = l1table
 *	x1 = vaddr
 *	x2 = paddr
 *	x3 = attr
 *	x4 = N entries
 */
l1_setblocks:
	stp	x0, lr, [sp, #-16]!

	and	x2, x2, #L1_ADDR_BITS
	mov	x8, #L1_BLOCK
	orr	x2, x2, x8
	orr	x2, x2, x3
	mov	x8, #(LX_BLKPAG_AF|LX_BLKPAG_AP_RW)
	orr	x2, x2, x8
#ifdef MULTIPROCESSOR
	orr	x2, x2, #LX_BLKPAG_SH_IS
#endif
	and	x1, x1, #L1_ADDR_BITS
	lsr	x1, x1, #L1_SHIFT
1:
	str	x2, [x0, x1, lsl #3]	/* l1table[x1] = x2 */
#ifdef DEBUG_MMU
	PRINT("L1 entry[")
	bl printn_x1
	PRINT("]=")
	bl print_x2
#endif
	mov	x3, #L1_SIZE
	add	x2, x2, x3
	add	x1, x1, #1
	subs	x4, x4, #1
	bne	1b

	ldp	x0, lr, [sp], #16
	ret

/*
 *	x0 = l1table
 *	x1 = vaddr
 *	x2 = l2table
 */
l1_settable:
	stp	x0, lr, [sp, #-16]!

	and	x2, x2, #~PAGE_MASK
	mov	x8, #L1_TABLE
	orr	x2, x2, x8
	and	x1, x1, #L1_ADDR_BITS
	lsr	x1, x1, #L1_SHIFT
	str	x2, [x0, x1, lsl #3]	/* l1table[x1] = x2 */

#ifdef DEBUG_MMU
	PRINT("L1 entry[")
	bl printn_x1
	PRINT("]=")
	bl print_x2
#endif

	ldp	x0, lr, [sp], #16
	ret

/*
 *	x0 = l2table
 *	x1 = vaddr
 *	x2 = paddr
 *	x3 = attr
 *	x4 = N entries
 */
l2_setblocks:
	stp	x0, lr, [sp, #-16]!

	and	x2, x2, #L2_ADDR_BITS
	mov	x8, #L2_BLOCK
	orr	x2, x2, x8
	orr	x2, x2, x3
	mov	x8, #(LX_BLKPAG_AF|LX_BLKPAG_AP_RW)
	orr	x2, x2, x8
#ifdef MULTIPROCESSOR
	orr	x2, x2, #LX_BLKPAG_SH_IS
#endif
	and	x1, x1, #L2_ADDR_BITS
	lsr	x1, x1, #L2_SHIFT
1:
	str	x2, [x0, x1, lsl #3]	/* l2table[x1] = x2 */
#ifdef DEBUG_MMU
	PRINT("L2 entry[")
	bl printn_x1
	PRINT("]=")
	bl print_x2
#endif
	mov	x3, #L2_SIZE
	add	x2, x2, x3
	add	x1, x1, #1
	subs	x4, x4, #1
	bne	1b

	ldp	x0, lr, [sp], #16
	ret

init_sysregs:
	stp	x0, lr, [sp, #-16]!

	/* Disable debug event */
	msr	mdscr_el1, xzr

	/* Clear context id register */
	msr	contextidr_el1, xzr

	/* No trap system register access, and Trap FP/SIMD access */
	msr	cpacr_el1, xzr

	/* any exception not masked */
	msr	daif, xzr

	ldp	x0, lr, [sp], #16
	ret

ENTRY(clear_ttbr0_l1tables)
	stp	x29, lr, [sp, #-16]!
	mov	x29, sp

	/* clear ttbr0_l1table for PA=VA */
	adr	x0, ttbr0_l1table
	add	x1, x0, PAGE_SIZE
1:	stp	xzr, xzr, [x0], #16
	cmp	x0, x1
	bcc	1b

	bl	aarch64_tlb_flushID

	ldp	x29, lr, [sp], #16
	ret
END(clear_ttbr0_l1tables)


mmu_disable:
	dsb	sy
	mrs	x0, sctlr_el1
	bic	x0, x2, SCTLR_M		/* clear MMU enable bit */
	msr	sctlr_el1, x0
	isb
	ret

mmu_enable:
	dsb	sy

	adr	x0, ttbr0_l0table
	msr	ttbr0_el1, x0
	adr	x0, ttbr1_l0table
	msr	ttbr1_el1, x0
	isb

	/* Invalidate the TLB */
	tlbi	vmalle1is

	ldr	x0, mair_setting
	msr	mair_el1, x0


	/* TCR_EL1:IPS[34:32] = AA64MMFR0:PARange[3:0] */
	ldr	x0, tcr_setting
	mrs	x1, id_aa64mmfr0_el1
	bfi	x0, x1, #32, #3
	msr	tcr_el1, x0

	/*
	 * configure SCTLR
	 */
	mrs	x0, sctlr_el1
	ldr	x1, sctlr_clear
	bic	x0, x0, x1
	ldr	x1, sctlr_set
	orr	x0, x0, x1

	ldr	x1, sctlr_ee
#ifdef __AARCH64EB__
	orr	x0, x0, x1	/* set: BigEndian */
#else
	bic	x0, x0, x1	/* clear: LittleEndian */
#endif
#ifdef MULTIPROCESSOR
	ldr	x1, tcr_setting_inner_shareable
	orr	x0, x0, x1
#endif
	msr	sctlr_el1, x0	/* enabling MMU! */
	isb

	ret

	.align 3
mair_setting:
	.quad (						\
	    __SHIFTIN(MAIR_NORMAL_WB, MAIR_ATTR0) |	\
	    __SHIFTIN(MAIR_NORMAL_NC, MAIR_ATTR1) |	\
	    __SHIFTIN(MAIR_NORMAL_WT, MAIR_ATTR2) |	\
	    __SHIFTIN(MAIR_DEVICE_nGnRnE, MAIR_ATTR3))

#define VIRT_BIT	48
tcr_setting:
	.quad (						\
	    __SHIFTIN(64 - VIRT_BIT, TCR_T1SZ) |	\
	    __SHIFTIN(64 - VIRT_BIT, TCR_T0SZ) |	\
	    TCR_AS64K |					\
	    TCR_TG1_4KB | TCR_TG0_4KB |			\
	    TCR_ORGN0_WB_WA |				\
	    TCR_IRGN0_WB_WA |				\
	    TCR_ORGN1_WB_WA |				\
	    TCR_IRGN1_WB_WA)
#ifdef MULTIPROCESSOR
tcr_setting_inner_shareable:
	.quad (TCR_SH0_INNER | TCR_SH1_INNER)
#endif


#ifdef AARCH64_ALIGNMENT_CHECK
#define SCTLR_A_CONFIG		SCTLR_A
#else
#define SCTLR_A_CONFIG		0
#endif

#ifdef AARCH64_EL0_STACK_ALIGNMENT_CHECK
#define SCTLR_SA0_CONFIG	SCTLR_SA0
#else
#define SCTLR_SA0_CONFIG	0
#endif

#ifdef AARCH64_EL1_STACK_ALIGNMENT_CHECK
#define SCTLR_SA_CONFIG		SCTLR_SA
#else
#define SCTLR_SA_CONFIG		0
#endif


sctlr_ee:
	.quad (SCTLR_EE | SCTLR_EOE)	/* Endiannes of Exception and EL0 */
sctlr_set:
	.quad ( \
	    SCTLR_LSMAOE |  /* Load/Store Multiple Atomicity and Ordering */ \
	    SCTLR_nTLSMD |  /* no Trap Load/Store Multiple to Device */ \
	    SCTLR_SPAN |    /* This field resets to 1 */ \
	    SCTLR_UCT |     /* Enables EL0 access to the CTR_EL0 */ \
	    SCTLR_DZE |     /* Enables access to the DC ZVA instruction */ \
	    SCTLR_I |       /* Instruction cache enable */ \
	    SCTLR_SED |     /* SETEND instruction disable */ \
	    SCTLR_SA0 |     /* Enable EL0 stack alignment check */ \
	    SCTLR_SA |      /* Enable SP alignment check */ \
	    SCTLR_C |       /* Cache enable */ \
	    SCTLR_M |       /* MMU Enable */ \
	    SCTLR_SA0_CONFIG | \
	    SCTLR_SA_CONFIG | \
	    SCTLR_A_CONFIG | \
	    0)
sctlr_clear:
	.quad ( \
	    SCTLR_UCI |     /* Enable EL0  DC {CVAU,CIVAC,CVAC}, IC IVAU */ \
	    SCTLR_IESB |    /* Enable Implicit ErrorSynchronizationBarrier */ \
	    SCTLR_WXN |     /* Write permission implies Execute Never (W^X) */ \
	    SCTLR_nTWE |    /* EL0 WFE non-trapping */ \
	    SCTLR_nTWI |    /* EL0 WFI non-trapping */ \
	    SCTLR_UMA |     /* EL0 Controls access to interrupt masks */ \
	    SCTLR_ITD |     /* IT instruction disable */ \
	    SCTLR_THEE |    /* T32EE is not implemented */ \
	    SCTLR_CP15BEN | /* CP15 barrier enable */ \
	    SCTLR_A |       /* Alignment check enable */ \
	    SCTLR_SA0 | \
	    SCTLR_SA | \
	    SCTLR_A | \
	    0)


	/*
	 * Kernel stack overflow, if ever, will destroy lower section
	 * beyond here until writing "start" ends up with kernel
	 * segfault, hopefully.
	 *
	 * XXXAARCH64: kernel segment is writable
	 */
	.align PGSHIFT
	.global _C_LABEL(lwp0uspace)
_C_LABEL(lwp0uspace):
	.space	UPAGES * PAGE_SIZE

	.align PGSHIFT
/*
 * PA == VA mapping using L1 1G block (whole 32bit)
 */
ttbr0_l0table:
	.space	PAGE_SIZE
ttbr0_l1table:
	.space	PAGE_SIZE

/*
 * KVA    => PA mapping using L2 2MB block (kernelsize, max 2MB*512=2Gbyte)
 * DEVMAP => PA mapping using L2 2MB block (devmap size, max 2MB*512=2Gbyte)
 * KSEG   => PA mapping using L1 1GB block * 512
 */
ttbr1_l0table:
	.space	PAGE_SIZE
ttbr1_l1table_kseg:
	.space	PAGE_SIZE
ttbr1_l1table_kva:
	.space	PAGE_SIZE
ttbr1_l2table_kva:
	.space	PAGE_SIZE
ttbr1_l2table_devmap:
	.space	PAGE_SIZE
