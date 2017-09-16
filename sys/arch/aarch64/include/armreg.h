/* $NetBSD: armreg.h,v 1.2 2015/04/27 06:54:12 skrll Exp $ */

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

#ifndef _AARCH64_ARMREG_H_
#define _AARCH64_ARMREG_H_

#include <arm/cputypes.h>

#ifdef __aarch64__

#include <sys/types.h>

#define AARCH64REG_READ_INLINE2(regname, regdesc)		\
static uint64_t inline						\
reg_##regname##_read(void)					\
{								\
	uint64_t __rv;						\
	__asm __volatile("mrs %0, " #regdesc : "=r"(__rv));	\
	return __rv;						\
}

#define AARCH64REG_WRITE_INLINE2(regname, regdesc)		\
static void inline						\
reg_##regname##_write(uint64_t __val)				\
{								\
	__asm __volatile("msr " #regdesc ", %0" :: "r"(__val));	\
}

#define AARCH64REG_WRITEIMM_INLINE2(regname, regdesc)		\
static void inline						\
reg_##regname##_write(uint64_t __val)				\
{								\
	__asm __volatile("msr " #regdesc ", %0" :: "n"(__val));	\
}

#define AARCH64REG_READ_INLINE(regname)				\
	AARCH64REG_READ_INLINE2(regname, regname)

#define AARCH64REG_WRITE_INLINE(regname)			\
	AARCH64REG_WRITE_INLINE2(regname, regname)

#define AARCH64REG_WRITEIMM_INLINE(regname)			\
	AARCH64REG_WRITEIMM_INLINE2(regname, regname)
/*
 * System registers available at EL0 (user)
 */
AARCH64REG_READ_INLINE(ctr_el0)		/* Cache Type Register */

static const uintmax_t
   CTR_EL0_CWG_LINE	= __BITS(27,24), /* Cacheback Writeback Granule */
   CTR_EL0_ERG_LINE	= __BITS(23,20), /* Exclusives Reservation Granule */
   CTR_EL0_DMIN_LINE	= __BITS(19,16), /* Dcache MIN LINE size (log2 - 2) */
   CTR_EL0_L1IP_MASK	= __BITS(15,14),
   CTR_EL0_L1IP_AIVIVT	= 1,	/* ASID-tagged Virtual Index, Virtual Tag */
   CTR_EL0_L1IP_VIPT	= 2,		/* Virtual Index, Physical Tag */
   CTR_EL0_L1IP_PIPT	= 3,		/* Physical Index, Physical Tag */
   CTR_EL0_IMIN_LINE	= __BITS(3,0);	/* Icache MIN LINE size (log2 - 2) */

AARCH64REG_READ_INLINE(dczid_el0)	/* Data Cache Zero ID Register */

static const uintmax_t
    DCZID_DZP = __BIT(4),		/* Data Zero Prohibited */
    DCZID_BS  = __BITS(3,0);		/* Block Size (log2 - 2) */

AARCH64REG_READ_INLINE(tpidrro_el0)	/* Thread Pointer ID Register (RO) */

AARCH64REG_READ_INLINE(fpcr)		/* Floating Point Control Register */
AARCH64REG_WRITE_INLINE(fpcr)

static const uintmax_t
    FPCR_AHP    = __BIT(26),		/* Alternative Half Precision */
    FPCR_DN     = __BIT(25),		/* Default Nan Control */
    FPCR_FZ     = __BIT(24),		/* Flush-To-Zero */
    FPCR_RMODE  = __BITS(23,22),	/* Rounding Mode */
     FPCR_RN     = 0,			/*  Round Nearest */
     FPCR_RP     = 1,			/*  Round towards Plus infinity */
     FPCR_RM     = 2,			/*  Round towards Minus infinity */
     FPCR_RZ     = 3,			/*  Round towards Zero */
    FPCR_STRIDE = __BITS(21,20),
    FPCR_LEN    = __BITS(18,16),
    FPCR_IDE    = __BIT(15),		/* Input Denormal Exception enable */
    FPCR_IXE    = __BIT(12),		/* IneXact Exception enable */
    FPCR_UFE    = __BIT(11),		/* UnderFlow Exception enable */
    FPCR_OFE    = __BIT(10),		/* OverFlow Exception enable */
    FPCR_DZE    = __BIT(9),		/* Divide by Zero Exception enable */
    FPCR_IOE    = __BIT(8),		/* Invalid Operation Exception enable */
    FPCR_ESUM   = 0x1F00;

AARCH64REG_READ_INLINE(fpsr)		/* Floating Point Status Register */
AARCH64REG_WRITE_INLINE(fpsr)

static const uintmax_t
    FPSR_N32  = __BIT(31),		/* AARCH32 Negative */
    FPSR_Z32  = __BIT(30),		/* AARCH32 Zero */
    FPSR_C32  = __BIT(29),		/* AARCH32 Carry */
    FPSR_V32  = __BIT(28),		/* AARCH32 Overflow */
    FPSR_QC   = __BIT(27),		/* SIMD Saturation */
    FPSR_IDC  = __BIT(7),		/* Input Denormal Cumulative status */
    FPSR_IXC  = __BIT(4),		/* IneXact Cumulative status */
    FPSR_UFC  = __BIT(3),		/* UnderFlow Cumulative status */
    FPSR_OFC  = __BIT(2),		/* OverFlow Cumulative status */
    FPSR_DZC  = __BIT(1),		/* Divide by Zero Cumulative status */
    FPSR_IOC  = __BIT(0),		/* Invalid Operation Cumulative status*/
    FPSR_CSUM = 0x1F;

AARCH64REG_READ_INLINE(nzcv)		/* condition codes */
AARCH64REG_WRITE_INLINE(nzcv)

static const uintmax_t
    NZCV_N = __BIT(31),			/* Negative */
    NZCV_Z = __BIT(30),			/* Zero */
    NZCV_C = __BIT(29),			/* Carry */
    NZCV_V = __BIT(28);			/* Overflow */

AARCH64REG_READ_INLINE(tpidr_el0)	/* Thread Pointer ID Register (RW) */
AARCH64REG_WRITE_INLINE(tpidr_el0)

/*
 * From here on, these can only be accessed at EL1 (kernel)
 */

/*
 * These are readonly registers
 */
AARCH64REG_READ_INLINE2(cbar_el1, s3_1_c15_c3_0)	/* Cortex-A57 */

static const uintmax_t CBAR_PA = __BITS(47,18);

AARCH64REG_READ_INLINE(clidr_el1)

static const uintmax_t
    CLIDR_LOUU   = __BITS(29,27),	/* Level of Unification Uniprocessor */
    CLIDR_LOC    = __BITS(26,24),	/* Level of Coherency */
    CLIDR_LOUIS  = __BITS(23,21),	/* Level of Unification InnerShareable*/
    CLIDR_CTYPE7 = __BITS(20,18),	/* Cache Type field for level7 */
    CLIDR_CTYPE6 = __BITS(17,15),	/* Cache Type field for level6 */
    CLIDR_CTYPE5 = __BITS(14,12),	/* Cache Type field for level5 */
    CLIDR_CTYPE4 = __BITS(11,9),	/* Cache Type field for level4 */
    CLIDR_CTYPE3 = __BITS(8,6),		/* Cache Type field for level3 */
    CLIDR_CTYPE2 = __BITS(5,3),		/* Cache Type field for level2 */
    CLIDR_CTYPE1 = __BITS(2,0),		/* Cache Type field for level1 */
     CLIDR_TYPE_NOCACHE		= 0,	/* No cache */
     CLIDR_TYPE_ICACHE		= 1,	/* Instruction cache only */
     CLIDR_TYPE_DCACHE		= 2,	/* Data cache only */
     CLIDR_TYPE_IDCACHE		= 3,	/* Separate inst and data caches */
     CLIDR_TYPE_UNIFIEDCACHE	= 4;	/* Unified cache */

AARCH64REG_READ_INLINE(ccsidr_el1)

static const uintmax_t
    CCSIDR_WT		= __BIT(31),	/* Write-through supported */
    CCSIDR_WB		= __BIT(30),	/* Write-back supported */
    CCSIDR_RA		= __BIT(29),	/* Read-allocation supported */
    CCSIDR_WA		= __BIT(28),	/* Write-allocation supported */
    CCSIDR_NUMSET	= __BITS(27,13),/* (Number of sets in cache) - 1 */
    CCSIDR_ASSOC	= __BITS(12,3),	/* (Associativity of cache) - 1 */
    CCSIDR_LINESIZE	= __BITS(2,0);	/* Number of bytes in cache line */

AARCH64REG_READ_INLINE(currentel)
AARCH64REG_READ_INLINE(id_afr0_el1)
AARCH64REG_READ_INLINE(id_adr0_el1)
AARCH64REG_READ_INLINE(id_isar0_el1)
AARCH64REG_READ_INLINE(id_isar1_el1)
AARCH64REG_READ_INLINE(id_isar2_el1)
AARCH64REG_READ_INLINE(id_isar3_el1)
AARCH64REG_READ_INLINE(id_isar4_el1)
AARCH64REG_READ_INLINE(id_isar5_el1)
AARCH64REG_READ_INLINE(id_mmfr0_el1)
AARCH64REG_READ_INLINE(id_mmfr1_el1)
AARCH64REG_READ_INLINE(id_mmfr2_el1)
AARCH64REG_READ_INLINE(id_mmfr3_el1)
AARCH64REG_READ_INLINE(id_prf0_el1)
AARCH64REG_READ_INLINE(id_prf1_el1)
AARCH64REG_READ_INLINE(isr_el1)
AARCH64REG_READ_INLINE(midr_el1)
AARCH64REG_READ_INLINE(mpidr_el1)

static const uintmax_t
    MPIDR_AFF3		= __BITS(32,39),
    MPIDR_U		= __BIT(30),		/* 1 = Uni-Processor System */
    MPIDR_MT		= __BIT(24),		/* 1 = SMT(AFF0 is logical) */
    MPIDR_AFF2		= __BITS(16,23),
    MPIDR_AFF1		= __BITS(8,15),
    MPIDR_AFF0		= __BITS(0,7);

AARCH64REG_READ_INLINE(mvfr0_el1)
AARCH64REG_READ_INLINE(mvfr1_el1)
AARCH64REG_READ_INLINE(mvfr2_el1)
AARCH64REG_READ_INLINE(revidr_el1)

/*
 * These are read/write registers
 */
AARCH64REG_READ_INLINE2(l2ctlr_el1, s3_1_c11_c0_2)  /* Cortex-A53,57,72,73 */
AARCH64REG_WRITE_INLINE2(l2ctlr_el1, s3_1_c11_c0_2) /* Cortex-A53,57,72,73 */

static const uintmax_t
    L2CTLR_NUMOFCORE		= __BITS(25,24),/* Number of cores */
    L2CTLR_CPUCACHEPROT		= __BIT(22),	/* CPU Cache Protection */
    L2CTLR_SCUL2CACHEPROT	= __BIT(21),	/* SCU-L2 Cache Protection */
    L2CTLR_L2_INPUT_LATENCY	= __BIT(5),	/* L2 Data RAM input latency */
    L2CTLR_L2_OUTPUT_LATENCY	= __BIT(0);	/* L2 Data RAM output latency */

AARCH64REG_READ_INLINE(csselr_el1)	/* Cache Size Selection Register */
AARCH64REG_WRITE_INLINE(csselr_el1)

static const uintmax_t
    CSSELR_LEVEL	= __BITS(3,1),	/* Cache level of required cache */
    CSSELR_IND		= __BIT(0);	/* Instruction not Data bit */

AARCH64REG_READ_INLINE(cpacr_el1)	/* Coprocessor Access Control Regiser */
AARCH64REG_WRITE_INLINE(cpacr_el1)

static const uintmax_t
    CPACR_TTA		= __BIT(28),	/* System Register Access Traps */
    CPACR_FPEN		= __BITS(21,20),
    CPACR_FPEN_NONE	= __SHIFTIN(0, CPACR_FPEN),
    CPACR_FPEN_EL1	= __SHIFTIN(1, CPACR_FPEN),
    CPACR_FPEN_NONE_2	= __SHIFTIN(2, CPACR_FPEN),
    CPACR_FPEN_ALL	= __SHIFTIN(3, CPACR_FPEN);

AARCH64REG_READ_INLINE(elr_el1)		/* Exception Link Register */
AARCH64REG_WRITE_INLINE(elr_el1)

AARCH64REG_READ_INLINE(esr_el1)		/* Exception Symdrone Register */
AARCH64REG_WRITE_INLINE(esr_el1)

static const uintmax_t
    ESR_EC =		__BITS(31,26),	/* Exception Cause */
     ESR_EC_UNKNOWN		= 0x00,	/* AXX:Unknown Reason */
     ESR_EC_WFX			= 0x01,	/* AXX:WFI or WFE Insn Execution */
     ESR_EC_CP15_RT		= 0x03,	/* A32:MCR/MRC access to CP15 !EC=0 */
     ESR_EC_CP15_RRT		= 0x04,	/* A32:MCRR/MRRC access to CP15 !EC=0 */
     ESR_EC_CP14_RT		= 0x05,	/* A32:MCR/MRC access to CP14 */
     ESR_EC_CP14_DT		= 0x06,	/* A32:LDC/STC access to CP14 */
     ESR_EC_FP_ACCESS		= 0x07,	/* AXX:Access to SIMD/FP Registers */
     ESR_EC_FPID		= 0x08,	/* A32:MCR/MRC access to CP10 !EC=7 */
     ESR_EC_CP14_RRT		= 0x0c,	/* A32:MRRC access to CP14 */
     ESR_EC_ILL_STATE		= 0x0e,	/* AXX:Illegal Execution State */
     ESR_EC_SVC_A32		= 0x11,	/* A32:SVC Instruction Execution */
     ESR_EC_HVC_A32		= 0x12,	/* A32:HVC Instruction Execution */
     ESR_EC_SMC_A32		= 0x13,	/* A32:SMC Instruction Execution */
     ESR_EC_SVC_A64		= 0x15,	/* A64:SVC Instruction Execution */
     ESR_EC_HVC_A64		= 0x16,	/* A64:HVC Instruction Execution */
     ESR_EC_SMC_A64		= 0x17,	/* A64:SMC Instruction Execution */
     ESR_EC_SYS_REG		= 0x18,	/* A64:MSR/MRS/SYS insn (!EC0/1/7) */
     ESR_EC_INSN_ABT_EL0	= 0x20,	/* AXX:Instruction Abort (EL0) */
     ESR_EC_INSN_ABT_EL1	= 0x21,	/* AXX:Instruction Abort (EL1) */
     ESR_EC_PC_ALIGNMENT	= 0x22,	/* AXX:Misaligned PC */
     ESR_EC_DATA_ABT_EL0	= 0x24,	/* AXX:Data Abort (EL0) */
     ESR_EC_DATA_ABT_EL1	= 0x25,	/* AXX:Data Abort (EL1) */
     ESR_EC_SP_ALIGNMENT	= 0x26,	/* AXX:Misaligned SP */
     ESR_EC_FP_TRAP_A32		= 0x28,	/* A32:FP Exception */
     ESR_EC_FP_TRAP_A64		= 0x2c,	/* A64:FP Exception */
     ESR_EC_SERROR		= 0x2f,	/* AXX:SError Interrupt */
     ESR_EC_BRKPNT_EL0		= 0x30,	/* AXX:Breakpoint Exception (EL0) */
     ESR_EC_BRKPNT_EL1		= 0x31,	/* AXX:Breakpoint Exception (EL1) */
     ESR_EC_SW_STEP_EL0		= 0x32,	/* AXX:Software Step (EL0) */
     ESR_EC_SW_STEP_EL1		= 0x33,	/* AXX:Software Step (EL1) */
     ESR_EC_WTCHPNT_EL0		= 0x34,	/* AXX:Watchpoint (EL0) */
     ESR_EC_WTCHPNT_EL1		= 0x35,	/* AXX:Watchpoint (EL1) */
     ESR_EC_BKPT_INSN_A32	= 0x38,	/* A32:BKPT Instruction Execution */
     ESR_EC_VECTOR_CATCH	= 0x3a,	/* A32:Vector Catch Exception */
     ESR_EC_BKPT_INSN_A64	= 0x3c,	/* A64:BKPT Instruction Execution */
    ESR_IL =		__BIT(25),	/* Instruction Length (1=32-bit) */
    ESR_ISS =		__BITS(24,0);	/* Instruction Specific Syndrome */


AARCH64REG_READ_INLINE(far_el1)		/* Fault Address Register */
AARCH64REG_WRITE_INLINE(far_el1)

AARCH64REG_READ_INLINE(mair_el1) /* Memory Attribute Indirection Register */
AARCH64REG_WRITE_INLINE(mair_el1)

static const uintmax_t
    MAIR_ATTR0		= __BITS(7,0),
    MAIR_ATTR1		= __BITS(15,8),
    MAIR_ATTR2		= __BITS(23,16),
    MAIR_ATTR3		= __BITS(31,24),
    MAIR_ATTR4		= __BITS(39,32),
    MAIR_ATTR5		= __BITS(47,40),
    MAIR_ATTR6		= __BITS(55,48),
    MAIR_ATTR7		= __BITS(63,56),
    MAIR_DEVICE_nGnRnE	= 0x00,	/* NoGathering,NoReordering,NoEarlyWriteAck. */
    MAIR_NORMAL_NC	= 0x44,
    MAIR_NORMAL_WT	= 0xbb,
    MAIR_NORMAL_WB	= 0xff;


AARCH64REG_READ_INLINE(par_el1)		/* Physical Address Register */
AARCH64REG_WRITE_INLINE(par_el1)

static const uintmax_t
    PAR_ATTR		= __BITS(63,56),/* F=0 memory attributes */
    PAR_PA		= __BITS(47,12),/* F=0 physical address */
    PAR_NS		= __BIT(9),	/* F=0 non-secure */
    PAR_S		= __BIT(9),	/* F=1 failure stage */
    PAR_SHA		= __BITS(8,7),	/* F=0 shareability attribute */
     PAR_SHA_NONE	= 0,
     PAR_SHA_OUTER	= 2,
     PAR_SHA_INNER	= 3,
    PAR_PTW		= __BIT(8),	/* F=1 partial table walk */
    PAR_FST		= __BITS(6,1),	/* F=1 fault status code */
    PAR_F		= __BIT(0);	/* translation failed */

AARCH64REG_READ_INLINE(rmr_el1)		/* Reset Management Register */
AARCH64REG_WRITE_INLINE(rmr_el1)

AARCH64REG_READ_INLINE(rvbar_el1)	/* Reset Vector Base Address Register */
AARCH64REG_WRITE_INLINE(rvbar_el1)

AARCH64REG_READ_INLINE(sctlr_el1)	/* System Control Register */
AARCH64REG_WRITE_INLINE(sctlr_el1)

static const uintmax_t
    SCTLR_RES0		= 0xc8222400,	/* Reserved ARMv8.0, write 0 */
    SCTLR_RES1		= 0x30d00800,	/* Reserved ARMv8.0, write 1 */
    SCTLR_M		= __BIT(0),
    SCTLR_A		= __BIT(1),
    SCTLR_C		= __BIT(2),
    SCTLR_SA		= __BIT(3),
    SCTLR_SA0		= __BIT(4),
    SCTLR_CP15BEN	= __BIT(5),
    SCTLR_THEE		= __BIT(6),
    SCTLR_ITD		= __BIT(7),
    SCTLR_SED		= __BIT(8),
    SCTLR_UMA		= __BIT(9),
    SCTLR_I		= __BIT(12),
    SCTLR_DZE		= __BIT(14),
    SCTLR_UCT		= __BIT(15),
    SCTLR_nTWI		= __BIT(16),
    SCTLR_nTWE		= __BIT(18),
    SCTLR_WXN		= __BIT(19),
    SCTLR_IESB		= __BIT(21),
    SCTLR_SPAN		= __BIT(23),
    SCTLR_EOE		= __BIT(24),
    SCTLR_EE		= __BIT(25),
    SCTLR_UCI		= __BIT(26),
    SCTLR_nTLSMD	= __BIT(28),
    SCTLR_LSMAOE	= __BIT(29);


AARCH64REG_READ_INLINE(spsel)		/* Stack Pointer Select */
AARCH64REG_WRITE_INLINE(spsel)

static const uintmax_t
    SPSEL_SP		= __BIT(0);	/* use SP_EL0 at all exception levels */

AARCH64REG_READ_INLINE(sp_el0)		/* Stack Pointer */
AARCH64REG_WRITE_INLINE(sp_el0)

#if 1 /* DAIF_DEBUG */
AARCH64REG_READ_INLINE(daif)		/* Debug Async Irq Fiq mask register */
AARCH64REG_WRITE_INLINE(daif)
AARCH64REG_WRITEIMM_INLINE(daifclr)
AARCH64REG_WRITEIMM_INLINE(daifset)
#else
/* DAIF read/write trace */
static uint64_t inline
_reg_daif_read(void)
{
	uint64_t __rv;
	__asm __volatile ("mrs %0, daif" : "=r"(__rv));
	return __rv;
}
static uint64_t inline
reg_daif_read(void)
{
	uint64_t __rv;
	__asm __volatile ("mrs %0, daif" : "=r"(__rv));
	printf("reg_daif_read = %016llx\n", __rv);
	return __rv;
}
static void inline
reg_daif_write(uint64_t val)
{
	__asm __volatile ("msr daif, %0" :: "r"(val));
	printf("reg_daif_write(%016llx) -> %016llx\n", val, _reg_daif_read());
}
static void inline
reg_daifclr_write(uint64_t val)
{
	__asm __volatile ("msr daifclr, %0" :: "n"(val));
	printf("reg_daifclr_write(%016llx) -> %016llx\n", val, _reg_daif_read());
}
static void inline
reg_daifset_write(uint64_t val)
{
	__asm __volatile ("msr daifset, %0" :: "n"(val));
	printf("reg_daifset_write(%016llx) -> %016llx\n", val, _reg_daif_read());
}
#endif /* DAIF_DEBUG */

static const uintmax_t
    DAIF_D		= __BIT(9),	/* Debug Exception Mask */
    DAIF_A		= __BIT(8),	/* SError Abort Mask */
    DAIF_I		= __BIT(7),	/* IRQ Mask */
    DAIF_F		= __BIT(6),	/* FIQ Mask */
    DAIF_IMM_SHIFT	= 6;		/* for daifset/daifclr #imm shift */

AARCH64REG_READ_INLINE(spsr_el1)	/* Saved Program Status Register */
AARCH64REG_WRITE_INLINE(spsr_el1)

static const uintmax_t
    SPSR_NZCV	  = __BITS(31,28),	/* mask of N Z C V */
     SPSR_N	  = __BIT(31),		/* Negative */
     SPSR_Z	  = __BIT(30),		/* Zero */
     SPSR_C	  = __BIT(29),		/* Carry */
     SPSR_V	  = __BIT(28),		/* oVerflow */
    SPSR_A32_Q	  = __BIT(27),		/* A32: Overflow */
    SPSR_A32_J	  = __BIT(24),		/* A32: Jazelle Mode */
    SPSR_A32_IT1  = __BIT(23),		/* A32: IT[1] */
    SPSR_A32_IT0  = __BIT(22),		/* A32: IT[0] */
    SPSR_SS	  = __BIT(21),		/* Software Step */
    SPSR_IL	  = __BIT(20),		/* Instruction Length */
    SPSR_GE	  = __BITS(19,16),	/* A32: SIMD GE */
    SPSR_IT7	  = __BIT(15),		/* A32: IT[7] */
    SPSR_IT6	  = __BIT(14),		/* A32: IT[6] */
    SPSR_IT5	  = __BIT(13),		/* A32: IT[5] */
    SPSR_IT4	  = __BIT(12),		/* A32: IT[4] */
    SPSR_IT3	  = __BIT(11),		/* A32: IT[3] */
    SPSR_IT2	  = __BIT(10),		/* A32: IT[2] */
    SPSR_A64_D	  = __BIT(9),		/* A64: Debug Exception Mask */
    SPSR_A32_E	  = __BIT(9),		/* A32: BE Endian Mode */
    SPSR_A	  = __BIT(8),		/* Async abort (SError) Mask */
    SPSR_I	  = __BIT(7),		/* IRQ Mask */
    SPSR_F	  = __BIT(6),		/* FIQ Mask */
    SPSR_A32_T	  = __BIT(5),		/* A32 Thumb Mode */
    SPSR_M	  = __BITS(4,0),	/* Execution State */
     SPSR_M_EL3H  = 0x0d,
     SPSR_M_EL3T  = 0x0c,
     SPSR_M_EL2H  = 0x09,
     SPSR_M_EL2T  = 0x08,
     SPSR_M_EL1H  = 0x05,
     SPSR_M_EL1T  = 0x04,
     SPSR_M_EL0T  = 0x00,
     SPSR_M_SYS32 = 0x1f,
     SPSR_M_UND32 = 0x1b,
     SPSR_M_ABT32 = 0x17,
     SPSR_M_SVC32 = 0x13,
     SPSR_M_IRQ32 = 0x12,
     SPSR_M_FIQ32 = 0x11,
     SPSR_M_USR32 = 0x10;

AARCH64REG_READ_INLINE(tcr_el1)		/* Translation Control Register */
AARCH64REG_WRITE_INLINE(tcr_el1)

#define TCR_PAGE_SIZE1(tcr)	(1L << ((1L << __SHIFTOUT(tcr, TCR_TG1)) + 8))

AARCH64REG_READ_INLINE(tpidr_el1)	/* Thread ID Register (EL1) */
AARCH64REG_WRITE_INLINE(tpidr_el1)

AARCH64REG_WRITE_INLINE(tpidrro_el0)	/* Thread ID Register (RO for EL0) */

AARCH64REG_READ_INLINE(ttbr0_el0) /* Translation Table Base Register 0 EL0 */
AARCH64REG_WRITE_INLINE(ttbr0_el0)

AARCH64REG_READ_INLINE(ttbr0_el1) /* Translation Table Base Register 0 EL0 */
AARCH64REG_WRITE_INLINE(ttbr0_el1)

AARCH64REG_READ_INLINE(ttbr1_el1) /* Translation Table Base Register 1 EL1 */
AARCH64REG_WRITE_INLINE(ttbr1_el1)

AARCH64REG_READ_INLINE(vbar_el1)	/* Vector Base Address Register */
AARCH64REG_WRITE_INLINE(vbar_el1)

AARCH64REG_READ_INLINE(pmccfiltr_el0)
AARCH64REG_WRITE_INLINE(pmccfiltr_el0)

static const uintmax_t
    PMCCFILTR_P		= __BIT(31),	/* Don't count cycles in EL1 */
    PMCCFILTR_U		= __BIT(30),	/* Don't count cycles in EL0 */
    PMCCFILTR_NSK	= __BIT(29),	/* Don't count cycles in NS EL1 */
    PMCCFILTR_NSU	= __BIT(28),	/* Don't count cycles in NS EL0 */
    PMCCFILTR_NSH	= __BIT(27),	/* Don't count cycles in NS EL2 */
    PMCCFILTR_M		= __BIT(26);	/* Don't count cycles in EL3 */

AARCH64REG_READ_INLINE(pmccntr_el0)

AARCH64REG_READ_INLINE(cntfrq_el0)

AARCH64REG_READ_INLINE(cnthctl_el2)
AARCH64REG_WRITE_INLINE(cnthctl_el2)

static const uintmax_t
    CNTHCTL_EVNTDIR	= __BIT(3),
    CNTHCTL_EVNTEN	= __BIT(2),
    CNTHCTL_EL1PCEN	= __BIT(1),
    CNTHCTL_EL1PCTEN	= __BIT(0);

AARCH64REG_READ_INLINE(cntkctl_el1)
AARCH64REG_WRITE_INLINE(cntkctl_el1)

static const uintmax_t
    CNTKCTL_EL0PTEN	= __BIT(9),	/* EL0 access for CNTP CVAL/TVAL/CTL */
    CNTKCTL_EL0VTEN	= __BIT(8),	/* EL0 access for CNTV CVAL/TVAL/CTL */
    CNTKCTL_ELNTI	= __BITS(7,4),
    CNTKCTL_EVNTDIR	= __BIT(3),
    CNTKCTL_EVNTEN	= __BIT(2),
    CNTKCTL_EL0VCTEN	= __BIT(1),	/* EL0 access for CNTVCT and CNTFRQ */
    CNTKCTL_EL0PCTEN	= __BIT(0);	/* EL0 access for CNTPCT and CNTFRQ */

AARCH64REG_READ_INLINE(cntp_ctl_el0)
AARCH64REG_WRITE_INLINE(cntp_ctl_el0)
AARCH64REG_READ_INLINE(cntp_cval_el0)
AARCH64REG_WRITE_INLINE(cntp_cval_el0)
AARCH64REG_READ_INLINE(cntp_tval_el0)
AARCH64REG_WRITE_INLINE(cntp_tval_el0)
AARCH64REG_READ_INLINE(cntpct_el0)
AARCH64REG_WRITE_INLINE(cntpct_el0)

AARCH64REG_READ_INLINE(cntps_ctl_el1)
AARCH64REG_WRITE_INLINE(cntps_ctl_el1)
AARCH64REG_READ_INLINE(cntps_cval_el1)
AARCH64REG_WRITE_INLINE(cntps_cval_el1)
AARCH64REG_READ_INLINE(cntps_tval_el1)
AARCH64REG_WRITE_INLINE(cntps_tval_el1)

AARCH64REG_READ_INLINE(cntv_ctl_el0)
AARCH64REG_WRITE_INLINE(cntv_ctl_el0)
AARCH64REG_READ_INLINE(cntv_cval_el0)
AARCH64REG_WRITE_INLINE(cntv_cval_el0)
AARCH64REG_READ_INLINE(cntv_tval_el0)
AARCH64REG_WRITE_INLINE(cntv_tval_el0)
AARCH64REG_READ_INLINE(cntvct_el0)
AARCH64REG_WRITE_INLINE(cntvct_el0)

static const uintmax_t
    CNTCTL_ISTATUS	= __BIT(2),	/* Interrupt Asserted */
    CNTCTL_IMASK	= __BIT(1),	/* Timer Interrupt is Masked */
    CNTCTL_ENABLE	= __BIT(0);	/* Timer Enabled */


/* ID_AA64PFR0_EL1: AArch64 Processor Feature Register 0 */
static const uintmax_t
    ID_AA64PFR0_EL1_GIC			= __BITS(24,27), /* GIC CPU IF */
    ID_AA64PFR0_EL1_GIC_SHIFT		= 24,
     ID_AA64PFR0_EL1_GIC_CPUIF_EN	= 1,
     ID_AA64PFR0_EL1_GIC_CPUIF_NONE	= 0,
    ID_AA64PFR0_EL1_ADVSIMD		= __BITS(23,20), /* SIMD */
     ID_AA64PFR0_EL1_ADV_SIMD_IMPL	= 0x0,
     ID_AA64PFR0_EL1_ADV_SIMD_NONE	= 0xf,
    ID_AA64PFR0_EL1_FP			= __BITS(19,16), /* FP */
     ID_AA64PFR0_EL1_FP_IMPL		= 0x0,
     ID_AA64PFR0_EL1_FP_NONE		= 0xf,
    ID_AA64PFR0_EL1_EL3			= __BITS(15,12), /* EL3 handling */
     ID_AA64PFR0_EL1_EL3_NONE		= 0,
     ID_AA64PFR0_EL1_EL3_64		= 1,
     ID_AA64PFR0_EL1_EL3_64_32		= 2,
    ID_AA64PFR0_EL1_EL2			= __BITS(11,8), /* EL2 handling */
     ID_AA64PFR0_EL1_EL2_NONE		= 0,
     ID_AA64PFR0_EL1_EL2_64		= 1,
     ID_AA64PFR0_EL1_EL2_64_32		= 2,
    ID_AA64PFR0_EL1_EL1			= __BITS(7,4), /* EL1 handling */
     ID_AA64PFR0_EL1_EL1_64		= 1,
     ID_AA64PFR0_EL1_EL1_64_32		= 2,
    ID_AA64PFR0_EL1_EL0			= __BITS(3,0), /* EL0 handling */
     ID_AA64PFR0_EL1_EL0_64		= 1,
     ID_AA64PFR0_EL1_EL0_64_32		= 2;

/* ICC_SRE_EL1: Interrupt Controller System Register Enable register */
static const uintmax_t
    ICC_SRE_EL1_SRE			= __BIT(0),
    ICC_SRE_EL1_DFB			= __BIT(1),
    ICC_SRE_EL1_DIB			= __BIT(2);

/* ICC_SRE_EL2: Interrupt Controller System Register Enable register */
static const uintmax_t
    ICC_SRE_EL2_SRE			= __BIT(0),
    ICC_SRE_EL2_DFB			= __BIT(1),
    ICC_SRE_EL2_DIB			= __BIT(2),
    ICC_SRE_EL2_EN			= __BIT(3);



#elif defined(__arm__)

#include <arm/armreg.h>

#endif /* __aarch64__/__arm__ */

#endif /* _AARCH64_ARMREG_H_ */
