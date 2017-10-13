/* $NetBSD: trap.c,v 1.2 2017/08/16 22:48:11 nisimura Exp $ */

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

#include <sys/cdefs.h>

__KERNEL_RCSID(1, "$NetBSD: trap.c,v 1.2 2017/08/16 22:48:11 nisimura Exp $");

#include "opt_arm_intr_impl.h"
#include "opt_compat_netbsd32.h"

#include <sys/param.h>
#include <sys/types.h>
#include <sys/cpu.h>
#include <sys/proc.h>
#include <sys/atomic.h>
#include <sys/systm.h>
#include <sys/signal.h>
#include <sys/signalvar.h>
#include <sys/siginfo.h>
#ifdef KDB
#include <sys/kdb.h>
#endif

#ifdef ARM_INTR_IMPL
#include ARM_INTR_IMPL
#else
#error ARM_INTR_IMPL not defined
#endif

#ifndef ARM_IRQ_HANDLER
#error ARM_IRQ_HANDLER not defined
#endif

#include <aarch64/userret.h>
#include <aarch64/frame.h>
#include <aarch64/machdep.h>
#include <aarch64/armreg.h>
#include <aarch64/locore.h>

#ifdef KDB
#include <machine/db_machdep.h>
#endif
#ifdef DDB
#include <ddb/db_output.h>
#include <machine/db_machdep.h>
#endif


const char * const trap_names[] = {
	[ESR_EC_UNKNOWN]	= "Unknown Reason (Illegal Instruction)",
	[ESR_EC_SERROR]		= "SError Interrupt",
	[ESR_EC_WFX]		= "WFI or WFE instruction execution",
	[ESR_EC_ILL_STATE]	= "Illegal Execution State",

	[ESR_EC_SYS_REG]	= "MSR/MRS/SYS instruction",
	[ESR_EC_SVC_A64]	= "SVC Instruction Execution",
	[ESR_EC_HVC_A64]	= "HVC Instruction Execution",
	[ESR_EC_SMC_A64]	= "SMC Instruction Execution",

	[ESR_EC_INSN_ABT_EL0]	= "Instruction Abort (EL0)",
	[ESR_EC_INSN_ABT_EL1]	= "Instruction Abort (EL1)",
	[ESR_EC_DATA_ABT_EL0]	= "Data Abort (EL0)",
	[ESR_EC_DATA_ABT_EL1]	= "Data Abort (EL1)",

	[ESR_EC_PC_ALIGNMENT]	= "Misaligned PC",
	[ESR_EC_SP_ALIGNMENT]	= "Misaligned SP",

	[ESR_EC_FP_ACCESS]	= "Access to SIMD/FP Registers",
	[ESR_EC_FP_TRAP_A64]	= "FP Exception",

	[ESR_EC_BRKPNT_EL0]	= "Breakpoint Exception (EL0)",
	[ESR_EC_BRKPNT_EL1]	= "Breakpoint Exception (EL1)",
	[ESR_EC_SW_STEP_EL0]	= "Software Step (EL0)",
	[ESR_EC_SW_STEP_EL1]	= "Software Step (EL1)",
	[ESR_EC_WTCHPNT_EL0]	= "Watchpoint (EL0)",
	[ESR_EC_WTCHPNT_EL1]	= "Watchpoint (EL1)",
	[ESR_EC_BKPT_INSN_A64]	= "BKPT Instruction Execution",

	[ESR_EC_CP15_RT]	= "A32: MCR/MRC access to CP15",
	[ESR_EC_CP15_RRT]	= "A32: MCRR/MRRC access to CP15",
	[ESR_EC_CP14_RT]	= "A32: MCR/MRC access to CP14",
	[ESR_EC_CP14_DT]	= "A32: LDC/STC access to CP14",
	[ESR_EC_CP14_RRT]	= "A32: MRRC access to CP14",
	[ESR_EC_SVC_A32]	= "A32: SVC Instruction Execution",
	[ESR_EC_HVC_A32]	= "A32: HVC Instruction Execution",
	[ESR_EC_SMC_A32]	= "A32: SMC Instruction Execution",
	[ESR_EC_FPID]		= "A32: MCR/MRC access to CP10",
	[ESR_EC_FP_TRAP_A32]	= "A32: FP Exception",
	[ESR_EC_BKPT_INSN_A32]	= "A32: BKPT Instruction Execution",
	[ESR_EC_VECTOR_CATCH]	= "A32: Vector Catch Exception"
};

void
userret(struct lwp *l)
{
	mi_userret(l);
}

void
trap_doast(struct trapframe *tf)
{
	/*
	 * allow to have a chance of context switch just prior to user
	 * exception return.
	 */

	atomic_swap_uint(&curcpu()->ci_astpending, 0);

	if (curlwp->l_pflag & LP_OWEUPC) {
		curlwp->l_pflag &= ~LP_OWEUPC;
		ADDUPROF(curlwp);
	}

	if (curcpu()->ci_want_resched)
		preempt();
}

void
trap_el1h_sync(struct trapframe *tf)
{
	const uint32_t esr = tf->tf_esr;
	const uint32_t eclass = __SHIFTOUT(esr, ESR_EC); /* exception class */
	const char *trapname;

	/* enable traps and interrupts */
	daif_enable(DAIF_D|DAIF_A|DAIF_I|DAIF_F);

	if (eclass >= __arraycount(trap_names) || trap_names[eclass] == NULL)
		trapname = trap_names[0];
	else
		trapname = trap_names[eclass];

	switch (eclass) {
	case ESR_EC_INSN_ABT_EL1:
	case ESR_EC_DATA_ABT_EL1:
		if (!data_abort_handler(tf, eclass, trapname))
			panic("Fatal abort");
		break;

	case ESR_EC_BRKPNT_EL1:
	case ESR_EC_SW_STEP_EL1:
	case ESR_EC_WTCHPNT_EL1:
	case ESR_EC_BKPT_INSN_A64:
#ifdef DDB
		kdb_trap(0, tf);
#else
		panic("No debugger in kernel");
#endif
		break;

	case ESR_EC_FP_ACCESS:
	case ESR_EC_FP_TRAP_A64:
	case ESR_EC_ILL_STATE:
	case ESR_EC_PC_ALIGNMENT:
	case ESR_EC_SP_ALIGNMENT:
	default:
		panic("Trap: fatal %s: pc=%016llx sp=%016llx esr=%08x", trapname,
		    tf->tf_pc, tf->tf_sp, esr);
		break;
	}
}

void
trap_el0_sync(struct trapframe *tf)
{
	struct lwp * const l = curlwp;
	const uint32_t esr = tf->tf_esr;
	const uint32_t eclass = __SHIFTOUT(esr, ESR_EC); /* exception class */

	/* enable traps and interrupts */
	daif_enable(DAIF_D|DAIF_A|DAIF_I|DAIF_F);

	/* XXXAARCH64 */
	const char *trapname;
	if (eclass >= __arraycount(trap_names) || trap_names[eclass] == NULL)
		trapname = trap_names[0];
	else
		trapname = trap_names[eclass];


	switch (eclass) {
	case ESR_EC_FP_ACCESS:
		fpu_load(l);
		userret(l);
		break;
	case ESR_EC_FP_TRAP_A64:
		do_trapsignal(l, SIGFPE, FPE_FLTUND, NULL, esr);	/* XXXAARCH64 */
		userret(l);
		break;
	case ESR_EC_SVC_A64:
		(*l->l_proc->p_md.md_syscall)(tf);
		break;

	case ESR_EC_INSN_ABT_EL0:
	case ESR_EC_DATA_ABT_EL0:
		data_abort_handler(tf, eclass, NULL);
		userret(l);
		break;

	case ESR_EC_PC_ALIGNMENT:
		do_trapsignal(l, SIGBUS, BUS_ADRALN, tf->tf_pc, esr);
		userret(l);
		break;
	case ESR_EC_SP_ALIGNMENT:
		do_trapsignal(l, SIGBUS, BUS_ADRALN, tf->tf_sp, esr);
		userret(l);
		break;

	case ESR_EC_BKPT_INSN_A64:
	case ESR_EC_BRKPNT_EL0:
	case ESR_EC_SW_STEP_EL0:
	case ESR_EC_WTCHPNT_EL0:
		// XXXAARCH64: notyet
		printf("%s: %s\n", __func__, trapname);
		do_trapsignal(l, SIGTRAP, TRAP_BRKPT, tf->tf_pc, esr);
		userret(l);
		break;

	default:
		// XXXAARCH64: notyet
		printf("%s:%d: %s\n", __func__, __LINE__, trapname);
		do_trapsignal(l, SIGILL, ILL_ILLTRP, tf->tf_pc, esr);
		userret(l);
		break;
	}
}

void
interrupt(struct trapframe *tf)
{
	struct cpu_info * const ci = curcpu();

	__asm __volatile ("clrex; dmb sy");	/* XXXAARCH64: really need dmb ? */

	/* enable traps */
	daif_enable(DAIF_D|DAIF_A);

	ci->ci_intr_depth++;
	ARM_IRQ_HANDLER(tf);
	ci->ci_intr_depth--;

	cpu_dosoftints();
}

void
trap_el0_32sync(struct trapframe *tf)
{
	struct lwp * const l = curlwp;
	const uint32_t esr = tf->tf_esr;
	const uint32_t eclass = __SHIFTOUT(esr, ESR_EC); /* exception class */

	/* enable traps and interrupts */
	daif_enable(DAIF_D|DAIF_A|DAIF_I|DAIF_F);

	/* XXXAARCH64 */
	const char *trapname;
	if (eclass >= __arraycount(trap_names) || trap_names[eclass] == NULL)
		trapname = trap_names[0];
	else
		trapname = trap_names[eclass];

	switch (eclass) {
#ifdef COMPAT_NETBSD32
	case ESR_EC_SVC_A32:
		(*l->l_proc->p_md.md_syscall)(tf);
		break;
	case ESR_EC_CP15_RT:
	case ESR_EC_CP15_RRT:
	case ESR_EC_CP14_RT:
	case ESR_EC_CP14_DT:
	case ESR_EC_CP14_RRT:
	case ESR_EC_FP_TRAP_A32:
	case ESR_EC_BKPT_INSN_A32:
		// XXXAARCH64: notyet
		printf("%s:%d: %s\n", __func__, __LINE__, trapname);
		do_trapsignal(l SIGILL, ILL_ILLTRP, tf->tf_pc, esr);
		userret(l);
		break;
#endif /* COMPAT_NETBSD32 */
	default:
		// XXXAARCH64: notyet
		printf("%s:%d: %s\n", __func__, __LINE__, trapname);
		do_trapsignal(l, SIGILL, ILL_ILLTRP, tf->tf_pc, esr);
		userret(l);
		break;
	}
}

#define bad_trap_panic(trapfunc)	\
void					\
trapfunc(struct trapframe *tf)		\
{					\
	panic("%s", __func__);		\
}
bad_trap_panic(trap_el1t_sync)
bad_trap_panic(trap_el1t_irq)
bad_trap_panic(trap_el1t_fiq)
bad_trap_panic(trap_el1t_error)
bad_trap_panic(trap_el1h_fiq)
bad_trap_panic(trap_el1h_error)
bad_trap_panic(trap_el0_fiq)
bad_trap_panic(trap_el0_error)
bad_trap_panic(trap_el0_32fiq)
bad_trap_panic(trap_el0_32error)


void
cpu_jump_onfault(struct trapframe *tf, const label_t *label)
{
	tf->tf_reg[19] = label->lb_reg[LBL_X19];
	tf->tf_reg[20] = label->lb_reg[LBL_X20];
	tf->tf_reg[21] = label->lb_reg[LBL_X21];
	tf->tf_reg[22] = label->lb_reg[LBL_X22];
	tf->tf_reg[23] = label->lb_reg[LBL_X23];
	tf->tf_reg[24] = label->lb_reg[LBL_X24];
	tf->tf_reg[25] = label->lb_reg[LBL_X25];
	tf->tf_reg[26] = label->lb_reg[LBL_X26];
	tf->tf_reg[27] = label->lb_reg[LBL_X27];
	tf->tf_reg[28] = label->lb_reg[LBL_X28];
	tf->tf_reg[29] = label->lb_reg[LBL_X29];
	tf->tf_reg[0] = EFAULT;
	tf->tf_sp = label->lb_reg[LBL_SP];
	tf->tf_lr = label->lb_reg[LBL_LR];
}

#if 1
/*
 * kcopy(9)
 * int kcopy(const void *src, void *dst, size_t len);
 *
 * copy(9)
 * int copyin(const void *uaddr, void *kaddr, size_t len);
 * int copyout(const void *kaddr, void *uaddr, size_t len);
 * int copystr(const void *kfaddr, void *kdaddr, size_t len, size_t *done);
 * int copyinstr(const void *uaddr, void *kaddr, size_t len, size_t *done);
 * int copyoutstr(const void *kaddr, void *uaddr, size_t len, size_t *done);
 */
int
kcopy(const void *kfaddr, void *kdaddr, size_t len)
{
	label_t label;
	int error;

	if ((error = cpu_set_onfault(&label)) == 0) {
		memcpy(kdaddr, kfaddr, len);
		cpu_unset_onfault();
	}
	return error;
}

int
copyin(const void *uaddr, void *kaddr, size_t len)
{
	label_t label;
	int error;

	if ((error = cpu_set_onfault(&label)) == 0) {
		memcpy(kaddr, uaddr, len);
		cpu_unset_onfault();
	}
	return error;
}

int
copyout(const void *kaddr, void *uaddr, size_t len)
{
	label_t label;
	int error;

	if ((error = cpu_set_onfault(&label)) == 0) {
		memcpy(uaddr, kaddr, len);
		cpu_unset_onfault();
	}
	return error;
}

static inline int
_copystr(char *dst, const char *src, size_t len, size_t *done)
{
	size_t i;

	for (i = 0; i < len; i++) {
		if ((*dst++ = *src++) == '\0') {
			if (done != NULL)
				*done = i + 1;
			return 0;
		}
	}
	if (done != NULL)
		*done = len;

	return ENAMETOOLONG;
}

int
copystr(const void *kfaddr, void *kdaddr, size_t len, size_t *done)
{
	label_t label;
	int error;

	if ((error = cpu_set_onfault(&label)) == 0) {
		error = _copystr(kdaddr, kfaddr, len, done);
		cpu_unset_onfault();
	}
	return error;
}

int
copyinstr(const void *uaddr, void *kaddr, size_t len, size_t *done)
{
	label_t label;
	int error;

	if ((error = cpu_set_onfault(&label)) == 0) {
		error = _copystr(kaddr, uaddr, len, done);
		cpu_unset_onfault();
	}
	return error;
}

int
copyoutstr(const void *kaddr, void *uaddr, size_t len, size_t *done)
{
	label_t label;
	int error;

	if ((error = cpu_set_onfault(&label)) == 0) {
		error = _copystr(uaddr, kaddr, len, done);
		cpu_unset_onfault();
	}
	return error;
}

/*
 * fetch(9)
 * int fubyte(const void *base);
 * int fusword(const void *base);
 * int fuswintr(const void *base);
 * long fuword(const void *base);
 *
 * store(9)
 * int subyte(void *base, int c);
 * int susword(void *base, short c);
 * int suswintr(void *base, short c);
 * int suword(void *base, long c);
 */

union xubuf {
	uint8_t b[4];
	uint16_t w[2];
	uint32_t l[1];
};

static inline bool
fetch_user_data(union xubuf *xu, const void *base, size_t len)
{
	label_t label;

	if (cpu_set_onfault(&label) == 0) {
		memcpy(xu->b, base, len);
		cpu_unset_onfault();
		return true;
	}
	return false;
}

int
fubyte(const void *base)
{
	union xubuf xu;

	if (fetch_user_data(&xu, base, sizeof(xu.b[0])))
		return xu.b[0];
	return -1;
}

int
fusword(const void *base)
{
	union xubuf xu;

	if (fetch_user_data(&xu, base, sizeof(xu.w[0])))
		return xu.w[0];
	return -1;
}

int
fuswintr(const void *base)
{
	panic("%s: not implemented", __func__);
	return -1;
}

long
fuword(const void *base)
{
	union xubuf xu;

	if (fetch_user_data(&xu, base, sizeof(xu.l[0])))
		return xu.l[0];
	return -1;
}

static inline int
store_user_data(void *base, const union xubuf *xu, size_t len)
{
	label_t label;

	if (cpu_set_onfault(&label) == 0) {
		memcpy(base, xu->b, len);
		cpu_unset_onfault();
		return 0;
	}
	return -1;
}

int
subyte(void *base, int c)
{
	union xubuf xu;

	xu.l[0] = 0; xu.b[0] = c; // { .b[0] = c, .b[1 ... 3] = 0 }
	return store_user_data(base, &xu, sizeof(xu.b[0]));
}

int
susword(void *base, short c)
{
	union xubuf xu;

	xu.l[0] = 0; xu.w[0] = c; // { .w[0] = c, .w[1] = 0 }
	return store_user_data(base, &xu, sizeof(xu.w[0]));
}

int
suswintr(void *base, short c)
{
	panic("%s: not implemented", __func__);
	return -1;
}

int
suword(void *base, long c)
{
	union xubuf xu;

	xu.l[0] = c; // { .l[0] = c }
	return store_user_data(base, &xu, sizeof(xu.l[0]));
}
#endif
