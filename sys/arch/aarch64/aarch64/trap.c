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

#include <sys/param.h>
#include <sys/types.h>
#include <sys/cpu.h>
#include <sys/proc.h>
#include <sys/atomic.h>
#include <sys/systm.h>

#include <uvm/uvm.h>

#include <sys/signal.h>
#include <sys/signalvar.h>
#include <sys/siginfo.h>

#include <aarch64/userret.h>
#include <aarch64/frame.h>
#include <aarch64/machdep.h>
#include <aarch64/pmap.h>
#include <aarch64/pte.h>
#include <aarch64/armreg.h>

static bool pagefault(struct trapframe *, ksiginfo_t *ksi);
static bool pagefault_refmod(struct trapframe *, struct pmap *);
static void trap_ksi_init(ksiginfo_t *, int, int, vaddr_t, register_t);
static void dump_trapframe(struct trapframe *, void (*)(const char *, ...));

static const char * const causenames[] = {
	[ESR_EC_UNKNOWN]	= "Unknown Reason",
	[ESR_EC_SERROR]		= "SError Interrupt",
	[ESR_EC_WFX]		= "WFI or WFE instruction execution",
	[ESR_EC_ILL_STATE]	= "Illegal Execution State",

	[ESR_EC_SYS_REG]	= "MSR/MRS/SYS instruction (!EC0/1/7)",
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

	[ESR_EC_CP15_RT]	= "A32: MCR/MRC access to CP15 !EC=0",
	[ESR_EC_CP15_RRT]	= "A32: MCRR/MRRC access to CP15 !EC=0",
	[ESR_EC_CP14_RT]	= "A32: MCR/MRC access to CP14",
	[ESR_EC_CP14_DT]	= "A32: LDC/STC access to CP14",
	[ESR_EC_CP14_RRT]	= "A32: MRRC access to CP14",
	[ESR_EC_SVC_A32]	= "A32: SVC Instruction Execution",
	[ESR_EC_HVC_A32]	= "A32: HVC Instruction Execution",
	[ESR_EC_SMC_A32]	= "A32: SMC Instruction Execution",
	[ESR_EC_FPID]		= "A32: MCR/MRC access to CP10 !EC=7",
	[ESR_EC_FP_TRAP_A32]	= "A32: FP Exception",
	[ESR_EC_BKPT_INSN_A32]	= "A32: BKPT Instruction Execution",
	[ESR_EC_VECTOR_CATCH]	= "A32: Vector Catch Exception",
};

static void
dump_trapframe(struct trapframe *tf, void (*pr)(const char *, ...))
{
	(*pr)("trapframe @ %p:\n", tf);
	(*pr)("esr=%016"PRIxREGISTER
	    ",  pc=%016"PRIxREGISTER
	    ",  lr=%016"PRIxREGISTER
	    ",  sp=%016"PRIxREGISTER"\n",
	    tf->tf_esr, tf->tf_pc, tf->tf_lr, tf->tf_sp);
	(*pr)(" x0=%016"PRIxREGISTER
	    ",  x1=%016"PRIxREGISTER
	    ",  x2=%016"PRIxREGISTER
	    ",  x3=%016"PRIxREGISTER"\n",
	    tf->tf_reg[0], tf->tf_reg[1], tf->tf_reg[2], tf->tf_reg[3]);
	(*pr)(" x4=%016"PRIxREGISTER
	    ",  x5=%016"PRIxREGISTER
	    ",  x6=%016"PRIxREGISTER
	    ",  x7=%016"PRIxREGISTER"\n",
	    tf->tf_reg[4], tf->tf_reg[5], tf->tf_reg[6], tf->tf_reg[7]);
	(*pr)(" x8=%016"PRIxREGISTER
	    ",  x9=%016"PRIxREGISTER
	    ", x10=%016"PRIxREGISTER
	    ", x11=%016"PRIxREGISTER"\n",
	    tf->tf_reg[8], tf->tf_reg[9], tf->tf_reg[10], tf->tf_reg[11]);
	(*pr)("x12=%016"PRIxREGISTER
	    ", x13=%016"PRIxREGISTER
	    ", x14=%016"PRIxREGISTER
	    ", x15=%016"PRIxREGISTER"\n",
	    tf->tf_reg[12], tf->tf_reg[13], tf->tf_reg[14], tf->tf_reg[15]);
	(*pr)("x16=%016"PRIxREGISTER
	    ", x17=%016"PRIxREGISTER
	    ", x18=%016"PRIxREGISTER
	    ", x19=%016"PRIxREGISTER"\n",
	    tf->tf_reg[16], tf->tf_reg[17], tf->tf_reg[18], tf->tf_reg[19]);
	(*pr)("x20=%016"PRIxREGISTER
	    ", x21=%016"PRIxREGISTER
	    ", x22=%016"PRIxREGISTER
	    ", x23=%016"PRIxREGISTER"\n",
	    tf->tf_reg[20], tf->tf_reg[21], tf->tf_reg[22], tf->tf_reg[23]);
	(*pr)("x24=%016"PRIxREGISTER
	    ", x25=%016"PRIxREGISTER
	    ", x26=%016"PRIxREGISTER
	    ", x27=%016"PRIxREGISTER"\n",
	    tf->tf_reg[24], tf->tf_reg[25], tf->tf_reg[26], tf->tf_reg[27]);
	(*pr)("x28=%016"PRIxREGISTER
	    ", x29=%016"PRIxREGISTER
	    ", x30=%016"PRIxREGISTER"\n",
	    tf->tf_reg[28], tf->tf_reg[29], tf->tf_reg[30]);
}

void
userret(struct lwp *l)
{
	/*
	 * dump l->l_md.md_utf here if necessary. user process is about to
	 * run with these set of registers.
	 */
	mi_userret(l);
}

#define USERMODE(esr)	(SPSR_M_EL0T == ((int)(esr) & SPSR_M))

void
trap(struct trapframe *tf, int reason)
{
	struct lwp * const l = curlwp;
	ksiginfo_t ksi;
	int cause;
	const char *causestr;
	bool usertrap_p = USERMODE(tf->tf_esr);
	bool ok = true;

	KASSERT(reason == 0);

	cause = __SHIFTOUT(tf->tf_esr, ESR_EC);
	if (cause >= __arraycount(causenames))
		cause = ESR_EC_UNKNOWN;
	causestr = causenames[cause];
	if (causestr == NULL)
		causestr = causenames[ESR_EC_UNKNOWN];

	switch (cause) {
	case ESR_EC_FP_ACCESS:
	case ESR_EC_FP_TRAP_A64:
		if (usertrap_p) {
			/* XXX handle delayed FP handover
			fpu_load(curlwp);
			XXX */
			break;
		}
		dump_trapframe(tf, printf);
		panic("%s: EL1 touched FP unit", __func__);

	case ESR_EC_INSN_ABT_EL0:
	case ESR_EC_INSN_ABT_EL1:
	case ESR_EC_DATA_ABT_EL0:
	case ESR_EC_DATA_ABT_EL1:
		ok = pagefault(tf, &ksi);
		break;

	case ESR_EC_PC_ALIGNMENT:
	case ESR_EC_SP_ALIGNMENT:
		ok = false;
		if (usertrap_p) {
			trap_ksi_init(&ksi,
			     SIGBUS, BUS_ADRALN, tf->tf_far, cause);
		}
		break;

	case ESR_EC_BRKPNT_EL1:
	case ESR_EC_SW_STEP_EL1:
	case ESR_EC_WTCHPNT_EL1:
	case ESR_EC_BKPT_INSN_A64:
#ifdef DDB
		/* XXX kdb_trap(tf) XXX */
		break;
#else
		panic("missing DDB");
#endif
	case ESR_EC_BRKPNT_EL0:
	case ESR_EC_SW_STEP_EL0:
	case ESR_EC_WTCHPNT_EL0:
		KASSERT(usertrap_p);
		trap_ksi_init(&ksi,
		     SIGTRAP, TRAP_BRKPT, tf->tf_far, cause);
		ok = false;
		break;
	default:
		usertrap_p = false;
		break;
	}

	if (usertrap_p) {
		if (!ok)
			(*l->l_proc->p_emul->e_trapsignal)(l, &ksi);
		userret(l);
	}
	else if (!ok) {
		printf("%s\n", causestr);
		dump_trapframe(tf, printf);
		panic("%s: unhandled trap", __func__);
	}
}

void
trap_doast(struct trapframe *tf)
{
	/*
	 * allow to have a chance of context switch just prior to user
	 * exception return.
	 */

        atomic_swap_uint(curcpu()->ci_astpending, 0);

        if (curlwp->l_pflag & LP_OWEUPC) {
                curlwp->l_pflag &= ~LP_OWEUPC;
                ADDUPROF(curlwp);
        }

        if (curcpu()->ci_want_resched)
                preempt();
}

static const char * const syndrome[] = {
	[1] = "bad synchronous exception",
	[2] = "bad irq exception",
	[3] = "bad fiq exception",
	[4] = "bad error exception",
};

#define SPSR_A32_P(esr)		((int)(esr) & 0x10)
#define SPSR_A32_DECODE(esr)	((int)(esr) & 0x0c)
#define SPSR_ELx_DECODE(esr)	(((int)(esr) & 0x0c) >> 2)

void
trap_a32(struct trapframe *tf, int reason)
{

	KASSERT(reason == 0);
	KASSERT(SPSR_A32_P(tf->tf_spsr));

	printf("A32 (%x) exception\n", SPSR_A32_DECODE(tf->tf_spsr));
	dump_trapframe(tf, printf);
	panic("%s: unexpected exception", __func__);
}

void
trap_bad(struct trapframe *tf, int reason)
{
	KASSERT(reason > 0 && reason <= 4);

	if (SPSR_A32_P(tf->tf_spsr))
		printf("A32 (%x) ", SPSR_A32_DECODE(tf->tf_spsr));
	else
		printf("EL%d%s ", SPSR_ELx_DECODE(tf->tf_spsr),
		    (tf->tf_spsr & 01) ? "h" : "t");
	printf("%s\n", syndrome[reason]);

	dump_trapframe(tf, printf);
	panic("%s: unexpected exception", __func__);
}

#undef SPSR_A32_P
#undef SPSR_A32_DECODE
#undef SPSR_ELx_DECODE

void
interrupt(struct trapframe *tf)
{
}

// XXXAARCH64 might be populated in frame.h in future

#define FB_X19	0
#define FB_X20	1
#define FB_X21	2
#define FB_X22	3
#define FB_X23	4
#define FB_X24	5
#define FB_X25	6
#define FB_X26	7
#define FB_X27	8
#define FB_X28	9
#define FB_X29	10
#define FB_SP	11
#define FB_LR	12
#define FB_V0	13
#define FB_MAX	14

struct faultbuf {
	register_t fb_reg[FB_MAX];
};

int	cpu_set_onfault(struct faultbuf *, register_t) __returns_twice;
void	cpu_jump_onfault(struct trapframe *, const struct faultbuf *);
void	cpu_unset_onfault(void);
struct faultbuf *cpu_disable_onfault(void);
void	cpu_enable_onfault(struct faultbuf *);

#define ESR_ELx_WNR	(1U<<6)

static bool
pagefault(struct trapframe *tf, ksiginfo_t *ksi)
{
	int cause = __SHIFTOUT(tf->tf_esr, ESR_EC);
	intptr_t addr = trunc_page(tf->tf_far);
	struct proc *p = curlwp->l_proc;
	struct vm_map *map = (addr >= 0) ? &p->p_vmspace->vm_map : kernel_map;
	vm_prot_t ftype;
	struct faultbuf *fb;
	int error, minor;

	/* EL0 -- user process touched kernel address space */
	if (USERMODE(tf->tf_esr) && addr < 0) {
		trap_ksi_init(ksi, SIGSEGV, SEGV_MAPERR, addr, cause);
		return false;
	}

	/* page reference/page modified tracking */
	error = pagefault_refmod(tf, map->pmap);
	if (error)
		return true;

	ftype = VM_PROT_READ |
		(tf->tf_esr & ESR_ELx_WNR) ? VM_PROT_WRITE : 0;

	/* EL0 -- page fault, or user stack growth */
	if (USERMODE(tf->tf_esr)) {
		error = uvm_fault(&p->p_vmspace->vm_map, addr, ftype);
		if (error != 0) {
			minor = (error == EACCES) ? SEGV_ACCERR : SEGV_MAPERR;
			trap_ksi_init(ksi, SIGSEGV, minor, addr, cause);
			return false;
		}
		uvm_grow(p, addr);
		return true;	/* address space growth handled */
	}

	/* page fault plus faultbail path */
	fb = cpu_disable_onfault();
	error = uvm_fault(map, addr, ftype);
	cpu_enable_onfault(fb);
	if (error == 0) {
		if (map != kernel_map)
			uvm_grow(p, addr);
		return true; 	/* address space growth handled */
	}
	if (fb == NULL)
		return false;
	cpu_jump_onfault(tf, fb);
	return true;
}

static bool
pagefault_refmod(struct trapframe *tf, struct pmap *pmap)
{
	intptr_t addr = trunc_page(tf->tf_far);
	struct vm_page *pg;
	struct vm_page_md *mdpg;
	u_int attr;
	paddr_t pa;

	pa = addr;
	pg = PHYS_TO_VM_PAGE(pa);
	attr = 0;
	attr |= VM_PAGE_MD_REFERENCED;
	attr |= VM_PAGE_MD_MODIFIED;

	mdpg = VM_PAGE_TO_MD(pg);
	atomic_or_uint(&mdpg->mdpg_attrs, attr);
	return true;
}

static void
trap_ksi_init(ksiginfo_t *ksi, int signo, int code, vaddr_t addr,
    register_t cause)
{

	KSI_INIT_TRAP(ksi);
	ksi->ksi_signo = signo;
	ksi->ksi_code = code;
	ksi->ksi_addr = (void *)addr;
	ksi->ksi_trap = cause;
}

void
cpu_jump_onfault(struct trapframe *tf, const struct faultbuf *fb)
{

	tf->tf_reg[19] = fb->fb_reg[FB_X19];
	tf->tf_reg[20] = fb->fb_reg[FB_X20];
	tf->tf_reg[21] = fb->fb_reg[FB_X21];
	tf->tf_reg[22] = fb->fb_reg[FB_X22];
	tf->tf_reg[23] = fb->fb_reg[FB_X23];
	tf->tf_reg[24] = fb->fb_reg[FB_X24];
	tf->tf_reg[25] = fb->fb_reg[FB_X25];
	tf->tf_reg[26] = fb->fb_reg[FB_X26];
	tf->tf_reg[27] = fb->fb_reg[FB_X27];
	tf->tf_reg[28] = fb->fb_reg[FB_X28];
	tf->tf_reg[29] = fb->fb_reg[FB_X29];
	tf->tf_reg[0] = fb->fb_reg[FB_V0];
	tf->tf_sp = fb->fb_reg[FB_SP];
	tf->tf_lr = fb->fb_reg[FB_LR];
}

void
cpu_unset_onfault(void)
{

	curlwp->l_md.md_onfault = NULL;
}

struct faultbuf *
cpu_disable_onfault(void)
{
	struct faultbuf * const fb = curlwp->l_md.md_onfault;

	curlwp->l_md.md_onfault = NULL;
	return fb;
}

void
cpu_enable_onfault(struct faultbuf *fb)
{

	curlwp->l_md.md_onfault = NULL;
}

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
	struct faultbuf fb;
	int error;

	if ((error = cpu_set_onfault(&fb, EFAULT)) == 0) {
		memcpy(kdaddr, kfaddr, len);
		cpu_unset_onfault();
	}
	return error;
}

int
copyin(const void *uaddr, void *kaddr, size_t len)
{
	struct faultbuf fb;
	int error;

	if ((error = cpu_set_onfault(&fb, EFAULT)) == 0) {
		memcpy(kaddr, uaddr, len);
		cpu_unset_onfault();
	}
	return error;
}

int
copyout(const void *kaddr, void *uaddr, size_t len)
{
	struct faultbuf fb;
	int error;

	if ((error = cpu_set_onfault(&fb, EFAULT)) == 0) {
		memcpy(uaddr, kaddr, len);
		cpu_unset_onfault();
	}
	return error;
}

int
copystr(const void *kfaddr, void *kdaddr, size_t len, size_t *done)
{
	struct faultbuf fb;
	int error;

	if ((error = cpu_set_onfault(&fb, EFAULT)) == 0) {
		len = strlcpy(kdaddr, kfaddr, len);
		cpu_unset_onfault();
		if (done != NULL) {
			*done = len;
		}
	}
	return error;
}

int
copyinstr(const void *uaddr, void *kaddr, size_t len, size_t *done)
{
	struct faultbuf fb;
	int error;

	if ((error = cpu_set_onfault(&fb, EFAULT)) == 0) {
		len = strlcpy(kaddr, uaddr, len);
		cpu_unset_onfault();
		if (done != NULL) {
			*done = len;
		}
	}
	return error;
}

int
copyoutstr(const void *kaddr, void *uaddr, size_t len, size_t *done)
{
	struct faultbuf fb;
	int error;

	if ((error = cpu_set_onfault(&fb, EFAULT)) == 0) {
		len = strlcpy(uaddr, kaddr, len);
		cpu_unset_onfault();
		if (done != NULL) {
			*done = len;
		}
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

static bool
fetch_user_data(union xubuf *xu, const void *base, size_t len)
{
	struct faultbuf fb;

	if (cpu_set_onfault(&fb, 1) == 0) {
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

static bool
store_user_data(void *base, const union xubuf *xu, size_t len)
{
	struct faultbuf fb;

	if (cpu_set_onfault(&fb, 1) == 0) {
		memcpy(base, xu->b, len);
		cpu_unset_onfault();
		return true;
	}
	return false;
}

int
subyte(void *base, int c)
{
	union xubuf xu;

	xu.l[0] = 0; xu.b[0] = c; // { .b[0] = c, .b[1 ... 3] = 0 }
	return store_user_data(base, &xu, sizeof(xu.b[0])) ? 0 : -1;
}

int
susword(void *base, short c)
{
	union xubuf xu;

	xu.l[0] = 0; xu.w[0] = c; // { .w[0] = c, .w[1] = 0 }
	return store_user_data(base, &xu, sizeof(xu.w[0])) ? 0 : -1;
}

int
suswintr(void *base, short c)
{

	return -1;
}

int
suword(void *base, long c)
{
	union xubuf xu;

	xu.l[0] = c; // { .l[0] = c }
	return store_user_data(base, &xu, sizeof(xu.l[0])) ? 0 : -1;
}
