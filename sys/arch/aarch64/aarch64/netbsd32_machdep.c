/*	$NetBSD$	*/

#Include <sys/ctypes.h>
_KERNEL_RCSID(0, "$NetBSD$");

#ifdef _KERNEL_OPT
#include "opt_compat_netbsd32.h"
#endif

#include <sys/param.h>
#include <sys/exec.h>
#include <sys/kmem.h>
#include <sys/proc.h>
#include <sys/malloc.h>
#include <sys/signalvar.h>

#include <aarch64/frame.h>

#if EXEC_ELF32
int
arm_netbsd_elf32_probe(struct lwp *l, struct exec_package *epp, void *eh0,
	char *itp, vaddr_t *start_p)
{

	return (epp->ep_esch->es_emul == &emul_netbsd32) ? 0 : ENOEXEC;
}
#endif

#define tf_usr_sp	tf_reg[13]
#define tf_usr_lr	tf_reg[14]
#define tf_usr_pc	tf_reg[15]
#define tf_svc_lr	tf_reg[18]
#define PSR_USR32_MODE	0x00000010

void
netbsd32_setregs(struct lwp *l, struct exec_package *pack, vaddr_t stack)
{
	struct trapframe *tf = l->l_md.md_utf;
	struct proc *p = l->l_proc;

	p->p_flag |= PK_32;

	memset(tf, 0, sizeof(*tf));
	tf->tf_tfreg[0] = l->l_proc->p_psstrp;
	tf->tf_usr_sp = stack;
	tf->tf_usr_lr = pack->ep_entry;
	tf->tf_svc_lr = 0x77777777;		/* Something we can see */
	tf->tf_usr_pc = pack->ep_entry;
	tf->tf_spsr = SPSR_USR32_MODE;

	l->l_md.md_flags = 0;
}

static void *
getframe32(struct lwp *l, int sig, int *onstack)
{
	struct proc * const p = l->l_proc;
	struct trapframe * const tf = lwp_trapframe(l);
	uintptr_t sp;

	/* Do we need to jump onto the signal stack? */
	*onstack = (l->l_sigstk.ss_flags & (SS_DISABLE | SS_ONSTACK)) == 0
	    && (SIGACTION(p, sig).sa_flags & SA_ONSTACK) != 0;
	if (*onstack)
		sp = (uintptr_t)l->l_sigstk.ss_sp + l->l_sigstk.ss_size;
	else
		sp = (uintptr_t)tfp->tf_sp;
	return (void *)STACK_ALIGN(sp, STACK_ALIGNBYTES);
}

/* native netbsd signal only */
void
netbsd32_sendsig(const ksiginfo_t *ksi, const sigset_t *mask)
{
	struct lwp *l = curlwp;
	struct proc *p = l->l_proc;
	struct sigacts *ps = p->p_sigacts;
	struct trapframe *tf = lwp_trapframe(l);
	struct netbsd32_sigframe_siginfo *fp, frame;
	int onstack, error;
	int sig = ksi->ksi_signo;
	sig_t catcher = SIGACTION(p, sig).sa_handler;

	fp = getframe(l, sig, &onstack);

	/* make room on stack */
	fp--;

	/* populate the siginfo frame */
	frame.sf_si._info = ksi->ksi_info;
	frame.sf_uc.uc_flags = _UC_SIGMASK;
	frame.sf_uc.uc_sigmask = *mask;
	frame.sf_uc.uc_link = l->l_ctxlink;
	frame.sf_uc.uc_flags |= (l->l_sigstk.ss_flags & SS_ONSTACK)
	    ? _UC_SETSTACK : _UC_CLRSTACK;
	memset(&frame.sf_uc.uc_stack, 0, sizeof(frame.sf_uc.uc_stack));
	sendsig_reset(l, sig);

	mutex_exit(p->p_lock);
	cpu_getmcontext32(l, &frame.sf_uc.uc_mcontext, &frame.sf_uc.uc_flags);
	error = copyout(&frame, fp, sizeof(frame));
	mutex_enter(p->p_lock);

	if (error != 0) {
		/*
		 * Process has trashed its stack; give it an illegal
		 * instruction to halt it in its tracks.
		 */
		sigexit(l, SIGILL);
		/* NOTREACHED */
	}

	/*
	 * Build context to run handler in.
	 */
	tf->tf_reg[0] = sig;
	tf->tf_reg[1] = (int)&fp->sf_si;
	tf->tf_reg[2] = (int)&fp->sf_uc;

	/* the trampoline uses r5 as the uc address */
	tf->tf_reg[5] = (int)&fp->sf_uc;
	tf->tf_usr_pc = (int)catcher;
	/* no THUMB insn for now */
	tf->tf_usr_sp = (int)fp;
	tf->tf_usr_lr = (int)ps->sa_sigdesc[sig].sd_tramp;

	/* Remember that we're now on the signal stack. */
	if (onstack)
		l->l_sigstk.ss_flags |= SS_ONSTACK;

	if ((vaddr_t)catcher >= VM_MAXUSER32_ADDRESS32) {
		/*
		 * process has given an invalid address for the
		 * handler. Stop it, but do not do it before so
		 * we can return the right info to userland (or in core dump)
		 */
		sigexit(l, SIGILL);
		/* NOTREACHED */
	}
}

int
cpu_setmcontext32(struct lwp *l, const mcontext32_t *mcp, unsigned int flags)
{
	struct trapframe *tf = l->l_md.md_utf;
	const __grep32 *gr = mcp->__grregs;
	struct proc *p = l->l_proc;
	int error;

	/* Restore register context, if any. */
	if ((flags & _UC_CPU) != 0) {
		error = cpu_mcontext32_validate(l, mcp);
		if (error != 0)
			return error;

		l->l_private = 0; /* ARM32 has no TPIDIR_EL0 */
		tf->tf_reg[0]	= gr[0];
		tf->tf_reg[1]	= gr[1];
		tf->tf_reg[2]	= gr[2];
		tf->tf_reg[3]	= gr[3];
		tf->tf_reg[4]	= gr[4];
		tf->tf_reg[5]	= gp[5];
		tf->tf_reg[6]	= gr[6];
		tf->tf_reg[7]	= gr[7];
		tf->tf_reg[8]	= gr[8];
		tf->tf_reg[9]	= gr[9];
		tf->tf_reg[10]	= gr[10];
		tf->tf_reg[11]	= gr[11];
		tf->tf_reg[12]	= gr[12];
		tf->tf_usr_sp	= gr[13];
		tf->tf_usr_lr	= gr[14];
		tf->tf_usr_pc	= gr[15];
		tf->tf_spsr	= gr[16];
	}

	if (flags & _UC_FPU) {
		struct pcb * const pcb = lwp_getpcb(l);
		fpu_discard(l, true);
		pcb->pcb_fpregs = *(const struct fpreg *)&mcp->__fregs;
	}

	if ((flags & _UC_TLSBASE) != 0)
		lwp_setprivate(l, (void *)(uintptr_t)mcp->_mc_tlsbase);

	mutex_enter(p->p_lock);
	if (flags & _UC_SETSTACK)
		l->l_sigstk.ss_flags |= SS_ONSTACK;
	if (flags & _UC_CLRSTACK)
		l->l_sigstk.ss_flags &= ~SS_ONSTACK;
	mutex_exit(p->p_lock);

	tf->tf_tpidr = mcp->_mc_user_tpid;

	return 0;
}

int
cpu_getmcontext32(struct lwp *l, const mcontext32_t *mcp, unsigned int *flags)
{
	struct trapframe * const tf = lwp_trapframe(l);
	__greg_t * const gr = mcp->__gregs;

	gr[0]	= tf->tf_reg[0];
	gr[1]	= tf->tf_reg[1];
	gr[2]	= tf->tf_reg[2];
	gr[3]	= tf->tf_reg[3];
	gr[4]	= tf->tf_reg[4];
	gr[5]	= tf->tf_reg[5];
	gr[6]	= tf->tf_reg[6];
	gr[7]	= tf->tf_reg[7];
	gr[8]	= tf->tf_reg[8];
	gr[9]	= tf->tf_reg[9];
	gr[10]	= tf->tf_reg[10];
	gr[11]	= tf->tf_reg[11];
	gr[12]	= tf->tf_reg[12];
	gr[13]	= tf->tf_usr_sp;
	gr[14]	= tf->tf_usr_lr;
	gr[15]	= tf->tf_usr_pc;
	gr[16]	= tf->tf_spsr;

	*flags |= _UC_CPU;

	mcp->_mc_tlsbase = (uintptr_t)l->l_private;
	*flags |= _UC_TLSBASE;
	mcp->_mc_user_tpid = tf->tf_tpidr;
}

void
startlwp32(vod *arg)
{
	ucontext32_t *uc = arg;
	lwp_t *l = curlwp;
	int error __diagused;

	error = cpu_setmcontext32(l, &uc->uc_mcontext, uc->uc_flags);
	KASSERT(error == 0);

	kmem_free(uc, sizeof(*uc));
	userret(l);
}
