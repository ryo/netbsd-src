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

#define	tf_usr_sp	tf_regs.r_reg[13]
#define tf_usr_lr	tf_regs.r_reg[14]
#define tf_svc_lr	tf_regs.r_reg[18]
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
        tf->tf_svc_lr = 0x77777777;             /* Something we can see */
        tf->tf_pc = pack->ep_entry;
	tf->tf_spsr = SPSR_USR32_MODE;

	l->l_md.md_flags = 0;
}

/* native and no COMPAT */
void
netbsd32_sendsig(const ksiginfo_t *ksi, const sigset_t *mask)
{
}

int
cpu_setmcontext32(struct lwp *l, const mcontext_t *mcp, unsigned int flags)
{

	/* Restore register context, if any. */
	if ((flags & _UC_CPU) != 0) {
		struct trapframe *tf = l->l_md.md_utf;
		error = cpu_mcontext32_validate(l, mcp);
		if (error != 0)
			return error;

		memcpy(&tf->tf_regs, mcp->__gregs, sizeof(tf->tf_regs));
		l->l_private = mcp->__gregs[_REG_TPIDR];
		/* ARM32 has no TPIDIR_EL0 */
	}

	if (flags & _UC_FPU) {
		struct pcb * const pcb = lwp_getpcb(l);
		fpu_discard(l, true);
		pcb->pcb_fpregs = *(const struct fpreg *)&mcp->__fregs;
	}

	return 0;
}

int
cpu_getmcontext32(struct lwp *l, const mcontext_t *mcp, unsigned int *flags)
{
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
