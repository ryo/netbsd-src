/* $NetBSD$ */

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

#include <sys/cdefs.h>

__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/proc.h>

#include <aarch64/db_machdep.h>
#include <aarch64/machdep.h>
#include <aarch64/armreg.h>

#include <ddb/db_access.h>
#include <ddb/db_command.h>
#include <ddb/db_output.h>
#include <ddb/db_variables.h>
#include <ddb/db_sym.h>
#include <ddb/db_extern.h>
#include <ddb/db_interface.h>

#define MAXBACKTRACE	128	/* against infinite loop */


#define IN_USER_VM_ADDRESS(addr)	\
	((VM_MIN_ADDRESS <= (addr)) && ((addr) < VM_MAX_ADDRESS))
#define IN_KERNEL_VM_ADDRESS(addr)	\
	((VM_MIN_KERNEL_ADDRESS <= (addr)) && ((addr) < VM_MAX_KERNEL_ADDRESS))

static void
pr_traceaddr(const char *prefix, uint64_t frame, uint64_t pc, const char **name,
    void (*pr)(const char *, ...))
{
	db_expr_t offset;
	db_sym_t sym;

	(*pr)("%s %016llx ", prefix, frame);
	sym = db_search_symbol(pc, DB_STGY_ANY, &offset);
	if (sym != DB_SYM_NULL) {
		db_symbol_values(sym, name, NULL);
	} else {
		*name = "?";
	}
	(*pr)("%s() at ", *name);
	(*pr)("%016llx ", pc);
	db_printsym(pc, DB_STGY_PROC, pr);
	(*pr)("\n", pc);
}

void
db_stack_trace_print(db_expr_t addr, bool have_addr, db_expr_t count,
    const char *modif, void (*pr)(const char *, ...))
{
	struct lwp *l;
	uint64_t lr, lastlr;
	uint64_t fp, lastfp;
	struct trapframe *tf;

#if 0
	db_printf("%s: addr=%016llx have_addr=%d, count=%lld, modif=%s\n",
	    __func__, addr, have_addr, count, modif);
#endif

	if (!have_addr) {
		fp = DDB_REGS->tf_reg[29];	/* fp = x29 */
		lr = DDB_REGS->tf_lr;
	} else {
		l = (struct lwp *)addr;
		db_read_bytes(&l->l_md.md_ktf, sizeof(tf), (char *)&tf);
		db_read_bytes(&tf->tf_reg[29], sizeof(fp), (char *)&fp);
		db_read_bytes(&tf->tf_reg[30], sizeof(lr), (char *)&lr);
	}

	if (count > MAXBACKTRACE)
		count = MAXBACKTRACE;

	for (; (count > 0) && (fp != 0); count--) {
		const char *name = NULL;

		/*
		 *
		 * interrupt handler trace
		 *
		 *  XXX: depend on implementation of vectors.S and cpuswitch.S
		 *
		 * in case of interrupted in kernel:
		 *
		 *     main()
		 *  ->        :
		 *  -> some-kernel-function()
		 *     ---INTERRUPT!---
		 *  -> <push TRAPFRAME>
		 *  -> el[01]_{sync,irq}() without fp
		 *  -> interrupt()
		 *  -> ARM_IRQ_HANDLER()
		 *  ->        :
		 *  ->        :
		 */

		extern char el1_trap[];	/* XXX */
		extern char el0_trap[];	/* XXX */
		if (((char *)(lr - 4) == (char *)el0_trap) ||
		    ((char *)(lr - 4) == (char *)el1_trap)) {

			tf = (struct trapframe *)(lastfp + 16);
			pr_traceaddr("tf", tf, lr - 4, &name, pr);

			lastlr = lr;
			lastfp = fp;

			db_read_bytes(&tf->tf_pc, sizeof(tf->tf_pc),
			    (char *)&lr);
			db_read_bytes(&tf->tf_reg[29], sizeof(tf->tf_lr),
			    (char *)&fp);

			(*pr)("--- trapframe %016llx (%d bytes) ---\n", tf, sizeof(*tf));
			dump_trapframe(tf, pr);
			(*pr)("----------------------------------------------\n");

		} else {
			/*
			 * normal stack trace
			 *
			 *  fp[0]  saved fp(x29) value
			 *  fp[1]  saved lr(x30) value
			 */
			pr_traceaddr("fp", fp, lr - 4, &name, pr);

			lastlr = lr;
			lastfp = fp;

			db_read_bytes(fp + 8, sizeof(lr), (char *)&lr);
			db_read_bytes(fp, sizeof(fp), (char *)&fp);
		}

		if (have_addr && !IN_KERNEL_VM_ADDRESS(lastfp))
			break;

		if (!IN_USER_VM_ADDRESS(lastfp) && !IN_KERNEL_VM_ADDRESS(lastfp))
			break;
	}
}
