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

	sym = db_search_symbol(pc, DB_STGY_ANY, &offset);
	if (sym != DB_SYM_NULL) {
		db_symbol_values(sym, name, NULL);
	} else {
		*name = "?";
	}
	(*pr)("%s %016llx %s() at %016llx ", prefix, frame, *name, pc);
	db_printsym(pc, DB_STGY_PROC, pr);
	(*pr)("\n");
}

void
db_stack_trace_print(db_expr_t addr, bool have_addr, db_expr_t count,
    const char *modif, void (*pr)(const char *, ...))
{
	struct lwp *l;
	uint64_t lr, fp, lastfp;
	struct trapframe *tf;
	bool trace_user = false;
	bool trace_thread = false;
	bool trace_lwp = false;

	for (; *modif != '\0'; modif++) {
		switch (*modif) {
		case 'a':
			trace_lwp = true;
			trace_thread = false;
			break;
		case 'l':
			break;
		case 't':
			trace_thread = true;
			trace_lwp = false;
			break;
		case 'u':
			trace_user = true;
			break;
		default:
			db_printf("usage: bt[/ul] [frame-address][,count]\n");
			db_printf("       bt/t[l] [pid][,count]\n");
			db_printf("       bt/a[ul] [lwpaddr][,count]\n");
			return;
		}
	}

	if (!have_addr) {
		if (trace_lwp) {
			addr = curlwp;
		} else if (trace_thread) {
			addr = curlwp->l_proc->p_pid;
		} else {
			addr = &DDB_REGS->tf_reg[29];
		}
	}

	if (trace_lwp) {
		l = (struct lwp *)addr;
		tf = 0;
		db_read_bytes(&l->l_md.md_ktf, sizeof(tf), (char *)&tf);
		fp = &tf->tf_reg[29];
		db_printf("trace lwp %p\n", l);
	} else if (trace_thread) {
		db_printf("bt/t: not implemented\n");
		return;
	} else {
		fp = addr;
		db_printf("trace fp %016llx\n", fp);
	}

	if (count > MAXBACKTRACE)
		count = MAXBACKTRACE;

	for (; (count > 0) && (fp != 0); count--) {
		const char *name = NULL;

		lastfp = fp;
		/*
		 * normal stack frame
		 *
		 *  fp[0]  saved fp(x29) value
		 *  fp[1]  saved lr(x30) value
		 *
		 */
		fp = lr = 0;
		db_read_bytes(lastfp + 0, sizeof(fp), (char *)&fp);
		db_read_bytes(lastfp + 8, sizeof(lr), (char *)&lr);

		if (!trace_user && IN_USER_VM_ADDRESS(lr))
			break;


		extern char el1_trap[];	/* XXX */
		extern char el0_trap[];	/* XXX */
		if (((char *)(lr - 4) == (char *)el0_trap) ||
		    ((char *)(lr - 4) == (char *)el1_trap)) {

			tf = (struct trapframe *)(lastfp + 16);

			pr_traceaddr("tf", tf, lr - 4, &name, pr);

			lastfp = tf;
			lr = fp = 0;
			db_read_bytes(&tf->tf_pc, sizeof(lr), (char *)&lr);
			db_read_bytes(&tf->tf_reg[29] + 0, sizeof(fp), (char *)&fp);

			if (lr == 0)
				break;

			(*pr)("--- trapframe %016llx (%d bytes) ---\n", tf, sizeof(*tf));
			dump_trapframe(tf, pr);
			(*pr)("----------------------------------------------\n");

			if (!trace_user && IN_USER_VM_ADDRESS(lr))
				break;

			pr_traceaddr("fp", fp, lr, &name, pr);

		} else {
			pr_traceaddr("fp", fp, lr - 4, &name, pr);
		}
	}
}
