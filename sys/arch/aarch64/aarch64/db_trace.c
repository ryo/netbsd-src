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

#include <aarch64/db_machdep.h>

#include <ddb/db_access.h>
#include <ddb/db_command.h>
#include <ddb/db_output.h>
#include <ddb/db_variables.h>
#include <ddb/db_sym.h>
#include <ddb/db_extern.h>
#include <ddb/db_interface.h>


/*
 *  fp[0]  saved fp(x29) value
 *  fp[1]  saved lr(x30) value
 */

#define MAXBACKTRACE	64	/* against loop */

void
db_stack_trace_print(db_expr_t addr, bool have_addr, db_expr_t count,
    const char *modif, void (*pr)(const char *, ...))
{
	uint64_t pc, sp, lr, lastlr;
	uint64_t frame;
	int maxbacktrace;

//	db_printf("%s: addr=%016llx have_addr=%d, count=%lld, modif=%s\n",
//	    __func__, addr, have_addr, count, modif);

	if (!have_addr) {
		frame = DDB_REGS->tf_reg[29];	/* fp = x29 */
		sp = DDB_REGS->tf_sp;
		lr = DDB_REGS->tf_lr;
	} else {
		db_printf("%s: %016llx: not supported now\n", __func__, addr);
		return;
	}

	for (maxbacktrace = MAXBACKTRACE;
	    (maxbacktrace > 0) && (frame != 0);) {
		db_expr_t offset;
		db_sym_t sym;
		const char *name;

		pc = lr - 4;

		sym = db_search_symbol(pc, DB_STGY_ANY, &offset);
		if (sym != DB_SYM_NULL) {
			db_symbol_values(sym, &name, NULL);

			(*pr)("%s() at ", name);
		}
		db_printsym(pc, DB_STGY_PROC, pr);
//		(*pr)(" (%016llx)", pc);
		(*pr)("\n", pc);

		lastlr = lr;
		db_read_bytes(frame + 8, sizeof(db_addr_t), (char *)&lr);
		db_read_bytes(frame, sizeof(db_addr_t), (char *)&frame);
	}

}
