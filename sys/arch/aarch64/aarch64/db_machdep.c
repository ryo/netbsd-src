/* $NetBSD: db_machdep.c,v 1.1 2014/08/10 05:47:37 matt Exp $ */

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
__KERNEL_RCSID(0, "$NetBSD$");

#include "opt_uvmhist.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/lwp.h>
#include <sys/intr.h>

#include <uvm/uvm.h>

#include <aarch64/db_machdep.h>
#include <aarch64/armreg.h>
#include <aarch64/locore.h>
#include <aarch64/pmap.h>

#include <ddb/db_access.h>
#include <ddb/db_command.h>
#include <ddb/db_output.h>
#include <ddb/db_variables.h>
#include <ddb/db_sym.h>
#include <ddb/db_extern.h>
#include <ddb/db_interface.h>

#include <dev/cons.h>

void db_md_cpuinfo_cmd(db_expr_t, bool, db_expr_t, const char *);
void db_md_frame_cmd(db_expr_t, bool, db_expr_t, const char *);
void db_md_lwp_cmd(db_expr_t, bool, db_expr_t, const char *);
#ifdef UVMHIST
void db_md_pmaphist_cmd(db_expr_t, bool, db_expr_t, const char *);
#endif
void db_md_pte_cmd(db_expr_t, bool, db_expr_t, const char *);
void db_md_sysreg_cmd(db_expr_t, bool, db_expr_t, const char *);
void db_md_tlb_cmd(db_expr_t, bool, db_expr_t, const char *);

const struct db_command db_machine_command_table[] = {
	{
		DDB_ADD_CMD(
		    "cpuinfo", db_md_cpuinfo_cmd, 0,
		    "Displays the cpuinfo",
		    NULL, NULL)
	},
	{
		DDB_ADD_CMD(
		    "frame", db_md_frame_cmd, 0,
		    "Displays the contents of a trapframe",
		    "<address>",
		    "\taddress:\taddress of trapfame to display")
	},
	{
		DDB_ADD_CMD(
		    "lwp", db_md_lwp_cmd, 0,
		    "Displays the lwp",
		    "<address>",
		    "\taddress:\taddress of lwp to display")
	},
#ifdef UVMHIST
	{
		DDB_ADD_CMD(
		    "pmaphist", db_md_pmaphist_cmd, 0,
		    "Dump the entire contents of the pmap history",
		    "<param>",
		    "\tparam: 0=clear")
	},
#endif /* UVMHIST */
	{
		DDB_ADD_CMD(
		    "pte", db_md_pte_cmd, 0,
		    "Display information of pte",
		    "<address>",
		    "\taddress:\tvirtual address of page")
	},
#ifdef _KERNEL
	{
		DDB_ADD_CMD(
		    "sysreg", db_md_sysreg_cmd, 0,
		    "Displays system registers",
		    NULL, NULL)
	},
#if 0
	{
		DDB_ADD_CMD(
		    "tlb", db_md_tlb_cmd, 0,
		    "Displays the TLB",
		    NULL, NULL)
	},
#endif
#endif /* _KERNEL */
#if defined(_KERNEL) && defined(MULTIPROCESSOR)
//XXXAARCH64
//	{
//		DDB_ADD_CMD(
//		    "cpu", db_switch_cpu_cmd, 0,
//		    "switch to a different cpu",
//		    NULL, NULL)
//	},
#endif
	{
		DDB_ADD_CMD(
		    "watch", db_md_watch_cmd, 0,
		    "set or clear watchpoint",
		    "<param>",
		    "\tparam: <address> | <#>")
	},
	{
		DDB_ADD_CMD(NULL, NULL, 0,
		    NULL,
		    NULL,NULL)
	}
};

const struct db_variable db_regs[] = {
	{ "x0",    (long *) &ddb_regs.tf_reg[0],  FCN_NULL, NULL },
	{ "x1",    (long *) &ddb_regs.tf_reg[1],  FCN_NULL, NULL },
	{ "x2",    (long *) &ddb_regs.tf_reg[2],  FCN_NULL, NULL },
	{ "x3",    (long *) &ddb_regs.tf_reg[3],  FCN_NULL, NULL },
	{ "x4",    (long *) &ddb_regs.tf_reg[4],  FCN_NULL, NULL },
	{ "x5",    (long *) &ddb_regs.tf_reg[5],  FCN_NULL, NULL },
	{ "x6",    (long *) &ddb_regs.tf_reg[6],  FCN_NULL, NULL },
	{ "x7",    (long *) &ddb_regs.tf_reg[7],  FCN_NULL, NULL },
	{ "x8",    (long *) &ddb_regs.tf_reg[8],  FCN_NULL, NULL },
	{ "x9",    (long *) &ddb_regs.tf_reg[9],  FCN_NULL, NULL },
	{ "x10",   (long *) &ddb_regs.tf_reg[10], FCN_NULL, NULL },
	{ "x11",   (long *) &ddb_regs.tf_reg[11], FCN_NULL, NULL },
	{ "x12",   (long *) &ddb_regs.tf_reg[12], FCN_NULL, NULL },
	{ "x13",   (long *) &ddb_regs.tf_reg[13], FCN_NULL, NULL },
	{ "x14",   (long *) &ddb_regs.tf_reg[14], FCN_NULL, NULL },
	{ "x15",   (long *) &ddb_regs.tf_reg[15], FCN_NULL, NULL },
	{ "x16",   (long *) &ddb_regs.tf_reg[16], FCN_NULL, NULL },
	{ "x17",   (long *) &ddb_regs.tf_reg[17], FCN_NULL, NULL },
	{ "x18",   (long *) &ddb_regs.tf_reg[18], FCN_NULL, NULL },
	{ "x19",   (long *) &ddb_regs.tf_reg[19], FCN_NULL, NULL },
	{ "x20",   (long *) &ddb_regs.tf_reg[20], FCN_NULL, NULL },
	{ "x21",   (long *) &ddb_regs.tf_reg[21], FCN_NULL, NULL },
	{ "x22",   (long *) &ddb_regs.tf_reg[22], FCN_NULL, NULL },
	{ "x23",   (long *) &ddb_regs.tf_reg[23], FCN_NULL, NULL },
	{ "x24",   (long *) &ddb_regs.tf_reg[24], FCN_NULL, NULL },
	{ "x25",   (long *) &ddb_regs.tf_reg[25], FCN_NULL, NULL },
	{ "x26",   (long *) &ddb_regs.tf_reg[26], FCN_NULL, NULL },
	{ "x27",   (long *) &ddb_regs.tf_reg[27], FCN_NULL, NULL },
	{ "x28",   (long *) &ddb_regs.tf_reg[28], FCN_NULL, NULL },
	{ "x29",   (long *) &ddb_regs.tf_reg[29], FCN_NULL, NULL },
	{ "x30",   (long *) &ddb_regs.tf_reg[30], FCN_NULL, NULL },
	{ "sp",    (long *) &ddb_regs.tf_sp,      FCN_NULL, NULL },
	{ "pc",    (long *) &ddb_regs.tf_pc,      FCN_NULL, NULL },
	{ "spsr",  (long *) &ddb_regs.tf_spsr,    FCN_NULL, NULL },
	{ "tpidr", (long *) &ddb_regs.tf_tpidr,   FCN_NULL, NULL },
};

const struct db_variable * const db_eregs = db_regs + __arraycount(db_regs);

int db_active;
db_regs_t ddb_regs;


void
dump_trapframe(struct trapframe *tf, void (*pr)(const char *, ...))
{
	(*pr)( "   pc=%016"PRIxREGISTER
	    ",   spsr=%016"PRIxREGISTER
	    ",    esr=%016"PRIxREGISTER
	    ",    far=%016"PRIxREGISTER"\n",
	    tf->tf_pc, tf->tf_spsr, tf->tf_esr, tf->tf_far);
	(*pr)( "   x0=%016"PRIxREGISTER
	    ",     x1=%016"PRIxREGISTER
	    ",     x2=%016"PRIxREGISTER
	    ",     x3=%016"PRIxREGISTER"\n",
	    tf->tf_reg[0], tf->tf_reg[1], tf->tf_reg[2], tf->tf_reg[3]);
	(*pr)( "   x4=%016"PRIxREGISTER
	    ",     x5=%016"PRIxREGISTER
	    ",     x6=%016"PRIxREGISTER
	    ",     x7=%016"PRIxREGISTER"\n",
	    tf->tf_reg[4], tf->tf_reg[5], tf->tf_reg[6], tf->tf_reg[7]);
	(*pr)( "   x8=%016"PRIxREGISTER
	    ",     x9=%016"PRIxREGISTER
	    ",    x10=%016"PRIxREGISTER
	    ",    x11=%016"PRIxREGISTER"\n",
	    tf->tf_reg[8], tf->tf_reg[9], tf->tf_reg[10], tf->tf_reg[11]);
	(*pr)( "  x12=%016"PRIxREGISTER
	    ",    x13=%016"PRIxREGISTER
	    ",    x14=%016"PRIxREGISTER
	    ",    x15=%016"PRIxREGISTER"\n",
	    tf->tf_reg[12], tf->tf_reg[13], tf->tf_reg[14], tf->tf_reg[15]);
	(*pr)( "  x16=%016"PRIxREGISTER
	    ",    x17=%016"PRIxREGISTER
	    ",    x18=%016"PRIxREGISTER
	    ",    x19=%016"PRIxREGISTER"\n",
	    tf->tf_reg[16], tf->tf_reg[17], tf->tf_reg[18], tf->tf_reg[19]);
	(*pr)( "  x20=%016"PRIxREGISTER
	    ",    x21=%016"PRIxREGISTER
	    ",    x22=%016"PRIxREGISTER
	    ",    x23=%016"PRIxREGISTER"\n",
	    tf->tf_reg[20], tf->tf_reg[21], tf->tf_reg[22], tf->tf_reg[23]);
	(*pr)( "  x24=%016"PRIxREGISTER
	    ",    x25=%016"PRIxREGISTER
	    ",    x26=%016"PRIxREGISTER
	    ",    x27=%016"PRIxREGISTER"\n",
	    tf->tf_reg[24], tf->tf_reg[25], tf->tf_reg[26], tf->tf_reg[27]);
	(*pr)( "  x28=%016"PRIxREGISTER
	    ", fp=x29=%016"PRIxREGISTER
	    ", lr=x30=%016"PRIxREGISTER
	    ",     sp=%016"PRIxREGISTER"\n",
	    tf->tf_reg[28], tf->tf_reg[29], tf->tf_reg[30],  tf->tf_sp);
}

void
db_md_cpuinfo_cmd(db_expr_t addr, bool have_addr, db_expr_t count, const char *modif)
{
	struct cpu_info *ci, cpuinfobuf;
	cpuid_t cpuid;
	int i;

	ci = curcpu();
	db_read_bytes(ci, sizeof(cpuinfobuf), (char *)&cpuinfobuf);

	cpuid = cpuinfobuf.ci_cpuid;
	db_printf("cpu_info=%p\n", ci);
	db_printf("%p cpu[%lu].ci_cpuid        = %lu\n",
	    &ci->ci_cpuid, cpuid, cpuinfobuf.ci_cpuid);
	db_printf("%p cpu[%lu].ci_curlwp       = %p\n",
	    &ci->ci_curlwp, cpuid, cpuinfobuf.ci_curlwp);
	for (i = 0; i < SOFTINT_COUNT; i++) {
		db_printf("%p cpu[%lu].ci_softlwps[%d] = %p\n",
		    &ci->ci_softlwps[i], cpuid, i, cpuinfobuf.ci_softlwps[i]);
	}
	db_printf("%p cpu[%lu].ci_lastintr     = %llu\n",
	    &ci->ci_lastintr, cpuid, cpuinfobuf.ci_lastintr);
	db_printf("%p cpu[%lu].ci_want_resched = %d\n",
	    &ci->ci_want_resched, cpuid, cpuinfobuf.ci_want_resched);
	db_printf("%p cpu[%lu].ci_cpl          = %d\n",
	    &ci->ci_cpl, cpuid, cpuinfobuf.ci_cpl);
	db_printf("%p cpu[%lu].ci_softints     = 0x%08x\n",
	    &ci->ci_softints, cpuid, cpuinfobuf.ci_softints);
	db_printf("%p cpu[%lu].ci_astpending   = 0x%08x\n",
	    &ci->ci_astpending, cpuid, cpuinfobuf.ci_astpending);
	db_printf("%p cpu[%lu].ci_intr_depth   = %u\n",
	    &ci->ci_intr_depth, cpuid, cpuinfobuf.ci_intr_depth);
}

void
db_md_frame_cmd(db_expr_t addr, bool have_addr, db_expr_t count, const char *modif)
{
	struct trapframe *tf;

	if (!have_addr) {
		db_printf("frame: <address>\n");
		return;
	}

	tf = (struct trapframe *)addr;
	dump_trapframe(tf, db_printf);
}

void
db_md_lwp_cmd(db_expr_t addr, bool have_addr, db_expr_t count, const char *modif)
{
	lwp_t *l;

	if (!have_addr) {
		db_printf("lwp: <address>\n");
		return;
	}
	l = (lwp_t *)addr;

#define SAFESTRPTR(p)	(((p) == NULL) ? "NULL" : (p))

	db_printf("lwp=%p\n", l);

	db_printf("\tlwp_getpcb(l)     =%p\n", lwp_getpcb(l));

	db_printf("\tl->l_md.md_utf    =%p\n", l->l_md.md_utf);
	dump_trapframe(l->l_md.md_utf, db_printf);
	db_printf("\tl->l_md.md_ktf    =%p\n", l->l_md.md_ktf);
	if (l->l_md.md_ktf != l->l_md.md_utf)
		dump_trapframe(l->l_md.md_ktf, db_printf);
	db_printf("\tl->l_md.md_onfault=%p\n", l->l_md.md_onfault);
	db_printf("\tl->l_md.md_cpacr  =%016llx\n", l->l_md.md_cpacr);
	db_printf("\tl->l_md.md_flags  =%08x\n", l->l_md.md_flags);

	db_printf("\tl->l_cpu          =%p\n", l->l_cpu);
	db_printf("\tl->l_proc         =%p\n", l->l_proc);
	db_printf("\tl->l_name         =%s\n", SAFESTRPTR(l->l_name));
	db_printf("\tl->l_wmesg        =%s\n", SAFESTRPTR(l->l_wmesg));
}

#ifdef UVMHIST

/* XXX: should be implement to kern_history.c */
static void
kernhist_entry_snprintf(char *buf, size_t buflen, const struct kern_history_ent *e)
{
	struct timeval tv;
	int len, maxlen;
	char *p;

	bintime2timeval(&e->bt, &tv);

	maxlen = buflen;
	p = buf;

	len = snprintf(p, maxlen, "%06" PRIu64 ".%06d ", tv.tv_sec, tv.tv_usec);
	p += len;
	maxlen -= len;
	if (maxlen <= 0)
		return;

	len = snprintf(p, maxlen, "%s#%ld@%d: ", e->fn, e->call, e->cpunum);
	p += len;
	maxlen -= len;
	if (maxlen <= 0)
		return;

	len = snprintf(p, maxlen, e->fmt, e->v[0], e->v[1], e->v[2], e->v[3]);
	p += len;
	maxlen -= len;
	if (maxlen <= 0)
		return;

	snprintf(p, maxlen, "\n");
}

static void
kernhist_dump_iterate(struct kern_history *l, void (*func)(const char *))
{
	int lcv;
	static char buf[512];	/* XXX */

	lcv = l->f;
	do {
		if (l->e[lcv].fmt) {
			kernhist_entry_snprintf(buf, sizeof(buf), &l->e[lcv]);
			func(buf);
		}
		lcv = (lcv + 1) % l->n;
	} while (lcv != l->f);
}


static const char *kernhist_match;

static void
print_pmaphist_func(const char *msg)
{
	if (strstr(msg, kernhist_match) != NULL)
		db_printf("%s", msg);
}

void
db_md_pmaphist_cmd(db_expr_t addr, bool have_addr, db_expr_t count, const char *modif)
{
	UVMHIST_DECL(pmaphist);

	if ((modif != NULL) && (*modif != '\0')) {
		if (strcmp(modif, "h") == 0) {
			db_printf("machine pmaphist[/<match>]\n");
			return;
		}

		kernhist_match = modif;
		db_printf("show pmaphist matched <%s>\n", kernhist_match);

		kernhist_dump_iterate(&pmaphist, print_pmaphist_func);
		return;
	}


	if (have_addr && (addr == 0)) {
		/* XXX */
		pmaphist.f = 0;
		memset(pmaphist.e, 0, sizeof(struct kern_history_ent) * pmaphist.n);

		db_printf("pmap history was cleared\n");
	}

	kernhist_dump(&pmaphist, db_printf);
}
#endif

void
db_md_pte_cmd(db_expr_t addr, bool have_addr, db_expr_t count, const char *modif)
{
	if (!have_addr) {
		db_printf("pte: <address>\n");
		return;
	}
	pmap_db_pteinfo(addr, db_printf);
}

void
db_md_sysreg_cmd(db_expr_t addr, bool have_addr, db_expr_t count, const char *modif)
{
#define SHOW_ARMREG(x)	\
	db_printf("%-16s = %016llx\n", #x, reg_ ## x ## _read())

	SHOW_ARMREG(cbar_el1);
	SHOW_ARMREG(ccsidr_el1);
	SHOW_ARMREG(clidr_el1);
	SHOW_ARMREG(cntfrq_el0);
	SHOW_ARMREG(cntkctl_el1);
	SHOW_ARMREG(cntp_ctl_el0);
	SHOW_ARMREG(cntp_cval_el0);
	SHOW_ARMREG(cntp_tval_el0);
	SHOW_ARMREG(cntpct_el0);
//	SHOW_ARMREG(cntps_ctl_el1);	/* need secure state */
//	SHOW_ARMREG(cntps_cval_el1);	/* need secure state */
//	SHOW_ARMREG(cntps_tval_el1);	/* need secure state */
	SHOW_ARMREG(cntv_ctl_el0);
	SHOW_ARMREG(cntv_ctl_el0);
	SHOW_ARMREG(cntv_cval_el0);
	SHOW_ARMREG(cntv_tval_el0);
	SHOW_ARMREG(cntv_tval_el0);
	SHOW_ARMREG(cntvct_el0);
	SHOW_ARMREG(cpacr_el1);
	SHOW_ARMREG(csselr_el1);
	SHOW_ARMREG(ctr_el0);
	SHOW_ARMREG(currentel);
	SHOW_ARMREG(daif);
	SHOW_ARMREG(dczid_el0);
	SHOW_ARMREG(elr_el1);
	SHOW_ARMREG(esr_el1);
	SHOW_ARMREG(far_el1);
//	SHOW_ARMREG(fpcr);	/* FP trap */
//	SHOW_ARMREG(fpsr);	/* FP trap */
	SHOW_ARMREG(id_aa64afr0_el1);
	SHOW_ARMREG(id_aa64afr1_el1);
	SHOW_ARMREG(id_aa64dfr0_el1);
	SHOW_ARMREG(id_aa64dfr1_el1);
	SHOW_ARMREG(id_aa64isar0_el1);
	SHOW_ARMREG(id_aa64isar1_el1);
	SHOW_ARMREG(id_aa64mmfr0_el1);
	SHOW_ARMREG(id_aa64mmfr1_el1);
	SHOW_ARMREG(id_aa64pfr0_el1);
	SHOW_ARMREG(id_aa64pfr1_el1);
	SHOW_ARMREG(isr_el1);
	SHOW_ARMREG(l2ctlr_el1);
	SHOW_ARMREG(mair_el1);
	SHOW_ARMREG(mdscr_el1);
	SHOW_ARMREG(midr_el1);
	SHOW_ARMREG(mpidr_el1);
	SHOW_ARMREG(mvfr0_el1);
	SHOW_ARMREG(mvfr1_el1);
	SHOW_ARMREG(mvfr2_el1);
	SHOW_ARMREG(nzcv);
	SHOW_ARMREG(par_el1);
	SHOW_ARMREG(pmccfiltr_el0);
	SHOW_ARMREG(pmccntr_el0);
	SHOW_ARMREG(revidr_el1);
//	SHOW_ARMREG(rmr_el1);		/* unknown reason trap */
//	SHOW_ARMREG(rvbar_el1);
	SHOW_ARMREG(sctlr_el1);
	SHOW_ARMREG(spsel);
	SHOW_ARMREG(spsr_el1);
	SHOW_ARMREG(tcr_el1);
	SHOW_ARMREG(tpidr_el0);
	SHOW_ARMREG(tpidr_el1);
	SHOW_ARMREG(ttbr0_el1);
	SHOW_ARMREG(ttbr1_el1);
	SHOW_ARMREG(vbar_el1);
}

/*
 * hardware breakpoint/watchpoint command
 */
static void
aarch64_set_bcr_bvr(int n, uint64_t bcr, uint64_t bvr)
{
#define DBG_BVR_BCR_SET_IF(regno)					\
	do {								\
		if (n == regno) {					\
			reg_dbgbcr ## regno ## _el1_write(bcr);		\
			reg_dbgbvr ## regno ## _el1_write(bvr);		\
			return;						\
		}							\
	} while (0 /* CONSTCOND */)

	DBG_BVR_BCR_SET_IF(0);
	DBG_BVR_BCR_SET_IF(1);
	DBG_BVR_BCR_SET_IF(2);
	DBG_BVR_BCR_SET_IF(3);
	DBG_BVR_BCR_SET_IF(4);
	DBG_BVR_BCR_SET_IF(5);
	DBG_BVR_BCR_SET_IF(6);
	DBG_BVR_BCR_SET_IF(7);
	DBG_BVR_BCR_SET_IF(8);
	DBG_BVR_BCR_SET_IF(9);
	DBG_BVR_BCR_SET_IF(10);
	DBG_BVR_BCR_SET_IF(11);
	DBG_BVR_BCR_SET_IF(12);
	DBG_BVR_BCR_SET_IF(13);
	DBG_BVR_BCR_SET_IF(14);
	DBG_BVR_BCR_SET_IF(15);
}

static void
aarch64_set_wcr_wvr(int n, uint64_t wcr, uint64_t wvr)
{
#define DBG_WVR_WCR_SET_IF(regno)					\
	do {								\
		if (n == regno) {					\
			reg_dbgwcr ## regno ## _el1_write(wcr);		\
			reg_dbgwvr ## regno ## _el1_write(wvr);		\
			return;						\
		}							\
	} while (0 /* CONSTCOND */)

	DBG_WVR_WCR_SET_IF(0);
	DBG_WVR_WCR_SET_IF(1);
	DBG_WVR_WCR_SET_IF(2);
	DBG_WVR_WCR_SET_IF(3);
	DBG_WVR_WCR_SET_IF(4);
	DBG_WVR_WCR_SET_IF(5);
	DBG_WVR_WCR_SET_IF(6);
	DBG_WVR_WCR_SET_IF(7);
	DBG_WVR_WCR_SET_IF(8);
	DBG_WVR_WCR_SET_IF(9);
	DBG_WVR_WCR_SET_IF(10);
	DBG_WVR_WCR_SET_IF(11);
	DBG_WVR_WCR_SET_IF(12);
	DBG_WVR_WCR_SET_IF(13);
	DBG_WVR_WCR_SET_IF(14);
	DBG_WVR_WCR_SET_IF(15);
}

static uint64_t
aarch64_get_dbgwcr(int n)
{
#define DBGWCR_READ_RET_IF(regno)					\
	do {								\
		if (n == regno) {					\
			return reg_dbgwcr ## regno ## _el1_read();	\
		}							\
	} while (0 /* CONSTCOND */)

	DBGWCR_READ_RET_IF(0);
	DBGWCR_READ_RET_IF(1);
	DBGWCR_READ_RET_IF(2);
	DBGWCR_READ_RET_IF(3);
	DBGWCR_READ_RET_IF(4);
	DBGWCR_READ_RET_IF(5);
	DBGWCR_READ_RET_IF(6);
	DBGWCR_READ_RET_IF(7);
	DBGWCR_READ_RET_IF(8);
	DBGWCR_READ_RET_IF(9);
	DBGWCR_READ_RET_IF(10);
	DBGWCR_READ_RET_IF(11);
	DBGWCR_READ_RET_IF(12);
	DBGWCR_READ_RET_IF(13);
	DBGWCR_READ_RET_IF(14);
	DBGWCR_READ_RET_IF(15);

	return 0;
}

static uint64_t
aarch64_get_dbgwvr(int n)
{
#define DBGWVR_READ_RET_IF(regno)					\
	do {								\
		if (n == regno) {					\
			return reg_dbgwvr ## regno ## _el1_read();	\
		}							\
	} while (0 /* CONSTCOND */)

	DBGWVR_READ_RET_IF(0);
	DBGWVR_READ_RET_IF(1);
	DBGWVR_READ_RET_IF(2);
	DBGWVR_READ_RET_IF(3);
	DBGWVR_READ_RET_IF(4);
	DBGWVR_READ_RET_IF(5);
	DBGWVR_READ_RET_IF(6);
	DBGWVR_READ_RET_IF(7);
	DBGWVR_READ_RET_IF(8);
	DBGWVR_READ_RET_IF(9);
	DBGWVR_READ_RET_IF(10);
	DBGWVR_READ_RET_IF(11);
	DBGWVR_READ_RET_IF(12);
	DBGWVR_READ_RET_IF(13);
	DBGWVR_READ_RET_IF(14);
	DBGWVR_READ_RET_IF(15);

	return 0;
}

void
aarch64_breakpoint_clear(int n)
{
	aarch64_set_bcr_bvr(n, 0, 0);
}

void
aarch64_watchpoint_clear(int n)
{
	aarch64_set_wcr_wvr(n, 0, 0);
}

void
aarch64_watchpoint_set(int n, vaddr_t addr, int size, int accesstype)
{
	uint64_t wvr, wcr;
	uint32_t matchbytebit;

	/* BAS must be  all of whose set bits are contiguous */
	matchbytebit = 0xff >> (8 - size);
	matchbytebit <<= (addr & 7);

	/* load, store, or both */
	accesstype &= WATCHPOINT_ACCESS_MASK;
	if (accesstype == 0)
		accesstype = WATCHPOINT_ACCESS_LOADSTORE;

	if (addr == 0) {
		wvr = 0;
		wcr = 0;
	} else {
		wvr = addr & DBGWVR_MASK;
		wcr =
		    __SHIFTIN(0, DBGWCR_MASK) |		/* MASK: no mask */
		    __SHIFTIN(0, DBGWCR_WT) |		/* WT: 0 */
		    __SHIFTIN(0, DBGWCR_LBN) |		/* LBN: 0 */
		    __SHIFTIN(0, DBGWCR_SSC) |		/* SSC: 00 */
		    __SHIFTIN(0, DBGWCR_HMC) |		/* HMC: 0 */
		    __SHIFTIN(matchbytebit, DBGWCR_BAS) | /* BAS: 0-8byte */
		    __SHIFTIN(accesstype, DBGWCR_LSC) |	/* LSC: Load/Store */
		    __SHIFTIN(3, DBGWCR_PAC) |		/* PAC: 11 */
		    __SHIFTIN(1, DBGWCR_E);		/* Enable */
	}

	aarch64_set_wcr_wvr(n, wcr, wvr);
}

#define MAX_BREAKPOINT	15
static int max_breakpoint = MAX_BREAKPOINT;

#define MAX_WATCHPOINT	15
static int max_watchpoint = MAX_WATCHPOINT;

void
db_machdep_init(void)
{
	uint64_t dfr, mdscr;
	int i;

	dfr = reg_id_aa64dfr0_el1_read();
	max_breakpoint = __SHIFTOUT(dfr, ID_AA64DFR0_EL1_BRPS);
	max_watchpoint = __SHIFTOUT(dfr, ID_AA64DFR0_EL1_WRPS);

	for (i = 0; i <= max_breakpoint; i++) {
		/* clear all breakpoints */
		aarch64_breakpoint_clear(i);
	}
	for (i = 0; i <= max_watchpoint; i++) {
		/* clear all watchpoints */
		aarch64_watchpoint_clear(i);
	}

	mdscr = reg_mdscr_el1_read();
	mdscr |= __BIT(15);
	mdscr |= __BIT(13);
	reg_mdscr_el1_write(mdscr);
	reg_oslar_el1_write(0);
	daif_enable(DAIF_D);
}

static void
show_watchpoints(void)
{
	uint64_t wcr, addr;
	unsigned int i, bas, offset, size, nused;

	for (nused = 0, i = 0; i <= max_watchpoint; i++) {
		addr = aarch64_get_dbgwvr(i) & DBGWVR_MASK;
		wcr = aarch64_get_dbgwcr(i);
		if (wcr & DBGWCR_E) {
			bas = __SHIFTOUT(wcr, DBGWCR_BAS);
			if (bas == 0) {
				db_printf("%d: disabled %016llx", i, addr);
			} else {
				offset = ffs(bas) - 1;
				addr += offset;
				bas >>= offset;
				size = ffs(~bas) - 1;

				db_printf("%d: watching %016llx, %d bytes", i,
				    addr, size);

				switch (__SHIFTOUT(wcr, DBGWCR_LSC)) {
				case WATCHPOINT_ACCESS_LOAD:
					db_printf(", load");
					break;
				case WATCHPOINT_ACCESS_STORE:
					db_printf(", store");
					break;
				case WATCHPOINT_ACCESS_LOADSTORE:
					db_printf(", load/store");
					break;
				}
			}
			db_printf("\n");
			nused++;
		}
	}
	db_printf("watchpoint used %d/%d\n", nused, max_watchpoint + 1);
}

void
db_md_watch_cmd(db_expr_t addr, bool have_addr, db_expr_t count, const char *modif)
{
	int i;
	int added, cleared;
	int accesstype, watchsize;

	if (!have_addr) {
		show_watchpoints();
		return;
	}

	accesstype = watchsize = 0;
	if ((modif != NULL) && (*modif != '\0')) {
		int ch;
		for (; *modif != '\0'; modif++) {
			ch = *modif;

			switch (ch) {
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
			case '8':
				watchsize = ch - '0';
				break;
			case 'l':
				accesstype |= WATCHPOINT_ACCESS_LOAD;
				break;
			case 's':
				accesstype |= WATCHPOINT_ACCESS_STORE;
				break;
			}
		}
	}
	if (watchsize == 0)
		watchsize = 4;	/* default: 4byte */
	if (accesstype == 0)
		accesstype = WATCHPOINT_ACCESS_LOADSTORE; /* default */

	added = -1;
	cleared = -1;
	if (0 <= addr && addr <= max_watchpoint) {
		i = addr;
		if (aarch64_get_dbgwcr(i) & DBGWCR_E) {
			/* clear watch */
			aarch64_watchpoint_set(i, 0, 0, 0);
			cleared = i;
		}
	} else {
		for (i = 0; i <= max_watchpoint; i++) {
			if ((aarch64_get_dbgwvr(i) & DBGWVR_MASK) ==
			    (addr & DBGWVR_MASK)) {
				/* clear watch */
				aarch64_watchpoint_set(i, 0, 0, 0);
				cleared = i;
			}
		}
		if (cleared == -1) {
			for (i = 0; i <= max_watchpoint; i++) {
				if (!(aarch64_get_dbgwcr(i) & DBGWCR_E)) {
					/* add watch */
					aarch64_watchpoint_set(i, addr, watchsize, accesstype);
					added = i;
					break;
				}
			}
			if (i > max_watchpoint) {
				db_printf("no more available watchpoint\n");
			}
		}
	}

	if (added >= 0)
		db_printf("add watchpoint %d as %016llx\n", added, addr);
	if (cleared >= 0)
		db_printf("clear watchpoint %d\n", cleared);

	show_watchpoints();
}

int
kdb_trap(int type, struct trapframe *tf)
{
	int s;

	switch (type) {
	case DB_TRAP_UNKNOWN:
	case DB_TRAP_BREAKPOINT:
	case DB_TRAP_BKPT_INSN:
	case DB_TRAP_WATCHPOINT:
	case DB_TRAP_SW_STEP:
		break;
	default:
		if (db_recover != 0) {
			db_error("Faulted in DDB: continuing...\n");
			/* NOTREACHED */
		}
		break;
	}

	/* Should switch to kdb`s own stack here. */
	ddb_regs = *tf;

	s = splhigh();
	db_active++;
	cnpollc(true);
	db_trap(type, 0/*code*/);
	cnpollc(false);
	db_active--;
	splx(s);

	*tf = ddb_regs;

	return 1;
}
