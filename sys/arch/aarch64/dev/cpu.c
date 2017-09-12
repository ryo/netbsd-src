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
__KERNEL_RCSID(1, "$NetBSD$");

#include "locators.h"
#include "opt_multiprocessor.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/cpu.h>

#include <aarch64/armreg.h>
#include <aarch64/cpuvar.h>
#include <aarch64/cpu.h>
#include <aarch64/cpufunc.h>

static int cpu_match(device_t, cfdata_t, void *);
static void cpu_attach(device_t, device_t, void *);
static void cpu_identify(device_t self, struct cpu_info *);

CFATTACH_DECL_NEW(cpu_cpunode, 0,
    cpu_match, cpu_attach, NULL, NULL);

static int
cpu_match(device_t parent, cfdata_t cf, void *aux)
{
	return 1;
}

static void
cpu_attach(device_t parent, device_t self, void *aux)
{
	struct cpu_attach_args *caa;
	struct cpu_info *ci;
	cpuid_t id;

	caa = (struct cpu_attach_args *)aux;
	id = caa->caa_cpucore;

	if (id == 0) {
		ci = curcpu();

	} else {
#ifdef MULTIPROCESSOR
		//XXXAARCH64: notyet?

		uint64_t mpidr = reg_mpidr_el1_read();

		KASSERT(cpu_info[id] == NULL);
		ci = kmem_zalloc(sizeof(*ci), KM_SLEEP);
		ci->ci_cpl = IPL_HIGH;
		ci->ci_cpuid = id;
		if (mpidr & MPIDR_MT) {
			ci->ci_data.cpu_smt_id = mpidr & MPIDR_AFF0;
			ci->ci_data.cpu_core_id = mpidr & MPIDR_AFF1;
			ci->ci_data.cpu_package_id = mpidr & MPIDR_AFF2;
		} else {
			ci->ci_data.cpu_core_id = mpidr & MPIDR_AFF0;
			ci->ci_data.cpu_package_id = mpidr & MPIDR_AFF1;
		}
		ci->ci_data.cpu_cc_freq = cpu_info_store.ci_data.cpu_cc_freq;
		cpu_info[ci->ci_cpuid] = ci;
		if ((arm_cpu_hatched & (1 << id)) == 0) {
			ci->ci_dev = parent;
			self->dv_private = ci;

			aprint_naive(": disabled\n");
			aprint_normal(": disabled (uniprocessor kernel)\n");
			return;
		}
#else
		aprint_naive(": disabled\n");
		aprint_normal(": disabled (uniprocessor kernel)\n");
		return;
#endif
	}

	ci->ci_dev = self;
	self->dv_private = ci;

#ifdef MULTIPROCESSOR
	if (caa->caa_cpucore != 0) {
		aprint_naive("\n");
		aprint_normal(": %s\n", cpu_getmodel());
		mi_cpu_attach(ci);

		// XXXAARCH64
		//pmap_tlb_info_attach();
	}
#endif

	cpu_identify(self, ci);
//	vfp_attach(ci);
}


enum cpu_class {
	CPU_CLASS_NONE,
	CPU_CLASS_CORTEX
};

struct cpuidtab {
	uint32_t cpuid;
	enum cpu_class cpu_class;
	const char *cpu_classname;
};

const struct cpuidtab cpuids[] = {
	{ CPU_ID_CORTEXA53R0, CPU_CLASS_CORTEX, "Cortex-A53" },
	{ CPU_ID_CORTEXA57R0, CPU_CLASS_CORTEX, "Cortex-A57" },
	{ CPU_ID_CORTEXA72R0, CPU_CLASS_CORTEX, "Cortex-A72" }
};

static enum cpu_class
identify_aarch64_model(uint32_t cpuid, char *buf, size_t len)
{
	int i;
	uint32_t cpupart, variant, revision;

	cpupart = cpuid & CPU_ID_PARTNO_MASK;
	variant = __SHIFTOUT(cpuid, CPU_ID_VARIANT_MASK);
	revision = __SHIFTOUT(cpuid, CPU_ID_REVISION_MASK);

	for (i = 0; i < __arraycount(cpuids); i++) {
		if (cpupart == (cpuids[i].cpuid & CPU_ID_PARTNO_MASK)) {
			snprintf(buf, len, "%s r%dp%d",
			    cpuids[i].cpu_classname, variant, revision);
			return cpuids[i].cpu_class;
		}
	}

	snprintf(buf, len, "unknown CPU (ID = 0x%08x)", cpuid);
	return CPU_CLASS_NONE;
}



struct aarch64_cache_info {
	u_int cache_line_size;
	u_int cache_ways;
	u_int cache_sets;
	u_int cache_way_size;
	u_int cache_size;
	bool cache_wb;
	bool cache_wt;
	bool cache_ra;
	bool cache_wa;
};

struct aarch64_cache_info aarch64_l1_icache;
struct aarch64_cache_info aarch64_l1_dcache;
struct aarch64_cache_info aarch64_l2_cache;

static void
extract_ccsidr(struct aarch64_cache_info *cinfo, uint32_t ccsidr)
{
	cinfo->cache_line_size = 1 << (__SHIFTOUT(ccsidr, CCSIDR_LINESIZE) + 4);
	cinfo->cache_ways = __SHIFTOUT(ccsidr, CCSIDR_ASSOC) + 1;
	cinfo->cache_sets = __SHIFTOUT(ccsidr, CCSIDR_NUMSET) + 1;

	/* calc waysize and whole size */
	cinfo->cache_way_size = cinfo->cache_line_size * cinfo->cache_sets;
	cinfo->cache_size = cinfo->cache_way_size * cinfo->cache_ways;

	/* cache types */
	cinfo->cache_wt = ccsidr & CCSIDR_WT;
	cinfo->cache_wb = ccsidr & CCSIDR_WB;
	cinfo->cache_ra = ccsidr & CCSIDR_RA;
	cinfo->cache_wa = ccsidr & CCSIDR_WA;
}

static void
prt_cache(device_t self, int level, int inst, const char *cachetype, const char *cachetype2)
{
	struct aarch64_cache_info cinfo;
	uint32_t ccsidr;

	/* select level N Instruction cache */
	reg_csselr_el1_write(__SHIFTIN(level, CSSELR_LEVEL) |
	    __SHIFTIN(inst == 0 ? 0 : 1, CSSELR_IND));
	asm("isb");

	ccsidr = reg_ccsidr_el1_read();
	extract_ccsidr(&cinfo, ccsidr);
	aprint_normal_dev(self, "L%d %dKB/%dB %d-way%s%s%s%s %s%s cache\n",
	    level + 1,
	    cinfo.cache_size / 1024,
	    cinfo.cache_line_size,
	    cinfo.cache_ways,
	    cinfo.cache_wt ? " write-through" : "",
	    cinfo.cache_wb ? " write-back" : "",
	    cinfo.cache_ra ? " read-allocate" : "",
	    cinfo.cache_wa ? " write-allocate" : "",
	    cachetype2, cachetype);
}

static void
cpu_identify(device_t self, struct cpu_info *ci)
{
	uint64_t mpidr;
	int level;
	uint32_t cpuid;
	uint32_t clidr, ctr, sctlr;	/* for cache */
	const char *cachetype;
	char model[128];

	cpuid = reg_midr_el1_read();
	identify_aarch64_model(cpuid, model, sizeof(model));
	if (ci->ci_cpuid == 0)
		cpu_setmodel("%s", model);

	aprint_naive("\n");
	aprint_normal(": %s\n", model);


	mpidr = reg_mpidr_el1_read();
	aprint_normal_dev(self, "CPU Affinity %llu-%llu-%llu-%llu\n",
	    __SHIFTOUT(mpidr, MPIDR_AFF3),
	    __SHIFTOUT(mpidr, MPIDR_AFF2),
	    __SHIFTOUT(mpidr, MPIDR_AFF1),
	    __SHIFTOUT(mpidr, MPIDR_AFF0));


	/* SCTLR - System Control Register */
	sctlr = reg_sctlr_el1_read();
	if (sctlr & SCTLR_I)
		aprint_normal_dev(self, "IC enabled");
	else
		aprint_normal_dev(self, "IC disabled");

	if (sctlr & SCTLR_C)
		aprint_normal(", DC enabled");
	else
		aprint_normal(", DC disabled");

	if (sctlr & SCTLR_A)
		aprint_normal(", Alignment check enabled\n");
	else {
		switch (sctlr & (SCTLR_SA | SCTLR_SA0)) {
		case SCTLR_SA | SCTLR_SA0:
			aprint_normal(", EL0/EL1 stack Alignment check enabled\n");
			break;
		case SCTLR_SA:
			aprint_normal(", EL1 stack Alignment check enabled\n");
			break;
		case SCTLR_SA0:
			aprint_normal(", EL0 stack Alignment check enabled\n");
			break;
		case 0:
			aprint_normal(", Alignment check disabled\n");
			break;
		}
	}


	/*
	 * CTR - Cache Type Register
	 */
	ctr = reg_ctr_el0_read();
	switch (__SHIFTOUT(ctr, CTR_EL0_L1IP_MASK)) {
	case CTR_EL0_L1IP_AIVIVT:
		cachetype = "ASID-tagged VIVT ";
		break;
	case CTR_EL0_L1IP_VIPT:
		cachetype = "VIPT ";
		break;
	case CTR_EL0_L1IP_PIPT:
		cachetype = "PIPT ";
		break;
	default:
		cachetype = "unknown type ";
		break;
	}

	aprint_normal_dev(self, "Cache Writeback Granule %lluB, Exclusives Reservation Granule %lluB\n",
	    __SHIFTOUT(ctr, CTR_EL0_CWG_LINE) * 4,
	    __SHIFTOUT(ctr, CTR_EL0_ERG_LINE) * 4);


	/*
	 * CLIDR -  Cache Level ID Register
	 * CSSELR - Cache Size Selection Register
	 * CCSIDR - CurrentCache Size ID Register (selected by CSSELR)
	 */

	/* L1, L2, L3, ..., L7 cache */
	for (level = 0, clidr = reg_clidr_el1_read();
	    level < 7; level++, clidr >>= 3) {
		if ((clidr & 7) == 0)	/* no more level cache */
			break;

		switch (clidr & 7) {
		case CLIDR_TYPE_ICACHE:
			prt_cache(self, level, 1, "Instruction", cachetype);
			break;
		case CLIDR_TYPE_DCACHE:
			prt_cache(self, level, 0, "Data", cachetype);
			break;
		case CLIDR_TYPE_UNIFIEDCACHE:
			prt_cache(self, level, 0, "Unified", cachetype);
			break;
		case CLIDR_TYPE_IDCACHE:
			prt_cache(self, level, 1, "Instruction", cachetype);
			prt_cache(self, level, 0, "Data", cachetype);
			break;
		}

		/* L2 or higher is PIPT */
		cachetype = "PIPT ";
	}

	/* and much more... */

	//  ID_AA64DFR0_EL1
	// *ID_AA64DFR1_EL1
	//  ID_AA64ISAR0_EL1
	// *ID_AA64ISAR1_EL1
	//  ID_AA64MMFR0_EL1
	// *ID_AA64MMFR1_EL1
	//  ID_AA64PFR0_EL1
	// *ID_AA64PFR1_EL1
}
