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
#include <sys/kmem.h>

#include <aarch64/armreg.h>
#include <aarch64/cpuvar.h>
#include <aarch64/cpu.h>
#include <aarch64/cpufunc.h>

static int cpu_match(device_t, cfdata_t, void *);
static void cpu_attach(device_t, device_t, void *);
static void cpu_identify(device_t self, struct cpu_info *);
static void cpu_identify2(device_t self, struct cpu_info *);

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
		panic("notyet");
	}
#endif

	cpu_identify(self, ci);

//xxxxxxxxxxxxxxxxx
void cpucache_wbinv(void);
	cpucache_wbinv();

	cpu_identify2(self, ci);
}

struct cpuidtab {
	uint32_t cpu_partnum;
	const char *cpu_name;
	const char *cpu_class;
	const char *cpu_architecture;
};

#define CPU_PARTMASK	(CPU_ID_IMPLEMENTOR_MASK | CPU_ID_PARTNO_MASK)

const struct cpuidtab cpuids[] = {
	{ CPU_ID_CORTEXA53R0 & CPU_PARTMASK, "Cortex-A53", "Cortex", "V8-A" },
	{ CPU_ID_CORTEXA57R0 & CPU_PARTMASK, "Cortex-A57", "Cortex", "V8-A" },
	{ CPU_ID_CORTEXA72R0 & CPU_PARTMASK, "Cortex-A72", "Cortex", "V8-A" },
	{ CPU_ID_CORTEXA73R0 & CPU_PARTMASK, "Cortex-A73", "Cortex", "V8-A" },
	{ CPU_ID_CORTEXA55R1 & CPU_PARTMASK, "Cortex-A55", "Cortex", "V8.2-A" },
	{ CPU_ID_CORTEXA75R2 & CPU_PARTMASK, "Cortex-A75", "Cortex", "V8.2-A" },
};

static void
identify_aarch64_model(uint32_t cpuid, char *buf, size_t len)
{
	int i;
	uint32_t cpupart, variant, revision;

	cpupart = cpuid & CPU_PARTMASK;
	variant = __SHIFTOUT(cpuid, CPU_ID_VARIANT_MASK);
	revision = __SHIFTOUT(cpuid, CPU_ID_REVISION_MASK);

	for (i = 0; i < __arraycount(cpuids); i++) {
		if (cpupart == cpuids[i].cpu_partnum) {
			snprintf(buf, len, "%s r%dp%d (%s %s core)",
			    cpuids[i].cpu_name, variant, revision,
			    cpuids[i].cpu_class,
			    cpuids[i].cpu_architecture);
			return;
		}
	}

	snprintf(buf, len, "unknown CPU (ID = 0x%08x)", cpuid);
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

// XXXAARCH64
#if 0
#define MAX_CACHE_LEVEL	8

struct aarch64_cache_type {
	int level;				/* 0:L1, 1:L2, ... 7:L8 cache */
	enum cachetype cachetype;		/* VIVT/VIPT/PIPT */
	struct aarch64_cache_info *icacheinfo;
	struct aarch64_cache_info *dcacheinfo;
}
struct aarch64_cache_type aarch64_cache_type[MAX_CACHE_LEVEL];
#endif

// XXXAARCH64
struct aarch64_cache_info aarch64_l1_icache;
struct aarch64_cache_info aarch64_l1_dcache;
struct aarch64_cache_info aarch64_l2_cache;

// XXXAARCH64
static inline void
cache_clean(int level, struct aarch64_cache_info *cinfo)
{
	uint64_t x;
	unsigned int set, way, setshift, wayshift;

	setshift = ffs(cinfo->cache_line_size) - 1;
	wayshift = 32 - (ffs(cinfo->cache_ways) - 1);

	cpu_drain_writebuf();

	for (way = 0; way < cinfo->cache_ways; way++) {
		for (set = 0; set < cinfo->cache_sets; set++) {
			x = (way << wayshift) | (set << setshift) | (level << 1);
			__asm __volatile ("dc csw, %0; dsb sy" :: "r"(x));
		}
	}
}

static inline void
cache_wbinv(int level, struct aarch64_cache_info *cinfo)
{
	uint64_t x;
	unsigned int set, way, setshift, wayshift;

	setshift = ffs(cinfo->cache_line_size) - 1;
	wayshift = 32 - (ffs(cinfo->cache_ways) - 1);

	cpu_drain_writebuf();

	for (way = 0; way < cinfo->cache_ways; way++) {
		for (set = 0; set < cinfo->cache_sets; set++) {
			x = (way << wayshift) | (set << setshift) | (level << 1);
			__asm __volatile ("dc cisw, %0; dsb sy" :: "r"(x));
		}
	}
}

static inline void
cache_inv(int level, struct aarch64_cache_info *cinfo)
{
	uint64_t x;
	unsigned int set, way, setshift, wayshift;

	setshift = ffs(cinfo->cache_line_size) - 1;
	wayshift = 32 - (ffs(cinfo->cache_ways) - 1);

	cpu_drain_writebuf();

	for (way = 0; way < cinfo->cache_ways; way++) {
		for (set = 0; set < cinfo->cache_sets; set++) {
			x = (way << wayshift) | (set << setshift) | (level << 1);
			__asm __volatile ("dc isw, %0; dsb sy" :: "r"(x));
		}
	}
}

void cpucache_clean(void);
void cpucache_wbinv(void);
void cpucache_inv(void);

void
cpucache_clean(void)
{
	cache_clean(0, &aarch64_l1_dcache);
	cache_clean(1, &aarch64_l2_cache);
}

void
cpucache_wbinv(void)
{
	cache_wbinv(0, &aarch64_l1_dcache);
	cache_wbinv(1, &aarch64_l2_cache);
}

void
cpucache_inv(void)
{
	cache_inv(0, &aarch64_l1_dcache);
	cache_inv(1, &aarch64_l2_cache);
}

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

	// XXXAARCH64
	if ((level == 0) && inst)
		memcpy(&aarch64_l1_icache, &cinfo, sizeof(cinfo));
	if ((level == 0) && !inst)
		memcpy(&aarch64_l1_dcache, &cinfo, sizeof(cinfo));
	if (level == 1)
		memcpy(&aarch64_l2_cache, &cinfo, sizeof(cinfo));


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
}


/*
 * identify vfp, etc.
 */
static void
cpu_identify2(device_t self, struct cpu_info *ci)
{
	uint64_t aidr, revidr;
	uint64_t dfr0, mmfr0;
	uint64_t isar0, pfr0, mvfr0, mvfr1;

	aidr = reg_id_aa64isar0_el1_read();
	revidr = reg_revidr_el1_read();

	dfr0 = reg_id_aa64dfr0_el1_read();
	mmfr0 = reg_id_aa64mmfr0_el1_read();

	isar0 = reg_id_aa64isar0_el1_read();
	pfr0 = reg_id_aa64pfr0_el1_read();
	mvfr0 = reg_mvfr0_el1_read();
	mvfr1 = reg_mvfr1_el1_read();


	aprint_normal_dev(self, "revID=0x%llx", revidr);

	/* ID_AA64DFR0_EL1 */
	switch (__SHIFTOUT(dfr0, ID_AA64DFR0_EL1_PMUVER)) {
	case ID_AA64DFR0_EL1_PMUVER_V3:
		aprint_normal(", PMCv3");
		break;
	case ID_AA64DFR0_EL1_PMUVER_NOV3:
		aprint_normal(", PMC");
		break;
	}

	/* ID_AA64MMFR0_EL1 */
	switch (__SHIFTOUT(mmfr0, ID_AA64MMFR0_EL1_TGRAN4)) {
	case ID_AA64MMFR0_EL1_TGRAN4_4KB:
		aprint_normal(", 4k table");
		break;
	}
	switch (__SHIFTOUT(mmfr0, ID_AA64MMFR0_EL1_TGRAN16)) {
	case ID_AA64MMFR0_EL1_TGRAN16_16KB:
		aprint_normal(", 16k table");
		break;
	}
	switch (__SHIFTOUT(mmfr0, ID_AA64MMFR0_EL1_TGRAN64)) {
	case ID_AA64MMFR0_EL1_TGRAN64_64KB:
		aprint_normal(", 64k table");
		break;
	}

	switch (__SHIFTOUT(mmfr0, ID_AA64MMFR0_EL1_ASIDBITS)) {
	case ID_AA64MMFR0_EL1_ASIDBITS_8BIT:
		aprint_normal(", 8bit ASID");
		break;
	case ID_AA64MMFR0_EL1_ASIDBITS_16BIT:
		aprint_normal(", 16bit ASID");
		break;
	}
	aprint_normal("\n");



	aprint_normal_dev(self, "auxID=0x%llx", aidr);

	/* PFR0 */
	switch (__SHIFTOUT(pfr0, ID_AA64PFR0_EL1_GIC)) {
	case ID_AA64PFR0_EL1_GIC_CPUIF_EN:
		aprint_normal(", GICv3");
		break;
	}
	switch (__SHIFTOUT(pfr0, ID_AA64PFR0_EL1_FP)) {
	case ID_AA64PFR0_EL1_FP_IMPL:
		aprint_normal(", FP");
		break;
	}

	/* ISAR0 */
	switch (__SHIFTOUT(isar0, ID_AA64ISAR0_EL1_CRC32)) {
	case ID_AA64ISAR0_EL1_CRC32_CRC32X:
		aprint_normal(", CRC32");
		break;
	}
	switch (__SHIFTOUT(isar0, ID_AA64ISAR0_EL1_SHA1)) {
	case ID_AA64ISAR0_EL1_SHA1_SHA1CPMHSU:
		aprint_normal(", SHA1");
		break;
	}
	switch (__SHIFTOUT(isar0, ID_AA64ISAR0_EL1_SHA2)) {
	case ID_AA64ISAR0_EL1_SHA2_SHA256HSU:
		aprint_normal(", SHA256");
		break;
	}
	switch (__SHIFTOUT(isar0, ID_AA64ISAR0_EL1_AES)) {
	case ID_AA64ISAR0_EL1_AES_AES:
		aprint_normal(", AES");
		break;
	case ID_AA64ISAR0_EL1_AES_PMUL:
		aprint_normal(", AES+PMULL");
		break;
	}


	/* PFR0:AdvSIMD */
	switch (__SHIFTOUT(pfr0, ID_AA64PFR0_EL1_ADVSIMD)) {
	case ID_AA64PFR0_EL1_ADV_SIMD_IMPL:
		aprint_normal(", NEON");
		break;
	}

	/* MVFR0/MVFR1 */
	switch (__SHIFTOUT(mvfr0, MVFR0_FPROUND)) {
	case MVFR0_FPROUND_ALL:
		aprint_normal(", rounding");
		break;
	}
	switch (__SHIFTOUT(mvfr0, MVFR0_FPTRAP)) {
	case MVFR0_FPTRAP_TRAP:
		aprint_normal(", exceptions");
		break;
	}
	switch (__SHIFTOUT(mvfr1, MVFR1_FPDNAN)) {
	case MVFR1_FPDNAN_NAN:
		aprint_normal(", NaN propagation");
		break;
	}
	switch (__SHIFTOUT(mvfr1, MVFR1_FPFTZ)) {
	case MVFR1_FPFTZ_DENORMAL:
		aprint_normal(", denormals");
		break;
	}
	switch (__SHIFTOUT(mvfr0, MVFR0_SIMDREG)) {
	case MVFR0_SIMDREG_16x64:
		aprint_normal(", 16x64bitRegs");
		break;
	case MVFR0_SIMDREG_32x64:
		aprint_normal(", 32x64bitRegs");
		break;
	}
	switch (__SHIFTOUT(mvfr1, MVFR1_SIMDFMAC)) {
	case MVFR1_SIMDFMAC_FMAC:
		aprint_normal(", Fused Multiply-Add");
		break;
	}

	aprint_normal("\n");
}
