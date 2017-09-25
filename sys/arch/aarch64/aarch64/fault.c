/*	$NetBSD$	*/

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
#include <sys/types.h>
#include <sys/proc.h>
#include <sys/siginfo.h>

#include <aarch64/frame.h>
#include <aarch64/machdep.h>
#include <aarch64/armreg.h>

const char * const fault_status_code[] = {
	[ESR_ISS_FSC_ADDRESS_SIZE_FAULT_0] = "Address Size Fault L0",
	[ESR_ISS_FSC_ADDRESS_SIZE_FAULT_1] = "Address Size Fault L1",
	[ESR_ISS_FSC_ADDRESS_SIZE_FAULT_2] = "Address Size Fault L2",
	[ESR_ISS_FSC_ADDRESS_SIZE_FAULT_3] = "Address Size Fault L3",
	[ESR_ISS_FSC_TRANSLATION_FAULT_0] = "Translation Fault L0",
	[ESR_ISS_FSC_TRANSLATION_FAULT_1] = "Translation Fault L1",
	[ESR_ISS_FSC_TRANSLATION_FAULT_2] = "Translation Fault L2",
	[ESR_ISS_FSC_TRANSLATION_FAULT_3] = "Translation Fault L3",
	[ESR_ISS_FSC_ACCESS_FAULT_0] = "Access Flag Fault L0",
	[ESR_ISS_FSC_ACCESS_FAULT_1] = "Access Flag Fault L1",
	[ESR_ISS_FSC_ACCESS_FAULT_2] = "Access Flag Fault L2",
	[ESR_ISS_FSC_ACCESS_FAULT_3] = "Access Flag Fault L3",
	[ESR_ISS_FSC_PERM_FAULT_0] = "Permission Fault L0",
	[ESR_ISS_FSC_PERM_FAULT_1] = "Permission Fault L1",
	[ESR_ISS_FSC_PERM_FAULT_2] = "Permission Fault L2",
	[ESR_ISS_FSC_PERM_FAULT_3] = "Permission Fault L3",
	[ESR_ISS_FSC_SYNC_EXTERNAL_ABORT]  =
	    "Synchronous External Abort",
	[ESR_ISS_FSC_SYNC_EXTERNAL_ABORT_TTWALK_0]  =
	    "Synchronous External Abort on translation table walk L0",
	[ESR_ISS_FSC_SYNC_EXTERNAL_ABORT_TTWALK_1]  =
	    "Synchronous External Abort on translation table walk L1",
	[ESR_ISS_FSC_SYNC_EXTERNAL_ABORT_TTWALK_2]  =
	    "Synchronous External Abort on translation table walk L2",
	[ESR_ISS_FSC_SYNC_EXTERNAL_ABORT_TTWALK_3]  =
	    "Synchronous External Abort on translation table walk L3",
	[ESR_ISS_FSC_SYNC_PARITY_ERROR] =
	    "Synchronous Parity error",
	[ESR_ISS_FSC_SYNC_PARITY_ERROR_ON_TTWALK_0] =
	    "Synchronous Parity error on translation table walk L0",
	[ESR_ISS_FSC_SYNC_PARITY_ERROR_ON_TTWALK_1] =
	    "Synchronous Parity error on translation table walk L1",
	[ESR_ISS_FSC_SYNC_PARITY_ERROR_ON_TTWALK_2] =
	    "Synchronous Parity error on translation table walk L2",
	[ESR_ISS_FSC_SYNC_PARITY_ERROR_ON_TTWALK_3] =
	    "Synchronous Parity error on translation table walk L3",
	[ESR_ISS_FSC_ALIGNMENT_FAULT] = "Alignment Fault",
	[ESR_ISS_FSC_TLB_CONFLICT_FAULT] = "TLB Conflict Fault",
	[ESR_ISS_FSC_LOCKDOWN_ABORT] = "Lockdown Abort",
	[ESR_ISS_FSC_UNSUPPORTED_EXCLUSIVE] = "Unsupported exclusive",
	[ESR_ISS_FSC_FIRST_LEVEL_DOMAIN_FAULT] =
	    "First Level Domain Fault",
	[ESR_ISS_FSC_SECOND_LEVEL_DOMAIN_FAULT] =
	    "Second Level Domain Fault",
};

static bool
is_fatal_abort(uint32_t esr)
{
	uint32_t fsc;

	fsc = __SHIFTOUT(esr, ESR_ISS_DATAABORT_DFSC);

	switch (fsc) {
	case ESR_ISS_FSC_TRANSLATION_FAULT_0:
	case ESR_ISS_FSC_TRANSLATION_FAULT_1:
	case ESR_ISS_FSC_TRANSLATION_FAULT_2:
	case ESR_ISS_FSC_TRANSLATION_FAULT_3:
	case ESR_ISS_FSC_ACCESS_FAULT_0:
	case ESR_ISS_FSC_ACCESS_FAULT_1:
	case ESR_ISS_FSC_ACCESS_FAULT_2:
	case ESR_ISS_FSC_ACCESS_FAULT_3:
	case ESR_ISS_FSC_PERM_FAULT_0:
	case ESR_ISS_FSC_PERM_FAULT_1:
	case ESR_ISS_FSC_PERM_FAULT_2:
	case ESR_ISS_FSC_PERM_FAULT_3:
		return false;
	}
	return true;
}

bool
data_abort_handler(struct trapframe *tf, ksiginfo_t *ksi, const char *trapname)
{
	const uint32_t esr = tf->tf_esr;

	if (is_fatal_abort(esr)) {
		uint32_t fsc, rw;
		const char *faultstr;

		fsc = __SHIFTOUT(esr, ESR_ISS_DATAABORT_DFSC); /* also IFSC */
		if (ksi != NULL) {
			/*
			 * fatal abort in usermode
			 */
			trap_ksi_init(ksi, SIGBUS, BUS_ADRALN, tf->tf_far, fsc);
			return false;
		}

		/*
		 * fatal abort in kernel
		 */
		printf("Trap: %s:", trapname);

		if ((fsc >= __arraycount(fault_status_code)) ||
		    ((faultstr = fault_status_code[fsc]) == NULL))
			printf(" unknown fault status 0x%x ", fsc);
		else
			printf(" %s", faultstr);

		rw = __SHIFTOUT(esr, ESR_ISS_DATAABORT_WnR); /* 0 if IFSC */
		if (__SHIFTOUT(esr, ESR_EC) == ESR_EC_DATA_ABT_EL1)
			printf(" with %s access", (rw == 0) ? "read" : "write");

		if (__SHIFTOUT(esr, ESR_ISS_DATAABORT_EA) != 0)
			printf(", External abort");

		if (__SHIFTOUT(esr, ESR_ISS_DATAABORT_S1PTW) != 0)
			printf(", State 2 Fault");

		printf("\n      pc=%016llx sp=%016llx far=%016llx\n",
		    tf->tf_pc, tf->tf_sp, tf->tf_far);

		return false;
	}

//XXXAARCH64
#if 0
	return pagefault(tf, ksi);
#else
	if (ksi == NULL)
		panic("pagefault in kernel");
	else
		panic("pagefault in usermode");
#endif

	return false;
}
