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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>

#include <aarch64/armreg.h>
#include <aarch64/cpuvar.h>

struct cpunode_softc {
	device_t sc_dev;
};

static int cpunode_match(device_t, cfdata_t, void *);
static void cpunode_attach(device_t, device_t, void *);
static int cpunode_print(void *, const char *);

CFATTACH_DECL_NEW(cpunode_mainbus, 0,
    cpunode_match, cpunode_attach, NULL, NULL);

static int
cpunode_match(device_t parent, cfdata_t cf, void *aux)
{
	return 1;
}

static void
cpunode_attach(device_t parent, device_t self, void *aux)
{
	struct cpunode_softc * const sc = device_private(self);
	int cpunum, ncores;

	sc->sc_dev = self;

	/*
	 * XXXAARCH64:
	 *  Not all CPU had L2CTLR register.
	 *  get from FDT or else.
	 */
	ncores = __SHIFTOUT(reg_l2ctlr_el1_read(), L2CTLR_NUMOFCORE) + 1;

	aprint_naive("\n");
	aprint_normal("\n");

	for (cpunum = 0; cpunum < ncores; cpunum++) {
		struct cpu_attach_args ccaa = {
			.caa_name = "cpu",
			.caa_cpucore = cpunum
		};
		config_found(self, &ccaa, cpunode_print);
	}
}

static int
cpunode_print(void *aux, const char *cpunode)
{
	struct cpu_attach_args * const caa = aux;

	if (cpunode != NULL)
		aprint_normal("%s", cpunode);

	if (caa->caa_cpucore != CPUNODECF_CORE_DEFAULT)
		aprint_normal("");

	return UNCONF;
}
