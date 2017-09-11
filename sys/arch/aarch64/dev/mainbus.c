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
#include "a64gic.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>

#include <aarch64/autoconf.h>

static int mainbus_match(device_t, cfdata_t, void *);
static void mainbus_attach(device_t, device_t, void *);
static int mainbus_search(device_t, cfdata_t, const int *, void *);
#if NA64GIC == 0
static int mainbus_early_search(device_t, cfdata_t, const int *, void *);
#endif
static int mainbus_print(void *, const char *);

CFATTACH_DECL_NEW(mainbus, 0,
    mainbus_match, mainbus_attach, NULL, NULL);

static int
mainbus_match(device_t parent, cfdata_t cf, void *aux)
{
	return 1;
}

static void
mainbus_attach(device_t parent, device_t self, void *aux)
{
	struct mainbus_attach_args mba = {
	    .mba_addr = MAINBUSCF_ADDR_DEFAULT,
	    .mba_size = MAINBUSCF_SIZE_DEFAULT,
	    .mba_intr = MAINBUSCF_INTR_DEFAULT,
	    .mba_unit = 0
	};

	aprint_naive("\n");
	aprint_normal("\n");

	/* attach cpunode first */
	mba.mba_name = "cpunode";
	config_found(self, &mba, mainbus_print);

	/*
	 * Need to attach interrupt controller at the first.
	 * if a64gic is not exists, attach other bus at mainbus includes
	 * interrupt controller, and attach mainbus's devices again.
	 */
#if NA64GIC > 0
	mba.mba_name = "a64gic";
	config_found(self, &mba, mainbus_print);

	config_search_ia(mainbus_search, self, "mainbus", &mba);
#else
	config_search_ia(mainbus_early_search, self, "mainbus", &mba);
	config_search_ia(mainbus_search, self, "mainbus", &mba);
#endif
}

#if NA64GIC == 0
static int
mainbus_early_search(device_t parent, cfdata_t cf, const int *ldesc, void *aux)
{
	struct mainbus_attach_args *mba = aux;

	/* a64gtmr require interrupt controller. attach later */
	if (strcmp(cf->cf_name, "a64gtmr") == 0)
		return 0;

	mba->mba_name = cf->cf_name;
	mba->mba_addr = cf->cf_loc[MAINBUSCF_ADDR];
	mba->mba_size = cf->cf_loc[MAINBUSCF_SIZE];
	mba->mba_intr = cf->cf_loc[MAINBUSCF_INTR];

	if (config_match(parent, cf, aux) > 0)
		config_attach(parent, cf, aux, mainbus_print);

	return 0;
}
#endif /* NA64GIC == 0 */

static int
mainbus_search(device_t parent, cfdata_t cf, const int *ldesc, void *aux)
{
	struct mainbus_attach_args *mba = aux;

	mba->mba_name = cf->cf_name;
	mba->mba_addr = cf->cf_loc[MAINBUSCF_ADDR];
	mba->mba_size = cf->cf_loc[MAINBUSCF_SIZE];
	mba->mba_intr = cf->cf_loc[MAINBUSCF_INTR];

	if (config_match(parent, cf, aux) > 0)
		config_attach(parent, cf, aux, mainbus_print);

	return 0;
}

static int
mainbus_print(void *aux, const char *mainbus)
{
	return UNCONF;
}
