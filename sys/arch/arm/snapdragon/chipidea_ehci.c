/*	$NetBSD$	*/

/*
 * Copyright (c) 2019 Ryo Shimizu <ryo@nerv.org>
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

/*
 * ChipIdea Highspeed Dual Role USB Controller driver
 */
#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/intr.h>
#include <sys/systm.h>
#include <sys/kernel.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdivar.h>
#include <dev/usb/usb_mem.h>
#include <dev/usb/ehcireg.h>
#include <dev/usb/ehcivar.h>

#include <dev/fdt/fdtvar.h>
#include <arm/snapdragon/chipideareg.h>

static int chipidea_fdt_match(device_t, cfdata_t, void *);
static void chipidea_fdt_attach(device_t, device_t, void *);

struct chipidea_softc {
	struct ehci_softc sc_ehci;

	bus_space_tag_t sc_bst;
	bus_space_handle_t sc_bsh;
	bus_dma_tag_t sc_dmat;
	int sc_phandle;

};

CFATTACH_DECL2_NEW(chipidea_fdt, sizeof(struct chipidea_softc),
	chipidea_fdt_match, chipidea_fdt_attach, NULL,
	ehci_activate, NULL, ehci_childdet);

static int
chipidea_fdt_match(device_t parent, cfdata_t cf, void *aux)
{
	static const struct device_compatible_entry compatible[] = {
		{ .compat = "qcom,ci-hdrc" },
		DEVICE_COMPAT_EOL
	};
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compatible);
}

static void
chipidea_fdt_attach(device_t parent, device_t self, void *aux)
{
	struct chipidea_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	struct fdtbus_reset *rst;
	struct fdtbus_phy *phy;
	struct clk *clk;
	char intrstr[128];
	bus_addr_t addr;
	bus_size_t size;
	int error;
	void *ih;
	u_int n;

	sc->sc_bst = faa->faa_bst;
	sc->sc_dmat = faa->faa_dmat;
	sc->sc_phandle = phandle;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}

	/* Enable clocks */
	for (n = 0; (clk = fdtbus_clock_get_index(phandle, n)) != NULL; n++) {
		if (clk_enable(clk) != 0) {
			aprint_error(": couldn't enable clock #%d\n", n);
			return;
		}
	}
	/* De-assert resets */
	for (n = 0; (rst = fdtbus_reset_get_index(phandle, n)) != NULL; n++) {
		if (fdtbus_reset_deassert(rst) != 0) {
			aprint_error(": couldn't de-assert reset #%d\n", n);
			return;
		}
	}

	/* Enable optional phy */
	phy = fdtbus_phy_get(phandle, "usb");
	if (phy && fdtbus_phy_enable(phy, true) != 0) {
		aprint_error(": couldn't enable phy\n");
		return;
	}

	sc->sc_ehci.sc_dev = self;
	sc->sc_ehci.sc_bus.ub_hcpriv = sc;
	sc->sc_ehci.sc_bus.ub_dmatag = sc->sc_dmat;
	sc->sc_ehci.sc_bus.ub_revision = USBREV_2_0;
	sc->sc_ehci.sc_ncomp = 0;
	sc->sc_ehci.sc_size = 0x100;
	sc->sc_ehci.iot = sc->sc_bst;

	if (bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}

	if (bus_space_subregion(sc->sc_bst, sc->sc_bsh,
	    CHIPIDEA_REG_CAPLENGTH, 0x100, &sc->sc_ehci.ioh) != 0) {
		aprint_error(": couldn't map subregion\n");
		return;
	}

	uint32_t id = bus_space_read_4(sc->sc_bst, sc->sc_bsh, CHIPIDEA_REG_ID);

	aprint_naive("\n");
	aprint_normal(": ChipIdea Highspeed Dual Role USB Controller, ID=0x%08x\n", id);

	/* Disable interrupts */
	sc->sc_ehci.sc_offs = EREAD1(&sc->sc_ehci, EHCI_CAPLENGTH);
	EOWRITE4(&sc->sc_ehci, EHCI_USBINTR, 0);

	if (!fdtbus_intr_str(phandle, 0, intrstr, sizeof(intrstr))) {
		aprint_error_dev(self, "failed to decode interrupt\n");
		return;
	}

	ih = fdtbus_intr_establish(phandle, 0, IPL_USB, FDT_INTR_MPSAFE,
	    ehci_intr, &sc->sc_ehci);
	if (ih == NULL) {
		aprint_error_dev(self, "couldn't establish interrupt on %s\n",
		    intrstr);
		return;
	}
	aprint_normal_dev(self, "interrupting on %s\n", intrstr);

	error = ehci_init(&sc->sc_ehci);
	if (error) {
		aprint_error_dev(self, "init failed, error = %d\n", error);
		return;
	}

	pmf_device_register1(self, NULL, NULL, ehci_shutdown);

	sc->sc_ehci.sc_child = config_found(self, &sc->sc_ehci.sc_bus, usbctlprint);
}
