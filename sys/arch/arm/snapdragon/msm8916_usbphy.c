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

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/intr.h>
#include <sys/systm.h>
#include <sys/time.h>

#include <dev/fdt/fdtvar.h>

static int msm8916_usbphy_match(device_t, cfdata_t, void *);
static void msm8916_usbphy_attach(device_t, device_t, void *);
static void *msm8916_usbphy_acquire(device_t, const void *, size_t);
static void msm8916_usbphy_release(device_t, void *);
static int msm8916_usbphy_enable(device_t, void *, bool);

struct msm8916_usbphy_softc {
	device_t sc_dev;
	bus_space_tag_t sc_bst;
	bus_space_handle_t sc_bsh;
	int sc_phandle;
	struct clk *sc_clk;
	struct fdtbus_reset *sc_rst;
};

static const char *compatible[] = {
	"qcom,usb-hs-phy-msm8916",
	NULL
};

const struct fdtbus_phy_controller_func msm8916_usbphy_funcs = {
	.acquire = msm8916_usbphy_acquire,
	.release = msm8916_usbphy_release,
	.enable = msm8916_usbphy_enable,
};

CFATTACH_DECL_NEW(msm8916_usbphy, sizeof(struct msm8916_usbphy_softc),
	msm8916_usbphy_match, msm8916_usbphy_attach, NULL, NULL);

static int
msm8916_usbphy_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;
	return of_match_compatible(faa->faa_phandle, compatible);
}

static void
msm8916_usbphy_attach(device_t parent, device_t self, void *aux)
{
	struct msm8916_usbphy_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	bus_addr_t addr;
	bus_size_t size;

	sc->sc_dev = self;

	sc->sc_phandle = faa->faa_phandle;
	sc->sc_bst = faa->faa_bst;
	if (fdtbus_get_reg(sc->sc_phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}
	if (bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}

	sc->sc_clk = fdtbus_clock_get_index(sc->sc_phandle, 0);
	if (sc->sc_clk == NULL) {
		aprint_error(": couldn't get clock\n");
		return;
	}
	sc->sc_rst = fdtbus_reset_get_index(sc->sc_phandle, 0);

	if (sc->sc_rst != NULL) {
		if (fdtbus_reset_deassert(sc->sc_rst) != 0) {
			aprint_error(": couldn't de-assert reset\n");
			return;
		}
	}
	if (clk_enable(sc->sc_clk) != 0) {
		aprint_error(": couldn't enable clock\n");
		return;
	}

	aprint_naive("\n");
	aprint_normal(": USB2 PHY\n");

	fdtbus_register_phy_controller(self, sc->sc_phandle, &msm8916_usbphy_funcs);
}

static void *
msm8916_usbphy_acquire(device_t dev, const void *data, size_t len)
{
	if (len != 0)
		return NULL;

	return (void *)(uintptr_t)1;
}

static void
msm8916_usbphy_release(device_t dev, void *priv)
{
}

static int
msm8916_usbphy_enable(device_t dev, void *priv, bool enable)
{
	return 0;
}
