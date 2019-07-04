/*	$NetBSD$	*/

/*
 * Copyright (c) 2018 Ryo Shimizu <ryo@nerv.org>
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
 * Qualcomm Snapdragon family SoC UART/UART_DM driver
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include "locators.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/poll.h>
#ifdef RND_COM
#include <sys/rndsource.h>
#endif
#include <sys/tty.h>
#include <dev/cons.h>
#include <dev/fdt/fdtvar.h>
#include <arm/snapdragon/msmuartreg.h>

int msmuart_match(device_t, cfdata_t, void *);
void msmuart_attach(device_t, device_t, void *);
//int msmuart_intr(void *);
//void msmuart_rxsoft(void *);
//int msmuart_cngetc(dev_t);
//void msmuart_cnputc(dev_t, int);
//void msmuart_cnpollc(dev_t, int);
//void msmuart_start(struct tty *);
//int msmuart_param(struct tty *, struct termios *);

static const char * const compatible[] = {
	"qcom,msm-uartdm-v1.4",
	"qcom,msm-uartdm",
	NULL
};

struct msmuart_softc {
	device_t sc_dev;
	int sc_unit;

	bus_space_tag_t sc_bst;
	bus_space_handle_t sc_bsh;

	struct clk *sc_clk;
	struct fdtbus_reset *sc_rst;
};

CFATTACH_DECL_NEW(msmuart, sizeof(struct msmuart_softc),
	msmuart_match, msmuart_attach, NULL, NULL);

int
msmuart_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;
	return of_match_compatible(faa->faa_phandle, compatible);
}

void
msmuart_attach(device_t parent, device_t self, void *aux)
{
	struct msmuart_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	bus_addr_t addr;
	bus_size_t size;
	char intrstr[128];

	sc->sc_dev = self;

	sc->sc_bst = faa->faa_bst;
	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}

	if (bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}

	if (!fdtbus_intr_str(phandle, 0, intrstr, sizeof(intrstr))) {
		aprint_error(": failed to decode interrupt\n");
		return;
	}

	if ((sc->sc_clk = fdtbus_clock_get_index(phandle, 0)) != NULL) {
		if (clk_enable(sc->sc_clk) != 0) {
			aprint_error(": couldn't enable clock\n");
			return;
		}
	}

	if ((sc->sc_rst = fdtbus_reset_get_index(phandle, 0)) != NULL) {
		if (fdtbus_reset_deassert(sc->sc_rst) != 0) {
			aprint_error(": couldn't de-assert reset\n");
			return;
		}
	}
}

static int
msmuart_console_match(int phandle)
{
	return of_match_compatible(phandle, compatible);
}


//#define XXXCNINIT

#ifdef XXXCNINIT
void snapdragon_platform_early_putchar(char);
void snapdragon_platform_early_puts(const char *);	/* debug */

static void
xxxputc(dev_t dev, int c)
{
	snapdragon_platform_early_putchar(c);
}

static struct consdev xxxconsole = {
	NULL, NULL, NULL, xxxputc, NULL, NULL,
	NULL, NULL, NODEV, CN_NORMAL
};
#endif

static void
msmuart_console_consinit(struct fdt_attach_args *faa, u_int uart_freq)
{
#ifdef XXXCNINIT
	cn_tab = &xxxconsole;
#else

	const int phandle = faa->faa_phandle;
	bus_space_tag_t bst = faa->faa_a4x_bst;
	bus_addr_t addr;
	tcflag_t flags;
	int speed;

	fdtbus_get_reg(phandle, 0, &addr, NULL);
	speed = fdtbus_get_stdout_speed();
	if (speed < 0)
		speed = 115200;	/* default */
	flags = fdtbus_get_stdout_flags();

	(void)&bst;
	(void)&flags;

//	if (comcnattach(bst, addr, speed, uart_freq, COM_TYPE_SUNXI, flags))
//		panic("Cannot initialize sunxi com console");
#endif
}

static const struct fdt_console msmuart_console = {
	.match = msmuart_console_match,
	.consinit = msmuart_console_consinit
};

FDT_CONSOLE(msmuart, &msmuart_console);
