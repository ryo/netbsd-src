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

#include "opt_console.h"
#include "opt_soc.h"
#include "opt_multiprocessor.h"

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <uvm/uvm_extern.h>

#include <machine/bootconfig.h>

#include <arm/cpufunc.h>
#include <arm/cortex/gtmr_var.h>

#include <arm/snapdragon/apq8016reg.h>
#include <arm/snapdragon/msmuartreg.h>
#include <arm/snapdragon/snapdragon_platform.h>

#include <arm/fdt/arm_fdtvar.h>
#include <dev/fdt/fdtvar.h>
#include <libfdt.h>

void snapdragon_platform_early_putchar(char);

extern struct arm32_bus_dma_tag arm_generic_dma_tag;
extern struct bus_space arm_generic_bs_tag;
extern struct bus_space arm_generic_a4x_bs_tag;
#define snapdragon_dma_tag	arm_generic_dma_tag
#define snapdragon_bs_tag	arm_generic_bs_tag
#define snapdragon_a4x_bs_tag	arm_generic_a4x_bs_tag

static const struct pmap_devmap *
snapdragon_platform_devmap(void)
{
	static const struct pmap_devmap devmap[] = {
		DEVMAP_ENTRY(SNAPDRAGON_SYS_VBASE,
		    SNAPDRAGON_SYS_PBASE,
		    SNAPDRAGON_SYS_SIZE),
		DEVMAP_ENTRY_END
	};

	return devmap;
}

static void
snapdragon_platform_bootstrap(void)
{
	void *fdt_data = __UNCONST(fdtbus_get_data());
	const int chosen_off = fdt_path_offset(fdt_data, "/chosen");
	if (chosen_off < 0)
		return;

	if (match_bootconf_option(boot_args, "console", "fb")) {
		const int framebuffer_off =
		    fdt_path_offset(fdt_data, "/chosen/framebuffer");
		if (framebuffer_off >= 0) {
			const char *status = fdt_getprop(fdt_data,
			    framebuffer_off, "status", NULL);
			if (status == NULL || strncmp(status, "ok", 2) == 0) {
				fdt_setprop_string(fdt_data, chosen_off,
				    "stdout-path", "/chosen/framebuffer");
			}
		}
	} else if (match_bootconf_option(boot_args, "console", "serial")) {
		fdt_setprop_string(fdt_data, chosen_off,
		    "stdout-path", "serial0:115200n8");
	}
}

static void
snapdragon_platform_init_attach_args(struct fdt_attach_args *faa)
{
	faa->faa_bst = &snapdragon_bs_tag;
	faa->faa_dmat = &snapdragon_dma_tag;
}

static void
snapdragon_platform_device_register(device_t dev, void *aux)
{
}

static void
snapdragon_platform_system_reset(void)
{
	bus_space_tag_t bst = &snapdragon_bs_tag;
	bus_space_handle_t bsh;

#define QCOM_PSHOLD	0x0004ab000	/* XXX: "qcom,pshold" */
	bus_space_map(bst, QCOM_PSHOLD, 0x1000, 0, &bsh);
	bus_space_write_4(bst, bsh, 0, 0);

	for (;;) {
		__asm("wfi");
	}

}

static u_int
snapdragon_platform_uart_freq(void)
{
	return 0;
}

void
snapdragon_platform_early_putchar(char c)
{
#ifdef CONSADDR
#define CONSADDR_VA	(CONSADDR - SNAPDRAGON_SYS_PBASE + SNAPDRAGON_SYS_VBASE)

#define REGWRITE(base, reg, val)	\
	(*(volatile uint32_t *)((volatile char *)(base) + (reg)) = (val))
#define REGREAD(base, reg)		\
	(*(volatile uint32_t *)((volatile char *)(base) + (reg)))

	volatile uint32_t *uart = cpu_earlydevice_va_p() ?
	    (volatile uint32_t *)CONSADDR_VA :
	    (volatile uint32_t *)CONSADDR;

	/*
	 * XXX: suppoted only UART_DM. non DM UART is not considered.
	 */

	while (((REGREAD(uart, UART_DM_SR) & UART_DM_SR_TXEMT) == 0) &&
	    ((REGREAD(uart, UART_DM_ISR) & UART_DM_IRQBIT_TX_READY) == 0))
		;
	REGWRITE(uart, UART_DM_CR, UART_DM_CR_GEN_CMD_RST_TX_RDY);

	REGWRITE(uart, UART_DM_NO_CHARS_FOR_TX, 1);	/* just one character */
	(void)REGREAD(uart, UART_DM_NO_CHARS_FOR_TX);

	/* wait TXRDY */
	while ((REGREAD(uart, UART_DM_SR) & UART_DM_SR_TXRDY) == 0)
		;

	/* put */
	REGWRITE(uart, UART_DM_TF, c);

	/* wait TXRDY */
	while ((REGREAD(uart, UART_DM_SR) & UART_DM_SR_TXRDY) == 0)
		;
#endif /* CONADDR */
}

#if defined(SOC_APQ8016)

static const struct arm_platform apq8016_platform = {
	.ap_devmap = snapdragon_platform_devmap,
	.ap_bootstrap = snapdragon_platform_bootstrap,
	.ap_init_attach_args = snapdragon_platform_init_attach_args,
	.ap_device_register = snapdragon_platform_device_register,
	.ap_reset = snapdragon_platform_system_reset,
	.ap_delay = gtmr_delay,
	.ap_uart_freq = snapdragon_platform_uart_freq,
};

ARM_PLATFORM(apq8016, "qcom,apq8016-sbc", &apq8016_platform);
#endif
