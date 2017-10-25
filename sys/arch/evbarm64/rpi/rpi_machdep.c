/*	$NetBSD$	*/

/*-
 * Copyright (c) 2012,2017 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Nick Hudson, and Ryo Shimizu.
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

#include "opt_arm_debug.h"
#include "opt_bcm283x.h"
#include "opt_rpi.h"
#include "opt_vcprop.h"
#include "opt_ddb.h"
#include "opt_kgdb.h"
#include "opt_rpi.h"
#include "opt_vcprop.h"

#include "sdhc.h"
#include "bcmsdhost.h"
#include "bcmdwctwo.h"
#include "bcmspi.h"
#include "bsciic.h"
#include "plcom.h"
#include "com.h"
#include "genfb.h"
#include "usb.h"
#include "ukbd.h"

#include <sys/param.h>
#include <sys/device.h>
#include <sys/termios.h>
#include <sys/bus.h>
#include <sys/systm.h>
#include <sys/reboot.h>

#include <net/if_ether.h>
#include <prop/proplib.h>

#include <dev/cons.h>
#include <uvm/uvm_extern.h>

#include <aarch64/bootconfig.h>
#include <aarch64/cpu.h>
#include <aarch64/frame.h>
#include <aarch64/machdep.h>
#include <aarch64/armreg.h>
#include <aarch64/cpufunc.h>
#include <aarch64/pmap.h>

#include <evbarm64/rpi/rpi.h>

#include <evbarm/rpi/vcio.h>
#include <evbarm/rpi/vcpm.h>
#include <evbarm/rpi/vcprop.h>

#include <machine/autoconf.h>

#include <aarch64/broadcom/bcm2835var.h>
#include <arm/broadcom/bcm2835_pmvar.h>
#include <arm/broadcom/bcm2835_mbox.h>
#include <arm/broadcom/bcm2835_gpio_subr.h>
#include <arm/broadcom/bcm_amba.h>

#if NPLCOM > 0
#include <evbarm/dev/plcomreg.h>
#include <evbarm/dev/plcomvar.h>
#endif

#if NCOM > 0
#include <dev/ic/comvar.h>
#endif

#if NGENFB > 0
#include <dev/videomode/videomode.h>
#include <dev/videomode/edidvar.h>
#include <dev/wscons/wsconsio.h>
#endif

#if NUKBD > 0
#include <dev/usb/ukbdvar.h>
#endif

void initarm(void);
static bool rpi_rev_has_btwifi(uint32_t);
static void rpi_uartinit(void);
static void rpi_bootparams(void);
static void rpi_device_register(device_t, void *);

#define EARLY_CONSOLE

#ifdef EARLY_CONSOLE
static void konsinit(void);
#endif

uint64_t uboot_args[4] = { 0 };	/* filled in by rpi_start.S (not in bss) */
BootConfig bootconfig;

int uart_clk = BCM2835_UART0_CLK;

#define PLCONADDR BCM2835_UART0_BASE

#ifndef CONSDEVNAME
#define CONSDEVNAME "plcom"
#endif

#ifndef PLCONSPEED
#define PLCONSPEED B115200
#endif
#ifndef PLCONMODE
#define PLCONMODE ((TTYDEF_CFLAG & ~(CSIZE | CSTOPB | PARENB)) | CS8) /* 8N1 */
#endif
#ifndef PLCOMCNUNIT
#define PLCOMCNUNIT -1
#endif

#if (NPLCOM > 0)
static const bus_addr_t consaddr = (bus_addr_t)PLCONADDR;

int plcomcnspeed = PLCONSPEED;
int plcomcnmode = PLCONMODE;
#endif

#if (NPLCOM > 0 && (defined(PLCONSOLE) || defined(KGDB)))
static struct plcom_instance rpi_pi = {
	.pi_type = PLCOM_TYPE_PL011,
	.pi_flags = PLC_FLAG_32BIT_ACCESS,
	.pi_iot = &bcm2835_bs_tag,
	.pi_size = BCM2835_UART0_SIZE
};
#endif

/* Smallest amount of RAM start.elf could give us. */
#define RPI_MINIMUM_SPLIT	(128U * 1024 * 1024)

static struct __aligned(64) {	/* should be cacheLineSize*N aligned */
	struct vcprop_buffer_hdr	vb_hdr;
	struct vcprop_tag_clockrate	vbt_uartclockrate;
	struct vcprop_tag_clockrate	vbt_coreclockrate;
	struct vcprop_tag_boardrev	vbt_boardrev;
	struct vcprop_tag end;
} vb_uart = {
	.vb_hdr = {
		.vpb_len = sizeof(vb_uart),
		.vpb_rcode = VCPROP_PROCESS_REQUEST,
	},
	.vbt_uartclockrate = {
		.tag = {
			.vpt_tag = VCPROPTAG_GET_CLOCKRATE,
			.vpt_len = VCPROPTAG_LEN(vb_uart.vbt_uartclockrate),
			.vpt_rcode = VCPROPTAG_REQUEST
		},
		.id = VCPROP_CLK_UART
	},
	.vbt_coreclockrate = {
		.tag = {
			.vpt_tag = VCPROPTAG_GET_CLOCKRATE,
			.vpt_len = VCPROPTAG_LEN(vb_uart.vbt_coreclockrate),
			.vpt_rcode = VCPROPTAG_REQUEST
		},
		.id = VCPROP_CLK_CORE
	},
	.vbt_boardrev = {
		.tag = {
			.vpt_tag = VCPROPTAG_GET_BOARDREVISION,
			.vpt_len = VCPROPTAG_LEN(vb_uart.vbt_boardrev),
			.vpt_rcode = VCPROPTAG_REQUEST
		},
	},
	.end = {
		.vpt_tag = VCPROPTAG_NULL
	}
};

static struct __aligned(16) {	/* should be cacheLineSize*N aligned */
	struct vcprop_buffer_hdr	vb_hdr;
	struct vcprop_tag_fwrev		vbt_fwrev;
	struct vcprop_tag_boardmodel	vbt_boardmodel;
	struct vcprop_tag_boardrev	vbt_boardrev;
	struct vcprop_tag_macaddr	vbt_macaddr;
	struct vcprop_tag_memory	vbt_memory;
	struct vcprop_tag_boardserial	vbt_serial;
	struct vcprop_tag_dmachan	vbt_dmachan;
	struct vcprop_tag_cmdline	vbt_cmdline;
	struct vcprop_tag_clockrate	vbt_emmcclockrate;
	struct vcprop_tag_clockrate	vbt_armclockrate;
	struct vcprop_tag_clockrate	vbt_coreclockrate;
	struct vcprop_tag end;
} vb = {
	.vb_hdr = {
		.vpb_len = sizeof(vb),
		.vpb_rcode = VCPROP_PROCESS_REQUEST,
	},
	.vbt_fwrev = {
		.tag = {
			.vpt_tag = VCPROPTAG_GET_FIRMWAREREV,
			.vpt_len = VCPROPTAG_LEN(vb.vbt_fwrev),
			.vpt_rcode = VCPROPTAG_REQUEST
		},
	},
	.vbt_boardmodel = {
		.tag = {
			.vpt_tag = VCPROPTAG_GET_BOARDMODEL,
			.vpt_len = VCPROPTAG_LEN(vb.vbt_boardmodel),
			.vpt_rcode = VCPROPTAG_REQUEST
		},
	},
	.vbt_boardrev = {
		.tag = {
			.vpt_tag = VCPROPTAG_GET_BOARDREVISION,
			.vpt_len = VCPROPTAG_LEN(vb.vbt_boardrev),
			.vpt_rcode = VCPROPTAG_REQUEST
		},
	},
	.vbt_macaddr = {
		.tag = {
			.vpt_tag = VCPROPTAG_GET_MACADDRESS,
			.vpt_len = VCPROPTAG_LEN(vb.vbt_macaddr),
			.vpt_rcode = VCPROPTAG_REQUEST
		},
	},
	.vbt_memory = {
		.tag = {
			.vpt_tag = VCPROPTAG_GET_ARMMEMORY,
			.vpt_len = VCPROPTAG_LEN(vb.vbt_memory),
			.vpt_rcode = VCPROPTAG_REQUEST
		},
	},
	.vbt_serial = {
		.tag = {
			.vpt_tag = VCPROPTAG_GET_BOARDSERIAL,
			.vpt_len = VCPROPTAG_LEN(vb.vbt_serial),
			.vpt_rcode = VCPROPTAG_REQUEST
		},
	},
	.vbt_dmachan = {
		.tag = {
			.vpt_tag = VCPROPTAG_GET_DMACHAN,
			.vpt_len = VCPROPTAG_LEN(vb.vbt_dmachan),
			.vpt_rcode = VCPROPTAG_REQUEST
		},
	},
	.vbt_cmdline = {
		.tag = {
			.vpt_tag = VCPROPTAG_GET_CMDLINE,
			.vpt_len = VCPROPTAG_LEN(vb.vbt_cmdline),
			.vpt_rcode = VCPROPTAG_REQUEST
		},
	},
	.vbt_emmcclockrate = {
		.tag = {
			.vpt_tag = VCPROPTAG_GET_CLOCKRATE,
			.vpt_len = VCPROPTAG_LEN(vb.vbt_emmcclockrate),
			.vpt_rcode = VCPROPTAG_REQUEST
		},
		.id = VCPROP_CLK_EMMC
	},
	.vbt_armclockrate = {
		.tag = {
			.vpt_tag = VCPROPTAG_GET_CLOCKRATE,
			.vpt_len = VCPROPTAG_LEN(vb.vbt_armclockrate),
			.vpt_rcode = VCPROPTAG_REQUEST
		},
		.id = VCPROP_CLK_ARM
	},
	.vbt_coreclockrate = {
		.tag = {
			.vpt_tag = VCPROPTAG_GET_CLOCKRATE,
			.vpt_len = VCPROPTAG_LEN(vb.vbt_coreclockrate),
			.vpt_rcode = VCPROPTAG_REQUEST
		},
		.id = VCPROP_CLK_CORE
	},
	.end = {
		.vpt_tag = VCPROPTAG_NULL
	}
};

static const struct pmap_devmap rpi_devmap[] = {
	{
		.pd_va = DEVMAP_TRUNC_ADDR(RPI_DEVMAP_VBASE + 0),
		.pd_pa = DEVMAP_TRUNC_ADDR(RPI_KERNEL_IO_PBASE),
		.pd_size = DEVMAP_ROUND_SIZE(RPI_KERNEL_IO_VSIZE), /* 16Mb */
		.pd_prot = VM_PROT_READ|VM_PROT_WRITE,
		.pd_flags = PMAP_NOCACHE
	},
#if defined(BCM2836)
	{
		.pd_va =
		    DEVMAP_TRUNC_ADDR(RPI_DEVMAP_VBASE + RPI_KERNEL_IO_VSIZE),
		.pd_pa = DEVMAP_TRUNC_ADDR(RPI_KERNEL_LOCAL_PBASE),
		.pd_size = DEVMAP_ROUND_SIZE(RPI_KERNEL_LOCAL_VSIZE),
		.pd_prot = VM_PROT_READ|VM_PROT_WRITE,
		.pd_flags = PMAP_NOCACHE
	},
#endif
	{ 0 }
};

/* cpu reset hook for early boot */
static void
raspi_early_reset(void)
{
#define BCM2835_WDOG_BASE		0x3f100000
#define  BCM2835_WDOG_RSTC_REG 7	/* 0x3f10001c */
#define  BCM2835_WDOG_RSTS_REG 8	/* 0x3f100020 */
#define  BCM2835_WDOG_WDOG_REG 9	/* 0x3f100024 */
#define  BCM2835_WDOG_MAGIC	0x5a000000
	volatile uint32_t *wdog = (volatile uint32_t *)BCM2835_WDOG_BASE;
	uint32_t v;

	v = wdog[BCM2835_WDOG_RSTC_REG];
	v &= ~0x30;
	v |= 0x20;
	wdog[BCM2835_WDOG_WDOG_REG] = BCM2835_WDOG_MAGIC | 50;
	wdog[BCM2835_WDOG_RSTC_REG] = BCM2835_WDOG_MAGIC | v;
}

void
initarm(void)
{
	cpu_reset_address0 = raspi_early_reset;	/* reset before attach devs */

	/* XXXAARCH64: DEBUG */
	boothowto = RB_SINGLE;


#ifdef EARLY_CONSOLE
	konsinit();	/* early console before consinit() */
#endif

	evbarm64_device_register = rpi_device_register;

	curcpu()->ci_data.cpu_cc_freq = 600 * 1000 * 1000;	/* default */

	/* map some peripheral registers */
	pmap_devmap_bootstrap(rpi_devmap);

#if defined(EARLY_CONSOLE) && defined(VERBOSE_INIT_ARM)
	delay(10000);	/* waiting for uart TX fifo empty before uartinit */
#endif
	rpi_uartinit();

	consinit();

#if 1
	/* XXXAARCH64: DEBUG */
	{
		static struct cnm_state cnm_state;
		cn_init_magic(&cnm_state);
		cn_set_magic("+++");
	}
#endif

	rpi_bootparams();	/* update curcpu()->ci_data.cpu_cc_freq */

	cpu_reset_address = bcm2835_system_reset;

	initarm64(&bootconfig);
}

/*
 * Return true if this model Raspberry Pi has Bluetooth/Wi-Fi support
 */
static bool
rpi_rev_has_btwifi(uint32_t rev)
{
	if ((rev & VCPROP_REV_ENCFLAG) == 0)
		return false;

	switch (__SHIFTOUT(rev, VCPROP_REV_MODEL)) {
	case RPI_MODEL_B_PI3:
	case RPI_MODEL_ZERO_W:
		return true;
	default:
		return false;
	}
}

static void
rpi_uartinit(void)
{
	const paddr_t pa =
	    BCM2835_PERIPHERALS_BUS_TO_PHYS(BCM2835_ARMMBOX_BASE);
	const bus_space_tag_t iot = &bcm2835_bs_tag;
	const bus_space_handle_t ioh = pmap_devmap_phystov(pa);
	uint32_t res;

	cpu_dcache_wbinv_range((vaddr_t)&vb_uart, sizeof(vb_uart));
	bcm2835_mbox_write(iot, ioh, BCMMBOX_CHANARM2VC,
	    KERN_VTOPHYS(&vb_uart));
	bcm2835_mbox_read(iot, ioh, BCMMBOX_CHANARM2VC, &res);

	if (vcprop_tag_success_p(&vb_uart.vbt_boardrev.tag)) {
		if (rpi_rev_has_btwifi(vb_uart.vbt_boardrev.rev)) {
#if NCOM > 0
			/* Enable AUX UART on GPIO header */
			bcm2835gpio_function_select(14, BCM2835_GPIO_ALT5);
			bcm2835gpio_function_select(15, BCM2835_GPIO_ALT5);
#else
			/* Enable UART0 (PL011) on GPIO header */
			bcm2835gpio_function_select(14, BCM2835_GPIO_ALT0);
			bcm2835gpio_function_select(15, BCM2835_GPIO_ALT0);
#endif
		}
	}

	if (vcprop_tag_success_p(&vb_uart.vbt_uartclockrate.tag))
		uart_clk = vb_uart.vbt_uartclockrate.rate;
}

static void
rpi_bootparams(void)
{
	const paddr_t pa =
	    BCM2835_PERIPHERALS_BUS_TO_PHYS(BCM2835_ARMMBOX_BASE);
	const bus_space_tag_t iot = &bcm2835_bs_tag;
	const bus_space_handle_t ioh = pmap_devmap_phystov(pa);
	uint32_t res;

	bcm2835_mbox_write(iot, ioh, BCMMBOX_CHANPM, (
#if (NSDHC > 0)
	    (1 << VCPM_POWER_SDCARD) |
#endif
#if (NPLCOM > 0)
	    (1 << VCPM_POWER_UART0) |
#endif
#if (NBCMDWCTWO > 0)
	    (1 << VCPM_POWER_USB) |
#endif
#if (NBSCIIC > 0)
	    (1 << VCPM_POWER_I2C0) | (1 << VCPM_POWER_I2C1) |
	/*  (1 << VCPM_POWER_I2C2) | */
#endif
#if (NBCMSPI > 0)
	    (1 << VCPM_POWER_SPI) |
#endif
	    0) << 4);

	cpu_dcache_wbinv_range((vaddr_t)&vb, sizeof(vb));
	bcm2835_mbox_write(iot, ioh, BCMMBOX_CHANARM2VC, KERN_VTOPHYS(&vb));
	bcm2835_mbox_read(iot, ioh, BCMMBOX_CHANARM2VC, &res);

	if (!vcprop_buffer_success_p(&vb.vb_hdr)) {
		printf("%s: Mailbox Property read ***FAILED***\n", __func__);
		bootconfig.dramblocks = 1;
		bootconfig.dram[0].address = 0x0;
		bootconfig.dram[0].pages = atop(RPI_MINIMUM_SPLIT);
		return;
	}

	struct vcprop_tag_memory *vptp_mem = &vb.vbt_memory;

	if (vcprop_tag_success_p(&vptp_mem->tag)) {
		size_t n = vcprop_tag_resplen(&vptp_mem->tag) /
		    sizeof(struct vcprop_memory);

		for (int i = 0; i < n; i++) {
			bootconfig.dram[i].address = vptp_mem->mem[i].base;
			bootconfig.dram[i].pages = atop(vptp_mem->mem[i].size);
			bootconfig.dram[i].vmfreelist = VM_FREELIST_DEFAULT;
			bootconfig.dram[i].flags = 0;
			bootconfig.dramblocks++;

			printf("%s: memory.base  %08x\n", __func__,
			    vptp_mem->mem[i].base);
			printf("%s: memory.size  %08x\n", __func__,
			    vptp_mem->mem[i].size);
		}
	}

	if (vcprop_tag_success_p(&vb.vbt_armclockrate.tag))
		curcpu()->ci_data.cpu_cc_freq = vb.vbt_armclockrate.rate;

#ifdef VERBOSE_INIT_ARM
	if (vcprop_tag_success_p(&vb.vbt_armclockrate.tag))
		printf("%s: armcockrate  %u\n", __func__,
		    vb.vbt_armclockrate.rate);

	if (vcprop_tag_success_p(&vb.vbt_fwrev.tag))
		printf("%s: firmware rev %x\n", __func__,
		    vb.vbt_fwrev.rev);
	if (vcprop_tag_success_p(&vb.vbt_macaddr.tag))
		printf("%s: mac-address  %llx\n", __func__,
		    vb.vbt_macaddr.addr);
	if (vcprop_tag_success_p(&vb.vbt_boardmodel.tag))
		printf("%s: board model  %x\n", __func__,
		    vb.vbt_boardmodel.model);
	if (vcprop_tag_success_p(&vb.vbt_boardrev.tag))
		printf("%s: board rev    %x\n", __func__,
		    vb.vbt_boardrev.rev);
	if (vcprop_tag_success_p(&vb.vbt_serial.tag))
		printf("%s: board serial %llx\n", __func__,
		    vb.vbt_serial.sn);
	if (vcprop_tag_success_p(&vb.vbt_dmachan.tag))
		printf("%s: DMA channel mask 0x%08x\n", __func__,
		    vb.vbt_dmachan.mask);

	if (vcprop_tag_success_p(&vb.vbt_cmdline.tag))
		printf("%s: cmdline      \"%s\"\n", __func__,
		    vb.vbt_cmdline.cmdline);
#endif
}

static void
consinit_plcom(void)
{
#if (NPLCOM > 0 && defined(PLCONSOLE))
	/*
	 * Initialise the diagnostic serial console
	 * This allows a means of generating output during initarm().
	 */
	rpi_pi.pi_iobase = consaddr;

	plcomcnattach(&rpi_pi, plcomcnspeed, uart_clk,
	    plcomcnmode, PLCOMCNUNIT);
#endif
}

static void
consinit_com(void)
{
#if NCOM > 0
	bus_space_tag_t iot = &bcm2835_a4x_bs_tag;
	const bus_addr_t addr = BCM2835_AUX_UART_BASE;
	const int speed = B115200;
	u_int freq = 0;
	const u_int flags = TTYDEF_CFLAG;

	if (vcprop_tag_success_p(&vb_uart.vbt_coreclockrate.tag))
		freq = vb.vbt_coreclockrate.rate * 2;

	comcnattach(iot, addr, speed, freq, COM_TYPE_BCMAUXUART, flags);
#endif
}

void
consinit(void)
{
	static int consinit_called = 0;
	bool use_auxuart = false;

	if (consinit_called != 0)
		return;

	consinit_called = 1;

#if NCOM > 0
	if (vcprop_tag_success_p(&vb_uart.vbt_boardrev.tag) &&
	    rpi_rev_has_btwifi(vb_uart.vbt_boardrev.rev)) {
		use_auxuart = true;
	}
#endif

	if (use_auxuart)
		consinit_com();
	else
		consinit_plcom();
}

#ifdef EARLY_CONSOLE
/*
 * EARLY_CONSOLE - console input/output functions with PA=VA mappings.
 * usable before consinit(), and no need to initialize pmap.
 * ddb(4) patly works too.
 */

/* in rpi_start.S */
void uartputc(char);
char uartgetc(void);

static dev_type_cngetc(konsgetc);
static dev_type_cnputc(konsputc);
static dev_type_cnpollc(konspollc);
static struct consdev konsole = {
	NULL, NULL, konsgetc, konsputc, konspollc, NULL,
	NULL, NULL, NODEV, CN_NORMAL
};

static void
konsinit(void)
{
	/* make debugging aid work */
	cn_tab = &konsole;
}

static int
konsgetc(dev_t dev)
{
	return uartgetc();
}

static void
konsputc(dev_t dev, int c)
{
	uartputc(c);
}

static void
konspollc(dev_t dev, int on)
{
}
#endif

static void
rpi_device_register(device_t dev, void *aux)
{
	prop_dictionary_t dict = device_properties(dev);

#if defined(BCM2836)
	if (device_is_a(dev, "a64gtmr")) {
		/*
		 * The frequency of the generic timer is the reference
		 * frequency.
		 */
		prop_dictionary_set_uint32(dict, "frequency", RPI_REF_FREQ);
		return;
	}
#endif

	if (device_is_a(dev, "plcom") &&
	    vcprop_tag_success_p(&vb_uart.vbt_uartclockrate.tag) &&
	    vb_uart.vbt_uartclockrate.rate > 0) {
		prop_dictionary_set_uint32(dict,
		    "frequency", vb_uart.vbt_uartclockrate.rate);
	}
	if (device_is_a(dev, "com") &&
	    vcprop_tag_success_p(&vb.vbt_coreclockrate.tag) &&
	    vb.vbt_coreclockrate.rate > 0) {
		prop_dictionary_set_uint32(dict,
		    "frequency", vb.vbt_coreclockrate.rate);
	}
	if (device_is_a(dev, "bcmdmac") &&
	    vcprop_tag_success_p(&vb.vbt_dmachan.tag)) {
		prop_dictionary_set_uint32(dict,
		    "chanmask", vb.vbt_dmachan.mask);
	}
	if (device_is_a(dev, "sdhc") &&
	    vcprop_tag_success_p(&vb.vbt_emmcclockrate.tag) &&
	    vb.vbt_emmcclockrate.rate > 0) {
		prop_dictionary_set_uint32(dict,
		    "frequency", vb.vbt_emmcclockrate.rate);
	}
	if (device_is_a(dev, "sdhost") &&
	    vcprop_tag_success_p(&vb.vbt_coreclockrate.tag) &&
	    vb.vbt_coreclockrate.rate > 0) {
		prop_dictionary_set_uint32(dict,
		    "frequency", vb.vbt_coreclockrate.rate);
		if (!rpi_rev_has_btwifi(vb.vbt_boardrev.rev)) {
			/* No btwifi and sdhost driver is present */
			prop_dictionary_set_bool(dict, "disable", true);
		}
	}
	if (booted_device == NULL &&
	    device_is_a(dev, "ld") &&
	    device_is_a(device_parent(dev), "sdmmc")) {
		booted_partition = 0;
		booted_device = dev;
	}
	if (device_is_a(dev, "usmsc") &&
	    vcprop_tag_success_p(&vb.vbt_macaddr.tag)) {
		const uint8_t enaddr[ETHER_ADDR_LEN] = {
		    (vb.vbt_macaddr.addr >> 0) & 0xff,
		    (vb.vbt_macaddr.addr >> 8) & 0xff,
		    (vb.vbt_macaddr.addr >> 16) & 0xff,
		    (vb.vbt_macaddr.addr >> 24) & 0xff,
		    (vb.vbt_macaddr.addr >> 32) & 0xff,
		    (vb.vbt_macaddr.addr >> 40) & 0xff
		};

		prop_data_t pd = prop_data_create_data(enaddr, ETHER_ADDR_LEN);
		KASSERT(pd != NULL);
		if (prop_dictionary_set(device_properties(dev), "mac-address",
		    pd) == false) {
			aprint_error_dev(dev,
			    "WARNING: Unable to set mac-address property\n");
		}
		prop_object_release(pd);
	}

#if 0 /* XXXAARCH64 */

#if NGENFB > 0
	if (device_is_a(dev, "genfb")) {
		char *ptr;

		bcmgenfb_set_console_dev(dev);
		bcmgenfb_set_ioctl(&rpi_ioctl);
#ifdef DDB
		db_trap_callback = bcmgenfb_ddb_trap_callback;
#endif

		if (rpi_fb_init(dict, aux) == false)
			return;
		if (get_bootconf_option(boot_args, "console",
		    BOOTOPT_TYPE_STRING, &ptr) && strncmp(ptr, "fb", 2) == 0) {
			prop_dictionary_set_bool(dict, "is_console", true);
#if NUKBD > 0
			/* allow ukbd to be the console keyboard */
			ukbd_cnattach();
#endif
		} else {
			prop_dictionary_set_bool(dict, "is_console", false);
		}
	}
#endif

#endif /* XXXAARCH64 */


	/* BSC0 is used internally on some boards */
	if (device_is_a(dev, "bsciic") &&
	    ((struct amba_attach_args *)aux)->aaa_addr == BCM2835_BSC0_BASE) {
		if (rpi_rev_has_btwifi(vb.vbt_boardrev.rev)) {
			prop_dictionary_set_bool(dict, "disable", true);
		}
	}
}

SYSCTL_SETUP(sysctl_machdep_rpi, "sysctl machdep subtree setup (rpi)")
{
	sysctl_createv(clog, 0, NULL, NULL,
	    CTLFLAG_PERMANENT, CTLTYPE_NODE, "machdep", NULL,
	    NULL, 0, NULL, 0, CTL_MACHDEP, CTL_EOL);

	sysctl_createv(clog, 0, NULL, NULL,
	    CTLFLAG_PERMANENT|CTLFLAG_READONLY,
	    CTLTYPE_INT, "firmware_revision", NULL, NULL, 0,
	    &vb.vbt_fwrev.rev, 0, CTL_MACHDEP, CTL_CREATE, CTL_EOL);

	sysctl_createv(clog, 0, NULL, NULL,
	    CTLFLAG_PERMANENT|CTLFLAG_READONLY,
	    CTLTYPE_INT, "board_model", NULL, NULL, 0,
	    &vb.vbt_boardmodel.model, 0, CTL_MACHDEP, CTL_CREATE, CTL_EOL);

	sysctl_createv(clog, 0, NULL, NULL,
	    CTLFLAG_PERMANENT|CTLFLAG_READONLY,
	    CTLTYPE_INT, "board_revision", NULL, NULL, 0,
	    &vb.vbt_boardrev.rev, 0, CTL_MACHDEP, CTL_CREATE, CTL_EOL);

	sysctl_createv(clog, 0, NULL, NULL,
	    CTLFLAG_PERMANENT|CTLFLAG_READONLY|CTLFLAG_HEX|CTLFLAG_PRIVATE,
	    CTLTYPE_QUAD, "serial", NULL, NULL, 0,
	    &vb.vbt_serial.sn, 0, CTL_MACHDEP, CTL_CREATE, CTL_EOL);
}
