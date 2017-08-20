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
#include <sys/device.h>

#include <dev/cons.h>

vaddr_t physical_start;
vaddr_t physical_end;

vaddr_t initarm(void *);
void uartputs(const char *);

static dev_type_cninit(kcomcninit);
static dev_type_cngetc(kcomcngetc);
static dev_type_cnputc(kcomcnputc);
static dev_type_cnpollc(kcomcnpollc);

static struct consdev kcomcons = {
	NULL, kcomcninit, kcomcngetc, kcomcnputc, kcomcnpollc, NULL,
	NULL, NULL, NODEV, CN_NORMAL
};

vaddr_t
initarm(void *arg)
{
	uartputs("Hello initarm()\r\n");

#if 1
	cn_tab = &kcomcons;
	(*cn_tab->cn_init)(&kcomcons);
#endif

	return NULL;
}


void
consinit(void)
{
	static int consinit_called = 0;

	if (consinit_called)
		return;

	consinit_called = 1;

#ifdef CONSDEVNAME
	// XXXAARCH64

	// init uart
#endif /* CONSDEVNAME */
}

#define AUX_MU_BASE	0x3f215000
#define AUX_MU_IO_REG	0x40	/* Mini Uart I/O Data (8bit) */
#define AUX_MU_IER_REG	0x44	/* Mini Uart Interrupt Enable (8bit) */
#define AUX_MU_IIR_REG	0x48	/* Mini Uart Interrupt Identify (8bit) */
#define AUX_MU_LCR_REG	0x4C	/* Mini Uart Line Control (8bit) */
#define AUX_MU_MCR_REG	0x50	/* Mini Uart Modem Control (8bit) */
#define AUX_MU_LSR_REG	0x54	/* Mini Uart Line Status (8bit) */
#define AUX_MU_MSR_REG	0x58	/* Mini Uart Modem Status (8bit) */
#define AUX_MU_SCRATCH	0x5C	/* Mini Uart Scratch (8bit) */
#define AUX_MU_CNTL_REG	0x60	/* Mini Uart Extra Control (8bit) */
#define AUX_MU_STAT_REG	0x64	/* Mini Uart Extra Status (32bit) */
#define AUX_MU_BAUD_REG	0x68	/* Mini Uart Baudrate (16bit) */

#define THR		0x40	/* octet write to Tx */
#define RBR		0x40	/* octet read to Rx */
#define LSR		0x4C	/* line status */
#define LSR_THRE	0x20
#define UART_READ(r)		*(volatile uint32_t *)(uartbase + (r))
#define UART_WRITE(r, v)	*(volatile uint32_t *)(uartbase + (r)) = (v)
#define LSR_TXEMPTY		0x20
#define LSR_OE			0x02
#define LSR_RXREADY		0x01

static uintptr_t uartbase;

static void
kcomcninit(struct consdev *cn)
{
	/*
	 * we do not touch UART operating parameters since bootloader
	 * is supposed to have done well.
	 */
	uartbase = AUX_MU_BASE /* + KVM device region base */;
}

static int
kcomcngetc(dev_t dev)
{
	unsigned lsr;
	int s, c;

	s = splserial();
#if 1
	do {
		lsr = UART_READ(LSR);
	} while ((lsr & LSR_OE) || (lsr & LSR_RXREADY) == 0);
#else
    again:
	do {
		lsr = UART_READ(LSR);
	} while ((lsr & LSR_RXREADY) == 0);
	if (lsr & (LSR_BE | LSR_FE | LSR_PE)) {
		(void)UART_READ(RBR);
		goto again;
	}
#endif
	c = UART_READ(RBR);
	splx(s);
	return c & 0xff;
}

static void
kcomcnputc(dev_t dev, int c)
{
	unsigned lsr, timo;
	int s;

	s = splserial();
	timo = 150000;
	do {
		lsr = UART_READ(LSR);
	} while (timo-- > 0 && (lsr & LSR_TXEMPTY) == 0);
	if (timo > 0)
		UART_WRITE(THR, c);
	splx(s);
}

static void
kcomcnpollc(dev_t dev, int on)
{
}
