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

#include <aarch64/cpu.h>
#include <aarch64/armreg.h>
#include <aarch64/machdep.h>

void initarm(void);
void uartputs(const char *);

static dev_type_cngetc(konsgetc);
static dev_type_cnputc(konsputc);
static dev_type_cnpollc(konspollc);

static struct consdev konsole = {
	NULL, NULL, konsgetc, konsputc, konspollc, NULL,
	NULL, NULL, NODEV, CN_NORMAL
};
static void konsinit(void);

void
initarm(void)
{
	konsinit();

	// XXXAARCH64
	physical_start = 0;
	physical_end = physical_start + MEMSIZE * 1024 * 1024;

	printf("%s:%d: curcpu()=%p\n", __func__, __LINE__, curcpu());

	initarm64();
}

void
consinit(void)
{
	static int consinit_called = 0;

	if (consinit_called)
		return;

	consinit_called = 1;
#if 0
	/*
	 * build bus_space tag to run real UART console in NetBSD way.
	 */
#endif
}

#define AUX_MU_BASE	0x3f215000
#define AUX_MU_IO_REG	0x40	/* Mini Uart I/O Data (8bit) */
#define AUX_MU_LSR_REG	0x54	/* Mini Uart Line Status (8bit) */

#define THR		0x40	/* octet write to Tx */
#define RBR		0x40	/* octet read to Rx */
#define LSR		0x54	/* line status */
#define LSR_THRE	0x20
#define UART_READ(r)		*(volatile uint32_t *)(uartbase + (r))
#define UART_WRITE(r, v)	*(volatile uint32_t *)(uartbase + (r)) = (v)
#define LSR_TXEMPTY		0x20
#define LSR_OE			0x02
#define LSR_RXREADY		0x01

static uintptr_t uartbase;

static void
konsinit(void)
{
	/* make debugging aid work */
	cn_tab = &konsole;
	uartbase = AUX_MU_BASE;
}

static int
konsgetc(dev_t dev)
{
	unsigned lsr;
	int s, c;

	s = splserial();
	do {
		lsr = UART_READ(LSR);
	} while ((lsr & LSR_OE) || (lsr & LSR_RXREADY) == 0);
	c = UART_READ(RBR);
	splx(s);
	return c & 0xff;
}

static void
konsputc(dev_t dev, int c)
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
konspollc(dev_t dev, int on)
{
}
