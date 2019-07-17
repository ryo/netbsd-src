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

#include "opt_console.h"
#include "locators.h"

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#define cn_trap()			\
	do {				\
		console_debugger();	\
		cn_trapped = 1;		\
		(void)cn_trapped;	\
	} while (/* CONSTCOND */ 0)

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/device.h>
#include <sys/ioctl.h>
#include <sys/kauth.h>
#include <sys/lwp.h>
#include <sys/poll.h>
#ifdef RND_COM
#include <sys/rndsource.h>
#endif
#include <sys/select.h>
#include <sys/tty.h>
#include <dev/cons.h>
#include <dev/fdt/fdtvar.h>
#include <arm/snapdragon/msmuartreg.h>

#define REGREAD(sc, reg)	\
	bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (reg))
#define REGWRITE(sc, reg, val)	\
	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (reg), (val))

struct msmuart_softc {
	device_t sc_dev;
	bus_space_tag_t sc_bst;
	bus_space_handle_t sc_bsh;
	int sc_phandle;
	bool sc_uartdm;

	kmutex_t sc_intr_lock;
	void *sc_ih;
	void *sc_sih;

	struct tty *sc_tty;
	struct clk *sc_clk;
	int sc_ospeed;

	/* RX */
	bool sc_rx_ready;
	unsigned int sc_rbuf_w;	/* write pos of sc_rbuf[] */
	unsigned int sc_rbuf_r;	/* read pos of sc_rbuf[] */
#define MSMUART_RBUFSZ	512	/* must be 2^n */
	u_char sc_rbuf[MSMUART_RBUFSZ];
#define RBUF_QUEUED(sc)	((sc)->sc_rbuf_w - (sc)->sc_rbuf_r)
#define RBUF_EMPTY(sc)	(RBUF_QUEUED((sc)) == 0)
#define RBUF_AVAIL(sc)	(RBUF_QUEUED((sc)) < (MSMUART_RBUFSZ - 1))
#define RBUF_ENQUEUE(sc, c) \
	do { \
		(sc)->sc_rbuf[(sc)->sc_rbuf_w++ & (MSMUART_RBUFSZ - 1)] = (c); \
	} while (/* CONSTCOND */0)
#define RBUF_DEQUEUE(sc) \
	((sc)->sc_rbuf[(sc)->sc_rbuf_r++ & (MSMUART_RBUFSZ - 1)] & 0xff)
#define RBUF_PEEK(sc) \
	((sc)->sc_rbuf[(sc)->sc_rbuf_r & (MSMUART_RBUFSZ - 1)] & 0xff)

	/* TX */
	bool sc_tx_ready;
	bool sc_tx_busy;
	bool sc_tx_stopped;
};

int msmuart_match(device_t, cfdata_t, void *);
void msmuart_attach(device_t, device_t, void *);

int msmuart_intr(void *);
void msmuart_softint(void *);
void msmuart_rxsoft(struct msmuart_softc *, struct tty *);
void msmuart_txsoft(struct msmuart_softc *, struct tty *);
void msmuart_start(struct tty *);
int msmuart_param(struct tty *, struct termios *);

extern struct cfdriver msmuart_cd;

static const struct device_compatible_entry compatible_uart[] = {
	{ .compat = "qcom,msm-uart" },
	DEVICE_COMPAT_EOL
};

static const struct device_compatible_entry compatible_uartdm[] = {
	{ .compat = "qcom,msm-uartdm-v1.1" },
	{ .compat = "qcom,msm-uartdm-v1.2" },
	{ .compat = "qcom,msm-uartdm-v1.3" },
	{ .compat = "qcom,msm-uartdm-v1.4" },
	{ .compat = "qcom,msm-uartdm" },
	DEVICE_COMPAT_EOL
};

static int msmuart_cmajor = -1;
static struct msmuart_softc msmuart_console_sc;
static struct cnm_state msmuart_cnm_state;
#define CN_CHECK_MAGIC(dev, c)	\
	cn_check_magic((dev), (c), msmuart_cnm_state)

/* consdev */
static int msmuart_console_match(int);
static void msmuart_console_consinit(struct fdt_attach_args *, u_int);
static int msmuart_cngetc(dev_t);
static void msmuart_busyputc(struct msmuart_softc *, int);
static void msmuart_cnputc(dev_t, int);
static void msmuart_cnpollc(dev_t, int);
static struct consdev msmuart_consdev = {
	.cn_getc = msmuart_cngetc,
	.cn_putc = msmuart_cnputc,
	.cn_pollc = msmuart_cnpollc,
	.cn_dev = NODEV,
	.cn_pri = CN_NORMAL
};
static const struct fdt_console msmuart_console = {
	.match = msmuart_console_match,
	.consinit = msmuart_console_consinit
};

/* cdevsw */
dev_type_open(msmuart_open);
dev_type_close(msmuart_close);
dev_type_read(msmuart_read);
dev_type_write(msmuart_write);
dev_type_ioctl(msmuart_ioctl);
dev_type_tty(msmuart_tty);
dev_type_poll(msmuart_poll);
dev_type_stop(msmuart_stop);
const struct cdevsw msmuart_cdevsw = {
	.d_open = msmuart_open,
	.d_close = msmuart_close,
	.d_read = msmuart_read,
	.d_write = msmuart_write,
	.d_ioctl = msmuart_ioctl,
	.d_stop = msmuart_stop,
	.d_tty = msmuart_tty,
	.d_poll = msmuart_poll,
	.d_mmap = nommap,
	.d_kqfilter = ttykqfilter,
	.d_discard = nodiscard,
	.d_flag = D_TTY
};


CFATTACH_DECL_NEW(msmuart, sizeof(struct msmuart_softc),
	msmuart_match, msmuart_attach, NULL, NULL);

int
msmuart_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;
	return of_compatible_match(faa->faa_phandle, compatible_uartdm) ||
	    of_compatible_match(faa->faa_phandle, compatible_uart);
}

void
msmuart_attach(device_t parent, device_t self, void *aux)
{
	struct msmuart_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	struct tty *tp;
	bus_addr_t addr;
	bus_size_t size;
	int major, minor, error, i;
	char intrstr[128];

	sc->sc_dev = self;

	sc->sc_phandle = faa->faa_phandle;
	sc->sc_bst = faa->faa_bst;
	if (of_compatible_match(sc->sc_phandle, compatible_uartdm) != 0)
		sc->sc_uartdm = true;
	else
		sc->sc_uartdm = false;

	if (fdtbus_get_reg(sc->sc_phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}

	if (bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}

	if (!fdtbus_intr_str(sc->sc_phandle, 0, intrstr, sizeof(intrstr))) {
		aprint_error(": failed to decode interrupt\n");
		return;
	}

	/* Enable clocks */
	struct clk *clk;
	for (i = 0; (clk = fdtbus_clock_get_index(sc->sc_phandle, i)); i++) {
printf("<ENABLE CLOCK %d>", i);
		if (clk_enable(clk) != 0) {
			aprint_error(": failed to enable clock #%d\n", i);
			return;
		}
		/* First clock is UARTCLK */
		if (i == 0)
			sc->sc_clk = clk;
	}

	/* disable interrupts */
	REGWRITE(sc, sc->sc_uartdm ? UART_DM_IMR : UART_IMR, 0);

	mutex_init(&sc->sc_intr_lock, MUTEX_DEFAULT, IPL_SERIAL);
	sc->sc_ih = fdtbus_intr_establish(sc->sc_phandle, 0, IPL_SERIAL,
	    FDT_INTR_MPSAFE, msmuart_intr, sc);
	if (sc->sc_ih == NULL) {
		aprint_error(": failed to establish interrupt on %s\n",
		    intrstr);
		return;
	}

	sc->sc_sih = softint_establish(SOFTINT_SERIAL, msmuart_softint, sc);
	if (sc->sc_sih == NULL) {
		aprint_error(": failed to establish softint\n");
		return;
	}

	if (msmuart_cmajor == -1) {
		/* allocate a major number */
		int bmajor = -1, cmajor = -1;
		error = devsw_attach("msmuart", NULL, &bmajor,
		    &msmuart_cdevsw, &cmajor);
		if (error) {
			aprint_error(": couldn't allocate major number\n");
			return;
		}
		msmuart_cmajor = cmajor;
	}

	major = cdevsw_lookup_major(&msmuart_cdevsw);
	minor = device_unit(self);
	tp = sc->sc_tty = tty_alloc();
	tp->t_oproc = msmuart_start;
	tp->t_param = msmuart_param;
	tp->t_dev = makedev(major, minor);
	tp->t_sc = sc;
	tty_attach(tp);

	aprint_naive("\n");
	if (sc->sc_phandle == msmuart_console_sc.sc_phandle) {
		cn_tab->cn_dev = tp->t_dev;
		aprint_normal(": console");
	}
	aprint_normal("\n");

	aprint_normal_dev(self, "interrupting on %s, %u Hz\n", intrstr, (sc->sc_clk == NULL) ? 0 : clk_get_rate(sc->sc_clk));

	//XXX: DEBUG
	aprint_normal_dev(self, "UART_DM_CSR=%08x\n", REGREAD(sc, UART_DM_CSR));
}

int
msmuart_intr(void *arg)
{
	struct msmuart_softc *sc = arg;
	struct tty *tp = sc->sc_tty;
	uint32_t status, val;
	int n;

	mutex_spin_enter(&sc->sc_intr_lock);

	status = REGREAD(sc, sc->sc_uartdm ? UART_DM_MISR : UART_MISR);

//	if (status & UART_DM_IRQBIT_DELTA_CTS) {
//		XXX: handle CTS
//	}

	if (status & UART_DM_IRQBIT_RX_BREAK_START) {
		REGWRITE(sc, UART_DM_CR, UART_DM_CR_CH_CMD_RST_BRK_START);
		int cn_trapped __unused = 0;
		CN_CHECK_MAGIC(tp->t_dev, CNC_BREAK);
	}

	/* read data and store to sc_rbuf[] */
	if (status & (UART_DM_IRQBIT_RX_LEV|UART_DM_IRQBIT_RX_STALE)) {
		if (!sc->sc_uartdm) {
			/* no FIFO. read one character */
			while ((REGREAD(sc, UART_SR) & UART_SR_RXRDY) != 0) {
				int cn_trapped = 0;

				val = REGREAD(sc, UART_RF);
				CN_CHECK_MAGIC(tp->t_dev, val & 0xff);
				if (cn_trapped)
					continue;
				if (RBUF_AVAIL(sc)) {
					RBUF_ENQUEUE(sc, val);
				}
			}
			goto rxintr_done;
		}

		/* read 4 bytes from FIFO register at a time */
		while ((REGREAD(sc, UART_DM_SR) & UART_DM_SR_RXRDY) != 0) {
			val = REGREAD(sc, UART_DM_RF);
			for (n = 0; n < 4; n++, val >>= 8) {
				int cn_trapped = 0;
				CN_CHECK_MAGIC(tp->t_dev, val & 0xff);
				if (cn_trapped)
					continue;

				if (!RBUF_AVAIL(sc))
					break;
				RBUF_ENQUEUE(sc, val);
			}
		}

		/* less than 4 chars exists? */
		val = REGREAD(sc, UART_DM_RXFS);
		n = __SHIFTOUT(val, UART_DM_RXFS_RX_BUFFER_STATE);
		if (n == 0)
			goto rxintr_done;
		REGWRITE(sc, UART_DM_CR, UART_DM_CR_GEN_CMD_FORCE_STALE);
		val = REGREAD(sc, UART_DM_RF);
		REGWRITE(sc, UART_DM_CR, UART_DM_CR_CH_CMD_RST_STALE_INT);
		REGWRITE(sc, UART_DM_DMRX, 0xffffff);
		REGWRITE(sc, UART_DM_CR, UART_DM_CR_GEN_CMD_EN_STALE_EVENT);
		for (; n > 0; n--, val >>= 8) {
			if (!RBUF_AVAIL(sc))
				break;
			RBUF_ENQUEUE(sc, val);
		}
 rxintr_done:
		sc->sc_rx_ready = true;
	}

	if (status & UART_DM_IRQBIT_TX_LEV)
		sc->sc_tx_ready = true;

	if (sc->sc_rx_ready || sc->sc_tx_ready)
		softint_schedule(sc->sc_sih);

	mutex_spin_exit(&sc->sc_intr_lock);
	return 0;
}

void
msmuart_softint(void *arg)
{
	struct msmuart_softc *sc = arg;
	struct tty *tp = sc->sc_tty;

	if (sc->sc_rx_ready) {
		sc->sc_rx_ready = false;
		msmuart_rxsoft(sc, tp);
	}
	if (sc->sc_tx_ready) {
		sc->sc_tx_ready = false;
		msmuart_txsoft(sc, tp);
	}
}

void
msmuart_txsoft(struct msmuart_softc *sc, struct tty *tp)
{
	/* XXX: notyet */
}

void
msmuart_start(struct tty *tp)
{
	struct msmuart_softc *sc = tp->t_sc;
	int s, c;

	s = spltty();

	if (ISSET(tp->t_state, TS_BUSY | TS_TIMEOUT | TS_TTSTOP))
		goto out;
	if (sc->sc_tx_stopped)
		goto out;
	if (!ttypull(tp))
		goto out;

	SET(tp->t_state, TS_BUSY);
	sc->sc_tx_busy = true;

	// XXX: using busy loop. use TX_LEV interrupt and txsoft()
	for (;;) {
		c = getc(&tp->t_outq);
		if (c == -1)
			break;
		msmuart_busyputc(sc, c);
	}

	sc->sc_tx_busy = false;
	CLR(tp->t_state, TS_BUSY);


 out:
	splx(s);
}

void
msmuart_rxsoft(struct msmuart_softc *sc, struct tty *tp)
{
	int c;

	if (!ISSET(tp->t_state, TS_ISOPEN))
		return;

	while (!RBUF_EMPTY(sc)) {
		c = RBUF_PEEK(sc);
		if (tp->t_linesw->l_rint(c, tp) == -1)
			break;
		sc->sc_rbuf_r++;	/* RBUF_DEQUEUE(sc) */
	}
}


//XXX
//
//static int
//msmuart_baudrate(int speed)
//{
//	static const int divider[16] = {
//		24576, 12288, 6144, 3072,
//		 1536,   768,  512,  384,
//		  256,   192,  128,   96,
//		   64,    48,   32,   16
//	};
//
//	decide best divider...
//
//}

int
msmuart_param(struct tty *tp, struct termios *t)
{
//XXX
//
//	struct msmuart_softc *sc =
//	    device_lookup_private(&msmuart_cd, TTUNIT(tp->t_dev));
//	int ospeed;
//
//	ospeed = msmuart_baudrate(t->c_ispeed);
//	if (ospeed < 0)
//		return EINVAL;
//	if (tp->t_ispeed && t->c_ispeed != t->c_ospeed)
//		return EINVAL;

	tp->t_ispeed = t->c_ispeed;
	tp->t_ospeed = t->c_ospeed;
	tp->t_cflag = t->c_cflag;

	return 0;
}

int
msmuart_open(dev_t dev, int flag, int mode, struct lwp *l)
{
	struct msmuart_softc *sc;
	struct tty *tp;
	int s;

	sc = device_lookup_private(&msmuart_cd, minor(dev));
	tp = sc->sc_tty;

	if (kauth_authorize_device_tty(l->l_cred, KAUTH_DEVICE_TTY_OPEN, tp))
		return EBUSY;

	s = spltty();
	if (!ISSET(tp->t_state, TS_ISOPEN) && tp->t_wopen == 0) {
		tp->t_dev = dev;
		ttychars(tp);
		tp->t_iflag = TTYDEF_IFLAG;
		tp->t_oflag = TTYDEF_OFLAG;
		tp->t_cflag = TTYDEF_CFLAG;
		tp->t_lflag = TTYDEF_LFLAG;
		tp->t_ispeed = tp->t_ospeed = TTYDEF_SPEED;
		ttsetwater(tp);

		/* enable interrupts */
		if (sc->sc_uartdm) {
			REGWRITE(sc, UART_DM_IMR,
			    UART_DM_IRQBIT_RX_LEV |
			    UART_DM_IRQBIT_RX_STALE |
			    UART_DM_IRQBIT_RX_BREAK_START);
		} else {
			REGWRITE(sc, UART_IMR,
			    UART_DM_IRQBIT_RX_LEV |
			    UART_DM_IRQBIT_RX_STALE);
		}
	}
	tp->t_state |= TS_CARR_ON;
	splx(s);

	return (*tp->t_linesw->l_open)(dev, tp);
}

int
msmuart_close(dev_t dev, int flag, int mode, struct lwp *l)
{
	struct msmuart_softc *sc;
	struct tty *tp;

	sc = device_lookup_private(&msmuart_cd, minor(dev));
	tp = sc->sc_tty;

	tp->t_linesw->l_close(tp, flag);
	ttyclose(tp);

	if (!ISSET(tp->t_state, TS_ISOPEN) && tp->t_wopen == 0) {
		/* disable interrupts */
		REGWRITE(sc, sc->sc_uartdm ? UART_DM_IMR : UART_IMR, 0);
	}

	return 0;
}

int
msmuart_read(dev_t dev, struct uio *uio, int flag)
{
	struct tty *tp = msmuart_tty(dev);
	return tp->t_linesw->l_read(tp, uio, flag);
}

int
msmuart_write(dev_t dev, struct uio *uio, int flag)
{
	struct tty *tp = msmuart_tty(dev);
	return tp->t_linesw->l_write(tp, uio, flag);
}

int
msmuart_poll(dev_t dev, int events, struct lwp *l)
{
	struct tty *tp = msmuart_tty(dev);
	return tp->t_linesw->l_poll(tp, events, l);
}

void
msmuart_stop(struct tty *ty, int flag)
{
	// XXX: notyet
}

struct tty *
msmuart_tty(dev_t dev)
{
	struct msmuart_softc *sc;

	sc = device_lookup_private(&msmuart_cd, minor(dev));
	return sc->sc_tty;
}

int
msmuart_ioctl(dev_t dev, u_long cmd, void *data, int flag, struct lwp *l)
{
	struct tty *tp = msmuart_tty(dev);
	int error;

	error = tp->t_linesw->l_ioctl(tp, cmd, data, flag, l);
	if (error != EPASSTHROUGH)
		return error;

	return ttioctl(tp, cmd, data, flag, l);
}

static int
msmuart_console_match(int phandle)
{
	return of_compatible_match(phandle, compatible_uartdm) ||
	    of_compatible_match(phandle, compatible_uart);
}

static void
msmuart_console_consinit(struct fdt_attach_args *faa, u_int uart_freq)
{
	struct msmuart_softc *sc = &msmuart_console_sc;
	bus_addr_t addr;
	bus_size_t size;
	int speed, error;

	speed = fdtbus_get_stdout_speed();
	if (speed < 0)
		speed = 115200;	/* default */

//	tcflag_t flags;
//	flags = fdtbus_get_stdout_flags();

	sc->sc_bst = faa->faa_bst;
	sc->sc_phandle = faa->faa_phandle;

	if (of_compatible_match(sc->sc_phandle, compatible_uartdm) != 0)
		sc->sc_uartdm = true;
	else
		sc->sc_uartdm = false;

	fdtbus_get_reg(sc->sc_phandle, 0, &addr, &size);
	error = bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh);
	if (error != 0)
		panic("failed to map console, error=%d\n", error);

	cn_tab = &msmuart_consdev;
	cn_init_magic(&msmuart_cnm_state);
	cn_set_magic("\047\001");	/* default magic is BREAK */
}

static int
msmuart_cngetc(dev_t dev)
{
	struct msmuart_softc *sc = &msmuart_console_sc;
	uint32_t val;
	int c, n, s;

	s = splserial();

	if (!sc->sc_uartdm) {
		/* no FIFO. read one character */
		if ((REGREAD(sc, UART_SR) & UART_SR_RXRDY) != 0) {
			val = REGREAD(sc, UART_RF);
			if (RBUF_AVAIL(sc))
				RBUF_ENQUEUE(sc, val);
		}
		goto readfromrbuf;
	}

	if (REGREAD(sc, UART_DM_MISR) & UART_DM_IRQBIT_RX_BREAK_START) {
		REGWRITE(sc, UART_DM_CR, UART_DM_CR_CH_CMD_RST_BRK_START);
		int cn_trapped __unused = 0;
		CN_CHECK_MAGIC(dev, CNC_BREAK);
	}

	/* read 1-4 bytes from FIFO register at a time */
	if ((REGREAD(sc, UART_DM_SR) & UART_DM_SR_RXRDY) != 0) {
		/* 4 chars exist */
		val = REGREAD(sc, UART_DM_RF);
		n = 4;
	} else {
		/* less than 4 chars exists? */
		val = REGREAD(sc, UART_DM_RXFS);
		n = __SHIFTOUT(val, UART_DM_RXFS_RX_BUFFER_STATE);
		if (n == 0)
			goto readfromrbuf;
		REGWRITE(sc, UART_DM_CR, UART_DM_CR_GEN_CMD_FORCE_STALE);
		val = REGREAD(sc, UART_DM_RF);
		REGWRITE(sc, UART_DM_CR, UART_DM_CR_CH_CMD_RST_STALE_INT);
		REGWRITE(sc, UART_DM_DMRX, 0xffffff);
		REGWRITE(sc, UART_DM_CR, UART_DM_CR_GEN_CMD_EN_STALE_EVENT);
	}
	for (; n > 0; n--, val >>= 8) {
		if (!RBUF_AVAIL(sc))
			break;
		RBUF_ENQUEUE(sc, val);
	}

 readfromrbuf:
	if (RBUF_EMPTY(sc))
		c = -1;
	else {
		c = RBUF_DEQUEUE(sc);
#if defined(DDB)
		extern int db_active;
		if (!db_active)
#endif
		{
			int cn_trapped __unused = 0;
			CN_CHECK_MAGIC(dev, c);
		}
	}

	splx(s);
	return c;
}


/* busyloop putc */
static void
msmuart_busyputc(struct msmuart_softc *sc, int c)
{
	if (!sc->sc_uartdm) {
		/* wait TXRDY */
		while ((REGREAD(sc, UART_SR) & UART_SR_TXRDY) == 0)
			;

		/* put */
		REGWRITE(sc, UART_TF, c);

		/* wait TXRDY */
		while ((REGREAD(sc, UART_SR) & UART_SR_TXRDY) == 0)
			;

		return;
	}

	while (((REGREAD(sc, UART_DM_SR) & UART_DM_SR_TXEMT) == 0) &&
	    ((REGREAD(sc, UART_DM_ISR) & UART_DM_IRQBIT_TX_READY) == 0))
		;
	REGWRITE(sc, UART_DM_CR, UART_DM_CR_GEN_CMD_RST_TX_RDY);

	REGWRITE(sc, UART_DM_NO_CHARS_FOR_TX, 1);	/* just one character */
	(void)REGREAD(sc, UART_DM_NO_CHARS_FOR_TX);

	/* wait TXRDY */
	while ((REGREAD(sc, UART_DM_SR) & UART_DM_SR_TXRDY) == 0)
		;

	/* put */
	REGWRITE(sc, UART_DM_TF, c);

	/* wait TXRDY */
	while ((REGREAD(sc, UART_DM_SR) & UART_DM_SR_TXRDY) == 0)
		;
}

static void
msmuart_cnputc(dev_t dev, int c)
{
	struct msmuart_softc *sc = &msmuart_console_sc;
	int s;

	s = splserial();
	msmuart_busyputc(sc, c);
	splx(s);
}

static void
msmuart_cnpollc(dev_t dev, int on)
{
}

FDT_CONSOLE(msmuart, &msmuart_console);
