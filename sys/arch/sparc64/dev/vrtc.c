/*	$NetBSD: vrtc.c,v 1.3 2021/01/04 14:48:52 thorpej Exp $	*/
/*	$OpenBSD: vrtc.c,v 1.1 2008/03/08 19:19:43 kettenis Exp $	*/
/*
 * Copyright (c) 2008 Mark Kettenis
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <sys/param.h>
#include <sys/device.h>
#include <sys/kmem.h>
#include <sys/systm.h>

#include <machine/autoconf.h>
#include <machine/hypervisor.h>
#include <machine/openfirm.h>

#include <dev/clock_subr.h>
#include <sparc64/dev/vbusvar.h>

extern todr_chip_handle_t todr_handle;

int	vrtc_match(device_t, cfdata_t, void *);
void	vrtc_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(vrtc, sizeof(device_t),
    vrtc_match, vrtc_attach, NULL, NULL);

int	vrtc_gettime(todr_chip_handle_t, struct timeval *);
int	vrtc_settime(todr_chip_handle_t, struct timeval *);

int
vrtc_match(device_t parent, cfdata_t match, void *aux)
{
	struct vbus_attach_args *va = aux;

	if (strcmp(va->va_name, "rtc") == 0)
		return (1);

	return (0);
}

void
vrtc_attach(device_t parent, device_t self, void *aux)
{
	todr_chip_handle_t handle;

	printf("\n");

	handle = kmem_alloc(sizeof(struct todr_chip_handle), KM_SLEEP);
	handle->cookie = self;
	handle->todr_gettime = vrtc_gettime;
	handle->todr_settime = vrtc_settime;

	handle->bus_cookie = NULL;
	handle->todr_setwen = NULL;

	todr_attach(handle);
}

int
vrtc_gettime(todr_chip_handle_t handle, struct timeval *tv)
{
	u_int64_t tod;

	if (hv_tod_get(&tod) != H_EOK)
		return (1);

	tv->tv_sec = tod;
	tv->tv_usec = 0;
	return (0);
}

int
vrtc_settime(todr_chip_handle_t handle, struct timeval *tv)
{
	if (hv_tod_set(tv->tv_sec) != H_EOK)
		return (1);

	return (0);
}		
