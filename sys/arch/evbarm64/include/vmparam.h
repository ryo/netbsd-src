/* $NetBSD: vmparam.h,v 1.1 2014/08/10 05:47:38 matt Exp $ */

#ifndef _EVBARM64_VMPARAM_H_
#define _EVBARM64_VMPARAM_H_

#include <aarch64/vmparam.h>

/*
 * 4GB minus  256MB of IO space
 */
#define	KERNEL_IO_VBASE	0xfffffffff0000000L
#define	KERNEL_IO_VSIZE (KERNEL_IO_VBASE - VM_MAX_KERNEL_ADDRESS)

#endif
