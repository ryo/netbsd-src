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

#ifndef _EVBARM64_RPI_RPI_H_
#define _EVBARM64_RPI_RPI_H_

#include <aarch64/vmparam.h>
#include <arm/broadcom/bcm2835reg.h>

/* reserve devmap area from last 32Mbyte of KVA area */
#define RPI_DEVMAP_SIZE		(1024 * 1024 * 32) /* XXX: must be 2M aligned */
#define RPI_DEVMAP_VBASE	(VM_MAX_KERNEL_ADDRESS - RPI_DEVMAP_SIZE)

/*
 * BCM2835 ARM Peripherals
 */
#define RPI_KERNEL_IO_PBASE	BCM2835_PERIPHERALS_BASE
#define RPI_KERNEL_IO_VSIZE	BCM2835_PERIPHERALS_SIZE

/*
 * BCM2836 Local control block
 */
#define RPI_KERNEL_LOCAL_PBASE	BCM2836_ARM_LOCAL_BASE
#define RPI_KERNEL_LOCAL_VSIZE	BCM2836_ARM_LOCAL_SIZE


#define RPI_REF_FREQ	19200000

#endif /* _EVBARM64_RPI_RPI_H_ */
