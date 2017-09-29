/* $NetBSD: locore.h,v 1.2 2017/03/16 16:13:19 chs Exp $ */

/*-
 * Copyright (c) 2014 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Matt Thomas of 3am Software Foundry.
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

#ifndef _AARCH64_LOCORE_H_
#define _AARCH64_LOCORE_H_

#ifdef __aarch64__

#include <sys/types.h>

#include <sys/cpu.h>
#include <sys/lwp.h>
#include <sys/bus.h>

#include <aarch64/armreg.h>
#include <aarch64/frame.h>

#ifdef MULTIPROCESSOR
// XXXAARCH64
/* for compatibility arch/arm */
extern u_int arm_cpu_max;
#endif

static inline void daif_enable(register_t daif) __attribute__((__unused__));
static inline register_t daif_disable(register_t daif) __attribute__((__unused__));

static inline void
daif_enable(register_t daif)
{
	reg_daifclr_write(daif);
}

static inline register_t
daif_disable(register_t daif)
{
	uint32_t olddaif = reg_daif_read() >> 6; /* DAIF := PSTATE[9:6] */
	reg_daifset_write(daif);
	return olddaif;
}

#define ENABLE_INTERRUPT()	daif_enable(DAIF_I|DAIF_F)
#define DISABLE_INTERRUPT()	daif_disable(DAIF_I|DAIF_F)

/* for compatibility arch/arm/pic/pic.c -- might be modified to DIAF_I|F */
#define I32_bit	SPSR_I
#define F32_bit	SPSR_F
#define cpsie(ifbit)	daif_enable((ifbit)>>6)
#define cpsid(ifbit)	daif_disable((ifbit)>>6)

#elif defined(__arm__)

#include <arm/locore.h>

#endif /* __aarch64__/__arm__ */

#endif /* _AARCH64_LOCORE_H_ */
