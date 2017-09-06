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

#ifndef _AARCH64_MACHDEP_H_
#define _AARCH64_MACHDEP_H_

extern paddr_t physical_start;
extern paddr_t physical_end;

extern void (*cpu_reset_address)(void);
extern void (*cpu_powerdown_address)(void);

extern char *booted_kernel;

void	lwp_trampoline(void);
void	cpu_dosoftints(void);
void	cpu_switchto_softint(struct lwp *, int);
void	trap(struct trapframe *, int);
void	interrupt(struct trapframe *);
void	dumpsys(void);
void	initarm64(void);
void	dosoftints(void);
paddr_t vtophys(vaddr_t);

#include <sys/pcu.h>

extern const pcu_ops_t pcu_fpu_ops;

static inline bool
fpu_used_p(lwp_t *l)
{
	return pcu_valid_p(&pcu_fpu_ops, l);
}

static inline void
fpu_discard(lwp_t *l, bool usesw)
{
	pcu_discard(&pcu_fpu_ops, l, usesw);
}

static inline void
fpu_save(lwp_t *l)
{
	pcu_save(&pcu_fpu_ops, l);
}

#endif /* _AARCH64_MACHDEP_H_ */
