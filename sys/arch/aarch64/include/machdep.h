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

#include <sys/siginfo.h>

#define KERN_VTOPHYS(va)	((paddr_t)((vaddr_t)(va) - VM_MIN_KERNEL_ADDRESS))
#define KERN_PHYSTOV(pa)	((vaddr_t)((paddr_t)(pa) + VM_MIN_KERNEL_ADDRESS))

extern paddr_t physical_start;
extern paddr_t physical_end;

extern void (*cpu_reset_address0)(void);
extern void (*cpu_reset_address)(void);
extern void (*cpu_powerdown_address)(void);

extern char *booted_kernel;

/* aarch64_machdep.c */
void initarm64(void);
void aarch64_cpu_configured(void);
void dumpsys(void);

struct trapframe;

/* fault.c */
bool data_abort_handler(struct trapframe *, ksiginfo_t *, uint32_t, const char *);

/* trap.c */
struct faultbuf;
void trap_ksi_init(ksiginfo_t *, int, int, vaddr_t, int);
int cpu_set_onfault(struct faultbuf *, register_t) __returns_twice;
void cpu_jump_onfault(struct trapframe *, const struct faultbuf *);
void cpu_unset_onfault(void);
struct faultbuf *cpu_get_onfault(void);
struct faultbuf *cpu_disable_onfault(void);
void cpu_enable_onfault(struct faultbuf *);
void lwp_trampoline(void);
void cpu_dosoftints(void);
void cpu_switchto_softint(struct lwp *, int);
void trap_doast(struct trapframe *);

void trap_el1t_sync(struct trapframe *);
void trap_el1t_irq(struct trapframe *);
void trap_el1t_fiq(struct trapframe *);
void trap_el1t_error(struct trapframe *);
void trap_el1h_sync(struct trapframe *);
void trap_el1h_fiq(struct trapframe *);
void trap_el1h_error(struct trapframe *);
void trap_el0_sync(struct trapframe *);
void trap_el0_fiq(struct trapframe *);
void trap_el0_error(struct trapframe *);
void trap_el0_32sync(struct trapframe *);
void trap_el0_32fiq(struct trapframe *);
void trap_el0_32error(struct trapframe *);

void interrupt(struct trapframe *);
void dosoftints(void);

struct fpreg;
void load_fpregs(struct fpreg *);
void save_fpregs(struct fpreg *);

#include <sys/pcu.h>

extern const pcu_ops_t pcu_fpu_ops;

static inline bool
fpu_used_p(lwp_t *l)
{
	return pcu_valid_p(&pcu_fpu_ops, l);
}

static inline void
fpu_save(lwp_t *l)
{
	pcu_save(&pcu_fpu_ops, l);
}

static inline void
fpu_load(lwp_t *l)
{
	pcu_load(&pcu_fpu_ops);
}

static inline void
fpu_discard(lwp_t *l, bool usesw)
{
	pcu_discard(&pcu_fpu_ops, l, usesw);
}

#endif /* _AARCH64_MACHDEP_H_ */
