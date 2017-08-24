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

/* Define SP_EL1 stack sizes in pages */
#ifndef EL1_STACK_SIZE
#ifdef IPKDB
#define EL1_STACK_SIZE	2
#else /* IPKDB */
#define EL1_STACK_SIZE	1
#endif /* IPKDB */
#endif /* EL1_STACK_SIZE */

extern paddr_t physical_start;
extern paddr_t physical_end;

extern void (*cpu_reset_address)(void);
extern void (*cpu_powerdown_address)(void);

extern char *booted_kernel;

/* from aarch64_reboot.c */
void dumpsys(void);

/* from aarch64_kvminit.c */
vaddr_t aarch64_kvminit(void);


/* from aarch64/cpu_machdep.c */
void dosoftints(void);
vaddr_t cpu_proc0_init(vaddr_t);

#endif /* _AARCH64_MACHDEP_H_ */
