/*	$NetBSD: elf_machdep.h,v 1.1 2001/11/25 15:56:04 thorpej Exp $	*/

#ifdef __aarch64__
#include <aarch64/elf_machdep.h>
#else
#include <arm/elf_machdep.h>
#endif
