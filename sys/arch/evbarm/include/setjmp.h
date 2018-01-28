/*	$NetBSD: setjmp.h,v 1.1 2001/11/25 15:56:05 thorpej Exp $	*/

#ifdef __aarch64__
#include <arch64/setjmp.h>
#else
#include <arm/setjmp.h>
#endif
