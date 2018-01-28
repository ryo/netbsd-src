/*	$NetBSD: mcontext.h,v 1.2 2003/01/17 22:45:39 thorpej Exp $	*/

#ifdef __aarch64__
#include <aarch64/mcontext.h>
#else
#include <arm/mcontext.h>
#endif
