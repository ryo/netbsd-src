/*	$NetBSD: pmap.h,v 1.3 2001/11/25 15:56:05 thorpej Exp $	*/

#ifdef __aarch64__
#include <aarch64/pmap.h>
#else
#include <arm/arm32/pmap.h>
#endif
