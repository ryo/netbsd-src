/*	$NetBSD: frame.h,v 1.2 2001/11/25 15:56:04 thorpej Exp $	*/

#ifdef __aarch64__
#include <aarch64/frame.h>
#else
#include <arm/arm32/frame.h>
#endif
