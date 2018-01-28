/*	$NetBSD: profile.h,v 1.1 2001/11/25 15:56:05 thorpej Exp $	*/

#ifdef __aarch64__
#include <aarch64/profile.h>
#else
#include <arm/profile.h>
#endif
