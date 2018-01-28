/*	$NetBSD: lock.h,v 1.1 2001/11/25 15:56:05 thorpej Exp $	*/

// XXXNH arm/lock.h has v8 stuff already but is a bit of a mess atm
#ifdef __aarch64__
#include <aarch64/lock.h>
#else
#include <arm/lock.h>
#endif
