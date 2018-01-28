/*	$NetBSD: db_machdep.h,v 1.1 2001/11/25 15:56:03 thorpej Exp $	*/

#ifdef __aarch64__
#include <aarch64/db_machdep.h>
#else
#include <arm/db_machdep.h>
#endif
