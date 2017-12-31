/*	$NetBSD: bootconfig.h,v 1.8 2017/07/05 09:37:14 jmcneill Exp $	*/

#ifdef _KERNEL_OPT
#include "opt_machdep.h"
#endif

#include <arm/bootconfig.h>

#include <aarch64/bootconfig.h>

extern BootConfig bootconfig;

/* End of bootconfig.h */
