# $NetBSD$
#
# This machine description file is used to generate the default NetBSD
# kernel.
#
# The machine description file can be customised for your specific
# machine to reduce the kernel size and improve its performance.
#
# For further information on compiling NetBSD kernels, see the config(8)
# man page.
#
# For further information on hardware support for this architecture, see
# the intro(4) man page.  For further information about kernel options
# for this architecture, see the options(4) man page.  For an explanation
# of each device driver in this file see the section 4 man page for the
# device.

include 	"arch/evbarm64/conf/RPI3"

ident		"RPi3.MP-$Revision: 0.99 $"

options		MULTIPROCESSOR		# include multiprocessor support
