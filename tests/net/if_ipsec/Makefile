# $NetBSD: Makefile,v 1.3 2019/01/17 02:49:11 knakahara Exp $
#

.include <bsd.own.mk>

TESTSDIR=	${TESTSBASE}/net/if_ipsec

.for name in ipsec ipsec_natt ipsec_pfil
TESTS_SH+=		t_${name}
TESTS_SH_SRC_t_${name}=	../net_common.sh t_${name}.sh \
	../ipsec/common.sh ../ipsec/algorithms.sh
.endfor

.include <bsd.test.mk>
