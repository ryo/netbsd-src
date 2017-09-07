/*	$NetBSD$	*/

/*
 * Copyright (c) 2017 Ryo Shimizu <ryo@nerv.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <aarch64/asm.h>
#include "assym.h"

RCSID("$NetBSD$")

/* uint8_t generic_bs_r_1(x0:tag, x1:addr, x2:offset) */
ENTRY_NP(generic_bs_r_1)
	dsb	sy
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
	ldrb	w0, [x1, x8]
	ret
END(generic_bs_r_1)

/* uint16_t generic_bs_r_2(x0:tag, x1:addr, x2:offset) */
ENTRY_NP(generic_bs_r_2)
	dsb	sy
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
	ldrh	w0, [x1, x8]
	ret
END(generic_bs_r_2)

/* uint32_t generic_bs_r_4(x0:tag, x1:addr, x2:offset) */
ENTRY_NP(generic_bs_r_4)
	dsb	sy
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
	ldr	w0, [x1, x8]
	ret
END(generic_bs_r_4)

/* uint64_t generic_bs_r_8(x0:tag, x1:addr, x2:offset) */
ENTRY_NP(generic_bs_r_8)
	dsb	sy
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
	ldr	x0, [x1, x8]
	ret
END(generic_bs_r_8)

/* uint16_t generic_bs_r_2_swap(x0:tag, x1:addr, x2:offset) */
ENTRY_NP(generic_bs_r_2_swap)
	dsb	sy
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
	ldrh	w0, [x1, x8]
	rev16	w0, w0
	ret
END(generic_bs_r_2_swap)

/* uint32_t generic_bs_r_4_swap(x0:tag, x2:addr, x3:offset) */
ENTRY_NP(generic_bs_r_4_swap)
	dsb	sy
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
	ldr	w0, [x1, x8]
	rev	w0, w0
	ret
END(generic_bs_r_4_swap)

/* uint64_t generic_bs_r_8_swap(x0:tag, x1:addr, x2:offset) */
ENTRY_NP(generic_bs_r_8_swap)
	dsb	sy
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
	ldr	x0, [x1, x8]
	rev	x0, x0
	ret
END(generic_bs_r_8_swap)

/* void generic_bs_rm_1(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rm_1)
	cbz	x4, 9f
	dsb	sy
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldrb	w9, [x1, x8]
	subs	x4, x4, #1	/* count-- */
	strb	w9, [x3], #1
	b.ne	1b
9:
	ret
END(generic_bs_rm_1)

/* void generic_bs_rm_2(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rm_2)
	cbz	x4, 9f
	dsb	sy
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldrh	w9, [x1, x8]
	subs	x4, x4, #1	/* count-- */
	strh	w9, [x3], #2
	b.ne	1b
9:
	ret
END(generic_bs_rm_2)

/* void generic_bs_rm_4(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rm_4)
	cbz	x4, 9f
	dsb	sy
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldr	w9, [x1, x8]
	subs	x4, x4, #1	/* count-- */
	str	w9, [x3], #4
	b.ne	1b
9:
	ret
END(generic_bs_rm_4)

/* void generic_bs_rm_8(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rm_8)
	cbz	x4, 9f
	dsb	sy
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldr	x9, [x1, x8]
	subs	x4, x4, #1	/* count-- */
	str	x9, [x3], #8
	b.ne	1b
9:
	ret
END(generic_bs_rm_8)

/* void generic_bs_rm_2_swap(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rm_2_swap)
	cbz	x4, 9f
	dsb	sy
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldrh	w9, [x1, x8]
	subs	x4, x4, #1	/* count-- */
	rev16	w9, w9
	strh	w9, [x3], #2
	b.ne	1b
9:
	ret
END(generic_bs_rm_2_swap)

/* void generic_bs_rm_4_swap(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rm_4_swap)
	cbz	x4, 9f
	dsb	sy
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldrh	w9, [x1, x8]
	subs	x4, x4, #1	/* count-- */
	rev	w9, w9
	str	w9, [x3], #4
	b.ne	1b
9:
	ret
END(generic_bs_rm_4_swap)

/* void generic_bs_rm_8_swap(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rm_8_swap)
	cbz	x4, 9f
	dsb	sy
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldrh	w9, [x1, x8]
	subs	x4, x4, #1	/* count-- */
	rev	x9, x9
	str	x9, [x3], #8
	b.ne	1b
9:
	ret
END(generic_bs_rm_8_swap)

/* void generic_bs_rr_1(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rr_1)
	cbz	x4, 9f
	dsb	sy
	ldr	w8, [x0, #BS_STRIDE]
	mov	x9, #1
	lsl	x9, x9, x8	/* delta = 1 << tag->bs_stride */
	lsl	x2, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldrb	w8, [x1, x2]	/* value = *src */
	subs	x4, x4, #1	/* count-- */
	add	x2, x2, x9	/* src += delta */
	strb	w8, [x3], #1	/* *dst++ = value */
	b.ne	1b
9:
	ret
END(generic_bs_rr_1)

/* void generic_bs_rr_2(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rr_2)
	cbz	x4, 9f
	dsb	sy
	ldr	w8, [x0, #BS_STRIDE]
	mov	x9, #1
	lsl	x9, x9, x8	/* delta = 1 << tag->bs_stride */
	cmp	x9, #2		/* if (delta < 2) { */
	bcs	0f		/*   delta = 2; */
	mov	x9, #2		/* } */
0:
	lsl	x2, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldrh	w8, [x1, x2]	/* value = *src */
	subs	x4, x4, #1	/* count-- */
	add	x2, x2, x9	/* src += delta */
	strh	w8, [x3], #2	/* *dst++ = value */
	b.ne	1b
9:
	ret
END(generic_bs_rr_2)

/* void generic_bs_rr_4(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rr_4)
	cbz	x4, 9f
	dsb	sy
	ldr	w8, [x0, #BS_STRIDE]
	mov	x9, #1
	lsl	x9, x9, x8	/* delta = 1 << tag->bs_stride */
	cmp	x9, #4		/* if (delta < 4) { */
	bcs	0f		/*   delta = 4; */
	mov	x9, #4		/* } */
0:
	lsl	x2, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldr	w8, [x1, x2]	/* value = *src */
	subs	x4, x4, #1	/* count-- */
	add	x2, x2, x9	/* src += delta */
	str	w8, [x3], #2	/* *dst++ = value */
	b.ne	1b
9:
	ret
END(generic_bs_rr_4)

/* void generic_bs_rr_8(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rr_8)
	cbz	x4, 9f
	dsb	sy
	ldr	w8, [x0, #BS_STRIDE]
	mov	x9, #1
	lsl	x9, x9, x8	/* delta = 1 << tag->bs_stride */
	cmp	x9, #8		/* if (delta < 8) { */
	bcs	0f		/*   delta = 8; */
	mov	x9, #8		/* } */
0:
	lsl	x2, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldr	x8, [x1, x2]	/* value = *src */
	subs	x4, x4, #1	/* count-- */
	add	x2, x2, x9	/* src += delta */
	str	x8, [x3], #2	/* *dst++ = value */
	b.ne	1b
9:
	ret
END(generic_bs_rr_8)

/* void generic_bs_rr_2_swap(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rr_2_swap)
	cbz	x4, 9f
	dsb	sy
	ldr	w8, [x0, #BS_STRIDE]
	mov	x9, #1
	lsl	x9, x9, x8	/* delta = 1 << tag->bs_stride */
	cmp	x9, #2		/* if (delta < 2) { */
	bcs	0f		/*   delta = 2; */
	mov	x9, #2		/* } */
0:
	lsl	x2, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldrh	w8, [x1, x9]	/* value = *src */
	subs	x4, x4, #1	/* count-- */
	add	x2, x2, x9	/* src += delta */
	rev16	w8, w8
	strh	w8, [x3], #2	/* *dst++ = value */
	b.ne	1b
9:
	ret
END(generic_bs_rr_2_swap)

/* void generic_bs_rr_4_swap(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rr_4_swap)
	cbz	x4, 9f
	dsb	sy
	ldr	w8, [x0, #BS_STRIDE]
	mov	x9, #1
	lsl	x9, x9, x8	/* delta = 1 << tag->bs_stride */
	cmp	x9, #4		/* if (delta < 4) { */
	bcs	0f		/*   delta = 4; */
	mov	x9, #4		/* } */
0:
	lsl	x2, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldr	w8, [x1, x9]	/* value = *src */
	subs	x4, x4, #1	/* count-- */
	add	x2, x2, x9	/* src += delta */
	rev	w8, w8
	str	w8, [x3], #2	/* *dst++ = value */
	b.ne	1b
9:
	ret
END(generic_bs_rr_4_swap)

/* void generic_bs_rr_8_swap(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rr_8_swap)
	cbz	x4, 9f
	dsb	sy
	ldr	w8, [x0, #BS_STRIDE]
	mov	x9, #1
	lsl	x9, x9, x8	/* delta = 1 << tag->bs_stride */
	cmp	x9, #8		/* if (delta < 8) { */
	bcs	0f		/*   delta = 8; */
	mov	x9, #8		/* } */
0:
	lsl	x2, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldr	x8, [x1, x9]	/* value = *src */
	subs	x4, x4, #1	/* count-- */
	add	x2, x2, x9	/* src += delta */
	rev	x8, x8
	str	x8, [x3], #2	/* *dst++ = value */
	b.ne	1b
9:
	ret
END(generic_bs_rr_8_swap)

/* void generic_bs_sm_1(x0:tag, x1:addr, x2:offset, x3:data, x4:count) */
ENTRY_NP(generic_bs_sm_1)
	cbz	x4, 9f
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
1:
	subs	x4, x4, #1	/* count-- */
	strb	w3, [x1, x8]
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_sm_1)

/* void generic_bs_sm_2(x0:tag, x1:addr, x2:offset, x3:data, x4:count) */
/* void generic_bs_sm_2_swap(x0:tag, x1:addr, x2:offset, x3:data, x4:count) */
ENTRY_NP(generic_bs_sm_2_swap)
	rev16	w3, w3
ENTRY_NP(generic_bs_sm_2)
	cbz	x4, 9f
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
1:
	subs	x4, x4, #1	/* count-- */
	strh	w3, [x1, x8]
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wm_2)
END(generic_bs_wm_2_swap)

/* void generic_bs_sm_4(x0:tag, x1:addr, x2:offset, x3:data, x4:count) */
/* void generic_bs_sm_4_swap(x0:tag, x1:addr, x2:offset, x3:data, x4:count) */
ENTRY_NP(generic_bs_sm_4_swap)
	rev	w3, w3
ENTRY_NP(generic_bs_sm_4)
	cbz	x4, 9f
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
1:
	subs	x4, x4, #1	/* count-- */
	str	w3, [x1, x8]
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_sm_4)
END(generic_bs_sm_4_swap)

/* void generic_bs_sm_8(x0:tag, x1:addr, x2:offset, x3:data, x4:count) */
/* void generic_bs_sm_8_swap(x0:tag, x1:addr, x2:offset, x3:data, x4:count) */
ENTRY_NP(generic_bs_sm_8_swap)
	rev	x3, x3
ENTRY_NP(generic_bs_sm_8)
	cbz	x4, 9f
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
1:
	subs	x4, x4, #1	/* count-- */
	str	x3, [x1, x8]
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_sm_8)
END(generic_bs_sm_8_swap)

/* void generic_bs_w_1(x0:tag, x1:addr, x2:offset, x3:data) */
ENTRY_NP(generic_bs_w_1)
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
	strb	w3, [x1, x8]
	dsb	sy
	ret
END(generic_bs_w_1)

/* void generic_bs_w_2(x0:tag, x1:addr, x2:offset, x3:data) */
/* void generic_bs_w_2_swap(x0:tag, x1:addr, x2:offset, x3:data) */
ENTRY_NP(generic_bs_w_2_swap)
	rev16	w3, w3
ENTRY_NP(generic_bs_w_2)
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
	strh	w3, [x1, x8]
	dsb	sy
	ret
END(generic_bs_w_2)
END(generic_bs_w_2_swap)

/* void generic_bs_w_4(x0:tag, x1:addr, x2:offset, x3:data) */
/* void generic_bs_w_4_swap(x0:tag, x1:addr, x2:offset, x3:data) */
ENTRY_NP(generic_bs_w_4_swap)
	rev	w3, w3
ENTRY_NP(generic_bs_w_4)
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
	str	w3, [x1, x8]
	dsb	sy
	ret
END(generic_bs_w_4)
END(generic_bs_w_4_swap)

/* void generic_bs_w_8(x0:tag, x1:addr, x2:offset, x3:data) */
/* void generic_bs_w_8_swap(x0:tag, x1:addr, x2:offset, x3:data) */
ENTRY_NP(generic_bs_w_8_swap)
	rev	x3, x3
ENTRY_NP(generic_bs_w_8)
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
	str	x3, [x1, x8]
	dsb	sy
	ret
END(generic_bs_w_8)
END(generic_bs_w_8_swap)

/* void generic_bs_wm_1(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wm_1)
	cbz	x4, 9f
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldrb	w9, [x3], #1
	subs	x4, x4, #1	/* count-- */
	strb	w9, [x1, x8]
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wm_1)

/* void generic_bs_wm_2(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wm_2)
	cbz	x4, 9f
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldrh	w9, [x3], #2
	subs	x4, x4, #1	/* count-- */
	strh	w9, [x1, x8]
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wm_2)

/* void generic_bs_wm_4(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wm_4)
	cbz	x4, 9f
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldr	w9, [x3], #4
	subs	x4, x4, #1	/* count-- */
	str	w9, [x1, x8]
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wm_4)

/* void generic_bs_wm_8(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wm_8)
	cbz	x4, 9f
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldr	x9, [x3], #8
	subs	x4, x4, #1	/* count-- */
	str	x9, [x1, x8]
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wm_8)

/* void generic_bs_wm_2_swap(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wm_2_swap)
	cbz	x4, 9f
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldrh	w9, [x3], #2
	subs	x4, x4, #1	/* count-- */
	rev16	w9, w9
	strh	w9, [x1, x8]
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wm_2_swap)

/* void generic_bs_wm_4_swap(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wm_4_swap)
	cbz	x4, 9f
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldr	w9, [x3], #4
	subs	x4, x4, #1	/* count-- */
	rev	w9, w9
	str	w9, [x1, x8]
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wm_4_swap)

/* void generic_bs_wm_8_swap(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wm_8_swap)
	cbz	x4, 9f
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x8, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldr	x9, [x3], #8
	subs	x4, x4, #1	/* count-- */
	rev	x9, x9
	str	x9, [x1, x8]
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wm_8_swap)

/* void generic_bs_wr_1(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wr_1)
	cbz	x4, 9f
	ldr	w8, [x0, #BS_STRIDE]
	mov	x9, #1
	lsl	x9, x9, x8	/* delta = 1 << tag->bs_stride */
	lsl	x2, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldrb	w8, [x3], #1	/* value = *src++ */
	subs	x4, x4, #1	/* count-- */
	strb	w8, [x1, x2]	/* *dst = value */
	add	x2, x2, x9	/* dst += delta */
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wr_1)

/* void generic_bs_wr_2(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wr_2)
	cbz	x4, 9f
	ldr	w8, [x0, #BS_STRIDE]
	mov	x9, #1
	lsl	x9, x9, x8	/* delta = 1 << tag->bs_stride */
	cmp	x9, #2		/* if (delta < 2) { */
	bcs	0f		/*   delta = 2; */
	mov	x9, #2		/* } */
0:
	lsl	x2, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldrh	w8, [x3], #2	/* value = *src++ */
	subs	x4, x4, #1	/* count-- */
	strh	w8, [x1, x2]	/* *dst = value */
	add	x2, x2, x9	/* dst += delta */
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wr_2)

/* void generic_bs_wr_4(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wr_4)
	cbz	x4, 9f
	ldr	w8, [x0, #BS_STRIDE]
	mov	x9, #1
	lsl	x9, x9, x8	/* delta = 1 << tag->bs_stride */
	cmp	x9, #4		/* if (delta < 4) { */
	bcs	0f		/*   delta = 4; */
	mov	x9, #4		/* } */
0:
	lsl	x2, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldr	w8, [x3], #4	/* value = *src++ */
	subs	x4, x4, #1	/* count-- */
	str	w8, [x1, x2]	/* *dst = value */
	add	x2, x2, x9	/* dst += delta */
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wr_4)

/* void generic_bs_wr_8(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wr_8)
	cbz	x4, 9f
	ldr	w8, [x0, #BS_STRIDE]
	mov	x9, #1
	lsl	x9, x9, x8	/* delta = 1 << tag->bs_stride */
	cmp	x9, #8		/* if (delta < 8) { */
	bcs	0f		/*   delta = 8; */
	mov	x9, #8		/* } */
0:
	lsl	x2, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldr	x8, [x3], #8	/* value = *src++ */
	subs	x4, x4, #1	/* count-- */
	str	x8, [x1, x2]	/* *dst = value */
	add	x2, x2, x9	/* dst += delta */
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wr_8)

/* void generic_bs_wr_2_swap(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wr_2_swap)
	cbz	x4, 9f
	ldr	w8, [x0, #BS_STRIDE]
	mov	x9, #1
	lsl	x9, x9, x8	/* delta = 1 << tag->bs_stride */
	cmp	x9, #2		/* if (delta < 2) { */
	bcs	0f		/*   delta = 2; */
	mov	x9, #2		/* } */
0:
	lsl	x2, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldrh	w8, [x3], #2	/* value = *src++ */
	rev16	w8, w8
	subs	x4, x4, #1	/* count-- */
	strh	w8, [x1, x2]	/* *dst = value */
	add	x2, x2, x9	/* dst += delta */
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wr_2_swap)

/* void generic_bs_wr_4_swap(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wr_4_swap)
	cbz	x4, 9f
	ldr	w8, [x0, #BS_STRIDE]
	mov	x9, #1
	lsl	x9, x9, x8	/* delta = 1 << tag->bs_stride */
	cmp	x9, #4		/* if (delta < 4) { */
	bcs	0f		/*   delta = 4; */
	mov	x9, #4		/* } */
0:
	lsl	x2, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldr	w8, [x3], #4	/* value = *src++ */
	rev	w8, w8
	subs	x4, x4, #1	/* count-- */
	str	w8, [x1, x2]	/* *dst = value */
	add	x2, x2, x9	/* dst += delta */
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wr_4_swap)

/* void generic_bs_wr_8_swap(x0:tag, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wr_8_swap)
	cbz	x4, 9f
	ldr	w8, [x0, #BS_STRIDE]
	mov	x9, #1
	lsl	x9, x9, x8	/* delta = 1 << tag->bs_stride */
	cmp	x9, #8		/* if (delta < 8) { */
	bcs	0f		/*   delta = 8; */
	mov	x9, #8		/* } */
0:
	lsl	x2, x2, x8	/* offset <<= tag->bs_stride */
1:
	ldr	x8, [x3], #8	/* value = *src++ */
	rev	x8, x8
	subs	x4, x4, #1	/* count-- */
	str	x8, [x1, x2]	/* *dst = value */
	add	x2, x2, x9	/* dst += delta */
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wr_8_swap)

/* void generic_bs_sr_1(x0:tag, x1:addr, x2:offset, x3:value, x4:count) */
ENTRY_NP(generic_bs_sr_1)
	cbz	x4, 9f
	ldr	w8, [x0, #BS_STRIDE]
	mov	x9, #1
	lsl	x9, x9, x8	/* delta = 1 << tag->bs_stride */
	lsl	x2, x2, x8	/* offset <<= tag->bs_stride */
1:
	strb	w3, [x1, x2]	/* *dst = value */
	subs	x4, x4, #1	/* count-- */
	add	x2, x2, x9	/* dst += delta */
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_sr_1)

/* void generic_bs_sr_2(x0:tag, x1:addr, x2:offset, x3:value, x4:count) */
/* void generic_bs_sr_2_swap(x0:tag, x1:addr, x2:offset, x3:value, x4:count) */
ENTRY_NP(generic_bs_sr_2_swap)
	rev16	w3, w3
ENTRY_NP(generic_bs_sr_2)
	cbz	x4, 9f
	ldr	w8, [x0, #BS_STRIDE]
	mov	x9, #1
	lsl	x9, x9, x8	/* delta = 1 << tag->bs_stride */
	cmp	x9, #2		/* if (delta < 2) { */
	bcs	0f		/*   delta = 2; */
	mov	x9, #2		/* } */
0:
	lsl	x2, x2, x8	/* offset <<= tag->bs_stride */
1:
	strh	w3, [x1, x2]	/* *dst = value */
	subs	x4, x4, #1	/* count-- */
	add	x2, x2, x9	/* dst += delta */
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_sr_2)
END(generic_bs_sr_2_swap)

/* void generic_bs_sr_4(x0:tag, x1:addr, x2:offset, x3:value, x4:count) */
/* void generic_bs_sr_4_swap(x0:tag, x1:addr, x2:offset, x3:value, x4:count) */
ENTRY_NP(generic_bs_sr_4_swap)
	rev	w3, w3
ENTRY_NP(generic_bs_sr_4)
	cbz	x4, 9f
	ldr	w8, [x0, #BS_STRIDE]
	mov	x9, #1
	lsl	x9, x9, x8	/* delta = 1 << tag->bs_stride */
	cmp	x9, #4		/* if (delta < 4) { */
	bcs	0f		/*   delta = 4; */
	mov	x9, #4		/* } */
0:
	lsl	x2, x2, x8	/* offset <<= tag->bs_stride */
1:
	str	w3, [x1, x2]	/* *dst = value */
	subs	x4, x4, #1	/* count-- */
	add	x2, x2, x9	/* dst += delta */
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_sr_4)
END(generic_bs_sr_4_swap)

/* void generic_bs_sr_8(x0:tag, x1:addr, x2:offset, x3:value, x4:count) */
/* void generic_bs_sr_8_swap(x0:tag, x1:addr, x2:offset, x3:value, x4:count) */
ENTRY_NP(generic_bs_sr_8_swap)
	rev	x3, x3
ENTRY_NP(generic_bs_sr_8)
	cbz	x4, 9f
	ldr	w8, [x0, #BS_STRIDE]
	mov	x9, #1
	lsl	x9, x9, x8	/* delta = 1 << tag->bs_stride */
	cmp	x9, #8		/* if (delta < 8) { */
	bcs	0f		/*   delta = 8; */
	mov	x9, #8		/* } */
0:
	lsl	x2, x2, x8	/* offset <<= tag->bs_stride */
1:
	str	x3, [x1, x2]	/* *dst = value */
	subs	x4, x4, #1	/* count-- */
	add	x2, x2, x9	/* dst += delta */
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_sr_8)
END(generic_bs_sr_8_swap)

/* void generic_bs_c_1(x0:tag, x1:addr1, x2:offset1, x3:addr2, x4:offset2, x5:count) */
ENTRY_NP(generic_bs_c_1)
	cbz	x5, 9f
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x2, x2, x8	/* offset1 <<= tag->bs_stride */
	lsl	x4, x4, x8	/* offset2 <<= tag->bs_stride */
	add	x1, x1, x2	/* addr1 += offset1 */
	add	x3, x3, x4	/* addr2 += offset2 */
	mov	x10, #1
	lsl	x10, x10, x8	/* delta = (1 << tag->bs_stride) */
	cmp	x1, x3		/* addr1 > addr2 ? */
	b.cc	2f
1:
	/* in case of (addr1 > addr2) */
	ldrb	w9, [x1]
	subs	x5, x5, #1
	strb	w9, [x3]
	add	x1, x1, x10	/* addr1 += delta */
	add	x3, x3, x10	/* addr2 += delta */
	b.ne	1b
	b	8f
2:
	lsl	x8, x5, x8	/* x8 = count << tag->bs_stride */
	add	x1, x1, x8	/* addr1 += count << tag->bs_stride */
	add	x3, x3, x8	/* addr2 += count << tag->bs_stride */
3:
	sub	x1, x1, x10	/* addr1 -= delta */
	sub	x3, x3, x10	/* addr2 -= delta */
	ldrb	w9, [x1]
	subs	x5, x5, #1
	strb	w9, [x3]
	b.ne	3b
8:	dsb	sy
9:
	ret
END(generic_bs_c_1)

/* void generic_bs_c_2(x0:tag, x1:addr1, x2:offset1, x3:addr2, x4:offset2, x5:count) */
ENTRY_NP(generic_bs_c_2)
	cbz	x5, 9f
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x2, x2, x8	/* offset1 <<= tag->bs_stride */
	lsl	x4, x4, x8	/* offset2 <<= tag->bs_stride */
	add	x1, x1, x2	/* addr1 += offset1 */
	add	x3, x3, x4	/* addr2 += offset2 */
	mov	x10, #1
	lsl	x10, x10, x8	/* delta = (1 << tag->bs_stride) */
	cmp	x10, #2		/* if (delta < 2) { */
	bcs	0f		/*   delta = 2; */
	mov	x10, #2		/* } */
0:
	cmp	x1, x3		/* addr1 > addr2 ? */
	b.cc	2f
1:
	/* in case of (addr1 > addr2) */
	ldrh	w9, [x1]
	subs	x5, x5, #1
	strh	w9, [x3]
	add	x1, x1, x10	/* addr1 += delta */
	add	x3, x3, x10	/* addr1 += delta */
	b.ne	1b
	b	8f
2:
	lsl	x8, x5, x8	/* x8 = count << tag->bs_stride */
	add	x1, x1, x8	/* addr1 += count << tag->bs_stride */
	add	x3, x3, x8	/* addr2 += count << tag->bs_stride */
3:
	sub	x1, x1, x10	/* addr1 -= delta */
	sub	x3, x3, x10	/* addr1 -= delta */
	ldrh	w9, [x1]
	subs	x5, x5, #1
	strh	w9, [x3]
	b.ne	3b
8:	dsb	sy
9:
	ret
END(generic_bs_c_2)

/* void generic_bs_c_4(x0:tag, x1:addr1, x2:offset1, x3:addr2, x4:offset2, x5:count) */
ENTRY_NP(generic_bs_c_4)
	cbz	x5, 9f
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x2, x2, x8	/* offset1 <<= tag->bs_stride */
	lsl	x4, x4, x8	/* offset2 <<= tag->bs_stride */
	add	x1, x1, x2	/* addr1 += offset1 */
	add	x3, x3, x4	/* addr2 += offset2 */
	mov	x10, #1
	lsl	x10, x10, x8	/* delta = (1 << tag->bs_stride) */
	cmp	x10, #4		/* if (delta < 4) { */
	bcs	0f		/*   delta = 4; */
	mov	x10, #4		/* } */
0:
	cmp	x1, x3		/* addr1 > addr2 ? */
	b.cc	2f
1:
	/* in case of (addr1 > addr2) */
	ldr	w9, [x1]
	subs	x5, x5, #1
	str	w9, [x3]
	add	x1, x1, x10	/* addr1 += delta */
	add	x3, x3, x10	/* addr2 += delta */
	b.ne	1b
	dsb	sy
	b	9f
2:
	lsl	x8, x5, x8	/* x8 = count << tag->bs_stride */
	add	x1, x1, x5	/* addr1 += count << tag->bs_delta */
	add	x3, x3, x5	/* addr2 += count << tag->bs_delta */
3:
	sub	x1, x1, x10	/* addr1 -= delta */
	sub	x3, x3, x10	/* addr1 -= delta */
	ldr	w9, [x1]
	subs	x5, x5, #1
	str	w9, [x3]
	b.ne	3b
	dsb	sy
9:
	ret
END(generic_bs_c_4)

/* void generic_bs_c_8(x0:tag, x1:addr1, x2:offset1, x3:addr2, x4:offset2, x5:count) */
ENTRY_NP(generic_bs_c_8)
	cbz	x5, 9f
	ldr	w8, [x0, #BS_STRIDE]
	lsl	x2, x2, x8	/* offset1 <<= tag->bs_stride */
	lsl	x4, x4, x8	/* offset2 <<= tag->bs_stride */
	add	x1, x1, x2	/* addr1 += offset1 */
	add	x3, x3, x4	/* addr2 += offset2 */
	mov	x10, #1
	lsl	x10, x10, x8	/* delta = (1 << tag->bs_stride) */
	cmp	x10, #8		/* if (delta < 8) { */
	bcs	0f		/*   delta = 8; */
	mov	x10, #8		/* } */
0:
	cmp	x1, x3		/* addr1 > addr2 ? */
	b.cc	2f
1:
	/* in case of (addr1 > addr2) */
	ldr	x9, [x1]
	subs	x5, x5, #1
	str	x9, [x3]
	add	x1, x1, x10	/* addr1 += delta */
	add	x3, x3, x10	/* addr1 += delta */
	b.ne	1b
	b	8f
2:
	lsl	x8, x5, x8	/* x8 = count << tag->bs_stride */
	add	x1, x1, x8	/* addr1 += count << tag->bs_stride */
	add	x3, x3, x8	/* addr2 += count << tag->bs_stride */
3:
	sub	x1, x1, x10	/* addr1 -= delta */
	sub	x3, x3, x10	/* addr1 -= delta */
	ldr	x9, [x1]
	subs	x5, x5, #1
	str	x9, [x3]
	b.ne	3b
8:	dsb	sy
9:
	ret
END(generic_bs_c_8)
