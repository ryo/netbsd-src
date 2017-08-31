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

RCSID("$NetBSD$")

/* uint8_t generic_bs_r_1(x0:x, x1:addr, x2:offset) */
ENTRY_NP(generic_bs_r_1)
	dsb	sy
	ldrb	w0, [x2, x1]
	ret
END(generic_bs_r_1)

/* uint16_t generic_bs_r_2(x0:x, x1:addr, x2:offset) */
ENTRY_NP(generic_bs_r_2)
	dsb	sy
	ldrh	w0, [x2, x1]
	ret
END(generic_bs_r_2)

/* uint32_t generic_bs_r_4(x0:x, x1:addr, x2:offset) */
ENTRY_NP(generic_bs_r_4)
	dsb	sy
	ldr	w0, [x2, x1]
	ret
END(generic_bs_r_4)

/* uint64_t generic_bs_r_8(x0:x, x1:addr, x2:offset) */
ENTRY_NP(generic_bs_r_8)
	dsb	sy
	ldr	x0, [x2, x1]
	ret
END(generic_bs_r_8)

/* uint16_t generic_bs_r_2_swap(x0:x, x1:addr, x2:offset) */
ENTRY_NP(generic_bs_r_2_swap)
	dsb	sy
	ldrh	w0, [x2, x1]
	rev16	w0, w0
	ret
END(generic_bs_r_2_swap)

/* uint32_t generic_bs_r_4_swap(x0:x, x2:addr, x3:offset) */
ENTRY_NP(generic_bs_r_4_swap)
	dsb	sy
	ldr	w0, [x2, x1]
	rev32	x0, x0
	ret
END(generic_bs_r_4_swap)

/* uint64_t generic_bs_r_8_swap(x0:x, x1:addr, x2:offset) */
ENTRY_NP(generic_bs_r_8_swap)
	dsb	sy
	ldr	x0, [x2, x1]
	rev	x0, x0
	ret
END(generic_bs_r_8_swap)

/* void generic_bs_rm_1(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rm_1)
	cbz	x4, 9f
	dsb	sy
1:
	ldrb	w8, [x1, x2]
	subs	x4, x4, #1
	strb	w8, [x3], #1
	b.ne	1b
9:
	ret
END(generic_bs_rm_1)

/* void generic_bs_rm_2(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rm_2)
	cbz	x4, 2f
	dsb	sy
1:
	ldrh	w8, [x1, x2]
	subs	x4, x4, #1
	strh	w8, [x3], #2
	b.ne	1b
2:
	ret
END(generic_bs_rm_2)

/* void generic_bs_rm_4(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rm_4)
	cbz	x4, 2f
	dsb	sy
1:
	ldr	w8, [x1, x2]
	subs	x4, x4, #1
	str	w8, [x3], #4
	b.ne	1b
2:
	ret
END(generic_bs_rm_4)

/* void generic_bs_rm_8(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rm_8)
	cbz	x4, 9f
	dsb	sy
1:
	ldr	x8, [x1, x2]
	subs	x4, x4, #1
	str	x8, [x3], #8
	b.ne	1b
9:
	ret
END(generic_bs_rm_8)

/* void generic_bs_rm_2_swap(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rm_2_swap)
	cbz	x4, 9f
	dsb	sy
1:
	ldrh	w8, [x1, x2]
	subs	x4, x4, #1
	rev16	w8, w8
	strh	w8, [x3], #2
	b.ne	1b
9:
	ret
END(generic_bs_rm_2_swap)

/* void generic_bs_rm_4_swap(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rm_4_swap)
	cbz	x4, 9f
	dsb	sy
1:
	ldr	w8, [x1, x2]
	subs	x4, x4, #1
	rev32	x8, x8
	str	w8, [x3], #4
	b.ne	1b
9:
	ret
END(generic_bs_rm_4_swap)

/* void generic_bs_rm_8_swap(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rm_8_swap)
	cbz	x4, 9f
	dsb	sy
1:
	ldr	x8, [x1, x2]
	subs	x4, x4, #1
	rev	x8, x8
	str	x8, [x3], #8
	b.ne	1b
9:
	ret
END(generic_bs_rm_8_swap)

/* void generic_bs_rr_1(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rr_1)
	cbz	x4, 9f
	add	x8, x1, x2
	dsb	sy
1:
	ldrb	w9, [x8], #1
	subs	x4, x4, #1
	strb	w9, [x3], #1
	b.ne	1b
9:
	ret
END(generic_bs_rr_1)

/* void generic_bs_rr_2(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rr_2)
	cbz	x4, 9f
	add	x8, x1, x2
	dsb	sy
1:
	ldrh	w9, [x8], #2
	subs	x4, x4, #1
	strh	w9, [x3], #2
	b.ne	1b
9:
	ret
END(generic_bs_rr_2)

/* void generic_bs_rr_4(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rr_4)
	cbz	x4, 9f
	add	x8, x1, x2
	dsb	sy
1:
	ldr	w9, [x8], #4
	subs	x4, x4, #1
	str	w9, [x3], #4
	b.ne	1b
9:
	ret
END(generic_bs_rr_4)

/* void generic_bs_rr_8(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rr_8)
	cbz	x4, 9f
	add	x8, x1, x2
	dsb	sy
1:
	ldr	x9, [x8], #8
	subs	x4, x4, #1
	str	x9, [x3], #8
	b.ne	1b
9:
	ret
END(generic_bs_rr_8)

/* void generic_bs_rr_2_swap(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rr_2_swap)
	cbz	x4, 9f
	add	x8, x1, x2
	dsb	sy
1:
	ldrh	w9, [x8], #2
	subs	x4, x4, #1
	rev16	w9, w9
	strh	w9, [x3], #2
	b.ne	1b
9:
	ret
END(generic_bs_rr_2_swap)

/* void generic_bs_rr_4_swap(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rr_4_swap)
	cbz	x4, 9f
	add	x8, x1, x2
	dsb	sy
1:
	ldr	w9, [x8], #4
	subs	x4, x4, #1
	rev32	x9, x9
	str	w9, [x3], #4
	b.ne	1b
9:
	ret
END(generic_bs_rr_4_swap)

/* void generic_bs_rr_8_swap(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_rr_8_swap)
	add	x8, x4, x2
	cmp	x8, x2
	b.ls	9f
	add	x8, x2, x1
	dsb	sy
1:
	ldr	x9, [x8], #8
	subs	x4, x4, #1
	rev	x9, x9
	str	x9, [x3], #8
	b.ne	1b
9:
	ret
END(generic_bs_rr_8_swap)

/* generic_bs_sm_1(x0:x, x1:addr, x2:offset, x3:data, x4:count) */
ENTRY_NP(generic_bs_sm_1)
	cbz	x4, 9f
1:
	subs	x4, x4, #1
	strb	w3, [x1, x2]
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_sm_1)

/* generic_bs_sm_2(x0:x, x1:addr, x2:offset, x3:data, x4:count) */
ENTRY_NP(generic_bs_sm_2)
	cbz	x4, 9f
1:
	subs	x4, x4, #1
	strh	w3, [x1, x2]
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_sm_2)

/* generic_bs_sm_4(x0:x, x1:addr, x2:offset, x3:data, x4:count) */
ENTRY_NP(generic_bs_sm_4)
	cbz	x4, 9f
1:
	subs	x4, x4, #1
	str	w3, [x1, x2]
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_sm_4)

/* generic_bs_sm_8(x0:x, x1:addr, x2:offset, x3:data, x4:count) */
ENTRY_NP(generic_bs_sm_8)
	cbz	x4, 9f
1:
	subs	x4, x4, #1
	str	x3, [x1, x2]
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_sm_8)

/* generic_bs_sm_2_swap(x0:x, x1:addr, x2:offset, x3:data, x4:count) */
ENTRY_NP(generic_bs_sm_2_swap)
	cbz	x4, 9f
	rev16	w3, w3
1:
	subs	x4, x4, #1
	strh	w3, [x1, x2]
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_sm_2_swap)

/* generic_bs_sm_4_swap(x0:x, x1:addr, x2:offset, x3:data, x4:count) */
ENTRY_NP(generic_bs_sm_4_swap)
	cbz	x4, 9f
	rev32	x3, x3
1:
	subs	x4, x4, #1
	str	x3, [x1, x2]
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_sm_4_swap)

/* generic_bs_sm_8_swap(x0:x, x1:addr, x2:offset, x3:data, x4:count) */
ENTRY_NP(generic_bs_sm_8_swap)
	cbz	x4, 9f
	rev	x3, x3
1:
	subs	x4, x4, #1
	str	x3, [x1, x2]
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_sm_8_swap)

/* void generic_bs_w_1(x0:x, x1:addr, x2:offset, x3:data) */
ENTRY_NP(generic_bs_w_1)
	strb	w3, [x2, x1]
	dsb	sy
	ret
END(generic_bs_w_1)

/* void generic_bs_w_2(x0:x, x1:addr, x2:offset, x3:data) */
/* void generic_bs_w_2_swap(x0:x, x1:addr, x2:offset, x3:data) */
ENTRY_NP(generic_bs_w_2_swap)
	rev16	w3, w3
ENTRY_NP(generic_bs_w_2)
	strh	w3, [x2, x1]
	dsb	sy
	ret
END(generic_bs_w_2)
END(generic_bs_w_2_swap)

/* void generic_bs_w_4(x0:x, x1:addr, x2:offset, x3:data) */
/* void generic_bs_w_4_swap(x0:x, x1:addr, x2:offset, x3:data) */
ENTRY_NP(generic_bs_w_4_swap)
	rev32	x3, x3
ENTRY_NP(generic_bs_w_4)
	str	w3, [x2, x1]
	dsb	sy
	ret
END(generic_bs_w_4)
END(generic_bs_w_4_swap)

/* void generic_bs_w_8(x0:x, x1:addr, x2:offset, x3:data) */
/* void generic_bs_w_8_swap(x0:x, x1:addr, x2:offset, x3:data) */
ENTRY_NP(generic_bs_w_8_swap)
	rev	x3, x3
ENTRY_NP(generic_bs_w_8)
	str	x3, [x2, x1]
	dsb	sy
	ret
END(generic_bs_w_8)
END(generic_bs_w_8_swap)

/* generic_bs_wm_1(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wm_1)
	cbz	x4, 9f
1:
	ldrb	w8, [x3], #1
	subs	x4, x4, #1
	strb	w8, [x1, x2]
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wm_1)

/* generic_bs_wm_2(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wm_2)
	cbz	x4, 9f
1:
	ldrh	w8, [x3], #2
	subs	x4, x4, #1
	strh	w8, [x1, x2]
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wm_2)

/* generic_bs_wm_4(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wm_4)
	cbz	x4, 9f
1:
	ldr	w8, [x3], #4
	subs	x4, x4, #1
	str	w8, [x1, x2]
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wm_4)

/* generic_bs_wm_8(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wm_8)
	cbz	x4, 9f
1:
	ldr	x8, [x3], #8
	subs	x4, x4, #1
	str	x8, [x1, x2]
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wm_8)

/* generic_bs_wm_2_swap(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wm_2_swap)
	cbz	x4, 9f
1:
	ldrh	w8, [x3], #2
	subs	x4, x4, #1
	rev16	w8, w8
	strh	w8, [x1, x2]
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wm_2_swap)

/* generic_bs_wm_4_swap(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wm_4_swap)
	cbz	x4, 9f
1:
	ldr	w8, [x3], #4
	subs	x4, x4, #1
	rev32	x8, x8
	str	w8, [x1, x2]
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wm_4_swap)

/* generic_bs_wm_8_swap(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wm_8_swap)
	cbz	x4, 9f
1:
	ldr	x8, [x3], #8
	subs	x4, x4, #1
	rev	x8, x8
	str	x8, [x1, x2]
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wm_8_swap)

/* generic_bs_wr_1(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wr_1)
	cbz	x4, 9f
	add	x8, x1, x2
1:
	ldrb	w9, [x3], #1
	subs	x4, x4, #1
	strb	w9, [x8], #1
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wr_1)

/* generic_bs_wr_2(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wr_2)
	cbz	x4, 9f
	add	x8, x1, x2
1:
	ldrh	w9, [x3], #2
	subs	x4, x4, #1
	strh	w9, [x8], #2
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wr_2)

/* generic_bs_wr_4(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wr_4)
	cbz	x4, 9f
	add	x8, x1, x2
1:
	ldr	w9, [x3], #4
	subs	x4, x4, #1
	str	w9, [x8], #4
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wr_4)

/* generic_bs_wr_8(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wr_8)
	cbz	x4, 9f
	add	x8, x1, x2
1:
	ldr	x9, [x3], #8
	subs	x4, x4, #1
	str	x9, [x8], #8
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wr_8)

/* void generic_bs_wr_2_swap(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wr_2_swap)
	cbz	x4, 9f
	add	x8, x1, x2
1:
	ldrh	w9, [x3], #2
	subs	x4, x4, #1
	rev16	w9, w9
	strh	w9, [x8], #2
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wr_2_swap)

/* void generic_bs_wr_4_swap(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wr_4_swap)
	cbz	x4, 9f
	add	x8, x1, x2
1:
	ldr	w9, [x3], #4
	subs	x4, x4, #1
	rev32	x9, x9
	str	w9, [x8], #4
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wr_4_swap)

/* void generic_bs_wr_8_swap(x0:x, x1:addr, x2:offset, x3:datap, x4:count) */
ENTRY_NP(generic_bs_wr_8_swap)
	cbz	x4, 9f
	add	x8, x1, x2
1:
	ldr	x9, [x3], #8
	subs	x4, x4, #1
	rev	x9, x9
	str	x9, [x8], #8
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_wr_8_swap)

/* void generic_bs_sr_1(x0:x, x1:addr, x2:offset, x3:value, x4:count) */
ENTRY_NP(generic_bs_sr_1)
	cbz	x4, 9f
	add	x8, x1, x2
1:
	subs	x4, x4, #1
	strb	w3, [x8], #1
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_sr_1)

/* void generic_bs_sr_2(x0:x, x1:addr, x2:offset, x3:value, x4:count) */
/* void generic_bs_sr_2_swap(x0:x, x1:addr, x2:offset, x3:value, x4:count) */
ENTRY_NP(generic_bs_sr_2_swap)
	rev16	w3, w3
ENTRY_NP(generic_bs_sr_2)
	cbz	x4, 9f
	add	x8, x1, x2
1:
	subs	x4, x4, #1
	strh	w3, [x8], #2
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_sr_2)
END(generic_bs_sr_2_swap)

/* void generic_bs_sr_4(x0:x, x1:addr, x2:offset, x3:value, x4:count) */
/* void generic_bs_sr_4_swap(x0:x, x1:addr, x2:offset, x3:value, x4:count) */
ENTRY_NP(generic_bs_sr_4_swap)
	rev32	x3, x3
ENTRY_NP(generic_bs_sr_4)
	cbz	x4, 9f
	add	x8, x1, x2
1:
	subs	x4, x4, #1
	str	w3, [x8], #4
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_sr_4)
END(generic_bs_sr_4_swap)

/* void generic_bs_sr_8(x0:x, x1:addr, x2:offset, x3:value, x4:count) */
/* void generic_bs_sr_8_swap(x0:x, x1:addr, x2:offset, x3:value, x4:count) */
ENTRY_NP(generic_bs_sr_swap_8)
	rev	x3, x3
ENTRY_NP(generic_bs_sr_8)
	cbz	x4, 9f
	add	x8, x1, x2
1:
	subs	x4, x4, #1
	str	x3, [x8], #8
	b.ne	1b
	dsb	sy
9:
	ret
END(generic_bs_sr_8)
END(generic_bs_sr_8_swap)

/* void generic_bs_c_1(x0:x, x1:addr1, x2:offset1, x3:addr2, x4:offset2, x5:count) */
ENTRY_NP(generic_bs_c_1)
	cbz	x5, 9f
	add	x1, x1, x2	/* addr1 += offset1 */
	add	x3, x3, x4	/* addr2 += offset2 */
	cmp	x1, x3		/* addr1 > addr2 ? */
	b.cc	2f
1:
	/* in case of (addr1 > addr2) */
	ldrb	w9, [x1], #1
	subs	x5, x5, #1
	strb	w9, [x3], #1
	b.ne	1b
	dsb	sy
	b	9f
2:
	add	x1, x1, x5	/* addr1 += count */
	add	x3, x3, x5	/* addr2 += count */
3:
	ldrb	w9, [x1, #-1]!
	subs	x5, x5, #1
	strb	w9, [x3, #-1]!
	b.ne	3b
	dsb	sy
9:
	ret
END(generic_bs_c_1)

/* void generic_bs_c_2(x0:x, x1:addr1, x2:offset1, x3:addr2, x4:offset2, x5:count) */
ENTRY_NP(generic_bs_c_2)
	cbz	x5, 9f
	add	x1, x1, x2	/* addr1 += offset1 */
	add	x3, x3, x4	/* addr2 += offset2 */
	cmp	x1, x3		/* addr1 > addr2 ? */
	b.cc	2f
1:
	/* in case of (addr1 > addr2) */
	ldrh	w9, [x1], #2
	subs	x5, x5, #1
	strh	w9, [x3], #2
	b.ne	1b
	dsb	sy
	b	9f
2:
	add	x1, x1, x5	/* addr1 += count */
	add	x3, x3, x5	/* addr2 += count */
3:
	ldrh	w9, [x1, #-2]!
	subs	x5, x5, #1
	strh	w9, [x3, #-2]!
	b.ne	3b
	dsb	sy
9:
	ret
END(generic_bs_c_2)

/* void generic_bs_c_4(x0:x, x1:addr1, x2:offset1, x3:addr2, x4:offset2, x5:count) */
ENTRY_NP(generic_bs_c_4)
	cbz	x5, 9f
	add	x1, x1, x2	/* addr1 += offset1 */
	add	x3, x3, x4	/* addr2 += offset2 */
	cmp	x1, x3		/* addr1 > addr2 ? */
	b.cc	2f
1:
	/* in case of (addr1 > addr2) */
	ldr	w9, [x1], #4
	subs	x5, x5, #1
	str	w9, [x3], #4
	b.ne	1b
	dsb	sy
	b	9f
2:
	add	x1, x1, x5	/* addr1 += count */
	add	x3, x3, x5	/* addr2 += count */
3:
	ldr	w9, [x1, #-4]!
	subs	x5, x5, #1
	str	w9, [x3, #-4]!
	b.ne	3b
	dsb	sy
9:
	ret
END(generic_bs_c_4)

/* void generic_bs_c_8(x0:x, x1:addr1, x2:offset1, x3:addr2, x4:offset2, x5:count) */
ENTRY_NP(generic_bs_c_8)
	cbz	x5, 9f
	add	x1, x1, x2	/* addr1 += offset1 */
	add	x3, x3, x4	/* addr2 += offset2 */
	cmp	x1, x3		/* addr1 > addr2 ? */
	b.cc	2f
1:
	ldr	x9, [x1], #8
	subs	x5, x5, #1
	str	x9, [x3], #8
	b.ne	1b
	dsb	sy
	b	9f
2:
	add	x1, x1, x5	/* addr1 += count */
	add	x3, x3, x5	/* addr2 += count */
3:
	ldr	x9, [x1, #-8]!
	subs	x5, x5, #1
	str	x9, [x3, #-8]!
	b.ne	3b
	dsb	sy
9:
	ret
END(generic_bs_c_8)

