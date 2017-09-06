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

#ifndef _AARCH64_BUS_SPACE_DEFS_H_
#define _AARCH64_BUS_SPACE_DEFS_H_

typedef uintptr_t bus_addr_t;
typedef uintptr_t bus_size_t;
typedef uintptr_t bus_space_handle_t;
typedef struct aarch64_bus_space *bus_space_tag_t;


#define BSTAG	void *
#define ADDR	bus_addr_t
#define HANDLE	bus_space_handle_t
#define SIZE	bus_size_t
#define OFFSET	bus_size_t
#define COUNT	bus_size_t

struct aarch64_bus_space {
	int bs_stride;	/* offset <<= bs_stride (if needed) */
	int bs_flags;

	int (*bs_map)(BSTAG, ADDR, SIZE, int, HANDLE *);
	void (*bs_unmap)(BSTAG, HANDLE, SIZE);
	int (*bs_subregion)(BSTAG, HANDLE, OFFSET, SIZE, HANDLE *);

	int (*bs_alloc)(BSTAG, ADDR, ADDR, SIZE, SIZE, SIZE,
	               int, ADDR *, HANDLE *);
	void (*bs_free)(BSTAG, HANDLE, SIZE);

	void *(*bs_vaddr)(BSTAG, HANDLE);
	paddr_t (*bs_mmap)(BSTAG, ADDR, off_t, int, int);

	void (*bs_barrier)(BSTAG, HANDLE, SIZE, SIZE, int);

	/* bus_space_read_[1248] */
	uint8_t  (*bs_r_1)(BSTAG, HANDLE, OFFSET);
	uint16_t (*bs_r_2)(BSTAG, HANDLE, OFFSET);
	uint32_t (*bs_r_4)(BSTAG, HANDLE, OFFSET);
	uint64_t (*bs_r_8)(BSTAG, HANDLE, OFFSET);

	/* bus_space_write_[1248] */
	void (*bs_w_1)(BSTAG, HANDLE, OFFSET, uint8_t);
	void (*bs_w_2)(BSTAG, HANDLE, OFFSET, uint16_t);
	void (*bs_w_4)(BSTAG, HANDLE, OFFSET, uint32_t);
	void (*bs_w_8)(BSTAG, HANDLE, OFFSET, uint64_t);

	/* bus_space_read_region_[1248] */
	void (*bs_rr_1)(BSTAG, HANDLE, OFFSET, uint8_t *, COUNT);
	void (*bs_rr_2)(BSTAG, HANDLE, OFFSET, uint16_t *, COUNT);
	void (*bs_rr_4)(BSTAG, HANDLE, OFFSET, uint32_t *, COUNT);
	void (*bs_rr_8)(BSTAG, HANDLE, OFFSET, uint64_t *, COUNT);

	/* bus_space_write_region_[1248] */
	void (*bs_wr_1)(BSTAG, HANDLE, OFFSET, const uint8_t *, COUNT);
	void (*bs_wr_2)(BSTAG, HANDLE, OFFSET, const uint16_t *, COUNT);
	void (*bs_wr_4)(BSTAG, HANDLE, OFFSET, const uint32_t *, COUNT);
	void (*bs_wr_8)(BSTAG, HANDLE, OFFSET, const uint64_t *, COUNT);

	/* bus_space_copy_region_[1248] */
	void (*bs_c_1)(BSTAG, HANDLE, OFFSET, HANDLE, OFFSET, COUNT);
	void (*bs_c_2)(BSTAG, HANDLE, OFFSET, HANDLE, OFFSET, COUNT);
	void (*bs_c_4)(BSTAG, HANDLE, OFFSET, HANDLE, OFFSET, COUNT);
	void (*bs_c_8)(BSTAG, HANDLE, OFFSET, HANDLE, OFFSET, COUNT);

	/* bus_spce_set_region_[1248] */
	void (*bs_sr_1)(BSTAG, HANDLE, OFFSET, uint8_t, COUNT);
	void (*bs_sr_2)(BSTAG, HANDLE, OFFSET, uint16_t, COUNT);
	void (*bs_sr_4)(BSTAG, HANDLE, OFFSET, uint32_t, COUNT);
	void (*bs_sr_8)(BSTAG, HANDLE, OFFSET, uint64_t, COUNT);

	/* bus_space_read_multi_[1248] */
	void (*bs_rm_1)(BSTAG, HANDLE, OFFSET, uint8_t *, COUNT);
	void (*bs_rm_2)(BSTAG, HANDLE, OFFSET, uint16_t *, COUNT);
	void (*bs_rm_4)(BSTAG, HANDLE, OFFSET, uint32_t *, COUNT);
	void (*bs_rm_8)(BSTAG, HANDLE, OFFSET, uint64_t *, COUNT);

	/* bus_space_write_multi_[1248] */
	void (*bs_wm_1)(BSTAG, HANDLE, OFFSET, const uint8_t *, COUNT);
	void (*bs_wm_2)(BSTAG, HANDLE, OFFSET, const uint16_t *, COUNT);
	void (*bs_wm_4)(BSTAG, HANDLE, OFFSET, const uint32_t *, COUNT);
	void (*bs_wm_8)(BSTAG, HANDLE, OFFSET, const uint64_t *, COUNT);

	/* bus_space_set_multi_[1248] */
	void (*bs_sm_1)(BSTAG, HANDLE, OFFSET, uint8_t, COUNT);
	void (*bs_sm_2)(BSTAG, HANDLE, OFFSET, uint16_t, COUNT);
	void (*bs_sm_4)(BSTAG, HANDLE, OFFSET, uint32_t, COUNT);
	void (*bs_sm_8)(BSTAG, HANDLE, OFFSET, uint64_t, COUNT);

	/* bus_space_peek_[1248] */
	int (*bs_pe_1)(BSTAG, HANDLE, OFFSET, uint8_t *);
	int (*bs_pe_2)(BSTAG, HANDLE, OFFSET, uint16_t *);
	int (*bs_pe_4)(BSTAG, HANDLE, OFFSET, uint32_t *);
	int (*bs_pe_8)(BSTAG, HANDLE, OFFSET, uint64_t *);

	/* bus_space_poke_[1248] */
	int (*bs_po_1)(BSTAG, HANDLE, OFFSET, uint8_t);
	int (*bs_po_2)(BSTAG, HANDLE, OFFSET, uint16_t);
	int (*bs_po_4)(BSTAG, HANDLE, OFFSET, uint32_t);
	int (*bs_po_8)(BSTAG, HANDLE, OFFSET, uint64_t);

#ifdef __BUS_SPACE_HAS_STREAM_METHODS
	/* bus_space_read_stream_[1248] */
	uint8_t  (*bs_r_1_s)(BSTAG, HANDLE, OFFSET);
	uint16_t (*bs_r_2_s)(BSTAG, HANDLE, OFFSET);
	uint32_t (*bs_r_4_s)(BSTAG, HANDLE, OFFSET);
	uint64_t (*bs_r_8_s)(BSTAG, HANDLE, OFFSET);

	/* bus_space_write_stream_[1248] */
	void (*bs_w_1_s)(BSTAG, HANDLE, OFFSET, uint8_t);
	void (*bs_w_2_s)(BSTAG, HANDLE, OFFSET, uint16_t);
	void (*bs_w_4_s)(BSTAG, HANDLE, OFFSET, uint32_t);
	void (*bs_w_8_s)(BSTAG, HANDLE, OFFSET, uint64_t);

	/* bus_space_read_region_stream[1248] */
	void (*bs_rr_1_s)(BSTAG, HANDLE, OFFSET, uint8_t *, COUNT);
	void (*bs_rr_2_s)(BSTAG, HANDLE, OFFSET, uint16_t *, COUNT);
	void (*bs_rr_4_s)(BSTAG, HANDLE, OFFSET, uint32_t *, COUNT);
	void (*bs_rr_8_s)(BSTAG, HANDLE, OFFSET, uint64_t *, COUNT);

	/* bus_space_write_region_stream_[1248] */
	void (*bs_wr_1_s)(BSTAG, HANDLE, OFFSET, const uint8_t *, COUNT);
	void (*bs_wr_2_s)(BSTAG, HANDLE, OFFSET, const uint16_t *, COUNT);
	void (*bs_wr_4_s)(BSTAG, HANDLE, OFFSET, const uint32_t *, COUNT);
	void (*bs_wr_8_s)(BSTAG, HANDLE, OFFSET, const uint64_t *, COUNT);

	/* bus_space_read_multi_stream_[1248] */
	void (*bs_rm_1_s)(BSTAG, HANDLE, OFFSET, uint8_t *, COUNT);
	void (*bs_rm_2_s)(BSTAG, HANDLE, OFFSET, uint16_t *, COUNT);
	void (*bs_rm_4_s)(BSTAG, HANDLE, OFFSET, uint32_t *, COUNT);
	void (*bs_rm_8_s)(BSTAG, HANDLE, OFFSET, uint64_t *, COUNT);

	/* bus_space_write_multi_stream_[1248] */
	void (*bs_wm_1_s)(BSTAG, HANDLE, OFFSET, const uint8_t *, COUNT);
	void (*bs_wm_2_s)(BSTAG, HANDLE, OFFSET, const uint16_t *, COUNT);
	void (*bs_wm_4_s)(BSTAG, HANDLE, OFFSET, const uint32_t *, COUNT);
	void (*bs_wm_8_s)(BSTAG, HANDLE, OFFSET, const uint64_t *, COUNT);
#endif /* __BUS_SPACE_HAS_STREAM_METHODS */
};

#undef BSTAG
#undef ADDR
#undef HANDLE
#undef SIZE
#undef OFFSET
#undef COUNT

#endif /* _AARCH64_BUS_SPACE_DEFS_H_ */
