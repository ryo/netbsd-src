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

#ifndef _AARCH64_BUS_FUNCS_H_
#define _AARCH64_BUS_FUNCS_H_

void bus_space_mallocok(void);

#define BUSOP(t, op, args...)			(*(t)->op)((t), ## args)

/* Bus Space macros */
#define bus_space_map(t, args...)		BUSOP(t, bs_map, ## args)
#define bus_space_unmap(t, args...)		BUSOP(t, bs_unmap, ## args)
#define bus_space_subregion(t, args...)		BUSOP(t, bs_subregion, ## args)
#define bus_space_alloc(t, args...)		BUSOP(t, bs_alloc, ## args)
#define bus_space_free(t, args...)		BUSOP(t, bs_free, ## args)
#define bus_space_vaddr(t, args...)		BUSOP(t, bs_vaddr, ## args)
#define bus_space_mmap(t, args...)		BUSOP(t, bs_mmap, ## args)
#define bus_space_barrier(t, args...)		BUSOP(t, bs_barrier, ## args)

#define bus_space_read_1(t, args...)		BUSOP(t, bs_r_1, ## args)
#define bus_space_read_2(t, args...)		BUSOP(t, bs_r_2, ## args)
#define bus_space_read_4(t, args...)		BUSOP(t, bs_r_4, ## args)
#define bus_space_read_8(t, args...)		BUSOP(t, bs_r_8, ## args)
#ifdef __BUS_SPACE_HAS_STREAM_METHODS
#define bus_space_read_stream_1(t, args...)	BUSOP(t, bs_r_s_1, ## args)
#define bus_space_read_stream_2(t, args...)	BUSOP(t, bs_r_s_2, ## args)
#define bus_space_read_stream_4(t, args...)	BUSOP(t, bs_r_s_4, ## args)
#define bus_space_read_stream_8(t, args...)	BUSOP(t, bs_r_s_8, ## args)
#endif

#define bus_space_read_multi_1(t, args...)	BUSOP(t, bs_rm_1, ## args)
#define bus_space_read_multi_2(t, args...)	BUSOP(t, bs_rm_2, ## args)
#define bus_space_read_multi_4(t, args...)	BUSOP(t, bs_rm_4, ## args)
#define bus_space_read_multi_8(t, args...)	BUSOP(t, bs_rm_8, ## args)
#ifdef __BUS_SPACE_HAS_STREAM_METHODS
#define bus_space_read_multi_stream_1(t, args...) BUSOP(t, bs_rm_s_1, ## args)
#define bus_space_read_multi_stream_2(t, args...) BUSOP(t, bs_rm_s_2, ## args)
#define bus_space_read_multi_stream_4(t, args...) BUSOP(t, bs_rm_s_4, ## args)
#define bus_space_read_multi_stream_8(t, args...) BUSOP(t, bs_rm_s_8, ## args)
#endif

#define bus_space_read_region_1(t, args...)	BUSOP(t, bs_rr_1, ## args)
#define bus_space_read_region_2(t, args...)	BUSOP(t, bs_rr_2, ## args)
#define bus_space_read_region_4(t, args...)	BUSOP(t, bs_rr_4, ## args)
#define bus_space_read_region_8(t, args...)	BUSOP(t, bs_rr_8, ## args)
#ifdef __BUS_SPACE_HAS_STREAM_METHODS
#define bus_space_read_region_stream_1(t, args...) BUSOP(t, bs_rr_s_1, ## args)
#define bus_space_read_region_stream_2(t, args...) BUSOP(t, bs_rr_s_2, ## args)
#define bus_space_read_region_stream_4(t, args...) BUSOP(t, bs_rr_s_4, ## args)
#define bus_space_read_region_stream_8(t, args...) BUSOP(t, bs_rr_s_8, ## args)
#endif

#define bus_space_write_1(t, args...)		BUSOP(t, bs_w_1, ## args)
#define bus_space_write_2(t, args...)		BUSOP(t, bs_w_2, ## args)
#define bus_space_write_4(t, args...)		BUSOP(t, bs_w_4, ## args)
#define bus_space_write_8(t, args...)		BUSOP(t, bs_w_8, ## args)
#ifdef __BUS_SPACE_HAS_STREAM_METHODS
#define bus_space_write_stream_1(t, args...)	BUSOP(t, bs_w_s_1, ## args)
#define bus_space_write_stream_2(t, args...)	BUSOP(t, bs_w_s_2, ## args)
#define bus_space_write_stream_4(t, args...)	BUSOP(t, bs_w_s_4, ## args)
#define bus_space_write_stream_8(t, args...)	BUSOP(t, bs_w_s_8, ## args)
#endif

#define bus_space_write_multi_1(t, args...)	BUSOP(t, bs_wm_1, ## args)
#define bus_space_write_multi_2(t, args...)	BUSOP(t, bs_wm_2, ## args)
#define bus_space_write_multi_4(t, args...)	BUSOP(t, bs_wm_4, ## args)
#define bus_space_write_multi_8(t, args...)	BUSOP(t, bs_wm_8, ## args)
#ifdef __BUS_SPACE_HAS_STREAM_METHODS
#define bus_space_write_multi_stream_1(t, args...) BUSOP(t, bs_wm_s_1, ## args)
#define bus_space_write_multi_stream_2(t, args...) BUSOP(t, bs_wm_s_2, ## args)
#define bus_space_write_multi_stream_4(t, args...) BUSOP(t, bs_wm_s_4, ## args)
#define bus_space_write_multi_stream_8(t, args...) BUSOP(t, bs_wm_s_8, ## args)
#endif

#define bus_space_write_region_1(t, args...)	BUSOP(t, bs_wr_1, ## args)
#define bus_space_write_region_2(t, args...)	BUSOP(t, bs_wr_2, ## args)
#define bus_space_write_region_4(t, args...)	BUSOP(t, bs_wr_4, ## args)
#define bus_space_write_region_8(t, args...)	BUSOP(t, bs_wr_8, ## args)
#ifdef __BUS_SPACE_HAS_STREAM_METHODS
#define bus_space_write_region_stream_1(t, args...) BUSOP(t, bs_wr_s_1, ## args)
#define bus_space_write_region_stream_2(t, args...) BUSOP(t, bs_wr_s_2, ## args)
#define bus_space_write_region_stream_4(t, args...) BUSOP(t, bs_wr_s_4, ## args)
#define bus_space_write_region_stream_8(t, args...) BUSOP(t, bs_wr_s_8, ## args)
#endif

#define bus_space_set_multi_1(t, args...)	BUSOP(t, bs_sm_1, ## args)
#define bus_space_set_multi_2(t, args...)	BUSOP(t, bs_sm_2, ## args)
#define bus_space_set_multi_4(t, args...)	BUSOP(t, bs_sm_4, ## args)
#define bus_space_set_multi_8(t, args...)	BUSOP(t, bs_sm_8, ## args)

#define bus_space_set_region_1(t, args...)	BUSOP(t, bs_sr_1, ## args)
#define bus_space_set_region_2(t, args...)	BUSOP(t, bs_sr_2, ## args)
#define bus_space_set_region_4(t, args...)	BUSOP(t, bs_sr_4, ## args)
#define bus_space_set_region_8(t, args...)	BUSOP(t, bs_sr_8, ## args)

#define bus_space_copy_region_1(t, args...)	BUSOP(t, bs_c_1, ## args)
#define bus_space_copy_region_2(t, args...)	BUSOP(t, bs_c_2, ## args)
#define bus_space_copy_region_4(t, args...)	BUSOP(t, bs_c_4, ## args)
#define bus_space_copy_region_8(t, args...)	BUSOP(t, bs_c_8, ## args)

#define bus_space_peek_1(t, args...)		BUSOP(t, bs_pe_1, ## args)
#define bus_space_peek_2(t, args...)		BUSOP(t, bs_pe_2, ## args)
#define bus_space_peek_4(t, args...)		BUSOP(t, bs_pe_4, ## args)
#define bus_space_peek_8(t, args...)		BUSOP(t, bs_pe_8, ## args)

#define bus_space_poke_1(t, args...)		BUSOP(t, bs_po_1, ## args)
#define bus_space_poke_2(t, args...)		BUSOP(t, bs_po_2, ## args)
#define bus_space_poke_4(t, args...)		BUSOP(t, bs_po_4, ## args)
#define bus_space_poke_8(t, args...)		BUSOP(t, bs_po_8, ## args)


/*
 * Macros to provide prototypes for all the functions used in the
 * bus_space structure
 */
#define bs_protos(f)							\
 int f##_bs_map(void *, bus_addr_t, bus_size_t, int, bus_space_handle_t *); \
 void f##_bs_unmap(void *, bus_space_handle_t, bus_size_t);		\
 int f##_bs_subregion(void *, bus_space_handle_t, bus_size_t,		\
                      bus_size_t, bus_space_handle_t *);		\
 int f##_bs_alloc(void *, bus_addr_t, bus_addr_t, bus_size_t, bus_size_t, \
                  bus_size_t, int, bus_addr_t *, bus_space_handle_t *);	\
 void f##_bs_free(void *, bus_space_handle_t, bus_size_t);		\
 void * f##_bs_vaddr(void *, bus_space_handle_t);			\
 paddr_t f##_bs_mmap(void *, bus_addr_t, off_t, int, int);		\
 void f##_bs_barrier(void *, bus_space_handle_t, bus_size_t, bus_size_t, int); \
 uint8_t  f##_bs_r_1(void *, bus_space_handle_t, bus_size_t);		\
 uint16_t f##_bs_r_2(void *, bus_space_handle_t, bus_size_t);		\
 uint32_t f##_bs_r_4(void *, bus_space_handle_t, bus_size_t);		\
 uint64_t f##_bs_r_8(void *, bus_space_handle_t, bus_size_t);		\
 uint16_t f##_bs_r_2_swap(void *, bus_space_handle_t, bus_size_t);	\
 uint32_t f##_bs_r_4_swap(void *, bus_space_handle_t, bus_size_t);	\
 uint64_t f##_bs_r_8_swap(void *, bus_space_handle_t, bus_size_t);	\
 void f##_bs_w_1(void *, bus_space_handle_t, bus_size_t, uint8_t);	\
 void f##_bs_w_2(void *, bus_space_handle_t, bus_size_t, uint16_t);	\
 void f##_bs_w_4(void *, bus_space_handle_t, bus_size_t, uint32_t);	\
 void f##_bs_w_8(void *, bus_space_handle_t, bus_size_t, uint64_t);	\
 void f##_bs_w_2_swap(void *, bus_space_handle_t,  bus_size_t, uint16_t); \
 void f##_bs_w_4_swap(void *, bus_space_handle_t, bus_size_t, uint32_t); \
 void f##_bs_w_8_swap(void *, bus_space_handle_t, bus_size_t, uint64_t); \
 void f##_bs_rm_1(void *, bus_space_handle_t, bus_size_t,		\
                  uint8_t *, bus_size_t);				\
 void f##_bs_rm_2(void *, bus_space_handle_t, bus_size_t,		\
                  uint16_t *,  bus_size_t);				\
 void f##_bs_rm_4(void *, bus_space_handle_t, bus_size_t,		\
                  uint32_t *, bus_size_t);				\
 void f##_bs_rm_8(void *, bus_space_handle_t, bus_size_t,		\
                  uint64_t *, bus_size_t);				\
 void f##_bs_rm_2_swap(void *, bus_space_handle_t, bus_size_t,		\
                       uint16_t *,  bus_size_t);			\
 void f##_bs_rm_4_swap(void *, bus_space_handle_t, bus_size_t,		\
                       uint32_t *, bus_size_t);				\
 void f##_bs_rm_8_swap(void *, bus_space_handle_t, bus_size_t,		\
                       uint64_t *, bus_size_t);				\
 void f##_bs_wm_1(void *, bus_space_handle_t, bus_size_t,		\
                  const uint8_t *, bus_size_t);				\
 void f##_bs_wm_2(void *, bus_space_handle_t, bus_size_t,		\
                  const uint16_t *, bus_size_t);			\
 void f##_bs_wm_4(void *, bus_space_handle_t, bus_size_t,		\
                  const uint32_t *, bus_size_t);			\
 void f##_bs_wm_8(void *, bus_space_handle_t, bus_size_t,		\
                  const uint64_t *, bus_size_t);			\
 void f##_bs_wm_2_swap(void *, bus_space_handle_t, bus_size_t,		\
                       const uint16_t *, bus_size_t);			\
 void f##_bs_wm_4_swap(void *, bus_space_handle_t, bus_size_t,		\
                       const uint32_t *, bus_size_t);			\
 void f##_bs_wm_8_swap(void *, bus_space_handle_t, bus_size_t,		\
                       const uint64_t *, bus_size_t);			\
 void f##_bs_rr_1(void *, bus_space_handle_t, bus_size_t,		\
                   uint8_t *, bus_size_t);				\
 void f##_bs_rr_2(void *, bus_space_handle_t, bus_size_t,		\
                  uint16_t *, bus_size_t);				\
 void f##_bs_rr_4(void *, bus_space_handle_t, bus_size_t,		\
                  uint32_t *, bus_size_t);				\
 void f##_bs_rr_8(void *, bus_space_handle_t, bus_size_t,		\
                  uint64_t *, bus_size_t);				\
 void f##_bs_rr_2_swap(void *, bus_space_handle_t, bus_size_t,		\
                       uint16_t *, bus_size_t);				\
 void f##_bs_rr_4_swap(void *, bus_space_handle_t, bus_size_t,		\
                       uint32_t *, bus_size_t);				\
 void f##_bs_rr_8_swap(void *, bus_space_handle_t, bus_size_t,		\
                       uint64_t *, bus_size_t);				\
 void f##_bs_wr_1(void *, bus_space_handle_t, bus_size_t,		\
                  const uint8_t *, bus_size_t);				\
 void f##_bs_wr_2(void *, bus_space_handle_t, bus_size_t,		\
                  const uint16_t *, bus_size_t);			\
 void f##_bs_wr_4(void *, bus_space_handle_t, bus_size_t,		\
                  const uint32_t *, bus_size_t);			\
 void f##_bs_wr_8(void *, bus_space_handle_t, bus_size_t,		\
                  const uint64_t *, bus_size_t);			\
 void f##_bs_wr_2_swap(void *, bus_space_handle_t, bus_size_t,		\
                       const uint16_t *, bus_size_t);			\
 void f##_bs_wr_4_swap(void *, bus_space_handle_t, bus_size_t,		\
                       const uint32_t *, bus_size_t);			\
 void f##_bs_wr_8_swap(void *, bus_space_handle_t, bus_size_t,		\
                       const uint64_t *, bus_size_t);			\
 void f##_bs_sm_1(void *, bus_space_handle_t, bus_size_t,		\
                  uint8_t, bus_size_t);					\
 void f##_bs_sm_2(void *, bus_space_handle_t, bus_size_t,		\
                   uint16_t, bus_size_t);				\
 void f##_bs_sm_4(void *, bus_space_handle_t, bus_size_t,		\
                  uint32_t, bus_size_t);				\
 void f##_bs_sm_8(void *, bus_space_handle_t, bus_size_t,		\
                  uint64_t, bus_size_t);				\
 void f##_bs_sr_1(void *, bus_space_handle_t, bus_size_t,		\
                  uint8_t, bus_size_t);					\
 void f##_bs_sr_2(void *, bus_space_handle_t, bus_size_t,		\
                  uint16_t, bus_size_t);				\
 void f##_bs_sr_4(void *, bus_space_handle_t, bus_size_t,		\
                  uint32_t, bus_size_t);				\
 void f##_bs_sr_8(void *, bus_space_handle_t, bus_size_t,		\
                  uint64_t, bus_size_t);				\
 void f##_bs_sr_2_swap(void *, bus_space_handle_t, bus_size_t,		\
                       uint16_t, bus_size_t);				\
 void f##_bs_sr_4_swap(void *, bus_space_handle_t, bus_size_t,		\
                       uint32_t, bus_size_t);				\
 void f##_bs_sr_8_swap(void *, bus_space_handle_t, bus_size_t,		\
                       uint64_t, bus_size_t);				\
 void f##_bs_c_1(void *, bus_space_handle_t, bus_size_t,		\
                 bus_space_handle_t, bus_size_t, bus_size_t);		\
 void f##_bs_c_2(void *, bus_space_handle_t, bus_size_t,		\
                 bus_space_handle_t, bus_size_t, bus_size_t);		\
 void f##_bs_c_4(void *, bus_space_handle_t, bus_size_t,		\
                 bus_space_handle_t, bus_size_t, bus_size_t);		\
 void f##_bs_c_8(void *, bus_space_handle_t, bus_size_t,		\
                 bus_space_handle_t, bus_size_t, bus_size_t);		\
 int f##_bs_pe_1(void *, bus_space_handle_t, bus_size_t, uint8_t *);	\
 int f##_bs_pe_2(void *, bus_space_handle_t, bus_size_t, uint16_t *);	\
 int f##_bs_pe_4(void *, bus_space_handle_t, bus_size_t, uint32_t *);	\
 int f##_bs_pe_8(void *, bus_space_handle_t, bus_size_t, uint64_t *);	\
 int f##_bs_po_1(void *, bus_space_handle_t, bus_size_t, uint8_t);	\
 int f##_bs_po_2(void *, bus_space_handle_t, bus_size_t, uint16_t);	\
 int f##_bs_po_4(void *, bus_space_handle_t, bus_size_t, uint32_t);	\
 int f##_bs_po_8(void *, bus_space_handle_t, bus_size_t, uint64_t);


/* Bus DMA macros */
#define bus_dmamap_create(t, args...)	BUSOP(t, _dmamap_create, ## args)
#define bus_dmamap_destroy(t, args...)	BUSOP(t, _dmamap_destroy, ## args)
#define bus_dmamap_load(t, args...)	BUSOP(t, _dmamap_load, ## args)
#define bus_dmamap_load_mbuf(t, args...) BUSOP(t, _dmamap_load_mbuf, ## args)
#define bus_dmamap_load_uio(t, args...)	BUSOP(t, _dmamap_uio, ## args)
#define bus_dmamap_load_raw(t, args...)	BUSOP(t, _dmamap_load_raw, ## args)
#define bus_dmamap_unload(t, args...)	BUSOP(t, _dmamap_unload, ## args)
#define bus_dmamap_sync(t, p, o, l, ops)				\
	do {								\
		if (((p)->_dm_flags &					\
		    (_BUS_DMAMAP_COHERENT|_BUS_DMAMAP_IS_BOUNCING)) ==	\
		    _BUS_DMAMAP_COHERENT)				\
			break;						\
		(*(t)->_dmamap_sync)((t), (p), (o), (l), (ops));	\
	} while (/*CONSTCOND*/ 0)
#define bus_dmamem_alloc(t, args...)	BUSOP(t, _dmamem_alloc, ## args)
#define bus_dmamem_free(t, args...)	BUSOP(t, _dmamem_free, ## args)
#define bus_dmamem_map(t, args...)	BUSOP(t, _dmamem_map, ## args)
#define bus_dmamem_unmap(t, args...)	BUSOP(t, _dmamem_unmap, ## args)
#define bus_dmamem_mmap(t, args...)	BUSOP(t, _dmamem_mmap, ## args)
#define bus_dmatag_subregion(t, args...) BUSOP(t, _dmamem_subregion, ## args)
#define bus_dmatag_destroy(t, args...)	BUSOP(t, _dmamem_destroy, ## args)


#ifdef _AARCH64_BUS_DMA_PRIVATE

struct mbuf;
struct uio;

int _bus_dmamap_create(bus_dma_tag_t, bus_size_t, int, bus_size_t, bus_size_t,
    int, bus_dmamap_t *);
void _bus_dmamap_destroy(bus_dma_tag_t, bus_dmamap_t);
int _bus_dmamap_load(bus_dma_tag_t, bus_dmamap_t, void *, bus_size_t,
    struct proc *, int);
int _bus_dmamap_load_mbuf(bus_dma_tag_t, bus_dmamap_t, struct mbuf *, int);
int _bus_dmamap_load_uio(bus_dma_tag_t, bus_dmamap_t, struct uio *, int);
int _bus_dmamap_load_raw(bus_dma_tag_t, bus_dmamap_t, bus_dma_segment_t *, int,
    bus_size_t, int);
void _bus_dmamap_unload(bus_dma_tag_t, bus_dmamap_t);
void _bus_dmamap_sync(bus_dma_tag_t, bus_dmamap_t, bus_addr_t, bus_size_t, int);

#define _BUS_DMAMAP_FUNCS					\
	._dmamap_create		= _bus_dmamap_create,		\
	._dmamap_destroy	= _bus_dmamap_destroy,		\
	._dmamap_load		= _bus_dmamap_load,		\
	._dmamap_load_mbuf	= _bus_dmamap_load_mbuf,	\
	._dmamap_load_raw	= _bus_dmamap_load_raw,		\
	._dmamap_load_uio	= _bus_dmamap_load_uio,		\
	._dmamap_unload		= _bus_dmamap_unload,		\
	._dmamap_sync		= _bus_dmamap_sync

int _bus_dmamem_alloc(bus_dma_tag_t, bus_size_t, bus_size_t, bus_size_t,
    bus_dma_segment_t *, int, int *, int);
void _bus_dmamem_free(bus_dma_tag_t, bus_dma_segment_t *, int);
int _bus_dmamem_map(bus_dma_tag_t, bus_dma_segment_t *, int, size_t, void **,
    int);
void _bus_dmamem_unmap(bus_dma_tag_t, void *, size_t);
paddr_t _bus_dmamem_mmap(bus_dma_tag_t, bus_dma_segment_t *, int, off_t, int,
    int);

#define _BUS_DMAMEM_FUNCS					\
	._dmamem_alloc =	_bus_dmamem_alloc,		\
	._dmamem_free =		_bus_dmamem_free,		\
	._dmamem_map =		_bus_dmamem_map,		\
	._dmamem_unmap =	_bus_dmamem_unmap,		\
	._dmamem_mmap =		_bus_dmamem_mmap

int _bus_dmamem_alloc_range(bus_dma_tag_t, bus_size_t, bus_size_t, bus_size_t,
    bus_dma_segment_t *, int, int *, int, vaddr_t, vaddr_t);
int _bus_dmatag_subregion(bus_dma_tag_t, bus_addr_t, bus_addr_t, bus_dma_tag_t *, int);
void _bus_dmatag_destroy(bus_dma_tag_t);

#define _BUS_DMATAG_FUNCS					\
	._dmatag_subregion =	_bus_dmatag_subregion,		\
	._dmatag_destroy =	_bus_dmatag_destroy

#endif /* _AARCH64_BUS_DMA_PRIVATE */

#endif /* _AARCH64_BUS_FUNCS_H_ */
