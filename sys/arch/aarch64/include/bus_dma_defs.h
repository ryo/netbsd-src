/*	$NetBSD$	*/

/*-
 * Copyright (c) 1996, 1997, 1998, 2001 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Jason R. Thorpe of the Numerical Aerospace Simulation Facility,
 * NASA Ames Research Center.
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
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Copyright (c) 1996 Charles M. Hannum.  All rights reserved.
 * Copyright (c) 1996 Christopher G. Demetriou.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by Christopher G. Demetriou
 *	for the NetBSD Project.
 * 4. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _AARCH64_BUS_DMA_DEFS_H_
#define _AARCH64_BUS_DMA_DEFS_H_

typedef struct aarch64_bus_dma_segment bus_dma_segment_t;
typedef struct aarch64_bus_dma_tag *bus_dma_tag_t;
typedef struct aarch64_bus_dmamap *bus_dmamap_t;

/* used in some device drivers */
#define BUS_SPACE_ALIGNED_POINTER(p, t)	ALIGNED_POINTER(p, t)

/* used in dev/pci/pci.c */
#define BUS_DMA_TAG_VALID(t)	((t) != (bus_dma_tag_t)NULL)

/*
 * Private flags stored in the DMA map.
 */
#define _BUS_DMAMAP_COHERENT	0x10000	/* no cache flush necessary on sync */
#define _BUS_DMAMAP_IS_BOUNCING 0x20000	/* is bouncing current xfer */
#define _BUS_DMAMAP_NOALLOC	0x40000	/* don't alloc memory from this range */

struct aarch64_bus_dma_segment {
	/* public: */
	bus_addr_t	ds_addr;	/* DMA address */
	bus_size_t	ds_len;		/* length of transfer */

	/* private: */
	uint32_t	_ds_flags;
};

/*
 * This structure describes a valid DMA range.
 */
struct aarch64_dma_range {
	bus_addr_t dr_sysbase;	/* system base address */
	bus_addr_t dr_busbase;	/* appears here on bus */
	bus_size_t dr_len;	/* length of range */
	uint32_t dr_flags;	/* flags for range */
};

/*
 * bus_dma_tag_t
 *
 * A machine-dependent opaque type describing the implementation of
 * DMA for a given bus.
 */
struct mbuf;
struct uio;
struct aarch64_bus_dma_tag {
	/*
	 * DMA range for this tag.  If the page doesn't fall within
	 * one of these ranges, an error is returned.  The caller
	 * may then decide what to do with the transfer.  If the
	 * range pointer is NULL, it is ignored.
	 */
	struct aarch64_dma_range *_ranges;
	int _nranges;

	/*
	 * Opaque cookie for use by back-end.
	 */
	void *_cookie;

	/*
	 * DMA mapping methods.
	 */
	int (*_dmamap_create)(bus_dma_tag_t, bus_size_t, int,
	    bus_size_t, bus_size_t, int, bus_dmamap_t *);
	void (*_dmamap_destroy)(bus_dma_tag_t, bus_dmamap_t);
	int (*_dmamap_load)(bus_dma_tag_t, bus_dmamap_t, void *,
	    bus_size_t, struct proc *, int);
	int (*_dmamap_load_mbuf)(bus_dma_tag_t, bus_dmamap_t,
	    struct mbuf *, int);
	int (*_dmamap_load_uio)(bus_dma_tag_t, bus_dmamap_t,
	    struct uio *, int);
	int (*_dmamap_load_raw)(bus_dma_tag_t, bus_dmamap_t,
	    bus_dma_segment_t *, int, bus_size_t, int);
	void (*_dmamap_unload)(bus_dma_tag_t, bus_dmamap_t);
	void (*_dmamap_sync)(bus_dma_tag_t, bus_dmamap_t,
	    bus_addr_t, bus_size_t, int);

	/*
	 * DMA memory utility functions.
	 */
	int (*_dmamem_alloc)(bus_dma_tag_t, bus_size_t, bus_size_t,
	    bus_size_t, bus_dma_segment_t *, int, int *, int);
	void (*_dmamem_free)(bus_dma_tag_t,
	    bus_dma_segment_t *, int);
	int (*_dmamem_map)(bus_dma_tag_t, bus_dma_segment_t *,
	    int, size_t, void **, int);
	void (*_dmamem_unmap)(bus_dma_tag_t, void *, size_t);
	paddr_t (*_dmamem_mmap)(bus_dma_tag_t, bus_dma_segment_t *,
	    int, off_t, int, int);

	/*
	 * DMA tag utility functions
	 */
	int (*_dmatag_subregion)(bus_dma_tag_t, bus_addr_t, bus_addr_t,
	     bus_dma_tag_t *, int);
	void (*_dmatag_destroy)(bus_dma_tag_t);

	/*
	 * State for bounce buffers
	 */
	int _tag_needs_free;
	int (*_may_bounce)(bus_dma_tag_t, bus_dmamap_t, int, int *);
};

/*
 *	bus_dmamap_t
 *
 *	Describes a DMA mapping.
 */
struct aarch64_bus_dmamap {
	/*
	 * PRIVATE MEMBERS: not for use by machine-independent code.
	 */
	bus_size_t	_dm_size;	/* largest DMA transfer mappable */
	int		_dm_segcnt;	/* number of segs this map can map */
	bus_size_t	_dm_maxmaxsegsz; /* fixed largest possible segment */
	bus_size_t	_dm_boundary;	/* don't cross this */
	int		_dm_flags;	/* misc. flags */

	void		*_dm_origbuf;	/* pointer to original buffer */
	int		_dm_buftype;	/* type of buffer */
	struct vmspace	*_dm_vmspace;	/* vmspace that owns the mapping */

	void		*_dm_cookie;	/* cookie for bus-specific functions */

	/*
	 * PUBLIC MEMBERS: these are used by machine-independent code.
	 */
	bus_size_t	dm_maxsegsz;	/* largest possible segment */
	bus_size_t	dm_mapsize;	/* size of the mapping */
	int		dm_nsegs;	/* # valid segments in mapping */
	bus_dma_segment_t dm_segs[1];	/* segments; variable length */
};

/* _dm_buftype */
#define _BUS_DMA_BUFTYPE_INVALID	0
#define _BUS_DMA_BUFTYPE_LINEAR		1
#define _BUS_DMA_BUFTYPE_MBUF		2
#define _BUS_DMA_BUFTYPE_UIO		3
#define _BUS_DMA_BUFTYPE_RAW		4

#ifdef _AARCH64_BUS_DMA_PRIVATE
/*
 * Cookie used for bounce buffers. A pointer to one of these it stashed in
 * the DMA map.
 */
struct aarch64_bus_dma_cookie {
	int	id_flags;		/* flags; see below */

	/*
	 * Information about the original buffer used during
	 * DMA map syncs.  Note that origibuflen is only used
	 * for ID_BUFTYPE_LINEAR.
	 */
	union {
		void	*un_origbuf;		/* pointer to orig buffer if
						   bouncing */
		char	*un_linearbuf;
		struct mbuf	*un_mbuf;
		struct uio	*un_uio;
	} id_origbuf_un;
#define id_origbuf		id_origbuf_un.un_origbuf
#define id_origlinearbuf	id_origbuf_un.un_linearbuf
#define id_origmbuf		id_origbuf_un.un_mbuf
#define id_origuio		id_origbuf_un.un_uio
	bus_size_t id_origbuflen;	/* ...and size */

	void	*id_bouncebuf;		/* pointer to the bounce buffer */
	bus_size_t id_bouncebuflen;	/* ...and size */
	int	id_nbouncesegs;		/* number of valid bounce segs */
	bus_dma_segment_t id_bouncesegs[0]; /* array of bounce buffer
					       physical memory segments */
};

/* id_flags */
#define _BUS_DMA_IS_BOUNCING		0x04	/* is bouncing current xfer */
#define _BUS_DMA_HAS_BOUNCE		0x02	/* has bounce buffers */
#endif /* _AARCH64_BUS_DMA_PRIVATE */
#define _BUS_DMA_MIGHT_NEED_BOUNCE	0x01	/* may need bounce buffers */

#endif /* _AARCH64_BUS_DMA_DEFS_H_ */
