/*
 * DA-Fragment framework for SPI Controllers
 *
 * Copyright (C) 2014 Martin Sperl
 *
 * This driver is inspired by:
 * spi-ath79.c, Copyright (C) 2009-2011 Gabor Juhos <juhosg@openwrt.org>
 * spi-atmel.c, Copyright (C) 2006 Atmel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 */

#ifndef __SPI_DMAFRAGMENT_H
#define __SPI_DMAFRAGMENT_H
#include <linux/dma-fragment.h>
#include <linux/spi/spi.h>

#ifndef SPI_OPTIMIZE_VARY_TX_BUF
#define SPI_OPTIMIZE_VARY_TX_BUF               (1<<0)
#define SPI_OPTIMIZE_VARY_RX_BUF               (1<<1)
#define SPI_OPTIMIZE_VARY_SPEED_HZ             (1<<2)
#define SPI_OPTIMIZE_VARY_DELAY_USECS          (1<<3)
#define SPI_OPTIMIZE_VARY_LENGTH               (1<<4)
#endif

/**
 * spi_merged_dma_fragment - structure of several
 * @merged_fragments: the list of dma_fragments that got merged into this
 * @transform_pre_dma_list: the list of transforms to execute prior to
 *    executing the DMA
 * @transform_post_dma_list: the list of transforms to execute after the
 *    the DMA has finished
 * @message: the spi message for which this fragment has been "composed"
 * @transfer: the current transfer during link phase - NULL otherwise
 * @last_transfer: the last transfer processed - mostly used to compare
 *    state changes...
 * @need_spi_setup: marks that we need to schedule an spi_setup prior
 *    to a new transfer
 * @link_dma_link: dma control-block linking function pointer.
 * @complete_data: TODO
 */
struct spi_merged_dma_fragment {
	struct dma_fragment dma_fragment;

	struct list_head transform_pre_dma_list;
	struct list_head transform_post_dma_list;

	/* message for which this is prepared */
	struct spi_message *message;
	/* transient data used during linking of fragments
	 * otherwise undefined
	 */
	struct spi_transfer *transfer;
	struct spi_transfer *last_transfer;

	bool needs_spi_setup;

	/* and the link function*/
	int (*link_dma_link)(struct dma_link *, struct dma_link *);

	void *complete_data;
};

void spi_merged_dma_fragment_release(
	struct dma_fragment *frag, int releasetocache);

static inline void spi_merged_dma_fragment_init(
	struct spi_merged_dma_fragment *frag,
	int (*link_dma_link)(struct dma_link *, struct dma_link *),
	size_t size
	)
{
	dma_fragment_init(&frag->dma_fragment, size, 0);

	INIT_LIST_HEAD(&frag->transform_pre_dma_list);
	INIT_LIST_HEAD(&frag->transform_post_dma_list);

	frag->link_dma_link = link_dma_link;
	frag->dma_fragment.release_fragment =
		&spi_merged_dma_fragment_release;
}

static inline
struct spi_merged_dma_fragment *spi_merged_dma_fragment_alloc(
	int (*link_dma_link)(struct dma_link *, struct dma_link *),
	size_t size, gfp_t gfpflags)
{
	struct spi_merged_dma_fragment *frag;
	size = max(size, sizeof(*frag));

	frag = kzalloc(size, gfpflags);
	if (frag)
		spi_merged_dma_fragment_init(
			frag,
			link_dma_link,
			size);

	return frag;
}

/**
 * spi_merged_dma_fragment_dump - dump the spi_merged_dma_fragment
 * into a spi_merged_dma_fragment
 * @fragment: the fragment cache from which to fetch the fragment
 * @dev: the devie to use during the dump
 * @tindent: the number of tab indents to add
 * @dma_link_dump: the function which to use to dump the dmablock
 */
void spi_merged_dma_fragment_dump(
	struct spi_merged_dma_fragment *fragment,
	struct device *dev,
	int tindent,
	void (*dma_cb_dump)(struct dma_link *,
			struct device *, int)
	);

static inline void spi_merged_dma_fragment_add_dma_fragment_transform(
	struct spi_merged_dma_fragment *frag,
	struct dma_fragment_transform *transform,
	int post)
{
	if (post)
		list_add_tail(&transform->transform_list,
			&frag->transform_post_dma_list);
	else
		list_add_tail(&transform->transform_list,
			&frag->transform_pre_dma_list);
}

/**
 * spi_merged_dma_fragment_merge_dma_fragment_from_cache - merge a
 * dma_fragment from a pool
 * into a spi_merged_dma_fragment
 * @fragment: the fragment cache from which to fetch the fragment
 * @merged: the merged fragment
 */
int spi_merged_dma_fragment_merge_fragment_cache(
	struct dma_fragment_cache *fragmentcache,
	struct spi_merged_dma_fragment *merged,
	gfp_t gfpflags);

/**
 * spi_merged_dma_fragment_add_predma_transform - add a pre-DMA
 *   transform to the fragment
 * @fragment: the fragment to which to add
 * @transform: the link object of the DMA controlblock to add
 */
static inline void spi_merged_dma_fragment_add_predma_transform(
	struct spi_merged_dma_fragment *fragment,
	struct dma_fragment_transform *transform
	)
{
	list_add_tail(&transform->transform_list,
		&fragment->transform_pre_dma_list);
}

/**
 * spi_merged_dma_fragment_addnew_predma_transform - add a new pre-dma
 *   transform
 * @addto: the spi_merged_dma_fragment to which we should add this
 * @frag: the dma_fragment for which we are doing this
 * @size: the size to allocate
 * @function: the function to call
 * @data: some extra data to pass to the function
 * @gpfflags: the flags used during allocation of memory
 */
static inline struct dma_fragment_transform *
spi_merged_dma_fragment_addnew_predma_transform(
	struct spi_merged_dma_fragment *addto,
	struct dma_fragment *frag,
	ssize_t size,
	int (*function)(struct dma_fragment_transform *, void *, gfp_t),
	void *data,
	gfp_t gfpflags)
{
	struct dma_fragment_transform *trans =
		dma_fragment_transform_alloc(
			size,
			function,
			frag,
			data,
			gfpflags);
	if (trans)
		spi_merged_dma_fragment_add_predma_transform(
			addto, trans);

	return trans;
}

/**
 * spi_merged_dma_fragment_execute_pre_dma_transforms - execute all the
 *   pre_dma transforms scheduled  for execution
 * @merged: the spi_merged_dma_fragment
 * @data: extra data to pass to the function
 * @gpfflags: the flags used during allocation of memory
 */
static inline int spi_merged_dma_fragment_execute_pre_dma_transforms(
	struct spi_merged_dma_fragment *merged, void *data, gfp_t gfpflags)
{
	return dma_fragment_transform_call_list(
		&merged->transform_pre_dma_list,
		data,
		gfpflags);
}

/**
 * spi_merged_dma_fragment_add_postdma_transform - add a post-DMA
 *   transform to the fragment
 * @fragment: the fragment to which to add
 * @transform: the link object of the DMA controlblock to add
 */
static inline void spi_merged_dma_fragment_add_postdma_transform(
	struct spi_merged_dma_fragment *fragment,
	struct dma_fragment_transform *transform
	)
{
	list_add_tail(&transform->transform_list,
		&fragment->transform_post_dma_list);
}

/**
 * spi_merged_dma_fragment_addnew_postdma_transform - add a new post-dma
 *   transform
 * @addto: the spi_merged_dma_fragment to which we should add this
 * @frag: the dma_fragment for which we are doing this
 * @size: the size to allocate
 * @function: the function to call
 * @data: some extra data to pass to the function
 * @gpfflags: the flags used during allocation of memory
 */
static inline struct dma_fragment_transform *
spi_merged_dma_fragment_addnew_postdma_transform(
	struct spi_merged_dma_fragment *addto,
	struct dma_fragment *frag,
	ssize_t size,
	int (*function)(struct dma_fragment_transform *, void *, gfp_t),
	void *data,
	gfp_t gfpflags)
{
	struct dma_fragment_transform *trans =
		dma_fragment_transform_alloc(
			size,
			function,
			frag,
			data,
			gfpflags);
	if (trans)
		spi_merged_dma_fragment_add_postdma_transform(
			addto, trans);

	return trans;
}

/**
 * spi_merged_dma_fragment_execute_post_dma_transforms - execute all the
 *   post_dma transforms scheduled  for execution
 * @merged: the spi_merged_dma_fragment
 * @data: extra data to pass to the function
 * @gpfflags: the flags used during allocation of memory
 */
static inline int spi_merged_dma_fragment_execute_post_dma_transforms(
	struct spi_merged_dma_fragment *merged, void *data, gfp_t gfpflags)
{
	return dma_fragment_transform_call_list(
		&merged->transform_post_dma_list,
		data,
		gfpflags);
}
#endif /* __SPI_DMAFRAGMENT_H */
