/*
 * Driver for DMA Fragments - initially used for BCM2835 DMA implementation
 *
 * Copyright (C) 2014 Martin Sperl
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

#ifndef __DMAFRAGMENT_H
#define __DMAFRAGMENT_H

#include <linux/types.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/dmapool.h>

struct dma_link;
struct dma_fragment;
struct dma_fragment_transform;
struct dma_fragment_cache;

/**
 * struct dma_link - basic building block of a dma-transfer controlblock
 *   which handles allocation, memberships to dma fragments and dumping
 *   of the control block
 * @cb:            the pointer to the DMA control-block
 * @cb_dma:        the dma-bus address of the control block
 * @size:          allocated size of this block
 * @desc:          description of this block
 * @pool:          from which dma_pool the control-block has been taken
 * @fragment:      the fragment to which we belong
 * @dma_link_list: the list of linked dma_links to which this object belongs
 * Note:
 *   allocation is done via pool, as this way we create propperly aligned
 *   controlblocks.
 */
struct dma_link {
	/* the control block itself */
	void                *cb;
	dma_addr_t          cb_dma;
	size_t              size;
	bool                embedded;
	const char          *desc;
	/* the pool from which this has been allocated */
	struct dma_pool     *pool;
	/* the membership to a fragment */
	struct dma_fragment *fragment;
	struct list_head    dma_link_list;
};

/**
 * dma_link_init - initialize a dma-link structure for the case
 *   where the dma_link has been preallocated
 * @dmalink: the dmalink to initialize
 * @pool: the dma pool from which the CB is allocated
 * @size: the allocated size of dmalink pointer - 0 for default size
 * @embedded: if not 0 then it is allocated in a different sturcture
 *            and does not need to get freed (besides the cb)
 * @gfpflags: the flags to use for allocation
 */
static inline struct dma_link *dma_link_init(struct dma_link *dmalink,
					struct dma_pool *pool,
					size_t size,
					bool embedded,
					gfp_t gfpflags)
{
	memset(dmalink, 0, sizeof(*dmalink));

	INIT_LIST_HEAD(&dmalink->dma_link_list);

	dmalink->pool     = pool;
	dmalink->size     = max(size, sizeof(*dmalink));
	dmalink->embedded = embedded;

	dmalink->cb       = dma_pool_alloc(
		dmalink->pool,
		gfpflags,
		&dmalink->cb_dma);
	if (!dmalink->cb)
		return NULL;
	else
		return dmalink;
}

/**
 * dma_link_alloc
 * @pool: the dma pool from which the dma controlblock is to get allocated
 * @size: the size of the dma_link object 0 for default size
 * @gfpflags: the flags to use for allocation
 */
static inline struct dma_link *dma_link_alloc(struct dma_pool *pool,
				size_t size,
				gfp_t gfpflags)
{
	struct dma_link *dmalink;

	size = max(size, sizeof(*dmalink));

	dmalink = kzalloc(size, gfpflags);
	if (!dmalink)
		return NULL;

	/* initialize the structure */
	if (!dma_link_init(dmalink, pool, size, 1, gfpflags)) {
		kfree(dmalink);
		return NULL;
	}
	return dmalink;
}

/**
 * dma_link_free - free the dma_link object
 *    in case it is embedded, only release the DMA controlblock
 * @dmalink: the object to free
 */
static inline
void dma_link_free(struct dma_link *dmalink)
{
	if (!dmalink->cb)
		return;

	list_del_init(&dmalink->dma_link_list);

	dma_pool_free(dmalink->pool,
		dmalink->cb,
		dmalink->cb_dma);
	dmalink->cb = NULL;

	if (!dmalink->embedded)
		kfree(dmalink);
}

/**
 * dma_link_dump - dump the dma_link object
 * @link:        the dma_link to dump
 * @dev:         device used for dev_printk
 * @tindent:     the number of tab indents to add
 * @dma_cb_dump: optional dump function for the dma control block itself
 */
void dma_link_dump(
	struct dma_link *link,
	struct device *dev,
	int tindent,
	void (*dma_cb_dump)(struct dma_link *, struct device *, int)
	);

/**
 * dma_fragment_transform - describes a transformation typically executed
 *   on a dma_fragment
 * @transform_list: list of transforms to get executed in sequence
 * @size:     the allocated size of dmafragment pointer
 *            - 0 for default size
 * @embedded: if not 0 then it is allocated in a different sturcture
 *            and does not need to get freed (besides the cb)
 * @function: the transformation function
 * @fragment: the fragment to which this belongs
 * @data:     additional static data for the transform
 */
struct dma_fragment_transform {
	struct list_head transform_list;
	size_t size;
	bool embedded;
	int    (*function)(struct dma_fragment_transform *, void *, gfp_t);
	struct dma_fragment *fragment;
	void *data;
};

/**
 * dma_fragment_transform_init - initialize a preallocated transform
 * @transform: the transform to initialize
 * @size:     the allocated size of dmafragment pointer
 *            - 0 for default size
 * @embedded: if not 0 then it is allocated in a different sturcture
 *            and does not need to get freed (besides the cb)
 * @function: the transformation function
 * @fragment: the fragment to which this belongs
 * @data:     additional static data for the transform
 */
static inline void dma_fragment_transform_init(
	struct dma_fragment_transform *transform,
	size_t size, bool embedded,
	int (*function)(struct dma_fragment_transform *, void *, gfp_t),
	struct dma_fragment *fragment,
	void *data)
{
	memset(transform, 0, sizeof(*transform));

	INIT_LIST_HEAD(&transform->transform_list);

	transform->function    = function;
	transform->fragment    = fragment;
	transform->data        = data;
	transform->size        = max(size, sizeof(*transform));
	transform->embedded    = embedded;
}

/**
 * dma_fragment_transform_alloc - allocate and initialize a transform
 * @size:     the allocated size of dmafragment pointer
 *            - 0 for default size
 * @function: the transformation function
 * @fragment: the fragment to which this belongs
 * @data:     additional static data for the transform
 * @gfpflags: the flags to use for allocation
 */
static inline struct dma_fragment_transform *dma_fragment_transform_alloc(
	size_t size,
	int (*function)(struct dma_fragment_transform *, void *, gfp_t),
	struct dma_fragment *fragment,
	void *data,
	gfp_t gfpflags)
{
	struct dma_fragment_transform *trans;

	size = max(size, sizeof(*trans));

	trans = kzalloc(size, gfpflags);
	if (trans)
		dma_fragment_transform_init(
			trans, size, 0,
			function, fragment, data);
	return trans;
}

/**
 * dma_fragment_transform_free - free the transform object
 *    in case it is embedded, only remove from list
 * @transform: the object to free
 */
static inline void dma_fragment_transform_free(
	struct dma_fragment_transform *transform)
{
	list_del_init(&transform->transform_list);
	if (!transform->embedded)
		kfree(transform);
}

/**
 * dma_fragment_transform_call - calls the transform function
 * @transform: the transfrom to execute
 * @fragment:  the fragment argument to the function
 * @data:      the data argment to the function
 * @gfpflags:  gfpflags to the function
 */
static inline int dma_fragment_transform_exec(
	struct dma_fragment_transform *transform,
	void *data,
	gfp_t gfpflags
	)
{
	return transform->function(
		transform,
		data,
		gfpflags);
}

/**
 * dma_fragment_transform_call_list - will execute all the transforms
 *  of the list
 * @list:     the listhead of dma_fragment_transforms to execute
 * @data:     the data to add as extra parameter
 * @gfpflags: gfpflags to add as extra arguments
 */
static inline int dma_fragment_transform_call_list(
	struct list_head *list,
	void *data,
	gfp_t gfpflags
	)
{
	struct dma_fragment_transform *transform;
	int err;

	list_for_each_entry(transform,
			list,
			transform_list) {
		err = dma_fragment_transform_exec(
			transform,
			data,
			gfpflags);
		if (err)
			return err;
	}

	return 0;
}

/**
 * dma_fragment_transform_dump - dump the transform object
 * @link:    the dma_link to dump
 * @dev:     device used for dev_printk
 * @tindent: the number of tab indents to add
 */
void dma_fragment_transform_dump(
	struct dma_fragment_transform *trans,
	struct device *dev,
	int tindent);

/**
 * dma_fragment - a collection of connected dma_links
 * @size:           size of this fragment
 * @embedded:         if not 0 then it is allocated in a different
 *                    structure and does not need to get freed
 * @desc:             description of fragment - helpfull during debuging
 * @cache:            the dma_fragment_cache from which this fragment
 *                    was taken
 * @cache_list:       the list inside the dma_fragment_cache
 * @sub_fragment_head:the list of fragments which are part of
 *                    this fragment
 * @sub_fragment_list:the list to which this fragment belongs
 * @dma_link_list:    the list of dma_links
 * @transform_list:   list of transforms that belong to this fragment
 *                    typically used when "merging" fragments
 * @link_head:        the dma_link that is the first in sequence
 *                    used to link dma fragments together
 * @link_tail:        the dma_link that is the last in sequence
 *                    used to link dma fragments together
 * @release_fragment: method called prior to returning the fragment
 *                    to the fragment cache
 */
struct dma_fragment {
	int size;
	int embedded;
	char *desc;
	/* the object in cache */
	struct dma_fragment_cache *cache;
	struct list_head cache_list;

	/* list of sub fragments */
	struct list_head sub_fragment_head;
	/* list of subfagments to which we belong */
	struct list_head sub_fragment_list;

	/* the linked list of dma_links */
	struct list_head dma_link_list;

	/* the linked list of dma_fragment_transforms */
	struct list_head transform_list;

	/* the first and the last object in the fragment
	 * note that this is not necessarily identical
	 * to dma_linked_list - especially if multiple
	 * DMA-Channels are needed for processing */
	struct dma_link *link_head;
	struct dma_link *link_tail;
	/* release fragment function
	 * - called prior to returning it to cache or freeing it
	 * if flag is set then we are freeing the fragment
	 */
	void (*release_fragment)(struct dma_fragment *, int);
};

/**
 * dma_fragment_init - initialize the dma_fragment
 * @fragment: fragment to initialize
 * @size:     the allocated size of dmalink pointer - 0 for default size
 * @embedded: if not 0 then it is allocated in a different sturcture
 *            and does not need to get freed (besides the cb)
 */
static inline void dma_fragment_init(
	struct dma_fragment *fragment,
	size_t size, int embedded)
{
	memset(fragment, 0, sizeof(*fragment));

	fragment->size = max(size, sizeof(*fragment));
	fragment->embedded = embedded;

	INIT_LIST_HEAD(&fragment->cache_list);

	INIT_LIST_HEAD(&fragment->sub_fragment_head);
	INIT_LIST_HEAD(&fragment->sub_fragment_list);

	INIT_LIST_HEAD(&fragment->dma_link_list);

	INIT_LIST_HEAD(&fragment->transform_list);
}

/**
 * dma_fragment_alloc - allocate a new dma_fragment
 * @size:     the size to really allocate
 * @gfpflags: the allocation flags
 */
static inline struct dma_fragment *dma_fragment_alloc(
	size_t size, gfp_t gfpflags)
{
	struct dma_fragment *frag;

	size = max(size, sizeof(*frag));

	frag = kzalloc(size, gfpflags);
	if (!frag)
		return NULL;

	dma_fragment_init(frag, size, 0);

	return frag;
}


/**
 * dma_fragment_release - release a dma_fragment
 *   either from memory or return to cache
 * @fragment: the fragment to free
 */
void dma_fragment_release(struct dma_fragment *fragment);

/**
 * dma_fragment_release_subfragments - release all
 *      dma_fragment_subfragments from this fragment and return them
 *      to cache
 * @fragment: the sub-fragment to release
 */
void dma_fragment_release_subfragments(struct dma_fragment *fragment);

/**
 * dma_fragment_dump - dump the given fragment including all the
 *   sub-fragments
 * @fragment:    the fragment to dump
 * @dev:         device used for dev_printk
 * @tindent:     the number of tab indents to add
 * @dma_cb_dump: optional dump function for the dma control block
 */
void dma_fragment_dump(struct dma_fragment *fragment,
		struct device *dev,
		int tindent,
		void (*dma_cb_dump)(struct dma_link *,
				struct device *, int)
	);

/**
 * dma_fragment_add_dma_link - add DMA controlblock to the fragment
 * @fragment:     the fragment to which to add the dma_link
 * @dmalink:      the link object of the DMA controlblock to add
 * @mark_as_tail: mark it as the head/tail dma_link, which is used for
 *                linking dma_fragments
 */
static inline void dma_fragment_add_dma_link(
	struct dma_fragment *fragment,
	struct dma_link *dmalink,
	int mark_as_tail)
{
	list_add_tail(&dmalink->dma_link_list, &fragment->dma_link_list);

	dmalink->fragment = fragment;

	if (mark_as_tail) {
		if (!fragment->link_head)
			fragment->link_head = dmalink;
		fragment->link_tail = dmalink;
	}
}

/**
 * dma_fragment_add_dma_transform - add DMA transform to the fragment
 * @fragment:  the fragment to which to add the transfrom
 * @transform: the transform to add
 */
static inline void dma_fragment_add_transform(
	struct dma_fragment *fragment,
	struct dma_fragment_transform *transform
	)
{
	list_add_tail(&transform->transform_list,
		&fragment->transform_list);
}

/**
 * dma_fragment_addnew_transform - allocate and add DMA transform
 *   to the fragment
 * @fragment: the fragment to which to add
 * @size:     the allocated size of dmafragment pointer
 *            - 0 for default size
 * @function: the transformation function
 * @data:     additional static data for the transform
 * @gfpflags: the flags to use for allocation
 */
static inline
struct dma_fragment_transform *dma_fragment_addnew_transform(
	struct dma_fragment *fragment,
	size_t size,
	int (*function)(struct dma_fragment_transform *, void *, gfp_t),
	void *data,
	gfp_t gfpflags)
{
	struct dma_fragment_transform *trans =
		dma_fragment_transform_alloc(
			size,
			function,
			fragment,
			data,
			gfpflags);
	if (trans)
		dma_fragment_add_transform(
			fragment, trans);

	return trans;
}
/**
 * dma_fragment_add_subfragment - add a sub-fragment to an existing
 *   fragment
 * @new: the new fragment
 */
static inline int dma_fragment_add_subfragment(
	struct dma_fragment *subfragment,
	struct dma_fragment *merged,
	int (*link_dma_link)(struct dma_link *, struct dma_link *),
	gfp_t gfpflags)
{
	int err = 0;
	/* add to the end of the list */
	list_add_tail(&subfragment->sub_fragment_list,
		&merged->sub_fragment_head);

	/* link to tail on the DMA level - if needed */
	if (link_dma_link) {
		if (merged->link_head) {
			err = link_dma_link(
				merged->link_tail,
				subfragment->link_head);
			if (err)
				return err;
		}
		/* and clean tail */
		err = link_dma_link(
			subfragment->link_tail, NULL);
		if (err)
			return err;
	}

	/* copy list head/list_tail */
	if (!merged->link_head)
		merged->link_head = subfragment->link_head;
	merged->link_tail = subfragment->link_tail;

	/* and finally execute the transforms */
	return dma_fragment_transform_call_list(
		&subfragment->transform_list, merged, gfpflags);
}

/**
 * struct dma_fragment_cache - cache of several dma_fragments
 *   used to avoid setup costs of memory allocation and initialization
 * @device:                 the device to which this cache belongs
 * @dev_attr:               the device attributes of this cache
 *                          (includes the name) - mostly for sysfs
 * @lock:                   lock for this structure
 * @idle:                   list of currently idle fragments
 * @allocateFragment:       the allocation code for fragments
 * @count_active:           number of currently active fragments
 * @count_idle:             number of currently idle fragments
 * @count_allocated:        number of allocated objects
 * @count_allocated_kernel: number of allocated objects with GFP_KERNEL
 * @count_fetched:          number of fetches from this cache
 */
struct dma_fragment_cache {
	/* the device and device_attribute used for sysfs */
	struct device           *device;
	struct device_attribute dev_attr;

	/* the lock for the cache */
	spinlock_t              lock;

	/* the idle queue list */
	struct list_head        idle;

	/* allocation function */
	struct dma_fragment     *(*allocateFragment)(
		struct device *, gfp_t);

	/* the counters exposed via sysfs */
	u32 count_active;
	u32 count_idle;
	u32 count_allocated;
	u32 count_allocated_kernel;
	u32 count_removed;
	u64 count_fetched;
};

/**
 * dma_fragment_cache_initialize - initialize a dma_fragment cache
 * @cache:            the cache to initialize
 * @device:           the device for which we run this cache
 * @name:             the identifier of the dma_fragment_cache
 * @allocateFragment: the fragment allocation/initialization function
 * @initial_size:     the initial size of the cache
 */
int dma_fragment_cache_initialize(
	struct dma_fragment_cache *cache,
	struct device *device,
	const char *name,
	struct dma_fragment *(*allocateFragment)(struct device *, gfp_t),
	int initial_size
	);

/**
 * dma_fragment_cache_release: release the DMA Fragment cache
 * @cache: the cache to release
 * note that this only releases idle dma_fragments.
 *   any still active fragments are NOT released.
 */
void dma_fragment_cache_release(struct dma_fragment_cache *cache);

/**
 * dma_fragment_cache_add_active - add an item to the dma cache active
 * @cache:    the cache to which to add an item
 * @gfpflags: the flags used for allocation
 */
struct dma_fragment *dma_fragment_cache_add_active(
	struct dma_fragment_cache *cache,
	gfp_t gfpflags);
/**
 * dma_fragment_cache_add_idle - add an item to the dma cache as idle
 * @cache:    the cache to which to add an item
 * @gfpflags: the flags used for allocation
 */
struct dma_fragment *dma_fragment_cache_add_idle(
	struct dma_fragment_cache *cache,
	gfp_t gfpflags);

/**
 * dma_fragment_cache_resize - add/remove items from the idle pool
 * @cache:    the cache to resize
 * @resizeby: increase/decrease the number of items in idle by this number
 */
int dma_fragment_cache_resize(
	struct dma_fragment_cache *cache,
	int resizeby
	);

/**
 * dma_fragment_cache_fetch - fetch an object from dma_fragment_cache
 *   creating new ones if needed
 * @cache:    the cache from which to fetch
 * @gfpflags: flags to use in case we need to allocate a new object
 */
static inline struct dma_fragment *dma_fragment_cache_fetch(
	struct dma_fragment_cache *cache, gfp_t gfpflags)
{
	unsigned long flags;
	int is_empty;
	struct dma_fragment *frag = NULL;

	/* fetch from cache if it exists*/
	spin_lock_irqsave(&cache->lock, flags);
	is_empty = list_empty(&cache->idle);
	if (!is_empty) {
		frag = list_first_entry(
			&cache->idle,
			typeof(*frag),
			cache_list);
		list_del_init(&frag->cache_list);

		cache->count_active++;
		cache->count_idle--;

		is_empty = list_empty(&cache->idle);
	}
	cache->count_fetched++;
	spin_unlock_irqrestore(&cache->lock, flags);

	/* allocate fragment outside of lock and add to active queue */
	if (!frag) {
		/* add to cache but remove it immediately */
		frag = dma_fragment_cache_add_active(cache, gfpflags);
	}

	/* if in GFP_KERNEL context, then allocate an additional one */
	if (gfpflags == GFP_KERNEL) {
		/* allocate one more to keep something idle in cache */
		if (is_empty)
			dma_fragment_cache_add_idle(cache, gfpflags);
	}

	return frag;
}

/**
 * dma_fragment_cache_return - return an object to dma_fragment_cache
 * @fragment: the dma_fragment to return or release
 */
static inline void dma_fragment_cache_return(
	struct dma_fragment *fragment)
{
	struct dma_fragment_cache *cache = fragment->cache;
	unsigned long flags;
	/* now return to cache */
	if (cache) {
		spin_lock_irqsave(&cache->lock, flags);
		list_add(&fragment->cache_list, &cache->idle);
		cache->count_idle++;
		cache->count_active--;
		spin_unlock_irqrestore(&cache->lock, flags);
	} else {
		dma_fragment_release(fragment);
	}
}

#endif /* __DMAFRAGMENT_H */
