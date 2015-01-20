/*
 * Driver for DMA Fragments
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dma-fragment.h>
#include <linux/device.h>
#include <linux/slab.h>


/**
 * _tab_indent - helper function to return a tabbed indent string
 * @indentdepth: the depth/tabs to return  in string
 * returns string
 */
static const char *_tab_indent_string = "\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t";

static inline const char *_tab_indent(int indent)
{
	return &_tab_indent_string[16-min(16, indent)];
}

static inline void _dump_extra_data(char *data, int size,
				struct device *dev, int tindent)
{
	char buffer[50];
	int offset = 0;
	int bytes_per_line = 16;
	while (size > 0) {
		hex_dump_to_buffer(
			data, min(bytes_per_line, size),
			bytes_per_line, 1,
			buffer, sizeof(buffer),
			0
			);
		dev_info(dev, "%sdata %pf - %02x: %s\n",
			_tab_indent(tindent),
			data, offset, buffer
			);
		data   += bytes_per_line;
		size   -= bytes_per_line;
		offset += bytes_per_line;
	}
}

/**
 * dma_link_cb_dump_generic - generic dma control block dumper
 *   only dumps the binary data
 * @link: the dma_link
 * @dev: the device to dump
 * @tindent: the number of tabs indenting
 */
static void dma_link_cb_dump_generic(
		struct dma_link *link,
		struct device *dev,
		int tindent)
{
	const char *indent = _tab_indent(tindent);
	dev_info(dev, "%scb_addr:\t%pK\n",
		indent,
		link->cb);
	dev_info(dev, "%scb_dma:\t\t%08lx\n",
		indent,
		(long unsigned)link->cb_dma);
	if (sizeof(*link) < link->size)
		_dump_extra_data(
			((char *)link) + sizeof(*link),
			link->size - sizeof(*link),
			dev, tindent);
}

void dma_link_dump(
	struct dma_link *link,
	struct device *dev,
	int tindent,
	void (*dma_cb_dump)(struct dma_link *, struct device *, int)
	)
{
	const char *indent = _tab_indent(tindent);

	if (!dma_cb_dump)
		dma_cb_dump = &dma_link_cb_dump_generic;

	dev_info(dev, "%sdma_link:\t%pK\n", indent,
		link);
	dev_info(dev, "%sdma_pool:\t%pK\n", indent,
		link->pool);
	dev_info(dev, "%sdma_fragment:\t%pK - %s\n", indent,
		link->fragment, link->fragment->desc);
	if (link->desc) {
		if ((link->fragment) && (link->fragment->desc))
			dev_info(dev,
				"%sdescription:\t%s.%s\n", indent,
				link->fragment->desc, link->desc);
		else
			dev_info(dev,
				"%sdescription:\t%s\n", indent,
				link->desc);
	}
	dma_cb_dump(link, dev, tindent+1);
}
EXPORT_SYMBOL_GPL(dma_link_dump);

void dma_fragment_transform_dump(
	struct dma_fragment_transform *trans,
	struct device *dev,
	int tindent)
{
	const char *indent = _tab_indent(tindent);

	dev_info(dev, "%saddr:\t%p\n",  indent, trans);
	dev_info(dev, "%sfunc:\t%pf\n", indent, trans->function);
	dev_info(dev, "%sfrag:\t%pk\n", indent, trans->fragment);
	dev_info(dev, "%sdata:\t%pk\n", indent, trans->data);

	if (sizeof(*trans) < trans->size)
		_dump_extra_data(
			((char *)trans) + sizeof(*trans),
			trans->size - sizeof(*trans),
			dev, tindent);
}
EXPORT_SYMBOL_GPL(dma_fragment_transform_dump);


void dma_fragment_dump_generic(
	struct dma_fragment *fragment,
	struct device *dev,
	int tindent) {
	if (sizeof(*fragment) < fragment->size)
		_dump_extra_data(
			((char *)fragment) + sizeof(*fragment),
			fragment->size - sizeof(*fragment),
			dev, tindent);
}

void dma_fragment_dump(
	struct dma_fragment *fragment,
	struct device *dev,
	int tindent,
	void (*dma_cb_dump)(struct dma_link *, struct device *, int)
	) {
	struct dma_link *link;
	struct dma_fragment *sfrag;
	struct dma_fragment_transform *transform;
	int i;
	const char *indent = _tab_indent(tindent);

	dev_info(dev, "%sDMA-Fragment:\t%pK\n", indent, fragment);
	tindent++;
	indent = _tab_indent(tindent+1);
	dev_info(dev, "%saddr:\t%pK\n", indent, fragment);
	dev_info(dev, "%scache:\t%pK\n", indent, fragment->cache);
	if (fragment->desc)
		dev_info(dev, "%sdescr:\t%s\n",	indent, fragment->desc);
	dev_info(dev, "%slink_h:\t%pK\n", indent, fragment->link_head);
	dev_info(dev, "%slink_t:\t%pK\n", indent, fragment->link_tail);
	dev_info(dev, "%srelease:\t%pK\n", indent,
		fragment->release_fragment);
	/* dump extra data */
	if (sizeof(*fragment) < fragment->size)
		_dump_extra_data(
			((char *)fragment) + sizeof(*fragment),
			fragment->size - sizeof(*fragment),
			dev, tindent);

	/* dump the individual dma_links */
	if (!list_empty(&fragment->dma_link_list)) {
		dev_info(dev, "%sDMA-Links:\n", indent);
		i = 0;
		list_for_each_entry(link,
				&fragment->dma_link_list,
				dma_link_list) {
			dev_info(dev, "%sDMA-Link %i:\n",
				_tab_indent(tindent+1), i++);
			dma_link_dump(link, dev, tindent+2, dma_cb_dump);
		}
	}

	/* dump the sub fragments */
	if (!list_empty(&fragment->sub_fragment_head)) {
		dev_info(dev, "%sSub-DMA-Fragments:\n", indent);
		list_for_each_entry(sfrag,
				&fragment->sub_fragment_head,
				sub_fragment_list) {
			dma_fragment_dump(sfrag, dev, tindent+1,
					dma_cb_dump);
		}
	}

	/* dump the individual dma_fragment_transforms */
	if (!list_empty(&fragment->transform_list)) {
		dev_info(dev, "%sDMA-Transforms:\n", indent);
		i = 0;
		list_for_each_entry(transform,
				&fragment->transform_list,
				transform_list) {
			dev_info(dev, "%sDMA-Transform %i:\n",
				_tab_indent(tindent+1), i++);
			dma_fragment_transform_dump(
				transform, dev, tindent+2);
		}
	}
}
EXPORT_SYMBOL_GPL(dma_fragment_dump);

MODULE_DESCRIPTION("generic dma-fragment debug infrastructure");
MODULE_AUTHOR("Martin Sperl <kernel@martin.sperl.org>");
MODULE_LICENSE("GPL");
