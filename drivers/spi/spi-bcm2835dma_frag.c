/*
 * Driver for Broadcom BCM2835 SPI Controllers using DMA-FRAGMENTS
 *
 * Copyright (C) 2012 Chris Boot
 * Copyright (C) 2013 Stephen Warren
 * Copyright (C) 2014 Martin Sperl
 *
 * This driver is inspired by:
 * spi-bcm2835.c, Copyright (C) 2012 Chris Boot, 2013 Stephen Warren
 * spi-ath79.c,   Copyright (C) 2009-2011 Gabor Juhos <juhosg@openwrt.org>
 * spi-atmel.c,   Copyright (C) 2006 Atmel Corporation
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
#include "spi-bcm2835dma.h"
#include <linux/spi/spi-dmafragment.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>

/*************************************************************************
 * the function creating dma_fragments - mostly used by dma_fragment_cache
 ************************************************************************/

/*------------------------------------------------------------------------
 * Helpers - some of these could be inlined functions
 *----------------------------------------------------------------------*/
#define GET_VARY_FROM_XFER(xfer)  (0+0*xfer->len) /*xfer->vary*/

/**
 * THIS_CB_MEMBER_DMA_ADDR - dma_addr_t of a DMA_CB.member
 * @member: the member field in the dma CB
 */
#define THIS_CB_MEMBER_DMA_ADDR(member)	\
	BCM2835_DMA_CB_MEMBER_DMA_ADDR(link, member)

/**
 * START_CREATE_FRAGMENT_ALLOCATE - macro that contains repetitive code
 *   and variable definitions used by most allocate functions
 * note: no semicolon for the last define, as we run into compiler
 *   warnings otherwise when it sees ";;" and then some more variable
 *   definitions and then complains about "mixed declaration and code"
 * @struct_name: name of structure to allocate as frag
 */
#define START_CREATE_FRAGMENT_ALLOCATE(struct_name)			\
	struct spi_master *master = (struct spi_master *)device;	\
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);	\
	struct dma_pool *pool = bs->pool;				\
	struct dma_link *link;						\
	struct bcm2835_dma_cb *cb;					\
	struct struct_name *frag =					\
		(struct struct_name *) dma_fragment_alloc(		\
			sizeof(struct struct_name),			\
			gfpflags);					\
	if (!frag)							\
		return NULL;						\
	((struct dma_fragment *)frag)->desc = #struct_name;

#define START_CREATE_FRAGMENT_USE_TRANS()				\
	struct dma_fragment_transform *trans				\

/**
 * END_CREATE_FRAGMENT_ALLOCATE - macro that contains repetitive code
 * used by all alloc functions on exit of function - including error
 * handling.
 */

#define END_CREATE_FRAGMENT_ALLOCATE()				\
	return (struct dma_fragment *)frag;			\
error:								\
	dma_fragment_release((struct dma_fragment *)frag);	\
	return NULL;

/**
 * ADD_DMA_LINK_TO_FRAGMENT_FLAGS - macro that allocates a new dma_link
 *  and adds it to the member field in the structure
 *  it also does some basic linking and sets up some fields with defaults
 * @field: the field to which to assign the dma_link to
 */
#define ADD_DMA_LINK_TO_FRAGMENT_FLAGS(field, astail)			\
	link = dma_link_init(&frag->field,				\
			pool,						\
			0, 1,						\
			gfpflags					\
		);							\
	if (!link)							\
		goto error;						\
	link->desc = #field;						\
	dma_fragment_add_dma_link(					\
		(struct dma_fragment *)frag,				\
		(struct dma_link *)&frag->field,			\
		astail							\
		);							\
	cb = (struct bcm2835_dma_cb *)link->cb;				\
	cb->next = 0;							\
	cb->stride = 0;

/**
 * LINK_TO - macro that links the dma_link pointed to by field in the
 * custom dma_fragment structure to the currently active dma_link
 * @field: the field name
 */
#define LINKTO(field)					\
	bcm2835_link_dma_link(&frag->field, link);

#define ADD_DMA_LINK_TO_FRAGMENT(field)		\
	ADD_DMA_LINK_TO_FRAGMENT_FLAGS(field, 1)
#define ADD_DMA_LINK_TO_FRAGMENT_NOLINK(field)	\
	ADD_DMA_LINK_TO_FRAGMENT_FLAGS(field, 0)


/*------------------------------------------------------------------------
 * helpers and macros to to make the basic "setup" of dma_fragments
 * easier to read
 *----------------------------------------------------------------------*/

#define GENERIC_TRANSFORM_WRAPPER(functionname, suffix)			\
	static int functionname ## suffix(				\
		struct dma_fragment_transform *transform,		\
		void *vp, gfp_t gfpflags)				\
	{								\
		return functionname(					\
			(void *)transform->fragment,			\
			(struct spi_merged_dma_fragment *)vp,		\
			transform->data,				\
			gfpflags);					\
	}
#define LINK_TRANSFORM_WRAPPER(functionname, suffix)		\
	GENERIC_TRANSFORM_WRAPPER(functionname, suffix)
#define VARY_TRANSFORM_WRAPPER(functionname, suffix)		\
	GENERIC_TRANSFORM_WRAPPER(functionname, suffix)

#define SCHEDULE_LINKTIME_TRANSFORM(function, data)		 \
	if (!dma_fragment_addnew_transform(			 \
			&frag->dma_fragment,			 \
			0,					 \
			&function,\
			data,					 \
			gfpflags))				 \
		goto error;

#define SCHEDULE_VARY_TRANSFORM(function, data)			 \
		goto error;

#define LINKVARY_TRANSFORM_WRAPPER(varymask, function, suffix, linktimecode) \
	GENERIC_TRANSFORM_WRAPPER(function, suffix##_wrap)		\
	static int function ## suffix(					\
		struct dma_fragment_transform *transform,		\
		void *vp,						\
		gfp_t gfpflags)						\
	{								\
		struct dma_fragment *frag  = transform->fragment;	\
		struct spi_merged_dma_fragment *merged = vp;		\
		/* get the vary flag from the transfer */		\
		int vary = GET_VARY_FROM_XFER(merged->transfer);	\
		void *data = transform->data;				\
		linktimecode;						\
		/* depending on vary and varymask make the decission */	\
		if ((!varymask) || (vary & varymask)) {		\
			if (!spi_merged_dma_fragment_addnew_predma_transform( \
					merged,				\
					frag,				\
					0,				\
					&function##suffix##_wrap,	\
					data,				\
					gfpflags))			\
				return -ENOMEM;				\
			else						\
				return 0;				\
		} else {						\
			return function(				\
				(void *)frag,				\
				merged,					\
				data,					\
				gfpflags);				\
		}							\
	}

#define SCHEDULE_LINKVARY_TRANSFORM(function, data)		 \
	if (!dma_fragment_addnew_transform(			 \
			&frag->dma_fragment,			 \
			0,					 \
			&function,				 \
			data,					 \
			gfpflags))				 \
		goto error;

/**
 * FIXED - macro that defines the field as one that can get assigned with
 * a static value when creating the fragment.
 * this is guaranteed to never get modified
 * @field: the field in the dma-controlblock
 * @value: the value to assign
 */
#define FIXED(field, value) cb->field = value;

/**
 * bcm2835dma_fragment_transform_spi_data_offset - transform function
 *   to fetch a u32 value from the spi_device specific data and copy it to
 *   the requested destination
 * @transform: the transform that contains all additional parameters
 * @transform_src: the offset in bytes from the start of
 *    spi_device_driver_data
 * @transform_dst: the (u32 *)pointer to which to assigne the value of src
 * @fragment: the fragment to which this transform belongs
 * @vp: the fragment into which this fragment is getting merged
 * Implicit assumption that we run during link time, so vp contains valid
 * message and transfer....
 */
struct  dma_fragment_transform_spi_drvdata {
	struct dma_fragment_transform transform;
	size_t offset;
};

static int bcm2835dma_fragment_transform_spi_ctldata_offset(
	struct dma_fragment_transform *tr,
	void *vp,
	gfp_t gfpflags)
{
	struct dma_fragment_transform_spi_drvdata *transform =
		(typeof(transform)) tr;
	/* the merged fragment from the extra void pointer argument */
	struct spi_merged_dma_fragment *merged_frag = vp;

	/* the spi device from the message pointer */
	struct spi_device *spi = merged_frag->message->spi;

	/* pointer to spi_device_data as a char pointer*/
	char *base = spi_get_ctldata(spi);
	base += transform->offset;

	/* copy the value */
	*((u32 *)transform->transform.data) = *((u32 *)(base));

	return 0;
}

/**
 * SPI - macro that copies some value from the spi_device_data structure
 * to the field in the dma_controlblock during link time
 * @field: the field name in the dma controlblock
 * @spi_data_field: the field in the spi_device_data structure
 *   this get executed during link time
 * Note: this is a bit inefficient, as it requires more calls/entries
 *   than necessary when we need to update multiple fields.
 *   in this case a "common" function works best...
 *   see SPISET and SPIDONE to get used to ducument those cases.
 */
#define SPI(field, spi_data_field)					\
	trans = dma_fragment_addnew_transform(				\
		&frag->dma_fragment,					\
		sizeof(struct dma_fragment_transform_spi_drvdata),	\
		&bcm2835dma_fragment_transform_spi_ctldata_offset,	\
		&dma_link_to_cb(link)->field,				\
		gfpflags);						\
	if (!trans)							\
		goto error;						\
	((struct  dma_fragment_transform_spi_drvdata *)trans)->offset =	\
		offsetof(struct bcm2835dma_spi_device_data,		\
			spi_data_field);

/**
 * SPISET - macro do set the field to the field from spi_data
 * @field: the field name in the dma controlblock
 * @spi_data_field: the field in the spi_device_data structure
 * @linkname: the dma_link for which we set it
 */
#define SPISET(field, spi_data_field, linkname)				\
	dma_link_to_cb(&frag->linkname)->field = spi_data->spi_data_field;

/**
 * SPIDONE - macro that marks that this field is supposed to get done by
 *  transform function - no code generated
 * @field: the field name in the dma controlblock
 * @spi_data_field: the field in the spi_device_data structure
 * @linkname: the dma_link for which we set it
 * note:
 *   macro is empty and mostly there to test that we did not forget
 *   setting anything...
 */
#define SPIDONE(field, spi_data_field, linkname)

/**
 * TXDMA - macro that assignes the DMA BASE_REGISTER of the transmit DMA
 *  + offset to the specific dma_controlblock field.
 * @field: the field name in the dma controlblock
 * @offset: the offset of the register we need to access
 */
#define TXDMA(field, offset) cb->field = bs->dma_tx.bus_addr+offset;

/**
 * IRQDMA - macro that assignes the DMA BASE_REGISTER of the interrupt DMA
 *  + offset to the specific dma_controlblock field.
 * @field: the field name in the dma controlblock
 * @offset: the offset of the register we need to access
 */
#define IRQDMA(field, offset) cb->field = bs->dma_irq.bus_addr+offset;

/**
 * VARY - macro that is just there to document that this value is set via
 * a dma_fragment_transform - for optimized spi_messages this also takes
 * xfer->vary into account as to when to execute the function.
 * this is only here to document what needs to get done
 * @field: the field name in the dma controlblock
 * @function: the function name that is repsonsible for setting this
 */
#define VARY(field, function, ...)

/**
 * LATER - macro that is there just to document that we are setting
 * the field a bit later in the sequence
 * @field: the field name in the dma controlblock
 */
#define LATER(field, ...)

/*------------------------------------------------------------------------
 * general remark on dma_fragment_transforms
 *
 * the transforms are applied to the individual dma_chains when linking
 * of a dma_fragment to the bcm2835dma_spi_merged_dma_fragment happens.
 *
 * the extra data argument to the transform function, is used to give
 * a pointer to the merged fragment to which this fragment gets linked.
 * this allows for scheduling additional transforms in the merged fragment
 *
 * for the merged fragment its transform are called:
 * * when unlinking the fragments (i.e. releasing the individual fragments
 *   back to cache)
 * * there are also special pre/post DMA queues that are executed before
 *   /after a DMA occurs.
 *----------------------------------------------------------------------*/

/*------------------------------------------------------------------------
 * allocator for setting up spi
 *----------------------------------------------------------------------*/

/**
 * dma_fragment_config_spi - the dma_fragment structure needed to configure
 *  spi and select Chip Select
 * @fragment: the main embedded dma_fragment structure
 * @cs_select: the dma_link that selects CS
 * @reset_spi_fifo: the dma_link responsible for resetting the spi fifo buffers
 * @config_clock_length: the dma_link responsible for setting the
 *    SPI clock divider and configure the number of bytes to transfer
 * @config_spi: the dma_link responsible for configuring the spi device
 *    to start DMA processing
 * @set_tx_dma_next: the dma_link responsible for setting the next address
 *    from which the TX-DMA will restart
 * @start_tx_dma: the dma_link responsible for starting the tx-dma
 */
struct dma_fragment_config_spi {
	struct dma_fragment dma_fragment;
	/* additional data - allocated already to reduce overhead */
	struct dma_link     cs_select;
	struct dma_link     reset_spi_fifo;
	struct dma_link     config_clock_length;
	struct dma_link     config_spi;
	struct dma_link     set_tx_dma_next;
	struct dma_link     start_tx_dma;
};

/**
 * bcm2835dma_fragment_transform_link_config_spi - transform that
 *   will initialize bcm2835dma_spi_merged_dma_fragment.txdma_link_to_here
 *   to the correct value.
 * @dma_frag: the dma fragment to which we apply this transform
 * @spi_merge: the spi_merge fragment, to which this gets added
 * @data: extra data - not used
 * @gfpflags: gfp flags used to allocate memory
 */
static inline int bcm2835dma_fragment_transform_link_speed(
	struct dma_fragment *dma_frag,
	struct spi_merged_dma_fragment *spi_merged,
	void *data,
	gfp_t gfpflags)
{
	/* cast to correct type */
	struct bcm2835dma_spi_merged_dma_fragment *merged =
		container_of(spi_merged, typeof(*merged), spi_fragment);
	/* cast to correct type */
	struct dma_fragment_config_spi *frag =
		container_of(dma_frag, typeof(*frag), dma_fragment);
	/* the spi-device and transfer for which we run this */
	struct spi_master *master = spi_merged->message->spi->master;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	struct spi_transfer *xfer = spi_merged->transfer;
	/* get the speed values */
	u32 spi_hz = xfer->speed_hz;
	u32 clk_hz = clk_get_rate(bs->clk);
	u32 cdiv;
	s32 delay;
	/* now calculate the clock divider and other delay cycles */
	if (spi_hz >= clk_hz / 2) {
		cdiv = 2; /* clk_hz/2 is the fastest we can go */
	} else if (spi_hz) {
		cdiv = DIV_ROUND_UP(clk_hz, spi_hz);
		/* as per documentation CDIV must be a power of two
		 * but empirically NOTRO found that it is not needed,
		 * so we do not add the following:
		 * cdiv = roundup_pow_of_two(cdiv);
		 */
		if (cdiv >= 65536)
			cdiv = 0; /* 0 is the slowest we can go */
	} else
		cdiv = 0; /* 0 is the slowest we can go */

	/* and set the clock divider */
	merged->speed_hz = spi_hz;
	merged->speed_cdiv = cdiv;
	dma_link_to_cb(&frag->config_clock_length)->pad[0] = cdiv;

	/* and now calculate the delay for a half clock cycle
	 * needs better forumula than those empirical values
	 * the main problem here is that the effective delay has several
	 * components:
	 * * prefetch of DMA Block - if the previous DMA is a DREQ transfer
	 *   then the prefetch of the control-block is hidden in the previous
	 *   DMA transfer
	 * * the delay of the loop itself (which is probably constant)
	 * this means that for the "DELAY" after a SPI-receive-DMA we do
	 * not expect any overhead of scheduling the dma, but if it is a
	 * POKE DMA without a DREQ then there is some delay for scheduling,
	 * so an offset needs to get subtracted
	 */
	delay = BCM2835_DMA_UDELAY_SLOPE * 1000000 / 2 / spi_hz;
	if (delay < 0)
		delay = 0;
	if (delay >= (1<<30))
		delay = (1<<30)-1;

	/* intercept is typically negative */
	if (delay < -BCM2835_DMA_UDELAY_INTERCEPT)
		merged->delay_half_cycle_cs_deselect_dma_length = 0;
	else
		merged->delay_half_cycle_cs_deselect_dma_length = delay
			+ BCM2835_DMA_UDELAY_INTERCEPT;

	/* for post_rx we need to factor in some other delays */
	merged->delay_half_cycle_post_rx_dma_length = delay;
	if (spi_hz < 1000000)
		merged->delay_half_cycle_post_rx_dma_length +=
			delay/2;

	return 0;
}
LINKVARY_TRANSFORM_WRAPPER(SPI_OPTIMIZE_VARY_SPEED_HZ,
			bcm2835dma_fragment_transform_link_speed,
			_vary,/* none */);

/**
 * bcm2835dma_fragment_transform_link_config_spi - transform that
 *   will initialize bcm2835dma_spi_merged_dma_fragment.txdma_link_to_here
 *   to the correct value.
 * @dma_frag: the dma fragment to which we apply this transform
 * @spi_merge: the spi_merge fragment, to which this gets added
 * @data: extra data - not used
 * @gfpflags: gfp flags used to allocate memory
 */
static inline int bcm2835dma_fragment_transform_link_config_spi(
	struct dma_fragment *dma_frag,
	struct spi_merged_dma_fragment *spi_merged,
	void *data,
	gfp_t gfpflags)
{
	/* cast to correct type */
	struct bcm2835dma_spi_merged_dma_fragment *merged =
		container_of(spi_merged, typeof(*merged), spi_fragment);
	/* cast to correct type */
	struct dma_fragment_config_spi *frag =
		container_of(dma_frag, typeof(*frag), dma_fragment);
	/* the spi-device for which we run this */
	struct bcm2835dma_spi_device_data *spi_data =
		spi_get_ctldata(spi_merged->message->spi);

	/* set up the address where to link a transfer */
	merged->txdma_link_to_here =
		&dma_link_to_cb(&frag->set_tx_dma_next)->pad[0];
	/* and setup "initial" total length and reset to 0 */
	merged->total_length =
		&dma_link_to_cb(&frag->config_clock_length)->pad[1];
	/* actually we would need to vary this part... */
	*merged->total_length = 0;

	/* implementing the SPIDONE below during link time*/
	SPISET(dst,    cs_select_gpio_reg, cs_select);
	SPISET(pad[0], cs_bitfield       , cs_select);
	SPISET(pad[0], spi_reset_fifo    , reset_spi_fifo);
	SPISET(pad[0], spi_config        , config_spi);

	/* and return OK */
	return 0;
}
LINK_TRANSFORM_WRAPPER(bcm2835dma_fragment_transform_link_config_spi,
		_wrap);

/**
 *  bcm2835dma_spi_create_fragment_config_spi - allocate and set up the
 *   dma_fragment to configure the SPI device
 * @device: the spi_master.device for  which we set this up
 * @gpfflags: the gpf_t flags to use when allocating memory
 */
struct dma_fragment *bcm2835dma_spi_create_fragment_config_spi(
	struct device *device, gfp_t gfpflags)
{
	START_CREATE_FRAGMENT_ALLOCATE(dma_fragment_config_spi);

	/* before we do any of this we need to
	 * schedule a cdiv calculation */
	/* schedule the SPI divider calculation */
	SCHEDULE_LINKVARY_TRANSFORM(
		bcm2835dma_fragment_transform_link_speed_vary,
		NULL);

	/* select chipselect - equivalent to:
	 *  writel(spi_dev_data->cs_bitfield,
	 *      spi_dev_data->cs_select_gpio_reg);
	*/
	ADD_DMA_LINK_TO_FRAGMENT(cs_select);
	FIXED(ti,       BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,      THIS_CB_MEMBER_DMA_ADDR(pad[0]));
	SPIDONE(dst,    cs_select_gpio_reg, cs_select);
	FIXED(length,   4);
	SPIDONE(pad[0], cs_bitfield, cs_select);

	/* reset SPI fifos - equivalent to:
	 * writel(spi_dev_data->spi_reset_fifo,BCM2835_SPI_CS);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(reset_spi_fifo);
	LINKTO(cs_select);
	FIXED(ti,       BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,      THIS_CB_MEMBER_DMA_ADDR(pad[0]));
	FIXED(dst,      (BCM2835_SPI_BASE_BUS + BCM2835_SPI_CS));
	FIXED(length,   4);
	SPIDONE(pad[0], spi_reset_fifo, reset_spi_fifo);

	/* configure clock divider and transfer length  - equivalent to:
	 * writel(cdiv, BCM2835_SPI_CLK);
	 * writel(total transfer length, BCM2835_SPI_DLEN);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(config_clock_length);
	LINKTO(reset_spi_fifo);
	FIXED(ti,     (BCM2835_DMA_TI_WAIT_RESP
			| BCM2835_DMA_TI_S_INC
			| BCM2835_DMA_TI_D_INC
			));
	FIXED(src,    THIS_CB_MEMBER_DMA_ADDR(pad[0]));
	FIXED(dst,    (BCM2835_SPI_BASE_BUS + BCM2835_SPI_CLK));
	FIXED(length, 8);
	VARY(pad[0],  bcm2835dma_fragment_transform_speed_hz);
	LATER(pad[1], bcm2835dma_fragment_transform_prepare_txlink
		/* set to 0 during link time */);

	/* configure and start spi - equivalent to:
	 * writel(spi_dev_data->spi_config,BCM2835_SPI_CS);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(config_spi);
	LINKTO(config_clock_length);
	FIXED(ti,       BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,      THIS_CB_MEMBER_DMA_ADDR(pad[0]));
	FIXED(dst,      (BCM2835_SPI_BASE_BUS + BCM2835_SPI_CS));
	FIXED(length,   4);
	SPIDONE(pad[0], spi_config, config_spi);

	/* set up the tx-dma start address - equivalent to:
	 * writel(dma_address_of_tx_transfer,txdma_base+BCM2835_DMA_ADDR);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(set_tx_dma_next);
	LINKTO(config_spi);
	FIXED(ti,     BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,    THIS_CB_MEMBER_DMA_ADDR(pad[0]));
	TXDMA(dst,    BCM2835_DMA_ADDR);
	FIXED(length, 4);
	LATER(pad[0], bcm2835dma_fragment_transform_prepare_txlink
		/* this is set later, when we know the dma_addr
		   of the TX-DMA-transfer */);

	/* start the tx-dma - equivalent to:
	 * writel(BCM2835_DMA_CS_ACTIVE|BCM2835_DMA_CS_DISDEBUG,
	 *        txdma_base+BCM2835_DMA_ADDR);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(start_tx_dma);
	LINKTO(set_tx_dma_next);
	FIXED(ti,     BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,    THIS_CB_MEMBER_DMA_ADDR(pad[0]));
	TXDMA(dst,    BCM2835_DMA_CS);
	FIXED(length, 4);
	FIXED(pad[0], BCM2835_DMA_CS_ACTIVE/*|BCM2835_DMA_CS_DISDEBUG*/);

	/* schedule link time handling of settings including SPIDONE */
	SCHEDULE_LINKTIME_TRANSFORM(
		bcm2835dma_fragment_transform_link_config_spi_wrap,
		NULL);

	END_CREATE_FRAGMENT_ALLOCATE();
}

/*------------------------------------------------------------------------
 * allocator for transfers
 *----------------------------------------------------------------------*/

/**
 * dma_fragment_transfer - the dma_fragment structure to schedule a
 *   single spi_transfer
 * @fragment: the main embedded dma_fragment structure
 * @xfer_rx: the dma_link responsible for receiving data from SPI
 * @xfer_tx: the dma_link responsible for transmitting data over SPI
 */

struct dma_fragment_transfer {
	struct dma_fragment           dma_fragment;
	struct dma_link               xfer_rx;
	struct dma_link               xfer_tx;
	struct dma_fragment_transform trans_dmaunmap;
	struct spi_transfer           *transfer;
};

/**
 * bcm2835dma_fragment_transform_link_length - transform that
 *   will set up the length for the transfers adding values if needed
 * @dma_frag: the dma fragment to which we apply this transform
 * @spi_merge: the spi_merge fragment, to which this gets added
 * @data: extra data
 * @gfpflags: gfp flags used to allocate memory
 */
static inline int bcm2835dma_fragment_transform_length(
	struct dma_fragment *dma_frag,
	struct spi_merged_dma_fragment *spi_merged,
	void *data,
	gfp_t gfpflags)
{
	/* cast to correct type */
	struct bcm2835dma_spi_merged_dma_fragment *merged =
		container_of(spi_merged, typeof(*merged), spi_fragment);
	/* cast to correct type */
	struct dma_fragment_transfer *frag =
		container_of(dma_frag, typeof(*frag), dma_fragment);
	/* the xfer can not get taken from spi_merged, as this
	 * happens potentially during scheduling,
	 * where transfer invalid at this time*/
	struct spi_transfer *xfer = frag->transfer;

	/* set length of transfers */
	dma_link_to_cb(&frag->xfer_tx)->length = xfer->len;
	dma_link_to_cb(&frag->xfer_rx)->length = xfer->len;

	/* set total length to (u32)data if set */
	if (data)
		*merged->total_length = (u32)data;

	/* and add to total length value - or set it if data is non 0 */
	*merged->total_length += xfer->len;

	/* and if we are not a multiple of 4 force cleaning of SPI-fifos */
	if (xfer->len & 0x03)
		spi_merged->needs_spi_setup = 1;

	/* and return */
	return 0;
}
VARY_TRANSFORM_WRAPPER(bcm2835dma_fragment_transform_length, _vary);

/**
 * bcm2835dma_fragment_transform_map_data - transform that
 *   will dma map data if necessary and set up the DMA flags
 * @dma_frag: the dma fragment to which we apply this transform
 * @spi_merge: the spi_merge fragment, to which this gets added
 * @data: extra data
 * @gfpflags: gfp flags used to allocate memory
 */
static inline int bcm2835dma_fragment_transform_mapset_data(
	struct dma_fragment *dma_frag,
	struct spi_merged_dma_fragment *spi_merged,
	void *data,
	gfp_t gfpflags)
{
	/* cast to correct type */
	struct bcm2835dma_spi_merged_dma_fragment *merged =
		container_of(spi_merged, typeof(*merged), spi_fragment);
	/* cast to correct type */
	struct dma_fragment_transfer *frag =
		container_of(dma_frag, typeof(*frag), dma_fragment);

	/* the xfer can not get taken from spi_merged, as this
	 * happens potentially during scheduling,
	 * where transfer invalid at this time*/
	struct spi_transfer *xfer = frag->transfer;
	/* the device which we map */
	struct device *dev = &spi_merged->message->spi->dev;

	dma_addr_t tx_dma = 0;
	dma_addr_t rx_dma = 0;
	/* we may need to map the data */
	if (spi_merged->message->is_dma_mapped)  {
		tx_dma = xfer->tx_dma;
		rx_dma = xfer->rx_dma;
	} else {
		if (xfer->tx_buf) {
			tx_dma = dma_map_single_attrs(
				dev,
				(void *)xfer->tx_buf,
				xfer->len,
				DMA_TO_DEVICE,
				NULL);
		}
		if (xfer->rx_buf) {
			rx_dma = dma_map_single_attrs(
				dev,
				(void *)xfer->rx_buf,
				xfer->len,
				DMA_FROM_DEVICE,
				NULL);
		}
	}

	/* set the rx/tx dma addresses */
	dma_link_to_cb(&frag->xfer_tx)->src = tx_dma;
	dma_link_to_cb(&frag->xfer_rx)->dst = rx_dma;

	/* set dma flags depending on if we got data or NULL */
	dma_link_to_cb(&frag->xfer_tx)->ti =
		BCM2835_DMA_TI_PER_MAP(BCM2835_DMA_DREQ_SPI_TX)
		| BCM2835_DMA_TI_D_DREQ
		| BCM2835_DMA_TI_NO_WIDE_BURSTS
		| ((xfer->tx_buf) ?
			BCM2835_DMA_TI_S_INC : BCM2835_DMA_TI_S_IGNORE);
	dma_link_to_cb(&frag->xfer_rx)->ti =
		BCM2835_DMA_TI_PER_MAP(BCM2835_DMA_DREQ_SPI_RX)
		| BCM2835_DMA_TI_S_DREQ
		| BCM2835_DMA_TI_NO_WIDE_BURSTS
		| ((xfer->rx_buf) ?
			BCM2835_DMA_TI_D_INC : BCM2835_DMA_TI_D_IGNORE);

	/* and return */
	return 0;
}
VARY_TRANSFORM_WRAPPER(bcm2835dma_fragment_transform_mapset_data, _vary);

/**
 * bcm2835dma_fragment_transform_unmap_data - transform that
 *   will release mapped memory regions
 * @dma_frag: the dma fragment to which we apply this transform
 * @spi_merge: the spi_merge fragment, to which this gets added
 * @data: extra data
 * @gfpflags: gfp flags used to allocate memory
 */
static inline int bcm2835dma_fragment_transform_unmap_data(
	struct dma_fragment *dma_frag,
	struct spi_merged_dma_fragment *spi_merged,
	void *data,
	gfp_t gfpflags)
{
#if 0
static inline int bcm2835dma_fragment_transform_dmaunmap(
	struct dma_fragment_transform *transform,
	void *vp,
	gfp_t gfpflags)
{
	/* the fragment's real type */
	struct dma_fragment_transfer *frag =
		(typeof(frag)) transform->fragment;

	/* the transfer links */
	struct dma_link *link_tx = frag->xfer_tx;
	struct dma_link *link_rx = frag->xfer_rx;

	/* and the controlblocks */
	struct bcm2835_dma_cb *cb_tx =
		(struct bcm2835_dma_cb *)link_tx->cb;
	struct bcm2835_dma_cb *cb_rx =
		(struct bcm2835_dma_cb *)link_rx->cb;

	/* get the real values */
	u32           length = cb_tx->length;
	dma_addr_t    tx_dma = cb_tx->src;
	dma_addr_t    rx_dma = cb_rx->dst;

	/* and get the device */
	struct spi_message *msg = transform->data;
	struct device *dev = &msg->spi->master->dev;

	/* now do the conditional unmap if it is not 0 */
	if (tx_dma)
		dma_unmap_single_attrs(
			dev,
			tx_dma,
			length,
			DMA_TO_DEVICE,
			NULL);
	if (rx_dma)
		dma_unmap_single_attrs(
			dev,
			rx_dma,
			length,
			DMA_FROM_DEVICE,
			NULL);
	return 0;
}
#endif
	return 0;
}
VARY_TRANSFORM_WRAPPER(bcm2835dma_fragment_transform_unmap_data, _postdma);

/**
 * bcm2835dma_fragment_transform_link_transfer - handle all the vary
 *   decissions for the linking of transfers
 *
 */
static int bcm2835dma_fragment_transform_transfer(
	struct dma_fragment *dma_frag,
	struct spi_merged_dma_fragment *spi_merged,
	void *data,
	gfp_t gfpflags)
{
	int err;
	/* cast to correct type */
	struct bcm2835dma_spi_merged_dma_fragment *merged =
		container_of(spi_merged, typeof(*merged), spi_fragment);
	/* cast to correct type */
	struct dma_fragment_transfer *frag =
		container_of(dma_frag, typeof(*frag), dma_fragment);
	/* the transfer */
	struct spi_message *message = spi_merged->message;
	struct spi_transfer *xfer   = spi_merged->transfer;
	/* get the vary information */
	int vary = GET_VARY_FROM_XFER(xfer);

	/* link tx-dma to last one - unfortunately we
	 * can't use the generic bcm2835_link_dma_link */
	*merged->txdma_link_to_here = frag->xfer_tx.cb_dma;

	/* connect link to here */
	merged->txdma_link_to_here =
		&dma_link_to_cb(&frag->xfer_tx)->next;

	/* set the transfer in the fragment - needed for callbacks */
	frag->transfer = spi_merged->transfer;

	/* and based on vary we act differently */
	if (vary & SPI_OPTIMIZE_VARY_LENGTH) {
		/* need to set length at pre-dma time */
		if (!spi_merged_dma_fragment_addnew_predma_transform(
				spi_merged,
				dma_frag,
				0,
				&bcm2835dma_fragment_transform_length_vary,
				(void *)*merged->total_length,
				gfpflags))
			return -ENOMEM;
		/* and mark last_transfer as NULL
		 * as we have to schedule a new spi config
		 * to reset dma fifos
		 * note: with some "extra" VARY_LENGTH_X4
		 * to signify that len will be a multiple of 4
		 * we could avoid this
		 */
		spi_merged->needs_spi_setup = 1;
	} else {
		/* otherwise we call it immediately,
		 *  but with NULL as data so that we just increment */
		err = bcm2835dma_fragment_transform_length(
			dma_frag, spi_merged, NULL, gfpflags);
		if (err)
			goto error;
	}

	/* map the addresses calling vary */
	if (vary & (SPI_OPTIMIZE_VARY_RX_BUF|SPI_OPTIMIZE_VARY_TX_BUF)) {
		bcm2835dma_fragment_transform_mapset_data(
			dma_frag,
			spi_merged,
			NULL,
			gfpflags);
	} else {
		/* note that this does only work because we have
		   transfer in the structure itself - see above */
		if (!spi_merged_dma_fragment_addnew_predma_transform(
				spi_merged,
				dma_frag,
				0,
				bcm2835dma_fragment_transform_mapset_data_vary,
				NULL,
				gfpflags))
			return -ENOMEM;
	}

	/* and if we are not dma-mapped, the schedule an unmap */
	if (!message->is_dma_mapped) {
		/* and schedule a unmap with a prelinked object */
		spi_merged_dma_fragment_add_postdma_transform(
			spi_merged, &frag->trans_dmaunmap);
	}

	return 0;
error:
	return err;
}
LINK_TRANSFORM_WRAPPER(bcm2835dma_fragment_transform_transfer, _link);

/**
 * bcm2835dma_spi_create_fragment_transfer - allocate and setup the
 *  dma_fragment to configure the DMA transfer
 * @device: the spi_master.device for  which we set this up
 * @gpfflags: the gpf_t flags to use when allocating memory
 */
static struct dma_fragment *bcm2835dma_spi_create_fragment_transfer(
	struct device *device, gfp_t gfpflags)
{
	START_CREATE_FRAGMENT_ALLOCATE(dma_fragment_transfer);

	/* the tx transfer - equivalent to:
	 * for (i=0 ; i<xfer->length; i++)
	 *   writel(xfer->tx_buf[i],BCM2835_SPI_FIFO);
	 * of 0 - in case the buffer is empty
	 */
	ADD_DMA_LINK_TO_FRAGMENT_NOLINK(xfer_tx);
	VARY(ti,      bcm2835dma_fragment_transform_linktx, xfer.tx_addr);
	VARY(src,     bcm2835dma_fragment_transform_linktx, xfer.tx_addr);
	FIXED(dst,    (BCM2835_SPI_BASE_BUS + BCM2835_SPI_FIFO));
	VARY(length,  bcm2835dma_fragment_transform_copyadd_length,
		xfer.length);
	FIXED(pad[0], 0); /* in case we have no pointer, so use this */

	/* the rx transfer - equivalent to:
	 * for (i=0 ; i<xfer->length; i++)
	 *   xfer->rx_buf[i] = readl(BCM2835_SPI_FIFO);
	 * of 0 - in case the buffer is empty
	 */
	ADD_DMA_LINK_TO_FRAGMENT(xfer_rx);
	VARY(ti,      bcm2835dma_fragment_transform_linktx, xfer.rx_addr);
	FIXED(src,    (BCM2835_SPI_BASE_BUS + BCM2835_SPI_FIFO));
	VARY(dst,     bcm2835dma_fragment_transform_linktx, xfer.rx_addr);
	VARY(length,  bcm2835dma_fragment_transform_copyadd_length,
		xfer.length);

	/* create the prepared dmaunmap transform,
	 * as there are very few dma_mapped spi_messages */
	dma_fragment_transform_init(
		&frag->trans_dmaunmap,
		sizeof(frag->trans_dmaunmap),
		1, /* is embedded */
		bcm2835dma_fragment_transform_unmap_data_postdma,
		&frag->dma_fragment,
		0);

	/* we also need to link the tx_channel to the previous, but as
	   this only happens during dma-fragment linking, we need to
	   schedule it...
	 */
	SCHEDULE_LINKTIME_TRANSFORM(
			bcm2835dma_fragment_transform_transfer_link,
			NULL);

	END_CREATE_FRAGMENT_ALLOCATE();
}

/*------------------------------------------------------------------------
 * allocator for deselecting cs
 *----------------------------------------------------------------------*/
struct dma_fragment_cs_deselect {
	struct dma_fragment dma_fragment;
	struct dma_link     delay_pre;
	struct dma_link     cs_deselect;
	struct dma_link     delay_post;
};

static inline int bcm2835dma_fragment_transform_set_cs_delay(
	struct dma_fragment *dma_frag,
	struct spi_merged_dma_fragment *spi_merged,
	void *data,
	gfp_t gfpflags)
{
	/* cast to correct type */
	struct bcm2835dma_spi_merged_dma_fragment *merged =
		container_of(spi_merged, typeof(*merged), spi_fragment);
	/* cast to correct type */
	struct dma_fragment_cs_deselect *frag =
		container_of(dma_frag, typeof(*frag), dma_fragment);

	/* the xfer can not get taken from spi_merged, as this
	 * happens potentially during scheduling,
	 * where transfer invalid at this time*/
	struct spi_transfer *xfer = spi_merged->transfer;

	/* set delays accordingly */
	dma_link_to_cb(&frag->delay_pre)->length =
		merged->delay_half_cycle_post_rx_dma_length
		+ xfer->delay_usecs * BCM2835_DMA_UDELAY_SLOPE
		;

	dma_link_to_cb(&frag->delay_post)->length =
		merged->delay_half_cycle_cs_deselect_dma_length;

	return 0;
}
LINKVARY_TRANSFORM_WRAPPER(
	(SPI_OPTIMIZE_VARY_DELAY_USECS | SPI_OPTIMIZE_VARY_SPEED_HZ),
	bcm2835dma_fragment_transform_set_cs_delay, _vary,
	/* mark the transfer as requireing a new spi-setup
	   - actually this needs to happen during link time... */
	merged->needs_spi_setup = 1;
	);

static struct dma_fragment *bcm2835dma_spi_create_fragment_cs_deselect(
	struct device *device, gfp_t gfpflags)
{
	START_CREATE_FRAGMENT_USE_TRANS();
	START_CREATE_FRAGMENT_ALLOCATE(dma_fragment_cs_deselect);

	/* delay by half a clock cycle or by the delay given in xfer
	 * equivalent to: udelay(max(xfer->delay_usecs,
	 *   500000/xfer->clock_frequency)))
	 */
	ADD_DMA_LINK_TO_FRAGMENT(delay_pre);
	FIXED(ti,     (BCM2835_DMA_TI_WAIT_RESP
			| BCM2835_DMA_TI_WAITS(0x1f)
			| BCM2835_DMA_TI_NO_WIDE_BURSTS
			| BCM2835_DMA_TI_S_IGNORE
			| BCM2835_DMA_TI_D_IGNORE));
	FIXED(src,    link->cb_dma);
	FIXED(dst,    0);
	VARY(length,  bcm2835dma_fragment_transform_set_delaylength,
		xfer->delay_usec);

	/* deselect chipselect - equivalent to:
	 * writel(spi_dev_data->cs_bitfield,
	 *        spi_dev_data->cs_deselect_gpio_reg);
	*/
	ADD_DMA_LINK_TO_FRAGMENT(cs_deselect);
	LINKTO(delay_pre);
	FIXED(ti,       BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,      THIS_CB_MEMBER_DMA_ADDR(pad[0]));
	SPI(dst,        cs_deselect_gpio_reg);
	FIXED(length,   4);
	SPI(pad[0],     cs_bitfield);

	/* delay by half a clock cycle
	 * equivalent to: udelay(500000/xfer->clock_frequency)
	 */
	ADD_DMA_LINK_TO_FRAGMENT(delay_post);
	LINKTO(cs_deselect);
	FIXED(ti,     (BCM2835_DMA_TI_WAIT_RESP
			| BCM2835_DMA_TI_WAITS(0x1f)
			| BCM2835_DMA_TI_NO_WIDE_BURSTS
			| BCM2835_DMA_TI_S_IGNORE
			| BCM2835_DMA_TI_D_IGNORE));
	FIXED(src,    link->cb_dma);
	FIXED(dst,    0);
	VARY(length,  bcm2835dma_fragment_transform_set_delaylength,
		half_clock_cycle);

	SCHEDULE_LINKTIME_TRANSFORM(
		bcm2835dma_fragment_transform_set_cs_delay_vary,
		NULL);

	END_CREATE_FRAGMENT_ALLOCATE();
}

/*------------------------------------------------------------------------
 * allocator for adding some delay
 *----------------------------------------------------------------------*/
struct dma_fragment_delay {
	struct dma_fragment dma_fragment;
	struct dma_link     delay;
};

static inline int bcm2835dma_fragment_transform_set_delay(
	struct dma_fragment *dma_frag,
	struct spi_merged_dma_fragment *spi_merged,
	void *data,
	gfp_t gfpflags)
{
	/* cast to correct type */
	struct bcm2835dma_spi_merged_dma_fragment *merged =
		container_of(spi_merged, typeof(*merged), spi_fragment);
	/* cast to correct type */
	struct dma_fragment_delay *frag =
		container_of(dma_frag, typeof(*frag), dma_fragment);

	/* the xfer can not get taken from spi_merged, as this
	 * happens potentially during scheduling,
	 * where transfer invalid at this time*/
	struct spi_transfer *xfer = spi_merged->transfer;

	/* calc the delay */
	s32 delay = xfer->delay_usecs * BCM2835_DMA_UDELAY_SLOPE
		+ BCM2835_DMA_UDELAY_INTERCEPT;
	if (delay < 0)
		delay = 0;
	if (delay >= (1<<30))
		delay = (1<<30)-1;

	dma_link_to_cb(&frag->delay)->length = delay;

	return 0;
}
LINKVARY_TRANSFORM_WRAPPER(
	SPI_OPTIMIZE_VARY_DELAY_USECS,
	bcm2835dma_fragment_transform_set_delay, _vary,
	/* mark the transfer as requireing a new spi-setup
	   - actually this needs to happen during link time... */
	merged->needs_spi_setup = 1;
	);

static struct dma_fragment *bcm2835dma_spi_create_fragment_delay(
	struct device *device, gfp_t gfpflags)
{
	START_CREATE_FRAGMENT_ALLOCATE(dma_fragment_delay);

	/* delay by the delay given in xfer
	 * equivalent to: udelay(xfer->delay_usecs);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(delay);
	FIXED(ti,     (BCM2835_DMA_TI_WAIT_RESP
			| BCM2835_DMA_TI_WAITS(0x1f)
			| BCM2835_DMA_TI_NO_WIDE_BURSTS
			| BCM2835_DMA_TI_S_IGNORE
			| BCM2835_DMA_TI_D_IGNORE));
	FIXED(src,    0);
	FIXED(dst,    0);
	VARY(length,  bcm2835dma_fragment_set_delaylength,
		xfer->delay_usec);

	/* schedule the vary transform for link-time
	   onyl set the xfer.delay_usec to the delay length
	 */
	SCHEDULE_LINKTIME_TRANSFORM(
		bcm2835dma_fragment_transform_set_delay_vary,
		NULL);

	END_CREATE_FRAGMENT_ALLOCATE();
}
/*------------------------------------------------------------------------
 * allocator for marking message as finished
 *----------------------------------------------------------------------*/
struct dma_fragment_message_finished {
	struct dma_fragment dma_fragment;
	struct dma_link     message_finished;
};

/**
 * bcm2835dma_fragment_transform_link_length - transform that
 *   will set up the length for the transfers adding values if needed
 * @dma_frag: the dma fragment to which we apply this transform
 * @spi_merge: the spi_merge fragment, to which this gets added
 * @data: extra data
 * @gfpflags: gfp flags used to allocate memory
 */
static inline int bcm2835dma_msg_transforms_prepare_for_schedule(
	struct dma_fragment *dma_frag,
	struct spi_merged_dma_fragment *spi_merged,
	void *data,
	gfp_t gfpflags)
{
	u32 *d = data;
	d[0] = 0;
	d[1] = 0;
	return 0;
}
LINK_TRANSFORM_WRAPPER(bcm2835dma_msg_transforms_prepare_for_schedule,
		_predma);

static inline int bcm2835dma_msg_transforms(
	struct dma_fragment *dma_frag,
	struct spi_merged_dma_fragment *spi_merged,
	void *data,
	gfp_t gfpflags)
{
	/* cast to correct type */
	struct bcm2835dma_spi_merged_dma_fragment *merged =
		container_of(spi_merged, typeof(*merged), spi_fragment);
	/* cast to correct type */
	struct dma_fragment_message_finished *frag =
		container_of(dma_frag, typeof(*frag), dma_fragment);
	struct bcm2835_dma_cb *cb = dma_link_to_cb(&frag->message_finished);

	/* set the pad0/pad1 of message_finished to 0 */
	cb->pad[0] = 0;
	cb->pad[1] = 0;

	/* and set the pointer so that the interrupt-handler may use it */
	spi_merged->complete_data = &cb->pad[0];

	/* schedule pre_dma */
	if (!spi_merged_dma_fragment_addnew_predma_transform(
			spi_merged,
			dma_frag,
			0,
			bcm2835dma_msg_transforms_prepare_for_schedule_predma,
			spi_merged->complete_data,
			gfpflags))
		return -ENOMEM;

	return 0;
}
LINK_TRANSFORM_WRAPPER(bcm2835dma_msg_transforms, _link);

static struct dma_fragment *bcm2835dma_spi_create_fragment_message_finished(
	struct device *device, gfp_t gfpflags)
{
	START_CREATE_FRAGMENT_ALLOCATE(dma_fragment_message_finished);

	/* copy the timestamp from the counter to a fixed address */
	ADD_DMA_LINK_TO_FRAGMENT(message_finished);
	FIXED(ti,       (BCM2835_DMA_TI_WAIT_RESP
				| BCM2835_DMA_TI_INT_EN
				| BCM2835_DMA_TI_S_INC
				| BCM2835_DMA_TI_D_INC));
	FIXED(src,      BCM2835_REG_COUNTER_64BIT_BUS);
	FIXED(dst,      THIS_CB_MEMBER_DMA_ADDR(pad[0]));
	FIXED(length,   8);

	/* schedule link time scheduling of complete callback */
	SCHEDULE_LINKTIME_TRANSFORM(
		bcm2835dma_msg_transforms_link,
		NULL);

	END_CREATE_FRAGMENT_ALLOCATE();
}

/*------------------------------------------------------------------------
 * allocator for triggering an interrupt
 *----------------------------------------------------------------------*/
struct dma_fragment_trigger_irq {
	struct dma_fragment dma_fragment;
	struct dma_link     set_irq_dma_next;
	struct dma_link     start_irq_dma;
	struct dma_link     trigger_irq;
};

static struct dma_fragment *bcm2835dma_spi_create_fragment_trigger_irq(
	struct device *device, gfp_t gfpflags)
{
	u32 *link_irq;
	START_CREATE_FRAGMENT_ALLOCATE(dma_fragment_trigger_irq);
	/* set up the tx-dma start address - equivalent to:
	 * writel(dma_address_of_tx_transfer,txdma_base+BCM2835_DMA_ADDR);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(set_irq_dma_next);
	FIXED(ti,       BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,      THIS_CB_MEMBER_DMA_ADDR(pad[0]));
	IRQDMA(dst,     BCM2835_DMA_ADDR);
	FIXED(length,   4);
	LATER(pad[0],  /* this is set later, when we know the dma_addr
			   of the TX-DMA-transfer */);
	link_irq = &cb->pad[0];

	/* copy the timestamp from the counter to a fixed address */
	ADD_DMA_LINK_TO_FRAGMENT_FLAGS(trigger_irq, 0);
	*link_irq = link->cb_dma;
	FIXED(ti,       (BCM2835_DMA_TI_WAIT_RESP
				| BCM2835_DMA_TI_INT_EN
				| BCM2835_DMA_TI_S_INC
				| BCM2835_DMA_TI_D_INC));
	FIXED(src,      0);
	FIXED(dst,      0);
	FIXED(length,   0);

	/* start the tx-dma - equivalent to:
	 * writel(BCM2835_DMA_CS_ACTIVE,txdma_base+BCM2835_DMA_ADDR);
	 */
	ADD_DMA_LINK_TO_FRAGMENT(start_irq_dma);
	LINKTO(set_irq_dma_next);
	FIXED(ti,       BCM2835_DMA_TI_WAIT_RESP);
	FIXED(src,      THIS_CB_MEMBER_DMA_ADDR(pad[0]));
	IRQDMA(dst,     BCM2835_DMA_CS);
	FIXED(length,   4);
	FIXED(pad[0],   BCM2835_DMA_CS_ACTIVE);

	END_CREATE_FRAGMENT_ALLOCATE();
}

/*************************************************************************
 * the release and initialization of dma_fragment caches
 * and function pointers
 ************************************************************************/
void bcm2835dma_release_dmafragment_components(
	struct spi_master *master)
{
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);

	if (!bs->pool)
		return;

	dma_fragment_cache_release(
		&bs->fragment_merged);
	dma_fragment_cache_release(
			&bs->fragment_setup_spi);
	dma_fragment_cache_release(
			&bs->fragment_transfer);
	dma_fragment_cache_release(
			&bs->fragment_cs_deselect);
	dma_fragment_cache_release(
			&bs->fragment_delay);
	dma_fragment_cache_release(
			&bs->fragment_trigger_irq);

	dma_pool_destroy(bs->pool);
	bs->pool = NULL;
}

static struct dma_fragment *bcm2835dma_merged_dma_fragments_alloc(
	struct device *device, gfp_t gfpflags)
{
	return &spi_merged_dma_fragment_alloc(
		&bcm2835_link_dma_link,
		sizeof(struct bcm2835dma_spi_merged_dma_fragment),
		gfpflags)->dma_fragment;
}

/* register all the stuff needed to control dmafragments
   note that the below requires that master has already been registered
   otherwise you get an oops...
 */

#define PREPARE 10 /* prepare the caches with a typical 10 messages */
int bcm2835dma_register_dmafragment_components(
	struct spi_master *master)
{
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	int err;

	/* allocate pool - need to use pdev here */
	bs->pool = dma_pool_create(
		"DMA-CB-pool",
		&master->dev,
		sizeof(struct bcm2835_dma_cb),
		64,
		0);
	if (!bs->pool) {
		dev_err(&master->dev,
			"could not allocate DMA-memory pool\n");
		return -ENOMEM;
	}

	/* initialize DMA Fragment pools */
	err = dma_fragment_cache_initialize(
		&bs->fragment_merged,
		&master->dev,
		"fragment_merged",
		&bcm2835dma_merged_dma_fragments_alloc,
		PREPARE*1
		);
	if (err)
		goto error;

	err = dma_fragment_cache_initialize(
		&bs->fragment_setup_spi,
		&master->dev,
		"config_spi",
		&bcm2835dma_spi_create_fragment_config_spi,
		PREPARE*2
		);
	if (err)
		goto error;

	err = dma_fragment_cache_initialize(
		&bs->fragment_transfer,
		&master->dev,
		"transfer",
		&bcm2835dma_spi_create_fragment_transfer,
		PREPARE*3
		);
	if (err)
		goto error;

	err = dma_fragment_cache_initialize(
		&bs->fragment_cs_deselect,
		&master->dev,
		"fragment_cs_deselect",
		&bcm2835dma_spi_create_fragment_cs_deselect,
		PREPARE*1
		);
	if (err)
		goto error;

	err = dma_fragment_cache_initialize(
		&bs->fragment_delay,
		&master->dev,
		"fragment_delay",
		&bcm2835dma_spi_create_fragment_delay,
		PREPARE/2
		);
	if (err)
		goto error;

	err = dma_fragment_cache_initialize(
		&bs->fragment_trigger_irq,
		&master->dev,
		"fragment_trigger_irq",
		&bcm2835dma_spi_create_fragment_trigger_irq,
		PREPARE
		);
	if (err)
		goto error;

	err = dma_fragment_cache_initialize(
		&bs->fragment_message_finished,
		&master->dev,
		"fragment_message_end",
		&bcm2835dma_spi_create_fragment_message_finished,
		PREPARE
		);
	if (err)
		goto error;

	return 0;
error:
	bcm2835dma_release_dmafragment_components(master);
	return err;
}
