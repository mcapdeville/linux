/*
 * Driver for Broadcom BCM2835 SPI Controllers using DMA-FRAGMENTS
 *
 * Copyright (C) 2012 Chris Boot
 * Copyright (C) 2013 Stephen Warren
 * Copyright (C) 2013 Martin Sperl
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

/* known limitations:
 *  * cs maps directly to GPIO (except for 0 and 1, which also map to 7+8)
 *    and the mode is not reverted when not used
 *  * if there is a transfer of say 13 bytes, then a total of 16 bytes
 *    will get (over)written, so any tightly packed data would get
 *    overwritten.
 *    Not sure how we should approach such a situation - if this is not a
 *    valid situation with drivers right now, then maybe we should make
 *    it a policy
 */
#include "spi-bcm2835dma.h"

#include <linux/version.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>

#include <linux/dma-mapping.h>
#include <linux/dmapool.h>

#include <linux/gpio.h>

#define SPI_TIMEOUT_MS	3000

#define DRV_NAME	"spi-bcm2835dma"

#ifndef list_last_entry
#define list_last_entry(ptr, type, member) \
         list_entry((ptr)->prev, type, member)
#endif


/* module parameter to dump the dma transforms */
int debug_dma = 0;
module_param(debug_dma, int, 0);
MODULE_PARM_DESC(debug_dma,
		"Run the driver with dma debugging enabled");
#define DEBUG_DMA_ASYNC   (1<<0)
#define DEBUG_DMA_ASYNC_DUMP_FRAGMENT (1<<1)
#define DEBUG_DMA_OPTIMIZE (1<<2)
#define DEBUG_DMA_OPTIMIZE_DUMP_FRAGMENT (1<<3)
#define DEBUG_DMA_IRQ      (1<<4)
bool use_optimize = 1;
module_param(use_optimize, bool, 0);
MODULE_PARM_DESC(use_optimize,
		"Run the driver with optimize support enabled");

/* some functions to measure delays on a logic analyzer
 * note: needs to get run first from non-atomic context!!! */
int debugpin1 = 0;
module_param(debugpin1, int, 0);
MODULE_PARM_DESC(debugpin1, "the pin that we should toggle");
u32 *gpio = 0;
static void set_low1(void)
{
	if (!gpio)
		gpio = ioremap(0x20200000, SZ_16K);
	gpio[0x28/4] = debugpin1;
}

static void set_high1(void)
{
	if (!gpio)
		gpio = ioremap(0x20200000, SZ_16K);
	gpio[0x1C/4] = debugpin1;
}

int debugpin2 = 0;
module_param(debugpin2, int, 0);
MODULE_PARM_DESC(debugpin2, "the pin that we should toggle");
static void set_low2(void)
{
	if (!gpio)
		gpio = ioremap(0x20200000, SZ_16K);
	gpio[0x28/4] = debugpin2;
}
static void set_high2(void)
{
	if (!gpio)
		gpio = ioremap(0x20200000, SZ_16K);
	gpio[0x1C/4] = debugpin2;
}

int debugpin3 = 0;
module_param(debugpin3, int, 0);
MODULE_PARM_DESC(debugpin3, "the pin that we should toggle");
static void set_low3(void)
{
	if (!gpio)
		gpio = ioremap(0x20200000, SZ_16K);
	gpio[0x28/4] = debugpin3;
}
static void set_high3(void)
{
	if (!gpio)
		gpio = ioremap(0x20200000, SZ_16K);
	gpio[0x1C/4] = debugpin3;
}

int debugpin4 = 0;
module_param(debugpin4, int, 0);
MODULE_PARM_DESC(debugpin4, "the pin that we should toggle");
static void set_low4(void)
{
	if (!gpio)
		gpio = ioremap(0x20200000, SZ_16K);
	gpio[0x28/4] = debugpin4;
}
static void set_high4(void)
{
	if (!gpio)
		gpio = ioremap(0x20200000, SZ_16K);
	gpio[0x1C/4] = debugpin4;
}

#ifdef SPI_HAVE_OPTIMIZE
static void bcm2835dma_spi_message_unoptimize(struct spi_message *msg);
#endif

/* schedule a DMA fragment on a specific DMA channel
 * this could probably get executed via the dma-engine...
 */
static inline void _bcm2835dma_schedule_dma_fragment(
	struct bcm2835dma_spi *bs,
	struct spi_merged_dma_fragment *frag)
{
	/* reset the RX DMA with a memory barrier */
	writel(BCM2835_DMA_CS_RESET, bs->dma_rx.base+BCM2835_DMA_CS);
	dsb();

	/* schedule transfer address */
	writel(frag->dma_fragment.link_head->cb_dma,
		bs->dma_rx.base+BCM2835_DMA_ADDR);

	/* and start it */
	writel(BCM2835_DMA_CS_ACTIVE, bs->dma_rx.base+BCM2835_DMA_CS);
	dsb();
}

static inline void _bcm2835dma_link_dma_fragment(
	struct spi_merged_dma_fragment *frag,
	struct spi_merged_dma_fragment *last_frag)
{
	/* we got an address, so link it */
	bcm2835_link_dma_link(
		last_frag->dma_fragment.link_tail,
		frag->dma_fragment.link_head);
	dsb(); /* this is absolutely necessary */
}

static inline int _check_complete(u32 *complete) {
	/* sync the data to from memory */
	dsb();
	/* check complete is empty */
	if (readl(&complete[0]))
		return 1;
	if (readl(&complete[1]))
		return 1;
	return 0;
}

void bcm2835dma_release_cb_chain_complete(struct spi_master *master)
{
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	struct spi_message *msg;
	struct spi_merged_dma_fragment *frag;
	u32 *complete;
	unsigned long flags;

	spin_lock_irqsave(&master->queue_lock, flags);

	/* check that we are not already running
	 * - otherwise we would have an inversion of the complete sequence
	 */
/*
	if (bs->cb_chain_complete_running)
		goto exit_running_already;
*/
	/* mark as running */
	bs->cb_chain_complete_running=1;

	set_low4();
	/* check the message queue */
	while (
		(msg = list_first_entry_or_null(
			&master->queue,
			typeof(*msg),
			queue))
		) {
		/* get the fragment for the current message */
		frag = msg->state;

		/* if we have a complete data pointer and it is empty,
		 * then check if it has completed
		 */
		if (frag->complete_data) {
			complete = (u32 *)frag->complete_data;
			/* check for complete twice before returning */
			if (!_check_complete(complete)) {
				if (!_check_complete(complete)) {
					goto exit;
				} else {
					bs->count_completed_on_retry++;
				}
			}
		} else {
			/* how to decide if we can release this message? */
		}

		/* otherwise we can release the message */
		list_del_init(&frag->message->queue);

		/* reset status */
		msg->status = 0;
		/* todo: handle bytes_transferred */
		msg->actual_length = 1;

		/* and execute the post-dma-fragments */
		spi_merged_dma_fragment_execute_post_dma_transforms(
			frag, frag, GFP_ATOMIC);

		/* and release fragment - if not optimized */
#ifdef SPI_HAVE_OPTIMIZE
		if ((!msg->is_optimized) && (use_optimize))
				bcm2835dma_spi_message_unoptimize(msg);
#endif
		/* call the complete call */
		if (msg->complete) {
			bs->last_complete = msg->complete;
			set_high4();
			msg->complete(msg->context);
			set_low4();

		}
	}
exit:
	bs->cb_chain_complete_running=0;
	set_high4();
exit_running_already:
	spin_unlock_irqrestore(&master->queue_lock, flags);
	return;
}

static int bcm2835dma_schedule_dma_fragment(
	struct spi_message *msg)
{
	unsigned long flags;
	struct spi_master *master = msg->spi->master;
	struct spi_merged_dma_fragment *frag = msg->state;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	struct spi_message *last_msg;

//	set_low1();

	/* now start locking things down */
	spin_lock_irqsave(&master->queue_lock, flags);

#ifdef SPI_HAVE_OPTIMIZE
	if ((frag->message->is_optimized) && (! list_empty(&frag->message->queue))) {
		printk(KERN_ERR "message still scheduled: %pf\n",
			frag->message->complete);
	}
#endif
	/* just in case: clean the fragment tail link to 0 */
	bcm2835_link_dma_link(
		frag->dma_fragment.link_tail,
		NULL);

	/* handle the queue */
	if (!list_empty(&master->queue)) {
		/* get the last entry */
		last_msg = list_last_entry(
			&master->queue, struct spi_message, queue);
		/* now add the fragment to the dma queue */
		_bcm2835dma_link_dma_fragment(
			frag, last_msg->state);
	}

	/* link the message to get scheduled to the queue */
	list_add_tail(&msg->queue, &master->queue);

	/* now check if we need to queue it */
	if (readl(bs->dma_rx.base+BCM2835_DMA_CS)
		& BCM2835_DMA_CS_ACTIVE) {
		dsb();
		/* dma is running, so see if it still has an address set */
		if (readl(bs->dma_rx.base+BCM2835_DMA_ADDR)) {
			dsb();
			/* and just in case check again for running */
			if (readl(bs->dma_rx.base+BCM2835_DMA_CS)
				& BCM2835_DMA_CS_ACTIVE) {
				/* and increase counters */
				bs->count_scheduled_msg_dma_running++;
				bs->last_dma_schedule_type = "linked";
				/* and exit */
				goto exit;
			}
		}
	}
	set_low3();
	/* and increase counters */
	bs->count_scheduled_msg_dma_restarted++;
	bs->last_dma_schedule_type = "restarted";
	/* to avoid possible race-conditions between interrupt handlers
	 * for DMA and other sources (=GPIO) we need to release the
	 * messages first
	 */
	bcm2835dma_release_cb_chain_complete(master);

	/* schedule it */
	bs->last_complete = (void*)1;
	_bcm2835dma_schedule_dma_fragment(bs, frag);
	set_high3();

exit:
	/* unlock */
	spin_unlock_irqrestore(&master->queue_lock, flags);
//	set_high1();
	return 0;
}

irqreturn_t bcm2835dma_spi_interrupt_dma_irq(int irq, void *dev_id)
{
	struct spi_master *master = dev_id;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	u32 t;
	set_low2();
	/* write interrupt message to debug */
	if (unlikely(debug_dma&DEBUG_DMA_IRQ))
		dev_err(&master->dev, "IRQ-Interrupt %i triggered\n", irq);

	/* we need to clean the IRQ flag as well for this dma
	 *
	 * Problem here is that we can not run this on the (main) RX DMA,
	 * as a chanined transfer will clear the interrupt automatically
	 * so that the interrupt will not really get handled
	 *
	 * The other problem is that we can not reliably use TX-dma either
	 * as there is a race condition between the interrupt handler and
	 * a subsequent chained transfer, that may also start TX-dma while
	 * the interrupt handler is not that far.
	 *
	 * TX-DMA issue in more details here:
	 * when the TX-DMA is triggering an interrupt, there may be already
	 * the next DMA transaction getting prepared via the RX DMA.
	 * and at some point this RX-DMA will again start configuring
	 * the TX dma for the "next" Transfer, so it will first set DMA_ADDR
	 * and then DMA_CS to trigger a start.
	 * and if the interrupt handler writes at or after this time to
	 * DMA_CS, then it will effectively "pause" the already running DMA
	 * and somehow we would need to detect this and take the "correct"
	 * corrective measures...
	 *
	 * So to avoid these situations we unfortunately have to use a
	 * separate DMA channel just for the interrupt callback, which is
	 * a waste of resources, but no means to detect this race and
	 * react accordingly has been found so far
	 */

	/* check if we got an address set for tx dma */
	t = readl(bs->dma_irq.base+BCM2835_DMA_ADDR);
	if (!t)  {
		/* now read the CS register */
		t = readl(bs->dma_irq.base+BCM2835_DMA_CS);
		/* and if the interrupt bit is set,
		 * then clear it by setting it
		 * and in case another irq-dma is just triggered,
		 * we make start it again - hopefully this never "stalls"
		 */
		if (t&BCM2835_DMA_CS_INT) {
			writel(t|BCM2835_DMA_CS_INT|BCM2835_DMA_CS_ACTIVE,
				bs->dma_irq.base+BCM2835_DMA_CS);
		}
	}

	/* release the control block chains until we reach the CB
	 * this will also call complete
	 */
	bcm2835dma_release_cb_chain_complete(master);

	bs->count_dma_interrupts++;

	set_high2();
	/* and return with the IRQ marked as handled */
	return IRQ_HANDLED;
}

/* device statistics */
static ssize_t bcm2835dma_sysfs_show_stats(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	unsigned long flags;
	struct spi_master *master =
		container_of(dev, typeof(*master), dev);
	struct bcm2835dma_spi *bs =
		container_of(attr, typeof(*bs), stats_attr);
	struct spi_message *msg;
	struct spi_merged_dma_fragment *frag;
	u32 *complete;
	int msg_count = 0;
	ssize_t len = 0;

	/* count  number of messages in queue */
	list_for_each_entry(msg, &master->queue, queue) {
		msg_count++;
	}
	/* now print them */
	len = scnprintf(buf, PAGE_SIZE,
			"bcm2835dma_stats_info - 0.1\n"
			"total spi_messages:\t%llu\n"
			"optimized spi_messages:\t%llu\n"
			"started dma:\t\t%llu\n"
			"linked to running dma:\t%llu\n"
			"last dma_schedule type:\t%s\n"
			"dma interrupts:\t\t%llu\n"
			"completed on retry:\t%llu\n"
			"last complete:\t\t%pf\n"
			"queued messages:\t%u\n",
			bs->count_spi_messages,
			bs->count_spi_optimized_messages,
			bs->count_scheduled_msg_dma_restarted,
			bs->count_scheduled_msg_dma_running,
			bs->last_dma_schedule_type,
			bs->count_dma_interrupts,
			bs->count_completed_on_retry,
			bs->last_complete,
			msg_count);
	/* and iterate over the pending messages */
	msg_count = 0;
	spin_lock_irqsave(&master->queue_lock, flags);
	list_for_each_entry(msg, &master->queue, queue) {
		msg_count++;
		frag = msg->state;
		complete = frag->complete_data;
		len += scnprintf(buf+len, PAGE_SIZE-len,
				"msg%i-ptr:\t\t%pk\n"
				"msg%i-fragment:\t\t%pk\n"
				"msg%i-complete:\t%pf\n",
				msg_count, msg,
				msg_count, frag,
				msg_count, msg->complete);
		if (complete)
			len += scnprintf(buf+len, PAGE_SIZE-len,
					"msg%i-frag-complete:\t%08x%08x\n",
					msg_count,
					complete[0],
					complete[1]);
	}
	spin_unlock_irqrestore(&master->queue_lock, flags);
	/* and return */
	return len;
}

static ssize_t bcm2835dma_sysfs_triggerdump(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t size)
{
	struct spi_master *master =
		container_of(dev, typeof(*master), dev);
	struct bcm2835dma_spi *bs =
		container_of(attr, typeof(*bs), stats_attr);
	struct spi_message *msg;
	int count = 0;

	unsigned long flags;

	/* dump spi */
	dev_info(dev, "SPI Registers\n");
	dev_info(dev, "\tCS:\t%08x\n",
		readl(bs->spi_regs + BCM2835_SPI_CS));
	/* do NOT read FIFO - even for Debug !!!*/
	dev_info(dev, "\tCLK:\t%08x\n",
		readl(bs->spi_regs + BCM2835_SPI_CLK));
	dev_info(dev, "\tDLEN:\t%08x\n",
		readl(bs->spi_regs + BCM2835_SPI_DLEN));
	dev_info(dev, "\tLOTH:\t%08x\n",
		readl(bs->spi_regs + BCM2835_SPI_LTOH));
	dev_info(dev, "\tDC:\t%08x\n",
		readl(bs->spi_regs + BCM2835_SPI_DC));

	/* dump DMA */
	dev_info(dev, "RX-DMA registers\n");
	bcm2835_dma_reg_dump(bs->dma_rx.base, dev, 1);
	dev_info(dev, "TX-DMA registers\n");
	bcm2835_dma_reg_dump(bs->dma_tx.base, dev, 1);
	dev_info(dev, "IRQ-DMA registers\n");
	bcm2835_dma_reg_dump(bs->dma_irq.base, dev, 1);

	/* lock structures */
	spin_lock_irqsave(&master->queue_lock, flags);
	/* dump each DMA message in queue */
	dev_info(dev, "Queued messages\n");
	list_for_each_entry(msg, &master->queue, queue) {
		dev_info(dev,
			"\tQueued SPI message %i:\n"
			"\tmsg-address: %pf",
			count++, msg);
		spi_merged_dma_fragment_dump(msg->state,
					dev,
					1,
					&bcm2835_dma_link_dump
			);
	}

	/* check if we got an entry */
	spin_unlock_irqrestore(&master->queue_lock, flags);

	/* and return size */
	return size;
}

/**
 * bcm2835dma_spi_message_to_dma_fragment - converts a spi_message to a
 *  dma_fragment
 * @msg:  the spi message to convert
 * @flags: some flags
 * @gfpflags: flags for allocation
 * notes:
 *   with minimal effort this probably could get added to the spi framework
 */
struct spi_merged_dma_fragment *bcm2835dma_spi_message_to_dma_fragment(
	struct spi_message *msg, int flags, gfp_t gfpflags)
{
	struct spi_device *spi    = msg->spi;
	struct spi_master *master = spi->master;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);

	struct spi_merged_dma_fragment *merged;
	struct spi_transfer *xfer;
	int err = 0;
	int is_last;

	/* some optimizations - it might help if we knew the length... */
	/* check if we got a frame that is of a single transfer */
	if (list_is_singular(&msg->transfers)) {
		/* check if we got something in the structure we could use */
	}

	/* fetch a merged fragment */
	merged = (typeof(merged))
		dma_fragment_cache_fetch(
			&bs->fragment_merged,
			gfpflags);
	if (!merged)
		return NULL;

	/* initialize some fields */
	merged->message       = msg;
	merged->transfer      = NULL;
	merged->last_transfer = NULL;
	merged->dma_fragment.link_head = NULL;
	merged->dma_fragment.link_tail = NULL;
	merged->complete_data = NULL;
	merged->needs_spi_setup = 1;
	/* now start iterating the transfers */
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		/* check if we are the last in the list */
		is_last = list_is_last(&xfer->transfer_list,
					&msg->transfers);
		/* assign the current transfer */
		merged->transfer = xfer;
		/* do we need to reconfigure spi
		   compared to the last transfer */
		if (!merged->needs_spi_setup) {
			if (merged->last_transfer->speed_hz
				!= xfer->speed_hz)
				merged->needs_spi_setup = 1;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,12,0)
			else if (merged->last_transfer->tx_nbits
				!= xfer->tx_nbits)
				merged->needs_spi_setup = 1;
			else if (merged->last_transfer->rx_nbits
				!= xfer->rx_nbits)
				merged->needs_spi_setup = 1;
#endif
			else if (merged->last_transfer->bits_per_word
				!= xfer->bits_per_word)
				merged->needs_spi_setup = 1;
		}
		/* if we have no last_transfer,
		   then we need to setup spi */
		if (merged->needs_spi_setup) {
			err = spi_merged_dma_fragment_merge_fragment_cache(
				&bs->fragment_setup_spi,
				merged,
				gfpflags);
			if (err)
				goto error;
			merged->needs_spi_setup = 0;
		}
		/* add transfer if the transfer length is not 0
		   or if we vary length */
		if ((xfer->len)
			/* || (xfer->vary & SPI_OPTIMIZE_VARY_LENGTH) */
			) {
			/* schedule transfer */
			err = spi_merged_dma_fragment_merge_fragment_cache(
				&bs->fragment_transfer,
				merged,
				gfpflags);
			if (err)
				goto error;
			/* set last transfer */
			merged->last_transfer = xfer;
		}
		/* add cs_change with optional extra delay
		   if requested or last in sequence */
		if ((xfer->cs_change) || (is_last)) {
			err = spi_merged_dma_fragment_merge_fragment_cache(
				&bs->fragment_cs_deselect,
				merged,
				gfpflags);
		} else if ((xfer->delay_usecs)
			/* || (xfer->vary & SPI_OPTIMIZE_VARY_DELAY) */
			) {
			/* or add a delay if requested */
			err = spi_merged_dma_fragment_merge_fragment_cache(
				&bs->fragment_delay,
				merged,
				gfpflags);
		}

		if (err)
			goto error;
	}

	/* and add an interrupt if we got a callback to handle
	 * if there is no callback, then we do not need to release it
	 * immediately - even for prepared messages
	 */
	if (msg->complete) {
		err = spi_merged_dma_fragment_merge_fragment_cache(
			&bs->fragment_trigger_irq,
			merged,
			gfpflags);
		if (err)
			goto error;
	}

	/* and the "end of transfer flag"
	 * - must be the last to avoid races... */
	err = spi_merged_dma_fragment_merge_fragment_cache(
		&bs->fragment_message_finished,
		merged,
		gfpflags);
	if (err)
		goto error;

	/* reset transfers, as these are invalid by the time
	 * we run the transforms */
	merged->transfer      = NULL;
	merged->last_transfer = NULL;

	/* and return it */
	return merged;

error:
	dev_err(&spi->dev,
		"bcm2835dma_spi_message_to_dma_fragment: err=%i\n",
		err);
	spi_merged_dma_fragment_dump(
		merged,
		&msg->spi->dev,
		0,
		&bcm2835_dma_link_dump
		);
	return NULL;
}

static int bcm2835dma_spi_transfer(struct spi_device *spi,
				struct spi_message *message)
{
	int err = 0;
	struct spi_merged_dma_fragment *merged;
	struct spi_master *master = spi->master;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	unsigned long flags;
	set_low1();

	/* fetch DMA fragment */
#ifdef SPI_HAVE_OPTIMIZE
	if ((message->is_optimized)) {
		merged = message->state;
	} else
#endif
	{
		merged = bcm2835dma_spi_message_to_dma_fragment(
			message,
			0,
			GFP_ATOMIC);
		message->state = merged;
	}
	if (!merged)
		return -ENOMEM;
	/* assign some values */
	message->actual_length = 0;

	/* and execute the pre-transforms */
	err = spi_merged_dma_fragment_execute_pre_dma_transforms(
		merged, merged, GFP_ATOMIC);
	if (err)
		goto error;

	if (unlikely(debug_dma & DEBUG_DMA_ASYNC)) {
		dev_info(&message->spi->dev,
			"Scheduling Message %pf fragment %pf\n",
			message, merged);
		if (debug_dma & DEBUG_DMA_ASYNC_DUMP_FRAGMENT) {
			spi_merged_dma_fragment_dump(merged,
						&message->spi->dev,
						0,
						&bcm2835_dma_link_dump
				);
		}
	}

	/* statistics on message submission */
	spin_lock_irqsave(&master->queue_lock, flags);
	bs->count_spi_messages++;
#ifdef SPI_HAVE_OPTIMIZE
	if ((message->is_optimized))
		bs->count_spi_optimized_messages++;
#endif
	spin_unlock_irqrestore(&master->queue_lock, flags);

	/* and schedule it */
	if (merged)
		bcm2835dma_schedule_dma_fragment(message);

	/* and return */
	set_high1();
	return 0;
error:
	set_high1();
	dev_err(&spi->dev, "spi_transfer_failed: %i", err);
	set_low1();
	dma_fragment_release(&merged->dma_fragment);
	set_high1();
	return -EPERM;
}

#ifdef SPI_HAVE_OPTIMIZE
static int bcm2835dma_spi_message_optimize(struct spi_message *message)
{
	message->state = bcm2835dma_spi_message_to_dma_fragment(
		message,
		0,
		GFP_ATOMIC);
	if (!message->state)
		return -ENOMEM;
	if (unlikely(debug_dma & DEBUG_DMA_OPTIMIZE)) {
		dev_info(&message->spi->dev,
			"Optimizing %pf %pf\n",
			message,
			message->state);
		if (debug_dma & DEBUG_DMA_OPTIMIZE_DUMP_FRAGMENT) {
			spi_merged_dma_fragment_dump(message->state,
						&message->spi->dev,
						0,
						&bcm2835_dma_link_dump
				);
		}
	}

	return 0;
}

static void bcm2835dma_spi_message_unoptimize(struct spi_message *msg)
{
	dma_fragment_release(msg->state);
	msg->state = NULL;
	if (unlikely(debug_dma&DEBUG_DMA_OPTIMIZE)) {
		dev_info(&msg->spi->dev,
			"Unoptimizing %pf\n", msg);
	}
}
#endif

static void bcm2835dma_release_dmachannel(struct spi_master *master,
			struct bcm2835_dmachannel *d)
{
	/* if no base, then we are not configured, so return */
	if (!d->base)
		return;
	/* reset the DMA */
	writel(BCM2835_DMA_CS_RESET, d->base+BCM2835_DMA_CS);
	writel(0, d->base+BCM2835_DMA_ADDR);

	/* release interrupt handler and dma */
	if (d->handler)
		free_irq(d->irq, master);
	bcm_dma_chan_free(d->chan);

	/* release description */
	kfree(d->desc);

	/* cleanup structure */
	d->base = NULL;
	d->bus_addr = 0;
	d->chan = 0;
	d->irq = 0;
	d->handler = NULL;
	d->desc = NULL;
}

static int bcm2835dma_allocate_dmachannel(struct spi_master *master,
					struct bcm2835_dmachannel *d,
					irq_handler_t handler,
					const char *desc
	)
{
	int ret;
	size_t len;
	/* fill in defaults */
	d->base = NULL;
	d->bus_addr = 0;
	d->chan = 0;
	d->irq = 0;
	d->handler = NULL;
	d->desc = NULL;
	/* register DMA channel */
#ifdef CONFIG_MACH_BCM2708
	ret = bcm_dma_chan_alloc(BCM_DMA_FEATURE_FAST,
				(void **)&d->base, &d->irq);
#else
	/* TODO - use dmaengine allocate */
	ret = -1;
#endif
	if (ret < 0) {
		d->base = NULL;
		d->chan = 0;
		d->irq = 0;
		dev_err(&master->dev,
			"couldn't allocate a DMA channel\n");
		return ret;
	}
	d->chan = ret;
	if (handler)
		dev_info(&master->dev,
			"%s-DMA channel %d at address %pK with irq %d"
			" and handler at %pf\n",
			desc, d->chan, d->base, d->irq, handler);
	else
		dev_info(&master->dev,
			"%s-DMA channel %d at address %pK with irq %d"
			" and no handler\n",
			desc, d->chan, d->base, d->irq);
	/* asign description */
	len = strlen(dev_name(&master->dev)) + strlen(desc) + 2;
	d->desc = kmalloc(len, GFP_KERNEL);
	snprintf((char *)d->desc, len, "%s-%s",
		dev_name(&master->dev), desc);
	/* and reset the DMA - just in case */
	writel(BCM2835_DMA_CS_RESET, d->base+BCM2835_DMA_CS);
	writel(0, d->base+BCM2835_DMA_ADDR);
	/* and add the irq handler */
	if (handler) {
		ret = request_irq(d->irq,
				handler,
				0,
				d->desc,
				master);
		if (ret) {
			dev_err(&master->dev,
				"could not request IRQ: %d\n", d->irq);
			bcm2835dma_release_dmachannel(master, d);
			return ret;
		}
	}
	/* assign other data */
	d->handler = handler;
	/* calculate the bus_addr */
	d->bus_addr = BCM2835_DMA_BASE_BUS(d->chan);
	/* and return */
	return 0;
}

static void bcm2835dma_release_dma(struct spi_master *master)
{
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);

	bcm2835dma_release_dmachannel(master, &bs->dma_tx);
	bcm2835dma_release_dmachannel(master, &bs->dma_rx);
	bcm2835dma_release_dmachannel(master, &bs->dma_irq);

	bcm2835dma_release_dmafragment_components(master);
}

static int bcm2835dma_allocate_dma(struct spi_master *master,
				struct platform_device *pdev)
{
	int err;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	/* allocate DMA-channels */
	err = bcm2835dma_allocate_dmachannel(
		master, &bs->dma_tx, NULL, "txdma");
	if (err)
		goto error;
	err = bcm2835dma_allocate_dmachannel(
		master, &bs->dma_rx, NULL, "rxdma");
	if (err)
		goto error;
	err = bcm2835dma_allocate_dmachannel(
		master, &bs->dma_irq,
		bcm2835dma_spi_interrupt_dma_irq,
		"irqdma");
	if (err)
		goto error;

	/* and register the dmafragment_caches */
	err = bcm2835dma_register_dmafragment_components(master);
	if (!err)
		return 0;
error:
	bcm2835dma_release_dma(master);
	return -ENOMEM;
}

static void bcm2835dma_set_gpio_mode(u8 pin, u32 mode)
{
	/* this is a bit of a hack, as there seems to be no official
	   way of doing this... */
	/* TODO - PINMUX */
	u32 *gpio = ioremap(0x20200000, SZ_16K);

	/* map pin */
	u32 *reg = &gpio[pin/10];
	u8 shift = ((pin)%10)*3;
	u32 v = *reg;
	v &= ~(((u32)(7)) << shift);
	v |= (mode & 7) << shift;
	*reg = v;
	iounmap(gpio);
}

static int bcm2835dma_spi_init_pinmode(struct device *dev)
{
	int err;
	err = devm_gpio_request_one(dev,
				BCM2835_SPI_GPIO_MISO,
				GPIOF_IN,
				DRV_NAME":MISO");
	if (err) {
		dev_err(dev, DRV_NAME": problems requesting SPI:MISO"
			" on GPIO %i - err %i\n", BCM2835_SPI_GPIO_MISO,
			err);
		return err;
	}
	err = devm_gpio_request_one(dev,
				BCM2835_SPI_GPIO_MOSI,
				GPIOF_OUT_INIT_HIGH,
				DRV_NAME":MOSI");
	if (err) {
		dev_err(dev, DRV_NAME": problems requesting SPI:MOSI"
			" on GPIO %i - err %i\n", BCM2835_SPI_GPIO_MOSI,
			err);
		goto error_miso;
	}
	err = devm_gpio_request_one(dev,
				BCM2835_SPI_GPIO_SCK,
				GPIOF_OUT_INIT_HIGH,
				DRV_NAME":SCK");
	if (err) {
		dev_err(dev, DRV_NAME": problems requesting SPI:SCK"
			"on GPIO %i - err %i\n", BCM2835_SPI_GPIO_SCK,
			err);
		goto error_mosi;
	}
	/* SET modes to ALT0=4 - unfortunately there is not API for that...
	* (which I found) */
	bcm2835dma_set_gpio_mode(BCM2835_SPI_GPIO_MISO, 4);
	bcm2835dma_set_gpio_mode(BCM2835_SPI_GPIO_MOSI, 4);
	bcm2835dma_set_gpio_mode(BCM2835_SPI_GPIO_SCK,  4);

	return 0;
error_mosi:
	devm_gpio_free(dev, BCM2835_SPI_GPIO_MOSI);
error_miso:
	devm_gpio_free(dev, BCM2835_SPI_GPIO_MISO);
	return err;
}

static void bcm2835dma_spi_restore_pinmodes(struct device *dev)
{
	/* we assume this will reset the MODE */
	devm_gpio_free(dev, BCM2835_SPI_GPIO_MISO);
	devm_gpio_free(dev, BCM2835_SPI_GPIO_MOSI);
	devm_gpio_free(dev, BCM2835_SPI_GPIO_SCK);
}

static void bcm2835dma_cleanup_spi_device_data(
	struct bcm2835dma_spi_device_data *data)
{
	/* remove from chain */
	list_del(&data->spi_device_data_chain);
	/* release GPIO */
	gpio_free(data->cs_gpio);
	/* and release memory */
	kfree(data);
}

static void bcm2835dma_spi_cleanup(struct spi_device *spi)
{
	/* note that surprisingly this does not necessarily
	   get called on driver unload */
	struct bcm2835dma_spi_device_data *data
		= spi_get_ctldata(spi);
	/* release the memory and GPIO allocated for the SPI device */
	if (data) {
		bcm2835dma_cleanup_spi_device_data(data);
		spi_set_ctldata(spi, NULL);
	}
}

static int bcm2835dma_spi_setup(struct spi_device *spi)
{
	struct spi_master *master = spi->master;
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);
	struct bcm2835dma_spi_device_data *data;
	u32 level;
	bool needs_gpioalloc=0;
	u32 tmp;
	int err;

	/* allocate data - if not allocated yet */
	data = spi_get_ctldata(spi);
	if (!data) {
		data = kzalloc(sizeof(*data), GFP_KERNEL);
		if (!data)
			return -ENOMEM;
		spi_set_ctldata(spi, data);
		list_add(&data->spi_device_data_chain,
			&bs->spi_device_data_chain);
		needs_gpioalloc=1;
	}
	/* calculate the real GPIO to use */
	if (spi->master->cs_gpios) {
		data->cs_gpio = spi->cs_gpio;
	} else {
		switch (spi->chip_select) {
		case 0:
			data->cs_gpio = BCM2835_SPI_GPIO_CS0;
			break;
		case 1:
			data->cs_gpio = BCM2835_SPI_GPIO_CS1;
			break;
		default:
			data->cs_gpio = spi->chip_select;
			break;
		}
	}

	/* check gpio for prohibited pins
	   - MISO, MOSI, SCK are not allowed... */
	if ((spi->mode & SPI_NO_CS)) {
		switch (data->cs_gpio) {
		case BCM2835_SPI_GPIO_MISO:
		case BCM2835_SPI_GPIO_MOSI:
		case BCM2835_SPI_GPIO_SCK:
			dev_err(&spi->dev,
				"Chipselect GPIO %i is not allowed"
				" as it is conflicting with the"
				" standard SPI lines\n",
				data->cs_gpio);
			err = -EPERM;
			goto error_free;
			break;
		default:
			break;
		}
	}

	/* set the chip_select fields correctly */
	data->cs_bitfield = ((u32)1) << (data->cs_gpio % 32);
	/* shift right gpio by 5 bit to get the
	   register offset we need to use*/
	data->cs_deselect_gpio_reg =
	data->cs_select_gpio_reg = BCM2835_REG_GPIO_OUTPUT_SET_BASE_BUS
		+ 4 * (data->cs_gpio >> 5);
	/* and create the name for the gpio */
	snprintf(data->cs_name, sizeof(data->cs_name),
		DRV_NAME":CS%i", spi->chip_select);

	/* based od SPI_CS configure the registers */
	if (spi->mode & SPI_CS_HIGH) {
		/* increment the registers accordingly */
		data->cs_deselect_gpio_reg +=
			(BCM2835_REG_GPIO_OUTPUT_CLEAR_BASE_BUS
				- BCM2835_REG_GPIO_OUTPUT_SET_BASE_BUS);
		level=GPIOF_OUT_INIT_LOW;
	} else {
		/* set the registers accordingly */
		data->cs_select_gpio_reg +=
			(BCM2835_REG_GPIO_OUTPUT_CLEAR_BASE_BUS
				- BCM2835_REG_GPIO_OUTPUT_SET_BASE_BUS);
		level=GPIOF_OUT_INIT_LOW;
	}
	/* request the GPIO with correct defaults*/
	if (needs_gpioalloc) {
		err = gpio_request_one(
			data->cs_gpio,
			level,
			data->cs_name);
		if (err)
			goto error_gpio;
	} else {
		/* set level as requested */
		gpio_set_value(data->cs_gpio,
			(level == GPIOF_OUT_INIT_HIGH));
	}

	/* and the CS register values used to configure SPI */
	tmp = BCM2835_SPI_CS_TA
		| BCM2835_SPI_CS_CS_01
		| BCM2835_SPI_CS_CS_10
		| ((spi->mode & SPI_CPOL) ? BCM2835_SPI_CS_CPOL : 0)
		| ((spi->mode & SPI_CPHA) ? BCM2835_SPI_CS_CPHA : 0)
		;
	/* the values used to reset SPI FIFOs*/
	data->spi_reset_fifo = tmp
		| BCM2835_SPI_CS_CLEAR_RX
		| BCM2835_SPI_CS_CLEAR_TX
		;
	/* the values used to reenable DMA transfers */
	data->spi_config = tmp
		| BCM2835_SPI_CS_DMAEN
		;

	return 0;

error_gpio:
	dev_err(&spi->dev, "Error allocating GPIO%i - error %i\n",
		data->cs_gpio, err);
error_free:
	kfree(data);
	dev_set_drvdata(&spi->dev, NULL);
	return err;
}

static int bcm2835dma_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct bcm2835dma_spi *bs;
	struct resource *res;
	int err;

	master = spi_alloc_master(&pdev->dev, sizeof(*bs));
	if (!master) {
		dev_err(&pdev->dev, "spi_alloc_master() failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, master);

	master->mode_bits = BCM2835_SPI_MODE_BITS;
	master->bits_per_word_mask = BIT(8 - 1);
#ifdef CONFIG_MACH_BCM2708
	master->bus_num = pdev->id;
#endif
	master->num_chipselect = BCM2835_SPI_MAX_CS;
	master->setup = bcm2835dma_spi_setup;
	master->cleanup = bcm2835dma_spi_cleanup;
	master->dev.of_node = pdev->dev.of_node;
	master->rt = 1;

	master->transfer = bcm2835dma_spi_transfer;
#ifdef SPI_HAVE_OPTIMIZE
	if (use_optimize) {
		master->optimize_message =
			bcm2835dma_spi_message_optimize;
		master->unoptimize_message =
			bcm2835dma_spi_message_unoptimize;
	}
#endif

	/* not sure if this is needed for the device tree case */
	master->dev.coherent_dma_mask = pdev->dev.coherent_dma_mask;

	bs = spi_master_get_devdata(master);

	/* initialize the queue - it does not get set up when using transfer*/
	INIT_LIST_HEAD(&master->queue);
	/* and the list of per device data - needed for cleanup */
	INIT_LIST_HEAD(&bs->spi_device_data_chain);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"could not get memory resource\n");
		err = -ENODEV;
		goto out_master_put;
	}

	bs->spi_regs = devm_ioremap_resource(&pdev->dev, res);
	if (!bs->spi_regs) {
		dev_err(&pdev->dev,
			"could not request/map memory region\n");
		err = -ENODEV;
		goto out_master_put;
	}

	bs->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(bs->clk)) {
		err = PTR_ERR(bs->clk);
		dev_err(&pdev->dev,
			"could not get clk: %d\n",
			err);
		goto out_master_put;
	}

	clk_prepare_enable(bs->clk);

	/* configure pin function for SPI */
	err = bcm2835dma_spi_init_pinmode(&pdev->dev);
	if (err) {
		dev_err(&pdev->dev,
			"could not register pins and set the mode: %d\n",
			err);
		goto out_release_clock;
	}

	err = spi_register_master(master);
	if (err) {
		dev_err(&pdev->dev,
			"could not register SPI master: %d\n",
			err);
		goto out_release_gpio;
	}

	err = bcm2835dma_allocate_dma(master, pdev);
	if (err) {
		dev_err(&pdev->dev,
			"could not register SPI master: %d\n",
			err);
		goto out_release_dma;
	}

	/* initialise the hardware with a reset of the SPI FIFO
	   disabling an existing transfer */
	writel(
		BCM2835_SPI_CS_CLEAR_RX | BCM2835_SPI_CS_CLEAR_TX,
		bs->spi_regs+BCM2835_SPI_CS);

	bs->stats_attr.attr.name = "stats";
	bs->stats_attr.attr.mode = S_IRUSR | S_IWUSR;
	bs->stats_attr.show      = bcm2835dma_sysfs_show_stats;
	bs->stats_attr.store     = bcm2835dma_sysfs_triggerdump;
	err = device_create_file(&master->dev, &bs->stats_attr);
	if (err)
		goto out_release_dma;

	return 0;
out_release_dma:
	bcm2835dma_release_dma(master);
/*out_release_dev: not used */
	spi_unregister_master(master);
out_release_gpio:
	bcm2835dma_spi_restore_pinmodes(&pdev->dev);
out_release_clock:
	clk_disable_unprepare(bs->clk);
out_master_put:
	spi_master_put(master);
	return err;
}

static int bcm2835dma_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct bcm2835dma_spi *bs = spi_master_get_devdata(master);

	/* release the spi_dev parts that do not get released otherwise
	 * cleanup does not get called on the spi_device when unloading
	 * the module
	 */
	while (!list_empty(&bs->spi_device_data_chain)) {
		struct bcm2835dma_spi_device_data *data
			= list_first_entry(&bs->spi_device_data_chain,
					typeof(*data),
					spi_device_data_chain);
		bcm2835dma_cleanup_spi_device_data(data);
	}

	/* release pool - also releases all objects */
	bcm2835dma_release_dma(master);

	/* in correct sequence */
	spi_unregister_master(master);

	/* Clear FIFOs, and disable the HW block */
	if (bs->spi_regs)
		writel(
			BCM2835_SPI_CS_CLEAR_RX
			| BCM2835_SPI_CS_CLEAR_TX,
			bs->spi_regs+BCM2835_SPI_CS);

	bcm2835dma_spi_restore_pinmodes(&pdev->dev);

	clk_disable_unprepare(bs->clk);

	spi_master_put(master);

	return 0;
}

static const struct of_device_id bcm2835dma_spi_match[] = {
	{ .compatible = "brcm,bcm2835-spi", },
	{}
};
MODULE_DEVICE_TABLE(of, bcm2835dma_spi_match);

/* and "normal" aliases */
#ifdef CONFIG_MACH_BCM2708
static const struct platform_device_id bcm2835dma_id_table[] = {
	{ "bcm2835_spi", 2835 },
	{ "bcm2708_spi", 2708 },
	{ },
};
#endif

static struct platform_driver bcm2835dma_spi_driver = {
	.driver	        = {
		.name		= DRV_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= bcm2835dma_spi_match,
	},
	.probe		= bcm2835dma_spi_probe,
	.remove		= bcm2835dma_spi_remove,
#ifdef CONFIG_MACH_BCM2708
	.id_table       = bcm2835dma_id_table,
#endif
};
module_platform_driver(bcm2835dma_spi_driver);

MODULE_DESCRIPTION("SPI controller driver for Broadcom BCM2835");
MODULE_AUTHOR("Chris Boot <bootc@bootc.net>, "
	"Martin Sperl <kernel@martin.sperl.org>");
MODULE_LICENSE("GPL v2");
