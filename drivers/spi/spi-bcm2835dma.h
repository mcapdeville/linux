/*
 * Driver for Broadcom BCM2835 SPI Controllers
 *
 * Copyright (C) 2012 Chris Boot
 * Copyright (C) 2013 Stephen Warren
 * Copyright (C) 2013 Martin Sperl
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

#include <linux/spi/spi-dmafragment.h>
#include <linux/dma/bcm2835-dmafragment.h>
#include <linux/spi/bcm2835.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/clk.h>
#include <mach/dma.h>

/* DMA addresses of some of the used registers */
#define BCM2835_REG_GPIO_OUTPUT_SET_BASE_BUS   0x7e20001C
#define BCM2835_REG_GPIO_OUTPUT_CLEAR_BASE_BUS 0x7e200028
#define BCM2835_REG_COUNTER_BASE_BUS           0x7e003000
#define BCM2835_REG_COUNTER_64BIT_BUS          0x7e003004

struct bcm2835_dmachannel {
	void __iomem *base;
	dma_addr_t bus_addr;
	int chan;
	int irq;
	irq_handler_t handler;
	const char *desc;
};

struct bcm2835dma_dma_status {
	u32 v1;
	u32 v2;
};

struct bcm2835dma_spi_device_data {
	struct list_head spi_device_data_chain;
	/* the SPI Registers for Set/Reset values */
	u32 spi_config;
	u32 spi_reset_fifo;
	/* the chip select parameters */
	dma_addr_t cs_select_gpio_reg;
	dma_addr_t cs_deselect_gpio_reg;
	u32        cs_bitfield;
	u8         cs_gpio;
	char       cs_name[20];
};

struct bcm2835dma_spi_merged_dma_fragment {
	struct spi_merged_dma_fragment spi_fragment;

	u32 *txdma_link_to_here;
	u32 *total_length;

	u32 speed_hz;
	u32 speed_cdiv;
	u32 delay_half_cycle_cs_deselect_dma_length;
	u32 delay_half_cycle_post_rx_dma_length;
};

struct bcm2835dma_spi {
	/* the SPI registers */
	void __iomem *spi_regs;
	void __iomem *gpio_regs;
	/* the clock */
	struct clk *clk;
	/* the DMA channels allocated */
	struct bcm2835_dmachannel dma_tx;
	struct bcm2835_dmachannel dma_rx;
	struct bcm2835_dmachannel dma_irq;
	/* the DMA-able pool we use to allocate control blocks from */
	struct dma_pool *pool;
	/* the fragment caches */
	struct dma_fragment_cache fragment_merged;
	struct dma_fragment_cache fragment_setup_spi;
	struct dma_fragment_cache fragment_transfer;
	struct dma_fragment_cache fragment_cs_deselect;
	struct dma_fragment_cache fragment_delay;
	struct dma_fragment_cache fragment_trigger_irq;
	struct dma_fragment_cache fragment_message_finished;
	/* the device configs list */
	struct list_head spi_device_data_chain;
	/* the device statistics */
	struct device_attribute stats_attr;
	u64 count_scheduled_msg_dma_restarted;
	u64 count_scheduled_msg_dma_running;
	u64 count_spi_messages;
	u64 count_spi_optimized_messages;
	u64 count_completed_on_retry;
	u64 count_dma_interrupts;

	bool cb_chain_complete_running;
	const char *last_dma_schedule_type;
	void (*last_complete)(void *);
};

int bcm2835dma_register_dmafragment_components(struct spi_master *);
void bcm2835dma_release_dmafragment_components(struct spi_master *);

/* the interrupt-handlers */
irqreturn_t bcm2835dma_spi_interrupt_dma_tx(int irq, void *dev_id);

/**
 * bcm2835dma_spi_message_to_dma_fragment - converts a spi_message to a
 *  dma_fragment
 * @msg:  the spi message to convert
 * @flags: some flags
 * @gfpflags: flags for allocation
 * notes:
 *  with minimal effort this probably could get added to the spi framework
 */
struct spi_merged_dma_fragment *bcm2835dma_spi_message_to_dma_fragment(
	struct spi_message *msg, int flags, gfp_t gfpflags);

/**
 * dmalink_to_cb - casts dma_link to a control_block
 * @dmalink: the dmalink to use
 */
static inline struct bcm2835_dma_cb *dma_link_to_cb(
	struct dma_link *link)
{
	return (struct bcm2835_dma_cb *)(link->cb);
}

/**
 * link_dma_link - links the second dma_link to get executed after the first
 * @first: the dma_link that is linked to the next
 * @second: the dma_link that is being linked to the first
 */
static inline void link_dma_link(struct dma_link *first,
				struct dma_link *second) {
	dma_link_to_cb(first)->next = second->cb_dma;
}
