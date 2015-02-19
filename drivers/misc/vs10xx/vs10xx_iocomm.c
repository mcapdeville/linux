/* ------------------------------------------------------------------------------------------------------------------------------
    vs10xx io functions.
    Copyright (C) 2010-2013 Richard van Paasen <rvpaasen@t3i.nl>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
   ----------------------------------------------------------------------------------------------------------------------------- */

#include "vs10xx_common.h"
#include "vs10xx_iocomm.h"
#include "vs10xx_queue.h"
#include "vs10xx_device.h"

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>

/* interrupt mode */
static int irqmode = 1;
module_param(irqmode, int, 0644);

/* dreq status */
static int skipdreq = 0;
module_param(skipdreq, int, 0644);

/* hwreset */
static int hwreset = 0;
module_param(hwreset, int, 0644);

/* hwreset */
static int use_dma = 0;
module_param(use_dma, int, 0644);

struct vs10xx_chip {
	int dreq_val;
	int gpio_reset;
	int gpio_dreq;
	int irq_dreq;
	struct spi_device *spi_ctrl;
	struct spi_device *spi_data;
	wait_queue_head_t wq;
	spinlock_t dreq_lock;
};


static struct vs10xx_chip vs10xx_chips[VS10XX_MAX_DEVICES];

/* ----------------------------------------------------------------------------------------------------------------------------- */
/* VS10XX SPI MISC                                                                                                               */
/* ----------------------------------------------------------------------------------------------------------------------------- */

static irqreturn_t vs10xx_io_irq(int irq, void *arg) {
/*
 *  Descr:  interrupt handler, record DREQ value and wakeup waiting task
 *  Return: irq handled
 */

	int id = (int)arg;

	vs10xx_chips[id].dreq_val = gpio_get_value(vs10xx_chips[id].gpio_dreq);

	if (vs10xx_chips[id].dreq_val) {
		wake_up(&vs10xx_chips[id].wq);
	}

	return IRQ_HANDLED;
}

int vs10xx_io_isready(int id) {
/*
 *  Descr:  Test if DREQ is high
 *  Return: 1 --> DREQ is high (ready)
 *          0 --> DREQ is low (busy)
 */

	int dreq_val = (irqmode ? vs10xx_chips[id].dreq_val : gpio_get_value(vs10xx_chips[id].gpio_dreq));

	vs10xx_nsy("id:%d dreq_val:%d", id, dreq_val);

	return (skipdreq ? 1 : dreq_val);
}

int vs10xx_io_wtready(int id, unsigned timeout) {
/*
 *  Descr:  Wait timeout [msec] for DREQ to become high
 *  Return: 1 --> DREQ is high (ready)
 *          0 --> timeout (busy)
 */

	if (irqmode) {

		spin_lock_irq(&vs10xx_chips[id].dreq_lock);

		if (!vs10xx_chips[id].dreq_val) {
			wait_event_interruptible_lock_irq_timeout(vs10xx_chips[id].wq, vs10xx_chips[id].dreq_val, vs10xx_chips[id].dreq_lock, msecs_to_jiffies(timeout));
		}

		spin_unlock_irq(&vs10xx_chips[id].dreq_lock);

	} else {

		unsigned long end = jiffies + msecs_to_jiffies(timeout);

		while ( !vs10xx_io_isready(id) && (jiffies < end) ) {

			msleep(1);
		}
	}

	return vs10xx_io_isready(id);
}

void vs10xx_io_reset(int id) {
/*
 *  Descr:  Reset by pulling down gpio
 *  Return: -
 */

	if (hwreset) {
		gpio_set_value(vs10xx_chips[id].gpio_reset, 0);
		udelay(50);
		gpio_set_value(vs10xx_chips[id].gpio_reset, 1);
		udelay(1800);
	}
}

/* ----------------------------------------------------------------------------------------------------------------------------- */
/* VS10XX SPI TRANSFER                                                                                                           */
/* ----------------------------------------------------------------------------------------------------------------------------- */

int vs10xx_io_ctrl_xf(int id, char *txbuf, unsigned txlen, char *rxbuf, unsigned rxlen) {

	struct spi_device *device = vs10xx_chips[id].spi_ctrl;
	struct spi_message	spi_mesg;
	struct spi_transfer	spi_xfer_tx;
	struct spi_transfer	spi_xfer_rx;
	int status = 0;
	dma_addr_t dma_base_tx=0;
	dma_addr_t dma_base_rx=0;

	spi_message_init(&spi_mesg);

	if (txbuf) {
		memset(&spi_xfer_tx, 0, sizeof spi_xfer_tx);
		spi_xfer_tx.tx_buf = txbuf;
		if (use_dma) {
			dma_base_tx = dma_map_single (&vs10xx_chips[id].spi_ctrl->dev, txbuf,txlen,DMA_TO_DEVICE);
			spi_mesg.is_dma_mapped = 1;
			spi_xfer_tx.tx_dma = dma_base_tx;
		}
		spi_xfer_tx.len = txlen;
		spi_xfer_tx.delay_usecs = rxbuf ? 0 : 10;
		spi_message_add_tail(&spi_xfer_tx, &spi_mesg);
	}

	if (rxbuf) {
		memset(&spi_xfer_rx, 0, sizeof spi_xfer_rx);
		spi_xfer_rx.rx_buf = rxbuf;
		if (use_dma) {
			dma_base_rx = dma_map_single (&vs10xx_chips[id].spi_ctrl->dev, rxbuf,rxlen,DMA_FROM_DEVICE);
			spi_mesg.is_dma_mapped = 1;
			spi_xfer_rx.rx_dma = dma_base_rx;
		}
		spi_xfer_rx.len = rxlen;
		spi_xfer_rx.delay_usecs = 10;
		spi_message_add_tail(&spi_xfer_rx, &spi_mesg);
	}

	status = spi_sync(device, &spi_mesg);
	if (status < 0) {
		vs10xx_err("id:%d spi_sync failed", id);
	}

	if (use_dma) {
	       if (rxbuf)
			dma_unmap_single(&vs10xx_chips[id].spi_ctrl->dev,dma_base_rx,rxlen,DMA_FROM_DEVICE);
	       if (txbuf)
			dma_unmap_single(&vs10xx_chips[id].spi_ctrl->dev,dma_base_tx,txlen,DMA_TO_DEVICE);
	}

	return status;
}

int vs10xx_io_ctrl_xfdata(int id, char *txbuf, unsigned txlen, char *rxbuf, unsigned rxlen, unsigned repeat) {

	struct spi_device *device = vs10xx_chips[id].spi_ctrl;
	struct spi_message	spi_mesg;
	struct spi_transfer	*spi_xfer_tx;
	struct spi_transfer	*spi_xfer_rx;
	char			*rxptr=rxbuf;
	dma_addr_t dma_base_tx=0,dma_base_rx=0;
	dma_addr_t dma_rx_ptr=0;
	int status = 0,i;

	spi_message_init(&spi_mesg);

	if (txbuf) {
		if (use_dma) {
			dma_base_tx = dma_map_single (&vs10xx_chips[id].spi_ctrl->dev, txbuf,txlen,DMA_TO_DEVICE);
			spi_mesg.is_dma_mapped = 1;
		}
		spi_xfer_tx = kzalloc(sizeof( struct spi_transfer )*repeat,GFP_KERNEL);
	}

	if (rxbuf) {
		if (use_dma) {
			dma_base_rx = dma_map_single (&vs10xx_chips[id].spi_ctrl->dev, rxbuf,rxlen*repeat,DMA_FROM_DEVICE);
			dma_rx_ptr = dma_base_rx;
			spi_mesg.is_dma_mapped = 1;
		}
		spi_xfer_rx = kzalloc(sizeof( struct spi_transfer )*repeat,GFP_KERNEL);
	}

	for (i=0 ; i< repeat ; i++)
	{
		if (txbuf) {
			spi_xfer_tx[i].tx_buf = txbuf;
			if (use_dma)
				spi_xfer_tx[i].tx_dma = dma_base_tx;
			spi_xfer_tx[i].len = txlen;
			spi_xfer_tx[i].delay_usecs = rxbuf ? 0 : 10;
			spi_message_add_tail(&(spi_xfer_tx[i]), &spi_mesg);
		}

		if (rxbuf) {
			spi_xfer_rx[i].rx_buf = rxptr;
			if (use_dma)
				spi_xfer_rx[i].rx_dma = dma_rx_ptr;
			spi_xfer_rx[i].len = rxlen;
			spi_xfer_rx[i].delay_usecs = 10;
			spi_xfer_rx[i].cs_change = 1;
			spi_message_add_tail(&(spi_xfer_rx[i]), &spi_mesg);
			rxptr+=rxlen;
			dma_rx_ptr+=rxlen;
		}
	}

	status = spi_sync(device, &spi_mesg);

	if (status < 0) {
		vs10xx_err("id:%d spi_sync failed (%d)", id, status);
	}

	if (rxbuf)
	{
		kfree(spi_xfer_rx);
		if (use_dma)
			dma_unmap_single(&vs10xx_chips[id].spi_ctrl->dev,dma_base_rx,rxlen*repeat,DMA_FROM_DEVICE);
	}

	if (txbuf) {
		kfree(spi_xfer_tx);
		if (use_dma)
			dma_unmap_single(&vs10xx_chips[id].spi_ctrl->dev,dma_base_tx,txlen,DMA_TO_DEVICE);
	}

	return status;
}

int vs10xx_io_data_rx(int id, char *rxbuf, unsigned rxlen) {

	struct spi_device *device = vs10xx_chips[id].spi_data;
	struct spi_message	spi_mesg;
	struct spi_transfer	spi_xfer_rx;
	int status = 0;
	dma_addr_t dma_base_rx=0;

	spi_message_init(&spi_mesg);

	memset(&spi_xfer_rx, 0, sizeof spi_xfer_rx);
	spi_xfer_rx.rx_buf = rxbuf;
	if (use_dma) {
		dma_base_rx = dma_map_single (&vs10xx_chips[id].spi_data->dev, rxbuf,rxlen,DMA_FROM_DEVICE);
		spi_mesg.is_dma_mapped = 1;
		spi_xfer_rx.rx_dma = dma_base_rx;
	}
	spi_xfer_rx.len = rxlen;
	spi_message_add_tail(&spi_xfer_rx, &spi_mesg);

	status = spi_sync(device, &spi_mesg);
	if (status < 0) {
		vs10xx_err("id:%d spi_sync failed", id);
	}

	if(use_dma)
		dma_unmap_single(&vs10xx_chips[id].spi_data->dev,dma_base_rx,rxlen,DMA_FROM_DEVICE);

	return status;
}

int vs10xx_io_data_tx(int id, char *txbuf, unsigned txlen) {

	struct spi_device *device = vs10xx_chips[id].spi_data;
	struct spi_message	spi_mesg;
	struct spi_transfer	spi_xfer_tx;
	int status = 0;
	dma_addr_t dma_base_tx=0;

	spi_message_init(&spi_mesg);

	memset(&spi_xfer_tx, 0, sizeof spi_xfer_tx);
	spi_xfer_tx.tx_buf = txbuf;
	if (use_dma) {
		dma_base_tx = dma_map_single (&vs10xx_chips[id].spi_data->dev, txbuf,txlen,DMA_TO_DEVICE);
		spi_mesg.is_dma_mapped = 1;
		spi_xfer_tx.tx_dma = dma_base_tx;
	}
	spi_xfer_tx.len = txlen;
	spi_message_add_tail(&spi_xfer_tx, &spi_mesg);

	status = spi_sync(device, &spi_mesg);
	if (status < 0) {
		vs10xx_err("id:%d spi_sync failed", id);
	}

	if (use_dma)
		dma_unmap_single(&vs10xx_chips[id].spi_data->dev,dma_base_tx,txlen,DMA_TO_DEVICE);

	return status;
}

/* ----------------------------------------------------------------------------------------------------------------------------- */
/* VS10XX SPI PROBES                                                                                                             */
/* ----------------------------------------------------------------------------------------------------------------------------- */

static int vs10xx_spi_ctrl_probe(struct spi_device *spi) {

	int status = 0;
	struct vs10xx_board_info* boardinfo = (struct vs10xx_board_info*) spi->dev.platform_data;

	printk("vs10xx : probing ctrl\n");
	if (boardinfo == NULL) {
		vs10xx_err("no board info provided for vs10xx-ctrl device");
		status = -1;
	}

	if (status == 0) {
		if (boardinfo->device_id < 0 || boardinfo->device_id >= VS10XX_MAX_DEVICES) {
			vs10xx_err("id out of range:%d", boardinfo->device_id);
			status = -1;
		}
	}

	if (status == 0) {
		vs10xx_chips[boardinfo->device_id].spi_ctrl = spi;
		vs10xx_chips[boardinfo->device_id].gpio_reset = boardinfo->gpio_reset;
		vs10xx_chips[boardinfo->device_id].gpio_dreq = boardinfo->gpio_dreq;

		vs10xx_inf("id:%d ctrl spi:%d.%d gpio_reset:%d gpio_dreq:%d",
			boardinfo->device_id, spi->master->bus_num, spi->chip_select, boardinfo->gpio_reset, boardinfo->gpio_dreq);

		if (vs10xx_chips[boardinfo->device_id].spi_ctrl && vs10xx_chips[boardinfo->device_id].spi_data)
			status = vs10xx_create_device(boardinfo->device_id);
	}

	if (use_dma) {
		if (dma_set_mask(&spi->dev,0xffffffff) == 0)
		{
			vs10xx_inf("id:%d ctrl spi: cant set dma_mask\n",boardinfo->device_id);
			use_dma=0;
		}
	}

	return status;
}


static struct spi_driver vs10xx_spi_ctrl = {
	.driver = {
		.name		= VS10XX_SPI_CTRL,
		.owner		= THIS_MODULE,
	},
	.probe		= vs10xx_spi_ctrl_probe,
//	.remove		= __devexit_p(vs10xx_ctrl_remove),
};


static int vs10xx_spi_data_probe(struct spi_device *spi) {

	int status = 0;
	struct vs10xx_board_info* boardinfo = (struct vs10xx_board_info*) spi->dev.platform_data;

	printk("vs10xx : probing data\n");
	if (boardinfo == NULL) {
		vs10xx_err("no board info provided for vs10xx-data device");
		status = -1;
	}

	if (status == 0) {
		if (boardinfo->device_id < 0 || boardinfo->device_id >= VS10XX_MAX_DEVICES) {
			vs10xx_err("id out of range:%d", boardinfo->device_id);
			status = -1;
		}
	}

	if (status == 0) {
		vs10xx_chips[boardinfo->device_id].spi_data = spi;

		vs10xx_inf("id:%d data spi:%d.%d gpio_reset:%d gpio_dreq:%d",
			boardinfo->device_id, spi->master->bus_num, spi->chip_select, boardinfo->gpio_reset, boardinfo->gpio_dreq);

		if (vs10xx_chips[boardinfo->device_id].spi_ctrl && vs10xx_chips[boardinfo->device_id].spi_data)
			status = vs10xx_create_device(boardinfo->device_id);
	}

	if (use_dma) {
		if (dma_set_mask(&spi->dev,0xffffffff) == 0)
		{
			vs10xx_inf("id:%d data spi: cant set dma_mask\n",boardinfo->device_id);
			use_dma=0;
		}
	}

	return status;
}


static struct spi_driver vs10xx_spi_data = {
	.driver = {
		.name		= VS10XX_SPI_DATA,
		.owner		= THIS_MODULE,
	},
	.probe		= vs10xx_spi_data_probe,
//	.remove		= __devexit_p(vs10xx_data_remove),
};


/* ----------------------------------------------------------------------------------------------------------------------------- */
/* VS10XX IO REG/UNREG                                                                                                           */
/* ----------------------------------------------------------------------------------------------------------------------------- */

int vs10xx_io_register(void) {

	int s1 = 0, s2 = 0;

	/* register vs10xx ctrl spi driver */
	s1 = spi_register_driver(&vs10xx_spi_ctrl);
	if (s1 < 0) {
		vs10xx_err("spi_register_driver: ctrl");
	}

	/* register vs10xx data spi driver */
	s2 = spi_register_driver(&vs10xx_spi_data);
	if (s2 < 0) {
		vs10xx_err("spi_register_driver: data");
	}

	return (s1==0 && s2==0 ? 0  : -1);
}


void vs10xx_io_unregister(void) {

	/* unregister spi devices */
	spi_unregister_driver(&vs10xx_spi_ctrl);
	spi_unregister_driver(&vs10xx_spi_data);
}

/* ----------------------------------------------------------------------------------------------------------------------------- */
/* VS10XX IO INIT/EXIT                                                                                                           */
/* ----------------------------------------------------------------------------------------------------------------------------- */

int vs10xx_io_init(int id) {

	struct vs10xx_chip *chip;
	int status = -1;

	if (id < 0 || id >= VS10XX_MAX_DEVICES) {

		vs10xx_err("id:%d out of range", id);

	} else {

		chip = &vs10xx_chips[id];

		if (chip->spi_ctrl == NULL || chip->spi_data == NULL) {

			vs10xx_dbg("id:%d no board config", id);

		} else {

			/* initialize wait queue */
			init_waitqueue_head(&chip->wq);

			/* initialize spin_lock */
			spin_lock_init(&chip->dreq_lock);


			/* request reset signal */
			status = gpio_request(chip->gpio_reset, "vs10xx_reset");
			if (status < 0) {
				vs10xx_err("gpio_request gpio_reset:%d", chip->gpio_reset);
				chip->gpio_reset = -1;
			}

			if (status == 0) {
				/* set gpio_reset as output, keep vs10xx in reset */
				status = gpio_direction_output(chip->gpio_reset, (hwreset ? 0 : 1));
				if (status < 0) {
					vs10xx_err("gpio_direction_output gpio_reset:%d", chip->gpio_reset);
				} else {
					gpio_set_value(chip->gpio_reset, (hwreset ? 0 : 1));
				}
			}

			if (status == 0) {
				/* request dreq signal */
				status = gpio_request(chip->gpio_dreq, "vs10xx_dreq");
				if (status < 0) {
					vs10xx_err("gpio_request gpio_dreq:%d", chip->gpio_dreq);
					chip->gpio_dreq = -1;
				}
			}

			if (status == 0) {
				/* set gpio_dreq as input */
				status = gpio_direction_input(chip->gpio_dreq);
				if (status < 0) {
					vs10xx_err("gpio_direction_input gpio_dreq:%d", chip->gpio_dreq);
				} else {
					chip->dreq_val = gpio_get_value(chip->gpio_dreq);
				}
			}

			if (irqmode) {
				if (status == 0) {
					/* request dreq irq */
					chip->irq_dreq = gpio_to_irq(chip->gpio_dreq);
					if (chip->irq_dreq < 0) {
						vs10xx_err("gpio_to_irq gpio_dreq:%d", chip->gpio_dreq);
						status = -1;
					}
				}

				if (status == 0) {
					/* request irq for gpio_dreq */
					status = request_irq(chip->irq_dreq, vs10xx_io_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, VS10XX_NAME, (void*)id);
					if (status < 0) {
						vs10xx_err("request_irq irq_dreq:%d", chip->irq_dreq);
						chip->irq_dreq = -1;
					}
				}
			}
		}
	}

	if (status == 0) {
		vs10xx_inf("id:%d gpio_reset:%d gpio_dreq:%d irq_dreq:%d", id, chip->gpio_reset, chip->gpio_dreq, chip->irq_dreq);
	}

	return status;
}

int vs10xx_io_set_ctrl_clock(int id,int Hz)
{
	vs10xx_chips[id].spi_ctrl->max_speed_hz=Hz;
	return spi_setup(vs10xx_chips[id].spi_ctrl);
}

int vs10xx_io_set_data_clock(int id,int Hz)
{
	vs10xx_chips[id].spi_data->max_speed_hz=Hz;
	return spi_setup(vs10xx_chips[id].spi_data);
}

void vs10xx_io_exit(int id) {

	struct vs10xx_chip *chip = &vs10xx_chips[id];

	/* release dreq irq */
	if (irqmode && chip->irq_dreq > 0) {
		synchronize_irq(chip->irq_dreq);
		free_irq(chip->irq_dreq, (void*)id);
	}

	/* free gpio dreq */
	if (chip->gpio_dreq > 0) {
	    gpio_free(chip->gpio_dreq);
	}

	/* free gpio reset */
	if (chip->gpio_reset > 0) {

		if (hwreset) {
			gpio_set_value(chip->gpio_reset, 0);
		}

		gpio_free(chip->gpio_reset);
	}
}
