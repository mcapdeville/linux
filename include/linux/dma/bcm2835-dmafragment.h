/*
 * DMA control block definition - may be coming from dmaengine
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

#ifndef __BCM2835_DMA_H
#define __BCM2835_DMA_H

#include <linux/types.h>
#include <linux/device.h>
#include <linux/dma-fragment.h>

/* the DMA registers and their bitflags */
#define BCM2835_DMA_CS                                  0x00
#define BCM2835_DMA_CS_ACTIVE				(1 <<  0)
#define BCM2835_DMA_CS_END				(1 <<  1)
#define BCM2835_DMA_CS_INT				(1 <<  2)
#define BCM2835_DMA_CS_DREQ				(1 <<  3)
#define BCM2835_DMA_CS_ISPAUSED				(1 <<  4)
#define BCM2835_DMA_CS_ISHELD				(1 <<  5)
#define BCM2835_DMA_CS_WAITING_FOR_OUTSTANDING_WRITES	(1 <<  6)
/* bit 7: reserved */
#define BCM2835_DMA_CS_ERR				(1 <<  8)
/* bit 9-15: reserved */
#define BCM2835_DMA_CS_PRIORITY(x)			(((x)&0x0f) << 16)
#define BCM2835_DMA_CS_PANICPRIORITY(x)			(((x)&0x0f) << 20)
/* bit 24-27: reserved */
#define BCM2835_DMA_CS_WAIT_FOR_OUTSTANDING_WRITES	(1 << 28)
#define BCM2835_DMA_CS_DISDEBUG				(1 << 29)
#define BCM2835_DMA_CS_ABORT				(1 << 30)
#define BCM2835_DMA_CS_RESET				(1 << 31)

#define BCM2835_DMA_ADDR                                0x04
#define BCM2835_DMA_TI                                  0x08
#define BCM2835_DMA_TI_INT_EN				(1 <<  0)
#define BCM2835_DMA_TI_TDMODE				(1 <<  1)
#define BCM2835_DMA_TI_RESERVED				(1 <<  2)
#define BCM2835_DMA_TI_WAIT_RESP			(1 <<  3)
#define BCM2835_DMA_TI_D_INC				(1 <<  4)
#define BCM2835_DMA_TI_D_WIDTH				(1 <<  5)
#define BCM2835_DMA_TI_D_DREQ				(1 <<  6)
#define BCM2835_DMA_TI_D_IGNORE				(1 <<  7)
#define BCM2835_DMA_TI_S_INC				(1 <<  8)
#define BCM2835_DMA_TI_S_WIDTH				(1 <<  9)
#define BCM2835_DMA_TI_S_DREQ				(1 << 10)
#define BCM2835_DMA_TI_S_IGNORE				(1 << 11)
#define BCM2835_DMA_TI_BURST(x)				(((x)&0x0f) << 12)
#define BCM2835_DMA_TI_PER_MAP(x)			(((x)&0x1f) << 16)
#define BCM2835_DMA_TI_WAITS(x)				(((x)&0x1f) << 21)
#define BCM2835_DMA_TI_NO_WIDE_BURSTS			(1 << 26)
/* bit 27-31: reserved */

#define BCM2835_DMA_S_ADDR                              0x0C
#define BCM2835_DMA_D_ADDR                              0x10
#define BCM2835_DMA_LEN                                 0x14
#define BCM2835_DMA_STRIDE                              0x18
#define BCM2835_DMA_NEXT                                0x1C
#define BCM2835_DMA_DEBUG                               0x20

#define BCM2835_DMA_DREQ_NONE                           0
#define BCM2835_DMA_DREQ_DSI                            1
#define BCM2835_DMA_DREQ_PCM_TX                         2
#define BCM2835_DMA_DREQ_PCM_RX                         3
#define BCM2835_DMA_DREQ_SMI                            4
#define BCM2835_DMA_DREQ_PWM                            5
#define BCM2835_DMA_DREQ_SPI_TX                         6
#define BCM2835_DMA_DREQ_SPI_RX                         7
#define BCM2835_DMA_DREQ_BSC_TX                         8
#define BCM2835_DMA_DREQ_BSC_RX                         9
#define BCM2835_DMA_DREQ_UNUSED                         10
#define BCM2835_DMA_DREQ_EMMC                           11
#define BCM2835_DMA_DREQ_UART_TX                        12
#define BCM2835_DMA_DREQ_SDHOST                         13
#define BCM2835_DMA_DREQ_UART_RX                        14
#define BCM2835_DMA_DREQ_DSI2                           15
#define BCM2835_DMA_DREQ_SLIM_MCTX                      16
#define BCM2835_DMA_DREQ_HDMI                           17
#define BCM2835_DMA_DREQ_SLIM_MCRX                      18
#define BCM2835_DMA_DREQ_SLIM_DC0                       19
#define BCM2835_DMA_DREQ_SLIM_DC1                       20
#define BCM2835_DMA_DREQ_SLIM_DC2                       21
#define BCM2835_DMA_DREQ_SLIM_DC3                       22
#define BCM2835_DMA_DREQ_SLIM_DC4                       23
#define BCM2835_DMA_DREQ_SCALER_FIFO_0_SMI              24
#define BCM2835_DMA_DREQ_SCALER_FIFO_1_SMI              25
#define BCM2835_DMA_DREQ_SCALER_FIFO_2_SMI              26
#define BCM2835_DMA_DREQ_SLIM_DC5                       27
#define BCM2835_DMA_DREQ_SLIM_DC6                       28
#define BCM2835_DMA_DREQ_SLIM_DC7                       29
#define BCM2835_DMA_DREQ_SLIM_DC8                       30
#define BCM2835_DMA_DREQ_SLIM_DC9                       31

#define BCM2835_REG_DMA0_BASE_BUS              0x7E007000
#define BCM2835_REG_DMA15_BASE_BUS             0x7EE05000

#define BCM2835_DMA_BASE_BUS(channel)				\
	((channel == 15) ? BCM2835_REG_DMA15_BASE_BUS		\
		: BCM2835_REG_DMA0_BASE_BUS + 256*channel)

/**
 * struct bcm2835_dma_cb the DMA control block
 * @ti: configuration register
 * @src: source-bus address for transfer
 * @dst: destination-bus address for transfer
 * @length: the length of the transfer
 * @stride_src: striding for src in 2D DMA-mode
 * @stride_dst: striding for dst in 2D DMA-mode
 * @next: the link to the next DMA control block to get exectued
 *        (0 for none)
 * @pad: padding - can get used for some data without having to
 *       allocate extra dma memory
 */
struct bcm2835_dma_cb {
	u32        ti;
	dma_addr_t src;
	dma_addr_t dst;
	u32        length;
	u32        stride;
	dma_addr_t next;
	u32        pad[2];
};

struct bcm2835_dma_cb_stride {
	s16 src;
	s16 dst;
};

static inline u32 bcm2835_dma_cb_compose_stride(
	s16 src_stride, s16 dst_stride)
{
	struct bcm2835_dma_cb_stride tmp;
	tmp.src = src_stride;
	tmp.dst = dst_stride;
	return *((u32 *)&tmp);
}

static inline int bcm2835_link_dma_link(
	struct dma_link *from, struct dma_link *to)
{
	dma_addr_t next = (to) ? to->cb_dma : 0;
	writel(next,
		&((struct bcm2835_dma_cb *)from->cb)->next);
	/* add a barrier here */
	dsb();
	return 0;
}

void bcm2835_dma_cb_dump_str(
	struct bcm2835_dma_cb *dmablock,
	dma_addr_t dmablock_dma,
	int tindent,
	char *buffer, size_t size);

void bcm2835_dma_cb_dump(
	struct bcm2835_dma_cb *dmablock,
	dma_addr_t dmablock_dma,
	struct device *dev,
	int tindent);

int bcm2835_dma_reg_dump_str(
	void *base, int tindent,
	char *buffer, size_t size);

void bcm2835_dma_reg_dump(
	void *base,
	struct device *dev,
	int tindent);


static inline void bcm2835_dma_link_dump(
	struct dma_link *link,
	struct device *dev,
	int tindent)
{
	bcm2835_dma_cb_dump(
		link->cb,
		link->cb_dma,
		dev,
		tindent);
}

#define BCM2835_DMA_CB_MEMBER_DMA_ADDR(link, member)	\
	(link->cb_dma + offsetof(struct bcm2835_dma_cb, member))

/* delay function definitions to get a certain delay via DMA only */
/* empirical measurements of 3 DMAs (set GPIO, delay_length,clear GPIO)
 * testing with the following TI flags:
 *   BCM2835_DMA_TI_WAIT_RESP
 *		| BCM2835_DMA_TI_WAITS(0x1f)
 *		| BCM2835_DMA_TI_NO_WIDE_BURSTS
 *		| BCM2835_DMA_TI_D_IGNORE
 * * a transfer from a Source to "nirvana" (FLAGS)
 * * a transfer from "nirvana" to "nirvana" (FLAGS| BCM2835_DMA_TI_S_IGNORE)
 * and with the following "polling" policy to see if DMA has finished:
 * * delay each loop by 10ms with msleep(10) and low CPU utilization
 *   while(readl(DMA)& BCM2835_DMA_CS_ACTIVE) { msleep(10); }
 * * no delay with high CPU utilization
 *   while(readl(DMA)& BCM2835_DMA_CS_ACTIVE) { ; }
 * shows the following timings on a logic analyzer:
   Length	SRC-10ms	SRC-nodelay	NOSRC-10ms	NOSRC-nodelay
   0	        0.000000250	0.000000250	0.000000188	0.000000188
   1	        0.000000438	0.000000188	0.000000438	0.000000188
   2	        0.000000437	0.000000250	0.000000375	0.000000188
   3	        0.000000625	0.000000250	0.000000438	0.000000188
   4	        0.000000313	0.000000250	0.000000500	0.000000187
   5	        0.000000625	0.000000438	0.000000375	0.000000187
   6	        0.000000500	0.000000375	0.000000375	0.000000187
   7	        0.000000500	0.000000438	0.000000437	0.000000250
   8	        0.000000688	0.000000375	0.000000437	0.000000187
   9	        0.000000875	0.000000563	0.000000437	0.000000250
   10	        0.000000750	0.000000562	0.000000375	0.000000187
   11	        0.000000750	0.000000563	0.000000375	0.000000187
   12	        0.000000750	0.000000562	0.000000312	0.000000188
   13	        0.000001000	0.000000687	0.000000375	0.000000188
   14	        0.000001125	0.000000750	0.000000375	0.000000187
   15	        0.000000875	0.000000750	0.000000375	0.000000187
   16	        0.000001000	0.000000688	0.000000438	0.000000187
   17	        0.000001063	0.000000938	0.000000375	0.000000250
   18	        0.000001000	0.000000938	0.000000375	0.000000187
   19	        0.000001125	0.000000938	0.000000312	0.000000250
   20	        0.000001000	0.000000938	0.000000375	0.000000250
   21	        0.000001312	0.000001063	0.000000438	0.000000250
   22	        0.000001250	0.000001063	0.000000375	0.000000187
   23	        0.000001250	0.000001063	0.000000375	0.000000250
   24	        0.000001187	0.000001063	0.000000438	0.000000250
   25	        0.000001438	0.000001188	0.000000375	0.000000250
   26	        0.000001438	0.000001250	0.000000375	0.000000188
   27	        0.000001563	0.000001250	0.000000375	0.000000187
   28	        0.000001687	0.000001250	0.000000312	0.000000250
   29	        0.000001688	0.000001375	0.000000312	0.000000188
   30	        0.000001625	0.000001437	0.000000312	0.000000250
   31	        0.000001687	0.000001375	0.000000438	0.000000250
   32	        0.000001813	0.000001375	0.000000375	0.000000250
   64	        0.000003125	0.000002688	0.000000563	0.000000312
   128	        0.000006000	0.000005438	0.000000625	0.000000375
   256	        0.000011812	0.000010813	0.000000688	0.000000500
   512	        0.000023438	0.000022375	0.000001063	0.000000813
   1024	        0.000044750	0.000043875	0.000001500	0.000001313
   2048	        0.000087312	0.000087125	0.000002813	0.000002500
   4096	        0.000173375	0.000174250	0.000005062	0.000004812
   8192	        0.000350250	0.000348938	0.000009625	0.000009438
   16384	0.000697875	0.000696688	0.000018875	0.000018625
   32768	0.001394563	0.001438938	0.000037250	0.000037063
   65536	0.002754500	0.002766188	0.000074187	0.000073938
   131072	0.005567312	0.005633750	0.000147813	0.000147688
   262144	0.011178000	0.011187688	0.000295313	0.000295125
   524288	0.022306375	0.022280125	0.000590250	0.000590063
   1048576	0.044503750	0.044527250	0.001180188	0.001179875
   2097152	0.089495188	0.089014125	0.002359750	0.002359563
   4194304	0.178094625	0.178543438	0.004719062	0.004719063
   8388608	0.356348063	0.355784250	0.009437937	0.009437938
   16777216	0.712111250	0.712517813	0.018875438	0.018875375
   33554432	1.424605875	1.424350938	0.037750438	0.037750438
   67108864	2.851138563	2.848466875	0.075500500	0.075500563
   134217728	5.699399438	5.698408188	0.151000688	0.151000688
   268435456	11.402305875	11.394397625	0.302000938	0.302000938
   536870912	22.794277813	22.792029750	0.604001500	0.604001500
Note that the DMA-Ranges are in the 0x5eXXXXXX range, so in L2 Cache coherent
range.

So depending on the maximum length to get achived different
policies are needed - obviously this also depends on other bus activity,
which is especially true for the copy from source case, which incurs more
of memory-bandwifth than the "S_IGNORE+D_IGNORE" case...

So this is the preferrred solution, but it comes at some cost
- we are limited to about a max delay of 1.2s

As the delay_usecs is defined as u16, this means a max delay of 65ms.
so the "option of not using memory-transfers" is preferred.

The function to get a certain delay is as follows:
F(usec)=888.86*usec-218

Note that this obviously also depends on the Memory clock speed
(which may vary with overclocking and also on how much other DMA traffic
happens.

*/
#define BCM2835_DMA_UDELAY_INTERCEPT -218
#define BCM2835_DMA_UDELAY_SLOPE     889

static inline void dump_dma_regs(
	struct device *dev,
	const char *str,
	void __iomem *base)
{
	dev_info(dev,
		"%s:%08x %08x %08x %08x %08x %08x\n",
		str,
		readl(base+BCM2835_DMA_CS),
		readl(base+BCM2835_DMA_ADDR),
		readl(base+BCM2835_DMA_TI),
		readl(base+BCM2835_DMA_S_ADDR),
		readl(base+BCM2835_DMA_D_ADDR),
		readl(base+BCM2835_DMA_LEN)
		);
}


struct dma_link * bcm2835_addDMAPoke(dma_addr_t addr,
				u32 val,
				struct dma_fragment *frag);
struct dma_link * bcm2835_addDMAPoke2(dma_addr_t addr,
				u32 val1,  u32 val2,
				struct dma_fragment *frag);
dma_addr_t* bcm2835_getDMAPokeReg(struct dma_link *link);
u32* bcm2835_getDMAPokeVal(struct dma_link *link);

struct dma_link *bcm2835_addDMADelay(u32 udelay);
int bcm2835_setDMADelay(struct dma_link *link, u32 udelay);

struct dma_link *bcm2835_addTransfer(
	dma_addr_t src, dma_addr_t dst,u32 length);
int bcm2835_setTransfer(
	dma_addr_t src, dma_addr_t dst,u32 length);




#endif /* __BCM2835_DMA_H */
