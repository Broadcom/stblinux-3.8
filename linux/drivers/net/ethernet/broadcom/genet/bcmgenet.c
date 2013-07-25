/*
 * Copyright (c) 2002-2008 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 *
 * File Name  : bcmgenet.c
 *
 * Description: This is Linux driver for the broadcom GENET ethernet MAC core.

*/

#include "bcmgenet.h"
#include "bcmmii.h"
#include "if_net.h"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/if_ether.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/pm.h>
#include <linux/clk.h>
#include <linux/version.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>

#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/netdevice.h>
#include <linux/inetdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/ipv6.h>

#include <asm/unaligned.h>

#if CONFIG_BRCM_GENET_VERSION == 1
/* Default # of tx queues for multi queue support */
#define GENET_MQ_CNT		1
/* Default # of bds for each queue for multi queue support */
#define GENET_MQ_BD_CNT		0
#else
/* Default # of tx queues for multi queue support */
#define GENET_MQ_CNT		4
/* Default # of bds for each queue for multi queue support */
#define GENET_MQ_BD_CNT		32

#endif	/* CONFIG_BRCM_GENET_VERSION != 1 */

/* Default highest priority queue for multi queue support */
#define GENET_Q0_PRIORITY	0

#define GENET_DEFAULT_BD_CNT	\
	(TOTAL_DESC - GENET_MQ_CNT * GENET_MQ_BD_CNT)

#define RX_BUF_LENGTH		2048
#define RX_BUF_BITS			12
#define SKB_ALIGNMENT		32
#define HFB_TCP_LEN			19
#define HFB_ARP_LEN			21

/* Tx/Rx DMA register offset, skip 256 descriptors */
#define GENET_TDMA_REG_OFF	(priv->hw_params->tdma_offset + \
				TOTAL_DESC * \
				priv->hw_params->words_per_bd * sizeof(u32))
#define GENET_RDMA_REG_OFF	(priv->hw_params->rdma_offset + \
				TOTAL_DESC * \
				priv->hw_params->words_per_bd * sizeof(u32))

static inline void dmadesc_set_addr(struct bcmgenet_priv *priv,
				    volatile struct dma_desc *d,
				    dma_addr_t addr)
{
	d->address_lo = addr & 0xffffffff;
#if CONFIG_BRCM_GENET_VERSION >= 4
	if (priv->hw_params->flags & GENET_HAS_40BITS)
		d->address_hi = (u64)addr >> 32;
#endif
}

static inline dma_addr_t dmadesc_get_addr(struct bcmgenet_priv *priv,
					  const volatile struct dma_desc *d)
{
	dma_addr_t addr;

	addr = d->address_lo;
#if CONFIG_BRCM_GENET_VERSION >= 4
	if (priv->hw_params->flags & GENET_HAS_40BITS)
		addr |= (u64)d->address_hi << 32;
#endif
	return addr;
}

#define GENET_VER_FMT	"%1d.%1d EPHY: 0x%04x"

#define GENET_MSG_DEFAULT	(NETIF_MSG_DRV | NETIF_MSG_PROBE | \
				NETIF_MSG_LINK)

static int debug = -1;
module_param(debug, int, 0);
MODULE_PARM_DESC(debug, "GENET debug level");

enum bcmgenet_version __genet_get_version(struct bcmgenet_priv *priv)
{
	return priv->version;
}

/* interrupt l2 registers accessors */
static inline u32 bcmgenet_intrl2_0_readl(struct bcmgenet_priv *priv, u32 off)
{
	return readl_relaxed(priv->base + GENET_INTRL2_0_OFF + off);
}

static inline void bcmgenet_intrl2_0_writel(struct bcmgenet_priv *priv,
						u32 val, u32 off)
{
	writel_relaxed(val, priv->base + GENET_INTRL2_0_OFF + off);
}

static inline u32 bcmgenet_intrl2_1_readl(struct bcmgenet_priv *priv, u32 off)
{
	return readl_relaxed(priv->base + GENET_INTRL2_1_OFF + off);
}

static inline void bcmgenet_intrl2_1_writel(struct bcmgenet_priv *priv,
						u32 val, u32 off)
{
	writel_relaxed(val, priv->base + GENET_INTRL2_1_OFF + off);
}

/* HFB register accessors  */
static inline u32 bcmgenet_hfb_readl(struct bcmgenet_priv *priv, u32 off)
{
	return readl_relaxed(priv->base + priv->hw_params->hfb_offset + off);
}

static inline void bcmgenet_hfb_writel(struct bcmgenet_priv *priv,
					u32 val, u32 off)
{
	writel_relaxed(val, priv->base + priv->hw_params->hfb_offset + off);
}

/* RBUF register accessors */
static inline u32 bcmgenet_rbuf_readl(struct bcmgenet_priv *priv, u32 off)
{
	return readl_relaxed(priv->base + GENET_RBUF_OFF + off);
}

static inline void bcmgenet_rbuf_writel(struct bcmgenet_priv *priv,
					u32 val, u32 off)
{
	writel_relaxed(val, priv->base + GENET_RBUF_OFF + off);
}

static inline u32 bcmgenet_rbuf_ctrl_get(struct bcmgenet_priv *priv)
{
	if (GENET_IS_V1(priv))
		return bcmgenet_rbuf_readl(priv, RBUF_FLUSH_CTRL_V1);
	else
		return bcmgenet_sys_readl(priv, SYS_RBUF_FLUSH_CTRL);
}

static inline void bcmgenet_rbuf_ctrl_set(struct bcmgenet_priv *priv, u32 val)
{
	if (GENET_IS_V1(priv))
		bcmgenet_rbuf_writel(priv, val, RBUF_FLUSH_CTRL_V1);
	else
		bcmgenet_sys_writel(priv, val, SYS_RBUF_FLUSH_CTRL);
}

/* These macros are defined to deal with register map change
 * between GENET1.1 and GENET2. Only those currently being used
 * by driver are defined.
 */
static inline u32 bcmgenet_tbuf_ctrl_get(struct bcmgenet_priv *priv)
{
	if (GENET_IS_V1(priv))
		return bcmgenet_rbuf_readl(priv, TBUF_CTRL_V1);
	else
		return readl_relaxed(priv->base +
				priv->hw_params->tbuf_offset + TBUF_CTRL);
}

static inline void bcmgenet_tbuf_ctrl_set(struct bcmgenet_priv *priv, u32 val)
{
	if (GENET_IS_V1(priv))
		bcmgenet_rbuf_writel(priv, val, TBUF_CTRL_V1);
	else
		writel_relaxed(val, priv->base +
				priv->hw_params->tbuf_offset + TBUF_CTRL);
}

static inline u32 bcmgenet_bp_mc_get(struct bcmgenet_priv *priv)
{
	if (GENET_IS_V1(priv))
		return bcmgenet_rbuf_readl(priv, TBUF_BP_MC_V1);
	else
		return readl_relaxed(priv->base +
				priv->hw_params->tbuf_offset + TBUF_BP_MC);
}

static inline void bcmgenet_bp_mc_set(struct bcmgenet_priv *priv, u32 val)
{
	if (GENET_IS_V1(priv))
		bcmgenet_rbuf_writel(priv, val, TBUF_BP_MC_V1);
	else
		writel_relaxed(val, priv->base +
				priv->hw_params->tbuf_offset + TBUF_BP_MC);
}

/* RX/TX DMA register accessors */
enum dma_reg {
	DMA_RING_CFG = 0,
	DMA_CTRL,
	DMA_STATUS,
	DMA_SCB_BURST_SIZE,
	DMA_ARB_CTRL,
	DMA_PRIORITY,
	DMA_RING_PRIORITY,
};

static const u8 bcmgenet_dma_regs_v3plus[] = {
	[DMA_RING_CFG]		= 0x00,
	[DMA_CTRL]		= 0x04,
	[DMA_STATUS]		= 0x08,
	[DMA_SCB_BURST_SIZE]	= 0x0C,
	[DMA_ARB_CTRL]		= 0x2C,
	[DMA_PRIORITY]		= 0x30,
	[DMA_RING_PRIORITY]	= 0x38,
};

static const u8 bcmgenet_dma_regs_v2[] = {
	[DMA_RING_CFG]		= 0x00,
	[DMA_CTRL]		= 0x04,
	[DMA_STATUS]		= 0x08,
	[DMA_SCB_BURST_SIZE]	= 0x0C,
	[DMA_ARB_CTRL]		= 0x30,
	[DMA_PRIORITY]		= 0x34,
	[DMA_RING_PRIORITY]	= 0x3C,
};

static const u8 bcmgenet_dma_regs_v1[] = {
	[DMA_CTRL]		= 0x00,
	[DMA_STATUS]		= 0x04,
	[DMA_SCB_BURST_SIZE]	= 0x0C,
	[DMA_ARB_CTRL]		= 0x30,
	[DMA_PRIORITY]		= 0x34,
	[DMA_RING_PRIORITY]	= 0x3C,
};

/* Set at runtime once bcmgenet version is known */
static const u8 *bcmgenet_dma_regs;

static inline u32 bcmgenet_tdma_readl(struct bcmgenet_priv *priv,
					enum dma_reg r)
{
	return readl_relaxed(priv->base + GENET_TDMA_REG_OFF +
			DMA_RINGS_SIZE + bcmgenet_dma_regs[r]);
}

static inline void bcmgenet_tdma_writel(struct bcmgenet_priv *priv,
					u32 val, enum dma_reg r)
{
	writel_relaxed(val, priv->base + GENET_TDMA_REG_OFF +
			DMA_RINGS_SIZE + bcmgenet_dma_regs[r]);
}

static inline u32 bcmgenet_rdma_readl(struct bcmgenet_priv *priv,
					enum dma_reg r)
{
	return readl_relaxed(priv->base + GENET_RDMA_REG_OFF +
			DMA_RINGS_SIZE + bcmgenet_dma_regs[r]);
}

static inline void bcmgenet_rdma_writel(struct bcmgenet_priv *priv,
					u32 val, enum dma_reg r)
{
	writel_relaxed(val, priv->base + GENET_RDMA_REG_OFF +
			DMA_RINGS_SIZE + bcmgenet_dma_regs[r]);
}

/* RDMA/TDMA ring registers and accessors
 * we merge the common fields and just prefix with T/D the registers
 * having different meaning depending on the direction
 */
enum dma_ring_reg {
	TDMA_READ_PTR = 0,
	RDMA_WRITE_PTR = TDMA_READ_PTR,
	TDMA_READ_PTR_HI,
	RDMA_WRITE_PTR_HI = TDMA_READ_PTR_HI,
	TDMA_CONS_INDEX,
	RDMA_PROD_INDEX = TDMA_CONS_INDEX,
	TDMA_PROD_INDEX,
	RDMA_CONS_INDEX = TDMA_PROD_INDEX,
	DMA_RING_BUF_SIZE,
	DMA_START_ADDR,
	DMA_START_ADDR_HI,
	DMA_END_ADDR,
	DMA_END_ADDR_HI,
	DMA_MBUF_DONE_THRESH,
	TDMA_FLOW_PERIOD,
	RDMA_XON_XOFF_THRESH = TDMA_FLOW_PERIOD,
	TDMA_WRITE_PTR,
	RDMA_READ_PTR = TDMA_WRITE_PTR,
	TDMA_WRITE_PTR_HI,
	RDMA_READ_PTR_HI = TDMA_WRITE_PTR_HI
};

/* GENET v4 supports 40-bits pointer addressing
 * for obvious reasons the LO and HI word parts
 * are contiguous, but this offsets the other
 * registers.
 */
static const u8 genet_dma_ring_regs_v4[] = {
	[TDMA_READ_PTR]			= 0x00,
	[TDMA_READ_PTR_HI]		= 0x04,
	[TDMA_CONS_INDEX]		= 0x08,
	[TDMA_PROD_INDEX]		= 0x0C,
	[DMA_RING_BUF_SIZE]		= 0x10,
	[DMA_START_ADDR]		= 0x14,
	[DMA_START_ADDR_HI]		= 0x18,
	[DMA_END_ADDR]			= 0x1C,
	[DMA_END_ADDR_HI]		= 0x20,
	[DMA_MBUF_DONE_THRESH]		= 0x24,
	[TDMA_FLOW_PERIOD]		= 0x28,
	[TDMA_WRITE_PTR]		= 0x2C,
	[TDMA_WRITE_PTR_HI]		= 0x30,
};

static const u8 genet_dma_ring_regs_v123[] = {
	[TDMA_READ_PTR]			= 0x00,
	[TDMA_CONS_INDEX]		= 0x04,
	[TDMA_PROD_INDEX]		= 0x08,
	[DMA_RING_BUF_SIZE]		= 0x0C,
	[DMA_START_ADDR]		= 0x10,
	[DMA_END_ADDR]			= 0x14,
	[DMA_MBUF_DONE_THRESH]		= 0x18,
	[TDMA_FLOW_PERIOD]		= 0x1C,
	[TDMA_WRITE_PTR]		= 0x20,
};

/* Set at runtime once GENET version is known */
static const u8 *genet_dma_ring_regs;

static inline u32 bcmgenet_tdma_ring_readl(struct bcmgenet_priv *priv,
						unsigned int ring,
						enum dma_ring_reg r)
{
	return readl_relaxed(priv->base + GENET_TDMA_REG_OFF +
			(DMA_RING_SIZE * ring) +
			genet_dma_ring_regs[r]);
}

static inline void bcmgenet_tdma_ring_writel(struct bcmgenet_priv *priv,
						unsigned int ring,
						u32 val,
						enum dma_ring_reg r)
{
	writel_relaxed(val, priv->base + GENET_TDMA_REG_OFF +
			(DMA_RING_SIZE * ring) +
			genet_dma_ring_regs[r]);
}

static inline u32 bcmgenet_rdma_ring_readl(struct bcmgenet_priv *priv,
						unsigned int ring,
						enum dma_ring_reg r)
{
	return readl_relaxed(priv->base + GENET_RDMA_REG_OFF +
			(DMA_RING_SIZE * ring) +
			genet_dma_ring_regs[r]);
}

static inline void bcmgenet_rdma_ring_writel(struct bcmgenet_priv *priv,
						unsigned int ring,
						u32 val,
						enum dma_ring_reg r)
{
	writel_relaxed(val, priv->base + GENET_RDMA_REG_OFF +
			(DMA_RING_SIZE * ring) +
			genet_dma_ring_regs[r]);
}

/* GENET v2+ HFB control and filter len helpers */
static inline u32 bcmgenet_hfb_reg_readl(struct bcmgenet_priv *priv, u32 off)
{
	return readl_relaxed(priv->base + priv->hw_params->hfb_reg_offset + off);
}

static inline void bcmgenet_hfb_reg_writel(struct bcmgenet_priv *priv,
					   u32 val, u32 off)
{
	writel_relaxed(val, priv->base + priv->hw_params->hfb_reg_offset + off);
}

/* GENET v2, v3 and v4 all share the same way of setting/getting the control
 * register
 */
static inline u32 bcmgenet_hfb_ctrl_readl(struct bcmgenet_priv *priv)
{
	if (GENET_IS_V1(priv))
		return bcmgenet_rbuf_readl(priv, RBUF_HFB_CTRL_V1);
	else
		return bcmgenet_hfb_reg_readl(priv, HFB_CTRL);
}

static inline void bcmgenet_hfb_ctrl_writel(struct bcmgenet_priv *priv,
						u32 val)
{
	if (GENET_IS_V1(priv))
		bcmgenet_rbuf_writel(priv, val, RBUF_HFB_CTRL_V1);
	else
		bcmgenet_hfb_reg_writel(priv, val, HFB_CTRL);
}

static inline u32 bcmgenet_hfb_flt_len_readl(struct bcmgenet_priv *priv,
						u8 filter)
{
	if (GENET_IS_V2(priv))
		return bcmgenet_hfb_reg_readl(priv, HFB_FLT_LEN_V2 + filter);
	else if (GENET_IS_V1(priv))
		return bcmgenet_rbuf_readl(priv, RBUF_HFB_CTRL_V1 + filter);
	else
		return bcmgenet_hfb_reg_readl(priv,
			HFB_FLT_LEN_V3PLUS + filter);
}

static inline void bcmgenet_hfb_flt_len_writel(struct bcmgenet_priv *priv,
						u8 filter, u32 val)
{
	if (GENET_IS_V2(priv))
		bcmgenet_hfb_reg_writel(priv, val, HFB_FLT_LEN_V2 + filter);
	else if (GENET_IS_V1(priv))
		bcmgenet_rbuf_writel(priv, val, RBUF_HFB_CTRL_V1 + filter);
	else
		bcmgenet_hfb_reg_writel(priv, val, HFB_FLT_LEN_V3PLUS + filter);
}

static inline u32 bcmgenet_hfb_flt_en_readl(struct bcmgenet_priv *priv,
						u8 filter)
{
	return bcmgenet_hfb_reg_readl(priv, HFB_FLT_ENABLE_V3PLUS + filter * 4);
}

static inline void bcmgenet_hfb_flt_en_writel(struct bcmgenet_priv *priv,
						u8 filter, u32 val)
{
	bcmgenet_hfb_reg_writel(priv, val, HFB_FLT_ENABLE_V3PLUS + filter * 4);
}

/* HFB data for ARP request.
 * In WoL (Magic Packet or ACPI) mode, we need to response
 * ARP request, so dedicate an HFB to filter the ARP request.
 * NOTE: the last two words are to be filled by destination.
 */
static unsigned int hfb_arp[] = {
	0x000FFFFF, 0x000FFFFF, 0x000FFFFF,	0x00000000,
	0x00000000, 0x00000000, 0x000F0806,	0x000F0001,
	0x000F0800,	0x000F0604, 0x000F0001,	0x00000000,
	0x00000000,	0x00000000,	0x00000000, 0x00000000,
	0x000F0000,	0x000F0000,	0x000F0000,	0x000F0000,
	0x000F0000
};

static inline void handle_alignment(struct bcmgenet_priv *priv,
		struct sk_buff *skb)
{
	/* We request to allocate 2048 + 32 bytes of buffers, and the
	 * dev_alloc_skb() added 16B for NET_SKB_PAD, so we totally
	 * requested 2048+32+16 bytes buffer, the size was aligned to
	 * SMP_CACHE_BYTES, which is 64B.(is it?), so we finnally ended
	 * up got 2112 bytes of buffer! Among which, the first 16B is
	 * reserved for NET_SKB_PAD, to make the skb->data aligned 32B
	 * boundary, we should have enough space to fullfill the 2KB
	 * buffer after alignment!
	 */

	unsigned long boundary32, cur_data, res_header;

	cur_data = (unsigned long) skb->data;
	boundary32 = (cur_data + (SKB_ALIGNMENT - 1)) & ~(SKB_ALIGNMENT - 1);
	res_header = boundary32 - cur_data ;
	/* 4 bytes for skb pointer */
	if (res_header < 4)
		boundary32 += 32;

	res_header = boundary32 - cur_data - 4;
	/* We'd have minimum 16B reserved by default. */
	skb_reserve(skb, res_header);

	*(unsigned int *)skb->data = (unsigned int)skb;
	skb_reserve(skb, 4);
	/* Make sure it is on 32B boundary, should never happen if our
	 * calculation was correct.
	 */
	if ((unsigned long) skb->data & (SKB_ALIGNMENT - 1)) {
		pr_warn("skb buffer is NOT aligned on %d boundary!\n",
			SKB_ALIGNMENT);
	}

	/* we don't reserve 2B for IP Header optimization here,
	 * use skb_pull when receiving packets
	 */
}

static void bcmgenet_gphy_link_status(struct work_struct *work)
{
	struct bcmgenet_priv *priv = container_of(work,
			struct bcmgenet_priv, bcmgenet_link_work);

	bcmgenet_mii_setup(priv->dev);
}

static void bcmgenet_gphy_link_timer(unsigned long data)
{
	struct bcmgenet_priv *priv = (struct bcmgenet_priv *)data;
	schedule_work(&priv->bcmgenet_link_work);
	mod_timer(&priv->timer, jiffies + HZ);
}

#ifdef CONFIG_BRCM_HAS_STANDBY
static int bcmgenet_wakeup_enable(void *ref)
{
	struct bcmgenet_priv *priv = (struct bcmgenet_priv *)ref;
	u32 mask;
	if (priv->phy_type == BRCM_PHY_TYPE_MOCA)
		mask = WOL_MOCA_MASK;
	else
		mask = priv->pdev->id ? WOL_MOCA_MASK : WOL_ENET_MASK;
	if (device_may_wakeup(&priv->dev->dev))
		brcm_pm_wakeup_source_enable(mask, 1);
	return 0;
}

static int bcmgenet_wakeup_disable(void *ref)
{
	struct bcmgenet_priv *priv = (struct bcmgenet_priv *)ref;
	u32 mask;
	if (priv->phy_type == BRCM_PHY_TYPE_MOCA)
		mask = WOL_MOCA_MASK;
	else
		mask = priv->pdev->id ? WOL_MOCA_MASK : WOL_ENET_MASK;
	if (device_may_wakeup(&priv->dev->dev))
		brcm_pm_wakeup_source_enable(mask, 0);
	return 0;
}

static int bcmgenet_wakeup_poll(void *ref)
{
	struct bcmgenet_priv *priv = (struct bcmgenet_priv *)ref;
	int retval = 0;
	u32 mask = 0;

	if (device_may_wakeup(&priv->dev->dev)) {
		if (priv->phy_type == BRCM_PHY_TYPE_MOCA)
			mask = WOL_MOCA_MASK;
		else
			mask = priv->pdev->id ? WOL_MOCA_MASK : WOL_ENET_MASK;
		retval = brcm_pm_wakeup_get_status(mask);
	}
	pr_debug("%s %s(%08x): %d\n", __func__,
	       priv->dev->name, mask, retval);
	return retval;
}

static struct brcm_wakeup_ops bcmgenet_wakeup_ops = {
	.enable = bcmgenet_wakeup_enable,
	.disable = bcmgenet_wakeup_disable,
	.poll = bcmgenet_wakeup_poll,
};
#endif



#define FILTER_POS(i)	(((priv->hw_params->hfb_filter_cnt-i-1)>>2))

static void bcmgenet_hfb_flt_set_len(struct bcmgenet_priv *priv,
						u8 filter, u32 len)
{
	u32 tmp;

	tmp = bcmgenet_hfb_flt_len_readl(priv, FILTER_POS(filter));
	tmp &= ~(RBUF_FLTR_LEN_MASK << (RBUF_FLTR_LEN_SHIFT * (filter & 0x03)));
	tmp |= (len << (RBUF_FLTR_LEN_SHIFT * (filter & 0x03)));
	bcmgenet_hfb_flt_len_writel(priv, FILTER_POS(filter), tmp);
}

static u32 bcmgenet_hfb_get_flt_len(struct bcmgenet_priv *priv, u8 filter)
{
	u32 tmp;
	tmp = bcmgenet_hfb_flt_len_readl(priv, FILTER_POS(filter));
	return (tmp >> (RBUF_FLTR_LEN_SHIFT * (filter & 0x03)))
		& RBUF_FLTR_LEN_MASK;
}

static inline u32 bcmgenet_hfb_get_filter_en(struct bcmgenet_priv *priv,
						u8 filter)
{
	u32 tmp = 0;

	if (GENET_IS_V1(priv) || GENET_IS_V2(priv)) {
		tmp = bcmgenet_hfb_ctrl_readl(priv);
		return (tmp >> (filter + RBUF_HFB_FILTER_EN_SHIFT)) & 0x01;
	} else {
		return bcmgenet_hfb_flt_en_readl(priv, filter < 32) &
			(1 << (filter % 32));
	}

}

static inline void bcmgenet_hfb_filter_enable(struct bcmgenet_priv *priv,
						u8 filter)
{
	u32 tmp;

	/* The order of GENET_x_HFB_FLT_ENBLE_0/1 is reversed !!! */
	if (GENET_IS_V1(priv) || GENET_IS_V2(priv)) {
		tmp = bcmgenet_hfb_ctrl_readl(priv);
		tmp |= 1 << (filter + RBUF_HFB_FILTER_EN_SHIFT);
		bcmgenet_hfb_ctrl_writel(priv, tmp);
	} else {
		tmp = bcmgenet_hfb_flt_en_readl(priv, filter < 32);
		tmp |= (1 << (filter % 32));
		bcmgenet_hfb_flt_en_writel(priv, filter, tmp);
	}

}

static inline void bcmgenet_hfb_filter_disable(struct bcmgenet_priv *priv,
						u8 filter)
{
	u32 tmp;

	if (GENET_IS_V1(priv) || GENET_IS_V2(priv)) {
		tmp = bcmgenet_hfb_ctrl_readl(priv);
		tmp &= ~(1 << (filter + RBUF_HFB_FILTER_EN_SHIFT));
		bcmgenet_hfb_ctrl_writel(priv, tmp);
	} else {
		tmp = bcmgenet_hfb_flt_en_readl(priv, filter < 32);
		tmp &= ~(1 << (filter % 32));
		bcmgenet_hfb_flt_en_writel(priv, filter, tmp);
	}
}

static inline void bcmgenet_hfb_filter_disable_all(struct bcmgenet_priv *priv)
{
	if (GENET_IS_V1(priv) || GENET_IS_V2(priv)) {
		u32 tmp;
		tmp = bcmgenet_hfb_ctrl_readl(priv);
		tmp &= ~(0xffff << (RBUF_HFB_FILTER_EN_SHIFT));
		bcmgenet_hfb_ctrl_writel(priv, tmp);
	} else {
		bcmgenet_hfb_flt_en_writel(priv, 0, 0);
		bcmgenet_hfb_flt_en_writel(priv, 4, 0);
	}
}



/* Program ACPI pattern into HFB. Return filter index if succesful.
 * if user == 1, the data will be copied from user space.
 */
int bcmgenet_update_hfb(struct net_device *dev, unsigned int *data,
		int len, int user)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);
	int filter, offset, count;
	unsigned int *tmp;

	netif_dbg(priv, wol, dev, "Updating HFB len=0x%d\n", len);
	count = priv->hw_params->hfb_filter_cnt;
	offset = 64;
	if (bcmgenet_hfb_ctrl_readl(priv) & RBUF_HFB_256B) {
		if (GENET_IS_V1(priv) || GENET_IS_V2(priv))
			count >>= 1;
		else
			offset = 128;
	}

	if (len > offset)
		return -EINVAL;

	/* find next unused filter */
	for (filter = 0; filter < count; filter++) {
		if (!bcmgenet_hfb_get_filter_en(priv, filter))
			break;
	}
	if (filter == count) {
		netif_err(priv, wol, dev, "no unused filter available!\n");
		return -EINVAL;	/* all filters have been enabled*/
	}

	if (user) {
		tmp = kmalloc(len*sizeof(unsigned int), GFP_KERNEL);
		if (tmp == NULL)
			return -EFAULT;

		/* copy pattern data */
		if (copy_from_user(tmp, data, len*sizeof(unsigned int)) != 0) {
			netif_err(priv, wol, dev,
				"Failed to copy user data: src=%p, dst=%p\n",
				data,
				priv->base + priv->hw_params->hfb_reg_offset + filter*offset);
			return -EFAULT;
		}
	} else {
		tmp = data;
	}
	/* Copy pattern data into HFB registers.*/
	for (count = 0; count < offset; count++) {
		if (count < len)
			bcmgenet_hfb_writel(priv, *(tmp + count),
				filter * offset + count);
		else
			bcmgenet_hfb_writel(priv, 0,
				filter * offset + count);
	}
	if (user)
		kfree(tmp);

	/* set the filter length*/
	bcmgenet_hfb_flt_set_len(priv, filter, len*2);

	/*enable this filter.*/
	bcmgenet_hfb_filter_enable(priv, filter);

	return filter;

}
EXPORT_SYMBOL(bcmgenet_update_hfb);

/* read ACPI pattern data for a particular filter. */
static int bcmgenet_read_hfb(struct net_device *dev, struct acpi_data *u_data)
{
	int filter, offset, count, len;
	struct bcmgenet_priv *priv = netdev_priv(dev);
	u32 *hfb;
	unsigned int i;

	if (get_user(filter, &(u_data->fltr_index))) {
		netif_err(priv, wol, dev, "Failed to get user data\n");
		return -EFAULT;
	}

	count = priv->hw_params->hfb_filter_cnt;
	offset = 128;
	if (bcmgenet_hfb_ctrl_readl(priv) & RBUF_HFB_256B) {
		if (GENET_IS_V1(priv) || GENET_IS_V2(priv))
			count >>= 1;
		else
			offset = 256;
	}

	if (filter > count)
		return -EINVAL;

	/* see if this filter is enabled, if not, return length 0 */
	if (!bcmgenet_hfb_get_filter_en(priv, filter)) {
		len = 0;
		put_user(len , &u_data->count);
		return 0;
	}
	/* check the filter length, in bytes */
	len = bcmgenet_hfb_get_flt_len(priv, filter);
	if (u_data->count < len)
		return -EINVAL;
	/* copy pattern data */

	hfb = kmalloc(len, GFP_KERNEL);
	if (!hfb)
		return -ENOMEM;

	for (i = 0; i < len; i += 4)
		hfb[i] = bcmgenet_hfb_readl(priv, filter*offset + i);

	if (copy_to_user((void *)(u_data->p_data), (void *)hfb, len)) {
		netif_err(priv, wol, dev,
			"Failed to copy data to user space: src=%p, dst=%p\n",
			priv->base + priv->hw_params->hfb_reg_offset +
			filter*offset,
			u_data->p_data);
		kfree(hfb);
		return -EFAULT;
	}
	kfree(hfb);
	return len;
}

/* clear the HFB, disable filter indexed by "filter" argument. */
static inline void bcmgenet_clear_hfb(struct bcmgenet_priv *priv,
		int filter)
{
	u32 reg;

	if (filter == CLEAR_ALL_HFB) {
		bcmgenet_hfb_filter_disable_all(priv);
		reg = bcmgenet_hfb_ctrl_readl(priv);
		reg &= ~RBUF_HFB_EN;
		bcmgenet_hfb_ctrl_writel(priv, reg);
	} else {
		/* disable this filter */
		bcmgenet_hfb_filter_disable(priv, filter);
		/* clear filter length register */
		bcmgenet_hfb_flt_set_len(priv, filter, 0);
	}

}

/* Utility function to get interface ip address in kernel space. */
static inline unsigned int bcmgenet_getip(struct net_device *dev)
{
	struct net_device *pnet_device;
	unsigned int ip = 0;

	read_lock(&dev_base_lock);
	/* read all devices */
	for_each_netdev(&init_net, pnet_device)
	{
		if ((netif_running(pnet_device)) &&
				(pnet_device->ip_ptr != NULL) &&
				(!strcmp(pnet_device->name, dev->name))) {
			struct in_device *pin_dev;
			pin_dev = (struct in_device *)(pnet_device->ip_ptr);
			if (pin_dev && pin_dev->ifa_list)
				ip = htonl(pin_dev->ifa_list->ifa_address);
			break;
		}
	}
	read_unlock(&dev_base_lock);
	return ip;
}

static void bcmgenet_clock_enable(struct bcmgenet_priv *priv)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	/* FIXME */
	/* clk_enable(priv->clk); */
	priv->clock_active = 1;
	spin_unlock_irqrestore(&priv->lock, flags);
}

static void bcmgenet_clock_disable(struct bcmgenet_priv *priv)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	priv->clock_active = 0;
	/* FIXME */
	/* clk_disable(priv->clk); */
	spin_unlock_irqrestore(&priv->lock, flags);
}

/* ethtool function - get WOL (Wake on LAN) settings,
 * Only Magic Packet Detection is supported through ethtool,
 * the ACPI (Pattern Matching) WOL option is supported in
 * bcmumac_do_ioctl function.
 */
static void bcmgenet_get_wol(struct net_device *dev,
		struct ethtool_wolinfo *wol)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);
	u32 reg;
	wol->supported = WAKE_MAGIC | WAKE_MAGICSECURE | WAKE_ARP;

	if (!netif_running(dev))
		return;

	wol->wolopts = priv->wolopts;
	if (wol->wolopts & WAKE_MAGICSECURE) {
		reg = bcmgenet_umac_readl(priv, UMAC_MPD_PW_MS);
		wol->sopass[1] = (reg >> 8) & 0xff;
		wol->sopass[0] = reg & 0xff;
		reg = bcmgenet_umac_readl(priv, UMAC_MPD_PW_LS);
		wol->sopass[5] = (reg >> 24) & 0xff;
		wol->sopass[4] = (reg >> 16) & 0xff;
		wol->sopass[3] = (reg >> 8) & 0xff;
		wol->sopass[2] = reg & 0xff;
	} else {
		memset(&wol->sopass[0], 0, sizeof(wol->sopass));
	}
}

/* ethtool function - set WOL (Wake on LAN) settings.
 * Only for magic packet detection mode.
 */
static int bcmgenet_set_wol(struct net_device *dev,
		struct ethtool_wolinfo *wol)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);
	u32 reg;

	if (wol->wolopts & ~(WAKE_MAGIC | WAKE_MAGICSECURE | WAKE_ARP))
		return -EINVAL;

	if (wol->wolopts & WAKE_MAGICSECURE) {
		bcmgenet_umac_writel(priv,
			wol->sopass[1] << 8 | wol->sopass[0], UMAC_MPD_PW_MS);
		bcmgenet_umac_writel(priv,
			wol->sopass[5] << 24 | wol->sopass[4] << 16 |
			wol->sopass[3] << 8 | wol->sopass[2],
			UMAC_MPD_PW_LS);
		reg = bcmgenet_umac_readl(priv, UMAC_MPD_CTRL);
		reg |= MPD_PW_EN;
		bcmgenet_umac_writel(priv, reg, UMAC_MPD_CTRL);
	}

	device_set_wakeup_enable(&dev->dev, wol->wolopts);
	priv->wolopts = wol->wolopts;
	return 0;
}

static int bcmgenet_get_settings(struct net_device *dev,
		struct ethtool_cmd *cmd)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);
	int rc = 0;

	if (priv->phy_type == BRCM_PHY_TYPE_MOCA) {
		/* see comments in bcmgenet_set_settings() */
		cmd->autoneg = netif_carrier_ok(priv->dev);
		cmd->speed = SPEED_1000;
		cmd->duplex = DUPLEX_HALF;
		cmd->port = PORT_BNC;
	} else {
		if (!netif_running(dev))
			return -EINVAL;
		rc = mii_ethtool_gset(&priv->mii, cmd);
	}

	return rc;
}

static int bcmgenet_set_settings(struct net_device *dev,
		struct ethtool_cmd *cmd)
{
	int err = 0;
	struct bcmgenet_priv *priv = netdev_priv(dev);

	if (priv->phy_type == BRCM_PHY_TYPE_MOCA) {
		/* mocad uses cmd->autoneg to control our RUNNING flag */
		if (cmd->autoneg)
			netif_carrier_on(priv->dev);
		else
			netif_carrier_off(priv->dev);
	} else {
		if (!netif_running(dev))
			return -EINVAL;

		err = mii_ethtool_sset(&priv->mii, cmd);
		if (err < 0)
			return err;
		bcmgenet_mii_setup(dev);
	}

	return err;
}

static void bcmgenet_get_drvinfo(struct net_device *dev,
		struct ethtool_drvinfo *info)
{
	strncpy(info->driver, CARDNAME, sizeof(info->driver));
	strncpy(info->version, VER_STR, sizeof(info->version));

}
static int bcmgenet_set_rx_csum(struct net_device *dev,
				netdev_features_t changed)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);
	u32 rbuf_ctrl;
	u32 rbuf_chk_ctrl;
	int rx_csum_en;
	int desc_64b_en;

	/* Either we request RX checksum or TX checksum is already enabled */
	rx_csum_en = changed & NETIF_F_RXCSUM;
	desc_64b_en = rx_csum_en || (dev->features & (NETIF_F_IP_CSUM |
						   NETIF_F_IPV6_CSUM));

	spin_lock_bh(&priv->bh_lock);
	rbuf_ctrl = bcmgenet_rbuf_readl(priv, RBUF_CTRL);
	rbuf_chk_ctrl = bcmgenet_rbuf_readl(priv, RBUF_CHK_CTRL);

	/* enable rx checksumming */
	if (!rx_csum_en)
		rbuf_chk_ctrl &= ~RBUF_RXCHK_EN;
	else
		rbuf_chk_ctrl |= RBUF_RXCHK_EN;

	/* no rx checksumming and no 64bytes enabled descriptor */
	if (!desc_64b_en)
		rbuf_ctrl &= ~RBUF_64B_EN;
	else
		rbuf_ctrl |= RBUF_64B_EN;

	priv->desc_64b_en = desc_64b_en;
	priv->desc_rxchk_en = rx_csum_en;
	bcmgenet_rbuf_writel(priv, rbuf_ctrl, RBUF_CTRL);
	bcmgenet_rbuf_writel(priv, rbuf_chk_ctrl, RBUF_CHK_CTRL);

	spin_unlock_bh(&priv->bh_lock);

	return 0;
}
static int bcmgenet_set_tx_csum(struct net_device *dev,
				netdev_features_t changed)
{
	unsigned long flags;
	struct bcmgenet_priv *priv = netdev_priv(dev);
	u32 reg;
	int desc_64b_en;

	spin_lock_irqsave(&priv->lock, flags);
	reg = bcmgenet_tbuf_ctrl_get(priv);

	/* Either we requested TX checksum or RX checksum is already
	 * enabled
	 */
	desc_64b_en = changed & (NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM) ||
			(dev->features & NETIF_F_RXCSUM);

	/* enable 64bytes descriptor */
	if (!desc_64b_en) {
		reg &= ~RBUF_64B_EN;
		if (dev->needed_headroom > 64)
			dev->needed_headroom -= 64;
	} else {
		reg |= RBUF_64B_EN;
		if (dev->needed_headroom < 64)
			dev->needed_headroom += 64;
	}
	priv->desc_64b_en = desc_64b_en;

	bcmgenet_tbuf_ctrl_set(priv, reg);
	spin_unlock_irqrestore(&priv->lock, flags);
	return 0;
}
static int bcmgenet_set_sg(struct net_device *dev, netdev_features_t changed)
{
	if ((changed & NETIF_F_SG) &&
		!(dev->features & NETIF_F_IP_CSUM)) {
		netdev_warn(dev,
			"Tx Checksum offloading disabled, not setting SG\n");
		return -EINVAL;
	}
	/* must have 64B tx status enabled */
	return 0;
}
static int bcmgenet_set_features(struct net_device *dev,
		netdev_features_t features)
{
	int ret = 0;
	netdev_features_t changed = features ^ dev->features;

	if (changed & (NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM))
		ret = bcmgenet_set_tx_csum(dev, changed);
	if (changed & (NETIF_F_RXCSUM))
		ret = bcmgenet_set_rx_csum(dev, changed);
	if (changed & NETIF_F_SG)
		ret = bcmgenet_set_sg(dev, changed);

	return ret;
}

static u32 bcmgenet_get_msglevel(struct net_device *dev)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);

	return priv->msg_enable;
}

static void bcmgenet_set_msglevel(struct net_device *dev, u32 level)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);

	priv->msg_enable = level;
}

/* standard ethtool support functions. */
static struct ethtool_ops bcmgenet_ethtool_ops = {
	.get_settings		= bcmgenet_get_settings,
	.set_settings		= bcmgenet_set_settings,
	.get_drvinfo		= bcmgenet_get_drvinfo,
	.get_wol		= bcmgenet_get_wol,
	.set_wol		= bcmgenet_set_wol,
	.get_link		= ethtool_op_get_link,
	.get_msglevel		= bcmgenet_get_msglevel,
	.set_msglevel		= bcmgenet_set_msglevel,
};

static int bcmgenet_enable_arp_filter(struct bcmgenet_priv *priv)
{
	struct net_device *dev = priv->dev;
	unsigned int ip;
	u32 reg;

	ip = bcmgenet_getip(dev);
	if (ip) {
		/* clear the lower halfwords */
		hfb_arp[HFB_ARP_LEN-2] &= ~0xffff;
		hfb_arp[HFB_ARP_LEN-1] &= ~0xffff;
		hfb_arp[HFB_ARP_LEN-2] |= (ip >> 16);
		hfb_arp[HFB_ARP_LEN-1] |= (ip & 0xFFFF);
		/* Enable HFB, to response to ARP request.*/
		if (bcmgenet_update_hfb(dev, hfb_arp, HFB_ARP_LEN, 0) < 0) {
			netif_err(priv, wol, dev, "%s: Unable to update HFB\n",
			       __func__);
			return -1;
		}
		reg = bcmgenet_hfb_ctrl_readl(priv);
		reg |= RBUF_HFB_EN;
		bcmgenet_hfb_ctrl_writel(priv, reg);
		return 0;
	}

	return -1;
}

static int bcmgenet_poll_wol_status(struct bcmgenet_priv *priv)
{
	struct net_device *dev = priv->dev;
	int retries = 0;

	while (!(bcmgenet_rbuf_readl(priv, RBUF_STATUS)
		& RBUF_STATUS_WOL)) {
		retries++;
		if (retries > 5) {
			netdev_crit(dev, "polling wol mode timeout\n");
			return -ETIMEDOUT;
		}
		mdelay(1);
	}

	return retries;
}

static int bcmgenet_wol_power_down_cfg(struct bcmgenet_priv *priv,
					int mode)
{
	int ret = 0;
	struct net_device *dev = priv->dev;
	int retries = 0;
	u32 cpu_mask_clear;
	u32 reg;

	if (mode == GENET_POWER_WOL_ACPI)
		cpu_mask_clear = (UMAC_IRQ_HFB_MM | UMAC_IRQ_HFB_SM);
	else if (mode == GENET_POWER_WOL_MAGIC)
		cpu_mask_clear = UMAC_IRQ_MPD_R;
	else {
		netif_err(priv, wol, dev, "unsupported mode: %d\n", mode);
		return -EINVAL;
	}

	if (mode == GENET_POWER_WOL_ACPI) {
		ret = bcmgenet_enable_arp_filter(priv);
		if (ret) {
			netdev_crit(dev, "failed to set HFB filter\n");
			return ret;
		}
	}

	/* disable RX */
	reg = bcmgenet_umac_readl(priv, UMAC_CMD);
	reg &= ~CMD_RX_EN;
	bcmgenet_umac_writel(priv, reg, UMAC_CMD);
	mdelay(10);

	if (mode == GENET_POWER_WOL_ACPI) {
		reg = bcmgenet_hfb_ctrl_readl(priv);
		reg |= RBUF_ACPI_EN;
		bcmgenet_hfb_ctrl_writel(priv, reg);
	} else {
		reg = bcmgenet_umac_readl(priv, UMAC_MPD_CTRL);
		reg |= MPD_EN;
		bcmgenet_umac_writel(priv, reg, UMAC_MPD_CTRL);
	}

	retries = bcmgenet_poll_wol_status(priv);
	if (retries < 0) {
		if (mode == GENET_POWER_WOL_ACPI) {
			reg = bcmgenet_hfb_ctrl_readl(priv);
			reg &= ~RBUF_ACPI_EN;
			bcmgenet_hfb_ctrl_writel(priv, reg);
		} else {
			reg = bcmgenet_umac_readl(priv, UMAC_MPD_CTRL);
			reg &= ~MPD_EN;
			bcmgenet_umac_writel(priv, reg, UMAC_MPD_CTRL);
		}
		return retries;
	}

	netif_dbg(priv, wol, dev, "%s WOL-ready status set after %d msec\n",
			mode == GENET_POWER_WOL_ACPI ? "ACPI" : "MP",
			retries);

	reg = bcmgenet_umac_readl(priv, UMAC_CMD);

	/* Enable CRC forward */
	if (mode == GENET_POWER_WOL_MAGIC) {
		priv->crc_fwd_en = 1;
		reg |= CMD_CRC_FWD;
	}

	/* Receiver must be enabled for WOL MP detection */
	reg |= CMD_RX_EN;
	bcmgenet_umac_writel(priv, reg, UMAC_CMD);

	if (priv->hw_params->flags & GENET_HAS_EXT && priv->dev_asleep) {
		reg = bcmgenet_ext_readl(priv, EXT_EXT_PWR_MGMT);
		reg &= ~EXT_ENERGY_DET_MASK;
		bcmgenet_ext_writel(priv, reg, EXT_EXT_PWR_MGMT);
	}

	bcmgenet_intrl2_0_writel(priv, cpu_mask_clear, INTRL2_CPU_MASK_CLEAR);

	set_bit(mode, &priv->wol_enabled);

	return 0;
}


/* Power down the unimac, based on mode. */
static void bcmgenet_power_down(struct bcmgenet_priv *priv, int mode)
{
	struct net_device *dev;
	int ret = 0;
	u32 reg;

	dev = priv->dev;
	switch (mode) {
	case GENET_POWER_CABLE_SENSE:
#if 0
		/* EPHY bug, setting ext_pwr_down_dll and ext_pwr_down_phy cause
		 * link IRQ bouncing.
		 */
		reg = bcmgenet_ext_readl(priv, EXT_EXT_PWR_MGMT);
		reg |= (EXT_PWR_DOWN_PHY |
				EXT_PWR_DOWN_DLL | EXT_PWR_DOWN_BIAS);
		bcmgenet_ext_writel(priv, reg, EXT_EXT_PWR_MGMT);
#else
		/* Workaround for putting EPHY in iddq mode. */
		bcmgenet_ephy_workaround_iddq(dev);
#endif
		break;
	case GENET_POWER_WOL_MAGIC:
	case GENET_POWER_WOL_ACPI:
		ret = bcmgenet_wol_power_down_cfg(priv, mode);
		if (ret)
			return;

	case GENET_POWER_PASSIVE:
		/* Power down LED */
		bcmgenet_mii_reset(priv->dev);
		if (priv->hw_params->flags & GENET_HAS_EXT) {
			reg = bcmgenet_ext_readl(priv, EXT_EXT_PWR_MGMT);
			reg |= (EXT_PWR_DOWN_PHY |
				EXT_PWR_DOWN_DLL | EXT_PWR_DOWN_BIAS);
			bcmgenet_ext_writel(priv, reg, EXT_EXT_PWR_MGMT);
		}
		break;
	default:
		break;
	}

}

static void bcmgenet_wol_power_up_cfg(struct bcmgenet_priv *priv,
					int mode)
{
	u32 reg;
	u32 cpu_mask_set;

	if (mode == GENET_POWER_WOL_ACPI) {
		reg = bcmgenet_hfb_ctrl_readl(priv);
		reg &= ~RBUF_ACPI_EN;
		bcmgenet_hfb_ctrl_writel(priv, reg);
		bcmgenet_clear_hfb(priv, CLEAR_ALL_HFB);

		/* Stop monitoring ACPI interrupts */
		cpu_mask_set = (UMAC_IRQ_HFB_SM | UMAC_IRQ_HFB_MM);
	} else if (mode == GENET_POWER_WOL_MAGIC) {
		reg = bcmgenet_umac_readl(priv, UMAC_MPD_CTRL);
		reg &= ~MPD_EN;
		bcmgenet_umac_writel(priv, reg, UMAC_MPD_CTRL);

		/* Disable CRC Forward */
		reg = bcmgenet_umac_readl(priv, UMAC_CMD);
		reg &= ~CMD_CRC_FWD;
		bcmgenet_umac_writel(priv, reg, UMAC_CMD);
		priv->crc_fwd_en = 0;

		/* Stop monitoring magic packet IRQ */
		cpu_mask_set = UMAC_IRQ_MPD_R;
	} else {
		netif_err(priv, wol, priv->dev, "invalid mode: %d\n", mode);
		return;
	}

	/* Stop monitoring magic packet IRQ */
	bcmgenet_intrl2_0_writel(priv, cpu_mask_set, INTRL2_CPU_MASK_SET);

	clear_bit(mode, &priv->wol_enabled);
}

static void bcmgenet_power_up(struct bcmgenet_priv *priv, int mode)
{
	u32 reg;

	switch (mode) {
	case GENET_POWER_CABLE_SENSE:
#if 0
		priv->ext->ext_pwr_mgmt &= ~EXT_PWR_DOWN_DLL;
		priv->ext->ext_pwr_mgmt &= ~EXT_PWR_DOWN_PHY;
		priv->ext->ext_pwr_mgmt &= ~EXT_PWR_DOWN_BIAS;
#endif
		/* enable APD */
		if (priv->hw_params->flags & GENET_HAS_EXT) {
			reg = bcmgenet_ext_readl(priv, EXT_EXT_PWR_MGMT);
			reg |= EXT_PWR_DN_EN_LD;
			bcmgenet_ext_writel(priv, reg, EXT_EXT_PWR_MGMT);
			bcmgenet_mii_reset(priv->dev);
		}
		break;
	case GENET_POWER_WOL_MAGIC:
	case GENET_POWER_WOL_ACPI:
		bcmgenet_wol_power_up_cfg(priv, mode);
		break;
	case GENET_POWER_PASSIVE:
		if (priv->hw_params->flags & GENET_HAS_EXT) {
			reg = bcmgenet_ext_readl(priv, EXT_EXT_PWR_MGMT);
			reg &= ~EXT_PWR_DOWN_DLL;
			reg &= ~EXT_PWR_DOWN_PHY;
			reg &= ~EXT_PWR_DOWN_BIAS;
			/* enable APD */
			reg |= EXT_PWR_DN_EN_LD;
			bcmgenet_ext_writel(priv, reg, EXT_EXT_PWR_MGMT);
			bcmgenet_mii_reset(priv->dev);
		}
	default:
		break;
	}
	bcmgenet_ephy_workaround(priv->dev);
}

/* ioctl handle special commands that are not present in ethtool. */
static int bcmgenet_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);
	unsigned long flags;
	struct acpi_data *u_data;
	int val = 0;

	if (!netif_running(dev))
		return -EINVAL;
	/* we can add sub-command in ifr_data if we need to in the future */
	switch (cmd) {
	case SIOCSACPISET:
		spin_lock_irqsave(&priv->lock, flags);
		bcmgenet_power_down(priv, GENET_POWER_WOL_ACPI);
		spin_unlock_irqrestore(&priv->lock, flags);
		break;
	case SIOCSACPICANCEL:
		spin_lock_irqsave(&priv->lock, flags);
		bcmgenet_power_up(priv, GENET_POWER_WOL_ACPI);
		spin_unlock_irqrestore(&priv->lock, flags);
		break;
	case SIOCSPATTERN:
		u_data = (struct acpi_data *)rq->ifr_data;
		val =  bcmgenet_update_hfb(dev, (unsigned int *)u_data->p_data,
				u_data->count, 1);
		if (val >= 0)
			put_user(val, &u_data->fltr_index);
		break;
	case SIOCGPATTERN:
		u_data = (struct acpi_data *)rq->ifr_data;
		val = bcmgenet_read_hfb(dev, u_data);
		break;
	case SIOCGMIIPHY:
	case SIOCGMIIREG:
	case SIOCSMIIREG:
		val = generic_mii_ioctl(&priv->mii, if_mii(rq), cmd, NULL);
		break;
	default:
		val = -EINVAL;
		break;
	}

	return val;
}

static struct enet_cb *bcmgenet_get_txcb(struct net_device *dev,
		int *pos, int index)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);
	struct enet_cb *tx_cb_ptr = NULL;

	if (index == DESC_INDEX || !netif_is_multiqueue(dev)) {
		tx_cb_ptr = priv->tx_cbs;
		tx_cb_ptr += (*pos - GENET_MQ_CNT*GENET_MQ_BD_CNT);
		tx_cb_ptr->bd_addr = &priv->tx_bds[*pos];
		/* Advancing local write pointer */
		if (*pos == (TOTAL_DESC - 1))
			*pos = (GENET_MQ_CNT*GENET_MQ_BD_CNT);
		else
			*pos += 1;

	} else {
		tx_cb_ptr = priv->tx_ring_cbs[index];
		tx_cb_ptr += (*pos - index*GENET_MQ_BD_CNT);
		tx_cb_ptr->bd_addr = &priv->tx_bds[*pos];
		if (*pos == (GENET_MQ_BD_CNT*(index+1) - 1))
			*pos = GENET_MQ_BD_CNT * index;
		else
			*pos += 1;
	}

	return tx_cb_ptr;
}

static void bcmgenet_tx_reclaim(struct net_device *dev, int index)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);
	unsigned int c_index;
	struct enet_cb *tx_cb_ptr;
	int last_tx_cn = 0, last_c_index = 0, num_tx_bds = 0;

	/* Compute how many buffers are transmited since last xmit call */
	c_index = bcmgenet_tdma_ring_readl(priv, index, TDMA_CONS_INDEX);

	if (index == DESC_INDEX || !netif_is_multiqueue(dev)) {
		last_c_index = priv->tx_last_c_index;
		num_tx_bds = GENET_DEFAULT_BD_CNT;
	} else {
		last_c_index = priv->tx_ring_c_index[index];
		num_tx_bds = GENET_MQ_BD_CNT;
	}

	c_index &= (num_tx_bds - 1);

	if (c_index >= last_c_index)
		last_tx_cn = c_index - last_c_index;
	else
		last_tx_cn = num_tx_bds - last_c_index + c_index;


	netif_dbg(priv, tx_done, dev,
			"%s index=%d c_index=%d "
			"last_tx_cn=%d tx_last_c_index=%d\n",
			__func__, index,
			c_index, last_tx_cn, last_c_index);

	/* Reclaim transmitted buffers */
	while (last_tx_cn-- > 0) {
		if (index == DESC_INDEX)
			tx_cb_ptr = &priv->tx_cbs[last_c_index];
		else
			tx_cb_ptr = priv->tx_ring_cbs[index] + last_c_index;
		if (tx_cb_ptr->skb != NULL) {
			dma_unmap_single(&priv->dev->dev,
					dma_unmap_addr(tx_cb_ptr, dma_addr),
					tx_cb_ptr->skb->len,
					DMA_TO_DEVICE);
			dev_kfree_skb_any(tx_cb_ptr->skb);
			tx_cb_ptr->skb = NULL;
			dma_unmap_addr_set(tx_cb_ptr, dma_addr, 0);
		} else if (dma_unmap_addr(tx_cb_ptr, dma_addr)) {
			dma_unmap_page(&priv->dev->dev,
					dma_unmap_addr(tx_cb_ptr, dma_addr),
					dma_unmap_len(tx_cb_ptr, dma_len),
					DMA_TO_DEVICE);
			dma_unmap_addr_set(tx_cb_ptr, dma_addr, 0);
		}
		if (index == DESC_INDEX)
			priv->tx_free_bds += 1;
		else
			priv->tx_ring_free_bds[index] += 1;

		if (last_c_index == (num_tx_bds - 1))
			last_c_index = 0;
		else
			last_c_index++;
	}

	if (index == DESC_INDEX || !netif_is_multiqueue(dev)) {
		if (priv->tx_free_bds > (MAX_SKB_FRAGS + 1)
			&& __netif_subqueue_stopped(dev, 0)) {
			/* Disable txdma bdone/pdone interrupt if we have
			 * free tx bds
			 */
			bcmgenet_intrl2_0_writel(priv,
				UMAC_IRQ_TXDMA_BDONE | UMAC_IRQ_TXDMA_PDONE,
				INTRL2_CPU_MASK_SET);
			netif_wake_subqueue(dev, 0);
		}
		priv->tx_last_c_index = c_index;
	} else{
		if (priv->tx_ring_free_bds[index] > (MAX_SKB_FRAGS + 1)
			&& __netif_subqueue_stopped(dev, index+1)) {
			bcmgenet_intrl2_1_writel(priv, (1 << index),
					INTRL2_CPU_MASK_SET);
			priv->int1_mask |= (1 << index);
			netif_wake_subqueue(dev, index+1);
		}
		priv->tx_ring_c_index[index] = c_index;
	}
}

static void bcmgenet_tx_reclaim_all(struct net_device *dev)
{
	int i;

	if (netif_is_multiqueue(dev)) {
		for (i = 0; i < GENET_MQ_CNT; i++)
			bcmgenet_tx_reclaim(dev, i);
	}

	bcmgenet_tx_reclaim(dev, DESC_INDEX);
}

/* Helper to increment the producer index and write pointer of a givent
 * TDMA ring
 */
static inline void bcmgenet_tdma_inc_prod_write(struct bcmgenet_priv *priv,
						unsigned int tdma_ring,
						unsigned int write_ptr)
{
	u32 val;

	/* advance producer index and write pointer.*/
	val = bcmgenet_tdma_ring_readl(priv,
			tdma_ring, TDMA_PROD_INDEX);
	val += 1;
	bcmgenet_tdma_ring_writel(priv, tdma_ring,
			val, TDMA_PROD_INDEX);
	bcmgenet_tdma_ring_writel(priv, tdma_ring,
			write_ptr, TDMA_WRITE_PTR);
}

/* Transmits a single SKB (either head of a fragment or a single SKB)
 * caller must hold priv->lock
 */
static int bcmgenet_xmit_single(struct net_device *dev,
				struct sk_buff *skb,
				u16 dma_desc_flags,
				int index,
				unsigned int tdma_ring,
				unsigned int write_ptr)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);
	dma_addr_t mapping;
	int ret;
	unsigned int skb_len;
	struct enet_cb *tx_cb_ptr;
	u32 length_status;

	tx_cb_ptr = bcmgenet_get_txcb(dev, &write_ptr, index);

	if (unlikely(!tx_cb_ptr))
		BUG();

	tx_cb_ptr->skb = skb;

	skb_len = skb_headlen(skb) < ETH_ZLEN ? ETH_ZLEN : skb_headlen(skb);

	mapping = dma_map_single(&priv->dev->dev,
			skb->data, skb_len, DMA_TO_DEVICE);
	ret = dma_mapping_error(&priv->dev->dev, mapping);
	if (ret) {
		netif_err(priv, tx_err, dev, "Tx DMA map failed\n");
		dev_kfree_skb(skb);
		return ret;
	}

	dma_unmap_addr_set(tx_cb_ptr, dma_addr, mapping);
	dma_unmap_len_set(tx_cb_ptr, dma_len, skb->len);
	dmadesc_set_addr(priv, tx_cb_ptr->bd_addr, mapping);
	length_status = (skb_len << 16) | dma_desc_flags |
			(priv->hw_params->qtag_mask << DMA_TX_QTAG_SHIFT) |
			DMA_TX_APPEND_CRC;

	if (skb->ip_summed  == CHECKSUM_PARTIAL)
		length_status |= DMA_TX_DO_CSUM;

	tx_cb_ptr->bd_addr->length_status = length_status;

#ifdef CONFIG_BCMGENET_DUMP_DATA
	netif_dbg(priv, pktdata, dev, "%s: data 0x%p len %d",
			__func__, skb->data, skb->len);
	if (netif_msg_pkdata(priv))
		print_hex_dump(KERN_DEBUG, "", DUMP_PREFIX_ADDRESS,
				16, 1, skb->data, skb->len, 0);
#endif
	/* Decrement total BD count and advance our write pointer */
	if (index == DESC_INDEX || !netif_is_multiqueue(dev))
		priv->tx_free_bds -= 1;
	else
		priv->tx_ring_free_bds[index] -= 1;

	bcmgenet_tdma_inc_prod_write(priv, tdma_ring, write_ptr);

	/* update stats */
	dev->stats.tx_bytes += skb_len;
	dev->stats.tx_packets++;

	return 0;
}

/* Transmit a SKB fragement */
static int bcmgenet_xmit_frag(struct net_device *dev,
				skb_frag_t *frag,
				u16 dma_desc_flags,
				int index,
				unsigned int tdma_ring,
				unsigned int write_ptr)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);
	dma_addr_t mapping;
	struct enet_cb *tx_cb_ptr;
	int ret;

	tx_cb_ptr = bcmgenet_get_txcb(dev, &write_ptr, index);

	if (unlikely(!tx_cb_ptr))
		BUG();
	tx_cb_ptr->skb = NULL;

	mapping = skb_frag_dma_map(&priv->dev->dev, frag, 0,
		skb_frag_size(frag), DMA_TO_DEVICE);
	ret = dma_mapping_error(&priv->dev->dev, mapping);
	if (ret) {
		netif_err(priv, tx_err, dev, "%s: Tx DMA map failed\n",
				__func__);
		/*TODO: Handle frag failure.*/
		return ret;
	}

	dma_unmap_addr_set(tx_cb_ptr, dma_addr, mapping);
	dma_unmap_len_set(tx_cb_ptr, dma_len, frag->size);
	dmadesc_set_addr(priv, tx_cb_ptr->bd_addr, mapping);
#ifdef CONFIG_BCMGENET_DUMP_DATA
	netif_dbg(priv, pktdata, dev, "%s: frag%d len %d",
				__func__, i, frag->size);
		print_hex_dump(KERN_DEBUG, "", DUMP_PREFIX_ADDRESS,
			16, 1,
			skb_frag_address(frag),
			skb_frag_size(frag), 0);
	}
#endif
	tx_cb_ptr->bd_addr->length_status =
			((unsigned long)frag->size << 16) | dma_desc_flags |
			(priv->hw_params->qtag_mask << DMA_TX_QTAG_SHIFT);

	if (index == DESC_INDEX || !netif_is_multiqueue(dev))
		priv->tx_free_bds -= 1;
	else
		priv->tx_ring_free_bds[index] -= 1;

	/* advance producer index and write pointer.*/
	bcmgenet_tdma_inc_prod_write(priv, tdma_ring, write_ptr);

	/* update stats */
	dev->stats.tx_bytes += frag->size;

	return 0;
}

/* Reallocate the SKB to put enough headroom in front of it and insert
 * the transmit checksum offsets in the descriptors
 */
static int bcmgenet_put_tx_csum(struct net_device *dev, struct sk_buff *skb)
{
	struct sk_buff *new_skb;
	u16 offset;
	struct status_64 *status = NULL;
	u8 ip_proto;
	u16 ip_ver;
	u32 tx_csum_info;

	/* We already have enough, good */
	if (skb_headroom(skb) >= sizeof(*status))
		goto adjust_skb;

	/* If 64 byte status block enabled, must make sure skb has
	 * enough headroom for us to insert 64B status block.
	 */
	new_skb = skb_realloc_headroom(skb, sizeof(*status));
	dev_kfree_skb(skb);
	if (!new_skb) {
		dev->stats.tx_errors++;
		dev->stats.tx_dropped++;
		return -ENOMEM;
	}

	skb = new_skb;
adjust_skb:
	skb_push(skb, sizeof(*status));
	status = (struct status_64 *)skb->data;

	if (skb->ip_summed  == CHECKSUM_PARTIAL) {
		ip_ver = htons(skb->protocol);
		switch (ip_ver) {
		case ETH_P_IP:
			ip_proto = ip_hdr(skb)->protocol;
			break;
		case ETH_P_IPV6:
			ip_proto = ipv6_hdr(skb)->nexthdr;
			break;
		default:
			return 0;
		}

		offset = skb_checksum_start_offset(skb) - sizeof(*status);
		tx_csum_info = (offset << STATUS_TX_CSUM_START_SHIFT) |
				(offset + skb->csum_offset);

		/* Set the length valid bit for TCP and UDP and just set
		 * the special UDP flag for IPv4, else just set to 0.
		 */
		if (ip_proto == IPPROTO_TCP || ip_proto == IPPROTO_UDP) {
			tx_csum_info |= STATUS_TX_CSUM_LV;
			if (ip_proto == IPPROTO_UDP && ip_ver == ETH_P_IP)
				tx_csum_info |= STATUS_TX_CSUM_PROTO_UDP;
		} else
			tx_csum_info = 0;

		status->tx_csum_info = tx_csum_info;
	}

	return 0;
}

static netdev_tx_t bcmgenet_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);
	unsigned int tdma_ring;
	unsigned int write_ptr = 0;
	int i = 0;
	unsigned long flags;
	int nr_frags = 0, index = DESC_INDEX;
	u16 dma_desc_flags;
	int ret;

	spin_lock_irqsave(&priv->lock, flags);

	if (!priv->clock_active) {
		netdev_err(dev, "transmitting with gated clock!\n");
		dev_kfree_skb_any(skb);
		spin_unlock_irqrestore(&priv->lock, flags);
		return NETDEV_TX_OK;
	}

	index = skb_get_queue_mapping(skb);
	/* Mapping strategy:
	 * queue_mapping = 0, unclassfieid, packet xmited through ring16
	 * queue_mapping = 1, goes to ring 0. (highest priority queue
	 * queue_mapping = 2, goes to ring 1.
	 * queue_mapping = 3, goes to ring 2.
	 * queue_mapping = 4, goes to ring 3.
	 */
	if (index == 0)
		index = DESC_INDEX;
	else
		index -= 1;
	if (index != DESC_INDEX && index > 3) {
		netdev_err(dev, "%s: queue_mapping %d is invalid\n",
				__func__, skb_get_queue_mapping(skb));
		spin_unlock_irqrestore(&priv->lock, flags);
		dev->stats.tx_errors++;
		dev->stats.tx_dropped++;
		return NETDEV_TX_BUSY;
	}
	nr_frags = skb_shinfo(skb)->nr_frags;
	if (index == DESC_INDEX || !netif_is_multiqueue(dev)) {
		if (priv->tx_free_bds <= nr_frags + 1) {
			netif_stop_subqueue(dev, 0);
			spin_unlock_irqrestore(&priv->lock, flags);
			netdev_err(dev,
				"%s: tx ring %d full when queue awake\n",
				__func__, index);
			return NETDEV_TX_BUSY;
		}
	} else if (priv->tx_ring_free_bds[index] <= nr_frags + 1) {
		netif_stop_subqueue(dev, index+1);
		spin_unlock_irqrestore(&priv->lock, flags);
		netdev_err(dev, "%s: tx ring %d full when queue awake\n",
				__func__, index);
		return NETDEV_TX_BUSY;
	}

	/* Reclaim xmited skb for each subqueue */
	if (netif_is_multiqueue(dev))
		for (i = 0; i < GENET_MQ_CNT; i++)
			bcmgenet_tx_reclaim(dev, i);

	/* reclaim xmited skb every 8 packets. */
	if ((index == DESC_INDEX) &&
		(priv->tx_free_bds < priv->num_tx_bds - 8))
		bcmgenet_tx_reclaim(dev, index);

	if (netif_is_multiqueue(dev)) {
		if ((index != DESC_INDEX) && (priv->tx_ring_free_bds[index]
				< GENET_MQ_BD_CNT - 8))
			bcmgenet_tx_reclaim(dev, index);
	}

	/* set the SKB transmit checksum */
	if (priv->desc_64b_en) {
		ret = bcmgenet_put_tx_csum(dev, skb);
		if (ret) {
			ret = NETDEV_TX_OK;
			goto out;
		}
	}

	tdma_ring = index;
	write_ptr = bcmgenet_tdma_ring_readl(priv, tdma_ring,
			TDMA_WRITE_PTR);

	dma_desc_flags = DMA_SOP;
	if (nr_frags == 0)
		dma_desc_flags |= DMA_EOP;

	/* Transmit single SKB or head of fragment list */
	ret = bcmgenet_xmit_single(dev, skb, dma_desc_flags,
					index, tdma_ring, write_ptr);
	if (ret) {
		ret = NETDEV_TX_OK;
		goto out;
	}

	/* xmit fragment */
	for (i = 0; i < nr_frags; i++) {
		/*TODO: Handle frag failure.*/
		ret = bcmgenet_xmit_frag(dev,
				&skb_shinfo(skb)->frags[i],
				(i == nr_frags - 1) ? DMA_EOP : 0,
				index, tdma_ring, write_ptr);
		if (ret) {
			ret = NETDEV_TX_OK;
			goto out;
		}
	}
	dev->stats.tx_packets++;

	if (index == DESC_INDEX || !netif_is_multiqueue(dev)) {
		if (priv->tx_free_bds <= (MAX_SKB_FRAGS + 1)) {
			netif_stop_subqueue(dev, 0);
			bcmgenet_intrl2_0_writel(priv,
				UMAC_IRQ_TXDMA_BDONE |
				UMAC_IRQ_TXDMA_PDONE,
				INTRL2_CPU_MASK_CLEAR);

		}
	} else if (priv->tx_ring_free_bds[index] <=
			(MAX_SKB_FRAGS + 1)) {
		netif_stop_subqueue(dev, index+1);
		bcmgenet_intrl2_1_writel(priv, (1 << index),
				INTRL2_CPU_MASK_CLEAR);
		priv->int1_mask &= ~(1 << index);
	}

	dev->trans_start = jiffies;
out:
	spin_unlock_irqrestore(&priv->lock, flags);

	return ret;
}

static int bcmgenet_rx_refill(struct bcmgenet_priv *priv,
				struct enet_cb *cb)
{
	dma_addr_t mapping;
	struct sk_buff *skb;
	int ret;

	skb = netdev_alloc_skb(priv->dev,
				priv->rx_buf_len + SKB_ALIGNMENT);
	if (!skb)
		return -ENOMEM;

	handle_alignment(priv, skb);
	cb->skb = skb;
	mapping = dma_map_single(&priv->dev->dev,
				skb->data, priv->rx_buf_len, DMA_FROM_DEVICE);
	ret = dma_mapping_error(&priv->dev->dev, mapping);
	if (ret) {
		dev_kfree_skb_any(skb);
		cb->skb = NULL;
		netif_err(priv, rx_err, priv->dev,
				"%s DMA map failed\n", __func__);
		return ret;
	}

	dma_unmap_addr_set(cb, dma_addr, mapping);
	/* assign packet, prepare descriptor, and advance pointer */
	dmadesc_set_addr(priv, priv->rx_bd_assign_ptr, mapping);
	priv->rx_bd_assign_ptr->length_status = (priv->rx_buf_len << 16);

	/* turn on the newly assigned BD for DMA to use */
	if (priv->rx_bd_assign_ptr ==
			priv->rx_bds+priv->num_rx_bds-1)
		priv->rx_bd_assign_ptr = priv->rx_bds;
	else
		priv->rx_bd_assign_ptr++;

	return 0;
}

/* bcmgenet_desc_rx - descriptor based rx process.
 * this could be called from bottom half, or from NAPI polling method.
 */
static unsigned int bcmgenet_desc_rx(void *ptr, unsigned int budget)
{
	struct bcmgenet_priv *priv = ptr;
	struct net_device *dev = priv->dev;
	struct enet_cb *cb;
	struct sk_buff *skb;
	unsigned long dma_flag;
	int len, err;
	unsigned int rxpktprocessed = 0, rxpkttoprocess = 0;
	unsigned int p_index = 0, c_index = 0, read_ptr = 0, cur_read_ptr;
	p_index = bcmgenet_rdma_ring_readl(priv,
			DESC_INDEX, RDMA_PROD_INDEX);
	p_index &= DMA_P_INDEX_MASK;
	c_index = bcmgenet_rdma_ring_readl(priv,
			DESC_INDEX, RDMA_CONS_INDEX);
	c_index &= DMA_C_INDEX_MASK;
	read_ptr = bcmgenet_rdma_ring_readl(priv,
			DESC_INDEX, RDMA_READ_PTR);

	if (p_index < c_index)
		rxpkttoprocess = (DMA_C_INDEX_MASK+1) - c_index + p_index;
	else
		rxpkttoprocess = p_index - c_index;

	netif_dbg(priv, rx_status, dev,
		"RDMA: rxpkttoprocess=%d\n", rxpkttoprocess);

	while ((rxpktprocessed < rxpkttoprocess) &&
			(rxpktprocessed < budget)) {

		dma_flag = (priv->rx_bds[read_ptr].length_status & 0xffff);
		len = ((priv->rx_bds[read_ptr].length_status)>>16);

		netif_dbg(priv, rx_status, dev,
			"%s:p_index=%d c_index=%d read_ptr=%d "
			"len_stat=0x%08x\n",
			__func__, p_index, c_index, read_ptr,
			priv->rx_bds[read_ptr].length_status);

		rxpktprocessed++;

		/* Retain read pointer index, if we need to drop the packet
		 * because of DMA, we have to advance to the next DMA
		 * descriptor
		 */
		cur_read_ptr = read_ptr;

		dmadesc_set_addr(priv, &priv->rx_bds[read_ptr], 0);

		if (read_ptr == priv->num_rx_bds-1)
			read_ptr = 0;
		else
			read_ptr++;

		if (unlikely(!(dma_flag & DMA_EOP) || !(dma_flag & DMA_SOP))) {
			netif_err(priv, rx_status, dev,
					"Droping fragmented packet!\n");
			dev->stats.rx_dropped++;
			dev->stats.rx_errors++;
			continue;
		}
		/* report errors */
		if (unlikely(dma_flag & (DMA_RX_CRC_ERROR |
						DMA_RX_OV |
						DMA_RX_NO |
						DMA_RX_LG |
						DMA_RX_RXER))) {
			netif_err(priv, rx_status, dev, "dma_flag=0x%x\n",
						(unsigned int)dma_flag);
			if (dma_flag & DMA_RX_CRC_ERROR)
				dev->stats.rx_crc_errors++;
			if (dma_flag & DMA_RX_OV)
				dev->stats.rx_over_errors++;
			if (dma_flag & DMA_RX_NO)
				dev->stats.rx_frame_errors++;
			if (dma_flag & DMA_RX_LG)
				dev->stats.rx_length_errors++;
			dev->stats.rx_dropped++;
			dev->stats.rx_errors++;
			continue;
		} /* error packet */

		cb = &priv->rx_cbs[cur_read_ptr];
		skb = cb->skb;
		BUG_ON(skb == NULL);
		dma_unmap_single(&dev->dev, dma_unmap_addr(cb, dma_addr),
				priv->rx_buf_len, DMA_FROM_DEVICE);

		skb_put(skb, len);
		if (priv->desc_64b_en) {
			struct status_64 *status;
			status = (struct status_64 *)skb->data;
			/* we have 64B rx status block enabled.*/
			if (priv->desc_rxchk_en &&
				(status->rx_csum & STATUS_RX_CSUM_OK))
				skb->ip_summed = CHECKSUM_UNNECESSARY;
			skb_pull(skb, 64);
			len -= 64;
		}

		/* remove hardware 2bytes added for IP alignment */
		skb_pull(skb, 2);
		len -= 2;

		if (priv->crc_fwd_en) {
			skb_trim(skb, len - ETH_FCS_LEN);
			len -= ETH_FCS_LEN;
		}
#ifdef CONFIG_BCMGENET_DUMP_DATA
		netif_dbg(priv, pktdata, dev,
			"bcmgenet_desc_rx : len=%d", skb->len);
		if (netif_msg_pktdata(priv))
			print_hex_dump(KERN_DEBUG, "", DUMP_PREFIX_ADDRESS,
				16, 1, skb->data, skb->len, 0);
#endif

		/*Finish setting up the received SKB and send it to the kernel*/
		skb->protocol = eth_type_trans(skb, priv->dev);
		dev->stats.rx_packets++;
		dev->stats.rx_bytes += len;
		if (dma_flag & DMA_RX_MULT)
			dev->stats.multicast++;

		/* Notify kernel */
		napi_gro_receive(&priv->napi, skb);
		cb->skb = NULL;
		netif_dbg(priv, rx_status, dev, "pushed up to kernel\n");

		/* refill RX path on the current control block */
		err = bcmgenet_rx_refill(priv, cb);
		if (err)
			netif_err(priv, rx_err, dev, "Rx refill failed\n");
	}

	return rxpktprocessed;
}

/* assign_rx_buffers:
 * Assign skb to RX DMA descriptor.
 */
static int assign_rx_buffers(struct bcmgenet_priv *priv)
{
	unsigned short bdsfilled = 0;
	struct enet_cb *cb;
	int ret;
	u32 reg;

	netif_dbg(priv, hw, priv->dev, "%s:\n", __func__);

	/* This function may be called from irq bottom-half. */
	spin_lock_bh(&priv->bh_lock);

	/* loop here for each buffer needing assign */
	while (dmadesc_get_addr(priv, priv->rx_bd_assign_ptr) == 0) {
		cb = &priv->rx_cbs[priv->rx_bd_assign_ptr-priv->rx_bds];
		ret = bcmgenet_rx_refill(priv, cb);
		if (ret)
			break;

		/* keep count of any BD's we refill */
		bdsfilled++;
	}

	/* Enable rx DMA incase it was disabled due to running out of rx BD */
	reg = bcmgenet_rdma_readl(priv, DMA_CTRL);
	reg |= DMA_EN;
	bcmgenet_rdma_writel(priv, reg, DMA_CTRL);

	spin_unlock_bh(&priv->bh_lock);

	netif_dbg(priv, hw, priv->dev, "%s return bdsfilled=%d\n",
		__func__, bdsfilled);

	return bdsfilled;
}




static void save_state(struct bcmgenet_priv *priv)
{
	int ii;
	volatile struct dma_desc *rx_bd_assign_ptr = priv->rx_bds;

	/* FIXME: Why is this code saving/restoring descriptors across suspend?
	 * Most other drivers just shut down and restart the network i/f.
	 */
	for (ii = 0; ii < priv->num_rx_bds; ++ii, ++rx_bd_assign_ptr) {
		priv->saved_rx_desc[ii].length_status =
			rx_bd_assign_ptr->length_status;
		dmadesc_set_addr(priv, &priv->saved_rx_desc[ii],
				 dmadesc_get_addr(priv, rx_bd_assign_ptr));
	}

	priv->int0_mask = bcmgenet_intrl2_0_readl(priv,
			INTRL2_CPU_MASK_STATUS);
	priv->rbuf_ctrl = bcmgenet_rbuf_readl(priv, RBUF_CTRL);
}

static void restore_state(struct bcmgenet_priv *priv)
{
	int ii;
	volatile struct dma_desc *rx_bd_assign_ptr = priv->rx_bds;
	u32 reg;

	bcmgenet_intrl2_0_writel(priv, 0xFFFFFFFF ^ priv->int0_mask,
			INTRL2_CPU_MASK_CLEAR);
	bcmgenet_rbuf_writel(priv, priv->rbuf_ctrl, RBUF_CTRL);

	for (ii = 0; ii < priv->num_rx_bds; ++ii, ++rx_bd_assign_ptr) {
		rx_bd_assign_ptr->length_status =
			priv->saved_rx_desc[ii].length_status;
		dmadesc_set_addr(priv, rx_bd_assign_ptr,
			dmadesc_get_addr(priv, &priv->saved_rx_desc[ii]));
	}

	reg = bcmgenet_rdma_readl(priv, DMA_CTRL);
	reg |= DMA_EN;
	bcmgenet_rdma_writel(priv, reg, DMA_CTRL);

}

/* init_umac: Initializes the uniMac controller */
static int init_umac(struct bcmgenet_priv *priv)
{
	u32 reg, cpu_mask_clear;
	int timeout = 0;

	dev_dbg(&priv->pdev->dev, "bcmgenet: init_umac\n");

	/* 7358a0/7552a0: bad default in RBUF_FLUSH_CTRL.umac_sw_rst */
	bcmgenet_rbuf_ctrl_set(priv, 0);
	udelay(10);

	/* disable MAC while updating its registers */
	bcmgenet_umac_writel(priv, 0, UMAC_CMD);

	/* issue soft reset, wait for it to complete */
	bcmgenet_umac_writel(priv, CMD_SW_RESET, UMAC_CMD);
	while (timeout++ < 1000) {
		reg = bcmgenet_umac_readl(priv, UMAC_CMD);
		if (!(reg & CMD_SW_RESET))
			break;
		udelay(1);
	}

	if (timeout == 1000) {
		dev_err(&priv->pdev->dev,
			"timeout waiting for MAC to come out of resetn\n");
		return -ETIMEDOUT;
	}

	bcmgenet_umac_writel(priv, 0, UMAC_CMD);
	/* clear tx/rx counter */
	bcmgenet_umac_writel(priv,
		MIB_RESET_RX | MIB_RESET_TX | MIB_RESET_RUNT, UMAC_MIB_CTRL);
	bcmgenet_umac_writel(priv, 0, UMAC_MIB_CTRL);

#ifdef MAC_LOOPBACK
	/* Enable GMII/MII loopback */
	reg = bcmgenet_umac_readl(priv, UMAC_CMD);
	reg |= CMD_LCL_LOOP_EN;
	bcmgenet_umac_writel(priv, reg, UMAC_CMD):
#endif
	bcmgenet_umac_writel(priv, ENET_MAX_MTU_SIZE, UMAC_MAX_FRAME_LEN);
	/*
	 * init rx registers, enable ip header optimization.
	 */
	reg = bcmgenet_rbuf_readl(priv, RBUF_CTRL);
	reg |= RBUF_ALIGN_2B ;
	bcmgenet_rbuf_writel(priv, reg, RBUF_CTRL);

	if (!GENET_IS_V1(priv) && !GENET_IS_V2(priv))
		bcmgenet_rbuf_writel(priv, 1, RBUF_TBUF_SIZE_CTRL);

	/* Mask all interrupts.*/
	bcmgenet_intrl2_0_writel(priv, 0xFFFFFFFF, INTRL2_CPU_MASK_SET);
	bcmgenet_intrl2_0_writel(priv, 0xFFFFFFFF, INTRL2_CPU_CLEAR);
	bcmgenet_intrl2_0_writel(priv, 0, INTRL2_CPU_MASK_CLEAR);

	cpu_mask_clear = UMAC_IRQ_RXDMA_BDONE;

	dev_dbg(&priv->pdev->dev,
		"%s:Enabling RXDMA_BDONE interrupt\n", __func__);

	/* Monitor cable plug/unpluged event for internal PHY */
	if (priv->phy_type == BRCM_PHY_TYPE_INT) {
#if !defined(CONFIG_BCM7445A0) && !defined(CONFIG_BCM7445B0)
		/* HW7445-870: energy detect on A0 and B0 silicon
		 * is unreliable
		 */
		cpu_mask_clear |= (UMAC_IRQ_PHY_DET_R | UMAC_IRQ_PHY_DET_F);
#endif /* !defined(CONFIG_BCM7445A0) && !defined(CONFIG_BCM7445B0) */
		cpu_mask_clear |= (UMAC_IRQ_LINK_DOWN | UMAC_IRQ_LINK_UP);
		/* Turn on ENERGY_DET interrupt in bcmgenet_open()
		 * TODO: fix me for active standby.
		 */
	} else if (priv->ext_phy) {
		cpu_mask_clear |= (UMAC_IRQ_LINK_DOWN | UMAC_IRQ_LINK_UP);
	} else if (priv->phy_type == BRCM_PHY_TYPE_MOCA) {
		reg = bcmgenet_bp_mc_get(priv);
		reg |= BIT(priv->hw_params->bp_in_en_shift);

		/* bp_mask: back pressure mask */
		if (netif_is_multiqueue(priv->dev))
			reg |= priv->hw_params->bp_in_mask;
		else
			reg &= ~priv->hw_params->bp_in_mask;
		bcmgenet_bp_mc_set(priv, reg);
	}

	/* Enable MDIO interrupts on GENET v3+ */
	if (priv->hw_params->flags & GENET_HAS_MDIO_INTR)
		cpu_mask_clear |= UMAC_IRQ_MDIO_DONE | UMAC_IRQ_MDIO_ERROR;

	bcmgenet_intrl2_0_writel(priv, cpu_mask_clear,
		INTRL2_CPU_MASK_CLEAR);

	/* Enable rx/tx engine.*/
	dev_dbg(&priv->pdev->dev, "done init umac\n");

	return 0;
}

/* init multi xmit queues, only available for GENET2
 * the queue is partitioned as follows:
 *
 * queue 0 - 3 is priority based, each one has 48 descriptors,
 * with queue 0 being the highest priority queue.
 *
 * queue 16 is the default tx queue, with 64 descriptors.
 */
static void bcmgenet_init_multiq(struct net_device *dev)
{
	int i, dma_enable;
	struct bcmgenet_priv *priv = netdev_priv(dev);
	u32 reg;
	u32 words_per_bd = priv->hw_params->words_per_bd;

	if (!netif_is_multiqueue(dev)) {
		netdev_warn(dev, "called with non multi queue aware HW\n");
		return;
	}

	reg = bcmgenet_tdma_readl(priv, DMA_CTRL);
	dma_enable = reg & DMA_EN;
	reg &= ~DMA_EN;
	bcmgenet_tdma_writel(priv, reg, DMA_CTRL);
	/* Enable strict priority arbiter mode */
	bcmgenet_tdma_writel(priv, 0x2, DMA_ARB_CTRL);
	for (i = 0; i < GENET_MQ_CNT; i++) {
		int first_bd;

		/* first 64 tx_cbs are reserved for default tx queue
		 * (ring 16)
		 */
		priv->tx_ring_cbs[i] = priv->tx_cbs +
			GENET_DEFAULT_BD_CNT + i * GENET_MQ_BD_CNT;
		priv->tx_ring_size[i] = GENET_MQ_BD_CNT;
		priv->tx_ring_c_index[i] = 0;
		priv->tx_ring_free_bds[i] = GENET_MQ_BD_CNT;

		bcmgenet_tdma_ring_writel(priv, i, 0, TDMA_PROD_INDEX);
		bcmgenet_tdma_ring_writel(priv, i, 0, TDMA_CONS_INDEX);
		bcmgenet_tdma_ring_writel(priv, i,
			(GENET_MQ_BD_CNT << DMA_RING_SIZE_SHIFT) |
			RX_BUF_LENGTH, DMA_RING_BUF_SIZE);

		first_bd = GENET_MQ_BD_CNT * i;
		bcmgenet_tdma_ring_writel(priv, i,
			first_bd * words_per_bd, DMA_START_ADDR);
		bcmgenet_tdma_ring_writel(priv, i,
			first_bd * words_per_bd, TDMA_READ_PTR);
		bcmgenet_tdma_ring_writel(priv, i,
			first_bd, TDMA_WRITE_PTR);

		bcmgenet_tdma_ring_writel(priv, i,
			words_per_bd * (i + 1) * GENET_MQ_BD_CNT - 1,
			DMA_END_ADDR);
		bcmgenet_tdma_ring_writel(priv, i,
			ENET_MAX_MTU_SIZE << 16, TDMA_FLOW_PERIOD);
		bcmgenet_tdma_ring_writel(priv, i,
			1, DMA_MBUF_DONE_THRESH);

		/* Configure ring as decriptor ring and setup priority */
		reg = bcmgenet_tdma_readl(priv, DMA_RING_CFG);
		reg |= (1 << i);
		bcmgenet_tdma_writel(priv, reg, DMA_RING_CFG);
		reg = bcmgenet_tdma_readl(priv, DMA_PRIORITY);
		reg |= ((GENET_Q0_PRIORITY + i) << 5*i);
		bcmgenet_tdma_writel(priv, reg, DMA_PRIORITY);
		reg = bcmgenet_tdma_readl(priv, DMA_CTRL);
		reg |= (1 << (i + DMA_RING_BUF_EN_SHIFT));
		bcmgenet_tdma_writel(priv, reg, DMA_CTRL);
	}
	/* Set ring #16 priority */
	reg = bcmgenet_tdma_readl(priv, DMA_RING_PRIORITY);
	reg |= ((GENET_Q0_PRIORITY + GENET_MQ_CNT) << 20);
	bcmgenet_tdma_writel(priv, reg, DMA_PRIORITY);
	if (dma_enable) {
		reg = bcmgenet_tdma_readl(priv, DMA_CTRL);
		reg |= DMA_EN;
		bcmgenet_tdma_writel(priv, reg, DMA_CTRL);
	}
}

/* init_edma: Initialize DMA control register */
static void init_edma(struct bcmgenet_priv *priv)
{
	int __maybe_unused first_bd;
	unsigned int rdma_desc;
	unsigned int tdma_desc;
	u32 words_per_bd = priv->hw_params->words_per_bd;

	netif_dbg(priv, hw, priv->dev, "bcmgenet: init_edma\n");

	/* init rDma */
	bcmgenet_rdma_writel(priv,
		DMA_MAX_BURST_LENGTH, DMA_SCB_BURST_SIZE);
	/* by default, enable ring 16 (descriptor based) */
	rdma_desc = DESC_INDEX;
	bcmgenet_rdma_ring_writel(priv, rdma_desc, 0, RDMA_WRITE_PTR);
	bcmgenet_rdma_ring_writel(priv, rdma_desc, 0, RDMA_PROD_INDEX);
	bcmgenet_rdma_ring_writel(priv, rdma_desc, 0, RDMA_CONS_INDEX);
	bcmgenet_rdma_ring_writel(priv, rdma_desc,
		((TOTAL_DESC << DMA_RING_SIZE_SHIFT) | RX_BUF_LENGTH),
		DMA_RING_BUF_SIZE);
	bcmgenet_rdma_ring_writel(priv, rdma_desc, 0, DMA_START_ADDR);
	bcmgenet_rdma_ring_writel(priv, rdma_desc,
		words_per_bd * TOTAL_DESC - 1, DMA_END_ADDR);
	bcmgenet_rdma_ring_writel(priv, rdma_desc,
			(DMA_FC_THRESH_LO << DMA_XOFF_THRESHOLD_SHIFT) |
			DMA_FC_THRESH_HI, RDMA_XON_XOFF_THRESH);
	bcmgenet_rdma_ring_writel(priv, rdma_desc, 0, RDMA_READ_PTR);

	/* Init tDma */
	bcmgenet_tdma_writel(priv,
		DMA_MAX_BURST_LENGTH, DMA_SCB_BURST_SIZE);
	/* by default, enable ring DESC_INDEX (descriptor based) */
	tdma_desc = DESC_INDEX;

	bcmgenet_tdma_ring_writel(priv, tdma_desc, 0, TDMA_PROD_INDEX);
	bcmgenet_tdma_ring_writel(priv, tdma_desc, 0, TDMA_CONS_INDEX);
	bcmgenet_tdma_ring_writel(priv, tdma_desc,
			1, DMA_MBUF_DONE_THRESH);
	/* Disable rate control for now */
	bcmgenet_tdma_ring_writel(priv, tdma_desc, 0, TDMA_FLOW_PERIOD);
	/* Unclassified traffic goes to ring 16 */
	bcmgenet_tdma_ring_writel(priv, tdma_desc,
			((GENET_DEFAULT_BD_CNT << DMA_RING_SIZE_SHIFT) |
			 RX_BUF_LENGTH), DMA_RING_BUF_SIZE);

	first_bd = GENET_MQ_CNT * GENET_MQ_BD_CNT;
	bcmgenet_tdma_ring_writel(priv, tdma_desc, first_bd * words_per_bd,
			DMA_START_ADDR);
	bcmgenet_tdma_ring_writel(priv, tdma_desc, first_bd * words_per_bd,
			TDMA_READ_PTR);
	/* FIXME: tdma_write_pointer and rdma_read_pointer are
	 * essentially
	 * scratch registers and the GENET block does not use them for
	 * anything.  We should maintain them in DRAM instead, because
	 * register accesses are expensive.
	 */
	bcmgenet_tdma_ring_writel(priv, tdma_desc, first_bd,
			TDMA_WRITE_PTR);
	bcmgenet_tdma_ring_writel(priv, tdma_desc,
			TOTAL_DESC * words_per_bd - 1, DMA_END_ADDR);
	priv->tx_free_bds = GENET_DEFAULT_BD_CNT;

	/* initiaize multi xmit queue */
	if (netif_is_multiqueue(priv->dev))
		bcmgenet_init_multiq(priv->dev);
}


/* NAPI polling method*/
static int bcmgenet_poll(struct napi_struct *napi, int budget)
{
	struct bcmgenet_priv *priv = container_of(napi,
			struct bcmgenet_priv, napi);
	unsigned int rdma_desc;
	unsigned int work_done;
	unsigned long flags;
	u32 reg;
	work_done = bcmgenet_desc_rx(priv, budget);

	/* tx reclaim */
	spin_lock_irqsave(&priv->lock, flags);
	bcmgenet_tx_reclaim(priv->dev, DESC_INDEX);
	spin_unlock_irqrestore(&priv->lock, flags);
	/* Advancing our read pointer and consumer index*/
	rdma_desc = DESC_INDEX;
	reg = bcmgenet_rdma_ring_readl(priv, rdma_desc, RDMA_CONS_INDEX);
	reg += work_done;
	bcmgenet_rdma_ring_writel(priv, rdma_desc, reg, RDMA_CONS_INDEX);
	reg = bcmgenet_rdma_ring_readl(priv, rdma_desc, RDMA_READ_PTR);
	bcmgenet_rdma_ring_writel(priv, rdma_desc,
		(work_done + reg) & (TOTAL_DESC - 1), RDMA_READ_PTR);
	if (work_done < budget) {
		napi_complete(napi);
		bcmgenet_intrl2_0_writel(priv,
			UMAC_IRQ_RXDMA_BDONE, INTRL2_CPU_MASK_CLEAR);
	}
	return work_done;
}

/* Interrupt bottom half */
static void bcmgenet_irq_task(struct work_struct *work)
{
	struct bcmgenet_priv *priv = container_of(
			work, struct bcmgenet_priv, bcmgenet_irq_work);
	struct net_device *dev;
	u32 reg;

	dev = priv->dev;

	netif_dbg(priv, intr, dev, "%s\n", __func__);
	/* Cable plugged/unplugged event */
	if (priv->phy_type == BRCM_PHY_TYPE_INT) {
		if (priv->irq0_stat & UMAC_IRQ_PHY_DET_R) {
			priv->irq0_stat &= ~UMAC_IRQ_PHY_DET_R;
			netif_crit(priv, link, dev,
				"cable plugged in, powering up\n");
			bcmgenet_power_up(priv, GENET_POWER_CABLE_SENSE);
		} else if (priv->irq0_stat & UMAC_IRQ_PHY_DET_F) {
			priv->irq0_stat &= ~UMAC_IRQ_PHY_DET_F;
			netif_crit(priv, link, dev,
				"cable unplugged, powering down\n");
			bcmgenet_power_down(priv, GENET_POWER_CABLE_SENSE);
		}
	}
	if (priv->irq0_stat & UMAC_IRQ_MPD_R) {
		priv->irq0_stat &= ~UMAC_IRQ_MPD_R;
		netif_crit(priv, wol, dev,
			"magic packet detected, waking up\n");
		/* disable mpd interrupt */
		bcmgenet_intrl2_0_writel(priv,
			UMAC_IRQ_MPD_R, INTRL2_CPU_MASK_SET);
		/* disable CRC forward.*/
		reg = bcmgenet_umac_readl(priv, UMAC_CMD);
		reg &= ~CMD_CRC_FWD;
		bcmgenet_umac_writel(priv, reg, UMAC_CMD);
		priv->crc_fwd_en = 0;
		if (priv->dev_asleep)
			bcmgenet_power_up(priv, GENET_POWER_WOL_MAGIC);

	} else if (priv->irq0_stat & (UMAC_IRQ_HFB_SM | UMAC_IRQ_HFB_MM)) {
		priv->irq0_stat &= ~(UMAC_IRQ_HFB_SM | UMAC_IRQ_HFB_MM);
		netif_crit(priv, wol, dev,
			"ACPI pattern matched, waking up\n");
		/* disable HFB match interrupts */
		bcmgenet_intrl2_0_writel(priv,
			UMAC_IRQ_HFB_SM | UMAC_IRQ_HFB_MM, INTRL2_CPU_MASK_SET);
		if (priv->dev_asleep)
			bcmgenet_power_up(priv, GENET_POWER_WOL_ACPI);
	}

	/* Link UP/DOWN event */
	if (priv->irq0_stat & (UMAC_IRQ_LINK_UP|UMAC_IRQ_LINK_DOWN)) {
		priv->irq0_stat &= ~(UMAC_IRQ_LINK_UP|UMAC_IRQ_LINK_DOWN);
		bcmgenet_mii_setup(priv->dev);
	}
}

/* bcmgenet_isr1: interrupt handler for ring buffer. */
static irqreturn_t bcmgenet_isr1(int irq, void *dev_id)
{
	struct bcmgenet_priv *priv = dev_id;
	unsigned int index;

	/* Save irq status for bottom-half processing. */
	priv->irq1_stat =
		bcmgenet_intrl2_1_readl(priv, INTRL2_CPU_STAT) &
		~priv->int1_mask;
	/* clear inerrupts*/
	bcmgenet_intrl2_1_writel(priv, priv->irq1_stat, INTRL2_CPU_CLEAR);

	netif_dbg(priv, intr, priv->dev,
		"%s: IRQ=0x%x\n", __func__, priv->irq1_stat);
	/* Check the MBDONE interrupts.
	 * packet is done, reclaim descriptors
	 */
	if (priv->irq1_stat & 0x0000ffff) {
		index = 0;
		for (index = 0; index < 16; index++) {
			if (priv->irq1_stat & (1<<index))
				bcmgenet_tx_reclaim(priv->dev, index);
		}
	}
	return IRQ_HANDLED;
}

/* bcmgenet_isr0: Handle various interrupts. */
static irqreturn_t bcmgenet_isr0(int irq, void *dev_id)
{
	struct bcmgenet_priv *priv = dev_id;

	/* Save irq status for bottom-half processing. */
	priv->irq0_stat =
		bcmgenet_intrl2_0_readl(priv, INTRL2_CPU_STAT) &
		~bcmgenet_intrl2_0_readl(priv, INTRL2_CPU_MASK_STATUS);
	/* clear inerrupts*/
	bcmgenet_intrl2_0_writel(priv, priv->irq0_stat, INTRL2_CPU_CLEAR);

	netif_dbg(priv, intr, priv->dev,
		"IRQ=0x%x\n", priv->irq0_stat);

	if (priv->irq0_stat & (UMAC_IRQ_RXDMA_BDONE|UMAC_IRQ_RXDMA_PDONE)) {
		/* We use NAPI(software interrupt throttling, if
		 * Rx Descriptor throttling is not used.
		 * Disable interrupt, will be enabled in the poll method.
		 */
		if (likely(napi_schedule_prep(&priv->napi))) {
			bcmgenet_intrl2_0_writel(priv,
				UMAC_IRQ_RXDMA_BDONE, INTRL2_CPU_MASK_SET);
			__napi_schedule(&priv->napi);
		}
	}
	if (priv->irq0_stat &
			(UMAC_IRQ_TXDMA_BDONE | UMAC_IRQ_TXDMA_PDONE)) {
		/* Tx reclaim */
		bcmgenet_tx_reclaim_all(priv->dev);
	}
	if (priv->irq0_stat & (UMAC_IRQ_PHY_DET_R |
				UMAC_IRQ_PHY_DET_F |
				UMAC_IRQ_LINK_UP |
				UMAC_IRQ_LINK_DOWN |
				UMAC_IRQ_HFB_SM |
				UMAC_IRQ_HFB_MM |
				UMAC_IRQ_MPD_R)) {
		/* all other interested interrupts handled in bottom half */
		schedule_work(&priv->bcmgenet_irq_work);
	}

	if ((priv->hw_params->flags & GENET_HAS_MDIO_INTR) &&
		priv->irq0_stat & (UMAC_IRQ_MDIO_DONE | UMAC_IRQ_MDIO_ERROR)) {
		priv->irq0_stat &= ~(UMAC_IRQ_MDIO_DONE | UMAC_IRQ_MDIO_ERROR);
		wake_up(&priv->wq);
	}

	return IRQ_HANDLED;
}

/* bcmgenet_init_dev: initialize uniMac devie
 * allocate Tx/Rx buffer descriptors pool, Tx control block pool.
 */
static int bcmgenet_init_dev(struct bcmgenet_priv *priv)
{
	int i, ret;
	u32 reg;

	priv->clk = clk_get(&priv->pdev->dev, "enet");
	priv->clk_wol = clk_get(&priv->pdev->dev, "enet-wol");
	bcmgenet_clock_enable(priv);

	reg = bcmgenet_sys_readl(priv, SYS_REV_CTRL);

	/* Print the GENET core version */
	netdev_info(priv->dev, "GENET " GENET_VER_FMT,
		(reg >> 24) & 0x0f, (reg >> 16) & 0x0f,
		reg & 0xffff);

	netif_dbg(priv, ifup, priv->dev, "%s\n", __func__);
	/* setup buffer/pointer relationships here */
	priv->num_tx_bds = priv->num_rx_bds = TOTAL_DESC;
	/* Always use 2KB buffer for 7420*/
	priv->rx_buf_len = RX_BUF_LENGTH;

	if (priv->hw_params->flags & GENET_HAS_EXT) {
	/* SWLINUX-1813: EXT block is not available on MOCA_GENET */
#if !defined(CONFIG_BCM7125)
		if (priv->pdev->id == 1)
#endif
			priv->hw_params->flags &= ~GENET_HAS_EXT;
	}
	priv->rx_bds = (struct dma_desc *)(priv->base +
			priv->hw_params->rdma_offset);
	priv->tx_bds = (struct dma_desc *)(priv->base +
			priv->hw_params->tdma_offset);

	netif_dbg(priv, ifup, priv->dev,
		"%s: rxbds=0x%08x txbds=0x%08x\n", __func__,
		(unsigned int)priv->rx_bds, (unsigned int)priv->tx_bds);

	/* alloc space for the tx control block pool */
	priv->tx_cbs = kzalloc(priv->num_tx_bds * sizeof(struct enet_cb),
			GFP_KERNEL);
	if (!priv->tx_cbs) {
		bcmgenet_clock_disable(priv);
		return -ENOMEM;
	}

	/* initialize rx ring pointer variables. */
	priv->rx_bd_assign_ptr = priv->rx_bds;
	priv->rx_cbs = kzalloc(priv->num_rx_bds * sizeof(struct enet_cb),
			GFP_KERNEL);
	if (!priv->rx_cbs) {
		ret = -ENOMEM;
		goto error2;
	}

	/* init the receive buffer descriptor ring */
	for (i = 0; i < priv->num_rx_bds; i++) {
		priv->rx_bds[i].length_status = (priv->rx_buf_len<<16);
		dmadesc_set_addr(priv, &priv->rx_bds[i], 0);
	}

	/* clear the transmit buffer descriptors */
	for (i = 0; i < priv->num_tx_bds; i++) {
		priv->tx_bds[i].length_status = 0<<16;
		dmadesc_set_addr(priv, &priv->tx_bds[i], 0);
	}
	priv->tx_free_bds = priv->num_tx_bds;

	/* fill receive buffers */
	if (assign_rx_buffers(priv) == 0) {
		netdev_err(priv->dev, "Failed to assign rx buffers\n");
		ret = -ENOMEM;
		goto error1;
	}

	netif_dbg(priv, ifup, priv->dev, "%s done!\n", __func__);
	/* init umac registers */
	ret = init_umac(priv);
	if (ret)
		goto error1;

	/* init dma registers */
	init_edma(priv);

	if (netif_is_multiqueue(priv->dev))
		priv->num_tx_bds = GENET_DEFAULT_BD_CNT;

	netif_dbg(priv, ifup, priv->dev, "%s done!\n", __func__);
	/* if we reach this point, we've init'ed successfully */
	return 0;
error1:
	kfree(priv->rx_cbs);
error2:
	kfree(priv->tx_cbs);
	bcmgenet_clock_disable(priv);

	netif_dbg(priv, ifup, priv->dev, "%s Failed!\n", __func__);
	return ret;
}

/* Uninitialize tx/rx buffer descriptor pools */
static void bcmgenet_uninit_dev(struct bcmgenet_priv *priv)
{
	int i;

	/* disable DMA */
	bcmgenet_rdma_writel(priv, 0, DMA_CTRL);
	bcmgenet_tdma_writel(priv, 0, DMA_CTRL);

	for (i = 0; i < priv->num_tx_bds; i++) {
		if (priv->tx_cbs[i].skb != NULL) {
			dev_kfree_skb(priv->tx_cbs[i].skb);
			priv->tx_cbs[i].skb = NULL;
		}
	}
	for (i = 0; i < priv->num_rx_bds; i++) {
		if (priv->rx_cbs[i].skb != NULL) {
			dev_kfree_skb(priv->rx_cbs[i].skb);
			priv->rx_cbs[i].skb = NULL;
		}
	}

	/* free the transmit buffer descriptor */
	if (priv->tx_bds)
		priv->tx_bds = NULL;
	/* free the receive buffer descriptor */
	if (priv->rx_bds)
		priv->rx_bds = NULL;
	/* free the transmit control block pool */
	kfree(priv->tx_cbs);
	/* free the transmit control block pool */
	kfree(priv->rx_cbs);

	clk_put(priv->clk);
}

static void bcmgenet_umac_reset(struct bcmgenet_priv *priv)
{
	u32 reg;

	reg = bcmgenet_rbuf_ctrl_get(priv);
	reg &= ~BIT(1);
	bcmgenet_rbuf_ctrl_set(priv, reg);
	udelay(10);
}

static void bcmgenet_set_hw_addr(struct bcmgenet_priv *priv,
				  unsigned char *addr)
{
	bcmgenet_umac_writel(priv, (addr[0] << 24 | addr[1] << 16 |
			addr[2] << 8 | addr[3]), UMAC_MAC0);
	bcmgenet_umac_writel(priv, addr[4] << 8 | addr[5], UMAC_MAC1);
}

static int bcmgenet_wol_resume(struct bcmgenet_priv *priv)
{
	int ret;

	/* From WOL-enabled suspend, switch to regular clock */
	clk_disable(priv->clk_wol);
	/* init umac registers to synchronize s/w with h/w */
	ret = init_umac(priv);
	if (ret)
		return ret;

	if (priv->sw_type)
		bcmgenet_ethsw_init(priv->dev);
	/* Speed settings must be restored */
	bcmgenet_mii_init(priv->dev);
	bcmgenet_mii_setup(priv->dev);

	return 0;
}

/* Returns a reusable dma control register value */
static u32 bcmgenet_dma_disable(struct bcmgenet_priv *priv)
{
	u32 reg;
	u32 dma_ctrl;

	/* disable DMA */
	dma_ctrl = 1 << (DESC_INDEX + DMA_RING_BUF_EN_SHIFT) | DMA_EN;
	reg = bcmgenet_tdma_readl(priv, DMA_CTRL);
	reg &= ~dma_ctrl;
	bcmgenet_tdma_writel(priv, reg, DMA_CTRL);
	reg = bcmgenet_rdma_readl(priv, DMA_CTRL);
	reg &= ~dma_ctrl;
	bcmgenet_rdma_writel(priv, reg, DMA_CTRL);
	bcmgenet_umac_writel(priv, 1, UMAC_TX_FLUSH);
	udelay(10);
	bcmgenet_umac_writel(priv, 0, UMAC_TX_FLUSH);

	return dma_ctrl;
}

static void bcmgenet_enable_dma(struct bcmgenet_priv *priv, u32 dma_ctrl)
{
	u32 reg;

	reg = bcmgenet_rdma_readl(priv, DMA_CTRL);
	reg |= dma_ctrl;
	bcmgenet_rdma_writel(priv, reg, DMA_CTRL);
	reg = bcmgenet_tdma_readl(priv, DMA_CTRL);
	reg |= dma_ctrl;
	bcmgenet_tdma_writel(priv, reg, DMA_CTRL);
}

static int bcmgenet_open(struct net_device *dev)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);
	unsigned long dma_ctrl;
	u32 reg;
	int ret;

	netif_dbg(priv, ifup, dev, "bcmgenet_open\n");

	bcmgenet_clock_enable(priv);

	/* take MAC out of reset */
	bcmgenet_umac_reset(priv);

	/* disable ethernet MAC while updating its registers */
	reg = bcmgenet_umac_readl(priv, UMAC_CMD);
	reg &= ~(CMD_TX_EN | CMD_RX_EN);
	bcmgenet_umac_writel(priv, reg, UMAC_CMD);

	bcmgenet_set_hw_addr(priv, dev->dev_addr);

	if (priv->wol_enabled) {
		ret = bcmgenet_wol_resume(priv);
		if (ret)
			return ret;
	}

	if (priv->phy_type == BRCM_PHY_TYPE_INT) {
		reg = bcmgenet_ext_readl(priv, EXT_EXT_PWR_MGMT);
		reg |= EXT_ENERGY_DET_MASK;
		bcmgenet_ext_writel(priv, reg, EXT_EXT_PWR_MGMT);
	}

	if (test_and_clear_bit(GENET_POWER_WOL_MAGIC, &priv->wol_enabled))
		bcmgenet_power_up(priv, GENET_POWER_WOL_MAGIC);
	if (test_and_clear_bit(GENET_POWER_WOL_ACPI, &priv->wol_enabled))
		bcmgenet_power_up(priv, GENET_POWER_WOL_ACPI);

	/* Disable RX/TX DMA and flush TX queues */
	dma_ctrl = bcmgenet_dma_disable(priv);

	/* reset dma, start from beginning of the ring. */
	init_edma(priv);
	/* reset internal book keeping variables. */
	priv->tx_last_c_index = 0;
	priv->rx_bd_assign_ptr = priv->rx_bds;

	if (0 /* FIXME brcm_pm_deep_sleep() */)
		restore_state(priv);
	else
		assign_rx_buffers(priv);

	priv->tx_free_bds = priv->num_tx_bds;

	/* Always enable ring 16 - descriptor ring */
	bcmgenet_enable_dma(priv, dma_ctrl);

	if (priv->ext_phy)
		mod_timer(&priv->timer, jiffies);

	ret = request_irq(priv->irq0, bcmgenet_isr0, IRQF_SHARED,
				dev->name, priv);
	if (ret < 0) {
		netdev_err(dev, "can't request IRQ %d\n", priv->irq0);
		goto err_timer;
	}

	ret = request_irq(priv->irq1, bcmgenet_isr1, IRQF_SHARED,
				dev->name, priv);
	if (ret < 0) {
		netdev_err(dev, "can't request IRQ %d\n", priv->irq1);
		goto err_free_irq0;
	}

	/* Start the network engine */
	netif_tx_start_all_queues(dev);
	napi_enable(&priv->napi);

	reg = bcmgenet_umac_readl(priv, UMAC_CMD);
	reg |= (CMD_TX_EN | CMD_RX_EN);
	bcmgenet_umac_writel(priv, reg, UMAC_CMD);

#ifdef CONFIG_BRCM_HAS_STANDBY
	brcm_pm_wakeup_register(&bcmgenet_wakeup_ops, priv, dev->name);
	device_set_wakeup_capable(&dev->dev, 1);
#endif

	if (priv->phy_type == BRCM_PHY_TYPE_INT)
		bcmgenet_power_up(priv, GENET_POWER_PASSIVE);

	return 0;

err_free_irq0:
	free_irq(priv->irq0, dev);
err_timer:
	del_timer_sync(&priv->timer);
	netif_tx_stop_all_queues(dev);

	return ret;
}

static int bcmgenet_dma_teardown(struct bcmgenet_priv *priv)
{
	int timeout = 0;
	u32 reg;

	/* Disable TDMA to stop add more frames in TX DMA */
	reg = bcmgenet_tdma_readl(priv, DMA_CTRL);
	reg &= ~DMA_EN;
	bcmgenet_tdma_writel(priv, reg, DMA_CTRL);
	/* Check TDMA status register to confirm TDMA is disabled */
	while (!(bcmgenet_tdma_readl(priv, DMA_STATUS) & DMA_DISABLED)) {
		if (timeout++ == 5000) {
			netdev_warn(priv->dev,
				"Timed out while disabling TX DMA\n");
			return -ETIMEDOUT;
		}
		udelay(1);
	}

	/* SWLINUX-2252: Workaround for rx flush issue causes rbuf overflow */
	/* Wait 10ms for packet drain in both tx and rx dma */
	usleep_range(10000, 20000);

	/* Disable RDMA */
	reg = bcmgenet_rdma_readl(priv, DMA_CTRL);
	reg &= ~DMA_EN;
	bcmgenet_rdma_writel(priv, reg, DMA_CTRL);

	timeout = 0;
	/* Check RDMA status register to confirm RDMA is disabled */
	while (!(bcmgenet_rdma_readl(priv, DMA_STATUS) & DMA_DISABLED)) {
		if (timeout++ == 5000) {
			netdev_warn(priv->dev,
				"Timed out while disabling RX DMA\n");
			return -ETIMEDOUT;
		}
		udelay(1);
	}

	return 0;
}

static void bcmgenet_stop_link_task(struct bcmgenet_priv *priv)
{
	if (priv->ext_phy) {
		del_timer_sync(&priv->timer);
		cancel_work_sync(&priv->bcmgenet_link_work);
	}
}

static int bcmgenet_close(struct net_device *dev)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);
	int ret;
	u32 reg;

	netif_dbg(priv, ifdown, dev, "bcmgenet_close\n");

	/* Disable MAC receive */
	reg = bcmgenet_umac_readl(priv, UMAC_CMD);
	reg &= ~CMD_RX_EN;
	bcmgenet_umac_writel(priv, reg, UMAC_CMD);

	netif_tx_stop_all_queues(dev);

	ret = bcmgenet_dma_teardown(priv);
	if (ret)
		return ret;

	/* Disable MAC transmit. TX DMA disabled have to done before this */
	reg = bcmgenet_umac_readl(priv, UMAC_CMD);
	reg &= ~CMD_TX_EN;
	bcmgenet_umac_writel(priv, reg, UMAC_CMD);

	napi_disable(&priv->napi);

	/* tx reclaim */
	bcmgenet_tx_reclaim_all(dev);
	free_irq(priv->irq0, (void *)priv);
	free_irq(priv->irq1, (void *)priv);

	bcmgenet_stop_link_task(priv);
	/* Wait for pending work items to complete - we are stopping
	 * the clock now. Since interrupts are disabled, no new work
	 * will be scheduled.
	 */
	cancel_work_sync(&priv->bcmgenet_irq_work);

	if (0 /* FIXME brcm_pm_deep_sleep() */)
		save_state(priv);

	if (device_may_wakeup(&dev->dev) && priv->dev_asleep) {
		if (priv->wolopts & (WAKE_MAGIC|WAKE_MAGICSECURE))
			bcmgenet_power_down(priv, GENET_POWER_WOL_MAGIC);
		if (priv->wolopts & WAKE_ARP)
			bcmgenet_power_down(priv, GENET_POWER_WOL_ACPI);
	} else if (priv->phy_type == BRCM_PHY_TYPE_INT)
		bcmgenet_power_down(priv, GENET_POWER_PASSIVE);

	if (priv->wol_enabled)
		clk_enable(priv->clk_wol);

	bcmgenet_clock_disable(priv);

	return 0;
}

static void bcmgenet_timeout(struct net_device *dev)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);

	BUG_ON(dev == NULL);

	netif_dbg(priv, tx_err, dev, "bcmgenet_timeout\n");

	dev->trans_start = jiffies;

	dev->stats.tx_errors++;

	netif_tx_wake_all_queues(dev);
}

#define MAX_MC_COUNT	16

static inline void bcmgenet_set_mdf_addr(struct bcmgenet_priv *priv,
					 unsigned char *addr,
					 int *i,
					 int *mc)
{
	u32 reg;

	bcmgenet_umac_writel(priv, addr[0] << 8 | addr[1],
			UMAC_MDF_ADDR + (*i * 4));
	bcmgenet_umac_writel(priv,
			addr[2] << 24 | addr[3] << 16 |
			addr[4] << 8 | addr[5],
			UMAC_MDF_ADDR + ((*i + 1) * 4));
	reg = bcmgenet_umac_readl(priv, UMAC_MDF_CTRL);
	reg |= (1 << (MAX_MC_COUNT - *mc));
	bcmgenet_umac_writel(priv, reg, UMAC_MDF_CTRL);
	*i += 2;
	(*mc)++;
}

static void bcmgenet_set_rx_mode(struct net_device *dev)
{
	struct bcmgenet_priv *priv = netdev_priv(dev);
	struct netdev_hw_addr *ha;
	int i, mc;
	u32 reg;

	netif_dbg(priv, hw, dev, "%s: %08X\n", __func__, dev->flags);

	/* Promiscous mode */
	reg = bcmgenet_umac_readl(priv, UMAC_CMD);
	if (dev->flags & IFF_PROMISC) {
		reg |= CMD_PROMISC;
		bcmgenet_umac_writel(priv, reg, UMAC_CMD);
		bcmgenet_umac_writel(priv, 0, UMAC_MDF_CTRL);
		return;
	} else {
		reg &= ~CMD_PROMISC;
		bcmgenet_umac_writel(priv, reg, UMAC_CMD);
	}

	/* UniMac doesn't support ALLMULTI */
	if (dev->flags & IFF_ALLMULTI)
		return;

	/* update MDF filter */
	i = 0;
	mc = 0;
	/* Broadcast */
	bcmgenet_set_mdf_addr(priv, dev->broadcast, &i, &mc);
	/* my own address.*/
	bcmgenet_set_mdf_addr(priv, dev->dev_addr, &i, &mc);
	/* Unicast list*/
	if (netdev_uc_count(dev) > (MAX_MC_COUNT - mc))
		return;

	if (!netdev_uc_empty(dev))
		netdev_for_each_uc_addr(ha, dev)
			bcmgenet_set_mdf_addr(priv, ha->addr, &i, &mc);
	/* Multicast */
	if (netdev_mc_empty(dev) || netdev_mc_count(dev) >= (MAX_MC_COUNT - mc))
		return;

	netdev_for_each_mc_addr(ha, dev)
		bcmgenet_set_mdf_addr(priv, ha->addr, &i, &mc);
}

/* Set the hardware MAC address. */
static int bcmgenet_set_mac_addr(struct net_device *dev, void *p)
{
	struct sockaddr *addr = p;

	if (netif_running(dev))
		return -EBUSY;

	memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);

	return 0;
}

static u16 __maybe_unused bcmgenet_select_queue(struct net_device *dev,
		struct sk_buff *skb)
{
	/* If multi-queue support is enabled, and NET_ACT_SKBEDIT is not
	 * defined, this function simply returns current queue_mapping set
	 * inside skb, this means other modules, (netaccel, for example),
	 * must provide a mechanism to set the queue_mapping before trying
	 * to send a packet.
	 */
	return netif_is_multiqueue(dev) ? skb->queue_mapping : 0;
}

static const struct net_device_ops bcmgenet_netdev_ops = {
	.ndo_open = bcmgenet_open,
	.ndo_stop = bcmgenet_close,
	.ndo_start_xmit = bcmgenet_xmit,
	.ndo_select_queue = bcmgenet_select_queue,
	.ndo_tx_timeout = bcmgenet_timeout,
	.ndo_set_rx_mode = bcmgenet_set_rx_mode,
	.ndo_set_mac_address = bcmgenet_set_mac_addr,
	.ndo_do_ioctl = bcmgenet_ioctl,
	.ndo_set_features = bcmgenet_set_features,
};

/* Array of GENET hardware parameters/characteristics */
static struct bcmgenet_hw_params bcmgenet_hw_params[] = {
	[GENET_V1] = {
		.tx_queues = 0,
		.rx_queues = 0,
		.bds_cnt = 0,
		.bp_in_en_shift = 16,
		.bp_in_mask = 0xffff,
		.hfb_filter_cnt = 16,
		.qtag_mask = 0x1F,
		.hfb_offset = 0x1000,
		.rdma_offset = 0x2000,
		.tdma_offset = 0x3000,
		.words_per_bd = 2,
	},
	[GENET_V2] = {
		.tx_queues = 4,
		.rx_queues = 4,
		.bds_cnt = 32,
		.bp_in_en_shift = 16,
		.bp_in_mask = 0xffff,
		.hfb_filter_cnt = 16,
		.qtag_mask = 0x1F,
		.tbuf_offset = 0x0600,
		.hfb_offset = 0x1000,
		.hfb_reg_offset = 0x2000,
		.rdma_offset = 0x3000,
		.tdma_offset = 0x4000,
		.words_per_bd = 2,
		.flags = GENET_HAS_EXT,
	},
	[GENET_V3] = {
		.tx_queues = 4,
		.rx_queues = 4,
		.bds_cnt = 32,
		.bp_in_en_shift = 17,
		.bp_in_mask = 0x1ffff,
		.hfb_filter_cnt = 48,
		.qtag_mask = 0x3F,
		.tbuf_offset = 0x0600,
		.hfb_offset = 0x8000,
		.hfb_reg_offset = 0xfc00,
		.rdma_offset = 0x10000,
		.tdma_offset = 0x11000,
		.words_per_bd = 2,
		.flags = GENET_HAS_EXT | GENET_HAS_MDIO_INTR,
	},
	[GENET_V4] = {
		.tx_queues = 4,
		.rx_queues = 4,
		.bds_cnt = 32,
		.bp_in_en_shift = 17,
		.bp_in_mask = 0x1ffff,
		.hfb_filter_cnt = 48,
		.qtag_mask = 0x3F,
		.tbuf_offset = 0x0600,
		.hfb_offset = 0x8000,
		.hfb_reg_offset = 0xfc00,
		.rdma_offset = 0x2000,
		.tdma_offset = 0x4000,
		.words_per_bd = 3,
		.flags = GENET_HAS_40BITS | GENET_HAS_EXT | GENET_HAS_MDIO_INTR,
	},
};

/* Infer hardware parameters from the detected GENET version */
static void bcmgenet_set_hw_params(struct bcmgenet_priv *priv)
{
	struct bcmgenet_hw_params *params;

	if (GENET_IS_V4(priv)) {
		bcmgenet_dma_regs = bcmgenet_dma_regs_v3plus;
		genet_dma_ring_regs = genet_dma_ring_regs_v4;
		priv->version = GENET_V4;
	} else if (GENET_IS_V3(priv)) {
		bcmgenet_dma_regs = bcmgenet_dma_regs_v3plus;
		genet_dma_ring_regs = genet_dma_ring_regs_v123;
		priv->version = GENET_V3;
	} else if (GENET_IS_V2(priv)) {
		bcmgenet_dma_regs = bcmgenet_dma_regs_v2;
		genet_dma_ring_regs = genet_dma_ring_regs_v123;
		priv->version = GENET_V2;
	} else if (GENET_IS_V1(priv)) {
		bcmgenet_dma_regs = bcmgenet_dma_regs_v1;
		genet_dma_ring_regs = genet_dma_ring_regs_v123;
		priv->version = GENET_V1;
	}

	/* enum genet_version starts at 1 */
	priv->hw_params = &bcmgenet_hw_params[priv->version];
	params = priv->hw_params;

	pr_info("Configuration for version: %d\n"
		"TXq: %1d, RXq: %1d, BDs: %1d\n"
		"BP << en: %2d, BP msk: 0x%05x\n"
		"HFB count: %2d, QTAQ msk: 0x%05x\n"
		"TBUF: 0x%04x, HFB: 0x%04x, HFBreg: 0x%04x\n"
		"RDMA: 0x%05x, TDMA: 0x%05x\n"
		"Words/BD: %d\n",
		priv->version,
		params->tx_queues, params->rx_queues, params->bds_cnt,
		params->bp_in_en_shift, params->bp_in_mask,
		params->hfb_filter_cnt, params->qtag_mask,
		params->tbuf_offset, params->hfb_offset,
		params->hfb_reg_offset,
		params->rdma_offset, params->tdma_offset,
		params->words_per_bd);
}

static int bcmgenet_drv_probe(struct platform_device *pdev)
{
	int err = -EIO;
	struct device_node *dn = pdev->dev.of_node;
	struct bcmgenet_priv *priv;
	struct net_device *dev;
	const void *macaddr;
	u32 propval;

	dev = alloc_etherdev_mq(sizeof(*(priv)), GENET_MQ_CNT+1);
	if (dev == NULL) {
		dev_err(&pdev->dev, "can't allocate net device\n");
		err = -ENOMEM;
		goto err0;
	}
	priv = (struct bcmgenet_priv *)netdev_priv(dev);
	priv->irq0 = irq_of_parse_and_map(dn, 0);
	priv->irq1 = irq_of_parse_and_map(dn, 1);

	if (!priv->irq0 || !priv->irq1) {
		dev_err(&pdev->dev, "can't find IRQs\n");
		return -EINVAL;
	}

	macaddr = of_get_mac_address(dn);
	if (!macaddr) {
		dev_err(&pdev->dev, "can't find MAC address\n");
		return -EINVAL;
	}

	priv->base = of_iomap(dn, 0);
	if (!priv->base) {
		dev_err(&pdev->dev, "can't ioremap\n");
		return -EINVAL;
	}

	dev->base_addr = (unsigned long)priv->base;
	SET_NETDEV_DEV(dev, &pdev->dev);
	dev_set_drvdata(&pdev->dev, priv);
	memcpy(dev->dev_addr, macaddr, ETH_ALEN);
	dev->irq = priv->irq0;
	dev->watchdog_timeo         = 2*HZ;
	SET_ETHTOOL_OPS(dev, &bcmgenet_ethtool_ops);
	dev->netdev_ops = &bcmgenet_netdev_ops;
	netif_napi_add(dev, &priv->napi, bcmgenet_poll, 64);

	priv->msg_enable = netif_msg_init(debug, GENET_MSG_DEFAULT);

	/* Set hardware features */
	dev->hw_features |= NETIF_F_SG | NETIF_F_IP_CSUM |
		NETIF_F_IPV6_CSUM | NETIF_F_RXCSUM;

	netdev_boot_setup_check(dev);

	priv->dev = dev;

#ifdef CONFIG_GENET_RUNTIME_DETECT
	if (of_device_is_compatible(dn, "brcm,genet-v4"))
		priv->version = GENET_V4;
	else if (of_device_is_compatible(dn, "brcm,genet-v3"))
		priv->version = GENET_V3;
	else if (of_device_is_compatible(dn, "brcm,genet-v2"))
		priv->version = GENET_V2;
	else if (of_device_is_compatible(dn, "brcm,genet-v1"))
		priv->version = GENET_V1;
	else {
		dev_err(&pdev->dev, "unknown GENET version\n");
		return -EINVAL;
	}
#endif

	bcmgenet_set_hw_params(priv);

	spin_lock_init(&priv->lock);
	spin_lock_init(&priv->bh_lock);
	mutex_init(&priv->mdio_mutex);
	/* Mii wait queue */
	init_waitqueue_head(&priv->wq);

	priv->phy_type = BRCM_PHY_TYPE_INT;
	priv->phy_addr = 10;
	priv->phy_speed = SPEED_1000;

	if (!of_property_read_u32(dn, "phy-type", &propval))
		priv->phy_type = propval;
	if (!of_property_read_u32(dn, "phy-id", &propval))
		priv->phy_addr = propval;
	if (!of_property_read_u32(dn, "phy-speed", &propval))
		priv->phy_speed = propval;

	priv->ext_phy = priv->phy_type != BRCM_PHY_TYPE_INT;
	priv->pdev = pdev;

	/* Init GENET registers, Tx/Rx buffers */
	err = bcmgenet_init_dev(priv);
	if (err)
		goto err1;

	if (!of_property_read_u32(dn, "ethsw-type", &propval)) {
		/* one-shot initialization; never poll for link status */
		priv->sw_type = propval;

		priv->sw_addr = priv->phy_addr;
		priv->phy_addr = BRCM_PHY_ID_NONE;

		bcmgenet_mii_init(dev);
		bcmgenet_ethsw_init(dev);
	} else
		bcmgenet_mii_init(dev);

	INIT_WORK(&priv->bcmgenet_irq_work, bcmgenet_irq_task);

	err = register_netdev(dev);
	if (err != 0)
		goto err2;

	if (priv->ext_phy) {
		/* No Link status IRQ */
		INIT_WORK(&priv->bcmgenet_link_work,
				bcmgenet_gphy_link_status);
		init_timer(&priv->timer);
		priv->timer.data = (unsigned long)priv;
		priv->timer.function = bcmgenet_gphy_link_timer;
	} else {
		/* check link status */
		bcmgenet_mii_setup(dev);
	}

	netif_carrier_off(priv->dev);
	bcmgenet_clock_disable(priv);

	return 0;

err2:
	bcmgenet_clock_disable(priv);
	bcmgenet_uninit_dev(priv);
err1:
	iounmap(priv->base);
	free_netdev(dev);
err0:
	return err;
}

static int bcmgenet_drv_remove(struct platform_device *pdev)
{
	struct bcmgenet_priv *priv = dev_get_drvdata(&pdev->dev);

	unregister_netdev(priv->dev);
	free_irq(priv->irq0, priv);
	free_irq(priv->irq1, priv);
	bcmgenet_uninit_dev(priv);
	iounmap(priv->base);
	free_netdev(priv->dev);
	return 0;
}

static int bcmgenet_drv_suspend(struct device *dev)
{
	int val = 0;
	struct bcmgenet_priv *priv = dev_get_drvdata(dev);

	cancel_work_sync(&priv->bcmgenet_irq_work);

	/* Save/restore the interface status across PM modes.
	 * FIXME: Don't use open/close for suspend/resume.
	 */
	priv->dev_opened = netif_running(priv->dev);
	if (priv->dev_opened && !priv->dev_asleep) {
		priv->dev_asleep = 1;
		val = bcmgenet_close(priv->dev);
	}

	return val;
}

static int bcmgenet_drv_resume(struct device *dev)
{
	int val = 0;
	struct bcmgenet_priv *priv = dev_get_drvdata(dev);

	if (priv->dev_opened)
		val = bcmgenet_open(priv->dev);
	priv->dev_asleep = 0;

	return val;
}

static const struct dev_pm_ops bcmgenet_pm_ops = {
	.suspend		= bcmgenet_drv_suspend,
	.resume			= bcmgenet_drv_resume,
};

static const struct of_device_id bcmgenet_match[] = {
	{ .compatible = "brcm,genet-v1", },
	{ .compatible = "brcm,genet-v2", },
	{ .compatible = "brcm,genet-v3", },
	{ .compatible = "brcm,genet-v4", },
	{ },
};

static struct platform_driver bcmgenet_plat_drv = {
	.probe =		bcmgenet_drv_probe,
	.remove =		bcmgenet_drv_remove,
	.driver = {
		.name =		"bcmgenet",
		.owner =	THIS_MODULE,
		.pm =		&bcmgenet_pm_ops,
		.of_match_table = bcmgenet_match,
	},
};

static int bcmgenet_module_init(void)
{
	platform_driver_register(&bcmgenet_plat_drv);
	return 0;
}

static void bcmgenet_module_cleanup(void)
{
	platform_driver_unregister(&bcmgenet_plat_drv);
}

module_init(bcmgenet_module_init);
module_exit(bcmgenet_module_cleanup);
MODULE_LICENSE("GPL");