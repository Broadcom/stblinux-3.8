/*
 *
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
*/
#ifndef __BCMGENET_H__
#define __BCMGENET_H__

#define CARDNAME				"bcmgenet"
#define VERSION     "2.0"
#define VER_STR     "v" VERSION " " __DATE__ " " __TIME__

#define pr_fmt(fmt)				CARDNAME ": " fmt

#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/mii.h>
#include <linux/if_vlan.h>

#include "bcmgenet_map.h"

#define BRCM_PHY_ID_AUTO	0x100
#define BRCM_PHY_ID_NONE	0x101

#define BRCM_PHY_TYPE_INT	1
#define BRCM_PHY_TYPE_EXT_MII	2
#define BRCM_PHY_TYPE_EXT_RVMII	3
#define BRCM_PHY_TYPE_EXT_RGMII	4
#define BRCM_PHY_TYPE_EXT_RGMII_IBS	5
#define BRCM_PHY_TYPE_EXT_RGMII_NO_ID	6
#define BRCM_PHY_TYPE_MOCA	7

/* total number of Buffer Descriptors, same for Rx/Tx */
#define TOTAL_DESC				256
/* which ring is descriptor based */
#define DESC_INDEX				16
/* Body(1500) + EH_SIZE(14) + VLANTAG(4) + BRCMTAG(6) + FCS(4) = 1528.
 * 1536 is multiple of 256 bytes
 */
#define ENET_BRCM_TAG_LEN	6
#define ENET_PAD		8
#define ENET_MAX_MTU_SIZE	(ETH_DATA_LEN + ETH_HLEN + VLAN_HLEN + \
				 ENET_BRCM_TAG_LEN + ETH_FCS_LEN + ENET_PAD)
#define DMA_MAX_BURST_LENGTH    0x10

#define GENET_TX_RING_COUNT		16
#define GENET_RX_RING_COUNT		16
#define GENET_ALLOC_TX_RING		1
#define GENET_ALLOC_RX_RING		0

/* misc. configuration */
#define CLEAR_ALL_HFB			0xFF
#define DMA_FC_THRESH_HI		(TOTAL_DESC >> 4)
#define DMA_FC_THRESH_LO		5


struct enet_cb {
	struct sk_buff      *skb;
	void __iomem *bd_addr;
	DEFINE_DMA_UNMAP_ADDR(dma_addr);
	DEFINE_DMA_UNMAP_LEN(dma_len);
};

/* power management mode */
#define GENET_POWER_CABLE_SENSE	0
#define GENET_POWER_WOL_MAGIC	1
#define GENET_POWER_WOL_ACPI	2
#define GENET_POWER_PASSIVE		3

struct bcmgenet_priv;

/* We support both runtime GENET detection and compile-time
 * to optimize code-paths for a given hardware
 */
enum bcmgenet_version {
	GENET_V1 = 1,
	GENET_V2,
	GENET_V3,
	GENET_V4
};

enum bcmgenet_version __genet_get_version(struct bcmgenet_priv *priv);

#ifdef CONFIG_BRCM_GENET_V1
# ifdef genet_get_version
#  undef genet_get_version
#  define genet_get_version(p)	__genet_get_version(p)
#  define CONFIG_GENET_RUNTIME_DETECT
# else
#  define genet_get_version(p)	GENET_V1
# endif
# define GENET_IS_V1(p)		(genet_get_version(p) == GENET_V1)
#else
# define GENET_IS_V1(p)		(0)
#endif

#ifdef CONFIG_BRCM_GENET_V2
# ifdef genet_get_version
#  undef genet_get_version
#  define genet_get_version(p)	__genet_get_version(p)
#  define CONFIG_GENET_RUNTIME_DETECT
# else
#  define genet_get_version(p)	GENET_V2
# endif
# define GENET_IS_V2(p)		(genet_get_version(p) == GENET_V2)
#else
# define GENET_IS_V2(p)		(0)
#endif

#ifdef CONFIG_BRCM_GENET_V3
# ifdef genet_get_version
#  undef genet_get_version
#  define genet_get_version(p)	__genet_get_version(p)
#  define CONFIG_GENET_RUNTIME_DETECT
# else
#  define genet_get_version(p)	GENET_V3
# endif
# define GENET_IS_V3(p)		(genet_get_version(p) == GENET_V3)
#else
# define GENET_IS_V3(p)		(0)
#endif

#ifdef CONFIG_BRCM_GENET_V4
# ifdef genet_get_version
#  undef genet_get_version
#  define genet_get_version(p)	__genet_get_version(p)
#  define CONFIG_GENET_RUNTIME_DETECT
# else
#  define genet_get_version(p)	GENET_V4
# endif
# define GENET_IS_V4(p)		(genet_get_version(p) == GENET_V4)
#else
# define GENET_IS_V4(p)		(0)
#endif

#ifndef genet_get_version
#error "No GENET configuration defined"
#endif

/* Hardware flags */
#define GENET_HAS_40BITS	(1 << 0)
#define GENET_HAS_EXT		(1 << 1)
#define GENET_HAS_MDIO_INTR	(1 << 2)

/* BCMGENET hardware parameters, keep this structure nicely aligned
 * since it is going to be used in hot paths */
struct bcmgenet_hw_params {
	u8		tx_queues;
	u8		rx_queues;
	u8		bds_cnt;
	u8		bp_in_en_shift;
	u32		bp_in_mask;
	u8		hfb_filter_cnt;
	u8		qtag_mask;
	u16		tbuf_offset;
	u32		hfb_offset;
	u32		hfb_reg_offset;
	u32		rdma_offset;
	u32		tdma_offset;
	u32		words_per_bd;
	u32		flags;
};

/* device context */
struct bcmgenet_priv {
	void __iomem *base;
	enum bcmgenet_version version;
	struct net_device *dev;	/* ptr to net_device */
	spinlock_t      lock;		/* Serializing lock */
	spinlock_t		bh_lock;	/* soft IRQ lock */

	/* NAPI for descriptor based rx */
	struct napi_struct napi ____cacheline_aligned;

	/* transmit variables */
	void __iomem *tx_bds;	/* location of tx Dma BD ring */
	struct enet_cb *tx_cbs;	/* locaation of tx control block pool */
	int	num_tx_bds;		/* number of transmit bds */

	struct enet_cb *tx_ring_cbs[17]; /* tx ring buffer control block*/
	unsigned int tx_ring_size[17];	/* size of each tx ring */
	unsigned int tx_ring_c_index[17]; /* last consumer index of each ring*/
	int tx_ring_free_bds[17];	/* # of free bds for each ring */
	unsigned int tx_ring_write_ptr[17]; /* Tx ring write pointer SW copy */
	unsigned int tx_ring_prod_index[17];

	/* receive variables */
	void __iomem *rx_bds;	/* location of rx bd ring */
	void __iomem *rx_bd_assign_ptr;	/*next rx bd to assign buffer*/
	int rx_bd_assign_index;	/* next rx bd to assign, index from rx_bds */
	struct enet_cb *rx_cbs;	/* location of rx control block pool */
	int	num_rx_bds;	/* number of receive bds */
	int	rx_buf_len;	/* size of rx buffers for DMA */
	unsigned int rx_read_ptr; /* Rx read pointer SW copy */
	unsigned int rx_c_index;  /* Rx consumer index SW copy */

	/* other misc variables */
	struct bcmgenet_hw_params *hw_params;
	struct mii_if_info mii;		/* mii interface info */
	struct mutex mdio_mutex;	/* mutex for mii_read/write */
	wait_queue_head_t	wq;		/* mii wait queue */
	struct timer_list timer;	/* Link status timer */
	int irq0;	/* regular IRQ */
	int	irq1;	/* ring buf IRQ */
	int	phy_addr;
	int	phy_type;
	int	phy_speed;
	int	ext_phy;
	int	sw_addr;
	int	sw_type;
	unsigned int irq0_stat;	/* sw copy of irq0 status, for IRQ BH */
	unsigned int irq1_stat;	/* sw copy of irq1 status, for NAPI rx */
	unsigned int desc_64b_en;
	unsigned int desc_rxchk_en;
	unsigned int crc_fwd_en;
	u32 msg_enable;

	struct work_struct bcmgenet_irq_work;	/* bottom half work */
	struct work_struct bcmgenet_link_work;	/* GPHY link status work */
	struct clk *clk;
	int dev_opened;		/* device opened. */
	int dev_asleep;		/* device is at sleep */
	struct platform_device *pdev;

	/* WOL */
	unsigned long	wol_enabled;
	struct	clk *clk_wol;
	int	clock_active;
	u32	wolopts;

	/* S3 warm boot */
	u32 *saved_rx_desc;
	u32 int0_mask;
	u32 int1_mask;
	u32 rbuf_ctrl;
};

/* Extension registers accessors */
static inline u32 bcmgenet_ext_readl(struct bcmgenet_priv *priv, u32 off)
{
	return __raw_readl(priv->base + GENET_EXT_OFF + off);
}

static inline void bcmgenet_ext_writel(struct bcmgenet_priv *priv,
					u32 val, u32 off)
{
	__raw_writel(val, priv->base + GENET_EXT_OFF + off);
}

/* UniMAC register accessors */
static inline u32 bcmgenet_umac_readl(struct bcmgenet_priv *priv, u32 off)
{
	return __raw_readl(priv->base + GENET_UMAC_OFF + off);
}

static inline void bcmgenet_umac_writel(struct bcmgenet_priv *priv,
					u32 val, u32 off)
{
	__raw_writel(val, priv->base + GENET_UMAC_OFF + off);
}

/* System registers */
static inline u32 bcmgenet_sys_readl(struct bcmgenet_priv *priv, u32 off)
{
	return __raw_readl(priv->base + off);
}

static inline void bcmgenet_sys_writel(struct bcmgenet_priv *priv,
					u32 val, u32 off)
{
	__raw_writel(val, priv->base + off);
}

#endif /* __BCMGENET_H__ */
