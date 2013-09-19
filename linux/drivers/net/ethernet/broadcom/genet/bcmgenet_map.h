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
*/
/* uniMAC register definations.*/

#ifndef __BCMGENET_MAP_H__
#define __BCMGENET_MAP_H__

#include <linux/types.h>
#include "bcmgenet_defs.h"

#ifndef __ASSEMBLY__

/* 64B status Block */
struct status_64 {
	u32	length_status;		/* length and peripheral status */
	u32	ext_status;		/* Extended status*/
	u32	rx_csum;		/* partial rx checksum */
#if CONFIG_BRCM_GENET_VERSION < 3
	u32	filter_index;		/* Filter index */
	u32	extracted_bytes[4];	/* Extracted byte 0 - 16 */
	u32	reserved[4];
#else /* GENET_V3+ */
	u32	filter_index[2];	/* Filter index */
	u32	extracted_bytes[4];	/* Extracted byte 0 - 16 */
	u32	reserved[3];
#endif
	u32	tx_csum_info;		/* Tx checksum info. */
	u32	unused[3];		/* unused */
} ;
/* Rx status bits */
#define STATUS_RX_EXT_MASK		0x1FFFFF
#define STATUS_RX_CSUM_MASK		0xFFFF
#define STATUS_RX_CSUM_OK		0x10000
#define STATUS_RX_CSUM_FR		0x20000
#define STATUS_RX_PROTO_TCP		0
#define STATUS_RX_PROTO_UDP		1
#define STATUS_RX_PROTO_ICMP	2
#define STATUS_RX_PROTO_OTHER	3
#define STATUS_RX_PROTO_MASK	3
#define STATUS_RX_PROTO_SHIFT	18
#define STATUS_FILTER_INDEX_MASK	0xFFFF
/* Tx status bits */
#define STATUS_TX_CSUM_START_MASK	0X7FFF
#define STATUS_TX_CSUM_START_SHIFT	16
#define STATUS_TX_CSUM_PROTO_UDP	0x8000
#define STATUS_TX_CSUM_OFFSET_MASK	0x7FFF
#define STATUS_TX_CSUM_LV			0x80000000

/*
** DMA Descriptor
*/
#define DMA_DESC_LENGTH_STATUS	0x00	/* in bytes of data in buffer */
#define DMA_DESC_ADDRESS_LO	0x04	/* lower bits of PA */
#define DMA_DESC_ADDRESS_HI	0x08	/* upper 32 bits of PA, GENETv4+ */

/*
** UniMAC TSV or RSV (Transmit Status Vector or Receive Status Vector
*/
/* Rx/Tx common counter group.*/
struct PktCounterSize {
	u32	cnt_64;		/* RO Recvied/Transmited 64 bytes packet */
	u32	cnt_127;	/* RO Rx/Tx 127 bytes packet */
	u32	cnt_255;	/* RO Rx/Tx 65-255 bytes packet */
	u32	cnt_511;	/* RO Rx/Tx 256-511 bytes packet */
	u32	cnt_1023;	/* RO Rx/Tx 512-1023 bytes packet */
	u32	cnt_1518;	/* RO Rx/Tx 1024-1518 bytes packet */
	u32	cnt_mgv;	/* RO Rx/Tx 1519-1522 good VLAN packet */
	u32	cnt_2047;	/* RO Rx/Tx 1522-2047 bytes packet*/
	u32	cnt_4095;	/* RO Rx/Tx 2048-4095 bytes packet*/
	u32	cnt_9216;	/* RO Rx/Tx 4096-9216 bytes packet*/
};
/* RSV, Receive Status Vector */
struct UniMacRSV {
	struct PktCounterSize stat_sz;	/* (0x400 - 0x424), stats of received
			packets classfied by size */
	u32	rx_pkt;		/* RO (0x428) Receive pkt count*/
	u32	rx_bytes;	/* RO Receive byte count */
	u32	rx_mca;		/* RO # of Received multicast pkt */
	u32	rx_bca;		/* RO # of Receive broadcast pkt */
	u32	rx_fcs;		/* RO # of Received FCS error  */
	u32	rx_cf;		/* RO # of Received control frame pkt*/
	u32	rx_pf;		/* RO # of Received pause frame pkt */
	u32	rx_uo;		/* RO # of unknown op code pkt */
	u32	rx_aln;		/* RO # of alignment error count */
	u32	rx_flr;		/* RO # of frame length out of range count */
	u32	rx_cde;		/* RO # of code error pkt */
	u32	rx_fcr;		/* RO # of carrier sense error pkt */
	u32	rx_ovr;		/* RO # of oversize pkt*/
	u32	rx_jbr;		/* RO # of jabber count */
	u32	rx_mtue;	/* RO # of MTU error pkt*/
	u32	rx_pok;		/* RO # of Received good pkt */
	u32	rx_uc;		/* RO # of unicast pkt */
	u32	rx_ppp;		/* RO # of PPP pkt */
	u32	rcrc;		/* RO (0x470),# of CRC match pkt */
};

/* TSV, Transmit Status Vector */
struct UniMacTSV {
	struct PktCounterSize stat_sz;	/* (0x480 - 0x0x4a4), statistics of
			xmited packets classified by size */
	u32	tx_pkt;		/* RO (0x4a8) Transmited pkt */
	u32	tx_mca;		/* RO # of xmited multicast pkt */
	u32	tx_bca;		/* RO # of xmited broadcast pkt */
	u32	tx_pf;		/* RO # of xmited pause frame count */
	u32	tx_cf;		/* RO # of xmited control frame count */
	u32	tx_fcs;		/* RO # of xmited FCS error count */
	u32	tx_ovr;		/* RO # of xmited oversize pkt */
	u32	tx_drf;		/* RO # of xmited deferral pkt */
	u32	tx_edf;		/* RO # of xmited Excessive deferral pkt*/
	u32	tx_scl;		/* RO # of xmited single collision pkt */
	u32	tx_mcl;		/* RO # of xmited multiple collision pkt*/
	u32	tx_lcl;		/* RO # of xmited late collision pkt */
	u32	tx_ecl;		/* RO # of xmited excessive collision pkt*/
	u32	tx_frg;		/* RO # of xmited fragments pkt*/
	u32	tx_ncl;		/* RO # of xmited total collision count */
	u32	tx_jbr;		/* RO # of xmited jabber count*/
	u32	tx_bytes;	/* RO # of xmited byte count */
	u32	tx_pok;		/* RO # of xmited good pkt */
	u32	tx_uc;		/* RO (0x0x4f0)# of xmited unitcast pkt */
};

#define UMAC_HD_BKP_CTRL	0x004
#define UMAC_CMD		0x008
#define UMAC_MAC0		0x00C
#define UMAC_MAC1		0x010
#define UMAC_MAX_FRAME_LEN	0x014

#define UMAC_TX_FLUSH		0x334

#define UMAC_MDIO_CMD		0x614
#define UMAC_MPD_CTRL		0x620
#define UMAC_MPD_PW_MS		0x624
#define UMAC_MPD_PW_LS		0x628
#define UMAC_MDF_CTRL		0x650
#define UMAC_MDF_ADDR		0x654
#define UMAC_MIB_CTRL		0x580

#define RBUF_CTRL			0x00
#define RBUF_STATUS			0x0C
#define RBUF_CHK_CTRL			0x14
#define RBUF_TBUF_SIZE_CTRL		0xb4

#define RBUF_HFB_CTRL_V1		0x38
#define RBUF_HFB_LEN_V1			0x3C

#define TBUF_CTRL			0x00
#define TBUF_BP_MC			0x0C

#define TBUF_CTRL_V1			0x80
#define TBUF_BP_MC_V1			0xA0

#define HFB_CTRL			0x00
#define HFB_FLT_ENABLE_V3PLUS		0x04
#define HFB_FLT_LEN_V2			0x04
#define HFB_FLT_LEN_V3PLUS		0x1C

/* uniMac intrl2 registers */
#define INTRL2_CPU_STAT			0x00
#define INTRL2_CPU_SET			0x04
#define INTRL2_CPU_CLEAR		0x08
#define INTRL2_CPU_MASK_STATUS		0x0C
#define INTRL2_CPU_MASK_SET		0x10
#define INTRL2_CPU_MASK_CLEAR		0x14

/* Register block offset */
#define GENET_GR_BRIDGE_OFF			0x0040
#define GENET_EXT_OFF				0x0080
#define GENET_INTRL2_0_OFF			0x0200
#define GENET_INTRL2_1_OFF			0x0240
#define GENET_RBUF_OFF				0X0300
#define GENET_UMAC_OFF				0x0800

#define SYS_REV_CTRL		0x00
#define SYS_PORT_CTRL		0x04
#define SYS_RBUF_FLUSH_CTRL	0x08
#define SYS_TBUF_FLUSH_CTRL	0x0C
#define RBUF_FLUSH_CTRL_V1	0x04

#define EXT_EXT_PWR_MGMT	0x00
#define EXT_RGMII_OOB_CTRL	0x0C
#define EXT_GPHY_CTRL		0x1C

/*
 * DMA rings size
 */
#define DMA_RING_SIZE		(0x40)
#define DMA_RINGS_SIZE		(DMA_RING_SIZE * 17)

#endif /* __ASSEMBLY__ */

#endif
