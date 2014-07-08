/*
 * Always ON (AON) register interface between bootloader and Linux
 *
 * Copyright © 2014 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __BRCMSTB_AON_DEFS_H__
#define __BRCMSTB_AON_DEFS_H__

#include <linux/compiler.h>

/* Magic number in upper 16-bits */
#define BRCMSTB_S3_MAGIC_MASK                   0xffff0000
#define BRCMSTB_S3_MAGIC_SHORT                  0x5AFE0000

enum {
	/* Restore random key for AES memory verification (off = fixed key) */
	S3_FLAG_LOAD_RANDKEY		= (1 << 0),
};

#define BRCMSTB_HASH_LEN		(128 / 8) /* 128-bit hash */

#define AON_REG_MAGIC_FLAGS			0x00
#define AON_REG_CONTROL_LOW			0x04
#define AON_REG_CONTROL_HIGH			0x08
#define AON_REG_S3_HASH				0x0c /* hash of S3 params */
#define AON_REG_CONTROL_HASH_LEN		0x1c

#define BRCMSTB_S3_MAGIC		0x5AFEB007
#define BOOTLOADER_SCRATCH_SIZE		64
#define IMAGE_DESCRIPTORS_BUFSIZE	(256 * 1024)

struct brcmstb_s3_params {
	/* scratch memory for bootloader */
	uint8_t scratch[BOOTLOADER_SCRATCH_SIZE];

	uint32_t magic; /* BRCMSTB_S3_MAGIC */
	uint64_t reentry; /* PA */

	/* descriptors */
	uint32_t hash[BRCMSTB_HASH_LEN / 4];

	/*
	 * If 0, then ignore this parameter (there is only one set of
	 *   descriptors)
	 *
	 * If non-0, then a second set of descriptors is stored at:
	 *
	 *   descriptors + desc_offset_2
	 *
	 * The MAC result of both descriptors is XOR'd and stored in @hash
	 */
	uint32_t desc_offset_2;

	uint32_t spare[72];

	uint8_t descriptors[IMAGE_DESCRIPTORS_BUFSIZE];
} __packed;

#endif /* __BRCMSTB_AON_DEFS_H__ */
