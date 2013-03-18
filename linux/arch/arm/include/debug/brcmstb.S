/*
 * Copyright (c) 2013 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/brcmstb/brcmstb.h>

#define UART_SHIFT 2

		.macro	addruart, rp, rv, tmp
		ldr	\rv, =BVIRTADDR(BCHP_UARTA_REG_START)
		ldr	\rp, =BPHYSADDR(BCHP_UARTA_REG_START)
		.endm

#include "8250_32.S"
