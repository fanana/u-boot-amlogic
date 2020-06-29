// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 */

#include <common.h>
#include <dm.h>
#include <env.h>
#include <env_internal.h>
#include <init.h>
#include <net.h>
#include <asm/io.h>
#include <asm/arch/axg.h>
#include <asm/arch/sm.h>
#include <asm/arch/eth.h>
#include <asm/arch/mem.h>
#include <u-boot/crc.h>

#define MAC_ADDR_LEN		6
#define EFUSE_MAC_SIZE		12

int misc_init_r(void)
{
	u8 mac_addr[MAC_ADDR_LEN];
	char serial_string[EFUSE_MAC_SIZE];

	meson_generate_serial_ethaddr();
	eth_env_get_enetaddr("ethaddr", mac_addr);

	if (!env_get("serial#")) {
		sprintf(serial_string, "%02X%02X%02X%02X%02X%02X", mac_addr[0], mac_addr[1], mac_addr[2],mac_addr[3], mac_addr[4], mac_addr[5]);
		env_set("serial#", serial_string);
	}

	return 0;
}
