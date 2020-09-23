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


int misc_init_r(void)
{
	char chip_serial[16];
	char serial_string[12];
	u8 serial[6];
	u32 sid;
	u16 sid16;

	meson_eth_init(PHY_INTERFACE_MODE_RMII,
		       MESON_USE_INTERNAL_RMII_PHY);

	meson_generate_serial_ethaddr();

	if (!env_get("serial#")) {
		if (!meson_sm_get_serial(chip_serial, SM_SERIAL_SIZE)) {
			sid = crc32(0, (unsigned char *)chip_serial, SM_SERIAL_SIZE);
			sid16 = crc16_ccitt(0, (unsigned char *)chip_serial,	SM_SERIAL_SIZE);

			/* Ensure the NIC specific bytes of the mac are not all 0 */
			if ((sid & 0xffffff) == 0)
				sid |= 0x800000;

			/* Non OUI  */
			serial[0] = ((sid16 >> 8) & 0xfc) | 0x02;
			serial[1] = (sid16 >>  0) & 0xff;
			serial[2] = (sid >> 24) & 0xff;
			serial[3] = (sid >> 16) & 0xff;
			serial[4] = (sid >>  8) & 0xff;
			serial[5] = (sid >>  0) & 0xff;
			sprintf(serial_string, "%02X%02X%02X%02X%02X%02X", serial[0], serial[1], serial[2],serial[3], serial[4], serial[5]);
			env_set("serial#", serial_string);

		} else
			return -EINVAL;
	}

	return 0;
}
