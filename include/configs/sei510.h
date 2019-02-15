/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration for the SEI510
 *
 * Copyright (C) 2019 Baylibre, SAS
 * Author: Jerome Brunet <jbrunet@baylibre.com>
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#define CONFIG_SYS_MMC_ENV_DEV	1
#define CONFIG_SYS_MMC_ENV_PART	1
#define CONFIG_ENV_SIZE		0x10000
#define CONFIG_ENV_OFFSET	(-0x10000)

#define BOOT_TARGET_DEVICES(func) \
	func(ROMUSB, romusb, na)  \
	func(MMC, mmc, 0) \
	BOOT_TARGET_DEVICES_USB(func)

#include <configs/meson64.h>

#endif /* __CONFIG_H */
