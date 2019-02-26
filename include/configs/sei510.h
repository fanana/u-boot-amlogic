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

#define ROOT_UUID "ddb8c3f6-d94d-4394-b633-3134139cc2e0;"
#define PARTS_DEFAULT \
	"uuid_disk=${uuid_gpt_disk};" \
	"name=boot,size=128M,bootable,uuid=${uuid_gpt_boot};" \
	"name=rootfs,size=-,uuid="ROOT_UUID

#define BOOTENV_DEV_SYSTEM(devtypeu, devtypel, instance) \
	"bootcmd_system=" \
		"if run check_recovery; then " \
			"fastboot 0;" \
		"else " \
			"mmc dev ${mmcdev};" \
			"setenv bootargs ${bootargs} console=${console};" \
			"part start mmc ${mmcdev} ${bootpart} boot_start; " \
			"part size mmc ${mmcdev} ${bootpart} boot_size; " \
			"mmc read ${loadaddr} ${boot_start} ${boot_size}; " \
			"bootm ${loadaddr};" \
			"reset;" \
		"fi\0"

#define BOOTENV_DEV_NAME_SYSTEM(devtypeu, devtypel, instance)	\
		"system "

#define BOOT_TARGET_DEVICES(func) \
	func(ROMUSB, romusb, na)  \
	func(SYSTEM, system, na) \
	func(MMC, mmc, 0) \
	BOOT_TARGET_DEVICES_USB(func) \
	func(PXE, pxe, na) \
	func(DHCP, dhcp, na) \

#define CONFIG_EXTRA_ENV_SETTINGS \
	"partitions=" PARTS_DEFAULT "\0" \
	"mmcdev=1\0" \
	"bootpart=1\0" \
	"gpio_recovery=88\0" \
	"check_recovery=gpio input ${gpio_recovery};test $? -eq 0;\0" \
	"console=/dev/ttyAML0\0" \
	"bootargs=no_console_suspend\0" \
	"stdin=" STDIN_CFG "\0" \
	"stdout=" STDOUT_CFG "\0" \
	"stderr=" STDOUT_CFG "\0" \
	"loadaddr=0x6000000\0" \
	"fdt_addr_r=0x08008000\0" \
	"scriptaddr=0x08000000\0" \
	"kernel_addr_r=0x08080000\0" \
	"pxefile_addr_r=0x01080000\0" \
	"ramdisk_addr_r=0x13000000\0" \
	"fdtfile=amlogic/" CONFIG_DEFAULT_DEVICE_TREE ".dtb\0" \
	BOOTENV

#include <configs/meson64.h>

#endif /* __CONFIG_H */
