/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration for Android Amlogic Meson 64bits SoCs
 *
 * Copyright (C) 2019 Baylibre, SAS
 * Author: Jerome Brunet <jbrunet@baylibre.com>
 */

#ifndef __MESON64_ANDROID_CONFIG_H
#define __MESON64_ANDROID_CONFIG_H

#define CONFIG_SYS_MMC_ENV_DEV	2
#define CONFIG_SYS_MMC_ENV_PART	1

#ifndef CONTROL_PARTITION
#define CONTROL_PARTITION "misc"
#endif

#ifndef BOOT_PARTITION
#define BOOT_PARTITION "boot"
#endif

#if defined(CONFIG_CMD_BCB)
#define ANDROIDBOOT_FASTBOOTD_CMD "androidboot_fastbootd=" \
	"if bcb load " __stringify(CONFIG_FASTBOOT_FLASH_MMC_DEV) " " \
		CONTROL_PARTITION "; then " \
			"bcb set recovery recovery:--fastboot:;" \
			"bcb set command boot-recovery;" \
			"bcb store;" \
	"else " \
		"echo Warning: BCB is corrupted or does not exist; " \
	"fi;\0"
#define ANDROIDBOOT_RECOVERY_CMD "androidboot_recovery=" \
	"if bcb load " __stringify(CONFIG_FASTBOOT_FLASH_MMC_DEV) " " \
		CONTROL_PARTITION "; then " \
			"bcb set recovery recovery:--recovery:;" \
			"bcb set command boot-recovery;" \
			"bcb store;" \
	"else " \
		"echo Warning: BCB is corrupted or does not exist; " \
	"fi;\0"
#else
#define ANDROIDBOOT_FASTBOOTD_CMD ""
#define ANDROIDBOOT_RECOVERY_CMD ""
#endif

#if defined(CONFIG_CMD_AVB)
#define AVB_VERIFY_CHECK "if test \"${force_avb}\" -eq 1; then " \
				"if run avb_verify; then " \
					"echo AVB verification OK.;" \
					"setenv bootargs \"$bootargs $avb_bootargs\";" \
				"else " \
					"echo AVB verification failed.;" \
				"exit; fi;" \
			"else " \
				"echo Running without AVB...; "\
			"fi;"

#define AVB_VERIFY_CMD "avb_verify=avb init ${mmcdev}; avb verify;\0"
#else
#define AVB_VERIFY_CHECK ""
#define AVB_VERIFY_CMD ""
#endif

 #define BOOTENV_DEV_FASTBOOT(devtypeu, devtypel, instance) \
	"bootcmd_fastboot=" \
		"setenv run_fastboot 0;" \
		"sm reboot_reason reason;" \
		"if test \"${boot_source}\" = \"usb\"; then " \
			"echo Fastboot forced by usb rom boot;" \
			"setenv run_fastboot 1;" \
		"fi;" \
		"if gpt verify mmc ${mmcdev} ${partitions}; then; " \
		"else " \
			"echo Broken MMC partition scheme;" \
			"setenv run_fastboot 1;" \
		"fi;" \
		"if test \"${reason}\" = fastboot; then " \
			"echo Fastboot userspace asked by reboot reason;" \
			"setenv force_recovery 1;" \
			"if run androidboot_fastbootd; then " \
				"echo BCB set OK.;" \
			"else " \
				"echo BCB set failed.;" \
			"exit; fi;" \
			"run bootcmd_recovery;" \
		"fi;" \
		"if test \"${reason}\" = bootloader; then " \
			"echo Fastboot asked by reboot reason;" \
			"setenv run_fastboot 1;" \
		"fi;" \
		"if bcb load " __stringify(CONFIG_FASTBOOT_FLASH_MMC_DEV) " " \
		CONTROL_PARTITION "; then " \
			"if bcb test command = bootonce-bootloader; then " \
				"echo BCB: Bootloader boot...; " \
				"bcb clear command; bcb store; " \
				"setenv run_fastboot 1;" \
			"fi; " \
		"else " \
			"echo Warning: BCB is corrupted or does not exist; " \
		"fi;" \
		"if test \"${run_fastboot}\" -eq 1; then " \
			"fastboot " __stringify(CONFIG_FASTBOOT_USB_DEV) "; " \
		"fi;\0"

#define BOOTENV_DEV_NAME_FASTBOOT(devtypeu, devtypel, instance)	\
		"fastboot "

#define BOOTENV_DEV_RECOVERY(devtypeu, devtypel, instance) \
	"bootcmd_recovery=" \
		"pinmux dev pinctrl@14;" \
		"pinmux dev pinctrl@40;" \
		"setenv run_recovery 0;" \
		"sm reboot_reason reason;" \
		"if run check_button; then " \
			"echo Recovery button is pressed;" \
			"setenv run_recovery 1;" \
		"fi; " \
		"if bcb load " __stringify(CONFIG_FASTBOOT_FLASH_MMC_DEV) " " \
		CONTROL_PARTITION "; then " \
			"if bcb test command = boot-recovery; then " \
				"echo BCB: Recovery boot...; " \
				"setenv run_recovery 1;" \
			"fi;" \
		"else " \
			"if test \"${reason}\" = \"bootloader\" -o " \
				"\"${reason}\" = \"recovery\"; then " \
				"echo Recovery asked by reboot reason;" \
				"setenv run_recovery 1;" \
			"fi;" \
		"fi;" \
		"if test \"${skip_recovery}\" -eq 1; then " \
			"echo Recovery skipped by environment;" \
			"setenv run_recovery 0;" \
		"fi;" \
		"if test \"${force_recovery}\" -eq 1; then " \
			"echo Recovery forced by environment;" \
			"setenv run_recovery 1;" \
		"fi;" \
		"if test \"${run_recovery}\" -eq 1; then " \
			"echo Running Recovery...;" \
			"mmc dev ${mmcdev};" \
			"setenv bootargs \"${bootargs} androidboot.serialno=${serial#}\";" \
			"part start mmc ${mmcdev} recovery boot_start;" \
			"part size mmc ${mmcdev} recovery boot_size;" \
			AVB_VERIFY_CHECK \
			"if mmc read ${loadaddr} ${boot_start} ${boot_size}; then " \
				"echo Running Android Recovery...;" \
				"bootm ${loadaddr};" \
			"fi;" \
			"echo Failed to boot Android...;" \
			"reset;" \
		"fi;\0"

#define BOOTENV_DEV_NAME_RECOVERY(devtypeu, devtypel, instance)	\
		"recovery "

#define BOOTENV_DEV_SYSTEM(devtypeu, devtypel, instance) \
	"bootcmd_system=" \
		"echo Loading Android " BOOT_PARTITION " partition...;" \
		"mmc dev ${mmcdev};" \
		"setenv bootargs \"${bootargs} androidboot.serialno=${serial#}\";" \
		"part start mmc ${mmcdev} " BOOT_PARTITION " boot_start;" \
		"part size mmc ${mmcdev} " BOOT_PARTITION " boot_size;" \
		AVB_VERIFY_CHECK \
		"if mmc read ${loadaddr} ${boot_start} ${boot_size}; then " \
			"echo Running Android...;" \
			"bootm ${loadaddr};" \
		"fi;" \
		"echo Failed to boot Android...;" \
		"reset\0"

#define BOOTENV_DEV_NAME_SYSTEM(devtypeu, devtypel, instance)	\
		"system "

#define BOOT_TARGET_DEVICES(func) \
	func(FASTBOOT, fastboot, na) \
	func(RECOVERY, recovery, na) \
	func(SYSTEM, system, na) \

#define PREBOOT_LOAD_LOGO \
	"if test \"${boot_source}\" != \"usb\" && " \
		"gpt verify mmc ${mmcdev} ${partitions}; then; " \
		"mmc dev ${mmcdev};" \
		"part start mmc ${mmcdev} ${logopart} boot_start;" \
		"part size mmc ${mmcdev} ${logopart} boot_size;" \
		"if mmc read ${loadaddr} ${boot_start} ${boot_size}; then " \
			"bmp display ${loadaddr} m m;" \
		"fi;" \
	"fi;"

#define CONFIG_EXTRA_ENV_SETTINGS                                     \
	"partitions=" PARTS_DEFAULT "\0"                              \
	AVB_VERIFY_CMD                                                \
	ANDROIDBOOT_FASTBOOTD_CMD                                     \
	ANDROIDBOOT_RECOVERY_CMD                                      \
	"mmcdev=2\0"                                                  \
	"logopart=1\0"                                                \
	"force_avb=0\0"                                               \
	"gpio_recovery=88\0"                                          \
	"check_button=gpio input ${gpio_recovery};test $? -eq 0;\0"   \
	"load_logo=" PREBOOT_LOAD_LOGO "\0"			      \
	"console=ttyAML0\0"                                      \
	"stdin=" STDIN_CFG "\0"                                       \
	"stdout=" STDOUT_CFG "\0"                                     \
	"stderr=" STDOUT_CFG "\0"                                     \
	"loadaddr=0x01000000\0"                                       \
	"fdt_addr_r=0x01000000\0"                                     \
	"scriptaddr=0x08000000\0"                                     \
	"kernel_addr_r=0x01080000\0"                                  \
	"pxefile_addr_r=0x01080000\0"                                 \
	"ramdisk_addr_r=0x13000000\0"                                 \
	"fdtfile=amlogic/" CONFIG_DEFAULT_DEVICE_TREE ".dtb\0" BOOTENV

#include <configs/meson64.h>

#endif /* __MESON64_ANDROID_CONFIG_H */
