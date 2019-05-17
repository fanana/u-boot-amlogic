// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 */

#include <common.h>
#include <dm.h>
#include <environment.h>
#include <asm/io.h>
#include <asm/arch/axg.h>
#include <asm/arch/sm.h>
#include <asm/arch/eth.h>
#include <asm/arch/mem.h>

#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_CUSTOM) && CONFIG_IS_ENABLED(MMC_WRITE)

#include <malloc.h>
#include <blk.h>
#include <fastboot.h>
#include <fastboot-internal.h>
#include <fb_board.h>
#include <image-sparse.h>
#include <mmc.h>
#include <div64.h>
#include <linux/compat.h>

#define FASBOOT_HWPART_MMC_ENV_DEV		CONFIG_SYS_MMC_ENV_DEV
#define FASBOOT_HWPART_MMC_ENV_PART		CONFIG_SYS_MMC_ENV_PART
#define FASBOOT_HWPART_BOOTLOADER_OFFSET 	0x200	/* Block 1 */
#define FASBOOT_HWPART_BOOTLOADER_LENGTH 	0xffe00	/* 1MiB */
#define FASBOOT_HWPART_BOOTENV_OFFSET		CONFIG_ENV_OFFSET
#define FASBOOT_HWPART_BOOTENV_LENGTH		CONFIG_ENV_SIZE

#define FASTBOOT_MAX_BLK_WRITE 16384

static bool fastboot_board_hwpart_process(const char *cmd, void *buffer,
					  u32 bytes, char *response)
{
	struct blk_desc *desc;
	void *blk_ptr = NULL;
	int offset, length;
	struct mmc *mmc;
	lbaint_t blkcnt;
	lbaint_t blk;
	lbaint_t blks_written;
	lbaint_t cur_blkcnt;
	lbaint_t blks = 0;
	int ret, i;

	/* Check if we handle these partitions */
	if (strcmp(cmd, "bootloader") == 0) {
		offset = FASBOOT_HWPART_BOOTLOADER_OFFSET;
		length = FASBOOT_HWPART_BOOTLOADER_LENGTH;
	} else if (strcmp(cmd, "bootenv") == 0) {
		offset = FASBOOT_HWPART_BOOTENV_OFFSET;
		length = FASBOOT_HWPART_BOOTENV_LENGTH;
	} else
		return false;

	if (buffer && is_sparse_image(buffer)) {
		pr_err("sparse buffer not supported");
		fastboot_fail("sparse buffer not supported", response);
		return true;
	}

	mmc = find_mmc_device(FASBOOT_HWPART_MMC_ENV_DEV);
	if (!mmc) {
		pr_err("invalid mmc device\n");
		fastboot_fail("invalid mmc device", response);
		return true;
	}

	ret = mmc_switch_part(mmc, FASBOOT_HWPART_MMC_ENV_PART);
	if (ret) {
		pr_err("invalid mmc hwpart\n");
		fastboot_fail("invalid mmc hwpart", response);
		return true;
	}

	desc = mmc_get_blk_desc(mmc);

	if (offset < 0)
		offset += mmc->capacity;

	printf("using custom hwpart %s at offset 0x%x and length %d bytes\n",
	       cmd, offset, length);

	/* Use full length in erase mode */
	if (!buffer)
		bytes = length;

	/* determine number of blocks to write */
	blkcnt = ((bytes + (desc->blksz - 1)) & ~(desc->blksz - 1));
	blkcnt = lldiv(blkcnt, desc->blksz);

	if ((blkcnt * desc->blksz) > length) {
		pr_err("too large for partition: '%s'\n", cmd);
		fastboot_fail("too large for partition", response);
		return true;
	}

	blk = offset / desc->blksz;

	printf("writing " LBAFU " blocks starting at %ld...\n", blkcnt, blk);

	for (i = 0; i < blkcnt; i += FASTBOOT_MAX_BLK_WRITE) {
		cur_blkcnt = min((int)blkcnt - i, FASTBOOT_MAX_BLK_WRITE);
		if (buffer) {
			if (fastboot_progress_callback)
				fastboot_progress_callback("writing");
			blks_written = blk_dwrite(desc, blk, cur_blkcnt,
						  buffer + (i * desc->blksz));
		} else if ((blk % mmc->erase_grp_size) == 0 &&
			   (cur_blkcnt % mmc->erase_grp_size) == 0) {
			/* Only call erase if multiple of erase group */
			if (fastboot_progress_callback)
				fastboot_progress_callback("erasing");
			blks_written = blk_derase(desc, blk, cur_blkcnt);
		} else {
			/* Write 0s if not aligned */
			if (!blk_ptr) {
				blk_ptr = malloc(desc->blksz);
				if (!blk_ptr) {
					pr_err("failed to allocate\n");
					fastboot_fail("failed to allocate",
						      response);
					return true;
				}
				memset(blk_ptr, 0, desc->blksz);
			}
			if (fastboot_progress_callback)
				fastboot_progress_callback("erasing");
			blks_written = 0;
			do {
				blks_written += blk_dwrite(desc, blk, 1,
							   blk_ptr);
			} while (--cur_blkcnt);
		}
		blk += blks_written;
		blks += blks_written;
	}

	if (blk_ptr)
		free(blk_ptr);

	if (blks != blkcnt) {
		pr_err("failed writing to device %d\n", desc->devnum);
		fastboot_fail("failed writing to device", response);
		return true;
	}

	printf("........ wrote " LBAFU " bytes to '%s'\n", blkcnt * desc->blksz,
	       cmd);
	fastboot_okay(NULL, response);

	return true;
}


bool fastboot_board_flash_write(const char *cmd, void *download_buffer,
				u32 download_bytes, char *response)
{
	return fastboot_board_hwpart_process(cmd, download_buffer,
					     download_bytes, response);
}

bool fastboot_board_erase(const char *cmd, char *response)
{
	return fastboot_board_hwpart_process(cmd, NULL, 0, response);
}
#endif

int misc_init_r(void)
{
	meson_eth_init(PHY_INTERFACE_MODE_RMII,
		       MESON_USE_INTERNAL_RMII_PHY);

	env_set("serial#", "AMLG12ASEI510");

	return 0;
}
