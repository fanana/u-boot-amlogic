// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2016 Beniamino Galvani <b.galvani@gmail.com>
 */

#include <common.h>
#include <cpu_func.h>
#include <fastboot.h>
#include <init.h>
#include <net.h>
#include <asm/arch/boot.h>
#include <env.h>
#include <asm/cache.h>
#include <asm/ptrace.h>
#include <linux/libfdt.h>
#include <linux/err.h>
#include <asm/arch/mem.h>
#include <asm/arch/sm.h>
#include <asm/armv8/mmu.h>
#include <asm/unaligned.h>
#include <efi_loader.h>
#include <u-boot/crc.h>

#if CONFIG_IS_ENABLED(FASTBOOT)
#include <asm/psci.h>
#include <fastboot.h>
#include <linux/usb/gadget.h>

#endif

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
#define FASBOOT_HWPART_BOOTLOADER_OFFSET 	0x200		/* Block 1 */
#define FASBOOT_HWPART_BOOTLOADER_LENGTH 	0x1ffe00	/* 2MiB */
#define FASBOOT_HWPART_BOOTENV_OFFSET		CONFIG_ENV_OFFSET
#define FASBOOT_HWPART_BOOTENV_LENGTH		CONFIG_ENV_SIZE

#define FASTBOOT_MAX_BLK_WRITE 16384

#endif

DECLARE_GLOBAL_DATA_PTR;

__weak int board_init(void)
{
	return 0;
}

int dram_init(void)
{
	const fdt64_t *val;
	int offset;
	int len;

	offset = fdt_path_offset(gd->fdt_blob, "/memory");
	if (offset < 0)
		return -EINVAL;

	val = fdt_getprop(gd->fdt_blob, offset, "reg", &len);
	if (len < sizeof(*val) * 2)
		return -EINVAL;

	/* Use unaligned access since cache is still disabled */
	gd->ram_size = get_unaligned_be64(&val[1]);

	return 0;
}

__weak int meson_ft_board_setup(void *blob, struct bd_info *bd)
{
	return 0;
}

int ft_board_setup(void *blob, struct bd_info *bd)
{
	meson_init_reserved_memory(blob);

	return meson_ft_board_setup(blob, bd);
}

void meson_board_add_reserved_memory(void *fdt, u64 start, u64 size)
{
	int ret;

	ret = fdt_add_mem_rsv(fdt, start, size);
	if (ret)
		printf("Could not reserve zone @ 0x%llx\n", start);

	if (IS_ENABLED(CONFIG_EFI_LOADER))
		efi_add_memory_map(start, size, EFI_RESERVED_MEMORY_TYPE);
}

int meson_generate_serial_ethaddr(void)
{
	u8 mac_addr[ARP_HLEN];
	char serial[SM_SERIAL_SIZE];
	u32 sid;
	u16 sid16;

	if (!meson_sm_get_serial(serial, SM_SERIAL_SIZE)) {
		sid = crc32(0, (unsigned char *)serial, SM_SERIAL_SIZE);
		sid16 = crc16_ccitt(0, (unsigned char *)serial,	SM_SERIAL_SIZE);

		/* Ensure the NIC specific bytes of the mac are not all 0 */
		if ((sid & 0xffffff) == 0)
			sid |= 0x800000;

		/* Non OUI / registered MAC address */
		mac_addr[0] = ((sid16 >> 8) & 0xfc) | 0x02;
		mac_addr[1] = (sid16 >>  0) & 0xff;
		mac_addr[2] = (sid >> 24) & 0xff;
		mac_addr[3] = (sid >> 16) & 0xff;
		mac_addr[4] = (sid >>  8) & 0xff;
		mac_addr[5] = (sid >>  0) & 0xff;

		eth_env_set_enetaddr("ethaddr", mac_addr);
	} else
		return -EINVAL;

	return 0;
}

static void meson_set_boot_source(void)
{
	const char *source;

	switch (meson_get_boot_device()) {
	case BOOT_DEVICE_EMMC:
		source = "emmc";
		break;

	case BOOT_DEVICE_NAND:
		source = "nand";
		break;

	case BOOT_DEVICE_SPI:
		source = "spi";
		break;

	case BOOT_DEVICE_SD:
		source = "sd";
		break;

	case BOOT_DEVICE_USB:
		source = "usb";
		break;

	default:
		source = "unknown";
	}

	env_set("boot_source", source);
}

__weak int meson_board_late_init(void)
{
	return 0;
}

int board_late_init(void)
{
	meson_set_boot_source();

	return meson_board_late_init();
}

#if CONFIG_IS_ENABLED(FASTBOOT)
static unsigned int reboot_reason = REBOOT_REASON_NORMAL;

void reset_cpu(ulong addr)
{
	struct pt_regs regs;

	regs.regs[0] = ARM_PSCI_0_2_FN_SYSTEM_RESET;
	regs.regs[1] = reboot_reason;

	usb_gadget_release(0);

	printf("Rebooting with reason: 0x%lx\n", regs.regs[1]);

	smc_call(&regs);

	while (1)
		;
}
#else
void reset_cpu(ulong addr)
{
	psci_system_reset();
}
#endif


#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_CUSTOM) && CONFIG_IS_ENABLED(MMC_WRITE)

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