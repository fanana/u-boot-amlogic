/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2019 BayLibre SAS
 */

#ifndef _FB_BOARD_H_
#define _FB_BOARD_H_

/**
 * fastboot_board_flash_write() - Write image to board partition for fastboot
 *
 * @cmd: Named partition to write image to
 * @download_buffer: Pointer to image data
 * @download_bytes: Size of image data
 * @response: Pointer to fastboot response buffer
 * @return true if write was handled, false if was not handled
 */
bool fastboot_board_flash_write(const char *cmd, void *download_buffer,
				u32 download_bytes, char *response);
/**
 * fastboot_board_flash_erase() - Erase board partition for fastboot
 *
 * @cmd: Named partition to erase
 * @response: Pointer to fastboot response buffer
 * @return true if erase was handled, false if was not handled
 */
bool fastboot_board_erase(const char *cmd, char *response);
#endif
