/* SPDX-License-Identifier: GPL-2.0+ WITH Linux-syscall-note */
/*
 * SPI driver for UWB SR1xx
 * Copyright (C) 2018-2022 NXP.
 *
 * Author: Manjunatha Venkatesh <manjunatha.venkatesh@xxxxxxx>
 */
#define SR1XX_MAGIC 0xEA
#define SR1XX_SET_PWR _IOW(SR1XX_MAGIC, 0x01, uint32_t)
#define SR1XX_SET_FWD _IOW(SR1XX_MAGIC, 0x02, uint32_t)

/* Power enable/disable and read abort ioctl arguments */
enum { PWR_DISABLE = 0, PWR_ENABLE, ABORT_READ_PENDING };