/**********************************************************************************
 * Copyright (C) 2023  Emiliano Silvestri                                         *
 *                                                                                *
 * This program is free software; you can redistribute it and/or                  *
 * modify it under the terms of the GNU General Public License                    *
 * as published by the Free Software Foundation; either version 2                 *
 * of the License, or (at your option) any later version.                         *
 *                                                                                *
 * This program is distributed in the hope that it will be useful,                *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of                 *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                  *
 * GNU General Public License for more details.                                   *
 *                                                                                *
 * You should have received a copy of the GNU General Public License              *
 * along with this program; if not, write to the Free Software                    *
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA. *
 **********************************************************************************/

#ifndef __SPI_MODULE_H
#define __SPI_MODULE_H

#include <linux/device.h>
#include <linux/device/class.h>
#include <linux/spinlock.h>
#include <linux/irq.h>
#include <linux/gpio/consumer.h>


/****************************************************
 *                    CONSTANTS                     *
 ****************************************************/

#define EMPTY_DATA	0x00
#define ACK_DATA	0xFF


/****************************************************
 *                    STRUCTURES                    *
 ****************************************************/

/**
 * The spimodule_device structure is used
 * to keep track of all information related
 * to the current instantiation of the driver.
 *
 * @chr_major:	major number assigned to the character
 * 				device registered
 * @chr_class:	pointer to the class of the already
 * 				registered character device
 * @chr_device:	pointer to the just created character
 * 				device
 * @irq_flags:	a mask indicating the way the IRQ must
 * 				be handled by the devised thread
 * @irq_chip:	structure containing data that describes
 * 				the related IRQ line
 * @reset_chip:	descriptor of the GPIO pin used to send
 * 				out a reset signal to the external chip
 * @spi_device:	the SPI device associate to this protocol
 * 				driver
 */
struct spimodule_device {
	int chr_major;
	struct class *chr_class;
	struct device *chr_device;

	unsigned long irq_flags;
	struct irq_data *irq_chip;
	struct gpio_desc *reset_chip;

	struct spi_device *spi_device;

	spinlock_t lock;

	unsigned long rem_bytes;
	char *buffer;

	unsigned int wait_ack;
	char ack;
};


/****************************************************
 *                    PROTOTYPES                    *
 ****************************************************/

static int chr_open(struct inode *, struct file *);
static int chr_release(struct inode *, struct file *);
static ssize_t chr_write(struct file *, const char __user *, size_t, loff_t *);

#endif