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

#include <linux/of.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/timekeeping.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio/consumer.h>
#include <linux/spi/spi.h>

#include "spi-module.h"


/****************************************************
 *                       DATA                       *
 ****************************************************/

static struct spimodule_device spim_dev;

static const struct file_operations chr_fops = {
	.open = chr_open,
	.owner = THIS_MODULE,
	.release = chr_release,
	.write = chr_write
};

static const struct spi_device_id spimodule_spi_ids[] = {
	{ .name = "spimodule" },
	{ }
};
MODULE_DEVICE_TABLE(spi, spimodule_spi_ids);

static const struct of_device_id spimodule_dt_ids[] = {
	{ .compatible = "spimodule" },
	{ }
};
MODULE_DEVICE_TABLE(of, spimodule_dt_ids);


/****************************************************
 *                 FILE OPERATIONS                  *
 ****************************************************/

static int chr_open(struct inode *node, struct file *filp)
{
	return 0;
}

static int chr_release(struct inode *node, struct file *filp)
{
	return 0;
}

static ssize_t chr_write(struct file *filp, const char __user *u_buf, size_t size, loff_t *pos)
{
	char *buf;
	unsigned long rem;

	if (size == 0)
		return 0;

	spin_lock(&spim_dev.lock);

	if (spim_dev.wait_ack != 0)
	{
		/****************************************************
		 * The SPI module is currently busy and the user's  *
		 * application must wait all previously submitted   *
		 * data is actually sent to the ATmega328P before a *
		 * new transfer session will be possible.           *
		 ****************************************************/
		spin_unlock(&spim_dev.lock);
		return -EBUSY;
	}

	buf = kmalloc(size, GFP_KERNEL);
	if (IS_ERR_OR_NULL(buf))
	{
		spin_unlock(&spim_dev.lock);
		return -ENOMEM;
	}

	rem = (unsigned long) size;
	if (copy_from_user(buf, (const void __user *) u_buf, rem))
	{
		kfree(buf);

		spin_unlock(&spim_dev.lock);
		return -EINVAL;
	}

	spim_dev.rem_bytes = rem;
	spim_dev.buffer = buf;
	spim_dev.wait_ack = 1;

	if (spi_write(spim_dev.spi_device, &spim_dev.buffer[spim_dev.rem_bytes-1], 1))
	{
		printk(KERN_ERR "[SPI Module] failed to write to SPI. The chip is going to be reset.\n");

		kfree(spim_dev.buffer);

		spim_dev.buffer = NULL;
		spim_dev.rem_bytes = 0;
		spim_dev.wait_ack = 0;

		gpiod_set_value_cansleep(spim_dev.reset_chip, 1);
		udelay(4);
		gpiod_set_value_cansleep(spim_dev.reset_chip, 0);

		spin_unlock(&spim_dev.lock);
		return -EFAULT;
	}

	printk(KERN_DEBUG "[SPI Module] DATA sent. Buffer[%lu]:%d.\n",
		spim_dev.rem_bytes-1, (int) spim_dev.buffer[spim_dev.rem_bytes-1]);

	spin_unlock(&spim_dev.lock);

	return size;
}


/****************************************************
 *                  DEVICE EVENTS                   *
 ****************************************************/

static int chr_dev_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	add_uevent_var(env, "DEVMODE=%#o", 0222);
	return 0;
}


/****************************************************
 *                   IRQ HANDLER                    *
 ****************************************************/

static irqreturn_t spi_thread_function(int irq, void *arg)
{
	spin_lock(&spim_dev.lock);

	if (spim_dev.wait_ack == 0)
	{
		/****************************************************
		 * Actually, no data is currently waiting to be     *
		 * acknowledged on both sides.                      *
		 ****************************************************/
		spin_unlock(&spim_dev.lock);
		return IRQ_HANDLED;
	}

	if (spi_read(spim_dev.spi_device, &spim_dev.ack, 1))
	{
		printk(KERN_ERR "[SPI Module] failed to read from SPI. The chip is going to be reset.\n");

		kfree(spim_dev.buffer);

		spim_dev.buffer = NULL;
		spim_dev.rem_bytes = 0;
		spim_dev.wait_ack = 0;

		gpiod_set_value_cansleep(spim_dev.reset_chip, 1);
		udelay(4);
		gpiod_set_value_cansleep(spim_dev.reset_chip, 0);

		spin_unlock(&spim_dev.lock);
		return IRQ_HANDLED;
	}

	if (spim_dev.ack == spim_dev.buffer[spim_dev.rem_bytes-1])
	{
		/****************************************************
		 * Once data is received, the ATmega328P sends back *
		 * the same data to the module in order to make it  *
		 * aware that the transfer completed succesfully.   *
		 * If the data matches the original one, the module *
		 * will send an ACK.                                *
		 ****************************************************/
		spim_dev.ack = ACK_DATA;
		spim_dev.rem_bytes -= 1;
	}
	else
	{
		/****************************************************
		 * The data sent by the ATmega328P does not match   *
		 * the original one. Thus, something bad has        *
		 * occurred during the transfers. This is notified  *
		 * to the MCU by sending a non-ACK value, which     *
		 * also means that this data must be re-sent.       *
		 ****************************************************/
		spim_dev.ack = EMPTY_DATA;
	}

	if (spi_write(spim_dev.spi_device, &spim_dev.ack, 1))
	{
		printk(KERN_ERR "[SPI Module] failed to write to SPI. The chip is going to be reset.\n");

		kfree(spim_dev.buffer);

		spim_dev.buffer = NULL;
		spim_dev.rem_bytes = 0;
		spim_dev.wait_ack = 0;

		gpiod_set_value_cansleep(spim_dev.reset_chip, 1);
		udelay(4);
		gpiod_set_value_cansleep(spim_dev.reset_chip, 0);

		spin_unlock(&spim_dev.lock);
		return IRQ_HANDLED;
	}

	if (spim_dev.rem_bytes == 0)
	{
		kfree(spim_dev.buffer);

		spim_dev.buffer = NULL;
		spim_dev.wait_ack = 0;

		spin_unlock(&spim_dev.lock);
		return IRQ_HANDLED;
	}

	udelay(1);

	if (spi_write(spim_dev.spi_device, &spim_dev.buffer[spim_dev.rem_bytes-1], 1))
	{
		printk(KERN_ERR "[SPI Module] failed to write to SPI. The chip is going to be reset.\n");

		kfree(spim_dev.buffer);

		spim_dev.buffer = NULL;
		spim_dev.rem_bytes = 0;
		spim_dev.wait_ack = 0;

		gpiod_set_value_cansleep(spim_dev.reset_chip, 1);
		udelay(4);
		gpiod_set_value_cansleep(spim_dev.reset_chip, 0);

		spin_unlock(&spim_dev.lock);
		return IRQ_HANDLED;
	}

	printk(KERN_DEBUG "[SPI Module] DATA sent. Buffer[%lu]:%d.\n",
		spim_dev.rem_bytes-1, (int) spim_dev.buffer[spim_dev.rem_bytes-1]);

	spin_unlock(&spim_dev.lock);

	return IRQ_HANDLED;
}

static irqreturn_t spi_irq_handler(int irq, void *arg)
{
	if (irq != spim_dev.irq_chip->irq)
		return IRQ_NONE;

	return IRQ_WAKE_THREAD;
}


/****************************************************
 *                DRIVER INIT & FINI                *
 ****************************************************/

static int spimodule_probe(struct spi_device *spi)
{
	int retval, d_time;
	ktime_t s_time, e_time;

	printk(KERN_INFO "[SPI Module] start probing the module\n");

	memset((void *) &spim_dev, 0, sizeof(struct spimodule_device));

	spin_lock_init(&spim_dev.lock);

	/****************************************************
	 * Create a character device that will be made      *
	 * available in user-space in order to perform      *
	 * writes that are destinated to the ATmega328P.    *
	 ****************************************************/
	spim_dev.chr_major = __register_chrdev(0, 0, 1, "spi_module_dev", &chr_fops);
	if (spim_dev.chr_major < 0)
	{
		printk(KERN_ERR "[SPI Module] failed to register the character device\n");
		goto err_init_0;
	}

	spim_dev.chr_class = class_create(THIS_MODULE, "spi_module_dev");
	if (IS_ERR_OR_NULL(spim_dev.chr_class))
	{
		printk(KERN_ERR "[SPI Module] failed to register the device class\n");
		goto err_init_1;
	}

	spim_dev.chr_class->dev_uevent = chr_dev_uevent;

	spim_dev.chr_device = device_create(spim_dev.chr_class, NULL, MKDEV(spim_dev.chr_major, 0), NULL, "spi_module_dev");
	if (IS_ERR_OR_NULL(spim_dev.chr_device))
	{
		printk(KERN_ERR "[SPI Module] failed to create the character device\n");
		goto err_init_2;
	}

	spim_dev.spi_device = spi;

	/****************************************************
	 * The spimodule device specified into the device   *
	 * tree source file also defines a GPIO pin that is *
	 * used to reset the ATmega328P whenever the driver *
	 * is initialized or the communication deviates     *
	 * from the specified protocol.                     *
	 ****************************************************/
	spim_dev.reset_chip = gpiod_get(&spi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(spim_dev.reset_chip))
	{
		printk(KERN_ERR "[SPI Module] failed to obtain the reset GPIO descriptor\n");
		goto err_init_3;
	}

	gpiod_set_value_cansleep(spim_dev.reset_chip, 1);
	s_time = ktime_get();

	/****************************************************
	 * The spimodule device specified into the device   *
	 * tree source file also defines a GPIO pin that is *
	 * used by ATmega328P to trigger interrupts once it *
	 * has completed some task and a response is ready  *
	 * to be read.                                      *
	 ****************************************************/
	spim_dev.irq_chip = irq_get_irq_data(spi->irq);
	if (IS_ERR_OR_NULL(spim_dev.irq_chip))
	{
		printk(KERN_ERR "[SPI Module] failed to obtain the IRQ chip data\n");
		goto err_init_4;
	}

	/****************************************************
	 * Interrupt is not reenabled after the Hard-IRQ    *
	 * handler finished. This is used by threaded       *
	 * interrupts which need to keep the IRQ line       *
	 * disabled until the handler has finished.         *
	 ****************************************************/
	spim_dev.irq_flags = irqd_get_trigger_type(spim_dev.irq_chip);
	retval = request_threaded_irq(spi->irq, spi_irq_handler, spi_thread_function, spim_dev.irq_flags, "spimodule", (void *) &spim_dev);
	if (retval)
	{
		printk(KERN_ERR "[SPI Module] failed to create threaded IRQ handler\n");
		goto err_init_4;
	}

	e_time = ktime_get();
	if ((d_time = (int) (e_time - s_time)) > 0 && d_time < 4000)
		udelay(4 - (d_time / 1000));
	else
		udelay(4);
	gpiod_set_value_cansleep(spim_dev.reset_chip, 0);

	return 0;

err_init_4:
	gpiod_set_value_cansleep(spim_dev.reset_chip, 0);
	gpiod_put(spim_dev.reset_chip);
err_init_3:
	device_destroy(spim_dev.chr_class, MKDEV(spim_dev.chr_major, 0));
err_init_2:
	class_destroy(spim_dev.chr_class);
err_init_1:
	__unregister_chrdev(spim_dev.chr_major, 0, 0, "spi_module_dev");
err_init_0:
	return -1;
}

static int spimodule_remove(struct spi_device *spi)
{
	printk(KERN_INFO "[SPI Module] start removing the module\n");

	free_irq(spi->irq, (void *) &spim_dev);
	gpiod_put(spim_dev.reset_chip);

	device_destroy(spim_dev.chr_class, MKDEV(spim_dev.chr_major, 0));
	class_destroy(spim_dev.chr_class);
	__unregister_chrdev(spim_dev.chr_major, 0, 0, "spi_module_dev");

	spin_lock(&spim_dev.lock);

	if (spim_dev.buffer != NULL)
	{
		kfree(spim_dev.buffer);

		spim_dev.buffer = NULL;
		spim_dev.rem_bytes = 0;
		spim_dev.wait_ack = 0;
	}

	spin_unlock(&spim_dev.lock);

	memset((void *) &spim_dev, 0, sizeof(struct spimodule_device));

	return 0;
}


static struct spi_driver spimodule_driver = {
	.id_table = spimodule_spi_ids,
	.probe = spimodule_probe,
	.remove = spimodule_remove,
	.driver = {
		.name = "spimodule",
		.of_match_table = of_match_ptr(spimodule_dt_ids),
	},
};

module_spi_driver(spimodule_driver);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Emiliano Silvestri <emisilve86@gmail.com>");
MODULE_DESCRIPTION("SPI Module");
MODULE_ALIAS("spi:spimodule");
