/*
 * Copyright (c) 2018, Anton Kikin <a.a.kikin@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * The full GNU General Public License is included in this distribution in the
 * file called LICENSE.
 *
 * Authors: Anton Kikin <a.a.kikin@gmail.com>
 *
 * Encoding: UTF-8
 * Recommended tab size: 4
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/version.h>
#include <linux/moduleparam.h>
#include <linux/uaccess.h>

#include "hw_drv.h"

/*--------------------------------------------------------------------------------*/

MODULE_AUTHOR       ("Anton Kikin <a.a.kikin@gmail.com>");
MODULE_LICENSE      ("GPL v2");
MODULE_DESCRIPTION  ("Hardware PCI test driver");
MODULE_VERSION      ("1.0");

/* Max BAR's count for PCI device */
#define HW_DRV_RESOURCES_COUNT_MAX  6

/*--------------------------------------------------------------------------------*/

typedef struct hw_dev_res
{
	void           *base_virt; /* Virtual address of the memory mapped BAR */
	unsigned long   base_phys; /* Physical address of the memory mapped BAR */
	unsigned long   length;    /* Size of the memory mapped using this BAR */
	unsigned int    cached;    /* Cached flag */

} hw_dev_res_t;

/*--------------------------------------------------------------------------------*/

#define IRQ_VECTOR_NAME_MAX	32

typedef struct hw_dev
{
	struct list_head list;

	/* Device lock */
	struct mutex lock;

	int                 msi;
	unsigned            dev_id;
	struct pci_dev	   *pci_dev;
	struct hw_dev_res   res[HW_DRV_RESOURCES_COUNT_MAX];

	unsigned long irq_counter;
	char irq_vector_name[IRQ_VECTOR_NAME_MAX];

	/* Virtual address */
	void *mem_base;

	/* Physical DMA address */
	dma_addr_t mem_base_dma;

	/* Memory size */
	unsigned int mem_size;

} hw_dev_t;

/*--------------------------------------------------------------------------------*/

/*
 * Global driver data
 */
typedef struct hw_drv
{
	struct list_head    list;
	rwlock_t            list_lock;

	unsigned int        dev_count;

	dev_t               chrdev_region;
	struct class       *chrdev_class;
	int                 chrdev_major;
	struct cdev         chrdev;
	unsigned int        chrdev_open_counter;

	hw_dev_t           *hw_dev_local;

	struct semaphore sem;

} hw_drv_t;

static hw_drv_t *hw_drv = 0;

/*--------------------------------------------------------------------------------*/

static long __hw_dev_fop_ioctl (struct file *file, unsigned int cmd, unsigned long arg);
static int  __hw_dev_fop_flush (struct file *file, fl_owner_t id);
static int  __hw_dev_fop_open  (struct inode *inode, struct file *file);

static const struct file_operations __hw_dev_fops =
{
	.owner          = THIS_MODULE,
	.open           = __hw_dev_fop_open,
	.flush          = __hw_dev_fop_flush,
	.unlocked_ioctl = __hw_dev_fop_ioctl,
	.compat_ioctl   = __hw_dev_fop_ioctl
};

/*--------------------------------------------------------------------------------*/

/*
 *	Module parameters
 */

#if (HW_DRV_USE_PCI_DEVICES)

	static int no_msi = 0;
	module_param       (no_msi, int, S_IRUGO);
	MODULE_PARM_DESC   (no_msi, "Disable MSI PCI interrupts support");

	static int dma_32 = 1;
	module_param       (dma_32, int, S_IRUGO);
	MODULE_PARM_DESC   (dma_32, "Set 32-bit DMA mask always");

#endif

static int mem_size = 2048;
module_param       (mem_size, int, S_IRUGO);
MODULE_PARM_DESC   (mem_size, "Memory size (in KiB)");

/*--------------------------------------------------------------------------------*/

/*
 * Init/destroy functions
 */
static int hw_dev_init_resources(hw_dev_t *hw_dev);
static int hw_dev_destroy_resources(hw_dev_t *hw_dev);

/*
 * Register/unregister/find functions
 */
static hw_dev_t *hw_dev_register_device(struct pci_dev *pci_dev);
static void hw_dev_unregister_device(hw_dev_t *hw_dev);
static hw_dev_t *hw_dev_find(const unsigned dev_id);

#if (HW_DRV_USE_PCI_DEVICES)

	/*
	 * Device interrupt functions
	 */
	static irqreturn_t hw_dev_isr_handler(int irq, void *arg);

	static int hw_dev_interrupts_init(hw_dev_t *hw_dev);
	static int hw_dev_interrupts_destroy(hw_dev_t *hw_dev);

#endif

/*
 * Memory init/destroy functions
 */
static int hw_dev_memory_init(hw_dev_t *hw_dev, const unsigned int size);
static int hw_dev_memory_destroy(hw_dev_t *hw_dev);

/*--------------------------------------------------------------------------------*/

#if (HW_DRV_USE_PCI_DEVICES)

/*
 * Interrupts
 */
static irqreturn_t hw_dev_isr_handler(int irq, void *arg)
{
	hw_dev_t *hw_dev = (hw_dev_t *)arg;

	if (hw_dev)
	{
		++hw_dev->irq_counter;

		printk("%s [%02d]: Interrupt %d (counter = %lu)\n",
			HW_DRV_NAME, hw_dev->dev_id, irq, hw_dev->irq_counter
		);

		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static int hw_dev_interrupts_init(hw_dev_t *hw_dev)
{
	int ret;

	if (!hw_dev->pci_dev)
		return 0;

	if (pci_find_capability(hw_dev->pci_dev, PCI_CAP_ID_MSI))
	{
		printk("%s [%02d]: Device is supported MSI\n",
			HW_DRV_NAME, hw_dev->dev_id);

		if (!no_msi)
		{			
			ret = pci_enable_msi(hw_dev->pci_dev);

			if (!ret)
			{
				hw_dev->msi = 1;

				printk("%s [%02d]: MSI enabled\n",
					HW_DRV_NAME, hw_dev->dev_id);
			}
			else
			{
				printk(
					"%s [%02d]: ERROR: Can't enable MSI on device "
					"(errcode = %d)\n",
					HW_DRV_NAME,
					hw_dev->dev_id,
					ret
				);
			}
		}
	}
	else
	{
		printk("%s [%02d]: Device is not supported MSI\n",
			HW_DRV_NAME, hw_dev->dev_id);
	}

	snprintf(hw_dev->irq_vector_name,
		IRQ_VECTOR_NAME_MAX, "hw_drv_%u", hw_dev->dev_id);

	ret = request_irq(
		hw_dev->pci_dev->irq,
		hw_dev_isr_handler,
		no_msi ? IRQF_SHARED : 0,
		hw_dev->irq_vector_name,
		hw_dev
	);

	if (ret)
	{
		printk(
			"%s [%02d]: ERROR: Can't register IRQ for device "
			"(errcode = %d)\n",
			HW_DRV_NAME,
			hw_dev->dev_id,
			ret
		);

		return -1;
	}

	printk("%s [%02d]: Registered the IRQ %d for device\n",
		HW_DRV_NAME, hw_dev->dev_id, hw_dev->pci_dev->irq);

	return 0;
}

static int hw_dev_interrupts_destroy(hw_dev_t *hw_dev)
{
	if (!hw_dev->pci_dev)
		return 0;

	free_irq(hw_dev->pci_dev->irq, hw_dev);

	if (hw_dev->msi)
		pci_disable_msi(hw_dev->pci_dev);

	return 0;
}

#endif

/*--------------------------------------------------------------------------------*/

static int hw_dev_memory_init(hw_dev_t *hw_dev, const unsigned int size)
{
	hw_dev->mem_size = size;

	if (!size)
		return -1;

	if (hw_dev->pci_dev)
	{
		hw_dev->mem_base = (void *)dma_alloc_coherent(
			&hw_dev->pci_dev->dev,
			 hw_dev->mem_size,
			&hw_dev->mem_base_dma,
			 GFP_KERNEL
		);
	}
	else
	{
		hw_dev->mem_base = kzalloc(hw_dev->mem_size, GFP_KERNEL);
	}

	if (!hw_dev->mem_base)
	{
		printk(
			"%s [%02d]: ERROR: Failed to allocate DMA memory\n",
			HW_DRV_NAME,
			hw_dev->dev_id
		);

		return -1;
	}

	if (!hw_dev->pci_dev)
		hw_dev->mem_base_dma = virt_to_phys(hw_dev->mem_base);

	memset(hw_dev->mem_base, 0xFF, hw_dev->mem_size);

	printk("%s [%02d]: Allocated %d bytes of DMA memory\n",
		HW_DRV_NAME, hw_dev->dev_id, size);

	printk(
		"%s [%02d]: Memory virtual base = 0x%16lx, "
		"physical (bus) base = 0x%016lx\n",
		HW_DRV_NAME,
		hw_dev->dev_id,
		(unsigned long)(uintptr_t)hw_dev->mem_base,
		(unsigned long)(uintptr_t)hw_dev->mem_base_dma
	);

	return 0;
}

static int hw_dev_memory_destroy(hw_dev_t *hw_dev)
{
	if (hw_dev->pci_dev)
	{
		dma_free_coherent(
			&hw_dev->pci_dev->dev,
			 hw_dev->mem_size,
			 hw_dev->mem_base,
			 hw_dev->mem_base_dma
		);
	}
	else
	{
		kfree(hw_dev->mem_base);
	}

	return 0;
}

/*--------------------------------------------------------------------------------*/

static hw_dev_t *hw_dev_find(const unsigned dev_id)
{
	struct list_head *l;
	hw_dev_t *hw_dev;

	read_lock(&hw_drv->list_lock);

	list_for_each(l, &hw_drv->list)
	{
		hw_dev = list_entry(l, hw_dev_t, list);

		if (hw_dev->dev_id == dev_id)
		{
			read_unlock(&hw_drv->list_lock);
			return hw_dev;
		}
	}

	read_unlock(&hw_drv->list_lock);

	return 0;
}

/*--------------------------------------------------------------------------------*/

static int hw_dev_init(hw_dev_t *hw_dev)
{
	hw_dev->irq_counter = 0;
	hw_dev->msi = 0;

	mutex_init(&hw_dev->lock);

	#if (HW_DRV_USE_PCI_DEVICES)

	if (hw_dev_interrupts_init(hw_dev))
	{
		printk(
			"%s [%02d]: ERROR: Can't initialize interrupts\n",
			HW_DRV_NAME,
			hw_dev->dev_id
		);

		return -1;
	}

	#endif

	if (hw_dev_memory_init(hw_dev, mem_size << 10))
	{
		printk(
			"%s [%02d]: ERROR: Can't allocate DMA memory (%d KiB)\n",
			HW_DRV_NAME,
			hw_dev->dev_id, mem_size
		);
			
		#if (HW_DRV_USE_PCI_DEVICES)
			hw_dev_interrupts_destroy(hw_dev);
		#endif

		return -1;
	}

	return 0;
}

static int hw_dev_destroy(hw_dev_t *hw_dev)
{
	hw_dev_memory_destroy(hw_dev);

	#if (HW_DRV_USE_PCI_DEVICES)
		hw_dev_interrupts_destroy(hw_dev);
	#endif

	return 0;
}

/*--------------------------------------------------------------------------------*/

static unsigned int hw_drv_dev_id_next = 0;

static hw_dev_t *hw_dev_register_device(struct pci_dev *pci_dev)
{
	hw_dev_t *hw_dev =
		(hw_dev_t *)kzalloc(sizeof(hw_dev_t), GFP_KERNEL);

	if (!hw_dev)
		return 0;

	hw_dev->pci_dev = pci_dev;
	hw_dev->dev_id  = hw_drv_dev_id_next++;

	/* Initialize device resources */
	if (hw_dev_init_resources(hw_dev) < 0)
	{
		printk(
			"%s [%02d]: ERROR: Can't initialize device resources\n",
			HW_DRV_NAME,
			hw_dev->dev_id
		);

		kfree(hw_dev);
		return 0;
	}

	if (hw_dev_init(hw_dev))
	{
		printk(
			"%s [%02d]: ERROR: Function hw_dev_init() failed\n",
			HW_DRV_NAME,
			hw_dev->dev_id
		);

		hw_dev_destroy_resources(hw_dev);
		kfree(hw_dev);

		return 0;
	}

	write_lock(&hw_drv->list_lock);

	list_add_tail(&hw_dev->list, &hw_drv->list);
	hw_drv->dev_count++;

	write_unlock(&hw_drv->list_lock);

	printk("%s [%02d]: Device initialized\n",
		HW_DRV_NAME, hw_dev->dev_id);

	return hw_dev;
}

static void hw_dev_unregister_device(hw_dev_t *hw_dev)
{
	write_lock(&hw_drv->list_lock);
	
	list_del(&hw_dev->list);
	hw_drv->dev_count--;

	write_unlock(&hw_drv->list_lock);

	hw_dev_destroy(hw_dev);

	hw_dev_destroy_resources(hw_dev);
	kfree(hw_dev);
}

/*--------------------------------------------------------------------------------*/

/*
 *	 _____
 *	|  __ \
 *	| |__) |___  ___  ___  _   _ _ __ ___ ___  ___
 *	|  _  // _ \/ __|/ _ \| | | | '__/ __/ _ \/ __|
 *	| | \ \  __/\__ \ (_) | |_| | | | (_|  __/\__ \
 *	|_|  \_\___||___/\___/ \__,_|_|  \___\___||___/
 *
 */

#if (HW_DRV_USE_PCI_DEVICES)

static void hw_dev_pci_set(struct pci_dev *pci_dev);
static void hw_dev_pci_clear(struct pci_dev *pci_dev);
static void hw_dev_pci_set_dma(struct pci_dev *pci_dev);

#endif

static int hw_dev_init_resources(hw_dev_t *hw_dev)
{
	int bar;

	if (!hw_dev->pci_dev)
	{
		for (bar = 0; bar < HW_DRV_RESOURCES_COUNT_MAX; bar++)
		{
			hw_dev->res[bar].length    = 0;
			hw_dev->res[bar].base_virt = 0;
			hw_dev->res[bar].base_phys = 0;
		}

		return 0;
	}

	#if (HW_DRV_USE_PCI_DEVICES)
	{
		resource_size_t bar_start;
		resource_size_t bar_len;
		resource_size_t bar_flags;

		hw_dev_pci_set_dma(hw_dev->pci_dev);

		if (pci_enable_device(hw_dev->pci_dev) != 0)
		{
			printk(
				"%s [%02d]: ERROR: Can't enable device\n",
				HW_DRV_NAME,
				hw_dev->dev_id
			);

			return -1;
		}
		
		pci_set_drvdata(hw_dev->pci_dev, hw_dev);
		hw_dev_pci_set(hw_dev->pci_dev);

		for (bar = 0; bar < HW_DRV_RESOURCES_COUNT_MAX; bar++)
		{
			hw_dev->res[bar].length    = 0;
			hw_dev->res[bar].base_virt = 0;
			hw_dev->res[bar].base_phys = 0;
			hw_dev->res[bar].cached    = 0;

			bar_start = pci_resource_start (hw_dev->pci_dev, bar);
			bar_len   = pci_resource_len   (hw_dev->pci_dev, bar);
			bar_flags = pci_resource_flags (hw_dev->pci_dev, bar);

			if (bar_len == 0)
			{
				/* BAR is unused */
				printk("  * BAR%d: -- DISABLED --\n", bar);
				continue;
			}

			/* Map the memory region. */
			if (bar_flags & IORESOURCE_MEM)
			{
				hw_dev->res[bar].base_phys = bar_start;
				hw_dev->res[bar].length    = bar_len;

				request_mem_region(
					hw_dev->res[bar].base_phys, bar_len, HW_DRV_NAME);

				hw_dev->res[bar].base_virt = ioremap_nocache(bar_start, bar_len);
				hw_dev->res[bar].cached    = 0;

				printk(
					"  * BAR%d: Physical = 0x%08x, virtual = 0x%16p, length = %d\n",
					bar,
					(uint32_t)hw_dev->res[bar].base_phys,
					          hw_dev->res[bar].base_virt,
					(uint32_t)hw_dev->res[bar].length
				);
			}
			else
			{
				/* Ignore IO space regions */
				printk("  * BAR%d: -- I/O space --\n", bar);
			}
		}
	}
	#endif

	return 0;
}

static int hw_dev_destroy_resources(hw_dev_t *hw_dev)
{
	if (!hw_dev->pci_dev)
		return 0;

	#if (HW_DRV_USE_PCI_DEVICES)
	{
		int bar;

		for (bar = 0; bar < HW_DRV_RESOURCES_COUNT_MAX; bar++)
		{
			if (hw_dev->res[bar].length == 0)
				continue;

			/* Unmap region & release the region. */
			iounmap(hw_dev->res[bar].base_virt);

			/* Unmap the memory region. */
			if (pci_resource_flags(hw_dev->pci_dev, bar) & IORESOURCE_MEM)
			{
				release_mem_region(
					hw_dev->res[bar].base_phys,
					hw_dev->res[bar].length
				);
			}

			printk("  * Released BAR%d: physical = 0x%08x\n",
				bar, (uint32_t)hw_dev->res[bar].base_phys);
		}

		hw_dev_pci_clear(hw_dev->pci_dev);
		pci_disable_device(hw_dev->pci_dev);
	}
	#endif

	return 0;
}

/*--------------------------------------------------------------------------------*/

#if (HW_DRV_USE_PCI_DEVICES)

/*
 *	 _____   _____ _____
 *	|  __ \ / ____|_   _|
 *	| |__) | |      | |
 *	|  ___/| |      | |
 *	| |    | |____ _| |_
 *	|_|     \_____|_____|
 *
 */

/*
 * Set the DMA mask for PCI device
 */
static void hw_dev_pci_set_dma(struct pci_dev *pci_dev)
{
	int ret = -1;

	/* Set the DMA mask */
	if (!dma_32)
		ret = pci_set_dma_mask(pci_dev, DMA_BIT_MASK(64));

	if (!ret)
	{
		pci_set_consistent_dma_mask(pci_dev, DMA_BIT_MASK(64));
		printk("Setted 64-bit DMA bitmask\r\n");
	}
	else
	{
		ret = pci_set_dma_mask(pci_dev, DMA_BIT_MASK(32));

		if (!ret)
		{
			pci_set_consistent_dma_mask(pci_dev, DMA_BIT_MASK(32));
			printk("Setted 32-bit DMA bitmask\r\n");
		}
	}

	if (ret)
		printk("ERROR: Failed to set DMA bitmask\r\n");
}

static void hw_dev_pci_set(struct pci_dev *pci_dev)
{
	int32_t  retVal;
	uint16_t cmdVal;

	pci_set_master(pci_dev);

	pci_write_config_byte(pci_dev, PCI_LATENCY_TIMER, 0x80);

	/* Enable memory write invalidate */
	retVal = pci_set_mwi(pci_dev);

	pci_read_config_word(pci_dev, PCI_COMMAND, (u16 *)&cmdVal);

	cmdVal |=  PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER | PCI_COMMAND_SERR;
	cmdVal &= ~PCI_COMMAND_INTX_DISABLE;

	pci_write_config_word(pci_dev, PCI_COMMAND, cmdVal);
}

static void hw_dev_pci_clear(struct pci_dev *pci_dev)
{
	uint16_t cmdVal;

	pci_clear_master(pci_dev);
	pci_clear_mwi(pci_dev);

	pci_read_config_word(pci_dev, PCI_COMMAND, (u16 *)&cmdVal);

	cmdVal &= ~PCI_COMMAND_MEMORY;
	cmdVal &= ~PCI_COMMAND_MASTER;
	cmdVal &= ~PCI_COMMAND_SERR;
	cmdVal &= ~PCI_COMMAND_INTX_DISABLE;

	pci_write_config_word(pci_dev, PCI_COMMAND, cmdVal);
}

/*--------------------------------------------------------------------------------*/

static struct pci_device_id hw_drv_pci_ids[] =
{
	HW_DRV_PCI_DEVICES_IDS
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, hw_drv_pci_ids);

/* PCI driver functions */
static int	hw_drv_pci_probe	(struct pci_dev *pci_dev, const struct pci_device_id *pci_id);
static void hw_drv_pci_remove	(struct pci_dev *pci_dev);
static int  hw_drv_pci_suspend	(struct pci_dev *pci_dev, pm_message_t state);
static int  hw_drv_pci_resume	(struct pci_dev *pci_dev);
static void hw_drv_pci_shutdown (struct pci_dev *pci_dev);

static struct pci_driver hw_drv_pci_driver =
{
	.name      = HW_DRV_NAME,
	.id_table  = hw_drv_pci_ids,
	.probe     = hw_drv_pci_probe,
	.remove    = hw_drv_pci_remove,
	.suspend   = hw_drv_pci_suspend,
	.resume    = hw_drv_pci_resume,
	.shutdown  = hw_drv_pci_shutdown
};

/*--------------------------------------------------------------------------------*/

static int hw_drv_pci_probe(struct pci_dev *pci_dev, const struct pci_device_id *pci_id)
{
	hw_dev_t *hw_dev;

	printk("%s: Founded PCI device (vendor = 0x%04X, device = 0x%04X)\n",
		HW_DRV_NAME,
		pci_dev->vendor,
		pci_dev->device);

	hw_dev = hw_dev_register_device(pci_dev);
	if (!hw_dev)
	{
		printk("%s: ERROR: Failed to register PCI device\n", HW_DRV_NAME);
		return -1;
	}

	return 0;
}

static void hw_drv_pci_remove(struct pci_dev *pci_dev)
{
	hw_dev_t *hw_dev = (hw_dev_t *)pci_get_drvdata(pci_dev);

	if (hw_dev)
		hw_dev_unregister_device(hw_dev);
}

static void hw_drv_pci_shutdown(struct pci_dev *pci_dev)
{
	hw_dev_t *hw_dev = (hw_dev_t *)pci_get_drvdata(pci_dev);

	if (hw_dev)
		hw_dev_unregister_device(hw_dev);
}

static int hw_drv_pci_suspend(struct pci_dev *pci_dev, pm_message_t state)
{
	return 0;
}

static int hw_drv_pci_resume(struct pci_dev *pci_dev)
{
	return 0;
}

/*--------------------------------------------------------------------------------*/

#endif  /* HW_DRV_USE_PCI_DEVICES */

/*--------------------------------------------------------------------------------*/

/*
 *	 _____       _ _
 *	|_   _|     (_) |
 *	  | |  _ __  _| |_
 *	  | | | '_ \| | __|
 *	 _| |_| | | | | |_
 *	|_____|_| |_|_|\__|
 *
 */
static int hw_drv_chrdev_init(void)
{
	int ret;

	/* Allocate character device region */
	ret = alloc_chrdev_region(&hw_drv->chrdev_region, 0, 1, HW_DRV_NAME);
	if (ret)
	{
		printk("%s: ERROR: alloc_chrdev_region() failed\n", HW_DRV_NAME);
		return ret;		
	}

	/* Create device class */
	hw_drv->chrdev_class = class_create(THIS_MODULE, HW_DRV_NAME);
	if (!hw_drv->chrdev_class)
	{
		printk("%s: ERROR: class_create() failed\n", HW_DRV_NAME);
		kfree(hw_drv);
		return ret;
	}

	hw_drv->chrdev_major = MAJOR(hw_drv->chrdev_region);

	cdev_init(&hw_drv->chrdev, &__hw_dev_fops);

	hw_drv->chrdev.owner = THIS_MODULE;
	hw_drv->chrdev.ops   = &__hw_dev_fops;

	ret = cdev_add(
		&hw_drv->chrdev,
		MKDEV(hw_drv->chrdev_major, 0),
		1
	);

	if (ret)
	{
		class_destroy(hw_drv->chrdev_class);
		unregister_chrdev_region(hw_drv->chrdev_region, 1);

		printk("%s: ERROR: cdev_add() failed\n", HW_DRV_NAME);
		return ret;
	}

	device_create(
		hw_drv->chrdev_class,
		NULL,
		MKDEV(hw_drv->chrdev_major, 0),
		hw_drv,
		HW_DRV_NAME
	);

	printk("%s: Created character device \"%s\"\n",
		HW_DRV_NAME, HW_DRV_NAME);

	return 0;
}

int hw_drv_init(void)
{
	int ret;

	printk("%s: Hardware test driver started\n", HW_DRV_NAME);

	/* Initialize driver data structure */
	hw_drv = (hw_drv_t *)kzalloc(sizeof(hw_drv_t), GFP_KERNEL);
	if (!hw_drv)
		return -1;

	hw_drv->dev_count = 0;

	sema_init(&hw_drv->sem, 1); 

	INIT_LIST_HEAD(&hw_drv->list);
	rwlock_init(&hw_drv->list_lock);

	/* Register local device (no PCI) */
	hw_drv->hw_dev_local = hw_dev_register_device(NULL);
	if (!hw_drv->hw_dev_local)
	{
		printk(
			"%s: ERROR: Could not create local device\n",
			HW_DRV_NAME
		);

		kfree(hw_drv);
		return -1;
	}

	#if (HW_DRV_USE_PCI_DEVICES)
	{
		ret = pci_register_driver(&hw_drv_pci_driver);

		if (ret)
		{
			printk(
				"%s: ERROR: Could not register device driver\n",
				HW_DRV_NAME
			);

			kfree(hw_drv);
			return -1;
		}
	}
	#endif

	ret = hw_drv_chrdev_init();

	if (ret)
	{
		printk(
			"%s: ERROR: Could not create character device\n",
			HW_DRV_NAME
		);

		#if (HW_DRV_USE_PCI_DEVICES)
			pci_unregister_driver(&hw_drv_pci_driver);
		#endif

		kfree(hw_drv);
		return -1;
	}

	return ret;
}

/*--------------------------------------------------------------------------------*/

/*
 *	 ______      _ _
 *	|  ____|    (_) |
 *	| |__  __  ___| |_
 *	|  __| \ \/ / | __|
 *	| |____ >  <| | |_
 *	|______/_/\_\_|\__|
 *
 */
static int hw_drv_chrdev_destroy(void)
{
	device_destroy(
		hw_drv->chrdev_class,
		MKDEV(hw_drv->chrdev_major, 0)
	);

	cdev_del(&hw_drv->chrdev);
	class_destroy(hw_drv->chrdev_class);
	unregister_chrdev_region(hw_drv->chrdev_region, 1);

	return 0;
}

void hw_drv_exit(void)
{
	hw_drv_chrdev_destroy();

	#if (HW_DRV_USE_PCI_DEVICES)
		pci_unregister_driver(&hw_drv_pci_driver);
	#endif

	hw_dev_unregister_device(hw_drv->hw_dev_local);
	kfree(hw_drv);
}

/*--------------------------------------------------------------------------------*/

module_init(hw_drv_init);
module_exit(hw_drv_exit);

/*--------------------------------------------------------------------------------*/

/*
 *	 _____ ____   _____ _______ _
 *	|_   _/ __ \ / ____|__   __| |
 *	  | || |  | | |       | |  | |
 *	  | || |  | | |       | |  | |
 *	 _| || |__| | |____   | |  | |____
 *	|_____\____/ \_____|  |_|  |______|
 *
 */
static int __hw_dev_fop_flush(struct file *file, fl_owner_t id)
{
	if (down_interruptible(&hw_drv->sem))
		return -ERESTARTSYS;

	--hw_drv->chrdev_open_counter;

	up(&hw_drv->sem);

	return 0;
}

/*--------------------------------------------------------------------------------*/

int __hw_dev_fop_open(struct inode *inode, struct file *file)
{
	if (down_interruptible(&hw_drv->sem))
		return -ERESTARTSYS;

	++hw_drv->chrdev_open_counter;

	up(&hw_drv->sem);

	return 0;
}

/*--------------------------------------------------------------------------------*/

static unsigned int swap32(unsigned int v)
{
	v =  (((v >> 24) & 0xff) <<  0) |
	     (((v >> 16) & 0xff) <<  8) |
	     (((v >>  8) & 0xff) << 16) |
	     (((v >>  0) & 0xff) << 24);

	return (v);
}

/*--------------------------------------------------------------------------------*/

static int __memcpy_toio_from_user(
	volatile void __iomem     *to,
	const void __user         *from,
	unsigned long              size,
	const hw_drv_memcpy_type_t memcpy_type
)
{
	int errcode = 0;

	switch (memcpy_type)
	{
		case MEMCPY_TYPE_DEFAULT:
		{
			errcode = copy_from_user(
				(void *)to,
				from,
				size
			);

			break;
		}

		case MEMCPY_TYPE_IO:
		case MEMCPY_TYPE_IO_8:
		case MEMCPY_TYPE_IO_16:
		case MEMCPY_TYPE_IO_32:
		case MEMCPY_TYPE_IO_64:
		{
			void *tmp_buffer = kmalloc(size, GFP_KERNEL);
			if (!tmp_buffer)
			{
				errcode = -ENOMEM;
				break;
			}

			errcode = copy_from_user(tmp_buffer, from, size);
			if (errcode)
			{
				kfree(tmp_buffer);
				break;
			}

			if (memcpy_type == MEMCPY_TYPE_IO)
			{
				memcpy_toio(to, tmp_buffer, size);
			}
			else if (memcpy_type == MEMCPY_TYPE_IO_8)
			{
				u8 *p = tmp_buffer;
				unsigned long i;

				for (i = 0; i < size; i += sizeof(u8))
					iowrite8(*(p++), (void __iomem *)(to + i));
			}
			else if (memcpy_type == MEMCPY_TYPE_IO_16)
			{
				u16 *p = tmp_buffer;
				unsigned long i;

				for (i = 0; i < size; i += sizeof(u16))
					iowrite16(*(p++), (void __iomem *)(to + i));
			}
			else if (memcpy_type == MEMCPY_TYPE_IO_32)
			{
				u32 *p = tmp_buffer;
				unsigned long i;

				for (i = 0; i < size; i += sizeof(u32))
					iowrite32(*(p++), (void __iomem *)(to + i));
			}
			else if (memcpy_type == MEMCPY_TYPE_IO_64)
			{
				u64 *p = tmp_buffer;
				unsigned long i;

				for (i = 0; i < size; i += sizeof(u64))
					writeq(*(p++), (void __iomem *)(to + i));
			}

			kfree(tmp_buffer);
			break;
		}

		default:
		{
			errcode = -EINVAL;
			break;
		}
	}

	return errcode;
}

static int __memcpy_fromio_to_user(
	void __user                 *to,
	const volatile void __iomem *from,
	unsigned long                size,
	const hw_drv_memcpy_type_t   memcpy_type
)
{
	int errcode = 0;

	switch (memcpy_type)
	{
		case MEMCPY_TYPE_DEFAULT:
		{
			errcode = copy_to_user(
				to,
				(const void *)from,
				size
			);

			break;
		}

		case MEMCPY_TYPE_IO:
		case MEMCPY_TYPE_IO_8:
		case MEMCPY_TYPE_IO_16:
		case MEMCPY_TYPE_IO_32:
		case MEMCPY_TYPE_IO_64:
		{
			void *tmp_buffer = kmalloc(size, GFP_KERNEL);
			if (!tmp_buffer)
			{
				errcode = -ENOMEM;
				break;
			}

			if (memcpy_type == MEMCPY_TYPE_IO)
			{
				memcpy_fromio(tmp_buffer, from, size);
			}
			else if (memcpy_type == MEMCPY_TYPE_IO_8)
			{
				u8 *p = tmp_buffer;
				unsigned long i;

				for (i = 0; i < size; i += sizeof(u8))
					*(p++) = ioread8((void __iomem *)(from + i));
			}
			else if (memcpy_type == MEMCPY_TYPE_IO_16)
			{
				u16 *p = tmp_buffer;
				unsigned long i;

				for (i = 0; i < size; i += sizeof(u16))
					*(p++) = ioread16((void __iomem *)(from + i));
			}
			else if (memcpy_type == MEMCPY_TYPE_IO_32)
			{
				u32 *p = tmp_buffer;
				unsigned long i;

				for (i = 0; i < size; i += sizeof(u32))
					*(p++) = ioread32((void __iomem *)(from + i));
			}
			else if (memcpy_type == MEMCPY_TYPE_IO_64)
			{
				u64 *p = tmp_buffer;
				unsigned long i;

				for (i = 0; i < size; i += sizeof(u64))
					*(p++) = readq((void __iomem *)(from + i));
			}

			errcode = copy_to_user(to, tmp_buffer, size);
			if (errcode)
			{
				kfree(tmp_buffer);
				break;
			}

			kfree(tmp_buffer);
			break;
		}

		default:
		{
			errcode = -EINVAL;
			break;
		}
	}

	return errcode;
}

/*--------------------------------------------------------------------------------*/

long __hw_dev_fop_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int       ret = 0;
	hw_dev_t *hw_dev = 0;

	switch (cmd)
	{
		case HW_DRV_IOCTL_BAR_READ:
		{
			hw_drv_ioctl_bar_read_t bar_rd;
			
			ret = copy_from_user(
				&bar_rd,
				 (void *)arg,
				 sizeof(hw_drv_ioctl_bar_read_t)
			);

			if (ret)
				return -EINVAL;

			if (bar_rd.bar_id >= HW_DRV_RESOURCES_COUNT_MAX)
				return -EINVAL;

			hw_dev = hw_dev_find(bar_rd.dev_id);

			if (!hw_dev)
				return -ENODEV;

			mutex_lock(&hw_dev->lock);

			if ((bar_rd.offset + bar_rd.size) > hw_dev->res[bar_rd.bar_id].length)
			{
				mutex_unlock(&hw_dev->lock);
				return -EINVAL;
			}

			if (hw_dev->res[bar_rd.bar_id].cached)
			{
				/* Invalidate cache */
				dma_sync_single_range_for_cpu(
					NULL,
					(dma_addr_t)hw_dev->res[bar_rd.bar_id].base_phys,
					bar_rd.offset,
					bar_rd.size,
					DMA_FROM_DEVICE
				);
			}

			ret = __memcpy_fromio_to_user(
				bar_rd.user_ptr,
				(const volatile void *)((uintptr_t)
					hw_dev->res[bar_rd.bar_id].base_virt + bar_rd.offset),
				bar_rd.size,
				bar_rd.memcpy_type
			);

			mutex_unlock(&hw_dev->lock);

			break;
		}

		case HW_DRV_IOCTL_BAR_WRITE:
		{
			hw_dev_t *hw_dev;
			hw_drv_ioctl_bar_read_t bar_wr;
			
			ret = copy_from_user(
				&bar_wr,
				 (void *)arg,
				 sizeof(hw_drv_ioctl_bar_write_t)
			);

			if (ret)
				return -EINVAL;

			if (bar_wr.bar_id >= HW_DRV_RESOURCES_COUNT_MAX)
				return -EINVAL;

			hw_dev = hw_dev_find(bar_wr.dev_id);

			if (!hw_dev)
				return -ENODEV;

			mutex_lock(&hw_dev->lock);

			if ((bar_wr.offset + bar_wr.size) > hw_dev->res[bar_wr.bar_id].length)
			{
				mutex_unlock(&hw_dev->lock);
				return -EINVAL;
			}

			ret = __memcpy_toio_from_user(
				(volatile void *)((uintptr_t)
					hw_dev->res[bar_wr.bar_id].base_virt + bar_wr.offset),
				bar_wr.user_ptr,
				bar_wr.size,
				bar_wr.memcpy_type
			);

			if (hw_dev->res[bar_wr.bar_id].cached)
			{
				/* Write back cache */
				dma_sync_single_range_for_device(
					NULL,
					(dma_addr_t)hw_dev->res[bar_wr.bar_id].base_phys,
					bar_wr.offset,
					bar_wr.size,
					DMA_TO_DEVICE
				);
			}

			mutex_unlock(&hw_dev->lock);

			break;
		}

		case HW_DRV_IOCTL_LOCAL_READ:
		{
			hw_drv_ioctl_local_read_t loc_rd;

			ret = copy_from_user(
				&loc_rd,
				 (void *)arg,
				 sizeof(hw_drv_ioctl_local_read_t)
			);

			if (ret)
				return -EINVAL;

			hw_dev = hw_dev_find(loc_rd.dev_id);

			if (!hw_dev)
				return -ENODEV;

			mutex_lock(&hw_dev->lock);

			if ((loc_rd.offset + loc_rd.size) > hw_dev->mem_size)
			{
				mutex_unlock(&hw_dev->lock);
				return -EINVAL;
			}

			ret = copy_to_user(
				loc_rd.user_ptr,
				(void *)((uintptr_t)hw_dev->mem_base + loc_rd.offset),
				loc_rd.size
			);

			mutex_unlock(&hw_dev->lock);

			break;
		}

		case HW_DRV_IOCTL_LOCAL_FIND:
		{
			unsigned int *u32ptr;
			unsigned int  i;

			hw_drv_ioctl_local_find_t find;
			
			ret = copy_from_user(
				&find,
				 (void *)arg,
				 sizeof(hw_drv_ioctl_local_find_t)
			);

			if (ret)
				return -EINVAL;

			hw_dev = hw_dev_find(find.dev_id);

			if (!hw_dev)
				return -ENODEV;

			mutex_lock(&hw_dev->lock);

 			u32ptr = (unsigned int *)((uintptr_t)hw_dev->mem_base);

			find.finded = 0;

			for (i = 0; i < (hw_dev->mem_size / sizeof(unsigned int)); i++)
			{
				if ((u32ptr[i] == find.value) ||
				    (swap32(u32ptr[i]) == find.value))
				{
					find.finded        = 1;
					find.finded_offset = i * sizeof(unsigned int);
					find.finded_value  = u32ptr[i];
					break;
				}
			}

			ret = copy_to_user(
				 (void *)arg,
				&find,
				 sizeof(hw_drv_ioctl_local_find_t)
			);

			mutex_unlock(&hw_dev->lock);

			break;
		}

		case HW_DRV_IOCTL_LOCAL_FILL:
		{
			hw_drv_ioctl_local_fill_t fill;
			
			unsigned int *u32ptr;
			unsigned int i;

			ret = copy_from_user(
				&fill,
				 (void *)arg,
				 sizeof(hw_drv_ioctl_local_fill_t)
			);

			if (ret)
				return -EINVAL;

			hw_dev = hw_dev_find(fill.dev_id);

			if (!hw_dev)
				return -ENODEV;

			mutex_lock(&hw_dev->lock);

			u32ptr = (unsigned int *)((uintptr_t)hw_dev->mem_base);

			for (i = 0; i < (hw_dev->mem_size / sizeof(unsigned int)); i++)
				u32ptr[i] = fill.value;

			mutex_unlock(&hw_dev->lock);

			break;
		}

		default:
		{
			printk(
				"%s: ERROR: Invalid IOCTL command 0x%08X\n",
				HW_DRV_NAME, cmd
			);

			ret = -1;
			break;
		}
	}

	return ret;
}

/*--------------------------------------------------------------------------------*/
