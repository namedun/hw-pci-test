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

#ifndef __HW_DRV_H__
#define __HW_DRV_H__

#ifndef HW_DRV_USE_PCI_DEVICES
#define HW_DRV_USE_PCI_DEVICES  1
#endif

#define HW_DRV_MAX_DEVICES  16

#define HW_DRV_NAME        "hw_test"
#define HW_DRV_CHRDEV_NAME  HW_DRV_NAME

#if (HW_DRV_USE_PCI_DEVICES)

	#define HW_DRV_PCI_DEVICES_IDS \
		{ PCI_DEVICE(0x1999, 0x7014), }, \
		{ PCI_DEVICE(0x10ee, 0x8024), }, \
		{ PCI_DEVICE(0x104c, 0xb005), },

#endif

/*--------------------------------------------------------------------------------*/

#define HW_DRV_IOCTL_MAGIC  'X'

/*--------------------------------------------------------------------------------*/

typedef enum
{
	MEMCPY_TYPE_DEFAULT,
	MEMCPY_TYPE_IO,
	MEMCPY_TYPE_IO_8,
	MEMCPY_TYPE_IO_16,
	MEMCPY_TYPE_IO_32,
	MEMCPY_TYPE_IO_64
	
} hw_drv_memcpy_type_t;

typedef struct
{
	unsigned int dev_id;

	unsigned int value;
	unsigned int finded;
	unsigned int finded_offset;
	unsigned int finded_value;

} hw_drv_ioctl_local_find_t;

#define HW_DRV_IOCTL_LOCAL_FIND \
	_IOC( \
		_IOC_WRITE | _IOC_READ, \
		HW_DRV_IOCTL_MAGIC, 1, \
		sizeof(hw_drv_ioctl_local_find_t *) \
	)

/*--------------------------------------------------------------------------------*/

typedef struct
{
	unsigned int dev_id;
	unsigned int offset;
	unsigned int size;

	void        *user_ptr;

} hw_drv_ioctl_local_read_t;

#define HW_DRV_IOCTL_LOCAL_READ \
	_IOC( \
		_IOC_WRITE, \
		HW_DRV_IOCTL_MAGIC, 2, \
		sizeof(hw_drv_ioctl_local_read_t *) \
	)

/*--------------------------------------------------------------------------------*/

typedef struct
{
	unsigned int dev_id;
	unsigned int value;

} hw_drv_ioctl_local_fill_t;

#define HW_DRV_IOCTL_LOCAL_FILL \
	_IOC( \
		_IOC_WRITE, \
		HW_DRV_IOCTL_MAGIC, 3, \
		sizeof(hw_drv_ioctl_local_fill_t *) \
	)

/*--------------------------------------------------------------------------------*/

typedef struct
{
	unsigned int dev_id;
	unsigned int bar_id;

	unsigned int offset;
	unsigned int size;

	void        *user_ptr;

	hw_drv_memcpy_type_t memcpy_type;

} hw_drv_ioctl_bar_read_t;

#define HW_DRV_IOCTL_BAR_READ \
	_IOC( \
		_IOC_WRITE, \
		HW_DRV_IOCTL_MAGIC, 4, \
		sizeof(hw_drv_ioctl_bar_read_t *) \
	)

/*--------------------------------------------------------------------------------*/

typedef struct
{
	unsigned dev_id;
	unsigned bar_id;

	unsigned int offset;
	unsigned int size;

	void        *user_ptr;

	hw_drv_memcpy_type_t memcpy_type;

} hw_drv_ioctl_bar_write_t;

#define HW_DRV_IOCTL_BAR_WRITE \
	_IOC( \
		_IOC_WRITE, \
		HW_DRV_IOCTL_MAGIC, 5, \
		sizeof(hw_drv_ioctl_bar_write_t *) \
	)

/*--------------------------------------------------------------------------------*/

#endif /* __HW_DRV_H__ */
