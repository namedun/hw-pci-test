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

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <sys/ioctl.h>

#include <linux/random.h>
#include <sys/syscall.h>

#include "gnu_getopt/gnu_getopt.h" /* GETOPT library */
#include "../module/hw_drv.h"

/*--------------------------------------------------------------------------------*/

#define APP_NAME "hw_test"

/*--------------------------------------------------------------------------------*/

#ifndef EOK
#define EOK 0
#endif

/*--------------------------------------------------------------------------------*/

static int cmd_wr      (int fhnd);
static int cmd_rd      (int fhnd);
static int cmd_wrrd    (int fhnd);
static int cmd_rw_test (int fhnd);

#define LINE_PRFX "    "

#define CMD_REQUIRED_BAR_OPT  0x1

/* Commands */
static struct cmd
{
	char        *name;
	unsigned int flags;
	int        (*func)(int fhnd);
	char        *help;

} cmds[] =
{
	{
		.name  = "wr",
		.flags = CMD_REQUIRED_BAR_OPT,
		.func  = cmd_wr,
		.help  =
			LINE_PRFX "Write (--size) bytes of data to BAR (--bar) with\n"
			LINE_PRFX "(--offset) bytes. Data is filled by 32-bit (--value).\n"
	},
	{
		.name  = "rd",
		.flags = CMD_REQUIRED_BAR_OPT,
		.func  = cmd_rd,
		.help  =
			LINE_PRFX "Read (--size) bytes from BAR (--bar) with (--offset) bytes.\n"
	},
	{
		.name  = "wrrd",
		.flags = CMD_REQUIRED_BAR_OPT,
		.func  = cmd_wrrd,
		.help  =
			LINE_PRFX "Write (--size) bytes of counter data to BAR (--bar) with\n"
			LINE_PRFX "(--offset) bytes, read writed data back and validate.\n"
	},
	{
		.name  = "rw_test",
		.flags = CMD_REQUIRED_BAR_OPT,
		.func  = cmd_rw_test,
		.help  =
			LINE_PRFX "Write/read test (--size can be 4, 8 or 16 bytes).\n"
	},
};

/*--------------------------------------------------------------------------------*/

static int __arg_apply(
	const char  option,
	const int   option_index,
	const char *optarg
);

/*--------------------------------------------------------------------------------*/

static void print_usage(const char *app)
{
	int i;

	printf(
		"\r\n"
		"Usage: %s <options>\r\n"
		"\r\n"
		"Options:\r\n"
		"\r\n"
		"  --dev=n,    -d n   : Device identifier (0 - local)\r\n"
		"  --cmd=n,    -c n   : Command\r\n"
		"  --bar=n,    -b n   : Device BAR number\r\n"
		"  --size=n,   -s n   : Operation size in bytes\r\n"
		"  --offset=n, -o n   : Operation offset in bytes\r\n"
		"  --value=n,  -v n   : Operation value (32-bit)\r\n"
		"  --r_size=n, -r n   : Operation read block size in bytes\r\n"
		"  --w_size=n, -w n   : Operation write block size in bytes\r\n"
		"  --memcpy=n, -m n   : Memory copy type (default, io, io8, io16, io32, io64)\r\n"
		"\r\n"
		"Commnands:\r\n"
		"\r\n",
		app
	);

	for (i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++)
	{
		printf("  %s:\r\n", cmds[i].name);
		printf("%s\r\n", cmds[i].help);
	}
}

/* Arguments */
static struct
{
	struct cmd          *cmd;         /* --cmd    (required) */
	int                  dev_id;      /* --dev    (required) */
	int                  bar;         /* --bar    (optional) */
	unsigned int         size;        /* --size   (optional) */
	unsigned int         offset;      /* --offset (optional) */
	unsigned int         value;       /* --value  (optional) */
	unsigned int         r_size;      /* --r_size (optional) */
	unsigned int         w_size;      /* --w_size (optional) */
	hw_drv_memcpy_type_t memcpy_type; /* --memcpy (optional) */

} args;

/*--------------------------------------------------------------------------------*/

static const char short_options[] = "c:d:b:s:o:v:r:w:hm:";

static const struct option long_options[] =
{
	{ "help",   no_argument,       NULL, 'h', "Display help" },
	{ "cmd",    required_argument, NULL, 'c', "Command" },
	{ "dev",    required_argument, NULL, 'd', "Device ID" },
	{ "bar",    required_argument, NULL, 'b', "BAR ID" },
	{ "size",   required_argument, NULL, 's', "Size (bytes)" },
	{ "offset", required_argument, NULL, 'o', "Offset (bytes)" },
	{ "value",  required_argument, NULL, 'v', "Value" },
	{ "r_size", required_argument, NULL, 'r', "Read block size (bytes)" },
	{ "w_size", required_argument, NULL, 'w', "Write block size (bytes)" },
	{ "memcpy", required_argument, NULL, 'm', "Memory copy type" },
	{  NULL,    no_argument,       NULL,  0,   NULL }
};

/*--------------------------------------------------------------------------------*/

/**
 *	Main function
 */
int main(int argc, char *argv[])
{
	int status = EOK;

	int drv_fhnd = -1;

	/* Default arguments */
	args.cmd         =  NULL;
	args.dev_id      = -1;
	args.bar         = -1;                    /* BAR0 */
	args.size        =  sizeof(unsigned int); /* 32-bit */
	args.offset      =  0;                    /* 0 */
	args.value       =  0xAA55AA55;
	args.r_size      =  0;                    /* auto ( == size) */
	args.w_size      =  0;                    /* auto ( == size) */
	args.memcpy_type =  MEMCPY_TYPE_IO_32;

	/* Parse arguments */
	while (1)
	{
		int  option_index;
		char option;

		option = gnu_getopt_long(
			 argc,
			 (char * const *)argv,
			 short_options,
			 long_options,
			&option_index
		);

		if (option == EOF)
			break;

		status = __arg_apply(
			option, option_index, gnu_optarg
		);

		if (status)
			return status;
	}

	if (args.dev_id < 0)
	{
		status = -EINVAL;
		printf("ERROR: Unspecified device ID (--dev)\n");
	}
	
	if (args.cmd == NULL)
	{
		status = -EINVAL;
		printf("ERROR: Unspecified command (--cmd)\n");
	}

	if (args.cmd->flags & CMD_REQUIRED_BAR_OPT)
	{
		if (args.bar < 0)
		{
			status = -EINVAL;

			printf(
				"ERROR: For the \"%s\" command, specify device BAR option (--bar)\n",
				args.cmd->name
			);
		}
	}

	if (status)
		return status;
	
	drv_fhnd = open("/dev/" HW_DRV_CHRDEV_NAME, O_RDWR);

	if (drv_fhnd < 0)
	{
		printf("ERROR: Can't open device\n");
		return -ENODEV;
	}

	status = args.cmd->func(drv_fhnd);
	close(drv_fhnd);
	
	return status;
}

/*--------------------------------------------------------------------------------*/

static int __arg_apply(
	const char  option,
	const int   option_index,
	const char *optarg
)
{
	switch(option)
	{
		case 'h': /* --help */
		{
			print_usage(APP_NAME);

			/* Exit from application */
			exit(EOK);
		}

		case 'c': /* --cmd */
		{
			if (optarg)
			{
				unsigned int i;
				struct cmd *cmd = NULL;

				for (i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++)
				{
					if (strcmp(cmds[i].name, optarg) == 0)
					{
						cmd = &cmds[i];
						break;
					}
				}

				if (!cmd)
				{
					printf("ERROR: Unknown command \"%s\"\r\n", optarg);
					return -EINVAL;
				}

				args.cmd = cmd;
			}

			break;
		}

		case 'm': /* --memcpy */
		{
			if (optarg)
			{
				if (strcmp(optarg, "default") == 0)
					args.memcpy_type = MEMCPY_TYPE_DEFAULT;
				else if (strcmp(optarg, "io") == 0)
					args.memcpy_type = MEMCPY_TYPE_IO;
				else if (strcmp(optarg, "io8") == 0)
					args.memcpy_type = MEMCPY_TYPE_IO_8;
				else if (strcmp(optarg, "io16") == 0)
					args.memcpy_type = MEMCPY_TYPE_IO_16;
				else if (strcmp(optarg, "io32") == 0)
					args.memcpy_type = MEMCPY_TYPE_IO_32;
				else if (strcmp(optarg, "io64") == 0)
					args.memcpy_type = MEMCPY_TYPE_IO_64;
				else
				{
					printf("ERROR: Invalid memory copy type \"%s\"\r\n", optarg);
					return -EINVAL;
				}
			}

			break;
		}

		case 'd': /* --dev */
		{
			if (optarg)
				args.dev_id = strtoul(optarg, NULL, 0);

			break;
		}

		case 's': /* --size */
		{
			if (optarg)
				args.size = strtoul(optarg, NULL, 0);

			break;
		}

		case 'o': /* --offset */
		{
			if (optarg)
				args.offset = strtoul(optarg, NULL, 0);

			break;
		}

		case 'b': /* --bar */
		{
			if (optarg)
				args.bar = strtoul(optarg, NULL, 0);

			break;
		}

		case 'v': /* --value */
		{
			if (optarg)
				args.value = strtoul(optarg, NULL, 0);

			break;
		}

		case 'r': /* --r_size */
		{
			if (optarg)
				args.r_size = strtoul(optarg, NULL, 0);

			break;
		}

		case 'w': /* --w_size */
		{
			if (optarg)
				args.w_size = strtoul(optarg, NULL, 0);

			break;
		}

		default:
			return -1;
	}

	return 0;
}

/*--------------------------------------------------------------------------------*/

static void memory_dump(
	const void        *address,
	const unsigned int count,
	const int          show_offset
)
{
	int i;

	const unsigned int *p =
		(const unsigned int *)address;

	unsigned int count_aligned =
		count & 0xFFFFFFFC;

	printf(
		"\nMemory dump: %d bytes from %llx:",
		count_aligned,
		(long long unsigned int)(uintptr_t)address
	);

	for (i = 0; i < count_aligned / 4; i++, p++)
	{
		if (i % 4 == 0)
		{
			if (show_offset)
			{
				printf("\n0x%08lx: ",
					(unsigned long)(uintptr_t)p -
					(unsigned long)(uintptr_t)address
				);
			}
			else
			{
				printf("\n0x%08lx: ",
					(unsigned long)(uintptr_t)p);
			}
		}

		printf("%08x ", *p);
	}

	printf("\n\n");
}

/*--------------------------------------------------------------------------------*/

static int cmd_rd(int fhnd)
{
	int status;

	hw_drv_ioctl_bar_read_t bar_rd;

	bar_rd.memcpy_type = args.memcpy_type;
	bar_rd.dev_id      = args.dev_id;
	bar_rd.bar_id      = args.bar;
	bar_rd.size        = args.size;
	bar_rd.offset      = args.offset;
	
	bar_rd.user_ptr = (void *)malloc(bar_rd.size);
	if (!bar_rd.user_ptr)
	{
		printf("ERROR: Can't allocate %d bytes\n", bar_rd.size);
		return -ENOMEM;
	}

	printf("Reading from BAR%d of DEV%d:\n",
		bar_rd.bar_id, bar_rd.dev_id);

	printf("  Offset = %d bytes\n",
		bar_rd.offset);

	printf("  Size   = %lu DWORD's (%d bytes)\n",
		bar_rd.size / sizeof(unsigned int), bar_rd.size);

	status = ioctl(fhnd, HW_DRV_IOCTL_BAR_READ, &bar_rd);

	if (status)
	{
		printf(
			"ERROR: HW_DRV_IOCTL_BAR_READ failed "
			"(errcode = %d)\n",
			status
		);

		free(bar_rd.user_ptr);
		return status;
	}

	printf("Success\n");

	memory_dump(bar_rd.user_ptr, bar_rd.size, 1);
	free(bar_rd.user_ptr);

	return status;
}

/*--------------------------------------------------------------------------------*/

static int cmd_wr(int fhnd)
{
	int status;

	unsigned int  value = 0;
	unsigned int *u32ptr;
	unsigned int  i;

	hw_drv_ioctl_bar_write_t bar_wr;

	bar_wr.memcpy_type = args.memcpy_type;
	bar_wr.dev_id      = args.dev_id;
	bar_wr.bar_id      = args.bar;
	bar_wr.size        = args.size;
	bar_wr.offset      = args.offset;

	value = args.value;

	bar_wr.user_ptr = (void *)malloc(bar_wr.size);
	if (!bar_wr.user_ptr)
	{
		printf("ERROR: Can't allocate %d bytes\n", bar_wr.size);
		return -ENOMEM;
	}

	u32ptr = (unsigned int *)bar_wr.user_ptr;

	for (i = 0; i < bar_wr.size / sizeof(unsigned int); i++)
		u32ptr[i] = value;

	printf("Writing to BAR%d of DEV%d:\n",
		bar_wr.bar_id, bar_wr.dev_id);

	printf("  Offset = %d bytes\n",
		bar_wr.offset);

	printf("  Size   = %lu DWORD's (%d bytes)\n",
		bar_wr.size / sizeof(unsigned int), bar_wr.size);

	printf("  Value  = 0x%08x\n", value);

	status = ioctl(fhnd, HW_DRV_IOCTL_BAR_WRITE, &bar_wr);

	if (status)
	{
		printf(
			"ERROR: HW_DRV_IOCTL_BAR_WRITE failed "
			"(errcode = %d)\n",
			status
		);

		free(bar_wr.user_ptr);
		return status;
	}

	printf("Success\n");
	free(bar_wr.user_ptr);
	return status;
}

/*--------------------------------------------------------------------------------*/

static int cmd_wrrd(int fhnd)
{
	int status;

	hw_drv_ioctl_bar_write_t wr;
	hw_drv_ioctl_bar_read_t  rd;

	void *buffer;

	unsigned int *u32ptr;
	unsigned int  i;
	unsigned int  counter_start = 0;
	unsigned int  counter;
	unsigned int  counter_fail = 0;

	wr.memcpy_type = args.memcpy_type;
	wr.dev_id      = args.dev_id;
	wr.bar_id      = args.bar;
	wr.size        = args.size;
	wr.offset      = args.offset;

	buffer = (void *)malloc(wr.size);
	if (!buffer)
	{
		printf("ERROR: Can't allocate %d bytes\n", wr.size);
		return -ENOMEM;
	}

	wr.user_ptr = buffer;

	rd.memcpy_type = wr.memcpy_type;
	rd.dev_id      = wr.dev_id;
	rd.bar_id      = wr.bar_id;
	rd.size        = wr.size;
	rd.offset      = wr.offset;
	rd.user_ptr    = wr.user_ptr;

	u32ptr = (unsigned int *)wr.user_ptr;

	counter = counter_start;
	for (i = 0; i < wr.size / sizeof(unsigned int); i++)
		u32ptr[i] = counter++;

	printf("Writing counter to BAR%d of DEV%d:\n",
		wr.bar_id, wr.dev_id);

	printf("  Offset = %d bytes\n", wr.offset);
	printf("  Size   = %lu DWORD's (%d bytes)\n",
		wr.size / sizeof(unsigned int), wr.size);

	status = ioctl(fhnd, HW_DRV_IOCTL_BAR_WRITE, &wr);

	if (status)
	{
		printf(
			"ERROR: HW_DRV_IOCTL_BAR_WRITE failed "
			"(errcode = %d)\n",
			status
		);

		free(buffer);
		return status;
	}

	u32ptr = (unsigned int *)rd.user_ptr;
	memset(u32ptr, 0, rd.size);

	printf("Reading counter from BAR%d of DEV%d:\n",
		rd.bar_id, rd.dev_id);

	printf("  Offset = %d bytes\n", rd.offset);
	printf("  Size   = %lu DWORD's (%d bytes)\n",
		rd.size / sizeof(unsigned int), rd.size);

	status = ioctl(fhnd, HW_DRV_IOCTL_BAR_READ, &rd);

	if (status)
	{
		printf(
			"ERROR: HW_DRV_IOCTL_BAR_READ failed "
			"(errcode = %d)\n",
			status
		);

		free(buffer);
		return status;
	}

	printf("Checking counter...\n");

	counter = counter_start;
	for (i = 0; i < rd.size / sizeof(unsigned int); i++)
	{
		if (u32ptr[i] != counter)
		{
			counter_fail = 1;
			printf(
				"FAILED: Counter value with offset %lu bytes is 0x%08x "
				"(expected 0x%08x)\n",
				i * sizeof(unsigned int),
				u32ptr[i], counter
			);
		}

		counter++;
	}

	if (counter_fail)
		printf("FAILED: Readed counter corrupted\n");

	printf("Success\n");
	free(buffer);
	return status;

}

/*--------------------------------------------------------------------------------*/

static unsigned int __rand_seed = 1;
static unsigned int __rand_a    = 0x41C64E6D;
static unsigned int __rand_c    = 0x00003039;

static void __rand32_init_seed(const unsigned int seed)
{
	__rand_seed = seed;
}

static void __rand32_init(void)
{
	unsigned int new_seed;

	syscall(SYS_getrandom, &new_seed, sizeof(new_seed), 0);
	__rand32_init_seed(new_seed);
}

static unsigned int __rand32(void)
{
	__rand_seed = __rand_seed * __rand_a + __rand_c;
	return __rand_seed;
}

/*--------------------------------------------------------------------------------*/

#define RW_TEST_MAX_SIZE           4

#define RW_TEST_RND_VALUES         64
#define RW_TEST_FIXED_VALUES       32
#define RW_TEST_TOTAL_VALUES      (RW_TEST_RND_VALUES + RW_TEST_FIXED_VALUES)

#define RW_TEST_SHOW_OK_VALUES     0
#define RW_TEST_SHOW_ERROR_BYTES   1
#define RW_TEST_ERROR_BYTE_SYMBOL '^'
#define RW_TEST_VALID_BYTE_SYMBOL ' '

/*--------------------------------------------------------------------------------*/

static int cmd_rw_test(int fhnd)
{
	int status;

	unsigned int size;
	unsigned int r_size = 0;
	unsigned int w_size = 0;
	unsigned int v_size = 0;

	unsigned int errors = 0;
	unsigned int i;
	unsigned int j;
	unsigned int readed_buffer[RW_TEST_MAX_SIZE] = { 0, 0, 0, 0 };

	static unsigned int test_values[RW_TEST_TOTAL_VALUES][RW_TEST_MAX_SIZE] =
	{
		{ 0x00000000, 0x00000000, 0x00000000, 0x00000000 }, /* 0 */
		{ 0x12345678, 0x12345678, 0x12345678, 0x12345678 },
		{ 0x11111111, 0x11111111, 0x11111111, 0x11111111 },
		{ 0x55555555, 0x55555555, 0x55555555, 0x55555555 },
		{ 0xAAAAAAAA, 0xAAAAAAAA, 0xAAAAAAAA, 0xAAAAAAAA },
		{ 0xAA55AA55, 0xAA55AA55, 0xAA55AA55, 0xAA55AA55 }, /* 5 */
		{ 0x55AA55AA, 0x55AA55AA, 0x55AA55AA, 0x55AA55AA },
		{ 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF },
		{ 0x5555AAAA, 0x5555AAAA, 0x5555AAAA, 0x5555AAAA },
		{ 0xAAAA5555, 0xAAAA5555, 0xAAAA5555, 0xAAAA5555 },
		{ 0x00000000, 0x00000000, 0x00000000, 0x00000000 }, /* 10 */
		{ 0x11111111, 0x11111111, 0x11111111, 0x11111111 },
		{ 0x22222222, 0x22222222, 0x22222222, 0x22222222 },
		{ 0x33333333, 0x33333333, 0x33333333, 0x33333333 },
		{ 0x44444444, 0x44444444, 0x44444444, 0x44444444 },
		{ 0x55555555, 0x55555555, 0x55555555, 0x55555555 }, /* 15 */
		{ 0x66666666, 0x66666666, 0x66666666, 0x66666666 },
		{ 0x77777777, 0x77777777, 0x77777777, 0x77777777 },
		{ 0x88888888, 0x88888888, 0x88888888, 0x88888888 },
		{ 0x99999999, 0x99999999, 0x99999999, 0x99999999 },
		{ 0xAAAAAAAA, 0xAAAAAAAA, 0xAAAAAAAA, 0xAAAAAAAA }, /* 20 */
		{ 0xBBBBBBBB, 0xBBBBBBBB, 0xBBBBBBBB, 0xBBBBBBBB },
		{ 0xCCCCCCCC, 0xCCCCCCCC, 0xCCCCCCCC, 0xCCCCCCCC },
		{ 0xDDDDDDDD, 0xDDDDDDDD, 0xDDDDDDDD, 0xDDDDDDDD },
		{ 0xEEEEEEEE, 0xEEEEEEEE, 0xEEEEEEEE, 0xEEEEEEEE },
		{ 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF }, /* 25 */
		{ 0x10101010, 0x10101010, 0x10101010, 0x10101010 },
		{ 0x01010101, 0x01010101, 0x01010101, 0x01010101 },
		{ 0x87654321, 0x87654321, 0x87654321, 0x87654321 },
		{ 0x12121212, 0x12121212, 0x12121212, 0x12121212 },
		{ 0xF0F0F0F0, 0xF0F0F0F0, 0xF0F0F0F0, 0xF0F0F0F0 }, /* 30 */
		{ 0x0F0F0F0F, 0x0F0F0F0F, 0x0F0F0F0F, 0x0F0F0F0F }, /* 31 */
	};

	hw_drv_ioctl_bar_write_t wr;
	hw_drv_ioctl_bar_read_t  rd;

	size = args.size / sizeof(unsigned int);

	if ((size < 1) || (size > RW_TEST_MAX_SIZE))
	{
		printf("ERROR: Invalid size\n");
		return -EINVAL;
	}

	v_size = sizeof(unsigned int) * size;

	wr.memcpy_type = args.memcpy_type;
	wr.dev_id      = args.dev_id;
	wr.bar_id      = args.bar;
	wr.size        = v_size;
	wr.offset      = args.offset;
	wr.user_ptr    = NULL;

	rd.memcpy_type = wr.memcpy_type;
	rd.dev_id      = wr.dev_id;
	rd.bar_id      = wr.bar_id;
	rd.size        = wr.size;
	rd.offset      = wr.offset;
	rd.user_ptr    = NULL;

	r_size = args.r_size;
	w_size = args.w_size;

	if (r_size == 0)
		r_size = rd.size;

	if (w_size == 0)
		w_size = wr.size;

	if ((r_size > rd.size) || ((rd.size % r_size) != 0))
	{
		printf("ERROR: Invalid r-size\n");
		return -EINVAL;
	}

	if ((w_size > wr.size) || ((wr.size % w_size) != 0))
	{
		printf("ERROR: Invalid w-size\n");
		return -EINVAL;
	}

	rd.size = r_size;
	wr.size = w_size;

	printf("Started test for %d %d-bit value(s):\n",
		RW_TEST_TOTAL_VALUES, size * 32);

	printf("  BAR:    %d\n"      , rd.bar_id);
	printf("  Offset: %d bytes\n", rd.offset);
	printf("  V-size: %d bytes\n", v_size);
	printf("  R-size: %d bytes\n", rd.size);
	printf("  W-size: %d bytes\n", wr.size);

	__rand32_init();

	/* Generate random test values */
	for (i = 0; i < RW_TEST_RND_VALUES; i++)
	{
		for (j = 0; j < RW_TEST_MAX_SIZE; j++)
			test_values[RW_TEST_FIXED_VALUES + i][j] = __rand32();
	}

	for (i = 0; i < RW_TEST_TOTAL_VALUES; i++)
	{
		int valid = 1;

		uintptr_t off = 0;

		wr.offset = args.offset;
		rd.offset = args.offset;

		/* Write data */
		for (off = 0; off < v_size; off += w_size)
		{
			wr.user_ptr = (void *)((uintptr_t)&test_values[i][0] + off);
			status = ioctl(fhnd, HW_DRV_IOCTL_BAR_WRITE, &wr);

			if (status)
			{
				printf(
					"ERROR: HW_DRV_IOCTL_BAR_WRITE failed "
					"(errcode = %d)\n",
					status
				);

				return status;
			}

			wr.offset += w_size;
		}

		/* Read data */
		for (off = 0; off < v_size; off += r_size)
		{
			rd.user_ptr = (void *)((uintptr_t)&readed_buffer[0] + off);
			status = ioctl(fhnd, HW_DRV_IOCTL_BAR_READ, &rd);

			if (status)
			{
				printf(
					"ERROR: HW_DRV_IOCTL_BAR_READ failed "
					"(errcode = %d)\n",
					status
				);

				return status;
			}

			rd.offset += r_size;
		}

		/* Validate readed data */
		valid = !memcmp(
			&readed_buffer[0],
			(void *)&test_values[i][0],
			size * sizeof(readed_buffer[0])
		);

		if (valid)
		{
			/* Readed data is valid */
			#if (RW_TEST_SHOW_OK_VALUES)
				printf("\n%3d: OK:     ", i);
			#endif
		}
		else
		{
			/* Readed data is invalid */
			errors++;
			printf("\n%3d: FAILED: ", i);
		}

		if (RW_TEST_SHOW_OK_VALUES || !valid)
		{
			printf("Writed: 0x%08x", test_values[i][0]);

			for (j = 1; j < size; j++)
				printf(" %08x", test_values[i][j]);

			printf("\n             Readed: 0x%08x", readed_buffer[0]);

			for (j = 1; j < size; j++)
				printf(" %08x", readed_buffer[j]);

			if (!valid)
			{
				printf(" <-- (%d)", errors);

				#if (RW_TEST_SHOW_ERROR_BYTES)
				{
					unsigned char *rd = (unsigned char *)&readed_buffer[0];
					unsigned char *wr = (unsigned char *)&test_values[i][0];

					unsigned int byte;

					printf("\n                       ");

					for (byte = 0; byte < v_size; byte++)
					{
						unsigned int swapped_byte;
						
						swapped_byte = (byte & (~0x03)) + 3 - (byte & 0x03);

						if (byte && !(byte % 4))
							putc(' ', stdout);

						printf("%c%c",
							(rd[swapped_byte] & 0xF0) == (wr[swapped_byte] & 0xF0)
								? RW_TEST_VALID_BYTE_SYMBOL
								: RW_TEST_ERROR_BYTE_SYMBOL,
							(rd[swapped_byte] & 0x0F) == (wr[swapped_byte] & 0x0F)
								? RW_TEST_VALID_BYTE_SYMBOL
								: RW_TEST_ERROR_BYTE_SYMBOL
						);
					}
				}
				#endif
			}

			printf("\n");
		}
	}

	if (errors)
	{
		printf("FAILED: %d error(s)\n", errors);
		return -EBADMSG;
	}

	printf("Success\n");
	return status;
}

/*--------------------------------------------------------------------------------*/

