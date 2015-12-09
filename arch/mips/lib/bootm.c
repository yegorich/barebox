#include <boot.h>
#include <common.h>
#include <libfile.h>
#include <init.h>
#include <fs.h>
#include <errno.h>
#include <binfmt.h>
#include <restart.h>
#include <memory.h>
#include <linux/sizes.h>

#include <asm/byteorder.h>

#define LINUX_MAX_ENVS          256
#define LINUX_MAX_ARGS          256

static int	linux_argc;
static char **	linux_argv;

static char **	linux_env;
static char *	linux_env_p;
static int	linux_env_idx;

static void linux_env_set (char *env_name, char *env_val)
{
        if (linux_env_idx < LINUX_MAX_ENVS - 1) {
                linux_env[linux_env_idx] = linux_env_p;

                strcpy (linux_env_p, env_name);
                linux_env_p += strlen (env_name);

                strcpy (linux_env_p, "=");
                linux_env_p += 1;

                strcpy (linux_env_p, env_val);
                linux_env_p += strlen (env_val);

                linux_env_p++;
                linux_env[++linux_env_idx] = 0;
        }
}

static void linux_params_init (ulong start, char *line)
{
	char *next, *quote, *argp;

	linux_argc = 1;
	linux_argv = (char **) start;
	linux_argv[0] = 0;
	argp = (char *) (linux_argv + LINUX_MAX_ARGS);

	next = line;

	while (line && *line && linux_argc < LINUX_MAX_ARGS) {
		quote = strchr (line, '"');
		next = strchr (line, ' ');

		while (next != NULL && quote != NULL && quote < next) {
			/* we found a left quote before the next blank
			 * now we have to find the matching right quote
			 */
			next = strchr (quote + 1, '"');
			if (next != NULL) {
				quote = strchr (next + 1, '"');
				next = strchr (next + 1, ' ');
			}
		}

		if (next == NULL) {
			next = line + strlen (line);
		}

		linux_argv[linux_argc] = argp;
		memcpy (argp, line, next - line);
		argp[next - line] = 0;

		argp += next - line + 1;
		linux_argc++;

		if (*next)
			next++;

		line = next;
	}

	linux_env = (char **) (((ulong) argp + 15) & ~15);
	linux_env[0] = 0;
	linux_env_p = (char *) (linux_env + LINUX_MAX_ENVS);
	linux_env_idx = 0;
}

void start_linux(void *adr, int swap, unsigned long initrd_address,
		unsigned long initrd_size, void *oftree)
{
	void (*theKernel) (int, char **, char **, int);
	struct memory_bank *mem;
	char *commandline = "board=CARAMBOLA2 mtdparts=ar7240-nor0:256k(u-boot),256k(parameter),15744k(kernel),64k(hwdata),64k(ART) mem=64M rootfstype=squashfs,jffs2 noinitrd";
	char env_buf[12];

	for_each_memory_bank(mem)
		printf("YY 0x%08x size %d\n", mem->start, mem->size);

	printf("YY addr: 0x%08x\n", adr);

	linux_params_init (0x80050000, commandline);

	sprintf (env_buf, "%lu", mem->size);
	linux_env_set ("memsize", env_buf);

	sprintf (env_buf, "0x%08X", (uint) 0);
	linux_env_set ("initrd_start", env_buf);

	sprintf (env_buf, "0x%X", (uint) 0);
	linux_env_set ("initrd_size", env_buf);

	sprintf (env_buf, "0x%08X", (uint) 0);
	linux_env_set ("flash_start", env_buf);

	sprintf (env_buf, "0x%X", (uint) 0);
	linux_env_set ("flash_size", env_buf);

	theKernel =
		(void (*)(int, char **, char **, int)) ntohl (adr);

	shutdown_barebox();
	theKernel (linux_argc, linux_argv, linux_env, 64*1024*1024);
}

/*
 * sdram_start_and_size() - determine place for putting the kernel/oftree/initrd
 *
 * @start:	returns the start address of the first RAM bank
 * @size:	returns the usable space at the beginning of the first RAM bank
 *
 * This function returns the base address of the first RAM bank and the free
 * space found there.
 *
 * return: 0 for success, negative error code otherwise
 */
static int sdram_start_and_size(unsigned long *start, unsigned long *size)
{
	struct memory_bank *bank;
	struct resource *res;

	/*
	 * We use the first memory bank for the kernel and other resources
	 */
	bank = list_first_entry_or_null(&memory_banks, struct memory_bank,
			list);
	if (!bank) {
		printf("cannot find first memory bank\n");
		return -EINVAL;
	}

	/*
	 * If the first memory bank has child resources we can use the bank up
	 * to the beginning of the first child resource, otherwise we can use
	 * the whole bank.
	 */
	res = list_first_entry_or_null(&bank->res->children, struct resource,
			sibling);
	if (res)
		*size = res->start - bank->start;
	else
		*size = bank->size;

	*start = bank->start;

	return 0;
}

static int __do_bootm_linux(struct image_data *data, unsigned long free_mem, int swap)
{
	unsigned long kernel;
	unsigned long initrd_start = 0, initrd_size = 0, initrd_end = 0;
	int ret;
	struct image_header *hdr = &data->os->header;

	kernel = data->os_res->start + data->os_entry;

	initrd_start = data->initrd_address;

	if (initrd_start == UIMAGE_INVALID_ADDRESS) {
		initrd_start = PAGE_ALIGN(free_mem);

		if (bootm_verbose(data)) {
			printf("no initrd load address, defaulting to 0x%08lx\n",
				initrd_start);
		}
	}

	ret = bootm_load_initrd(data, initrd_start);
	if (ret)
		return ret;

	if (data->initrd_res) {
		initrd_start = data->initrd_res->start;
		initrd_end = data->initrd_res->end;
		initrd_size = resource_size(data->initrd_res);
		free_mem = PAGE_ALIGN(initrd_end);
	}

	ret = bootm_load_devicetree(data, free_mem);
	if (ret)
		return ret;

	if (bootm_verbose(data)) {
		printf("\nStarting kernel at 0x%08lx", kernel);
		if (initrd_size)
			printf(", initrd at 0x%08lx", initrd_start);
		if (data->oftree)
			printf(", oftree at 0x%p", data->oftree);
		printf("...\n");
	}

	start_linux((void *)hdr->ih_ep, swap, initrd_start, initrd_size, data->oftree);

	restart_machine();

	return -ERESTARTSYS;
}

static int do_bootm_linux(struct image_data *data)
{
	unsigned long load_address, mem_start, mem_size, mem_free;
	int ret;

	ret = sdram_start_and_size(&mem_start, &mem_size);
	if (ret)
		return ret;

	load_address = data->os_address;

	if (load_address == UIMAGE_INVALID_ADDRESS) {
		/*
		 * Just use a conservative default of 4 times the size of the
		 * compressed image, to avoid the need for the kernel to
		 * relocate itself before decompression.
		 */
		load_address = mem_start + PAGE_ALIGN(
		               uimage_get_size(data->os, data->os_num) * 4);
		if (bootm_verbose(data))
			printf("no OS load address, defaulting to 0x%08lx\n",
				load_address);
	}

	ret = bootm_load_os(data, load_address);
	if (ret)
		return ret;

	/*
	 * put oftree/initrd close behind compressed kernel image to avoid
	 * placing it outside of the kernels lowmem.
	 */
	mem_free = PAGE_ALIGN(data->os_res->end + SZ_1M);

	return __do_bootm_linux(data, mem_free, 0);
}

static int do_bootm_barebox(struct image_data *data)
{
	void (*barebox)(void);

	barebox = read_file(data->os_file, NULL);
	if (!barebox)
		return -EINVAL;

	shutdown_barebox();

	barebox();

	restart_machine();
}

static struct image_handler uimage_handler = {
	.name = "MIPS Linux uImage",
	.bootm = do_bootm_linux,
	.filetype = filetype_uimage,
	.ih_os = IH_OS_LINUX,
};

static struct image_handler barebox_handler = {
	.name = "MIPS barebox",
	.bootm = do_bootm_barebox,
	.filetype = filetype_mips_barebox,
};

static struct binfmt_hook binfmt_barebox_hook = {
	.type = filetype_mips_barebox,
	.exec = "bootm",
};

static int mips_register_image_handler(void)
{
	register_image_handler(&barebox_handler);
	register_image_handler(&uimage_handler);
	binfmt_register(&binfmt_barebox_hook);

	return 0;
}
late_initcall(mips_register_image_handler);
