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

void start_linux(void *adr, int swap, unsigned long initrd_address,
		unsigned long initrd_size, void *oftree)
{
	void (*kernel)(int zero, int arch, void *params) = adr;
	void *params = NULL;
	int architecture;

	if (oftree) {
		pr_debug("booting kernel with devicetree\n");
		params = oftree;
	} else {
		setup_tags(initrd_address, initrd_size, swap);
		params = armlinux_get_bootparams();
	}
	architecture = armlinux_get_architecture();

	shutdown_barebox();
	if (swap) {
		u32 reg;
		__asm__ __volatile__("mrc p15, 0, %0, c1, c0" : "=r" (reg));
		reg ^= CR_B; /* swap big-endian flag */
		__asm__ __volatile__("mcr p15, 0, %0, c1, c0" :: "r" (reg));
	}

#ifdef CONFIG_THUMB2_BAREBOX
	__asm__ __volatile__ (
		"mov r0, #0\n"
		"mov r1, %0\n"
		"mov r2, %1\n"
		"bx %2\n"
		:
		: "r" (architecture), "r" (params), "r" (kernel)
		: "r0", "r1", "r2"
	);
#else
	kernel(0, architecture, params);
#endif
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

	start_linux((void *)kernel, swap, initrd_start, initrd_size, data->oftree);

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
