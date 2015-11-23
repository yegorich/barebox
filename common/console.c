/*
 * (C) Copyright 2000
 * Paolo Scaffardi, AIRVENT SAM s.p.a - RIMINI(ITALY), arsenio@tin.it
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <config.h>
#include <common.h>
#include <stdarg.h>
#include <malloc.h>
#include <param.h>
#include <console.h>
#include <driver.h>
#include <fs.h>
#include <of.h>
#include <init.h>
#include <clock.h>
#include <kfifo.h>
#include <module.h>
#include <poller.h>
#include <magicvar.h>
#include <globalvar.h>
#include <linux/list.h>
#include <linux/stringify.h>
#include <debug_ll.h>

LIST_HEAD(console_list);
EXPORT_SYMBOL(console_list);

#define CONSOLE_UNINITIALIZED		0
#define CONSOLE_INITIALIZED_BUFFER	1
#define CONSOLE_INIT_FULL		2

#define to_console_dev(d) container_of(d, struct console_device, class_dev)

static int initialized = 0;

#define CONSOLE_BUFFER_SIZE	1024

static char console_input_buffer[CONSOLE_BUFFER_SIZE];
static char console_output_buffer[CONSOLE_BUFFER_SIZE];

static struct kfifo __console_input_fifo;
static struct kfifo __console_output_fifo;
static struct kfifo *console_input_fifo = &__console_input_fifo;
static struct kfifo *console_output_fifo = &__console_output_fifo;

int console_set_active(struct console_device *cdev, unsigned flag)
{
	int ret, i;

	if (!cdev->getc)
		flag &= ~CONSOLE_STDIN;
	if (!cdev->putc)
		flag &= ~(CONSOLE_STDOUT | CONSOLE_STDERR);

	if (!flag && cdev->f_active && cdev->flush)
		cdev->flush(cdev);

	if (cdev->set_active) {
		ret = cdev->set_active(cdev, flag);
		if (ret)
			return ret;
	}

	cdev->f_active = flag;

	if (IS_ENABLED(CONFIG_PARAMETER)) {
		i = 0;

		if (flag & CONSOLE_STDIN)
			cdev->active[i++] = 'i';
		if (flag & CONSOLE_STDOUT)
			cdev->active[i++] = 'o';
		if (flag & CONSOLE_STDERR)
			cdev->active[i++] = 'e';
		cdev->active[i] = 0;
	}

	if (initialized < CONSOLE_INIT_FULL) {
		char ch;
		initialized = CONSOLE_INIT_FULL;
		puts_ll("Switch to console [");
		puts_ll(dev_name(&cdev->class_dev));
		puts_ll("]\n");
		barebox_banner();
		while (kfifo_getc(console_output_fifo, &ch) == 0)
			console_putc(CONSOLE_STDOUT, ch);
	}

	return 0;
}

unsigned console_get_active(struct console_device *cdev)
{
	return cdev->f_active;
}

static int console_active_set(struct device_d *dev, struct param_d *param,
		const char *val)
{
	struct console_device *cdev = to_console_dev(dev);
	unsigned int flag = 0;

	if (val) {
		if (strchr(val, 'i'))
			flag |= CONSOLE_STDIN;
		if (strchr(val, 'o'))
			flag |= CONSOLE_STDOUT;
		if (strchr(val, 'e'))
			flag |= CONSOLE_STDERR;
	}

	return console_set_active(cdev, flag);
}

static const char *console_active_get(struct device_d *dev,
		struct param_d *param)
{
	struct console_device *cdev = to_console_dev(dev);

	return cdev->active;
}

int console_set_baudrate(struct console_device *cdev, unsigned baudrate)
{
	int ret;
	unsigned char c;

	if (!cdev->setbrg)
		return -ENOSYS;

	if (cdev->baudrate == baudrate)
		return 0;

	/*
	 * If the device is already active, change its baudrate.
	 * The baudrate of an inactive device will be set at activation time.
	 */
	if (cdev->f_active) {
		printf("## Switch baudrate on console %s to %d bps and press ENTER ...\n",
			dev_name(&cdev->class_dev), baudrate);
		mdelay(50);
	}

	ret = cdev->setbrg(cdev, baudrate);
	if (ret)
		return ret;

	if (cdev->f_active) {
		mdelay(50);
		do {
			c = getc();
		} while (c != '\r' && c != '\n');
	}

	cdev->baudrate = baudrate;
	cdev->baudrate_param = baudrate;

	return 0;
}

unsigned console_get_baudrate(struct console_device *cdev)
{
	return cdev->baudrate;
}

static int console_baudrate_set(struct param_d *param, void *priv)
{
	struct console_device *cdev = priv;

	return console_set_baudrate(cdev, cdev->baudrate_param);
}

static void console_init_early(void)
{
	kfifo_init(console_input_fifo, console_input_buffer,
			CONSOLE_BUFFER_SIZE);
	kfifo_init(console_output_fifo, console_output_buffer,
			CONSOLE_BUFFER_SIZE);

	initialized = CONSOLE_INITIALIZED_BUFFER;
}

static void console_set_stdoutpath(struct console_device *cdev)
{
	int id;
	char *str;

	if (!cdev->linux_console_name)
		return;

	id = of_alias_get_id(cdev->dev->device_node, "serial");
	if (id < 0)
		return;

	str = asprintf("console=%s%d,%dn8", cdev->linux_console_name,
			id, cdev->baudrate);

	globalvar_add_simple("linux.bootargs.console", str);

	free(str);
}

static int __console_puts(struct console_device *cdev, const char *s)
{
	int n = 0;

	while (*s) {
		if (*s == '\n') {
			cdev->putc(cdev, '\r');
			n++;
		}
		cdev->putc(cdev, *s);
		n++;
		s++;
	}
	return n;
}

int console_register(struct console_device *newcdev)
{
	struct device_d *dev = &newcdev->class_dev;
	int activate = 0, ret;

	if (initialized == CONSOLE_UNINITIALIZED)
		console_init_early();

	if (newcdev->devname) {
		dev->id = newcdev->devid;
		strcpy(dev->name, newcdev->devname);
	} else {
		dev->id = DEVICE_ID_DYNAMIC;
		strcpy(dev->name, "cs");
	}

	if (newcdev->dev)
		dev->parent = newcdev->dev;
	platform_device_register(dev);

	if (newcdev->setbrg) {
		ret = newcdev->setbrg(newcdev, CONFIG_BAUDRATE);
		if (ret)
			return ret;
		newcdev->baudrate = CONFIG_BAUDRATE;
		dev_add_param_int(dev, "baudrate", console_baudrate_set,
			NULL, &newcdev->baudrate_param, "%u", newcdev);
	}

	if (newcdev->putc && !newcdev->puts)
		newcdev->puts = __console_puts;

	dev_add_param(dev, "active", console_active_set, console_active_get, 0);

	if (IS_ENABLED(CONFIG_CONSOLE_ACTIVATE_FIRST)) {
		if (list_empty(&console_list))
			activate = 1;
	} else if (IS_ENABLED(CONFIG_CONSOLE_ACTIVATE_ALL)) {
		activate = 1;
	}

	if (newcdev->dev && of_device_is_stdout_path(newcdev->dev)) {
		activate = 1;
		console_set_stdoutpath(newcdev);
	}

	list_add_tail(&newcdev->list, &console_list);

	if (activate)
		console_set_active(newcdev, CONSOLE_STDIN |
				CONSOLE_STDOUT | CONSOLE_STDERR);

	return 0;
}
EXPORT_SYMBOL(console_register);

int console_unregister(struct console_device *cdev)
{
	struct device_d *dev = &cdev->class_dev;
	int status;

	list_del(&cdev->list);
	if (list_empty(&console_list))
		initialized = CONSOLE_UNINITIALIZED;

	status = unregister_device(dev);
	if (!status)
		memset(cdev, 0, sizeof(*cdev));
	return status;
}
EXPORT_SYMBOL(console_unregister);

static int getc_raw(void)
{
	struct console_device *cdev;
	int active = 0;

	while (1) {
		for_each_console(cdev) {
			if (!(cdev->f_active & CONSOLE_STDIN))
				continue;
			active = 1;
			if (cdev->tstc(cdev))
				return cdev->getc(cdev);
		}
		if (!active)
			/* no active console found. bail out */
			return -1;
	}
}

static int tstc_raw(void)
{
	struct console_device *cdev;

	for_each_console(cdev) {
		if (!(cdev->f_active & CONSOLE_STDIN))
			continue;
		if (cdev->tstc(cdev))
			return 1;
	}

	return 0;
}

int getc(void)
{
	unsigned char ch;
	uint64_t start;

	/*
	 * For 100us we read the characters from the serial driver
	 * into a kfifo. This helps us not to lose characters
	 * in small hardware fifos.
	 */
	start = get_time_ns();
	while (1) {
		if (tstc_raw()) {
			kfifo_putc(console_input_fifo, getc_raw());

			start = get_time_ns();
		}
		if (is_timeout(start, 100 * USECOND) &&
				kfifo_len(console_input_fifo))
			break;
	}

	kfifo_getc(console_input_fifo, &ch);
	return ch;
}
EXPORT_SYMBOL(getc);

int fgetc(int fd)
{
	char c;

	if (!fd)
		return getc();
	return read(fd, &c, 1);
}
EXPORT_SYMBOL(fgetc);

int tstc(void)
{
	return kfifo_len(console_input_fifo) || tstc_raw();
}
EXPORT_SYMBOL(tstc);

void console_putc(unsigned int ch, char c)
{
	struct console_device *cdev;
	int init = initialized;

	switch (init) {
	case CONSOLE_UNINITIALIZED:
		console_init_early();
		/* fall through */

	case CONSOLE_INITIALIZED_BUFFER:
		kfifo_putc(console_output_fifo, c);
		putc_ll(c);
		return;

	case CONSOLE_INIT_FULL:
		for_each_console(cdev) {
			if (cdev->f_active & ch) {
				if (c == '\n')
					cdev->putc(cdev, '\r');
				cdev->putc(cdev, c);
			}
		}
		return;
	default:
		/* If we have problems inititalizing our data
		 * get them early
		 */
		hang();
	}
}
EXPORT_SYMBOL(console_putc);

int console_puts(unsigned int ch, const char *str)
{
	struct console_device *cdev;
	const char *s = str;
	int n = 0;

	if (initialized == CONSOLE_INIT_FULL) {
		for_each_console(cdev) {
			if (cdev->f_active & ch) {
				n = cdev->puts(cdev, str);
			}
		}
		return n;
	}

	while (*s) {
		if (*s == '\n') {
			console_putc(ch, '\r');
			n++;
		}
		console_putc(ch, *s);
		n++;
		s++;
	}
	return n;
}
EXPORT_SYMBOL(console_puts);

void console_flush(void)
{
	struct console_device *cdev;

	for_each_console(cdev) {
		if (cdev->flush)
			cdev->flush(cdev);
	}
}
EXPORT_SYMBOL(console_flush);

#ifndef ARCH_HAS_CTRLC
/* test if ctrl-c was pressed */
int ctrlc (void)
{
	poller_call();

	if (tstc() && getc() == 3)
		return 1;
	return 0;
}
EXPORT_SYMBOL(ctrlc);
#endif /* ARCH_HAS_CTRC */

BAREBOX_MAGICVAR_NAMED(global_linux_bootargs_console, global.linux.bootargs.console,
		"console= argument for Linux from the linux,stdout-path property in /chosen node");
