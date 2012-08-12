/*
 * Copyright (C) 2009 LGE, Inc. , wingrime (C) 2010-2011
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/platform_device.h>
#include <linux/gpio_event.h>

#include <asm/mach-types.h>

static unsigned int keypad_col_gpios_swift[] = { 34, 35, 36 };
static unsigned int keypad_row_gpios_swift[] = { 37, 38, 39 };

#define KEYMAP_INDEX_SWIFT(col, row) ((col)*ARRAY_SIZE(keypad_row_gpios_swift) + (row))

static const unsigned short keypad_keymap_swift[ARRAY_SIZE(keypad_col_gpios_swift) *
					  ARRAY_SIZE(keypad_row_gpios_swift)] = {
	[KEYMAP_INDEX_SWIFT(0, 0)] = KEY_VOLUMEUP, 
	[KEYMAP_INDEX_SWIFT(0, 1)] = KEY_VOLUMEDOWN, 
	[KEYMAP_INDEX_SWIFT(1, 0)] = KEY_CAMERA_FOCUS, 
	[KEYMAP_INDEX_SWIFT(1, 1)] = KEY_CAMERA_SNAPSHOT, 
	[KEYMAP_INDEX_SWIFT(1, 2)] = KEY_SEARCH,
	[KEYMAP_INDEX_SWIFT(2, 0)] = KEY_SEND,
	[KEYMAP_INDEX_SWIFT(2, 1)] = KEY_HOME,
};

static const unsigned short keypad_virtual_keys[] = {
	KEY_END,
	KEY_POWER
};

static int keypad_gpio_event_matrix_func(struct gpio_event_input_devs *input_dev,
					  struct gpio_event_info *info,
					  void **data, int func);

struct gpio_event_matrix_info swift_keypad_matrix_info = {
	.info.func	= keypad_gpio_event_matrix_func,
	.keymap		= keypad_keymap_swift,
	.output_gpios	= keypad_col_gpios_swift,
	.input_gpios	= keypad_row_gpios_swift,
	.noutputs	= ARRAY_SIZE(keypad_col_gpios_swift),
	.ninputs	= ARRAY_SIZE(keypad_row_gpios_swift),
	.settle_time.tv_nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv_nsec = 20 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE |
			  GPIOKPF_PRINT_MAPPED_KEYS
};

struct gpio_event_info *swift_keypad_info[] = {
	&swift_keypad_matrix_info.info,
};

struct gpio_event_platform_data swift_keypad_data = {
	.name		= "swift_keypad",
	.info		= swift_keypad_info,
	.info_count	= ARRAY_SIZE(swift_keypad_info),
};

struct platform_device keypad_device_swift = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= 0,
	.dev	= {
		.platform_data	= &swift_keypad_data,
	},
};

static struct input_dev *keypad_dev;

static int keypad_gpio_event_matrix_func(struct gpio_event_input_devs *input_devs,
					  struct gpio_event_info *info,
					  void **data, int func)
{
	int err;
	int i;

	err = gpio_event_matrix_func(input_devs, info, data, func);

	if (func == GPIO_EVENT_FUNC_INIT && !err) {
		keypad_dev = input_devs->dev[0];
		for (i = 0; i < ARRAY_SIZE(keypad_virtual_keys); i++)
			set_bit(keypad_virtual_keys[i],
				keypad_dev->keybit);
	} else if (func == GPIO_EVENT_FUNC_UNINIT) {
		keypad_dev = NULL;
	}

	return err;
}

struct input_dev *msm_keypad_get_input_dev(void)
{
	return keypad_dev;
}
