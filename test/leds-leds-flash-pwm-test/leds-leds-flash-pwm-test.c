// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Jozsef Horvath <info@ministro.hu>
 *
 */
#define _GNU_SOURCE
#include <stdlib.h>
#include <getopt.h>
#include <signal.h>
#include <unistd.h>
#include <dirent.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <poll.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/kernel.h>
#include <linux/videodev2.h>
#include <sys/eventfd.h>
#include <time.h>
#include <dirent.h>
#include <endian.h>

#include "utils.h"

const char* g_short_options = "d:ih";
const struct option g_long_options[] = {
	{ "device", required_argument, NULL, 'd' },
	{ "info", no_argument, NULL, 'i' },
	{ "help", no_argument, NULL, 'h' },
	{ NULL, 0, NULL, 0 },
};

static struct {
	bool info;
	char* video_device;
	int device_fd;
} g_app_ctx = {
};

static void print_help(FILE *output)
{
	fprintf(output,
		"\nleds-leds-flash-pwm-test - leds-flash-pwm command line test utility\n"
		"usage: leds-leds-flash-pwm-test [options]\n"
		"\n"
		"  -d, --device=PATH_TO_V4L2_DEVICE\n"
		"  -i, --info\n"
		"  -h, --help\n"
	);
}

static void print_help_exit(FILE *output, int exit_code)
{
	print_help(output);
	exit(exit_code);
}

static int is_control_supported(int video_fd, unsigned int control_id)
{
	struct v4l2_capability caps;
	int ret = ioctl(video_fd, VIDIOC_QUERYCAP, &caps);
	if (ret) {
		fprintf(stderr, "Query device capabilities error: %d\n", errno);
		return ret;
	}

	return caps.device_caps & control_id ? 1 : 0;
}

static int set_control_value(int video_fd, unsigned int control_id, int control_value)
{
	int ret;
	struct v4l2_control ctl = {
		.id = control_id,
		.value = control_value,
	};

	ret = ioctl(video_fd, VIDIOC_S_CTRL, &ctl);
	if (ret == -1) {
		fprintf(stderr, "Set control value error: %d\n", errno);
		return ret;
	}

	return 0;
}

static int get_control_value(int video_fd, unsigned int control_id, int* control_value)
{
	int ret;
	struct v4l2_control ctl = {
		.id = control_id,
	};

	ret = ioctl(video_fd, VIDIOC_G_CTRL, &ctl);
	if (ret == -1) {
		fprintf(stderr, "Get control value error: %d\n", errno);
		return ret;
	}

	*control_value = ctl.value;

	return 0;
}

static int print_is_control_supported(FILE* output,
	int video_fd,
	unsigned int control_id,
	const char* control)
{
	int ret;
	
	ret = is_control_supported(video_fd, control_id);
	if (ret < 0) return -EINVAL;
	fprintf(output,
		"%s is %ssupported\n",
		control,
		!ret ? "not " : "");
	
	return 0;
}

static void print_info(FILE* output, int video_fd)
{
	int ret;
	
	ret = print_is_control_supported(output, video_fd, V4L2_CID_FLASH_LED_MODE, "V4L2_CID_FLASH_LED_MODE");
	if (ret)
		return;
	ret = print_is_control_supported(output, video_fd, V4L2_CID_FLASH_TORCH_INTENSITY, "V4L2_CID_FLASH_TORCH_INTENSITY");
	if (ret)
		return;
	ret = print_is_control_supported(output, video_fd, V4L2_CID_FLASH_STROBE, "V4L2_CID_FLASH_STROBE");
	if (ret)
		return;
	ret = print_is_control_supported(output, video_fd, V4L2_CID_FLASH_STROBE_STOP, "V4L2_CID_FLASH_STROBE_STOP");
	if (ret)
		return;
	ret = print_is_control_supported(output, video_fd, V4L2_CID_FLASH_STROBE_SOURCE, "V4L2_CID_FLASH_STROBE_SOURCE");
	if (ret)
		return;
	ret = print_is_control_supported(output, video_fd, V4L2_CID_FLASH_STROBE_STATUS, "V4L2_CID_FLASH_STROBE_STATUS");
	if (ret)
		return;
	ret = print_is_control_supported(output, video_fd, V4L2_CID_FLASH_TIMEOUT, "V4L2_CID_FLASH_TIMEOUT");
	if (ret)
		return;
	ret = print_is_control_supported(output, video_fd, V4L2_CID_FLASH_INTENSITY, "V4L2_CID_FLASH_INTENSITY");
	if (ret)
		return;
}

static int update_control_value_with_info(FILE* output,
	int video_fd,
	unsigned int control_id,
	const char* control,
	int new_value)
{
	int current_value = 0;
	int ret;

	ret = get_control_value(video_fd, control_id, &current_value);
	if (!ret) {
		fprintf(output, "%s current value: %d\n", control, current_value);
	}
	ret = set_control_value(video_fd, control_id, V4L2_FLASH_LED_MODE_TORCH);
	ret = get_control_value(video_fd, control_id, &current_value);
	if (!ret) {
		fprintf(output, "%s new value: %d\n", control, current_value);
	}

	return 0;
}

int main(int argc, char ** argv)
{
	int next_option;
	int ret = 0;

	while ((next_option = getopt_long(argc, argv, g_short_options, g_long_options, NULL)) != -1) {
		switch (next_option) {
		case 'd':
			if (optarg == NULL || strlen(optarg) == 0)
				print_help_exit(stderr, EXIT_FAILURE);
			
			g_app_ctx.video_device = strdup(optarg);
		break;
		case 'i':
			g_app_ctx.info = true;
		break;
		default:
			print_help_exit(stderr, EXIT_FAILURE);
		break;
		}
	}

	g_app_ctx.device_fd = open(g_app_ctx.video_device, O_RDWR | O_NONBLOCK);
	if (g_app_ctx.device_fd == -1) {
		fprintf(stdout, "Device open error: %d", errno);
		return 1;
	}
	
	if (g_app_ctx.info)
		print_info(stdout, g_app_ctx.device_fd);

	update_control_value_with_info(stdout,
		g_app_ctx.device_fd,
		V4L2_CID_FLASH_LED_MODE,
		"V4L2_CID_FLASH_LED_MODE",
		V4L2_FLASH_LED_MODE_TORCH);

	return ret;
}
