#define _GNU_SOURCE
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <limits.h>
#include <sys/timerfd.h>

#include "utils.h"

int stream_printf(FILE *output, const char *fmt, ...)
{
	int ret = 0;
	va_list arg_list;
	va_start(arg_list, fmt);
	ret = vfprintf(output, fmt, arg_list);
	if (ret >= 0)
		fflush(output);
	va_end(arg_list);

	return ret;
}
