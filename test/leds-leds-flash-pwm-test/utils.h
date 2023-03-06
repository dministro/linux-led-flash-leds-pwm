// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Jozsef Horvath <info@ministro.hu>
 *
 */
#ifndef _UTILS_H_
#define _UTILS_H_

int stream_printf(FILE *output, const char *fmt, ...)
	__attribute__ ((format (printf, 2, 3)));

#endif
