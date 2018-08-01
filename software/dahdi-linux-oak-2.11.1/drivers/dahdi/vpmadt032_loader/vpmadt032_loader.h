/*
 * DAHDI Telephony Interface to VPMADT032 Firmware Loader
 *
 * Copyright (C) 2008 Digium, Inc.
 *
 * All rights reserved.
 *
 * See http://www.asterisk.org for more information about
 * the Asterisk project. Please do not directly contact
 * any of the maintainers of this project for assistance;
 * the project provides a web site, mailing lists and IRC
 * channels for your use.
 *
 * This program is free software, distributed under the terms of
 * the GNU General Public License Version 2 as published by the
 * Free Software Foundation. See the LICENSE file included with
 * this program for more details.
 */

#if !defined(_VPMADT032_LOADER_H_)
#define _VPMADT032_LOADER_H_ 

#define vpmlinkage __attribute__((regparm(0)))

vpmlinkage void 
__vpmadt032_init(vpmlinkage int (*logger)(const char *format, ...), 
                 unsigned int debug, vpmlinkage void *(*memalloc)(size_t len),
		 vpmlinkage void (*memfree)(void *ptr));

vpmlinkage int 
__vpmadt032_start_load(uint32_t iobase, uint32_t id, void **context);

vpmlinkage int
__vpmadt032_done(void *context);

vpmlinkage void
__vpmadt032_receive(void *context, void *buffer);

vpmlinkage void
__vpmadt032_transmit(void *context, void *buffer);

vpmlinkage void
__vpmadt032_cleanup(void *context);



#endif /* !defined(_VPMADT032_LOADER_H_) */


