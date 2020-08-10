/* libc-internal interface for thread-specific data.  Stub or TLS version.
   Copyright (C) 1998,2001,02 Free Software Foundation, Inc.
   This file is part of the GNU C Library.

   The GNU C Library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   The GNU C Library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the GNU C Library; if not, write to the Free
   Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
   02111-1307 USA.  */

#ifndef _GENERIC_BITS_LIBC_TSD_H
#define _GENERIC_BITS_LIBC_TSD_H 1

/* This file defines the following macros for accessing a small fixed
   set of thread-specific `void *' data used only internally by libc.

   __libc_tsd_define(CLASS, KEY)	-- Define or declare a `void *' datum
   					   for KEY.  CLASS can be `static' for
					   keys used in only one source file,
					   empty for global definitions, or
					   `extern' for global declarations.
   __libc_tsd_address(KEY)		-- Return the `void **' pointing to
   					   the current thread's datum for KEY.
   __libc_tsd_get(KEY)			-- Return the `void *' datum for KEY.
   __libc_tsd_set(KEY, VALUE)		-- Set the datum for KEY to VALUE.

   The set of available KEY's will usually be provided as an enum,
   and contains (at least):
		_LIBC_TSD_KEY_MALLOC
		_LIBC_TSD_KEY_DL_ERROR
		_LIBC_TSD_KEY_RPC_VARS
   All uses must be the literal _LIBC_TSD_* name in the __libc_tsd_* macros.
   Some implementations may not provide any enum at all and instead
   using string pasting in the macros.  */

#include <tls.h>

/* When full support for __thread variables is available, this interface is
   just a trivial wrapper for it.  Without TLS, this is the generic/stub
   implementation for wholly single-threaded systems.

   We don't define an enum for the possible key values, because the KEYs
   translate directly into variables by macro magic.  */

#if USE_TLS && HAVE___THREAD
# define __libc_tsd_define(CLASS, KEY)	CLASS __thread void *__libc_tsd_##KEY;

# define __libc_tsd_address(KEY)	(&__libc_tsd_##KEY)
# define __libc_tsd_get(KEY)		(__libc_tsd_##KEY)
# define __libc_tsd_set(KEY, VALUE)	(__libc_tsd_##KEY = (VALUE))
#else
# define __libc_tsd_define(CLASS, KEY)	CLASS void *__libc_tsd_##KEY##_data;

# define __libc_tsd_address(KEY)	(&__libc_tsd_##KEY##_data)
# define __libc_tsd_get(KEY)		(__libc_tsd_##KEY##_data)
# define __libc_tsd_set(KEY, VALUE)	(__libc_tsd_##KEY##_data = (VALUE))
#endif

#endif	/* bits/libc-tsd.h */
