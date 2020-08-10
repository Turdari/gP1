/* Inline math functions for x86-64.
   Copyright (C) 2002 Free Software Foundation, Inc.
   This file is part of the GNU C Library.
   Contributed by Andreas Jaeger <aj@suse.de>, 2002.

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

#ifndef _MATH_H
# error "Never use <bits/mathinline.h> directly; include <math.h> instead."
#endif

#ifdef __cplusplus
# define __MATH_INLINE __inline
#else
# define __MATH_INLINE extern __inline
#endif


#if defined __USE_ISOC99 && defined __GNUC__ && __GNUC__ >= 2
/* GCC has builtins that can be used.  */
#  define isgreater(x, y) __builtin_isgreater (x, y)
#  define isgreaterequal(x, y) __builtin_isgreaterequal (x, y)
#  define isless(x, y) __builtin_isless (x, y)
#  define islessequal(x, y) __builtin_islessequal (x, y)
#  define islessgreater(x, y) __builtin_islessgreater (x, y)
#  define isunordered(x, y) __builtin_isunordered (x, y)
#endif
