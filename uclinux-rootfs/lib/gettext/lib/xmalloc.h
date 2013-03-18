/* malloc with out of memory checking.
   Copyright (C) 2001 Free Software Foundation, Inc.
   Written by Bruno Haible <haible@clisp.cons.org>, 2001.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.  */

#ifndef _XMALLOC_H
#define _XMALLOC_H

#include <stddef.h>


/* Defined in xmalloc.c.  */

/* Allocate SIZE bytes of memory dynamically, with error checking.  */
extern void *xmalloc PARAMS ((size_t size));

/* Allocate memory for NMEMB elements of SIZE bytes, with error checking.  */
extern void *xcalloc PARAMS ((size_t nmemb, size_t size));

/* Change the size of an allocated block of memory PTR to SIZE bytes,
   with error checking.  If PTR is NULL, run xmalloc.  */
extern void *xrealloc PARAMS ((void *ptr, size_t size));


/* Defined in xstrdup.c.  */

/* Return a newly allocated copy of STRING.  */
extern char *xstrdup PARAMS ((const char *string));


#endif /* _XMALLOC_H */
