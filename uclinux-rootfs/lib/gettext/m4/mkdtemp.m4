# mkdtemp.m4 serial 1 (gettext-0.11)
dnl Copyright (C) 2001-2002 Free Software Foundation, Inc.
dnl This file is free software, distributed under the terms of the GNU
dnl General Public License.  As a special exception to the GNU General
dnl Public License, this file may be distributed as part of a program
dnl that contains a configuration script generated by Autoconf, under
dnl the same distribution terms as the rest of that program.

# Prerequisites of lib/mkdtemp.c

AC_DEFUN([gt_FUNC_MKDTEMP],
[
  AC_REPLACE_FUNCS(mkdtemp)
  AC_STAT_MACROS_BROKEN
  jm_AC_HEADER_INTTYPES_H
  jm_AC_HEADER_STDINT_H
  AC_CHECK_HEADERS(fcntl.h sys/time.h time.h unistd.h)
  AC_CHECK_FUNCS(gettimeofday)
])
