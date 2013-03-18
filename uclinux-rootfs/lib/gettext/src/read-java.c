/* Reading Java ResourceBundles.
   Copyright (C) 2001-2002 Free Software Foundation, Inc.
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

#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

/* Specification.  */
#include "read-java.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include "msgunfmt.h"
#include "javaexec.h"
#include "pipe.h"
#include "wait-process.h"
#include "read-po.h"
#include "error.h"
#include "exit.h"
#include "gettext.h"

#define _(str) gettext (str)


/* Prototypes for local functions.  Needed to ensure compiler checking of
   function argument counts despite of K&R C function definition syntax.  */
static bool execute_and_read_po_output PARAMS ((const char *progname,
						const char *prog_path,
						char **prog_argv,
						void *private_data));


/* A Java resource name can only be manipulated by a Java virtual machine.
   So we start a JVM to execute the DumpResource program, and read its
   output, which is .po format without comments.  */

struct locals
{
  /* OUT */
  msgdomain_list_ty *mdlp;
};

static bool
execute_and_read_po_output (progname, prog_path, prog_argv, private_data)
     const char *progname;
     const char *prog_path;
     char **prog_argv;
     void *private_data;
{
  struct locals *l = (struct locals *) private_data;
  pid_t child;
  int fd[1];
  FILE *fp;
  int exitstatus;

  /* Open a pipe to the JVM.  */
  child = create_pipe_in (progname, prog_path, prog_argv, "/dev/null", false,
			  true, fd);

  fp = fdopen (fd[0], "r");
  if (fp == NULL)
    error (EXIT_FAILURE, errno, _("fdopen() failed"));

  /* Read the message list.  */
  l->mdlp = read_po (fp, "(pipe)", "(pipe)");

  fclose (fp);

  /* Remove zombie process from process list, and retrieve exit status.  */
  exitstatus = wait_subprocess (child, progname, true);
  if (exitstatus != 0)
    error (EXIT_FAILURE, 0, _("%s subprocess failed with exit code %d"),
	   progname, exitstatus);

  return false;
}


msgdomain_list_ty *
msgdomain_read_java (resource_name, locale_name)
     const char *resource_name;
     const char *locale_name;
{
  const char *class_name = "gnu.gettext.DumpResource";
  const char *gettextjexedir;
  const char *gettextjar;
  const char *args[3];
  struct locals locals;

#if USEJEXE
  /* Make it possible to override the executable's location.  This is
     necessary for running the testsuite before "make install".  */
  gettextjexedir = getenv ("GETTEXTJEXEDIR");
  if (gettextjexedir == NULL || gettextjexedir[0] == '\0')
    gettextjexedir = GETTEXTJEXEDIR;
#else
  gettextjexedir = NULL;
#endif

  /* Make it possible to override the gettext.jar location.  This is
     necessary for running the testsuite before "make install".  */
  gettextjar = getenv ("GETTEXTJAR");
  if (gettextjar == NULL || gettextjar[0] == '\0')
    gettextjar = GETTEXTJAR;

  /* Assign a default value to the resource name.  */
  if (resource_name == NULL)
    resource_name = "Messages";

  /* Prepare arguments.  */
  args[0] = resource_name;
  if (locale_name != NULL)
    {
      args[1] = locale_name;
      args[2] = NULL;
    }
  else
    args[1] = NULL;

  /* Dump the resource and retrieve the resulting output.
     Here we use the user's CLASSPATH, not a minimal one, so that the
     resource can be found.  */
  if (execute_java_class (class_name, &gettextjar, 1, false, gettextjexedir,
			  args,
			  verbose, false,
			  execute_and_read_po_output, &locals))
    /* An error message should already have been provided.  */
    exit (EXIT_FAILURE);

  return locals.mdlp;
}
