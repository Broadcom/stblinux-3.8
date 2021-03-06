/*
 * line - read one line
 *
 * Gunnar Ritter, Freiburg i. Br., Germany, December 2000.
 *
 * Public Domain.
 */

#ident	"@(#)line.c	1.7 (gritter) 7/5/02"

#include	<stdio.h>
#include	<unistd.h>

static int	status;		/* exit status */

static void
doline(int fd)
{
	char c;

	for (;;) {
		if (read(fd, &c, 1) <= 0) {
			status = 1;
			break;
		}
		if (c == '\n')
			break;
		putchar(c);
	}
	putchar('\n');
}

int
main(int argc, char **argv)
{
	doline(0);
	return status;
}
