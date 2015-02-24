/*
  outb.c

  Writes a value to an x86 I/O port.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <stdio.h>
#include <sys/io.h>             /* outb */

int main(int argc, char * argv[])
{
  int val;
  int port;

  if (argc < 3 ||
      1 != sscanf(argv[1], "%i", &val) ||
      1 != sscanf(argv[2], "%i", &port)) {
    fprintf(stderr, "usage: outb <val> <port>\n");
    return 1;
  }

#if HAVE_IOPL
  if (-1 == iopl(3)) {
    perror("iopl");
    return 1;
  }
#endif

  (void) outb(val, port);

  return 0;
}
