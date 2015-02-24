/*
  inb.c

  Reads and prints a value from an x86 I/O port.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <stdio.h>
#include <sys/io.h>             /* inb */

int main(int argc, char * argv[])
{
  int port;

  if (argc < 2 ||
      1 != sscanf(argv[1], "%i", &port)) {
    fprintf(stderr, "usage: inb <port>\n");
    return 1;
  }

#if HAVE_IOPL
  if (-1 == iopl(3)) {
    perror("iopl");
    return 1;
  }
#endif

  printf("0x%X\n", (int) inb(port));

  return 0;
}
