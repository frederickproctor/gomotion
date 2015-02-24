/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

/*
  \file emu_smartmotor.c

  \brief Emulates an Animatics Smart Motor via a serial interface.

  Syntax: emu_smartmotor <port>
*/

#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ulapi.h>

#define NULLTERM(ARRAY) (ARRAY)[sizeof(ARRAY)-1] = 0

int main(int argc, char *argv[])
{
  void *portptr;
#define BUFFERLEN 256
#define DELIMITER '\r'
  char buffer[BUFFERLEN];
  char reply[BUFFERLEN];
  int nchars;
  int buildlen = BUFFERLEN;
  ptrdiff_t offset;
  char *buffer_ptr;
  char *buffer_end;
  char *build = NULL;
  char *build_ptr;
  char *build_end;
  int position = 0;

  if (argc < 2) {
    fprintf(stderr, "need a port name\n");
    return 1;
  }

  portptr = ulapi_serial_new();
  if (NULL == portptr) {
    fprintf(stderr, "can't allocate a port\n");
    return 1;
  }

  if (ULAPI_OK != ulapi_serial_open(argv[1], portptr)) {
    fprintf(stderr, "can't open port %s\n", argv[1]);
    return 1;
  }

  build = realloc(build, buildlen * sizeof(char));
  build_ptr = build;
  build_end = build + buildlen;

  NULLTERM(buffer);
  NULLTERM(reply);

  for (;;) {
    nchars = ulapi_serial_read(portptr, buffer, sizeof(buffer) - 1);
    if (nchars <= 0) continue;

    buffer_ptr = buffer;
    buffer_end = buffer + nchars;

    while (buffer_ptr != buffer_end) {
      if (build_ptr == build_end) {
	offset = build_ptr - build;
	buildlen *= 2;
	build = (char *) realloc(build, buildlen * sizeof(char));
	build_ptr = build + offset;
	build_end = build + buildlen;
      }
      *build_ptr++ = *buffer_ptr;
      if (*buffer_ptr++ == DELIMITER) {
	offset = build_ptr - build;
	build_ptr = build;
	build[offset] = 0;
	if (! strncmp(build, "RP", 2)) {
	  ulapi_snprintf(reply, sizeof(reply) - 1, "%d\r", position);
	  ulapi_serial_write(portptr, reply, strlen(reply));
	  printf("reporting %d\n", position);
	} else if (! strncmp(build, "P=", 2)) {
	  position = atoi(&build[2]);
	  printf("setting %d\n", position);
	}
      }
    }
  }

  ulapi_exit();

  return 0;
}
