#include <stdio.h>
#include <ulapi.h>
#include "go.h"

int main(int argc, char *argv[])
{
  enum {BUFFERLEN = 256};
  char buffer[BUFFERLEN];
  int option;
  go_flag do_rads = 0;
  go_rpy rpy;
  go_quat quat;
  double d1, d2, d3;

  if (ULAPI_OK != ulapi_init()) {
    fprintf(stderr, "can't init ulapi\n");
    return 1;
  }

  opterr = 0;
  while (1) {
    option = ulapi_getopt(argc, argv, ":rd");
    if (option == -1)
      break;

    switch (option) {
    case 'r':
      do_rads = 1;
      break;

    case 'd':
      do_rads = 0;
      break;

    case ':':
      fprintf(stderr, "missing value for -%c\n", ulapi_optopt);
      return 1;
      break;

    default:			/* '?' */
      fprintf (stderr, "unrecognized option -%c\n", ulapi_optopt);
      return 1;
      break;
    }
  }
  if (ulapi_optind < argc) {
    fprintf(stderr, "extra non-option characters: %s\n", argv[ulapi_optind]);
    return 1;
  }

  if (0 != go_init()) {
    fprintf(stderr, "can't init go\n");
    return 1;
  }

  while (! feof(stdin)) {
    if (NULL == fgets(buffer, BUFFERLEN, stdin)) {
      break;
    }

    if (3 != sscanf(buffer, "%lf %lf %lf", &d1, &d2, &d3)) continue;

    if (do_rads) {
      rpy.r = d1, rpy.p = d2, rpy.y = d3;
    } else {
      rpy.r = GO_TO_RAD(d1), rpy.p = GO_TO_RAD(d2), rpy.y = GO_TO_RAD(d3);
    }

    go_rpy_quat_convert(&rpy, &quat);

    printf("%f %f %f %f\n", quat.x, quat.y, quat.z, quat.s);
  }

  return 0;
}
