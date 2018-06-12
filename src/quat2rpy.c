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
  double d1, d2, d3, d4;

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

    if (4 != sscanf(buffer, "%lf %lf %lf %lf", &d1, &d2, &d3, &d4)) continue;

    quat.x = d1, quat.y = d2, quat.z = d3, quat.s = d4;

    go_quat_rpy_convert(&quat, &rpy);

    if (do_rads) {
      printf("%f %f %f\n", (double) rpy.r, (double) rpy.p, (double) rpy.y);
    } else {
      printf("%f %f %f\n", (double) GO_TO_DEG(rpy.r), (double) GO_TO_DEG(rpy.p), (double) GO_TO_DEG(rpy.y));
    }
  }

  return 0;
}
