#include <stdio.h>		/* sscanf, fprintf, stderr */
#include <stdlib.h>		/* malloc */
#include <stddef.h>		/* NULL */
#include <string.h>		/* strncmp */
#include <sys/time.h>		/* struct timeval */
#include <time.h>		/* gettimeofday */
#include "go.h"

static go_real timestamp(void)
{
  struct timeval tv;

  gettimeofday(&tv, NULL);

  return ((go_real) tv.tv_sec) + ((go_real) tv.tv_usec) * 1.0e-6;
}

/* syntax: gotestmmavg {<size>} */
int main(int argc, char * argv[])
{
  enum {BUFFERLEN = 80};
  char buffer[BUFFERLEN];
  int i1;
  double d1;
  go_mmavg h;
  go_real * space;
  go_timestamped_real tsr;
  go_real * ptr;

  if (argc < 2) {
    go_mmavg_init(&h, NULL, 0, timestamp);
  } else {
    if (1 != sscanf(argv[1], "%d", &i1) ||
	i1 <= 0) {
      fprintf(stderr, "syntax: gotestmmavg {<size>}\n");
      return 1;
    }
    space = malloc(i1 * sizeof(go_real));
    if (NULL == space) {
      fprintf(stderr, "can't allocate %d values\n", i1);
      return 1;
    }
    go_mmavg_init(&h, space, (go_real) i1, timestamp);
  }

  while (! feof(stdin)) {
    if (NULL == fgets(buffer, BUFFERLEN, stdin)) break;
    if ('q' == *buffer) {
      break;
    } else if (1 == sscanf(buffer, "%lf", &d1)) {
      go_mmavg_add(&h, (go_real) d1);
    } else if (! strncmp(buffer, "min", 3)) {
      printf("%f\n", (double) go_mmavg_min(&h));
    } else if (! strncmp(buffer, "max", 3)) {
      printf("%f\n", (double) go_mmavg_max(&h));
    } else if (! strncmp(buffer, "avg", 3)) {
      printf("%f\n", (double) go_mmavg_avg(&h));
    } else if (! strncmp(buffer, "lifemin", 7)) {
      tsr = go_mmavg_lifemin(&h);
      printf("%f @ %f\n", (double) tsr.val, (double) tsr.timestamp);
    } else if (! strncmp(buffer, "lifemax", 7)) {
      tsr = go_mmavg_lifemax(&h);
      printf("%f @ %f\n", (double) tsr.val, (double) tsr.timestamp);
    } else if ('\n' == *buffer) {
      for (i1 = 0, ptr = h.start; i1 < h.num; i1++) {
	printf("%f ", (double) *ptr);
	ptr++;
	if (ptr == h.end_ptr) ptr = h.start_ptr;
      }
      printf("\n");
    } else {
      printf("?\n");
    }
  }
  
  return 0;
}

