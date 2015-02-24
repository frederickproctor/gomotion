#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "go.h"

/*
  syntax: gointerptest c | p | pv | pdv | pva | pvda | pdva <T> <N>

  Reads points from stdin, puts onto interpolator and interpolates them
  to stdout.

  'c' means constant interpolation. One point per input line.

  'p' means linear interpolation. One point per input line.

  'pv', 'pdv' and 'pfv' mean cubic interpolation, with 'pv' signifying true
  pos-vel pairs, 'pdv' signifying position only with differenced vel, and
  'pfv' signifying point-fit. Two points per input line with 'pv', one point
  per line with 'pdv' and 'pfv'.

  'pva', 'pvda', 'pdva' and 'pfva' signify quintic interpolation, with 'pva'
  signifying true pos-vel-acc triples, 'pvda' signifying pos-vel with
  differenced acc, 'pdva' pos only with differenced vel-acc, and 'pfva'
  signifying point-fit. Three points per input line with 'pva', two with
  'pvda', and one with 'pdva' and 'pfva'.

  'T' is the total segment real time.
  'N' is the number of interpolations to do between points.
 */

int main(int argc, char *argv[])
{
  enum { BUFFERSIZE = 256 };
  char buffer[BUFFERSIZE];
  char *ptr;
  double d0, d1, d2;
  char order;			/* polynomial order, 0,1,3,5 */
  char diffs;			/* number of boundary derivs to estimate */
  double T;
  int N, n;
  go_real x0, x1, x2;
  go_interp interp;
  go_real time;			/* normalized time, 0..1 */
  go_real time_incr;		/* normalized time increment */
  double time_cum;		/* real cumulative time */
  double time_cum_incr;		/* real time increment */

  if (argc != 4) {
    fprintf(stderr,
	    "args: c | p | pv | pdv | pfv | pva | pvda | pdva | pfva <T> <N>\n");
    return 1;
  }

  if (1 != sscanf(argv[2], "%lf", &T) || T <= 0.0) {
    fprintf(stderr, "args: <T> positive\n");
    return 1;
  }

  if (1 != sscanf(argv[3], "%d", &N) || N <= 0) {
    fprintf(stderr, "args: <N> positive\n");
    return 1;
  }

  if (!strcmp(argv[1], "c")) {
    order = 0;
    diffs = 0;
  } else if (!strcmp(argv[1], "p")) {
    order = 1;
    diffs = 0;
  } else if (!strcmp(argv[1], "pv")) {
    order = 3;
    diffs = 0;
  } else if (!strcmp(argv[1], "pdv")) {
    order = 3;
    diffs = 1;
  } else if (!strcmp(argv[1], "pfv")) {
    order = 3;
    diffs = 2;
  } else if (!strcmp(argv[1], "pva")) {
    order = 5;
    diffs = 0;
  } else if (!strcmp(argv[1], "pvda")) {
    order = 5;
    diffs = 1;
  } else if (!strcmp(argv[1], "pdva")) {
    order = 5;
    diffs = 2;
  } else if (!strcmp(argv[1], "pfva")) {
    order = 5;
    diffs = 3;
  } else {
    fprintf(stderr,
	    "args: need p | pv | pdv | pfv | pva | pvda | pdva | pfva\n");
    return 1;
  }

  go_interp_init(&interp);
  time_incr = (go_real) (1.0 / ((double) N));
  time_cum_incr = (go_real) (T / ((double) N));
  time_cum = 0.0;

  while (!feof(stdin)) {
    if (NULL == fgets(buffer, BUFFERSIZE, stdin)) {
      break;
    }

    /* ignore comment or blank lines */
    ptr = buffer;
    while (isspace(*ptr)) {
      ptr++;
    }
    if (*ptr == ';' || *ptr == '#') {
      continue;
    }
    if (*ptr == 0) {
      continue;
    }

    if (order == 0) {
      if (1 != sscanf(buffer, "%lf", &d0)) {
	break;
      }
      x0 = (go_real) d0;
      go_interp_add_constant(&interp, x0);
    } else if (order == 1) {
      if (1 != sscanf(buffer, "%lf", &d0)) {
	break;
      }
      x0 = (go_real) d0;
      go_interp_add_linear(&interp, x0);
    } else if (order == 3) {
      if (diffs == 0) {		/* pv */
	if (2 != sscanf(buffer, "%lf %lf", &d0, &d1)) {
	  break;
	}
	x0 = (go_real) d0;
	x1 = (go_real) d1;
	go_interp_add_cubic_pv(&interp, x0, x1);
      } else if (diffs == 1) {	/* pdv */
	if (1 != sscanf(buffer, "%lf", &d0)) {
	  break;
	}
	x0 = (go_real) d0;
	go_interp_add_cubic_pdv(&interp, x0);
      } else {			/* pfv */
	if (1 != sscanf(buffer, "%lf", &d0)) {
	  break;
	}
	x0 = (go_real) d0;
	go_interp_add_cubic_pf(&interp, x0);
      }
    } else {
      if (diffs == 0) {		/* pva */
	if (3 != sscanf(buffer, "%lf %lf %lf", &d0, &d1, &d2)) {
	  break;
	}
	x0 = (go_real) d0;
	x1 = (go_real) d1;
	x2 = (go_real) d2;
	go_interp_add_quintic_pva(&interp, x0, x1, x2);
      } else if (diffs == 1) {	/* pvda */
	if (2 != sscanf(buffer, "%lf %lf", &d0, &d1)) {
	  break;
	}
	x0 = (go_real) d0;
	x1 = (go_real) d1;
	go_interp_add_quintic_pvda(&interp, x0, x1);
      } else if (diffs == 2) {	/* pdva */
	if (1 != sscanf(buffer, "%lf", &d0)) {
	  break;
	}
	x0 = (go_real) d0;
	go_interp_add_quintic_pdva(&interp, x0);
      } else {			/* pfva */
	if (1 != sscanf(buffer, "%lf", &d0)) {
	  break;
	}
	x0 = (go_real) d0;
	go_interp_add_quintic_pf(&interp, x0);
      }
    }

    /* now do the interpolation */
    for (n = 0, time = 0.0; n < N; n++, time += time_incr) {
      if (order == 0) {
	x0 = go_interp_eval_constant(&interp, time);
      } else if (order == 1) {
	x0 = go_interp_eval_linear(&interp, time);
      } else if (order == 3) {
	x0 = go_interp_eval_cubic(&interp, time);
      } else {
	x0 = go_interp_eval_quintic(&interp, time);
      }
      printf("%f\t%f\n", (double) time_cum, (double) x0);
      time_cum += time_cum_incr;
    }
  }				/* end while there's input */

  return 0;
}
