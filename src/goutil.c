/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

/*!
  \file goutil.c

  \brief Utility functions for random number generation, timestamps,
  min-max-average.
*/

#include <stddef.h>		/* NULL */
#include "gotypes.h"		/* go_integer */
#include "goutil.h"		/* these decls */

char *go_strncpy(char *dest, const char *src, go_integer n)
{
  char *ptr;
  go_integer i;

  for (ptr = dest, i = 0; i < n; ptr++, src++, i++) {
    *ptr = *src;
    if (0 == *src) break;
  }

  return dest;
}

#define MODULUS 2147483647
#define A 16807
#define Q 127773
#define R 2836
static long int seed = 65521;

/*
  Returns an integer [1 ... MODULUS - 1], inclusive. Note that zero
  and MODULUS are never returned, and in fact if the seed were ever
  zero or MODULUS, then it would alternate between the two.
*/

static long int go_random_integer(void)
{
  long int lo, hi, test;

  hi = seed / Q;
  lo = seed % Q;
  test = A * lo - R * hi;

  if (test > 0) seed = test;
  else seed = test + MODULUS;

  return seed;
}

/*
  Returns a real number, [0.0 ... 0.999999999534339] by shifting down
  to zero and dividing by the MODULUS - 1 numbers possible.
*/

go_real go_random(void)
{
  return ((go_real) (go_random_integer() - 1)) / ((go_real) (MODULUS - 1));
}

void go_random_seed(go_integer s)
{
  if (0 == s) {
    seed = 1;
    return;
  }

  if (s < 0) s = -s;

  seed = ((long int) s) % MODULUS;
}

go_result go_mmavg_init(go_mmavg * h, go_real * space, go_integer size, go_timestamp_func func)
{
  if (space == NULL) {
    h->start_ptr = &h->val[0];
    h->end_ptr = &h->val[GO_MMAVG_SIZE];
    h->size = GO_MMAVG_SIZE;
  } else {
    if (size <= 0) return GO_RESULT_ERROR;
    h->start_ptr = &space[0];
    h->end_ptr = &space[size];
    h->size = size;
  }
  h->start = h->start_ptr;
  h->end = h->start;
  h->inv_num = 0.0;		/* will get set upon first add */
  h->min = 0.0;
  h->max = 0.0;
  h->lifemin = 0.0;
  h->lifemax = 0.0;
  h->lifemin_ts = 0.0;
  h->lifemax_ts = 0.0;
  h->sum = 0.0;
  h->num = 0;
  h->id = 0;
  h->window_minmax = 0;		/* by default we don't keep track of
				   min and max in the window */
  h->ts_func = func;

  return GO_RESULT_OK;
}

go_result go_mmavg_window_minmax(go_mmavg * h, go_flag doit)
{
  h->window_minmax = doit;

  return GO_RESULT_OK;
}

static go_real go_mmavg_find_min(go_mmavg * h)
{
  go_real min;
  go_integer i;

  if (! h->window_minmax) {
    return h->lifemin;
  }

  min = h->start_ptr[0];
  for (i = 1; i < h->num; i++) {
    if (h->start_ptr[i] < min) min = h->start_ptr[i];
  }

  return min;
}

static go_real go_mmavg_find_max(go_mmavg * h)
{
  go_real max;
  go_integer i;

  if (! h->window_minmax) {
    return h->lifemax;
  }

  max = h->start_ptr[0];
  for (i = 1; i < h->num; i++) {
    if (h->start_ptr[i] > max) max = h->start_ptr[i];
  }

  return max;
}

go_result go_mmavg_add(go_mmavg * h, go_real val)
{
  go_real timestamp;
  go_real dropped;

  if (NULL == h->ts_func) {
    h->id++;
    timestamp = (go_real) h->id;
  } else {
    timestamp = h->ts_func();
  }

  dropped = *h->start;		/* save the start in case it's dropped */
  *h->end = val;		/* add val, possibly clobbering start */
  /* move the end pointer down to the next slot, wrapping around
     if necessary */
  h->end++;
  if (h->end == h->end_ptr) {
    h->end = h->start_ptr;
  }
  h->sum += val;		/* accumulate the val */

  if (h->num == 0) {
    /* first one, so set the min, maxes accordingly */
    h->num = 1;
    h->inv_num = 1.0;
    h->min = val;
    h->max = val;
    h->lifemin = val;
    h->lifemax = val;
    h->lifemin_ts = timestamp;
    h->lifemax_ts = timestamp;
  } else {
    if (h->num < h->size) {
      /* the history isn't full, so leave start pointer where it is,
	 and increment the number of elements in the history */
      h->num++;
      h->inv_num = 1.0 / h->num;
    } else {
      /* the history is full, so the saved value is indeed dropped.
	 Move the start pointer up, wrapping around if necessary. */
      h->start++;
      if (h->start == h->end_ptr) {
	h->start = h->start_ptr;
      }
      h->sum -= dropped;
      /* adjust min and max noting the loss of the dropped value */
      if (dropped <= h->min) {
	/* we dropped the min value, so find the new one. This is
	   an O(n) operation. */
	h->min = go_mmavg_find_min(h);
      } else if (dropped >= h->max) {
	/* we dropped the max value, so find the new one. This is
	   an O(n) operation. */
	h->max = go_mmavg_find_max(h);
      }
    }
    /* in either not-empty or full cases, we'll adjust min and max
       the same way */
    if (val < h->min) {
      h->min = val;
    } else if (val > h->max) {
      h->max = val;
    }
    if (val < h->lifemin) {
      h->lifemin = val;
      h->lifemin_ts = timestamp;
    } else if (val > h->lifemax) {
      h->lifemax = val;
      h->lifemax_ts = timestamp;
    }
  }

  return GO_RESULT_OK;
}

go_real go_mmavg_min(go_mmavg * h)
{
  return h->min;
}

go_real go_mmavg_max(go_mmavg * h)
{
  return h->max;
}

go_real go_mmavg_avg(go_mmavg * h)
{
  return h->sum * h->inv_num;
}

go_timestamped_real go_mmavg_lifemin(go_mmavg * h)
{
  go_timestamped_real tsr;

  tsr.val = h->lifemin;
  tsr.timestamp = h->lifemin_ts;

  return tsr;
}


go_timestamped_real go_mmavg_lifemax(go_mmavg * h)
{
  go_timestamped_real tsr;

  tsr.val = h->lifemax;
  tsr.timestamp = h->lifemax_ts;

  return tsr;
}

