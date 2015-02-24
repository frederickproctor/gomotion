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
  \file goutil.h

  \brief Utility functions for random number generation, timestamps,
  min-max-average.
*/

#ifndef GOUTIL_H
#define GOUTIL_H

#include <stddef.h>
#include "gotypes.h"		/* go_integer */

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

/* number of elements in an array */
#define GO_ARRAYELS(ARRAY) (sizeof(ARRAY)/sizeof(*(ARRAY)))

/* non-zero if the INDEX is out of bounds for the ARRAY */
#define GO_ARRAYBAD(ARRAY,INDEX) (((INDEX) < 0) || ((INDEX) >= (sizeof(ARRAY)/sizeof(*(ARRAY)))))

extern char *go_strncpy(char *dest, const char *src, go_integer n);

/*!
  \defgroup RNG Random Number Generator

  The Go Motion random number generator is the one described in
  "Random Number Generators: Good Ones are Hard to Find," Stephen
  K. Park and Keith W. Miller, Communications of the ACM, Volume 31,
  Number 10, October 1988.
  
  This generator is proposed as the "minimal standard" that will port
  to all systems where the maximum integer size is 2^32 - 1 or larger.

  The generating function is z(n+1) = a*z mod m, where m is the
  modulus 2^32 - 1. It generates all numbers in the range 1..m-1, or 1
  to 2147483646. 0 and the modulus 2^32-1 are never generated, so two
  numbers out of the 2^32 possible in the four bytes of a long int are
  not generated, and 2^32-2 are generated.

  The function is shifted so that numbers in the range 0..2^32-3 are
  returned, and a normalized function go_random() is provided that
  returns a double, the shifted integer divided by 2^32-2, resulting
  in a floating point range [0.0000 .. 0.99999X], where 0.99999X is
  (2^32-3)/(2^32-2). This can be multiplied by an integer M and
  rounded to produce random integers in the range 0..M-1 with almost
  balanced distribution.
*/

/*!
  Returns a double in the range [0.000000 .. 0.99999X].  You can
  multiply this by an integer M to get random numbers in the range
  [0..M-1], approximately evenly distributed.
*/
extern go_real go_random(void);

/*!
  Seeds the random number generator with \a s, so you can start
  at a different sequence point than the default.
*/
extern void go_random_seed(go_integer s);

/*!
  \defgroup MMAVG Min-Max-Average History

  The Go Motion min-max-average functions provide a history of the
  minimum, maximum and average values of elements placed onto the
  queue, computed over the length of a window of entries. Old
  elements fall out of the window as new ones are placed on, if the
  window is full. Lifetime min and max values are maintained along
  with their timestamps.

  History of min and max values in the window can be inhibited to
  save time, in which case the min and max values are the lifetime
  values.
*/

enum {GO_MMAVG_SIZE = 100};

typedef go_real (*go_timestamp_func)(void);

typedef struct {
  go_real val[GO_MMAVG_SIZE];	/*< the actual stored values  */
  go_real * start_ptr;		/*< a pointer to the start */
  go_real * end_ptr;		/*< a pointer just past the end */
  go_real * start;		/*< a pointer to the oldest value  */
  go_real * end;		/*< a pointer to the next available spot */
  go_real inv_num;		/*< 1/num, for efficient computation  */
  go_real min;			/*< the min value in the window */
  go_real max;			/*< the max value in the window */
  go_real lifemin;		/*< the min value over the lifetime */
  go_real lifemax;		/*< the max value over the lifetime */
  go_real lifemin_ts;		/*< the timestamp of the lifetime min */
  go_real lifemax_ts;		/*< the timestamp of the lifetime max */
  go_real sum;			/*< sum of elements */
  go_integer size;		/*< total size of history */
  go_integer num;		/*< how many are in the history */
  go_integer id;		/*< incremented for each addition,
				  used for default timestamping  */
  go_flag window_minmax;	/*< if true, keep track of the min and max
				  in the averaging window */
  go_timestamp_func ts_func;	/*< the user's time stamp function */
} go_mmavg;

typedef struct {
  go_real val;
  go_real timestamp;
} go_timestamped_real;

/*!
  Initialize the min-max-average history. If \a space is not \c NULL, it
  should point to the first element of an array of \a size \c go_real types.
  Otherwise the default space of \c GO_MMAVG_SIZE elements is used.
 */
extern go_result go_mmavg_init(go_mmavg * h, go_real * space, go_integer size, go_timestamp_func func);

/*!
  Turn off or on the min/max tracking in the window. If \a doit is
  non-zero, then tracking is turned on, and \a go_mmavg_min and
  \a go_mmavg_max will return the min and max values in the current window
  of \a size elements as established by \a go_mmavg_init. If \a doit is
  zero, then these functions will return the lifetime min and max values.
  Turning off min/max tracking in the window will speed things up, since
  searching through the window is avoided.
*/
extern go_result go_mmavg_window_minmax(go_mmavg * h, go_flag doit);

extern go_result go_mmavg_add(go_mmavg * h, go_real val);

extern go_real go_mmavg_min(go_mmavg * h);

extern go_real go_mmavg_max(go_mmavg * h);

extern go_real go_mmavg_avg(go_mmavg * h);

extern go_timestamped_real go_mmavg_lifemin(go_mmavg * h);

extern go_timestamped_real go_mmavg_lifemax(go_mmavg * h);

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif /* GOUTIL_H */
