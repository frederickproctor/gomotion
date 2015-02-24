/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by
  statute is not subject to copyright in the United States.
  Recipients of this software assume all responsibility associated
  with its operation, modification, maintenance, and subsequent
  redistribution.
*/

#ifndef VARIATES_H
#define VARIATES_H

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

typedef struct {
  long int seed;
} unit_random_struct;

/* The underlying integer random numbers have been partitioned into bins
   of equally-spaced seeds for convenience in setting up independent
   random number generators. */

/* Returns the number of bins. You can have up to this many independent
   random number generators by seeding each with a seed retrieved from
   'get_random_seed'. It's set to 60, a number divisible by many other
   numbers. It can be changed by running through a manual process that
   cycles through the entire space and periodically prints out seeds. */
extern long int get_random_bins(void);

/* Returns a seed appropriate for this 'key', by finding which bin this
   key fits into via division modulo the number of bins, then moving up
   into that bin by 'key' steps. If the keys in the set are contiguous,
   the generators will be nicely spread out. Using keys [0 .. <bins-1>]
   is the recommended method. */
extern long int get_random_seed(long int key);

extern void unit_random_init(unit_random_struct *r);
extern void unit_random_seed(unit_random_struct *r, long int s);
extern long int unit_random_integer_min(unit_random_struct *r);
extern long int unit_random_integer_max(unit_random_struct *r);
extern long int unit_random_integer(unit_random_struct *r);
extern double unit_random_real(unit_random_struct *r);

typedef struct {
  unit_random_struct u;
  double min;
  double diff;
} uniform_random_struct;

extern void uniform_random_init(uniform_random_struct *r, double a, double b);
extern void uniform_random_set(uniform_random_struct *r, double a, double b);
extern void uniform_random_seed(uniform_random_struct *r, long int s);
extern double uniform_random_real(uniform_random_struct *r);

typedef struct {
  unit_random_struct u1, u2;
  double x1, x2;
  double mean;
  double sd;
  char return_x2;
} normal_random_struct;

extern void normal_random_init(normal_random_struct *r, double mean, double sd);
extern void normal_random_set(normal_random_struct *r, double mean, double sd);
extern void normal_random_seed(normal_random_struct *r, long int s1, long int s2);
extern double normal_random_real(normal_random_struct *r);

typedef struct {
  unit_random_struct u;
  double sd;
} exponential_random_struct;

extern void exponential_random_init(exponential_random_struct *r, double sd);
extern void exponential_random_set(exponential_random_struct *r, double sd);
extern void exponential_random_seed(exponential_random_struct *r, long int s);
extern double exponential_random_real(exponential_random_struct *r);

typedef struct {
  unit_random_struct u;
  double alpha_inv;
  double beta;
  char degen;
} weibull_random_struct;

extern void weibull_random_init(weibull_random_struct *r, double alpha, double beta);
extern void weibull_random_set(weibull_random_struct *r, double alpha, double beta);
extern void weibull_random_seed(weibull_random_struct *r, long int s);
extern double weibull_random_real(weibull_random_struct *r);

typedef struct {
  unit_random_struct u1, u2;
  double alpha, alpha_inv;
  double beta;
  double a, b, q, theta, d;
  char range;
} gamma_random_struct;

extern void gamma_random_init(gamma_random_struct *r, double alpha, double beta);
extern void gamma_random_set(gamma_random_struct *r, double alpha, double beta);
extern void gamma_random_seed(gamma_random_struct *r, long int s1, long int s2);
extern double gamma_random_real(gamma_random_struct *r);

typedef struct {
  gamma_random_struct g;
} pearson_v_random_struct;

extern void pearson_v_random_init(pearson_v_random_struct *r, double alpha, double beta);
extern void pearson_v_random_set(pearson_v_random_struct *r, double alpha, double beta);
extern void pearson_v_random_seed(pearson_v_random_struct *r, long int s1, long int s2);
extern double pearson_v_random_real(pearson_v_random_struct *r);

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif	/* VARIATES_H */

