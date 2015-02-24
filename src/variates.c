/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by
  statute is not subject to copyright in the United States.
  Recipients of this software assume all responsibility associated
  with its operation, modification, maintenance, and subsequent
  redistribution.
*/

/*
  This random number generator is the one described in "Random Number
  Generators: Good Ones are Hard to Find," Stephen K. Park and Keith
  W. Miller, Communications of the ACM, Volume 31, Number 10, October
  1988. This generator is proposed as the "minimal standard" that will
  port to all systems where the maximum integer size is 2^32 - 1 or
  larger.
*/

#include <math.h>
#include <float.h>
#include "variates.h"

#ifndef M_E
#define M_E 2.7182818284590452354
#endif

#define MODULUS 2147483647
#define A 16807
#define Q 127773
#define R 2836

#define HALFWAY_SEED 676806766

/*
  If you want separate chunks of random number generators, here are
  the seeds of 60 evenly-spread ranges with 35,791,394 in each:
*/

static long int seeds[] = {
  1101211447,
  732872487,
  1341477281,
  140551306,
  1089516043,
  2021127233,
  2144273118,
  592647446,
  590873088,
  1226723742,
  1925176231,
  1960025475,
  1840709629,
  211142264,
  793409767,
  1304948567,
  1434390853,
  1277767120,
  1240549668,
  725647116,
  1375081611,
  691153750,
  521732465,
  1223408244,
  1930996785,
  774234184,
  352861476,
  173289762,
  2027005741,
  1674350464,
  676806766,
  1550442304,
  2115150470,
  932366687,
  244525773,
  934251302,
  1481856611,
  1955136849,
  1521489386,
  1706503408,
  1589551955,
  247711646,
  1598466347,
  657953731,
  721826862,
  1316071563,
  771715511,
  218408793,
  1366473622,
  1012345181,
  1713378112,
  797556371,
  1268709810,
  1057452850,
  978975099,
  573050001,
  2129966605,
  936807553,
  1686740160,
  998658690,
};

long int get_random_bins(void)
{
  return sizeof(seeds) / sizeof(*seeds);
}

long int get_random_seed(long int key)
{
  unit_random_struct u;
  long int index;
  long int t;

  unit_random_init(&u);
  if (key < 0) key = -key;
  index = key % get_random_bins();
  unit_random_seed(&u, seeds[index]);
  for (t = 0; t < key; t++) {
    (void) unit_random_integer(&u);
  }

  return u.seed;
}

/*
  The term 'unit' refers to the uniform random number between
  zero and one.
*/

void unit_random_init(unit_random_struct *r)
{
  r->seed = 65521;
}

/*
  Returns an integer [1 ... MODULUS - 1], inclusive. Note that zero
  and MODULUS are never returned, and in fact if the seed were ever
  zero or MODULUS, then it would alternate between the two.
*/

long int unit_random_integer_min(unit_random_struct *r)
{
  return 1;
}

long int unit_random_integer_max(unit_random_struct *r)
{
  return MODULUS - 1;
}

long int unit_random_integer(unit_random_struct *r)
{
  long int lo, hi, test;

  hi = r->seed / Q;
  lo = r->seed % Q;
  test = A * lo - R * hi;
  
  if (test > 0)
    r->seed = test;
  else
    r->seed = test + MODULUS;

  return r->seed;
}

/*
  Returns a real number, [0.0 ... 0.999999999534339] by shifting down
  to zero and dividing by the MODULUS - 1 numbers possible.
*/

double unit_random_real(unit_random_struct *r)
{
  return ((double) (unit_random_integer(r) - 1)) / ((double) (MODULUS - 1));
}

/*
  Seeds the random number generator.
*/

void unit_random_seed(unit_random_struct *r, long int s)
{
  if (s <= 0 ) {
    r->seed = 1;
    return;
  }

  if (s == MODULUS) {
    r->seed = MODULUS - 1;
    return;
  }

  r->seed = ((long int) s) % MODULUS;
}

/*
  The uniform random functions just adds the [a,b) interval to a 
  unit random generator.
*/
void uniform_random_init(uniform_random_struct *r, double a, double b)
{
  unit_random_init(&r->u);
  uniform_random_set(r, a, b);
}

void uniform_random_set(uniform_random_struct *r, double a, double b)
{
  if (a < b) {
    r->min = a;
    r->diff = b - a;
  } else {
    r->min = b;
    r->diff = a - b;
  }
}

void uniform_random_seed(uniform_random_struct *r, long int s)
{
  unit_random_seed(&r->u, s);
}

double uniform_random_real(uniform_random_struct *r)
{
  return r->min + r->diff * unit_random_real(&r->u);
}

/*
  The normal, exponential, and weibull random number generators are
  those described in _Simulation Modeling and Analysis_, Averill
  M. Law and W. David Kelton, Second Edition, pp. 486, 490.
*/

void normal_random_init(normal_random_struct *r, double mean, double sd)
{
  unit_random_init(&r->u1);
  unit_random_init(&r->u2);
  unit_random_seed(&r->u2, HALFWAY_SEED);
  r->mean = mean;
  r->sd = sd;
  r->return_x2 = 0;
}

void normal_random_set(normal_random_struct *r, double mean, double sd)
{
  /* inhibit setting the same value since we saved x2 and recalculating
     this is expensive */
  if (fabs(mean - r->mean) > DBL_EPSILON) {
    r->mean = mean;
    r->sd = sd;
    r->return_x2 = 0;
  } else if (fabs(sd - r->sd) > DBL_EPSILON) {
    r->mean = mean;
    r->sd = sd;
    r->return_x2 = 0;
  }
}

void normal_random_seed(normal_random_struct *r, long int s1, long int s2)
{
  unit_random_seed(&r->u1, s1);
  unit_random_seed(&r->u2, s2);
}

double normal_random_real(normal_random_struct *r)
{
  double v1, v2;
  double w, y;

  if (0 != r->return_x2) {
    r->return_x2 = 0;
    return r->x2;
  }

  do {
    v1 = 2.0 * unit_random_real(&r->u1) - 1;
    v2 = 2.0 * unit_random_real(&r->u2) - 1;
    w = v1*v1 + v2*v2;
  } while (w > 1.0 || w < DBL_EPSILON);

  y = sqrt(-2.0 * log(w) / w);
  
  r->x1 = r->sd * (v1 * y) + r->mean;
  r->x2 = r->sd * (v2 * y) + r->mean;

  r->return_x2 = 1;

  return r->x1;
}

void weibull_random_init(weibull_random_struct *r, double alpha, double beta)
{
  unit_random_init(&r->u);
  weibull_random_set(r, alpha, beta);
}

void weibull_random_set(weibull_random_struct *r, double alpha, double beta)
{
  if (alpha < DBL_EPSILON) {
    r->degen = 1;
  } else {
    r->degen = 0;
    r->alpha_inv = 1.0 / alpha;
  }
  r->beta = beta;
}

void weibull_random_seed(weibull_random_struct *r, long int s)
{
  unit_random_seed(&r->u, s);
}

double weibull_random_real(weibull_random_struct *r)
{
  double v;

  if (r->degen) {
    /* no likelihood of the infinite spike at zero */
    return 0;
  }

  do {
    v = unit_random_real(&r->u);
  } while (v < DBL_EPSILON);
  
  return r->beta * pow(-log(v), r->alpha_inv);
}

void exponential_random_init(exponential_random_struct *r, double sd)
{
  unit_random_init(&r->u);
  r->sd = sd;			/* mean is the same as std dev */
}

void exponential_random_set(exponential_random_struct *r, double sd)
{
  r->sd = sd;
}

void exponential_random_seed(exponential_random_struct *r, long int s)
{
  unit_random_seed(&r->u, s);
}

double exponential_random_real(exponential_random_struct *r)
{
  double v;

  do {
    v = 1.0 - unit_random_real(&r->u);
  } while (v < DBL_EPSILON);

  return -r->sd * log(v);
}

void gamma_random_init(gamma_random_struct *r, double alpha, double beta)
{
  unit_random_init(&r->u1);
  unit_random_init(&r->u2);
  unit_random_seed(&r->u2, HALFWAY_SEED);
  gamma_random_set(r, alpha, beta);
}

void gamma_random_set(gamma_random_struct *r, double alpha, double beta)
{
  r->beta = beta;

  if (alpha - 1 < DBL_EPSILON) {
    /* for alpha = 1, this degenerates to the exponential distribution */
    r->range = 0;
    return;
  }

  if (alpha < 1) {
    r->range = 1;		/* 0 < alpha < 1 */
    r->alpha = alpha;
    r->alpha_inv = 1.0 / alpha;
    r->b = (M_E + alpha)/M_E;
    return;
  }

  r->range = 2;			/* 1 < alpha */
  r->alpha = alpha;
  r->a = sqrt(alpha+alpha-1);		/* intermediate 1/a*/
  r->q = alpha + r->a;		/* q = alpha + 1/a */
  r->a = 1.0 / r->a;		/* a is done */
  r->b = alpha - log(4.0);
  r->theta = 4.5;
  r->d = 1.0 + log(4.5);
}

void gamma_random_seed(gamma_random_struct *r, long int s1, long int s2)
{
  unit_random_seed(&r->u1, s1);
  unit_random_seed(&r->u2, s2);
}

double gamma_random_real(gamma_random_struct *r)
{
  double u1, u2;
  double p, y, v, z, w;

  if (r->range == 0) {
    /* alpha = 1, use exponential distribution */
    do {
      u1 = 1.0 - unit_random_real(&r->u1);
    } while (u1 < DBL_EPSILON);
    return -r->beta * log(u1);
  }

  if (r->range == 1) {
    /* 0 < alpha < 1 */
  STEP_A1:
    u1 = unit_random_real(&r->u1);
    p = r->b * u1;
    if (p > 1) goto STEP_A3;
    /* step 2 */
    y = pow(p, r->alpha_inv);
    u2 = unit_random_real(&r->u2);
    if (u2 <= exp(-y)) return y;
    /* otherwise go to step 1 */
    goto STEP_A1;
  STEP_A3:
    y = (r->b - p)*r->alpha_inv; /* intermediate y */
    if (y < DBL_EPSILON) return y;
    y = -log(y);
    u2 = unit_random_real(&r->u2);
    if (u2 <= pow(y, r->alpha-1)) return y;
    /* otherwise go to step 1 */
    goto STEP_A1;
  }

  /* else 1 < alpha */
 STEP_B1:
  do {
    u1 = unit_random_real(&r->u1);
  } while (u1 < DBL_EPSILON || (1-u1) < DBL_EPSILON);
  u2 = unit_random_real(&r->u2);
  v = r->a * log(u1/(1-u1));
  y = r->alpha * exp(v);
  z = u1*u1*u2;
  w = r->b + r->q*v - y;
  if (w + r->d - r->theta*z >= 0) return y;
  if (z < DBL_EPSILON || w >= log(z)) return y;
  goto STEP_B1;
}

void pearson_v_random_init(pearson_v_random_struct *r, double alpha, double beta)
{
  gamma_random_init(&r->g, alpha, beta < DBL_EPSILON ? DBL_MAX : 1.0 / beta);
}

void pearson_v_random_set(pearson_v_random_struct *r, double alpha, double beta)
{
  gamma_random_set(&r->g, alpha, beta < DBL_EPSILON ? DBL_MAX : 1.0 / beta);
}

void pearson_v_random_seed(pearson_v_random_struct *r, long int s1, long int s2)
{
  gamma_random_seed(&r->g, s1, s2);
}

double pearson_v_random_real(pearson_v_random_struct *r)
{
  double v;

  v = gamma_random_real(&r->g);

  return v < DBL_EPSILON ? DBL_MAX : 1.0 / v;
}

#ifdef MAIN

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "variates.h"

/*
  This function generates an equally-spaced number of random seeds
  that partition the space into independent generators.
*/
static int bins(int argc, char *argv[])
{
  unit_random_struct u;
  long int num_rands;
  long int bins = 60;
  long int num_in_bin;
  long int t, tt;

  if (argc > 1) {
    bins = atoi(argv[2]);
  }

  unit_random_init(&u);

  fprintf(stderr, "number of bins: %ld\n", get_random_bins());

  num_rands = unit_random_integer_max(&u) - unit_random_integer_min(&u) + 1;
  num_in_bin = num_rands / bins;

  for (t = 0; t < bins; t++) {
    printf("%ld\t%ld\n", t, unit_random_integer(&u));
    for (tt = 1; tt < num_in_bin; tt++) {
      (void) unit_random_integer(&u);
    }
  }

  return 0;
}

/*
  Usage: variates <type> <number to generate> <params ...>

  If <type> is "bins", then it generates the seeds for an equally-spaced
  <number> of bins. 

  Otherwise, <type> is one of unit, uniform, normal, exponential, weibull,
  gamma, or pearson_v.
*/

int main(int argc, char *argv[])
{
  unit_random_struct u;
  uniform_random_struct f;
  normal_random_struct n;
  exponential_random_struct e;
  weibull_random_struct w;
  gamma_random_struct g;
  pearson_v_random_struct pv;
  double a, b;
  int num;
  int t;

  if (argc < 3 ||
      1 != sscanf(argv[2], "%i", &num)) {
    fprintf(stderr, "usage: <type> <number to generate> <params ...>\n");
    return 1;
  }

  if (! strcmp(argv[1], "bins")) {
    return bins(argc, argv);
  } else if (! strcmp(argv[1], "unit")) {
    if (argc != 3) {
      fprintf(stderr, "usage: unit <number to generate>\n");
      return 1;
    }
    unit_random_init(&u);
    for (t = 0; t < num; t++) {
      printf("%f\n", unit_random_real(&u));
    }
  } else if (! strcmp(argv[1], "uniform")) {
    if (argc != 5 ||
	1 != sscanf(argv[3], "%lf", &a) ||
	1 != sscanf(argv[4], "%lf", &b)) {
      fprintf(stderr, "usage: uniform <number to generate> <a> <b>\n");
      return 1;
    }
    uniform_random_init(&f, a, b);
    for (t = 0; t < num; t++) {
      printf("%f\n", uniform_random_real(&f));
    }
  } else if (! strcmp(argv[1], "normal")) {
    if (argc != 5 ||
	1 != sscanf(argv[3], "%lf", &a) ||
	1 != sscanf(argv[4], "%lf", &b)) {
      fprintf(stderr, "usage: normal <number to generate> <mean> <std dev>\n");
      return 1;
    }
    normal_random_init(&n, a, b);
    for (t = 0; t < num; t++) {
      printf("%f\n", normal_random_real(&n));
    }
  } else if (! strcmp(argv[1], "exponential")) {
    if (argc != 4 ||
	1 != sscanf(argv[3], "%lf", &a)) {
      fprintf(stderr, "usage: exponential <number to generate> <mean>\n");
      return 1;
    }
    exponential_random_init(&e, a);
    for (t = 0; t < num; t++) {
      printf("%f\n", exponential_random_real(&e));
    }
  } else if (! strcmp(argv[1], "weibull")) {
    if (argc != 5 ||
	1 != sscanf(argv[3], "%lf", &a) ||
	1 != sscanf(argv[4], "%lf", &b)) {
      fprintf(stderr, "usage: weibull <number to generate> <shape> <scale>\n");
      return 1;
    }
    weibull_random_init(&w, a, b);
    for (t = 0; t < num; t++) {
      printf("%f\n", weibull_random_real(&w));
    }
  } else if (! strcmp(argv[1], "gamma")) {
    if (argc != 5 ||
	1 != sscanf(argv[3], "%lf", &a) ||
	1 != sscanf(argv[4], "%lf", &b)) {
      fprintf(stderr, "usage: gamma <number to generate> <shape> <scale>\n");
      return 1;
    }
    gamma_random_init(&g, a, b);
    for (t = 0; t < num; t++) {
      printf("%f\n", gamma_random_real(&g));
    }
  } else if (! strcmp(argv[1], "pearson_v")) {
    if (argc != 5 ||
	1 != sscanf(argv[3], "%lf", &a) ||
	1 != sscanf(argv[4], "%lf", &b)) {
      fprintf(stderr, "usage: pearson_v <number to generate> <shape> <scale>\n");
      return 1;
    }
    pearson_v_random_init(&pv, a, b);
    for (t = 0; t < num; t++) {
      printf("%f\n", pearson_v_random_real(&pv));
    }
  } else {
    fprintf(stderr, "usage: need one of\n unit\n uniform\n normal\n exponential\n weibull\n 'gamma\n pearson_v\n");
    return 1;
  }

  return 0;
}

#endif	/* MAIN */
