#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>
#include <stdlib.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include "go.h"

#define PROG "sixdof"

typedef struct {
  double x, y, z;
  double r, p, w;
} pose;

typedef struct {
  pose a;
  pose b;
} posepair;

static posepair makeposepair(double d[12])
{
  posepair it;

  it.a.x = d[0];
  it.a.y = d[1];
  it.a.z = d[2];
  it.a.r = d[3];
  it.a.p = d[4];
  it.a.w = d[5];

  it.b.x = d[6];
  it.b.y = d[7];
  it.b.z = d[8];
  it.b.r = d[9];
  it.b.p = d[10];
  it.b.w = d[11];

  return it;
}

static void gsl_matrix_print(gsl_matrix *m)
{
  int row, col;

  printf("Transpose(<");
  for (row = 0; row < m->size1-1; row++) {
    printf("<");
    for (col = 0; col < m->size2-1; col++) {
      printf("%f,", gsl_matrix_get(m, row, col));
    }
    printf("%f>|", gsl_matrix_get(m, row, m->size2-1));
  }
  printf("<");
  for (col = 0; col < m->size2-1; col++) {
    printf("%f,", gsl_matrix_get(m, m->size1-1, col));
  }
  printf("%f>>)\n", gsl_matrix_get(m, m->size1-1, m->size2-1));
}

static void gsl_vector_print(gsl_vector *v)
{
  int index;

  printf("<");
  for (index = 0; index < v->size-1; index++) {
    printf("%f,", gsl_vector_get(v, index));
  }
  printf("%f>\n", gsl_vector_get(v, v->size-1));
}

static int gsl_matrix_rpy_convert(gsl_matrix *m, go_rpy *rpy)
{
  go_mat mat;

  if (m->size1 != 3) return -1;
  if (m->size2 != 3) return -1;

  mat.x.x = gsl_matrix_get(m, 0, 0);
  mat.y.x = gsl_matrix_get(m, 0, 1);
  mat.z.x = gsl_matrix_get(m, 0, 2);

  mat.x.y = gsl_matrix_get(m, 1, 0);
  mat.y.y = gsl_matrix_get(m, 1, 1);
  mat.z.y = gsl_matrix_get(m, 1, 2);

  mat.x.z = gsl_matrix_get(m, 2, 0);
  mat.y.z = gsl_matrix_get(m, 2, 1);
  mat.z.z = gsl_matrix_get(m, 2, 2);

  return GO_RESULT_OK == go_mat_rpy_convert(&mat, rpy) ? 0 : -1;
}

int main (int argc, char *argv[])
{
  FILE *fp;
  enum {BUFFERLEN = 256};
  char buffer[BUFFERLEN];
  char *ptr;
  double dees[12];
  int linenumber;
  int numpoints = 0;
  posepair *posepairarray = NULL;
  int posepairmax = 8;
  int t;
  go_rpy rpy;
  go_mat mat;
  gsl_matrix *X, *Xhat, *XXhatT, *V, *R;
  gsl_vector *S, *Work;

  if (argc < 2) {
    fp = stdin;
  } else {
    fp = fopen(argv[1], "r");
    if (NULL == fp) {
      fprintf(stderr, "%s: Can't open file ``%s'': %s\n", PROG, argv[1], strerror(errno));
      return 1;
    }
  }

  posepairarray = malloc(posepairmax * sizeof(*posepairarray));

  /* looking for lines with 12 numbers, X Y Z R P W X Y Z R P W */
  linenumber = 0;
  while (! feof(fp)) {
    if (NULL == fgets(buffer, sizeof(buffer)-1, fp)) break;
    linenumber++;
    ptr = buffer;
    while (isspace(*ptr)) ptr++;
    if (0 == *ptr) continue;	/* blank line */
    if ('#' == *ptr) continue;	/* comment */
    if (12 != sscanf(buffer, 
		     "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
		     &dees[0], &dees[1], &dees[2], &dees[3],
		     &dees[4], &dees[5], &dees[6], &dees[7],
		     &dees[8], &dees[9], &dees[10], &dees[11])) {
      fprintf(stderr, "%s: Bad input at line %d: %s\n", PROG, linenumber, buffer);
      return 1;
    }
    if (numpoints == posepairmax) {
      posepairmax *= 2;
      posepairarray = realloc(posepairarray, posepairmax * sizeof(*posepairarray));
    }
    posepairarray[numpoints++] = makeposepair(dees);
  }

  if (0 == numpoints) return 0;

  X = gsl_matrix_alloc(3, 4 * numpoints);
  Xhat = gsl_matrix_alloc(3, 4 * numpoints);
  XXhatT = gsl_matrix_alloc(3, 3);

  for (t = 0; t < numpoints; t++) {
    rpy.r = posepairarray[t].a.r;
    rpy.p = posepairarray[t].a.p;
    rpy.y = posepairarray[t].a.w;
    go_rpy_mat_convert(&rpy, &mat);

    gsl_matrix_set(X, 0, 4*t + 0, mat.x.x);
    gsl_matrix_set(X, 0, 4*t + 1, mat.y.x);
    gsl_matrix_set(X, 0, 4*t + 2, mat.z.x);
    gsl_matrix_set(X, 0, 4*t + 3, posepairarray[t].a.x);

    gsl_matrix_set(X, 1, 4*t + 0, mat.x.y);
    gsl_matrix_set(X, 1, 4*t + 1, mat.y.y);
    gsl_matrix_set(X, 1, 4*t + 2, mat.z.y);
    gsl_matrix_set(X, 1, 4*t + 3, posepairarray[t].a.y);

    gsl_matrix_set(X, 2, 4*t + 0, mat.x.z);
    gsl_matrix_set(X, 2, 4*t + 1, mat.y.z);
    gsl_matrix_set(X, 2, 4*t + 2, mat.z.z);
    gsl_matrix_set(X, 2, 4*t + 3, posepairarray[t].a.z);

    rpy.r = posepairarray[t].b.r;
    rpy.p = posepairarray[t].b.p;
    rpy.y = posepairarray[t].b.w;
    go_rpy_mat_convert(&rpy, &mat);

    gsl_matrix_set(Xhat, 0, 4*t + 0, mat.x.x);
    gsl_matrix_set(Xhat, 0, 4*t + 1, mat.y.x);
    gsl_matrix_set(Xhat, 0, 4*t + 2, mat.z.x);
    gsl_matrix_set(Xhat, 0, 4*t + 3, posepairarray[t].b.x);

    gsl_matrix_set(Xhat, 1, 4*t + 0, mat.x.y);
    gsl_matrix_set(Xhat, 1, 4*t + 1, mat.y.y);
    gsl_matrix_set(Xhat, 1, 4*t + 2, mat.z.y);
    gsl_matrix_set(Xhat, 1, 4*t + 3, posepairarray[t].b.y);

    gsl_matrix_set(Xhat, 2, 4*t + 0, mat.x.z);
    gsl_matrix_set(Xhat, 2, 4*t + 1, mat.y.z);
    gsl_matrix_set(Xhat, 2, 4*t + 2, mat.z.z);
    gsl_matrix_set(Xhat, 2, 4*t + 3, posepairarray[t].b.z);
  }

#if 0
  gsl_matrix_print(X);
  gsl_matrix_print(Xhat);
#endif

  /*
    int gsl_blas_dgemm(
    CBLAS_TRANSPOSE_t TransA,
    CBLAS_TRANSPOSE_t TransB, 
    double alpha, const gsl_matrix * A,
    const gsl_matrix * B,
    double beta, gsl_matrix * C);

    This function computes the matrix-matrix product and sum 
    C = alpha op(A) op(B) + beta C
    where op(A) = A or A^T for CblasNoTrans or CblasTrans,
    and similarly for the parameter TransB.
  */
  gsl_matrix_set_zero(XXhatT);
  gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, X, Xhat, 0.0, XXhatT);

#if 0
  gsl_matrix_print(XXhatT);
#endif

  /*
    int gsl_linalg_SV_decomp(
    gsl_matrix * A, gsl_matrix * V, gsl_vector * S, gsl_vector * work);

    This function factorizes the M-by-N matrix A into the singular
    value decomposition A = U S V^T for M >= N. On output the matrix A
    is replaced by U. The diagonal elements of the singular value
    matrix S are stored in the vector S. The singular values are
    non-negative and form a non-increasing sequence from S_1 to
    S_N. The matrix V contains the elements of V in untransposed
    form. To form the product U S V^T it is necessary to take the
    transpose of V. A workspace of length N is required in work.
  */

  V = gsl_matrix_alloc(3, 3);
  S = gsl_vector_alloc(3);
  Work = gsl_vector_alloc(3);

  gsl_linalg_SV_decomp(XXhatT, V, S, Work);
  /* now XXhatT is U */

#if 1
  gsl_matrix_print(XXhatT);
  gsl_matrix_print(V);
  gsl_vector_print(S);
#endif

  /* R = V U^T */
  R = gsl_matrix_alloc(3, 3);
  gsl_matrix_set_zero(R);
  gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, V, XXhatT, 0.0, R);

#if 0
  gsl_matrix_print(R);
#endif

  gsl_matrix_rpy_convert(R, &rpy);
  printf("%f %f %f\n", rpy.r, rpy.p, rpy.y);

  return 0;
}
