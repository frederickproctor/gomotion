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
  \file gomathtest.c

  \brief A motley bunch of test routines for verifying that gomath.c works.
*/

#include <stdio.h>
#include <stdlib.h>		/* atoi */
#include <math.h>		/* fabs */
#include "go.h"

static int test_rotations(void)
{
  go_rvec rvec, rout;
  go_mat mat, mout;
  go_quat quat;
  go_zyz zyz;
  go_zyx zyx;
  go_rpy rpy;
  go_uxz uxz;
  go_real x, y, z;
  go_real start, end;
#define SET(_x,_y,_z) rvec.x = (_x), rvec.y = (_y), rvec.z = (_z)

  start = -GO_PI_2, end = GO_PI_2;

  for (x = start; x <= end; x += 0.1) {
    for (y = start; y <= end; y += 0.1) {
      for (z = start; z <= end; z += 0.1) {
	SET(x, y, z);
	go_rvec_mat_convert(&rvec, &mat);
	go_mat_quat_convert(&mat, &quat);
	go_quat_zyz_convert(&quat, &zyz);
	go_zyz_zyx_convert(&zyz, &zyx);
	go_zyx_rpy_convert(&zyx, &rpy);
	go_rpy_rvec_convert(&rpy, &rout);
	if (! go_rvec_rvec_compare(&rvec, &rout)) return 1;
	/* handle some new ones manually */
	go_mat_uxz_convert(&mat, &uxz);
	go_uxz_mat_convert(&uxz, &mout);
	if (! go_mat_is_norm(&mout)) return 1;
	go_mat_rvec_convert(&mout, &rout);
	if (! go_rvec_rvec_compare(&rvec, &rout)) return 1;
      }
    }
  }

  return 0;
#undef SET
}

static int test_cart_convert(void)
{
  go_cart c, cc;
  go_sph s;
  go_cyl l;

  c.x = 1;
  c.y = 2;
  c.z = -3;

  go_cart_sph_convert(&c, &s);
  go_sph_cart_convert(&s, &cc);
  if (! go_cart_cart_compare(&c, &cc)) {
    return 1;
  }

  go_cart_cyl_convert(&c, &l);
  go_cyl_cart_convert(&l, &cc);
  if (! go_cart_cart_compare(&c, &cc)) {
    return 1;
  }

  return 0;
}

static int test_quat_convert(void)
{
  go_rvec rot, rott;
  go_quat quat;

  rot.x = GO_PI_4;
  rot.y = 0.0;
  rot.z = 0.0;

  go_rvec_quat_convert(&rot, &quat);
  go_quat_rvec_convert(&quat, &rott);
  if (! go_rvec_rvec_compare(&rot, &rott)) {
    return 1;
  }

  return 0;
}

static int test_matrix_size(void)
{
  GO_MATRIX_DECLARE(A, Astg, 3, 2);
  GO_MATRIX_DECLARE(B, Bstg, 4, 3);

  go_matrix_init(A, Astg, 4, 3);
  go_matrix_init(B, Bstg, 3, 2);

  if (A.rows != 3 || A.cols != 2) return 1;
  if (B.rows != 3 || B.cols != 2) return 1;

  return 0;
}

static int test_matrix_matrix_mult(void)
{
  GO_MATRIX_DECLARE(A, Astg, 3, 2);
  GO_MATRIX_DECLARE(B, Bstg, 2, 4);
  GO_MATRIX_DECLARE(C, Cstg, 3, 4);
  GO_MATRIX_DECLARE(D, Dstg, 3, 3);
  GO_MATRIX_DECLARE(E, Estg, 2, 2);
  int row, col;

  go_matrix_init(A, Astg, 3, 2);
  go_matrix_init(B, Bstg, 2, 4);
  go_matrix_init(C, Cstg, 3, 4);
  go_matrix_init(D, Dstg, 3, 3);
  go_matrix_init(E, Estg, 2, 2);

  for (row = 0; row < A.rows; row++) {
    for (col = 0; col < A.cols; col++) {
      A.el[row][col] = row + col;
    }
  }

  for (row = 0; row < B.rows; row++) {
    for (col = 0; col < B.cols; col++) {
      B.el[row][col] = row + col;
    }
  }

  for (row = 0; row < D.rows; row++) {
    for (col = 0; col < D.cols; col++) {
      D.el[row][col] = row + col;
    }
  }

  for (row = 0; row < E.rows; row++) {
    for (col = 0; col < E.cols; col++) {
      E.el[row][col] = row + col;
    }
  }

  if (GO_RESULT_OK != go_matrix_matrix_mult(&A, &B, &C)) return 1;

  if (! GO_CLOSE(C.el[0][0], 1)) return 1;
  if (! GO_CLOSE(C.el[0][1], 2)) return 1;
  if (! GO_CLOSE(C.el[0][2], 3)) return 1;
  if (! GO_CLOSE(C.el[0][3], 4)) return 1;

  if (! GO_CLOSE(C.el[1][0], 2)) return 1;
  if (! GO_CLOSE(C.el[1][1], 5)) return 1;
  if (! GO_CLOSE(C.el[1][2], 8)) return 1;
  if (! GO_CLOSE(C.el[1][3], 11)) return 1;

  if (! GO_CLOSE(C.el[2][0], 3)) return 1;
  if (! GO_CLOSE(C.el[2][1], 8)) return 1;
  if (! GO_CLOSE(C.el[2][2], 13)) return 1;
  if (! GO_CLOSE(C.el[2][3], 18)) return 1;

  if (GO_RESULT_OK != go_matrix_matrix_mult(&A, &E, &A)) return 1;

  if (! GO_CLOSE(A.el[0][0], 1)) return 1;
  if (! GO_CLOSE(A.el[0][1], 2)) return 1;

  if (! GO_CLOSE(A.el[1][0], 2)) return 1;
  if (! GO_CLOSE(A.el[1][1], 5)) return 1;

  if (! GO_CLOSE(A.el[2][0], 3)) return 1;
  if (! GO_CLOSE(A.el[2][1], 8)) return 1;

  if (GO_RESULT_OK != go_matrix_matrix_mult(&E, &B, &B)) return 1;

  if (! GO_CLOSE(B.el[0][0], 1)) return 1;
  if (! GO_CLOSE(B.el[0][1], 2)) return 1;
  if (! GO_CLOSE(B.el[0][2], 3)) return 1;
  if (! GO_CLOSE(B.el[0][3], 4)) return 1;

  if (! GO_CLOSE(B.el[1][0], 2)) return 1;
  if (! GO_CLOSE(B.el[1][1], 5)) return 1;
  if (! GO_CLOSE(B.el[1][2], 8)) return 1;
  if (! GO_CLOSE(B.el[1][3], 11)) return 1;

  return 0;
}

static int test_matrix_vector_mult(void)
{
  GO_MATRIX_DECLARE(A, Astg, 3, 4);
  go_vector V[4] = {1, 2, 3, 4};
  go_vector AV[3];
  int row, col;

  go_matrix_init(A, Astg, 3, 4);

  for (row = 0; row < A.rows; row++) {
    for (col = 0; col < A.cols; col++) {
      A.el[row][col] = row + col;
    }
  }

  if (GO_RESULT_OK != go_matrix_vector_mult(&A, V, AV)) return 1;

  if (! GO_CLOSE(AV[0], 20)) return 1;
  if (! GO_CLOSE(AV[1], 30)) return 1;
  if (! GO_CLOSE(AV[2], 40)) return 1;

  if (GO_RESULT_OK != go_matrix_vector_mult(&A, V, V)) return 1;

  if (! GO_CLOSE(V[0], 20)) return 1;
  if (! GO_CLOSE(V[1], 30)) return 1;
  if (! GO_CLOSE(V[2], 40)) return 1;

  return 0;
}

/*
  Tests if (AV) x B = (A x B)V, where x is generalized (column-wise)
  cross product.
*/
static int test_matrix_vector_cross(void)
{
  GO_MATRIX_DECLARE(A, Astg, 3, 4); /* 3x4 */
  GO_MATRIX_DECLARE(AxB, AxBstg, 3, 4); /* 3x4 */
  go_vector V[4] = {1, 2, 3, 4};	/* 4x1 */
  go_vector AV[3];		/* 3x4 . 4x1 = 3x1 */
  go_cart AVc;
  go_vector B[3] = {5, 6, 7};			/* 3x1 */
  go_cart Bc = {5, 6, 7};			/* 3x1 */
  go_cart AVxBc;
  go_vector AxBV[3];
  int row, col;

  go_matrix_init(A, Astg, 3, 4);
  go_matrix_init(AxB, AxBstg, 3, 4);

  for (row = 0; row < A.rows; row++) {
    for (col = 0; col < A.cols; col++) {
      A.el[row][col] = row + col;
    }
  }

  if (GO_RESULT_OK != go_matrix_vector_mult(&A, V, AV)) return 1;
  AVc.x = AV[0], AVc.y = AV[1], AVc.z = AV[2];
  if (GO_RESULT_OK != go_cart_cart_cross(&AVc, &Bc, &AVxBc)) return 1;

  if (GO_RESULT_OK != go_matrix_vector_cross(&A, B, &AxB)) return 1;
  if (GO_RESULT_OK != go_matrix_vector_mult(&AxB, V, AxBV)) return 1;

  if (! GO_CLOSE(AVxBc.x, AxBV[0])) return 1;
  if (! GO_CLOSE(AVxBc.y, AxBV[1])) return 1;
  if (! GO_CLOSE(AVxBc.z, AxBV[2])) return 1;

  if (GO_RESULT_OK != go_matrix_vector_cross(&A, B, &A)) return 1;
  if (GO_RESULT_OK != go_matrix_vector_mult(&A, V, AxBV)) return 1;

  if (! GO_CLOSE(AVxBc.x, AxBV[0])) return 1;
  if (! GO_CLOSE(AVxBc.y, AxBV[1])) return 1;
  if (! GO_CLOSE(AVxBc.z, AxBV[2])) return 1;

  return 0;
}

static int test_matrix_transpose(void)
{
  GO_MATRIX_DECLARE(A, Astg, 3, 2);
  GO_MATRIX_DECLARE(AT, ATstg, 2, 3);
  GO_MATRIX_DECLARE(B, Bstg, 3, 3);
  int row, col;

  go_matrix_init(A, Astg, 3, 2);
  go_matrix_init(AT, ATstg, 2, 3);
  go_matrix_init(B, Bstg, 3, 3);

  for (row = 0; row < A.rows; row++) {
    for (col = 0; col < A.cols; col++) {
      A.el[row][col] = row + col;
    }
  }

  for (row = 0; row < B.rows; row++) {
    for (col = 0; col < B.cols; col++) {
      B.el[row][col] = row + col;
    }
  }

  if (GO_RESULT_OK != go_matrix_transpose(&A, &AT)) return 1;

  for (row = 0; row < AT.rows; row++) {
    for (col = 0; col< AT.cols; col++) {
      if (! GO_CLOSE(AT.el[row][col], A.el[col][row])) return 1;
    }
  }

  if (GO_RESULT_OK != go_matrix_transpose(&B, &B)) return 1;

  for (row = 0; row < B.rows; row++) {
    for (col = 0; col< B.cols; col++) {
      if (! GO_CLOSE(B.el[col][row], row + col)) return 1;
    }
  }

  return 0;
}

static int test_matrix_invert(void)
{
  GO_MATRIX_DECLARE(a_m, a_mstg, 6, 6);
  GO_MATRIX_DECLARE(a_mcpy, a_mcpystg, 6, 6);
  GO_MATRIX_DECLARE(ainv_m, ainv_mstg, 6, 6);
  GO_MATRIX_DECLARE(I_m, I_mstg, 6, 6);
  int row, col;

  go_matrix_init(a_m, a_mstg, 6, 6);
  go_matrix_init(a_mcpy, a_mcpystg, 6, 6);
  go_matrix_init(ainv_m, ainv_mstg, 6, 6);
  go_matrix_init(I_m, I_mstg, 6, 6);
  
  for (row = 0; row < 6; row++) {
    for (col = 0; col < 6; col++) {
      a_m.el[row][col] = go_random();
    }
  }

  go_matrix_matrix_copy(&a_m, &a_mcpy);

  if (GO_RESULT_OK != go_matrix_inv(&a_m, &ainv_m)) return 1;

  go_matrix_matrix_mult(&a_m,
			&ainv_m,
			&I_m);

  for (row = 0; row < 6; row++) {
    for (col = 0; col < 6; col++) {
      if (row == col) {
	if (! GO_CLOSE(I_m.el[row][col], 1)) return 1;
      }	else {
	if (! GO_CLOSE(I_m.el[row][col], 0)) return 1;
      }
    }
  }

  if (GO_RESULT_OK != go_matrix_inv(&a_m, &a_m)) return 1;

  go_matrix_matrix_mult(&a_mcpy,
			&a_m,
			&I_m);

  for (row = 0; row < 6; row++) {
    for (col = 0; col < 6; col++) {
      if (row == col) {
	if (! GO_CLOSE(I_m.el[row][col], 1)) return 1;
      }	else {
	if (! GO_CLOSE(I_m.el[row][col], 0)) return 1;
      }
    }
  }

  return 0;
}

static int test_dh_convert(void)
{
  go_dh dhin, dhout;
  go_pose p;

  for (dhin.a = -1.0; dhin.a <= 1.0; dhin.a += 0.1) {
    for (dhin.alpha = -1.0; dhin.alpha <= 1.0; dhin.alpha += 0.1) {
      for (dhin.d = -1.0; dhin.d <= 1.0; dhin.d += 0.1) {
	for (dhin.theta = -1.0; dhin.theta <= 1.0; dhin.theta += 0.1) {
	  go_dh_pose_convert(&dhin, &p);
	  go_pose_dh_convert(&p, &dhout);
	  if (! GO_TRAN_CLOSE(dhin.a, dhout.a) ||
	      ! GO_ROT_CLOSE(dhin.alpha, dhout.alpha) ||
	      ! GO_TRAN_CLOSE(dhin.d, dhout.d) ||
	      ! GO_ROT_CLOSE(dhin.theta, dhout.theta)) {
	    return 1;
	  }
	}
      }
    }
  }

  return 0;
}

static go_complex eval_quadratic(go_quadratic * quad, go_complex z)
{
  go_complex result;
  go_complex zn;

  result.re = quad->b, result.im = 0.0;
  zn = z;
  result = go_complex_add(result, go_complex_scale(zn, quad->a));
  zn = go_complex_mult(zn, z);	/* z^2 */
  result = go_complex_add(result, zn);

  return result;
}

static go_complex eval_cubic(go_cubic * cub, go_complex z)
{
  go_complex result;
  go_complex zn;

  result.re = cub->c, result.im = 0.0;
  zn = z;
  result = go_complex_add(result, go_complex_scale(zn, cub->b));
  zn = go_complex_mult(zn, z);	/* z^2 */
  result = go_complex_add(result, go_complex_scale(zn, cub->a));
  zn = go_complex_mult(zn, z);	/* z^3 */
  result = go_complex_add(result, zn);

  return result;
}

static go_complex eval_quartic(go_quartic * quart, go_complex z)
{
  go_complex result;
  go_complex zn;

  result.re = quart->d, result.im = 0.0;
  zn = z;
  result = go_complex_add(result, go_complex_scale(zn, quart->c));
  zn = go_complex_mult(zn, z);	/* z^2 */
  result = go_complex_add(result, go_complex_scale(zn, quart->b));
  zn = go_complex_mult(zn, z);	/* z^3 */
  result = go_complex_add(result, go_complex_scale(zn, quart->a));
  zn = go_complex_mult(zn, z);	/* z^4 */
  result = go_complex_add(result, zn);

  return result;
}

/* GO_SMALL is too small for our root tests, which are good to about 1e-5 */
#define TEST_SMALL(x) (fabs(x) < 1.0e-4)

static int test_quadratic(void)
{
  int a, b;
  go_quadratic quad;
  go_complex z1, z2, y;

#define START -10
#define END 10
  for (a = START; a <= END; a++) {
    for (b = START; b <= END; b++) {
      quad.a = a, quad.b = b;
      if (GO_RESULT_OK != 
	  go_quadratic_solve(&quad, &z1, &z2)) {
	printf("go_quadratic_solve failed on %d %d\n", a, b);
	return 1;
      }
#define ROOT_FAIL(ZZ)							\
      y = eval_quadratic(&quad, ZZ);					\
      if (! TEST_SMALL(y.re) || ! TEST_SMALL(y.im)) {			\
	printf("eval_quadratic failed on %d %d, root %f + %f I -> %f + %f I\n",	\
	       a, b, (double) ZZ.re, (double) ZZ.im, (double) y.re, (double) y.im); \
      }
      ROOT_FAIL(z1);
      ROOT_FAIL(z2);
    }
  }

#undef ROOT_FAIL
#undef START
#undef END

  return 0;
}

/* the cubic solution for floats needs to be relaxed */
#if defined(GO_REAL_FLOAT)
#undef TEST_SMALL
#define TEST_SMALL(x) (fabs(x) < 0.001)
#endif

static int test_cubic(void)
{
  int a, b, c;
  go_cubic cub;
  go_complex z1, z2, z3, y;

#define START -10
#define END 10
  for (a = START; a <= END; a++) {
    for (b = START; b <= END; b++) {
      for (c = START; c <= END; c++) {
	cub.a = a, cub.b = b, cub.c = c;
	if (GO_RESULT_OK != 
	    go_cubic_solve(&cub, &z1, &z2, &z3)) {
	  printf("go_cubic_solve failed on %d %d %d\n", a, b, c);
	  return 1;
	}
#define ROOT_FAIL(ZZ)							\
	y = eval_cubic(&cub, ZZ);					\
	if (! TEST_SMALL(y.re) || ! TEST_SMALL(y.im)) {			\
	  printf("eval_cubic failed on %d %d %d, root %f + %f I -> %g + %g I\n", \
		 a, b, c, (double) ZZ.re, (double) ZZ.im, (double) y.re, (double) y.im); \
	}
	ROOT_FAIL(z1);
	ROOT_FAIL(z2);
	ROOT_FAIL(z3);
      }
    }
  }

#undef ROOT_FAIL
#undef START
#undef END

  return 0;
}

/* the quartic solution isn't very good - relax TEST_SMALL */
#if defined(GO_REAL_LONG_DOUBLE)
#undef TEST_SMALL
#define TEST_SMALL(x) (fabs(x) < 0.002)
#elif defined(GO_REAL_DOUBLE)
#undef TEST_SMALL
#define TEST_SMALL(x) (fabs(x) < 0.002)
#elif defined(GO_REAL_FLOAT)
#undef TEST_SMALL
#define TEST_SMALL(x) (fabs(x) < 0.9)
#endif

static int test_quartic(void)
{
  int a, b, c, d;
  go_quartic quart;
  go_complex z1, z2, z3, z4, y;

#define START -10
#define END 10
  for (a = START; a <= END; a++) {
    for (b = START; b <= END; b++) {
      for (c = START; c <= END; c++) {
	for (d = START; d <= END; d++) {
	  quart.a = a, quart.b = b, quart.c = c, quart.d = d;
	  if (GO_RESULT_OK != 
	      go_quartic_solve(&quart, &z1, &z2, &z3, &z4)) {
	    printf("go_quartic_solve failed on %d %d %d %d\n", a, b, c, d);
	    return 1;
	  }
#define ROOT_FAIL(ZZ)							\
	  y = eval_quartic(&quart, ZZ);					\
	  if (! TEST_SMALL(y.re) || ! TEST_SMALL(y.im)) {			\
	    printf("eval_quartic failed on %d %d %d %d, root %f + %f I -> %f + %f I (%d)\n",	\
		   a, b, c, d, (double) ZZ.re, (double) ZZ.im, (double) y.re, (double) y.im, (int) gocode); \
	  }
	  ROOT_FAIL(z1);
	  ROOT_FAIL(z2);
	  ROOT_FAIL(z3);
	  ROOT_FAIL(z4);
	}
      }
    }
  }

#undef ROOT_FAIL
#undef START
#undef END

  return 0;
}

static void mat_vec_mult(go_real ** mat, go_real * vec, go_integer n, go_real * vout)
{
  int row, col;

  for (row = 0; row < n; row++) {
    vout[row] = 0.0;
    for (col = 0; col < n; col++) {
      vout[row] += mat[row][col] * vec[col];
    }
  }
}

static void vec_scal_mult(go_real * vec, go_real scal, go_integer n, go_real * vout)
{
  int row;

  for (row = 0; row < n; row++) {
    vout[row] = vec[row] * scal;
  }
}

static int test_eigenvectors(void)
{
  go_real min0[3] = {1., 2., 3.};
  go_real min1[3] = {2., -5., 7.};
  go_real min2[3] = {3., 7., -2.};
  go_real * min[3] = {min0, min1, min2};
  go_real mout0[3];
  go_real mout1[3];
  go_real mout2[3];
  go_real * mout[3] = {mout0, mout1, mout2};
  go_real d[3], e[3], eigenvec[3], Ax[3], lambda_x[3];
  int row, col;

  /* copy min to mout, since mout will be changed */
  for (row = 0; row < 3; row++) {
    for (col = 0; col < 3; col++) {
      mout[row][col] = min[row][col];
    }
  }

  /* compute eigenvals and eigenvecs */
  go_tridiag_reduce(mout, 3, d, e);
  go_tridiag_ql(d, e, 3, mout);
  /* matrix[] columns are eigenvectors, d[] holds the eigenvalues */

  /* check them, morig * matrix[col] == d[col] * matrix[col] */
  for (col = 0; col < 3; col++) {
    for (row = 0; row < 3; row++) {
      eigenvec[row] = mout[row][col];
    }
    mat_vec_mult(min, eigenvec, 3, Ax);
    vec_scal_mult(eigenvec, d[col], 3, lambda_x);
    for (row = 0; row < 3; row++) {
      if (! GO_SMALL(Ax[row] - lambda_x[row])) return 1;
    }
  }

  return 0;
}

static int test_cart_cart_pose(void)
{
  go_cart v1[4] = {{1.23, 2.34, 3.45},
		   {0.98, -0.84, -0.32},
		   {-5.6, 6.7, -7.8},
		   {-10, -11, -12}};
  go_cart v2[4], v1c[4], v2c[4];
  go_rpy rpy = {0.1, -0.2, 0.3};
  go_pose pose = {{1, -2, 3}, {1, 0, 0, 0}}, pout;
  int t;

  /* fill in our starting pose with convenient RPY numbers */
  go_rpy_quat_convert(&rpy, &pose.rot);

  /* transform the v1 vectors by our starting pose to get v2 vectors */
  for (t = 0; t < 4; t++) {
    go_pose_cart_mult(&pose, &v1[t], &v2[t]);
    /* and tweak a bit */
#define TWEAK 0.1
    v2[t].x += TWEAK * (go_random() - 0.5);
    v2[t].y += TWEAK * (go_random() - 0.5);
    v2[t].z += TWEAK * (go_random() - 0.5);
  }

  /* compute the starting pose given v1 and v2 */
  go_cart_cart_pose(v1, v2, v1c, v2c, 4, &pout);

  /* transform the v1 by the computed pose and compare against the v2 */
  for (t = 0; t < 4; t++) {
    go_pose_cart_mult(&pout, &v1[t], &v1[t]);
    if (fabs(v1[t].x - v2[t].x) > TWEAK) return 1;
    if (fabs(v1[t].y - v2[t].y) > TWEAK) return 1;
    if (fabs(v1[t].z - v2[t].z) > TWEAK) return 1;
  }

  return 0;
#undef TWEAK
}

static int test_cart_trilaterate(void)
{
  go_cart c1 = {2, 1, 1};
  go_cart c2 = {5, 2, 1};
  go_cart c3 = {1, 4, 1};
  go_cart p = {3, 2, 1};
  go_cart diff;
  go_cart out1, out2;
  go_real l1, l2, l3;
  go_result retval;

  go_cart_cart_sub(&p, &c1, &diff);
  go_cart_mag(&diff, &l1);
  go_cart_cart_sub(&p, &c2, &diff);
  go_cart_mag(&diff, &l2);
  go_cart_cart_sub(&p, &c3, &diff);
  go_cart_mag(&diff, &l3);

  retval = go_cart_trilaterate(&c1, &c2, &c3, l1, l2, l3, &out1, &out2);
  if (GO_RESULT_OK != retval) return retval;

  if (! go_cart_cart_compare(&p, &out1) ||
      ! go_cart_cart_compare(&p, &out2)) {
    return 1;
  }

  return 0;
}

#define BARFLINE {printf("line %d ", __LINE__); return 1;}

static int test_lines_and_planes(void)
{
  int t;
  go_real d1, d2, distance;
  go_cart point, direction;
  go_cart p1, p2, p3;
  go_line line1, line2, line3;
  go_plane plane1, plane2, plane3;
  go_flag f1, f2;

  point.x = go_random();
  point.y = go_random();
  point.z = go_random();
  do {
    direction.x = go_random();
    direction.y = go_random();
    direction.z = go_random();
  } while (GO_RESULT_OK != go_line_from_point_direction(&point, &direction, &line1));

  go_line_evaluate(&line1, -1.0, &p1);
  go_line_evaluate(&line1, 2.0, &p2);

  go_line_from_points(&p1, &p2, &line2);
  if (! go_line_line_compare(&line1, &line2)) BARFLINE;

  line3 = line2;
  line3.point.x += (10.0 * GO_REAL_EPSILON);
  if (go_line_line_compare(&line1, &line3)) BARFLINE;

  line3 = line2;
  line3.direction.z += (10.0 * GO_REAL_EPSILON);
  if (go_line_line_compare(&line1, &line3)) BARFLINE;

  go_cart_normal(&line1.direction, &direction);
  go_cart_cart_add(&line1.point, &direction, &direction);
  go_point_line_distance(&direction, &line1, &d1);
  if (! GO_CLOSE(d1, 1)) BARFLINE;

  /* test planes for various initializations */
  go_plane_from_abcd(1, 2, 3, 4, &plane1);
  go_plane_evaluate(&plane1, 0, 1, &p1);
  go_plane_evaluate(&plane1, 1, 0, &p2);
  go_plane_evaluate(&plane1, 1, 1, &p3);

  /* from points */
  go_plane_from_points(&p1, &p2, &p3, &plane2);
  f1 = go_plane_plane_compare(&plane1, &plane2);
  /* switch handedness and build another plane */
  go_plane_from_points(&p1, &p3, &p2, &plane3);
  f2 = go_plane_plane_compare(&plane1, &plane3);
  /* exactly one should equal the original plane */
  if (f1 == f2) BARFLINE;

  /* from point-line */
  go_line_from_points(&p2, &p3, &line1);
  go_plane_from_point_line(&p1, &line1, &plane2);
  f1 = go_plane_plane_compare(&plane1, &plane2);
  /* switch handedness and build another plane */
  go_line_from_points(&p3, &p2, &line1);
  go_plane_from_point_line(&p1, &line1, &plane3);
  f2 = go_plane_plane_compare(&plane1, &plane3);
  /* exactly one should equal the original plane */
  if (f1 == f2) BARFLINE;

  go_plane_from_point_normal(&line1.point, &line1.direction, &plane1);
  go_line_evaluate(&line1, d1 = go_random(), &p1);
  go_point_plane_distance(&p1, &plane1, &d2);
  if (! GO_CLOSE(d1, d2)) BARFLINE;

  go_plane_evaluate(&plane1, go_random(), go_random(), &p1);
  go_point_plane_distance(&p1, &plane1, &d1);
  if (! GO_SMALL(d1)) BARFLINE;

  /* test line and plane intersections */

  /* X axis perp to YZ plane */
  p1.x = 0, p1.y = 0, p1.z = 0;
  p2.x = 1, p2.y = 0, p2.z = 0;
  go_line_from_points(&p1, &p2, &line1);
  p1.x = 0, p1.y = 0, p1.z = 0;
  p2.x = 1, p2.y = 0, p2.z = 0;
  go_plane_from_point_normal(&p1, &p2, &plane1);
  if (GO_RESULT_OK != go_line_plane_intersect(&line1, &plane1, &point, &distance)) BARFLINE;
  if (! go_cart_cart_compare(&p1, &point)) BARFLINE;
  if (! GO_SMALL(distance)) BARFLINE;

  /* X axis in XY plane, error */
  p1.x = 0, p1.y = 0, p1.z = 0;
  p2.x = 1, p2.y = 0, p2.z = 0;
  go_line_from_points(&p1, &p2, &line1);
  p1.x = 0, p1.y = 0, p1.z = 0;
  p2.x = 0, p2.y = 0, p2.z = 1;
  go_plane_from_point_normal(&p1, &p2, &plane1);
  if (GO_RESULT_OK == go_line_plane_intersect(&line1, &plane1, &point, &distance)) BARFLINE;

  /* Y axis up at Z=1 perp to XZ plane */
  p1.x = 0, p1.y = 0, p1.z = 1;
  p2.x = 0, p2.y = 1, p2.z = 1;
  go_line_from_points(&p1, &p2, &line1);
  p1.x = 0, p1.y = 0, p1.z = 0;
  p2.x = 0, p2.y = 1, p2.z = 0;
  go_plane_from_point_normal(&p1, &p2, &plane1);
  if (GO_RESULT_OK != go_line_plane_intersect(&line1, &plane1, &point, &distance)) BARFLINE;
  if (! GO_CLOSE(point.x, 0) ||
      ! GO_CLOSE(point.y, 0) ||
      ! GO_CLOSE(point.z, 1) ||
      ! GO_CLOSE(distance, 0)) BARFLINE;

  /* line 1,1,1 at 1,1,1 to plane Z=3 */
  p1.x = 1, p1.y = 1, p1.z = 1;
  p2.x = 2, p2.y = 2, p2.z = 2;
  go_line_from_points(&p1, &p2, &line1);
  p1.x = 0, p1.y = 0, p1.z = 3;
  p2.x = 0, p2.y = 0, p2.z = 1;
  go_plane_from_point_normal(&p1, &p2, &plane1);
  if (GO_RESULT_OK != go_line_plane_intersect(&line1, &plane1, &point, &distance)) BARFLINE;
  if (! GO_CLOSE(point.x, 3) ||
      ! GO_CLOSE(point.y, 3) ||
      ! GO_CLOSE(point.z, 3)) BARFLINE;
  if (! GO_CLOSE(distance, 2*sqrt(3))) BARFLINE;

  /* try some random ones */
  for (t = 0; t < 100; t++) {
    p1.x = go_random(), p1.y = go_random(), p1.z = go_random();
    p2.x = go_random(), p2.y = go_random(), p2.z = go_random();
    if (GO_RESULT_OK != go_line_from_points(&p1, &p2, &line1)) continue;
    p1.x = go_random(), p1.y = go_random(), p1.z = go_random();
    p2.x = go_random(), p2.y = go_random(), p2.z = go_random();
    p3.x = go_random(), p3.y = go_random(), p3.z = go_random();
    if (GO_RESULT_OK != go_plane_from_points(&p1, &p2, &p3, &plane1)) continue;
    if (GO_RESULT_OK == go_line_plane_intersect(&line1, &plane1, &point, &distance)) {
      go_line_evaluate(&line1, distance, &p1);
      go_point_plane_distance(&p1, &plane1, &d1);
      if (! GO_SMALL(d1)) BARFLINE;
    }
  }

  /* test line from planes */
  /* ZX-XY gives X axis */
  plane1.normal.x = 0, plane1.normal.y = 1, plane1.normal.z = 0, plane1.d = 0;
  plane2.normal.x = 0, plane2.normal.y = 0, plane2.normal.z = 1, plane2.d = 0;
  if (GO_RESULT_OK != go_line_from_planes(&plane1, &plane2, &line1)) BARFLINE;
  if (! GO_CLOSE(line1.direction.x, 1) ||
      ! GO_CLOSE(line1.direction.y, 0) ||
      ! GO_CLOSE(line1.direction.z, 0)) BARFLINE;
  /* XY-YZ gives Y axis */
  plane1.normal.x = 0, plane1.normal.y = 0, plane1.normal.z = 1, plane1.d = 0;
  plane2.normal.x = 1, plane2.normal.y = 0, plane2.normal.z = 0, plane2.d = 0;
  if (GO_RESULT_OK != go_line_from_planes(&plane1, &plane2, &line1)) BARFLINE;
  if (! GO_CLOSE(line1.direction.x, 0) ||
      ! GO_CLOSE(line1.direction.y, 1) ||
      ! GO_CLOSE(line1.direction.z, 0)) BARFLINE;
  /* YZ-ZX gives Z axis */
  plane1.normal.x = 1, plane1.normal.y = 0, plane1.normal.z = 0, plane1.d = 0;
  plane2.normal.x = 0, plane2.normal.y = 1, plane2.normal.z = 0, plane2.d = 0;
  if (GO_RESULT_OK != go_line_from_planes(&plane1, &plane2, &line1)) BARFLINE;
  if (! GO_CLOSE(line1.direction.x, 0) ||
      ! GO_CLOSE(line1.direction.y, 0) ||
      ! GO_CLOSE(line1.direction.z, 1)) BARFLINE;
  /* try some random ones, checking that the line is in both planes */
  for (t = 0; t < 100; t++) {
    if (GO_RESULT_OK != go_plane_from_abcd(go_random(), go_random(), go_random(), go_random(), &plane1)) continue;
    if (GO_RESULT_OK != go_plane_from_abcd(go_random(), go_random(), go_random(), go_random(), &plane2)) continue;
    if (GO_RESULT_OK != go_line_from_planes(&plane1, &plane2, &line1)) continue;
    /* check for point lying in plane */
    (void) go_cart_cart_dot(&line1.point, &plane1.normal, &d1);
    d1 += plane1.d;
    if (! GO_SMALL(d1)) BARFLINE;
    (void) go_cart_cart_dot(&line1.point, &plane2.normal, &d1);
    d1 += plane2.d;
    if (! GO_SMALL(d1)) BARFLINE;
  }

  return 0;
}

static int see_dh_pose(void)
{
  enum {BUFFERLEN = 80};
  char buffer[BUFFERLEN];
  go_rpy rpy;
  go_pose pose;
  go_dh dh;
  double d1, d2, d3, d4, d5, d6;

  while (! feof(stdin)) {
    printf("Enter the pose of the link, X Y Z R P W:  ");
    fflush(stdout);
    if (NULL == fgets(buffer, BUFFERLEN, stdin)) break;
    if (6 != sscanf(buffer, "%lf %lf %lf %lf %lf %lf",
		    &d1, &d2, &d3, &d4, &d5, &d6)) break;
    pose.tran.x = d1;
    pose.tran.y = d2;
    pose.tran.z = d3;
    rpy.r = GO_TO_RAD(d4);
    rpy.p = GO_TO_RAD(d5);
    rpy.y = GO_TO_RAD(d6);
    go_rpy_quat_convert(&rpy, &pose.rot);
    go_pose_dh_convert(&pose, &dh);
    printf("%f %f %f %f\n", 
	   (double) dh.a, (double) GO_TO_DEG(dh.alpha), (double) dh.d, (double) GO_TO_DEG(dh.theta));
    go_dh_pose_convert(&dh, &pose);
    go_quat_rpy_convert(&pose.rot, &rpy);
    printf("%f %f %f %f %f %f\n",
	   (double) pose.tran.x, (double) pose.tran.y, (double) pose.tran.z,
	   (double) GO_TO_DEG(rpy.r), (double) GO_TO_DEG(rpy.p), (double) GO_TO_DEG(rpy.y));
    go_pose_dh_convert(&pose, &dh);
    printf("%f %f %f %f\n", 
	   (double) dh.a, (double) GO_TO_DEG(dh.alpha), (double) dh.d, (double) GO_TO_DEG(dh.theta));
  }

  return 0;
}

static int see_pose_pose_interp(void)
{
  go_pose p1, p2, p3;
  go_real t1, t2, t3;
  go_rpy rpy;
  go_real tincr;

  p1.tran.x = 1, p1.tran.y = -2, p1.tran.z = 3;
  rpy.r = GO_TO_RAD(10), rpy.p = GO_TO_RAD(-20), rpy.y = GO_TO_RAD(30);
  go_rpy_quat_convert(&rpy, &p1.rot);
  t1 = 1;

  p2.tran.x = -2, p2.tran.y = 3, p2.tran.z = 5;
  rpy.r = GO_TO_RAD(-20), rpy.p = GO_TO_RAD(30), rpy.y = GO_TO_RAD(40);
  go_rpy_quat_convert(&rpy, &p2.rot);
  t2 = -3;

  for (tincr = 0.1 * (t2 - t1), t3 = t1; t2 > t1 ? t3 < t2 + 0.5 * tincr : t3 > t2 + 0.5 * tincr; t3 += tincr) {
    go_pose_pose_interp(t1, &p1, t2, &p2, t3, &p3);
    go_quat_rpy_convert(&p3.rot, &rpy);
    printf("%f %f %f %f %f %f %f\n", (double) t3,
	   (double) p3.tran.x, (double) p3.tran.y, (double) p3.tran.z,
	   (double) GO_TO_DEG(rpy.r), (double) GO_TO_DEG(rpy.p), (double) GO_TO_DEG(rpy.y));
  }

  return 0;
}

static int test_cart_cart_rot(void)
{
  go_cart v1, v2, v3;
  go_quat q1, q2;
  go_rvec rvec;
  int t;

  /* try some extreme cases */

  /* zero vectors should give errors */
  v1.x = 0, v1.y = 0, v1.z = 0;
  v2 = v1;
  if (GO_RESULT_OK == go_cart_cart_rot(&v1, &v2, &q1)) return 1;

  /* parallel X */
  v1.x = 1, v1.y = 0, v1.z = 0;
  v2 = v1;
#undef DOIT
#define DOIT \
  if (GO_RESULT_OK != go_cart_cart_rot(&v1, &v2, &q1)) return 1; \
  (void) go_quat_cart_mult(&q1, &v1, &v3); \
  if (! go_cart_cart_compare(&v2, &v3)) return 1
  DOIT;

  /* parallel Y */
  v1.x = 0, v1.y = 1, v1.z = 0;
  v2 = v1;
  DOIT;

  /* parallel Z */
  v1.x = 0, v1.y = 0, v1.z = 1;
  v2 = v1;
  DOIT;

  /* parallel random */
  v1.x = go_random(), v1.y = go_random(), v1.z = go_random();
  v2 = v1;
  DOIT;

  /* antiparallel X */
  v1.x = 1, v1.y = 0, v1.z = 0;
  go_cart_neg(&v1, &v2);
  DOIT;

  /* antiparallel Y */
  v1.x = 0, v1.y = 1, v1.z = 0;
  go_cart_neg(&v1, &v2);
  DOIT;

  /* antiparallel Z */
  v1.x = 0, v1.y = 0, v1.z = 1;
  go_cart_neg(&v1, &v2);
  DOIT;

  /* antiparallel random */
  v1.x = go_random(), v1.y = go_random(), v1.z = go_random();
  go_cart_neg(&v1, &v2);
  DOIT;

  /* try a bunch of random cases */
  for (t = 0; t < 10; t++) {
    do {
      do {
	v1.x = go_random(), v1.y = go_random(), v1.z = go_random();
      } while (GO_RESULT_OK != go_cart_unit(&v1, &v1));
      go_cart_scale_mult(&v1, go_random() * GO_2_PI - GO_PI, &v1);
      go_cart_rvec_convert(&v1, &rvec);
    } while (GO_RESULT_OK != go_rvec_quat_convert(&rvec, &q1));
    v1.x = go_random(), v1.y = go_random(), v1.z = go_random();
    (void) go_quat_cart_mult(&q1, &v1, &v2);
    if (GO_RESULT_OK != go_cart_cart_rot(&v1, &v2, &q2)) return 1;
    (void) go_quat_cart_mult(&q2, &v1, &v3);
    if (! go_cart_cart_compare(&v2, &v3)) return 1;
  }

  return 0;
}

static int test_mat3_inv(void)
{
  go_real a[3][3];
  go_real vin[3], vout[3];
  go_result retval;
  int i, row, col;

  for (i = 0; i < 1000; i++) {
    for (row = 0; row < 3; row++) {
      for (col = 0; col < 3; col++) {
	a[row][col] = go_random();
      }
      vin[row] = go_random();
    }

    go_mat3_vec3_mult(a, vin, vout);

    retval = go_mat3_inv(a, a);

    if (GO_RESULT_OK == retval) {
      go_mat3_vec3_mult(a, vout, vout);
      for (row = 0; row < 3; row++) {
	if (! GO_CLOSE(vout[row], vin[row])) {
	  return 1;
	}
      }
    }
  }

  return 0;
}

static int test_mat4_inv(void)
{
  go_real a[4][4];
  go_real vin[4], vout[4];
  go_result retval;
  int i, row, col;

  for (i = 0; i < 1000; i++) {
    for (row = 0; row < 4; row++) {
      for (col = 0; col < 4; col++) {
	a[row][col] = go_random();
      }
      vin[row] = go_random();
    }

    go_mat4_vec4_mult(a, vin, vout);

    retval = go_mat4_inv(a, a);

    if (GO_RESULT_OK == retval) {
      go_mat4_vec4_mult(a, vout, vout);
      for (row = 0; row < 4; row++) {
	if (! GO_CLOSE(vout[row], vin[row])) {
	  return 1;
	}
      }
    }
  }

  return 0;
}

static int test_mat6_inv(void)
{
  go_real a[6][6];
  go_real vin[6], vout[6];
  go_result retval;
  int i, row, col;

  for (i = 0; i < 1000; i++) {
    for (row = 0; row < 6; row++) {
      for (col = 0; col < 6; col++) {
	a[row][col] = go_random();
      }
      vin[row] = go_random();
    }

    go_mat6_vec6_mult(a, vin, vout);

    retval = go_mat6_inv(a, a);

    if (GO_RESULT_OK == retval) {
      go_mat6_vec6_mult(a, vout, vout);
      for (row = 0; row < 6; row++) {
	if (! GO_CLOSE(vout[row], vin[row])) {
	  return 1;
	}
      }
    }
  }

  return 0;
}

int test_point_line_proj()
{
  go_cart point, pout, vout;
  go_line line;
  int i;

  for (i = 0; i < 1000; i++) {
    point.x = go_random();
    point.y = go_random();
    point.z = go_random();
    line.point.x = go_random();
    line.point.y = go_random();
    line.point.z = go_random();
    line.direction.x = go_random();
    line.direction.y = go_random();
    line.direction.z = go_random();

    if (GO_RESULT_OK == go_point_line_proj(&point, &line, &pout)) {
      /* test that pout is on the line */
      go_cart_cart_sub(&pout, &line.point, &vout);
      if (! go_cart_cart_par(&line.direction, &vout)) return 1;
      /* test that point-pout is perpendicular to the line */
      go_cart_cart_sub(&point, &pout, &vout);
      if (! go_cart_cart_perp(&vout, &line.direction)) return 1;
    }
  }

  return 0;
}

int test_point_plane_proj()
{
  go_cart point, pout, vout;
  go_plane plane;
  int i;

  for (i = 0; i < 1000; i++) {
    point.x = go_random();
    point.y = go_random();
    point.z = go_random();
    plane.normal.x = go_random();
    plane.normal.y = go_random();
    plane.normal.z = go_random();
    plane.d = go_random();
    if (GO_RESULT_OK == go_point_plane_proj(&point, &plane, &pout)) {
      /* test that pout is in the plane */
      if (! GO_TRAN_SMALL(plane.normal.x*pout.x +
			  plane.normal.y*pout.y +
			  plane.normal.z*pout.z +
			  plane.d)) return 1;
      /* test that point-pout is perpendicular to the plane */
      go_cart_cart_sub(&point, &pout, &vout);
      if (! go_cart_cart_par(&plane.normal, &vout)) return 1;
    }
  }

  return 0;
}

int test_line_plane_proj()
{
  go_line line, lout;
  go_plane plane;
  go_cart vout;
  int i;

  for (i = 0; i < 1000; i++) {
    line.point.x = go_random();
    line.point.y = go_random();
    line.point.z = go_random();
    line.direction.x = go_random();
    line.direction.y = go_random();
    line.direction.z = go_random();
    plane.normal.x = go_random();
    plane.normal.y = go_random();
    plane.normal.z = go_random();
    plane.d = go_random();
    if (GO_RESULT_OK == go_line_plane_proj(&line, &plane, &lout)) {
      /* test that a point on lout is in the plane */
      if (! GO_TRAN_SMALL(plane.normal.x*lout.point.x +
			  plane.normal.y*lout.point.y +
			  plane.normal.z*lout.point.z +
			  plane.d)) return 1;
      /* test that lout is perp to the plane normal */
      if (! go_cart_cart_perp(&plane.normal, &lout.direction)) return 1;
      /* test that line cross lout is in the plane */
      go_cart_cart_cross(&line.direction, &lout.direction, &vout);
      if (! go_cart_cart_perp(&plane.normal, &vout)) return 1;
    }
  }

  return 0;
}

int test_linear_cos_sin_solve()
{
  go_real a, b, th1, th2;
  int i;

  for (i = 0; i < 1000; i++) {
    a = go_random();
    b = go_random();
    if (GO_RESULT_OK == go_linear_cos_sin_solve(a, b, &th1, &th2)) {
      if (! GO_ROT_CLOSE(a*cos(th1) + b*sin(th1), 1)) return 1;
      if (! GO_ROT_CLOSE(a*cos(th2) + b*sin(th2), 1)) return 1;
    }
  }

  return 0;
}

int test_atrigs()
{
  go_real v, thp, thn;
  int i;

  for (i = 0; i < 1000; i++) {
    v = sin(GO_2_PI * go_random());
    if (GO_RESULT_OK != go_asines(v, &thp, &thn)) {
      printf("asines bad for %f\n", (double) v);
      return 1;
    }
    if (thp < 0 || thn > 0) {
      printf("asines not split: %f %f %f\n", (double) v, (double) thp, (double) thn);
      return 1;
    }
    if (! GO_ROT_CLOSE(v, sin(thp))) {
      printf("asin pos not close: %f %f\n", (double) v, (double) thp);
      return 1;
    }
    if (! GO_ROT_CLOSE(v, sin(thn))) {
      printf("asin neg not close: %f %f\n", (double) v, (double) thn);
      return 1;
    }
  }

  for (i = 0; i < 1000; i++) {
    v = cos(GO_2_PI * go_random());
    if (GO_RESULT_OK != go_acoses(v, &thp, &thn)) {
      printf("acoses bad for %f\n", (double) v);
      return 1;
    }
    if (thp < 0 || thn > 0) {
      printf("acoses not split: %f %f %f\n", (double) v, (double) thp, (double) thn);
      return 1;
    }
    if (! GO_ROT_CLOSE(v, cos(thp))) {
      printf("acos pos not close: %f %f\n", (double) v, (double) thp);
      return 1;
    }
    if (! GO_ROT_CLOSE(v, cos(thn))) {
      printf("acos neg not close: %f %f\n", (double) v, (double) thn);
      return 1;
    }
  }

  for (i = 0; i < 1000; i++) {
    v = tan(GO_2_PI * go_random());
    if (GO_RESULT_OK != go_atans(v, &thp, &thn)) {
      printf("atans bad for %f\n", (double) v);
      return 1;
    }
    if (thp < 0 || thn > 0) {
      printf("atans not split: %f %f %f\n", (double) v, (double) thp, (double) thn);
      return 1;
    }
    if (! GO_ROT_CLOSE(v, tan(thp))) {
      printf("atan pos not close: %f %f\n", (double) v, (double) thp);
      return 1;
    }
    if (! GO_ROT_CLOSE(v, tan(thn))) {
      printf("atan neg not close: %f %f\n", (double) v, (double) thn);
      return 1;
    }
  }

  return 0;
}

/*
  Usage: gomathtest {<option number>}

  Option '1' enters an interactive pose-to-DH-parameter session
  Option '2' prints some hard-coded pose interpolation
*/

int main(int argc, char *argv[])
{
  int which;

  if (go_init()) {
    printf("go_init error\n");
    return 1;
  }

  if (argc > 1) {
    which = atoi(argv[1]);
    switch (which) {
    case 1:
      return see_dh_pose();
      break;
    case 2:
      return see_pose_pose_interp();
      break;
    default:
      fprintf(stderr, "no test case for option %d\n", which);
      return 1;
      break;
    }
  }

  printf("test_rotations: ");
  fflush(stdout);
  if (test_rotations()) {
    printf("failed\n");
    return 1;
  }
  printf("ok\n");

  printf("test_cart_convert: ");
  fflush(stdout);
  if (test_cart_convert()) {
    printf("failed\n");
    return 1;
  }
  printf("ok\n");

  printf("test_quat_convert: ");
  fflush(stdout);
  if (test_quat_convert()) {
    printf("failed\n");
    return 1;
  }
  printf("ok\n");

  printf("test_matrix_size: ");
  fflush(stdout);
  if (test_matrix_size()) {
    printf("failed\n");
    return 1;
  }
  printf("ok\n");

  printf("test_matrix_matrix_mult: ");
  fflush(stdout);
  if (test_matrix_matrix_mult()) {
    printf("failed\n");
    return 1;
  }
  printf("ok\n");

  printf("test_matrix_vector_mult: ");
  fflush(stdout);
  if (test_matrix_vector_mult()) {
    printf("failed\n");
    return 1;
  }
  printf("ok\n");

  printf("test_matrix_vector_cross: ");
  fflush(stdout);
  if (test_matrix_vector_cross()) {
    printf("failed\n");
    return 1;
  }
  printf("ok\n");

  printf("test_matrix_transpose: ");
  fflush(stdout);
  if (test_matrix_transpose()) {
    printf("failed\n");
    return 1;
  }
  printf("ok\n");

  printf("test_matrix_invert: ");
  fflush(stdout);
  if (test_matrix_invert()) {
    printf("failed\n");
    return 1;
  }
  printf("ok\n");

  printf("test_dh_convert: ");
  fflush(stdout);
  if (test_dh_convert()) {
    printf("failed\n");
    return 1;
  }
  printf("ok\n");

  printf("test_quadratic: ");
  fflush(stdout);
  if (test_quadratic()) {
    printf("failed\n");
    return 1;
  }
  printf("ok\n");

  printf("test_cubic: ");
  fflush(stdout);
  if (test_cubic()) {
    printf("failed\n");
    return 1;
  }
  printf("ok\n");

  printf("test_quartic: ");
  fflush(stdout);
  if (test_quartic()) {
    printf("failed\n");
    return 1;
  }
  printf("ok\n");

  printf("test_eigenvectors: ");
  fflush(stdout);
  if (test_eigenvectors()) {
    printf("failed\n");
    return 1;
  }
  printf("ok\n");

  printf("test_cart_cart_pose: ");
  fflush(stdout);
  if (test_cart_cart_pose()) {
    printf("failed\n");
    return 1;
  }
  printf("ok\n");

  printf("test_cart_trilaterate: ");
  fflush(stdout);
  if (test_cart_trilaterate()) {
    printf("failed\n");
    return 1;
  }
  printf("ok\n");

  printf("test_lines_and_planes: ");
  fflush(stdout);
  if (test_lines_and_planes()) {
    printf("failed\n");
    return 1;
  }
  printf("ok\n");

  printf("test_cart_cart_rot: ");
  fflush(stdout);
  if (test_cart_cart_rot()) {
    printf("failed\n");
    return 1;
  }
  printf("ok\n");

  printf("test_mat3_inv: ");
  fflush(stdout);
  if (test_mat3_inv()) {
    printf("failed\n");
    return 1;
  }
  printf("ok\n");

  printf("test_mat4_inv: ");
  fflush(stdout);
  if (test_mat4_inv()) {
    printf("failed\n");
    return 1;
  }
  printf("ok\n");

  printf("test_mat6_inv: ");
  fflush(stdout);
  if (test_mat6_inv()) {
    printf("failed\n");
    return 1;
  }
  printf("ok\n");

  printf("test_point_line_proj: ");
  fflush(stdout);
  if (test_point_line_proj()) {
    printf("failed\n");
    return 1;
  }
  printf("ok\n");

  printf("test_point_plane_proj: ");
  fflush(stdout);
  if (test_point_plane_proj()) {
    printf("failed\n");
    return 1;
  }
  printf("ok\n");

  printf("test_line_plane_proj: ");
  fflush(stdout);
  if (test_line_plane_proj()) {
    printf("failed\n");
    return 1;
  }
  printf("ok\n");

  printf("test_linear_cos_sin_solve: ");
  fflush(stdout);
  if (test_linear_cos_sin_solve()) {
    printf("failed\n");
    return 1;
  }
  printf("ok\n");

  printf("test_atrigs: ");
  fflush(stdout);
  if (test_atrigs()) {
    printf("failed\n");
    return 1;
  }
  printf("ok\n");

  return 0;
}
