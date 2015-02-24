/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

/*
  genhexkins.c

  Originally written by R. Brian Register, 1999

  These are the forward and inverse kinematic functions for a class of
  machines referred to as "Stewart Platforms".

  The genhex_kin_fwd function solves the forward kinematics using an
  iterative algorithm.  Due to the iterative nature of this algorithm
  the function requires an initial value to begin the iterative
  routine and then converges to the "nearest" solution. The forward
  kinematics problem is given the strut lengths and returns the pose
  of the platform.  For this problem there arein multiple solutions.
  The function will return only one of these solutions which will be
  the solution nearest to the initial value given.  It is possible
  that there are no solutions "near" the given initial value and the
  iteration will not converge and no solution will be returned.
  Assuming there is a solution "near" the initial value, the function
  will always return one correct solution out of the multiple possible
  solutions.
*/

#include <math.h>
#include "gotypes.h"		/* go_result, go_integer */
#include "gomath.h"		/* go_pose */
#include "genhexkins.h"

/******************************* MatInvert() ***************************/

/*
  This is a function that inverts a 6x6 matrix.
*/

static go_result mat_invert(go_real jac[][GENHEX_NUM_JOINTS], 
			    go_real inv_jac[][GENHEX_NUM_JOINTS])
{
  go_real j_aug[GENHEX_NUM_JOINTS][12], m, temp;
  go_integer j, k, n;

  /* This function determines the inverse of a 6x6 matrix using
     Gauss-Jordan elimination */

  /* Augment the Identity matrix to the Jacobian matrix */

  for (j = 0; j <= 5; ++j) {
    for (k = 0; k <= 5; ++k) {	/* Assign J matrix to first 6 columns of AugJ */
      j_aug[j][k] = jac[j][k];
    }
    for (k = 6; k <= 11; ++k) {	/* Assign I matrix to last six columns of AugJ */
      if (k - 6 == j) {
	j_aug[j][k] = 1;
      } else {
	j_aug[j][k] = 0;
      }
    }
  }

  /* Perform Gauss elimination */
  for (k = 0; k <= 4; ++k) {	/* Pivot        */
    if ((j_aug[k][k] < 0.01) && (j_aug[k][k] > -0.01)) {
      for (j = k + 1; j <= 5; ++j) {
	if ((j_aug[j][k] > 0.01) || (j_aug[j][k] < -0.01)) {
	  for (n = 0; n <= 11; ++n) {
	    temp = j_aug[k][n];
	    j_aug[k][n] = j_aug[j][n];
	    j_aug[j][n] = temp;
	  }
	  break;
	}
      }
    }
    for (j = k + 1; j <= 5; ++j) {	/* Pivot */
      m = -j_aug[j][k] / j_aug[k][k];
      for (n = 0; n <= 11; ++n) {
	j_aug[j][n] = j_aug[j][n] + m * j_aug[k][n];	/* (Row j) + m * (Row k) */
	if ((j_aug[j][n] < 0.000001) && (j_aug[j][n] > -0.000001)) {
	  j_aug[j][n] = 0;
	}
      }
    }
  }

  /* Normalization of Diagonal Terms */
  for (j = 0; j <= 5; ++j) {
    m = 1 / j_aug[j][j];
    for (k = 0; k <= 11; ++k) {
      j_aug[j][k] = m * j_aug[j][k];
    }
  }

  /* Perform Gauss Jordan Steps */
  for (k = 5; k >= 0; --k) {
    for (j = k - 1; j >= 0; --j) {
      m = -j_aug[j][k] / j_aug[k][k];
      for (n = 0; n <= 11; ++n) {
	j_aug[j][n] = j_aug[j][n] + m * j_aug[k][n];
      }
    }
  }

  /* Assign last 6 columns of j_aug to inv_jac */
  for (j = 0; j <= 5; ++j) {
    for (k = 0; k <= 5; ++k) {
      inv_jac[j][k] = j_aug[j][k + 6];

    }
  }

  return GO_RESULT_OK;	      /* FIXME-- check divisors for 0 above */
}

/*
  This function simply multiplies a 6x6 matrix by a 1x6 vector
*/

static void mat_mult(go_real jac[][6], const go_real x[], go_real ans[])
{
  go_integer j, k;

  for (j = 0; j <= 5; ++j) {
    ans[j] = 0;
    for (k = 0; k <= 5; ++k) {
      ans[j] = jac[j][k] * x[k] + ans[j];
    }
  }
}

go_integer genhex_kin_size(void)
{
  return (go_integer) sizeof(genhex_struct);
}

go_result genhex_kin_init(void * kins)
{
  /* these are for the unit hexapod */
  go_real sqrt_3_2 = sqrt(3.0) * 0.5;
  go_link params[6];

  params[0].type = GO_LINK_PK;
  params[0].quantity = GO_QUANTITY_LENGTH;
  go_body_init(&params[0].body);
  params[0].u.pk.base.x = 0;
  params[0].u.pk.base.y = 1;
  params[0].u.pk.base.z = 0;
  params[0].u.pk.platform.x = sqrt_3_2;
  params[0].u.pk.platform.y = 0.5;
  params[0].u.pk.platform.z = 0;

  params[1].type = GO_LINK_PK;
  params[1].quantity = GO_QUANTITY_LENGTH;
  go_body_init(&params[1].body);
  params[1].u.pk.base.x = sqrt_3_2;
  params[1].u.pk.base.y = -0.5;
  params[1].u.pk.base.z = 0;
  params[1].u.pk.platform.x = sqrt_3_2;
  params[1].u.pk.platform.y = 0.5;
  params[1].u.pk.platform.z = 0;

  params[2].type = GO_LINK_PK;
  params[2].quantity = GO_QUANTITY_LENGTH;
  go_body_init(&params[2].body);
  params[2].u.pk.base.x = sqrt_3_2;
  params[2].u.pk.base.y = -0.5;
  params[2].u.pk.base.z = 0;
  params[2].u.pk.platform.x = 0;
  params[2].u.pk.platform.y = -1;
  params[2].u.pk.platform.z = 0;

  params[3].type = GO_LINK_PK;
  params[3].quantity = GO_QUANTITY_LENGTH;
  go_body_init(&params[3].body);
  params[3].u.pk.base.x = -sqrt_3_2;
  params[3].u.pk.base.y = -0.5;
  params[3].u.pk.base.z = 0;
  params[3].u.pk.platform.x = 0;
  params[3].u.pk.platform.y = -1;
  params[3].u.pk.platform.z = 0;

  params[4].type = GO_LINK_PK;
  params[4].quantity = GO_QUANTITY_LENGTH;
  go_body_init(&params[4].body);
  params[4].u.pk.base.x = -sqrt_3_2;
  params[4].u.pk.base.y = -0.5;
  params[4].u.pk.base.z = 0;
  params[4].u.pk.platform.x = -sqrt_3_2;
  params[4].u.pk.platform.y = 0.5;
  params[4].u.pk.platform.z = 0;

  params[5].type = GO_LINK_PK;
  params[5].quantity = GO_QUANTITY_LENGTH;
  go_body_init(&params[5].body);
  params[5].u.pk.base.x = 0;
  params[5].u.pk.base.y = 1;
  params[5].u.pk.base.z = 0;
  params[5].u.pk.platform.x = -sqrt_3_2;
  params[5].u.pk.platform.y = 0.5;
  params[5].u.pk.platform.z = 0;

  return genhex_kin_set_parameters(kins, params, 6);
}

const char * genhex_kin_get_name(void)
{
  return "genhexkins";
}

go_integer genhex_kin_num_joints(void * kins)
{
  return GENHEX_NUM_JOINTS;
}

go_result genhex_kin_set_parameters(void * kins, go_link * params, go_integer num)
{
  genhex_struct * genhex = (genhex_struct *) kins;
  go_integer t;

  if (num > GENHEX_NUM_JOINTS) return GO_RESULT_ERROR;

  for (t = 0; t < GENHEX_NUM_JOINTS; t++) {
    if (params[t].type != GO_LINK_PK ||
	params[t].quantity != GO_QUANTITY_LENGTH) return GO_RESULT_ERROR;
    genhex->base[t].x = params[t].u.pk.base.x;
    genhex->base[t].y = params[t].u.pk.base.y;
    genhex->base[t].z = params[t].u.pk.base.z;
    genhex->platform[t].x = params[t].u.pk.platform.x;
    genhex->platform[t].y = params[t].u.pk.platform.y;
    genhex->platform[t].z = params[t].u.pk.platform.z; 
  }

  return GO_RESULT_OK;
}

go_result genhex_kin_get_parameters(void * kins, go_link * params, go_integer num)
{
  genhex_struct * genhex = (genhex_struct *) kins;
  go_integer t;

  if (num < GENHEX_NUM_JOINTS) return GO_RESULT_ERROR;

  for (t = 0; t < GENHEX_NUM_JOINTS; t++) {
    params[t].type = GO_LINK_PK;
    params[t].quantity = GO_QUANTITY_LENGTH;
    params[t].u.pk.base.x = genhex->base[t].x;
    params[t].u.pk.base.y = genhex->base[t].y;
    params[t].u.pk.base.z = genhex->base[t].z;
    params[t].u.pk.platform.x = genhex->platform[t].x;
    params[t].u.pk.platform.y = genhex->platform[t].y;
    params[t].u.pk.platform.z = genhex->platform[t].z;
  }

  return GO_RESULT_OK;
}

static go_result j_inv_mat(genhex_struct * genhex, 
			   const go_pose * pos,
			   go_real inv_jac[][GENHEX_NUM_JOINTS])
{
  go_integer i;
  go_cart aw, r_matrix_a;
  go_cart inv_kin_strut_vec, inv_kin_strut_vec_unit;
  go_cart r_matrix_a_cross_strut;

  /* Enter for loop to build Inverse Jacobian */
  for (i = 0; i < GENHEX_NUM_JOINTS; i++) {
    /* run part of inverse kins to get strut vectors */
    go_quat_cart_mult(&pos->rot, &genhex->platform[i], &r_matrix_a);
    go_cart_cart_add(&pos->tran, &r_matrix_a, &aw);
    go_cart_cart_sub(&aw, &genhex->base[i], &inv_kin_strut_vec);

    /* Determine r_matrix_a_cross_strut */
    if (0 != go_cart_unit(&inv_kin_strut_vec, &inv_kin_strut_vec_unit)) {
      return GO_RESULT_ERROR;
    }
    go_cart_cart_cross(&r_matrix_a, &inv_kin_strut_vec_unit, &r_matrix_a_cross_strut);

    /* Build Inverse Jacobian Matrix */
    inv_jac[i][0] = inv_kin_strut_vec_unit.x;
    inv_jac[i][1] = inv_kin_strut_vec_unit.y;
    inv_jac[i][2] = inv_kin_strut_vec_unit.z;
    inv_jac[i][3] = r_matrix_a_cross_strut.x;
    inv_jac[i][4] = r_matrix_a_cross_strut.y;
    inv_jac[i][5] = r_matrix_a_cross_strut.z;
  }

  return GO_RESULT_OK;
}

go_result genhex_kin_jac_inv(void * kins,
			     const go_pose * pos,
			     const go_vel * vel,
			     const go_real *joints,
			     go_real *jointvels)
{
  genhex_struct * genhex = (genhex_struct *) kins;
  go_real inv_jac[GENHEX_NUM_JOINTS][GENHEX_NUM_JOINTS];
  go_real velmatrix[6];
  go_result retval;

  retval = j_inv_mat(genhex, pos, inv_jac);
  if (GO_RESULT_OK != retval) return retval;

  /* Multiply Jinv[] by vel[] to get jointvels */
  velmatrix[0] = vel->v.x;	/* dx/dt */
  velmatrix[1] = vel->v.y;	/* dy/dt */
  velmatrix[2] = vel->v.z;	/* dz/dt */
  velmatrix[3] = vel->w.x;	/* droll/dt */
  velmatrix[4] = vel->w.y;	/* dpitch/dt */
  velmatrix[5] = vel->w.z;	/* dyaw/dt */
  mat_mult(inv_jac, velmatrix, jointvels);

  return GO_RESULT_OK;
}

/* FIXME-- could use a better implementation than computing the
   inverse and then inverting it */
go_result genhex_kin_jac_fwd(void * kins,
			     const go_real *joints,
			     const go_real *jointvels,
			     const go_pose * pos,
			     go_vel * vel)
{
  genhex_struct * genhex = (genhex_struct *) kins;
  go_real inv_jac[GENHEX_NUM_JOINTS][GENHEX_NUM_JOINTS];
  go_real jac[GENHEX_NUM_JOINTS][GENHEX_NUM_JOINTS];
  go_real velmatrix[6];

  if (0 != j_inv_mat(genhex, pos, inv_jac)) {
    return GO_RESULT_ERROR;
  }
  if (0 != mat_invert(inv_jac, jac)) {
    return GO_RESULT_ERROR;
  }

  /* Multiply J[] by jointvels to get vels */
  mat_mult(jac, jointvels, velmatrix);
  vel->v.x = velmatrix[0];
  vel->v.y = velmatrix[1];
  vel->v.z = velmatrix[2];
  vel->w.x = velmatrix[3];
  vel->w.y = velmatrix[4];
  vel->w.z = velmatrix[5];

  return GO_RESULT_OK;
}

go_result genhex_kin_fwd(void * kins,
			 const go_real *joints,
			 go_pose * pos)
{
  genhex_struct * genhex = (genhex_struct *) kins;
  go_cart aw;
  go_cart inv_kin_strut_vec, inv_kin_strut_vec_unit;
  go_cart q_tran, r_matrix_a, r_matrix_a_cross_strut;

  go_real jac[GENHEX_NUM_JOINTS][GENHEX_NUM_JOINTS];
  go_real inv_jac[GENHEX_NUM_JOINTS][GENHEX_NUM_JOINTS];
  go_real inv_kin_strut_len, strut_len_diff[GENHEX_NUM_JOINTS];
  go_real delta[GENHEX_NUM_JOINTS];
  go_real conv_err = 1.0;

  go_mat r_matrix;
  go_rpy q_rpy;

  go_integer iterate = 1;
  go_integer i;
  go_result retval = GO_RESULT_OK;

#define HIGH_CONV_CRITERION   (1e-12)
#define MEDIUM_CONV_CRITERION (1e-5)
#define LOW_CONV_CRITERION    (1e-3)
#define MEDIUM_CONV_ITERATIONS  50
#define LOW_CONV_ITERATIONS    100
#define FAIL_CONV_ITERATIONS   150
#define LARGE_CONV_ERROR 10000
  go_real conv_criterion = HIGH_CONV_CRITERION;

  genhex->iteration = 0;

  /* abort on obvious problems, like joints <= 0 */
  /* FIXME-- should check against triangle inequality, so that joints
     are never too short to span shared base and platform sides */
  if (joints[0] <= 0.0 ||
      joints[1] <= 0.0 ||
      joints[2] <= 0.0 ||
      joints[3] <= 0.0 || joints[4] <= 0.0 || joints[5] <= 0.0) {
    return GO_RESULT_ERROR;
  }

  /* assign a,b,c to roll, pitch, yaw angles */
  go_quat_rpy_convert(&pos->rot, &q_rpy);

  /* Assign translation values in pos to q_tran */
  q_tran = pos->tran;

  /* Enter Newton-Raphson iterative method   */
  while (iterate) {
    /* check for large error and return error flag if no convergence */
    if ((conv_err > +LARGE_CONV_ERROR) || (conv_err < -LARGE_CONV_ERROR)) {
      /* we can't converge */
      return GO_RESULT_ERROR;
    };

    genhex->iteration++;

#if 0
    /* if forward kinematics are having a difficult time converging
       ease the restrictions on the convergence criterion */
    if (genhex->iteration == MEDIUM_CONV_ITERATIONS) {
      conv_criterion = MEDIUM_CONV_CRITERION;
      retval = GO_RESULT_OK;
    }

    if (genhex->iteration == LOW_CONV_ITERATIONS) {
      conv_criterion = LOW_CONV_CRITERION;
      retval = GO_RESULT_OK;
    }
#endif

    /* check iteration to see if the kinematics can reach the
       convergence criterion and return error flag if it can't */
    if (genhex->iteration > FAIL_CONV_ITERATIONS) {
      /* we can't converge */
      return GO_RESULT_ERROR;
    }

    /* Convert q_rpy to Rotation Matrix */
    go_rpy_mat_convert(&q_rpy, &r_matrix);

    /* compute strut_len_diff[] by running inverse kins on Cartesian
       estimate to get joint estimate, subtract joints to get joint deltas,
       and compute inv J while we're at it */
    for (i = 0; i < GENHEX_NUM_JOINTS; i++) {
      go_mat_cart_mult(&r_matrix, &genhex->platform[i], &r_matrix_a);
      go_cart_cart_add(&q_tran, &r_matrix_a, &aw);
      go_cart_cart_sub(&aw, &genhex->base[i], &inv_kin_strut_vec);
      if (0 != go_cart_unit(&inv_kin_strut_vec, &inv_kin_strut_vec_unit)) {
	return GO_RESULT_ERROR;
      }
      go_cart_mag(&inv_kin_strut_vec, &inv_kin_strut_len);
      strut_len_diff[i] = inv_kin_strut_len - joints[i];

      /* Determine r_matrix_a_cross_strut */
      go_cart_cart_cross(&r_matrix_a, &inv_kin_strut_vec_unit, &r_matrix_a_cross_strut);

      /* Build Inverse Jacobian Matrix */
      inv_jac[i][0] = inv_kin_strut_vec_unit.x;
      inv_jac[i][1] = inv_kin_strut_vec_unit.y;
      inv_jac[i][2] = inv_kin_strut_vec_unit.z;
      inv_jac[i][3] = r_matrix_a_cross_strut.x;
      inv_jac[i][4] = r_matrix_a_cross_strut.y;
      inv_jac[i][5] = r_matrix_a_cross_strut.z;
    }

    /* invert Inverse Jacobian */
    mat_invert(inv_jac, jac);

    /* multiply Jacobian by LegLengthDiff */
    mat_mult(jac, strut_len_diff, delta);

    /* subtract delta from last iterations pos values */
    q_tran.x -= delta[0];
    q_tran.y -= delta[1];
    q_tran.z -= delta[2];
    q_rpy.r -= delta[3];
    q_rpy.p -= delta[4];
    q_rpy.y -= delta[5];

    /* determine value of conv_error (used to determine if no convergence) */
    conv_err = 0.0;
    for (i = 0; i < GENHEX_NUM_JOINTS; i++) {
      conv_err += fabs(strut_len_diff[i]);
    }

    /* enter loop to determine if a strut needs another iteration */
    iterate = 0;		/*assume iteration is done */
    for (i = 0; i < GENHEX_NUM_JOINTS; i++) {
      if (fabs(strut_len_diff[i]) > conv_criterion) {
	iterate = 1;
      }
    }
  }				/* exit Newton-Raphson Iterative loop */

  /* assign r,p,w to a,b,c */
  go_rpy_quat_convert(&q_rpy, &pos->rot);

  /* assign q_tran to pos */
  pos->tran = q_tran;

  return retval;
}

go_integer genhex_kin_fwd_iterations(genhex_struct * genhex)
{
  return genhex->iteration;
}

/*
  The inverse kinematics take world coordinates and determine joint values,
  given the inverse kinematics flags to resolve any ambiguities. The forward
  flags are set to indicate their value appropriate to the world coordinates
  passed in.
*/

go_result genhex_kin_inv(void  * kins,
			 const go_pose * pos,
			 go_real *joints)
{
  genhex_struct * genhex = (genhex_struct *) kins;
  go_cart aw;
  go_integer i;

  for (i = 0; i < GENHEX_NUM_JOINTS; i++) {
    /* convert location of platform strut end from platform
       to world coordinates */
    go_pose_cart_mult(pos, &genhex->platform[i], &aw);

    /* define strut lengths */
    go_cart_cart_sub(&aw, &genhex->base[i], &aw);
    go_cart_mag(&aw, &joints[i]);
  }

  return GO_RESULT_OK;
}

go_kin_type genhex_kin_get_type(void * kins)
{
  return GO_KIN_BOTH;
}
