/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#include <stddef.h>		/* sizeof */
#include <math.h>		/* sin, cos */

#include "gotypes.h"		/* go_result, go_integer */
#include "gomath.h"		/* go_pose */
#include "gokin.h"		/* go_kin_fwd_flags */
#include "gokin.h"		/* go_kin_type */
#include "spheristkins.h"		/* these decls */

go_integer spherist_kin_size(void)
{
  return (go_integer) sizeof(spherist_kin_struct);
} 

/* default values are for PUMA 560, in [m] */
#define PUMA560_A2 0.300
#define PUMA560_A3 0.050
#define PUMA560_D3 0.070
#define PUMA560_D4 0.400

#define SINGULAR_FUZZ (1.0e-6)
#define FLAG_FUZZ     (1.0e-6)

go_result spherist_kin_init(void *kins)
{
  spherist_kin_struct *sphk = (spherist_kin_struct *) kins;

  sphk->a1 = 0;
  sphk->sa1 = -1;			/* alpha1 = -90, sin(-90) = -1 */
  sphk->ca1 = 0;			/* cos(-90) = 0 */

  sphk->a2 = PUMA560_A2;
  sphk->sa2 = 0;			/* alpha2 = 0 */
  sphk->ca2 = 1;
  sphk->d2 = 0;

  sphk->a3 = PUMA560_A3;
  sphk->sa3 = -1;			/* alpha3 = -90 */
  sphk->ca3 = 0;
  sphk->d3 = PUMA560_D3;

  sphk->d4 = PUMA560_D4;

  sphk->sa4 = 1;			/* alpha4 = 90 */
  sphk->ca4 = 0;

  sphk->sa5 = -1;			/* alpha5 = -90 */
  sphk->ca5 = 0;

  sphk->iflags = 0;

  return GO_RESULT_OK;
} 

const char * spherist_kin_get_name(void)
{
  return "spheristkins";
} 

go_integer spherist_kin_num_joints(void *kins)
{
  return SPHERIST_NUM_JOINTS;
}

go_result spherist_kin_fwd(void *kins,
			   const go_real *joint,
			   go_pose *world)
{
  spherist_kin_struct *sphk = (spherist_kin_struct *) kins;
  go_hom h;
  go_hom hcum;
  go_real sth, cth;

  sphk->iflags = 0;

  /* recall: */
  /*            |  m.x.x   m.y.x   m.z.x  | */
  /* go_mat m = |  m.x.y   m.y.y   m.z.y  | */
  /*            |  m.x.z   m.y.z   m.z.z  | */

  go_sincos(joint[0], &sth, &cth);
  hcum.rot.x.x = cth, hcum.rot.y.x = -sth, hcum.rot.z.x = 0;
  hcum.rot.x.y = sth, hcum.rot.y.y = cth, hcum.rot.z.y = 0;
  hcum.rot.x.z = 0, hcum.rot.y.z = 0, hcum.rot.z.z = 1;
  hcum.tran.x = 0, hcum.tran.y = 0, hcum.tran.z = 0;

  go_sincos(joint[1], &sth, &cth);
  h.rot.x.x = cth, h.rot.y.x = -sth, h.rot.z.x = 0;
  h.rot.x.y = sth*sphk->ca1, h.rot.y.y = cth*sphk->ca1, h.rot.z.y = -sphk->sa1;
  h.rot.x.z = sth*sphk->sa1, h.rot.y.z = cth*sphk->sa1, h.rot.z.z = sphk->ca1;
  h.tran.x = sphk->a1, h.tran.y = -sphk->sa1*sphk->d2, h.tran.z = sphk->ca1*sphk->d2;
  go_hom_hom_mult(&hcum, &h, &hcum);

  go_sincos(joint[2], &sth, &cth);
  h.rot.x.x = cth, h.rot.y.x = -sth, h.rot.z.x = 0;
  h.rot.x.y = sth*sphk->ca2, h.rot.y.y = cth*sphk->ca2, h.rot.z.y = -sphk->sa2;
  h.rot.x.z = sth*sphk->sa2, h.rot.y.z = cth*sphk->sa2, h.rot.z.z = sphk->ca2;
  h.tran.x = sphk->a2, h.tran.y = -sphk->sa2*sphk->d3, h.tran.z = sphk->ca2*sphk->d3;
  go_hom_hom_mult(&hcum, &h, &hcum);

  go_sincos(joint[3], &sth, &cth);
  h.rot.x.x = cth, h.rot.y.x = -sth, h.rot.z.x = 0;
  h.rot.x.y = sth*sphk->ca3, h.rot.y.y = cth*sphk->ca3, h.rot.z.y = -sphk->sa3;
  h.rot.x.z = sth*sphk->sa3, h.rot.y.z = cth*sphk->sa3, h.rot.z.z = sphk->ca3;
  h.tran.x = sphk->a3, h.tran.y = -sphk->sa3*sphk->d4, h.tran.z = sphk->ca3*sphk->d4;
  go_hom_hom_mult(&hcum, &h, &hcum);

  go_sincos(joint[4], &sth, &cth);
  h.rot.x.x = cth, h.rot.y.x = -sth, h.rot.z.x = 0;
  h.rot.x.y = 0, h.rot.y.y = 0, h.rot.z.y = -1;
  h.rot.x.z = sth, h.rot.y.z = cth, h.rot.z.z = 0;
  h.tran.x = 0, h.tran.y = 0, h.tran.z = 0;
  go_hom_hom_mult(&hcum, &h, &hcum);

  go_sincos(joint[5], &sth, &cth);
  h.rot.x.x = cth, h.rot.y.x = -sth, h.rot.z.x = 0;
  h.rot.x.y = 0, h.rot.y.y = 0, h.rot.z.y = 1;
  h.rot.x.z = -sth, h.rot.y.z = -cth, h.rot.z.z = 0;
  h.tran.x = 0, h.tran.y = 0, h.tran.z = 0;
  go_hom_hom_mult(&hcum, &h, &hcum);

  if (joint[2] < 0) {
    sphk->iflags |= SPHERIST_ELBOW_DOWN;
  } else {
    sphk->iflags &= ~SPHERIST_ELBOW_DOWN;
  }

  return go_hom_pose_convert(&hcum, world);
} 

go_result spherist_kin_inv(void *kins,
			   const go_pose * world,
			   go_real * joint)
{
  spherist_kin_struct *sphk = (spherist_kin_struct *) kins;

  go_real Y, A, B, C;
  go_real a, b, c;
  go_real discr;
  go_real th3a, th3b;

  if (GO_ROT_SMALL(sphk->sa1)) {
    /*
      From J. Craig p. 112-114, 

      z = k4

      k4 = f3 ca1 + d2 ca1

      f3 = a3 sa2 (s3) - d4 sa3 sa2 (c3) + d4 ca2 ca3 + d3 ca2

      Subsitituing and factoring out (s3) and (c3), we get

      z = (a3 sa2 ca1) (s3) - (d4 sa3 sa2 ca1) (c3) +
      d4 ca2 ca3 ca1 + d3 ca2 ca1 +
      d2 ca1

      Using the identities 

      sin(th) = 2 tan(th/2) / (1 + tan^2(th/2))
      cos(th) = (1 - tan^2(th/2)) / (1 + tan^2(th/2))

      and our form Y = A sin(th3) + B cos(th3) + C, we get
      Y = A*(2*t)/(1+t*t) + B*(1-t*t)/(1+t*t) + C, where
      the solution is RootOf((Y + B - C) _Z^2  - 2 A _Z + Y - C - B),
      where the quadratic is now of the form

      a t^2 + b t + c, with
    
      a = Y + B - C
      b = -2 A
      c = Y - B - C

      so we need to take the coefficients of (s3) as A, (c3) as B,
      z as Y and the rest as C.

      A = a3 sa2 ca1
      B = -d4 sa3 sa2 ca1
      C = d4 ca2 ca3 ca1 + d3 ca2 ca1 + d2 ca1
      Y = z
    */
    Y = world->tran.z;
    A = sphk->a3 * sphk->sa2 * sphk->ca1;
    B = -sphk->d4 * sphk->sa3 * sphk->sa2 * sphk->ca1;
    C = sphk->ca1 * (sphk->ca2 * (sphk->d4 * sphk->ca3 + sphk->d3) + sphk->d2);
  } else if (GO_TRAN_SMALL(sphk->a1)) {
    /*
      Likewise, 

      r^2 = k3
    
      k3 = f1^2 + f2^2 + f3^2 + a1^2 + d2^2 + 2 d2 f3

      f1^2 + f2^2 + f3^2 = a3^2 + d4^2 + d3^2 + a2^2 + 2 d4 d3 ca3 + 
      2 a2 a3 (c3) + 2 a2 d4 sa3 (s3)

      f3 = a3 sa2 (s3) - d4 sa3 sa2 (c3) + d4 ca2 ca3 + d3 ca2
    
      Combining,

      r^2 = a3^2 + d4^2 + d3^2 + a2^2 + 2 d4 d3 ca3 + 
      2 a2 a3 (c3) + 2 a2 d4 sa3 (s3) + a1^2 + d2^2 + 
      2 d2 (a3 sa2 (s3) - d4 sa3 sa2 (c3) + d4 ca2 ca3 + d3 ca2)

      r^2 = a3^2 + d4^2 + d3^2 + a2^2 + 2 d4 d3 ca3 + 
      a1^2 + d2^2 + 2 d2 (d4 ca2 ca3 + d3 ca2) +
      (2 a2 d4 sa3 + 2 d2 a3 sa2) (s3) +
      (2 a2 a3 - 2 d2 d4 sa3 sa2) (c3)

      Using the identities 

      sin(th) = 2 tan(th/2) / (1 + tan^2(th/2))
      cos(th) = (1 - tan^2(th/2)) / (1 + tan^2(th/2))

      and our form Y = A sin(th3) + B cos(th3) + C, we get
      Y = A*(2*t)/(1+t*t) + B*(1-t*t)/(1+t*t) + C, where
      the solution is RootOf((Y + B - C) _Z^2  - 2 A _Z + Y - C - B),
      where the quadratic is now of the form

      a t^2 + b t + c, with
    
      a = Y + B - C
      b = -2 A
      c = Y - B - C

      so we need to take the coefficients of (s3) as A, (c3) as B,
      rsq as Y and the rest as C.

      A = 2 a2 d4 sa3 + 2 d2 a3 sa2
      B = 2 a2 a3 - 2 d2 d4 sa3 sa2
      C = a3^2 + d4^2 + d3^2 + a2^2 + 2 d4 d3 ca3 + 
      a1^2 + d2^2 + 2 d2 (d4 ca2 ca3 + d3 ca2)
    */

    Y = world->tran.x * world->tran.x +
      world->tran.y * world->tran.y +
      world->tran.z * world->tran.z;

    A = 2 * (sphk->a2 * sphk->d4 * sphk->sa3 +
	     sphk->d2 * sphk->a3 * sphk->sa2);

    B = 2 * (sphk->a2 * sphk->a3 - 
	     sphk->d2 * sphk->d4 * sphk->sa3 * sphk->sa2);

    C = sphk->a1 * sphk->a1 + sphk->a2 * sphk->a2 + sphk->a3 * sphk->a3 +
      sphk->d2 * sphk->d2 + sphk->d3 * sphk->d3 + sphk->d4 * sphk->d4 +
      2 * (sphk->d4 * sphk->d3 * sphk->ca3 + 
	   sphk->d2 * (sphk->d4 * sphk->ca2 * sphk->ca3 + 
		       sphk->d3 * sphk->ca2));
  } else {
    /* FIXME -- need to implement the solution of the fourth order polynomial */
    return GO_RESULT_IMPL_ERROR;
  }

  a = Y + B - C;
  b = -2 * A;
  c = Y - B - C;

  a = 2*a;			/* double it for later */
  if (GO_SMALL(a)) return GO_RESULT_ERROR;

  discr = b*b - 2*a*c;		/* 'a' was doubled */

  if (discr < -GO_REAL_EPSILON) return GO_RESULT_ERROR;
  else if (discr < 0) discr = 0;
  discr = sqrt(discr);
  a = 1/a;

  th3a = (-b + discr) * a;
  th3b = (-b - discr) * a;

  /* these are tan(th3/2), so to get th3 we need 2 arctan() */
  th3a = 2 * atan(th3a);
  th3b = 2 * atan(th3b);

  /* FIXME-- this doesn't always give the correct choice-- why not? */
  if (sphk->iflags & SPHERIST_ELBOW_DOWN) {
    joint[2] = th3b < 0 ? th3b : th3a;
  } else {
    joint[2] = th3b > 0 ? th3b : th3a;
  }

  /* FIXME-- we need to get the other joints */
  return GO_RESULT_IMPL_ERROR;
}

go_kin_type spherist_kin_get_type(void *kins)
{
  return GO_KIN_BOTH;
} 

go_result spherist_kin_set_parameters(void *kins, go_link *params, go_integer num)
{
  spherist_kin_struct *sphk = (spherist_kin_struct *) kins;
  go_dh dh;

  if (num > SPHERIST_NUM_JOINTS) return GO_RESULT_ERROR;

  if (params[1].quantity != GO_QUANTITY_ANGLE) return GO_RESULT_ERROR;
  if (params[1].type == GO_LINK_DH) {
    sphk->a1 = params[1].u.dh.a;
    go_sincos(params[1].u.dh.alpha, &sphk->sa1, &sphk->ca1);
    sphk->d2 = params[1].u.dh.d;
  } else if (params[1].type == GO_LINK_PP) {
    go_pose_dh_convert(&params[1].u.pp.pose, &dh);
    sphk->a1 = dh.a;
    go_sincos(dh.alpha, &sphk->sa1, &sphk->ca1);
    sphk->d2 = dh.d;
  } else return GO_RESULT_ERROR;

  if (params[2].quantity != GO_QUANTITY_ANGLE) return GO_RESULT_ERROR;
  if (params[2].type == GO_LINK_DH) {
    sphk->a2 = params[2].u.dh.a;
    go_sincos(params[2].u.dh.alpha, &sphk->sa2, &sphk->ca2);
    sphk->d3 = params[2].u.dh.d;
  } else if (params[2].type == GO_LINK_PP) {
    go_pose_dh_convert(&params[2].u.pp.pose, &dh);
    sphk->a2 = dh.a;
    go_sincos(dh.alpha, &sphk->sa2, &sphk->ca2);
    sphk->d3 = dh.d;
  } else return GO_RESULT_ERROR;

  if (params[3].quantity != GO_QUANTITY_ANGLE) return GO_RESULT_ERROR;
  if (params[3].type == GO_LINK_DH) {
    sphk->a3 = params[3].u.dh.a;
    go_sincos(params[3].u.dh.alpha, &sphk->sa3, &sphk->ca3);
    sphk->d4 = params[3].u.dh.d;
  } else if (params[3].type == GO_LINK_PP) {
    go_pose_dh_convert(&params[3].u.pp.pose, &dh);
    sphk->a3 = dh.a;
    go_sincos(dh.alpha, &sphk->sa3, &sphk->ca3);
    sphk->d4 = dh.d;
  } else return GO_RESULT_ERROR;

  return GO_RESULT_OK;
} 

go_result spherist_kin_get_parameters(void *kins, go_link *params, go_integer num)
{
  spherist_kin_struct *sphk = (spherist_kin_struct *) kins;

  if (num < SPHERIST_NUM_JOINTS) return GO_RESULT_ERROR;

  params[0].type = GO_LINK_DH;
  params[0].quantity = GO_QUANTITY_ANGLE;
  params[0].u.dh.a = 0;
  params[0].u.dh.alpha = 0;
  params[0].u.dh.d = 0;

  params[1].type = GO_LINK_DH;
  params[1].quantity = GO_QUANTITY_ANGLE;
  params[1].u.dh.a = sphk->a1;
  params[1].u.dh.alpha = atan2(sphk->sa1, sphk->ca1);
  params[1].u.dh.d = sphk->d2;

  params[2].type = GO_LINK_DH;
  params[2].quantity = GO_QUANTITY_ANGLE;
  params[2].u.dh.a = sphk->a2;
  params[2].u.dh.alpha = atan2(sphk->sa2, sphk->ca2);
  params[2].u.dh.d = sphk->d3;

  params[3].type = GO_LINK_DH;
  params[3].quantity = GO_QUANTITY_ANGLE;
  params[3].u.dh.a = sphk->a3;
  params[3].u.dh.alpha = atan2(sphk->sa2, sphk->ca2);
  params[3].u.dh.d = sphk->d4;

  params[4].type = GO_LINK_DH;
  params[4].quantity = GO_QUANTITY_ANGLE;
  params[4].u.dh.a = 0;
  params[4].u.dh.alpha = GO_PI_2;
  params[4].u.dh.d = 0;

  params[5].type = GO_LINK_DH;
  params[5].quantity = GO_QUANTITY_ANGLE;
  params[5].u.dh.a = 0;
  params[5].u.dh.alpha = -GO_PI_2;
  params[5].u.dh.d = 0;
 
  return GO_RESULT_OK;
} 

/*
  From doc/spherist.mpl Maple program, with e.g.
  collect(v6_0[1],{thd1,thd2,thd3});


  V_6_0[1] = 
  (
  + sin(th1) d4 sin(th3) cos(th2) 
  + sin(th1) d4 cos(th3) sin(th2)
  + sin(th1) a3 sin(th3) sin(th2)
  - sin(th1) a3 cos(th3) cos(th2)
  - sin(th1) cos(th2) a2
  - cos(th1) d3
  ) dth1 +

  (
  - cos(th1) cos(th2) sin(th3) a3
  - cos(th1) sin(th2) cos(th3) a3
  + cos(th1) sin(th2) sin(th3) d4
  - cos(th1) cos(th2) cos(th3) d4
  - cos(th1) sin(th2) a2
  ) dth2 + 

  (
  - cos(th1) cos(th2) sin(th3) a3
  - cos(th1) sin(th2) cos(th3) a3 
  - cos(th1) cos(th2) cos(th3) d4
  + cos(th1) sin(th2) sin(th3) d4
  ) dth3


  V_6_0[2] =

  (
  - cos(th1) d4 cos(th3) sin(th2)
  - cos(th1) d4 sin(th3) cos(th2)
  - cos(th1) a3 sin(th3) sin(th2)
  + cos(th1) a3 cos(th3) cos(th2)
  + cos(th1) cos(th2) a2
  - sin(th1) d3 
  ) dth1 + 

  (
  - sin(th1) cos(th2) sin(th3) a3
  - sin(th1) sin(th2) cos(th3) a3
  + sin(th1) sin(th2) sin(th3) d4
  - sin(th1) cos(th2) cos(th3) d4
  - sin(th1) sin(th2) a2
  ) dth2 +

  (
  - sin(th1) cos(th2) cos(th3) d4
  + sin(th1) sin(th2) sin(th3) d4
  - sin(th1) cos(th2) sin(th3) a3
  - sin(th1) sin(th2) cos(th3) a3
  ) dth3


  V_6_0[3] =

  (
  + cos(th2) sin(th3) d4
  + sin(th2) cos(th3) d4
  + sin(th2) sin(th3) a3
  - cos(th2) cos(th3) a3
  - cos(th2) a2 
  ) dth2 +

  (
  - cos(th2) cos(th3) a3
  + sin(th2) sin(th3) a3
  + cos(th2) sin(th3) d4
  + sin(th2) cos(th3) d4
  ) dth3


  W_6_0[1] =

  - sin(th1) dth2
  - sin(th1) dth3 +

  (
  - cos(th1) sin(th2) cos(th3)
  - cos(th1) cos(th2) sin(th3)
  ) dth4 +

  (
  + cos(th1) cos(th2) cos(th3) sin(th4)
  - cos(th1) sin(th2) sin(th3) sin(th4)
  - sin(th1) cos(th4)
  ) dth5 +

  (
  + cos(th1) sin(th2) sin(th3) cos(th4) sin(th5)
  - cos(th1) cos(th2) cos(th3) cos(th4) sin(th5)
  - cos(th1) cos(th2) sin(th3) cos(th5)
  - cos(th1) sin(th2) cos(th3) cos(th5)
  - sin(th1) sin(th4) sin(th5)
  ) dth6


  W_6_0[2] =

  + cos(th1) dth2

  + cos(th1) dth3 +

  (
  - sin(th1) sin(th2) cos(th3) 
  - sin(th1) cos(th2) sin(th3)
  ) dth4 +

  (
  + sin(th1) cos(th2) cos(th3) sin(th4)
  - sin(th1) sin(th2) sin(th3) sin(th4)
  + cos(th1) cos(th4)
  ) dth5 +

  (
  + sin(th1) sin(th2) sin(th3) cos(th4) sin(th5)
  - sin(th1) cos(th2) cos(th3) cos(th4) sin(th5)
  - sin(th1) cos(th2) sin(th3) cos(th5)
  - sin(th1) sin(th2) cos(th3) cos(th5)
  + cos(th1) sin(th4) sin(th5)
  ) dth6


  W_6_0[3] =

  dth1 +

  (
  + sin(th2) sin(th3)
  - cos(th2) cos(th3)
  ) dth4 +

  (
  - cos(th2) sin(th3) sin(th4)
  - sin(th2) cos(th3) sin(th4)
  ) dth5 +

  (
  + sin(th2) cos(th3) cos(th4) sin(th5)
  + cos(th2) sin(th3) cos(th4) sin(th5) 
  + sin(th2) sin(th3) cos(th5)
  - cos(th2) cos(th3) cos(th5)
  ) dth6

*/

static go_result jac_fwd_mat(const spherist_kin_struct *kins,
			     const go_real *joints,
			     go_real mat[6][6])
{
  go_real s1, s2, s3, s4, s5, s6;
  go_real c1, c2, c3, c4, c5, c6;
  go_real s23, c23;		/* sin,cos(th2 + th3) */
  go_real a2, a3, d3, d4;

  s1 = sin(joints[0]), c1 = cos(joints[0]);
  s2 = sin(joints[1]), c2 = cos(joints[1]);
  s3 = sin(joints[2]), c3 = cos(joints[2]);
  s4 = sin(joints[3]), c4 = cos(joints[3]);
  s5 = sin(joints[4]), c5 = cos(joints[4]);
  s6 = sin(joints[5]), c6 = cos(joints[5]);
  s23 = s2*c3 + c2*s3;
  c23 = c2*c3 - s2*s3;
  /* shorthand */
  a2 = kins->a2, a3 = kins->a3, d3 = kins->d3, d4 = kins->d4;

  /* sin(x+y) = sx cy + cx sy */
  /* sin(x-y) = sx cy - cx sy */
  /* cos(x+y) = cx cy - sx sy */
  /* cos(x-y) = cx cy + sx sy */

  /* vx */

  mat[0][0] =			/* dth1 */
    /*
      + sin(th1) d4 sin(th3) cos(th2) 
      + sin(th1) d4 cos(th3) sin(th2)
    */
    + s1*d4*s23
    /*
      + sin(th1) a3 sin(th3) sin(th2)
      - sin(th1) a3 cos(th3) cos(th2)
    */
    - s1*a3*c23
    /*
      - sin(th1) cos(th2) a2
      - cos(th1) d3
    */
    - s1*c2*a2
    - c1*d3;

  mat[0][1] = 			/* dth2 */
    /*
      - cos(th1) cos(th2) sin(th3) a3
      - cos(th1) sin(th2) cos(th3) a3
    */
    - c1*a3*s23
    /*
      + cos(th1) sin(th2) sin(th3) d4
      - cos(th1) cos(th2) cos(th3) d4
    */
    - c1*d4*c23
    /*
      - cos(th1) sin(th2) a2
    */
    - c1*s2*a2;

  mat[0][2] =			/* dth3 */
    /*
      - cos(th1) cos(th2) sin(th3) a3
      - cos(th1) sin(th2) cos(th3) a3 
    */
    - c1*a3*s23
    /*
      - cos(th1) cos(th2) cos(th3) d4
      + cos(th1) sin(th2) sin(th3) d4
    */
    - c1*d4*c23;

  mat[0][3] = 0;		/* dth4 */
  mat[0][4] = 0;		/* dth5 */
  mat[0][5] = 0;		/* dth6 */

  /* vy */

  mat[1][0] =			/* dth1 */
    /*
      - cos(th1) d4 cos(th3) sin(th2)
      - cos(th1) d4 sin(th3) cos(th2)
    */
    - c1*d4*s23
    /*
      - cos(th1) a3 sin(th3) sin(th2)
      + cos(th1) a3 cos(th3) cos(th2)
    */
    + c1*a3*c23
    /*
      + cos(th1) cos(th2) a2
      - sin(th1) d3 
    */
    + c1*c2*a2 - s1*d3;

  mat[1][1] =			/* dth2 */
    /*
      - sin(th1) cos(th2) sin(th3) a3
      - sin(th1) sin(th2) cos(th3) a3
    */
    - s1*a3*s23
    /*
      + sin(th1) sin(th2) sin(th3) d4
      - sin(th1) cos(th2) cos(th3) d4
    */
    - s1*d4*c23
    /*
      - sin(th1) sin(th2) a2
    */
    - s1*s2*a2;

  mat[1][2] =			/* dth3 */
    /*
      - sin(th1) cos(th2) cos(th3) d4
      + sin(th1) sin(th2) sin(th3) d4
    */
    - s1*d4*c23
    /*
      - sin(th1) cos(th2) sin(th3) a3
      - sin(th1) sin(th2) cos(th3) a3
    */
    - s1*a3*s23;

  mat[1][3] = 0;		/* dth4 */
  mat[1][4] = 0;		/* dth5 */
  mat[1][5] = 0;		/* dth6 */

  /* vz */

  mat[2][0] = 0;		/* dth1 */

  mat[2][1] =			/* dth2 */
    /*
      + cos(th2) sin(th3) d4
      + sin(th2) cos(th3) d4
    */
    + d4*s23
    /*
      + sin(th2) sin(th3) a3
      - cos(th2) cos(th3) a3
    */
    - a3*c23
    /*
      - cos(th2) a2 
    */
    - c2*a2;

  mat[2][2] =			/* dth3 */
    /*
      - cos(th2) cos(th3) a3
      + sin(th2) sin(th3) a3
    */
    - a3*c23
    /*
      + cos(th2) sin(th3) d4
      + sin(th2) cos(th3) d4
    */
    + d4*s23;

  mat[2][3] = 0;
  mat[2][4] = 0;
  mat[2][5] = 0;
  
  /* wx */

  mat[3][0] = 0;		/* dth1 */

  mat[3][1] =			/* dth2 */
    /*
      - sin(th1) dth2
    */
    - s1;

  mat[3][2] =			/* dth3 */
    /*
      - sin(th1) dth3
    */
    - s1;

  mat[3][3] =			/* dth4 */
    /*
      - cos(th1) sin(th2) cos(th3)
      - cos(th1) cos(th2) sin(th3)
    */
    - c1*s23;

  mat[3][4] =			/* dth5 */
    /*
      + cos(th1) cos(th2) cos(th3) sin(th4)
      - cos(th1) sin(th2) sin(th3) sin(th4)
    */
    + c1*s4*c23
    /*
      - sin(th1) cos(th4)
    */
    - s1*c4;

  mat[3][5] =			/* dth6 */
    /*
      + cos(th1) sin(th2) sin(th3) cos(th4) sin(th5)
      - cos(th1) cos(th2) cos(th3) cos(th4) sin(th5)
    */
    - c1*c4*s5*c23
    /*
      - cos(th1) cos(th2) sin(th3) cos(th5)
      - cos(th1) sin(th2) cos(th3) cos(th5)
    */
    - c1*c5*s23
    /*
      - sin(th1) sin(th4) sin(th5)
    */
    - s1*s4*s5;

  /* wy */

  mat[4][0] = 0;		/* dth1 */

  mat[4][1] = 			/* dth2 */
  /*
    + cos(th1)
  */
    + c1;

  mat[4][2] =			/* dth3 */
  /*
    + cos(th1)
  */
    + c1;

  mat[4][3] =			/* dth4 */
  /*
    - sin(th1) sin(th2) cos(th3) 
    - sin(th1) cos(th2) sin(th3)
  */
    - s1*s23;

  mat[4][4] = 			/* dth5 */
  /*
    + sin(th1) cos(th2) cos(th3) sin(th4)
    - sin(th1) sin(th2) sin(th3) sin(th4)
    + cos(th1) cos(th4)
  */
    + s1*s4*c23
    + c1*c4;

  mat[4][5] =			/* dth6 */
    /*
      + sin(th1) sin(th2) sin(th3) cos(th4) sin(th5)
      - sin(th1) cos(th2) cos(th3) cos(th4) sin(th5)
    */
    - s1*c4*s5*c23
    /*
      - sin(th1) cos(th2) sin(th3) cos(th5)
      - sin(th1) sin(th2) cos(th3) cos(th5)
    */
    - s1*c5*s23
    /*
      + cos(th1) sin(th4) sin(th5)
    */
    + c1*s4*s5;
    
  /* wz */

  mat[5][0] = 1;		/* dth1 */
  mat[5][1] = 0;		/* dth2 */
  mat[5][2] = 0;		/* dth3 */

  mat[5][3] =			/* dth4 */
    /*
      + sin(th2) sin(th3)
      - cos(th2) cos(th3)
    */
    - c23;

  mat[5][4] =			/* dth5 */
    /*
      - cos(th2) sin(th3) sin(th4)
      - sin(th2) cos(th3) sin(th4)
    */
    - s4*s23;

  mat[5][5] =			/* dth6 */
    /*
      + sin(th2) cos(th3) cos(th4) sin(th5)
      + cos(th2) sin(th3) cos(th4) sin(th5) 
    */
    + c4*s5*s23
    /*
      + sin(th2) sin(th3) cos(th5)
      - cos(th2) cos(th3) cos(th5)
    */
    - c5*c23;

  return GO_RESULT_OK;
}

go_result spherist_kin_jac_fwd(void *kins,
			   const go_real *joints,
			   const go_real *jointvels,
			   const go_pose *pos,
			   go_vel *vel)
{
  go_real mat[6][6];
  go_real velvec[6];
  go_result retval;

  retval = jac_fwd_mat(kins, joints, mat);
  if (GO_RESULT_OK != retval) return retval;

  retval = go_mat6_vec6_mult(mat, (go_real *) jointvels, velvec);
  if (GO_RESULT_OK != retval) return retval;

  vel->v.x = velvec[0];
  vel->v.y = velvec[1];
  vel->v.z = velvec[2];
  vel->w.x = velvec[3];
  vel->w.y = velvec[4];
  vel->w.z = velvec[5];

  return GO_RESULT_OK;
}

go_result spherist_kin_jac_inv(void *kins,
			   const go_pose *pos,
			   const go_vel *vel,
			   const go_real *their_joints, 
			   go_real *jointvels)
{
  go_real mat[6][6];
  go_real inv[6][6];
  go_real our_joints[6];
  go_real * joints;
  go_real velvec[6];
   go_result retval;

  if (their_joints == NULL) {
    retval = spherist_kin_inv(kins, pos, our_joints);
    if (GO_RESULT_OK != retval) return retval;
    joints = our_joints;
  } else {
    joints = (go_real *) their_joints;
  }

  retval = jac_fwd_mat(kins, joints, mat);
  if (GO_RESULT_OK != retval) return retval;
  
  retval = go_mat6_inv(mat, inv);
  if (GO_RESULT_OK != retval) return retval;

  velvec[0] = vel->v.x;
  velvec[1] = vel->v.y;
  velvec[2] = vel->v.z;
  velvec[3] = vel->w.x;
  velvec[4] = vel->w.y;
  velvec[5] = vel->w.z;

  return go_mat6_vec6_mult(inv, velvec, jointvels);
} 
