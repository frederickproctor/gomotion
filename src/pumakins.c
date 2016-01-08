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
#include "pumakins.h"		/* these decls */

#define PUMA_NUM_JOINTS 6

go_integer puma_kin_size(void)
{
  return (go_integer) sizeof(puma_kin_struct);
}

/* default values are for PUMA 560, in [m] */
#define PUMA560_A2 0.300
#define PUMA560_A3 0.050
#define PUMA560_D3 0.070
#define PUMA560_D4 0.400

#define SINGULAR_FUZZ (1.0e-6)
#define FLAG_FUZZ     (1.0e-6)

go_result puma_kin_init(void *kins)
{
  puma_kin_struct *pk = (puma_kin_struct *) kins;

  pk->a2 = PUMA560_A2;
  pk->a3 = PUMA560_A3;
  pk->d3 = PUMA560_D3;
  pk->d4 = PUMA560_D4;
  pk->iflags = 0;

  return GO_RESULT_OK;
}

const char *puma_kin_get_name(void)
{
  return "pumakins";
}

go_integer puma_kin_num_joints(void *kins)
{
  return PUMA_NUM_JOINTS;
}

go_result puma_kin_fwd(void *kins,
		       const go_real *joint,
		       go_pose *world)
{
  puma_kin_struct *pk = (puma_kin_struct *) kins;
  go_real s1, s2, s3, s4, s5, s6;
  go_real c1, c2, c3, c4, c5, c6;
  go_real s23;
  go_real c23;
  go_real t1, t2, t3, t4, t5;
  go_real sumSq, k;
  go_hom hom;

  s1 = sin(joint[0]);
  s2 = sin(joint[1]);
  s3 = sin(joint[2]);
  s4 = sin(joint[3]);
  s5 = sin(joint[4]);
  s6 = sin(joint[5]);

  c1 = cos(joint[0]);
  c2 = cos(joint[1]);
  c3 = cos(joint[2]);
  c4 = cos(joint[3]);
  c5 = cos(joint[4]);
  c6 = cos(joint[5]);

  /* sin,cos(2+3) */
  s23 = c2 * s3 + s2 * c3;
  c23 = c2 * c3 - s2 * s3;

  /* Calculate terms to be used in definition of... */
  /* first column of rotation matrix.               */
  t1 = c4 * c5 * c6 - s4 * s6;
  t2 = s23 * s5 * c6;
  t3 = s4 * c5 * c6 + c4 * s6;
  t4 = c23 * t1 - t2;
  t5 = c23 * s5 * c6;

  /* Define first column of rotation matrix */
  hom.rot.x.x = c1 * t4 + s1 * t3;
  hom.rot.x.y = s1 * t4 - c1 * t3;
  hom.rot.x.z = -s23 * t1 - t5;

  /* Calculate terms to be used in definition of...  */
  /* second column of rotation matrix.               */
  t1 = -c4 * c5 * s6 - s4 * c6;
  t2 = s23 * s5 * s6;
  t3 = c4 * c6 - s4 * c5 * s6;
  t4 = c23 * t1 + t2;
  t5 = c23 * s5 * s6;

  /* Define second column of rotation matrix */
  hom.rot.y.x = c1 * t4 + s1 * t3;
  hom.rot.y.y = s1 * t4 - c1 * t3;
  hom.rot.y.z = -s23 * t1 + t5;

  /* Calculate term to be used in definition of... */
  /* third column of rotation matrix.              */
  t1 = c23 * c4 * s5 + s23 * c5;

  /* Define third column of rotation matrix */
  hom.rot.z.x = -c1 * t1 - s1 * s4 * s5;
  hom.rot.z.y = -s1 * t1 + c1 * s4 * s5;
  hom.rot.z.z = s23 * c4 * s5 - c23 * c5;

  /* Calculate term to be used in definition of...  */
  /* position vector.                               */
  t1 = pk->a2 * c2 + pk->a3 * c23 - pk->d4 * s23;

  /* Define position vector */
  hom.tran.x = c1 * t1 - pk->d3 * s1;
  hom.tran.y = s1 * t1 + pk->d3 * c1;
  hom.tran.z = -pk->a3 * s23 - pk->a2 * s2 - pk->d4 * c23;

  /* Calculate terms to be used to...   */
  /* determine flags.                   */
  sumSq = hom.tran.x * hom.tran.x + hom.tran.y * hom.tran.y - pk->d3 * pk->d3;
  k = (sumSq + hom.tran.z * hom.tran.z - pk->a2 * pk->a2 -
       pk->a3 * pk->a3 - pk->d4 * pk->d4) / (2.0 * pk->a2);

  /* reset flags */
  pk->iflags = 0;

  /* Set shoulder-up flag if necessary */
  if (fabs(joint[0] - atan2(hom.tran.y, hom.tran.x) +
	   atan2(pk->d3, -sqrt(sumSq))) < FLAG_FUZZ) {
    pk->iflags |= PUMA_SHOULDER_RIGHT;
  }

  /* Set elbow down flag if necessary */
  if (fabs(joint[2] - atan2(pk->a3, pk->d4) +
	   atan2(k, -sqrt(pk->a3 * pk->a3 +
			  pk->d4 * pk->d4 - k * k))) < FLAG_FUZZ) {
    pk->iflags |= PUMA_ELBOW_DOWN;
  }

  /* set singular flag if necessary */
  t1 = -hom.rot.z.x * s1 + hom.rot.z.y * c1;
  t2 = -hom.rot.z.x * c1 * c23 - hom.rot.z.y * s1 * c23 + hom.rot.z.z * s23;

  if (fabs(t1) < SINGULAR_FUZZ && fabs(t2) < SINGULAR_FUZZ) {
    pk->iflags |= PUMA_SINGULAR;
  } else {
    /* if not singular set wrist flip flag if necessary */
    if (!(fabs(joint[3] - atan2(t1, t2)) < FLAG_FUZZ)) {
      pk->iflags |= PUMA_WRIST_FLIP;
    }
  }

  return go_hom_pose_convert(&hom, world);
}

go_result puma_kin_inv(void *kins,
		       const go_pose *world,
		       go_real *joint)
{
  puma_kin_struct *pk = (puma_kin_struct *) kins;
  go_hom hom;

  go_real t1, t2, t3;
  go_real k;
  go_real sum_sq;

  go_real th1;
  go_real th3;
  go_real th23;
  go_real th2;
  go_real th4;
  go_real th5;
  go_real th6;

  go_real s1, c1;
  go_real s3, c3;
  go_real s23, c23;
  go_real s4, c4;
  go_real s5, c5;
  go_real s6, c6;

  /* convert pose to hom */
  go_pose_hom_convert(world, &hom);

  /* Joint 1 (2 independent solutions) */

  /* save sum of squares for this and subsequent calcs */
  sum_sq = hom.tran.x * hom.tran.x + hom.tran.y * hom.tran.y -
    pk->d3 * pk->d3;

  /* FIXME-- is use of + sqrt shoulder right or left? */
  if (pk->iflags & PUMA_SHOULDER_RIGHT) {
    th1 = atan2(hom.tran.y, hom.tran.x) - atan2(pk->d3, -sqrt(sum_sq));
  } else {
    th1 = atan2(hom.tran.y, hom.tran.x) - atan2(pk->d3, sqrt(sum_sq));
  }

  /* save sin, cos for later calcs */
  s1 = sin(th1);
  c1 = cos(th1);

  /* Joint 3 (2 independent solutions) */

  k = (sum_sq + hom.tran.z * hom.tran.z - pk->a2 * pk->a2 -
       pk->a3 * pk->a3 - pk->d4 * pk->d4) / (2.0 * pk->a2);

  /* FIXME-- is use of + sqrt elbow up or down? */
  if (pk->iflags & PUMA_ELBOW_DOWN) {
    th3 =
      atan2(pk->a3, pk->d4) - atan2(k,
				    -sqrt(pk->a3 * pk->a3 + pk->d4 * pk->d4 -
					  k * k));
  } else {
    th3 = atan2(pk->a3, pk->d4) -
      atan2(k, sqrt(pk->a3 * pk->a3 + pk->d4 * pk->d4 - k * k));
  }

  /* compute sin, cos for later calcs */
  s3 = sin(th3);
  c3 = cos(th3);

  /* Joint 2 */

  t1 = (-pk->a3 - pk->a2 * c3) * hom.tran.z +
    (c1 * hom.tran.x + s1 * hom.tran.y) * (pk->a2 * s3 - pk->d4);
  t2 = (pk->a2 * s3 - pk->d4) * hom.tran.z +
    (pk->a3 + pk->a2 * c3) * (c1 * hom.tran.x + s1 * hom.tran.y);
  t3 = hom.tran.z * hom.tran.z + (c1 * hom.tran.x + s1 * hom.tran.y) *
    (c1 * hom.tran.x + s1 * hom.tran.y);

  th23 = atan2(t1, t2);
  th2 = th23 - th3;

  /* compute sin, cos for later calcs */
  s23 = t1 / t3;
  c23 = t2 / t3;

  /* Joint 4 */

  t1 = -hom.rot.z.x * s1 + hom.rot.z.y * c1;
  t2 = -hom.rot.z.x * c1 * c23 - hom.rot.z.y * s1 * c23 + hom.rot.z.z * s23;
  if (fabs(t1) < SINGULAR_FUZZ && fabs(t2) < SINGULAR_FUZZ) {
    return GO_RESULT_SINGULAR;
  }

  th4 = atan2(t1, t2);

  /* compute sin, cos for later calcs */
  s4 = sin(th4);
  c4 = cos(th4);

  /* Joint 5 */

  s5 = hom.rot.z.z * (s23 * c4) -
    hom.rot.z.x * (c1 * c23 * c4 + s1 * s4) -
    hom.rot.z.y * (s1 * c23 * c4 - c1 * s4);
  c5 = -hom.rot.z.x * (c1 * s23) - hom.rot.z.y *
    (s1 * s23) - hom.rot.z.z * c23;
  th5 = atan2(s5, c5);

  /* Joint 6 */

  s6 = hom.rot.x.z * (s23 * s4) - hom.rot.x.x *
    (c1 * c23 * s4 - s1 * c4) - hom.rot.x.y * (s1 * c23 * s4 + c1 * c4);
  c6 = hom.rot.x.x * ((c1 * c23 * c4 + s1 * s4) *
		      c5 - c1 * s23 * s5) + hom.rot.x.y *
    ((s1 * c23 * c4 - c1 * s4) * c5 - s1 * s23 * s5) -
    hom.rot.x.z * (s23 * c4 * c5 + c23 * s5);
  th6 = atan2(s6, c6);

  /*
     Is wrist flip the normal or offset result? Absent agreement on
     this, we'll just define it ourselves.
   */
  if (pk->iflags & PUMA_WRIST_FLIP) {
    th4 = th4 + GO_PI;
    th5 = -th5;
    th6 = th6 + GO_PI;
  }

  /* copy out */
  joint[0] = th1;
  joint[1] = th2;
  joint[2] = th3;
  joint[3] = th4;
  joint[4] = th5;
  joint[5] = th6;

  return GO_RESULT_OK;
}

go_kin_type puma_kin_get_type(void *kins)
{
  return GO_KIN_BOTH;
}

go_result puma_kin_set_parameters(void *kins, go_link *params, go_integer num)
{
  puma_kin_struct *pk = (puma_kin_struct *) kins;
  go_dh dh;

  if (num > PUMA_NUM_JOINTS) {
    return GO_RESULT_ERROR;
  }
  if (params[2].quantity != GO_QUANTITY_ANGLE) {
    return GO_RESULT_ERROR;
  }

  if (params[2].type == GO_LINK_DH) {
    pk->a2 = params[2].u.dh.a;
    pk->d3 = params[2].u.dh.d;
  } else if (params[2].type == GO_LINK_PP) {
    go_pose_dh_convert(&params[2].u.pp.pose, &dh);
    pk->a2 = dh.a;
    pk->d3 = dh.d;
  } else {
    return GO_RESULT_ERROR;
  }

  if (params[3].quantity != GO_QUANTITY_ANGLE) {
    return GO_RESULT_ERROR;
  }
  if (params[3].type == GO_LINK_DH) {
    pk->a3 = params[3].u.dh.a;
    pk->d4 = params[3].u.dh.d;
  } else if (params[3].type == GO_LINK_PP) {
    go_pose_dh_convert(&params[3].u.pp.pose, &dh);
    pk->a3 = dh.a;
    pk->d4 = dh.d;
  } else {
    return GO_RESULT_ERROR;
  }

  return GO_RESULT_OK;
}

go_result puma_kin_get_parameters(void *kins,
				  go_link *params,
				  go_integer num)
{
  puma_kin_struct *pk = (puma_kin_struct *) kins;

  if (num < PUMA_NUM_JOINTS) {
    return GO_RESULT_ERROR;
  }

  params[2].type = GO_LINK_DH;
  params[2].quantity = GO_QUANTITY_ANGLE;
  params[2].u.dh.a = pk->a2;
  params[2].u.dh.d = pk->d3;

  params[3].type = GO_LINK_DH;
  params[3].quantity = GO_QUANTITY_ANGLE;
  params[3].u.dh.a = pk->a3;
  params[3].u.dh.d = pk->d4;

  return GO_RESULT_OK;
}

/*
  From doc/puma.mpl Maple program, with e.g.
  collect(V_6_0[1],{dth1,dth2,dth3});


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

static go_result jac_fwd_mat(const puma_kin_struct *kins,
			     const go_real *joints,
			     go_real mat[6][6])
{
  go_real s1, s2, s3, s4, s5;
  go_real c1, c2, c3, c4, c5;
  go_real s23, c23;		/* sin,cos(th2 + th3) */
  go_real a2, a3, d3, d4;

  s1 = sin(joints[0]), c1 = cos(joints[0]);
  s2 = sin(joints[1]), c2 = cos(joints[1]);
  s3 = sin(joints[2]), c3 = cos(joints[2]);
  s4 = sin(joints[3]), c4 = cos(joints[3]);
  s5 = sin(joints[4]), c5 = cos(joints[4]);
  s23 = s2 * c3 + c2 * s3;
  c23 = c2 * c3 - s2 * s3;
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
    +s1 * d4 * s23
    /*
       + sin(th1) a3 sin(th3) sin(th2)
       - sin(th1) a3 cos(th3) cos(th2)
     */
    - s1 * a3 * c23
    /*
       - sin(th1) cos(th2) a2
       - cos(th1) d3
     */
    - s1 * c2 * a2 - c1 * d3;

  mat[0][1] =			/* dth2 */
    /*
       - cos(th1) cos(th2) sin(th3) a3
       - cos(th1) sin(th2) cos(th3) a3
     */
    -c1 * a3 * s23
    /*
       + cos(th1) sin(th2) sin(th3) d4
       - cos(th1) cos(th2) cos(th3) d4
     */
    - c1 * d4 * c23
    /*
       - cos(th1) sin(th2) a2
     */
    - c1 * s2 * a2;

  mat[0][2] =			/* dth3 */
    /*
       - cos(th1) cos(th2) sin(th3) a3
       - cos(th1) sin(th2) cos(th3) a3 
     */
    -c1 * a3 * s23
    /*
       - cos(th1) cos(th2) cos(th3) d4
       + cos(th1) sin(th2) sin(th3) d4
     */
    - c1 * d4 * c23;

  mat[0][3] = 0;		/* dth4 */
  mat[0][4] = 0;		/* dth5 */
  mat[0][5] = 0;		/* dth6 */

  /* vy */

  mat[1][0] =			/* dth1 */
    /*
       - cos(th1) d4 cos(th3) sin(th2)
       - cos(th1) d4 sin(th3) cos(th2)
     */
    -c1 * d4 * s23
    /*
       - cos(th1) a3 sin(th3) sin(th2)
       + cos(th1) a3 cos(th3) cos(th2)
     */
    + c1 * a3 * c23
    /*
       + cos(th1) cos(th2) a2
       - sin(th1) d3 
     */
    + c1 * c2 * a2 - s1 * d3;

  mat[1][1] =			/* dth2 */
    /*
       - sin(th1) cos(th2) sin(th3) a3
       - sin(th1) sin(th2) cos(th3) a3
     */
    -s1 * a3 * s23
    /*
       + sin(th1) sin(th2) sin(th3) d4
       - sin(th1) cos(th2) cos(th3) d4
     */
    - s1 * d4 * c23
    /*
       - sin(th1) sin(th2) a2
     */
    - s1 * s2 * a2;

  mat[1][2] =			/* dth3 */
    /*
       - sin(th1) cos(th2) cos(th3) d4
       + sin(th1) sin(th2) sin(th3) d4
     */
    -s1 * d4 * c23
    /*
       - sin(th1) cos(th2) sin(th3) a3
       - sin(th1) sin(th2) cos(th3) a3
     */
    - s1 * a3 * s23;

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
    +d4 * s23
    /*
       + sin(th2) sin(th3) a3
       - cos(th2) cos(th3) a3
     */
    - a3 * c23
    /*
       - cos(th2) a2 
     */
    - c2 * a2;

  mat[2][2] =			/* dth3 */
    /*
       - cos(th2) cos(th3) a3
       + sin(th2) sin(th3) a3
     */
    -a3 * c23
    /*
       + cos(th2) sin(th3) d4
       + sin(th2) cos(th3) d4
     */
    + d4 * s23;

  mat[2][3] = 0;
  mat[2][4] = 0;
  mat[2][5] = 0;

  /* wx */

  mat[3][0] = 0;		/* dth1 */

  mat[3][1] =			/* dth2 */
    /*
       - sin(th1) dth2
     */
    -s1;

  mat[3][2] =			/* dth3 */
    /*
       - sin(th1) dth3
     */
    -s1;

  mat[3][3] =			/* dth4 */
    /*
       - cos(th1) sin(th2) cos(th3)
       - cos(th1) cos(th2) sin(th3)
     */
    -c1 * s23;

  mat[3][4] =			/* dth5 */
    /*
       + cos(th1) cos(th2) cos(th3) sin(th4)
       - cos(th1) sin(th2) sin(th3) sin(th4)
     */
    +c1 * s4 * c23
    /*
       - sin(th1) cos(th4)
     */
    - s1 * c4;

  mat[3][5] =			/* dth6 */
    /*
       + cos(th1) sin(th2) sin(th3) cos(th4) sin(th5)
       - cos(th1) cos(th2) cos(th3) cos(th4) sin(th5)
     */
    -c1 * c4 * s5 * c23
    /*
       - cos(th1) cos(th2) sin(th3) cos(th5)
       - cos(th1) sin(th2) cos(th3) cos(th5)
     */
    - c1 * c5 * s23
    /*
       - sin(th1) sin(th4) sin(th5)
     */
    - s1 * s4 * s5;

  /* wy */

  mat[4][0] = 0;		/* dth1 */

  mat[4][1] =			/* dth2 */
    /*
       + cos(th1)
     */
    +c1;

  mat[4][2] =			/* dth3 */
    /*
       + cos(th1)
     */
    +c1;

  mat[4][3] =			/* dth4 */
    /*
       - sin(th1) sin(th2) cos(th3) 
       - sin(th1) cos(th2) sin(th3)
     */
    -s1 * s23;

  mat[4][4] =			/* dth5 */
    /*
       + sin(th1) cos(th2) cos(th3) sin(th4)
       - sin(th1) sin(th2) sin(th3) sin(th4)
       + cos(th1) cos(th4)
     */
    +s1 * s4 * c23 + c1 * c4;

  mat[4][5] =			/* dth6 */
    /*
       + sin(th1) sin(th2) sin(th3) cos(th4) sin(th5)
       - sin(th1) cos(th2) cos(th3) cos(th4) sin(th5)
     */
    -s1 * c4 * s5 * c23
    /*
       - sin(th1) cos(th2) sin(th3) cos(th5)
       - sin(th1) sin(th2) cos(th3) cos(th5)
     */
    - s1 * c5 * s23
    /*
       + cos(th1) sin(th4) sin(th5)
     */
    + c1 * s4 * s5;

  /* wz */

  mat[5][0] = 1;		/* dth1 */
  mat[5][1] = 0;		/* dth2 */
  mat[5][2] = 0;		/* dth3 */

  mat[5][3] =			/* dth4 */
    /*
       + sin(th2) sin(th3)
       - cos(th2) cos(th3)
     */
    -c23;

  mat[5][4] =			/* dth5 */
    /*
       - cos(th2) sin(th3) sin(th4)
       - sin(th2) cos(th3) sin(th4)
     */
    -s4 * s23;

  mat[5][5] =			/* dth6 */
    /*
       + sin(th2) cos(th3) cos(th4) sin(th5)
       + cos(th2) sin(th3) cos(th4) sin(th5) 
     */
    +c4 * s5 * s23
    /*
       + sin(th2) sin(th3) cos(th5)
       - cos(th2) cos(th3) cos(th5)
     */
    - c5 * c23;

  return GO_RESULT_OK;
}

go_result puma_kin_jac_fwd(void *kins,
			   const go_real * joints,
			   const go_real * jointvels,
			   const go_pose * pos, go_vel * vel)
{
  go_real mat[6][6];
  go_real velvec[6];
  go_result retval;

  retval = jac_fwd_mat(kins, joints, mat);
  if (GO_RESULT_OK != retval) {
    return retval;
  }

  retval = go_mat6_vec6_mult(mat, (go_real *) jointvels, velvec);
  if (GO_RESULT_OK != retval) {
    return retval;
  }

  vel->v.x = velvec[0];
  vel->v.y = velvec[1];
  vel->v.z = velvec[2];
  vel->w.x = velvec[3];
  vel->w.y = velvec[4];
  vel->w.z = velvec[5];

  return GO_RESULT_OK;
}

go_result puma_kin_jac_inv(void *kins,
			   const go_pose *pos,
			   const go_vel *vel,
			   const go_real *their_joints,
			   go_real *jointvels)
{
  go_real mat[6][6];
  go_real inv[6][6];
  go_real our_joints[6];
  go_real *joints;
  go_real velvec[6];
  go_result retval;

  if (their_joints == NULL) {
    retval = puma_kin_inv(kins, pos, our_joints);
    if (GO_RESULT_OK != retval) {
      return retval;
    }
    joints = our_joints;
  } else {
    joints = (go_real *) their_joints;
  }

  retval = jac_fwd_mat(kins, joints, mat);
  if (GO_RESULT_OK != retval) {
    return retval;
  }

  retval = go_mat6_inv(mat, inv);
  if (GO_RESULT_OK != retval) {
    return retval;
  }

  velvec[0] = vel->v.x;
  velvec[1] = vel->v.y;
  velvec[2] = vel->v.z;
  velvec[3] = vel->w.x;
  velvec[4] = vel->w.y;
  velvec[5] = vel->w.z;

  return go_mat6_vec6_mult(inv, velvec, jointvels);
}

/* extensions */

go_result puma_kin_jac_transpose(void *kins,
				 const go_pose *pos,
				 const go_vel *ft,
				 const go_real *their_joints,
				 go_real *joint_ft)
{
  go_real mat[6][6];
  go_real transpose[6][6];
  go_real our_joints[6];
  go_real *joints;
  go_real ftvec[6];
  go_result retval;

  if (their_joints == NULL) {
    retval = puma_kin_inv(kins, pos, our_joints);
    if (GO_RESULT_OK != retval) {
      return retval;
    }
    joints = our_joints;
  } else {
    joints = (go_real *) their_joints;
  }

  retval = jac_fwd_mat(kins, joints, mat);
  if (GO_RESULT_OK != retval) {
    return retval;
  }

  retval = go_mat6_transpose(mat, transpose);
  if (GO_RESULT_OK != retval) {
    return retval;
  }

  ftvec[0] = ft->v.x;
  ftvec[1] = ft->v.y;
  ftvec[2] = ft->v.z;
  ftvec[3] = ft->w.x;
  ftvec[4] = ft->w.y;
  ftvec[5] = ft->w.z;

  return go_mat6_vec6_mult(transpose, ftvec, joint_ft);
}
