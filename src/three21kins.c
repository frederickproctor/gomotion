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
  three21kins.c
*/

#include <math.h>
#include "gotypes.h"		/* go_result, go_integer */
#include "gomath.h"		/* go_pose */
#include "gokin.h"		/* go_kin_type */
#include "genserkins.h"		/* Jacobians */
#include "three21kins.h"	/* these decls */

#if THREE21_KIN_NUM_JOINTS != 6
#error THREE21_KIN_NUM_JOINTS must equal 6
#endif

#define SINGULAR_FUZZ (1.0e-6)
#define FLAG_FUZZ     (1.0e-6)

go_integer three21_kin_size(void)
{
  return (go_integer) sizeof(three21_kin_struct);
}

go_result three21_kin_init(three21_kin_struct *kins)
{
  /* set the offsets to be non-zero so that we qualify as a normal 321 arm */
  kins->a1 = 1;
  kins->a2 = 1;
  kins->a3 = 1;
  kins->d2 = 1;
  kins->d3 = 1;
  kins->d4 = 1;
  kins->iflags = 0;

  return genser_kin_init(&kins->gk);
}

const char *three21_kin_get_name(void)
{
  return "three21kins";
}

go_integer three21_kin_num_joints(three21_kin_struct *kins)
{
  return THREE21_KIN_NUM_JOINTS;
}

go_result three21_kin_set_parameters(three21_kin_struct *kins, go_link *params, go_integer num)
{
  go_dh dh;
  go_integer index;
  go_result retval;

  if (num < 6) {
    return GO_RESULT_ERROR;
  }

#define CHECK_IT(INDEX)						\
  index = INDEX;						\
  if (params[INDEX].quantity != GO_QUANTITY_ANGLE) {		\
    return GO_RESULT_ERROR;					\
  }								\
  if (params[INDEX].type == GO_LINK_DH) {			\
    dh = params[INDEX].u.dh;					\
  } else if (params[INDEX].type == GO_LINK_PP) {		\
    retval = go_pose_dh_convert(&params[INDEX].u.pp.pose, &dh);	\
    if (GO_RESULT_OK != retval) {				\
      return retval;						\
    }								\
  } else {							\
    return GO_RESULT_ERROR;					\
  }								\
  kins->gk.links[INDEX] = params[INDEX]

  CHECK_IT(0);
  if (! GO_TRAN_CLOSE(dh.a, 0) || /* a0 */
      ! GO_ROT_CLOSE(dh.alpha, 0) || /* alpha0 */
      ! GO_TRAN_CLOSE(dh.d, 0)) {    /* d1 */
    return GO_RESULT_ERROR;
  }

  CHECK_IT(1);
  if (! GO_ROT_CLOSE(dh.alpha, -GO_PI_2)) { /* alpha1 */
    return GO_RESULT_ERROR;
  }
  kins->a1 = dh.a;
  kins->d2 = dh.d;

  CHECK_IT(2);
  if (! GO_ROT_CLOSE(dh.alpha, 0)) { /* alpha2 */
    return GO_RESULT_ERROR;
  }
  kins->a2 = dh.a;
  kins->d3 = dh.d;

  CHECK_IT(3);
  if (! GO_ROT_CLOSE(dh.alpha, -GO_PI_2)) { /* alpha3 */
    return GO_RESULT_ERROR;
  }
  kins->a3 = dh.a;
  kins->d4 = dh.d;

  CHECK_IT(4);
  if (! GO_TRAN_CLOSE(dh.a, 0) || /* a4 */
      ! GO_ROT_CLOSE(dh.alpha, GO_PI_2) || /* alpha4 */
      ! GO_TRAN_CLOSE(dh.d, 0)) {    /* d5 */
    return GO_RESULT_ERROR;
  }

  CHECK_IT(5);
  if (! GO_TRAN_CLOSE(dh.a, 0) || /* a5 */
      ! GO_ROT_CLOSE(dh.alpha, -GO_PI_2) || /* alpha5 */
      ! GO_TRAN_CLOSE(dh.d, 0)) {    /* d6 */
    return GO_RESULT_ERROR;
  }

  return GO_RESULT_OK;
}

go_result three21_kin_get_parameters(three21_kin_struct *kins, go_link *params, go_integer num)
{
  if (num < 6) return GO_RESULT_ERROR;

  params[0].quantity = GO_QUANTITY_ANGLE;
  params[0].type = GO_LINK_DH;
  params[0].u.dh.a = 0;
  params[0].u.dh.alpha = 0;
  params[0].u.dh.d = 0;
  params[0].u.dh.theta = 0;

  params[1].quantity = GO_QUANTITY_ANGLE;
  params[1].type = GO_LINK_DH;
  params[1].u.dh.a = kins->a1;
  params[1].u.dh.alpha = -GO_PI_2;
  params[1].u.dh.d = kins->d2;
  params[1].u.dh.theta = 0;

  params[2].quantity = GO_QUANTITY_ANGLE;
  params[2].type = GO_LINK_DH;
  params[2].u.dh.a = kins->a2;
  params[2].u.dh.alpha = 0;
  params[2].u.dh.d = kins->d3;
  params[2].u.dh.theta = 0;

  params[3].quantity = GO_QUANTITY_ANGLE;
  params[3].type = GO_LINK_DH;
  params[3].u.dh.a = kins->a3;
  params[3].u.dh.alpha = -GO_PI_2;
  params[3].u.dh.d = kins->d4;
  params[3].u.dh.theta = 0;

  params[4].quantity = GO_QUANTITY_ANGLE;
  params[4].type = GO_LINK_DH;
  params[4].u.dh.a = 0;
  params[4].u.dh.alpha = GO_PI_2;
  params[4].u.dh.d = 0;
  params[4].u.dh.theta = 0;

  params[5].quantity = GO_QUANTITY_ANGLE;
  params[5].type = GO_LINK_DH;
  params[5].u.dh.a = -GO_PI_2;
  params[5].u.dh.alpha = 0;
  params[5].u.dh.d = 0;
  params[5].u.dh.theta = 0;

  return GO_RESULT_OK;
}

go_result three21_kin_fwd(three21_kin_struct *kins,
			const go_real *joints,
			go_pose *world)
{
  go_real s1, s2, s3, s4, s5, s6;
  go_real c1, c2, c3, c4, c5, c6;
  go_real s23;
  go_real c23;
  go_real t1, t2, t3, t4, t5;
  go_real sum_sq, k, d23;
  go_hom hom;

  go_sincos(joints[0], &s1, &c1);
  go_sincos(joints[1], &s2, &c2);
  go_sincos(joints[2], &s3, &c3);
  go_sincos(joints[3], &s4, &c4);
  go_sincos(joints[4], &s5, &c5);
  go_sincos(joints[5], &s6, &c6);

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
  t1 = kins->a1 + kins->a2 * c2 + kins->a3 * c23 - kins->d4 * s23;

  /* Define position vector */
  d23 = kins->d2 + kins->d3;
  hom.tran.x = c1 * t1 - d23 * s1;
  hom.tran.y = s1 * t1 + d23 * c1;
  hom.tran.z = -kins->a3 * s23 - kins->a2 * s2 - kins->d4 * c23;

  /* Calculate terms to be used to...   */
  /* determine flags.                   */
  sum_sq = hom.tran.x * hom.tran.x
    + hom.tran.y * hom.tran.y
    - d23 * d23;
  k = (sum_sq
       + hom.tran.z * hom.tran.z 
       + kins->a1 * kins->a1
       - 2 * kins->a1 * (c1 * hom.tran.x + s1 * hom.tran.y)
       - kins->a2 * kins->a2
       - kins->a3 * kins->a3
       - kins->d4 * kins->d4) / (2.0 * kins->a2);

  /* reset flags */
  kins->iflags = 0;

  /* Set shoulder-up flag if necessary */
  if (fabs(joints[0] - atan2(hom.tran.y, hom.tran.x) +
	   atan2(d23, -sqrt(sum_sq))) < FLAG_FUZZ) {
    kins->iflags |= THREE21_SHOULDER_RIGHT;
  }

  /* Set elbow down flag if necessary */
  if (fabs(joints[2] - atan2(kins->a3, kins->d4) +
	   atan2(k, -sqrt(kins->a3 * kins->a3 +
			  kins->d4 * kins->d4 - k * k))) < FLAG_FUZZ) {
    kins->iflags |= THREE21_ELBOW_DOWN;
  }

  /* set singular flag if necessary */
  t1 = -hom.rot.z.x * s1 + hom.rot.z.y * c1;
  t2 = -hom.rot.z.x * c1 * c23 - hom.rot.z.y * s1 * c23 + hom.rot.z.z * s23;

  if (fabs(t1) < SINGULAR_FUZZ && fabs(t2) < SINGULAR_FUZZ) {
    kins->iflags |= THREE21_SINGULAR;
  } else {
    /* if not singular set wrist flip flag if necessary */
    if (!(fabs(joints[3] - atan2(t1, t2)) < FLAG_FUZZ)) {
      kins->iflags |= THREE21_WRIST_FLIP;
    }
  }

  return go_hom_pose_convert(&hom, world);
}

go_result three21_kin_inv(three21_kin_struct *kins,
			  const go_pose *world,
			  go_real *joints)
{
  go_hom hom;

  go_real t1, t2, t3;
  go_real k, sum_sq, d23;

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
  d23 = kins->d2 + kins->d3;
  sum_sq = hom.tran.x * hom.tran.x
    + hom.tran.y * hom.tran.y 
    - d23 * d23;

  /* FIXME-- is use of + sqrt shoulder right or left? */
  if (kins->iflags & THREE21_SHOULDER_RIGHT) {
    th1 = atan2(hom.tran.y, hom.tran.x) - atan2(d23, -sqrt(sum_sq));
  } else {
    th1 = atan2(hom.tran.y, hom.tran.x) - atan2(d23, sqrt(sum_sq));
  }

  /* save sin, cos for later calcs */
  go_sincos(th1, &s1, &c1);

  /* Joint 3 (2 independent solutions) */

  k = (sum_sq
       + hom.tran.z * hom.tran.z
       + kins->a1 * kins->a1
       - 2 * kins->a1 * (c1 * hom.tran.x + s1 * hom.tran.y)
       - kins->a2 * kins->a2
       - kins->a3 * kins->a3
       - kins->d4 * kins->d4) / (2.0 * kins->a2);

  /* FIXME-- is use of + sqrt elbow up or down? */
  if (kins->iflags & THREE21_ELBOW_DOWN) {
    th3 = atan2(kins->a3, kins->d4) - atan2(k, -sqrt(kins->a3 * kins->a3 + kins->d4 * kins->d4 - k * k));
  } else {
    th3 = atan2(kins->a3, kins->d4) - atan2(k, sqrt(kins->a3 * kins->a3 + kins->d4 * kins->d4 - k * k));
  }

  /* compute sin, cos for later calcs */
  go_sincos(th3, &s3, &c3);

  /* Joint 2 */

  t1 = (-kins->a3 - kins->a2 * c3) * hom.tran.z + (c1 * hom.tran.x + s1 * hom.tran.y - kins->a1) * (kins->a2 * s3 - kins->d4);
  t2 = (kins->a2 * s3 - kins->d4) * hom.tran.z + (kins->a3 + kins->a2 * c3) * (c1 * hom.tran.x + s1 * hom.tran.y - kins->a1);
  t3 = hom.tran.z * hom.tran.z + (c1 * hom.tran.x + s1 * hom.tran.y - kins->a1) * (c1 * hom.tran.x + s1 * hom.tran.y - kins->a1);

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
  if (kins->iflags & THREE21_WRIST_FLIP) {
    th4 = th4 + GO_PI;
    th5 = -th5;
    th6 = th6 + GO_PI;
  }

  /* copy out */
  joints[0] = th1;
  joints[1] = th2;
  joints[2] = th3;
  joints[3] = th4;
  joints[4] = th5;
  joints[5] = th6;

  return GO_RESULT_OK;
}

go_kin_type three21_kin_get_type(three21_kin_struct *kins)
{
  return GO_KIN_BOTH;
}

go_result three21_kin_jac_fwd(three21_kin_struct *kins,
			      const go_real *joints,
			      const go_real *jointvels,
			      const go_pose *pos, 
			      go_vel *vel)
{
  return genser_kin_jac_fwd(&kins->gk, joints, jointvels, pos, vel);
}

go_result three21_kin_jac_inv(three21_kin_struct *kins,
			      const go_pose *pos,
			      const go_vel *vel,
			      const go_real *joints, 
			      go_real *jointvels)
{
  return genser_kin_jac_inv(&kins->gk, pos, vel, joints, jointvels);
}
