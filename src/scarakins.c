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
  scarakins.c

  Kinematics for SCARA 4-axis robots
*/

#include <stddef.h>		/* sizeof */
#include <math.h>		/* sin, cos */

#include "gotypes.h"		/* go_result, go_integer */
#include "gomath.h"		/* go_pose */
#include "gokin.h"		/* go_kin_type */
#include "scarakins.h"		/* these decls */

#define SCARA_NUM_JOINTS 4

go_integer scara_kin_size(void)
{
  return (go_integer) sizeof(scarakin_struct);
} 

/* default values for unit SCARA robot */
#define SCARA_L1 1.0
#define SCARA_L2 1.0

#define SINGULAR_FUZZ (1.0e-6)
#define FLAG_FUZZ     (1.0e-6)

go_result scara_kin_init(void * kins)
{
  scarakin_struct * sk = (scarakin_struct *) kins;

  sk->L1 = SCARA_L1;
  sk->L2 = SCARA_L2;
  sk->inv_2L1L2 = 0.5 / (sk->L1 * sk->L2);
  sk->iflags = 0;

  return GO_RESULT_OK;
} 

const char * scara_kin_get_name(void)
{
  return "scarakins";
} 

go_integer scara_kin_num_joints(void * kins)
{
  return SCARA_NUM_JOINTS;
}

/*
  x and y are easily found by vector math as

  x = L1 c1 + L2 c12
  y = L1 s1 + L2 s12

  z is even easier, as -l3. 

  Roll and pitch are always 0.

  Yaw is the sum of the planar joint angles th1 + th2 + th3.
*/

go_result scara_kin_fwd(void * kins,
			const go_real *joint,
			go_pose * world)
{
  scarakin_struct * sk = (scarakin_struct *) kins;
  go_rpy rpy;

  /* set elbow flag */
  sk->iflags = 0;
  if (joint[1] < 0.0) {
    sk->iflags |= SCARA_ELBOW_DOWN;
  }

  /* check if we're at a singular configuration */
  if (fabs(joint[1]) < SINGULAR_FUZZ ||
      fabs(joint[1] - GO_PI) < SINGULAR_FUZZ) {
    sk->iflags |= SCARA_SINGULAR;
  }

  world->tran.x = sk->L1 * cos(joint[0]) + sk->L2 * cos(joint[0] + joint[1]);
  world->tran.y = sk->L1 * sin(joint[0]) + sk->L2 * sin(joint[0] + joint[1]);
  world->tran.z = -joint[3];
  rpy.r = 0, rpy.p = 0, rpy.y = joint[0] + joint[1] + joint[2];

  return go_rpy_quat_convert(&rpy, &world->rot);
} 

/*
  The inverse kins follow the algebraic technique from John J. Craig's
  _Introduction to Robotics_, for the planar arm in section 4.4.
*/

go_result scara_kin_inv(void * kins,
			const go_pose * world,
			go_real *joint)
{
  scarakin_struct * sk = (scarakin_struct *) kins;
  go_real c2, s2, discr;
  go_real k1, k2;
  go_rpy rpy;

  c2 = (go_sq(world->tran.x) + go_sq(world->tran.y) -
	go_sq(sk->L1) - go_sq(sk->L2)) * sk->inv_2L1L2;

  discr = 1.0 - go_sq(c2);
  if (discr < 0.0) {
    return GO_RESULT_SINGULAR;
  }

  s2 = sqrt(discr);
  if (sk->iflags & SCARA_ELBOW_DOWN) {
    s2 = -s2;
  }

  joint[1] = atan2(s2, c2);

  k1 = sk->L1 + sk->L2 * c2;
  k2 = sk->L2 * s2;
  joint[0] = atan2(world->tran.y, world->tran.x) - atan2(k2, k1);

  go_quat_rpy_convert(&world->rot, &rpy);
  joint[2] = rpy.y - joint[0] - joint[1];

  joint[3] = -world->tran.z;	/* this one's easy */

  return GO_RESULT_OK;
} 

go_kin_type scara_kin_get_type(void * kins)
{
  return GO_KIN_BOTH;
} 

go_result scara_kin_set_parameters(void * kins, go_link * params, go_integer num)
{
  scarakin_struct * sk = (scarakin_struct *) kins;
  go_dh dh;

  if (num > SCARA_NUM_JOINTS) return GO_RESULT_ERROR;

  if (params[0].quantity != GO_QUANTITY_LENGTH) return GO_RESULT_ERROR;
  if (params[0].type == GO_LINK_DH) {
    sk->L1 = params[0].u.dh.d;
  } else if (params[0].type == GO_LINK_PP) {
    go_pose_dh_convert(&params[0].u.pp.pose, &dh);
    sk->L1 = dh.d;
  } else return GO_RESULT_ERROR;

  if (params[1].quantity != GO_QUANTITY_LENGTH) return GO_RESULT_ERROR;
  if (params[1].type == GO_LINK_DH) {
    sk->L2 = params[1].u.dh.d;
  } else if (params[1].type == GO_LINK_PP) {
    go_pose_dh_convert(&params[1].u.pp.pose, &dh);
    sk->L2 = dh.d;
  } else return GO_RESULT_ERROR;

  sk->inv_2L1L2 = 0.5 / (sk->L1 * sk->L2);

  return GO_RESULT_OK;
} 

go_result scara_kin_get_parameters(void * kins, go_link * params, go_integer num)
{
  scarakin_struct * sk = (scarakin_struct *) kins;

  if (num < SCARA_NUM_JOINTS) return GO_RESULT_ERROR;

  params[0].type = GO_LINK_DH;
  params[0].quantity = GO_QUANTITY_LENGTH;
  params[0].u.dh.d = sk->L1;

  params[1].type = GO_LINK_DH;
  params[1].quantity = GO_QUANTITY_LENGTH;
  params[1].u.dh.d = sk->L2;

  return GO_RESULT_OK;
} 

/*
  We define the joint vector as [th1 th2 th3 l3] and the world vector
  as [x y z w], where w is yaw.

  Given x and y as

  x = L1 c1 + L2 c12
  y = L1 s1 + L2 s12

  by straight differentiation we get

  xdot = -L1 s1 th1dot - L2 s12 (th1dot + th2dot)
  ydot = L1 c1 th1dot + L2 c12 (th1dot + th2dot)

  Trivially we have z = -l3 and w = th1 + th2 + th3, and

  zdot = -l3dot
  wdot = th1dot + th2dot  th3dot
*/

go_result scara_kin_jac_fwd(void * kins,
			   const go_real * joints,
			   const go_real * jointvels,
			   const go_pose * pos,
			   go_vel * vel)
{
  scarakin_struct * sk = (scarakin_struct *) kins;
  go_real s1, c1, s12, c12;

  s1 = sin(joints[0]), c1 = cos(joints[0]);
  s12 = sin(joints[0] + joints[1]), c12 = cos(joints[0] + joints[1]);

  vel->v.x = -sk->L1 * s1 * jointvels[0] - 
    sk->L2 * s12 * (jointvels[0] + jointvels[1]);

  vel->v.y = sk->L1 * c1 * jointvels[0] +
    sk->L2 * c12 * (jointvels[0] + jointvels[1]);

  vel->v.z = -jointvels[3];

  
vel->w.x = 0, vel->w.y = 0;
  vel->w.z = jointvels[0] + jointvels[1] + jointvels[2];

  return GO_RESULT_OK;
}

/*
  th1dot and th2dot are obtained by inverting the two Jacobian equations, 

  xdot = -L1 s1 th1dot - L2 s12 (th1dot + th2dot)
  ydot = L1 c1 th1dot + L2 c12 (th1dot + th2dot)

  This gives

  th1dot = (L2 c12 xdot + L2 s12 ydot) / D
  th2dot = ((L2 c12 + L1 c1) xdot + (L2 s12 + L1 s1) ydot) / (-D)
  or
  th2dot = (L1 c1 xdot + L1 s1 ydot) / (-D) - th1dot

  where D is L1 L2 s2.

  From wdot = th1dot + th2dot + th3dot, we get
  th3dot = wdot - th1dot - th2dot.

  Trivially we have l3dot = -zdot.
*/

go_result scara_kin_jac_inv(void * kins,
			   const go_pose * pos,
			   const go_vel * vel,
			   const go_real * joints, 
			   go_real * jointvels)
{
  scarakin_struct * sk = (scarakin_struct *) kins;
  go_real s1, c1, s12, c12;
  go_real L1L2s2;

  s1 = sin(joints[0]), c1 = cos(joints[0]);
  s12 = sin(joints[0] + joints[1]), c12 = cos(joints[0] + joints[1]);
  L1L2s2 = sk->L1 * sk->L2 * sin(joints[1]);

  if (GO_SMALL(L1L2s2)) {
    return GO_RESULT_SINGULAR;
  }
  L1L2s2 = 1.0 / L1L2s2;

  /* th1dot */
  jointvels[0] = sk->L2 * (c12 * vel->v.x + s12 * vel->v.y) * L1L2s2;
  /* th2dot */
  jointvels[1] = -sk->L1 * (c1 * vel->v.x + s1 * vel->v.y) * L1L2s2 - jointvels[0];

  /* th3dot */
  jointvels[2] = vel->w.z - jointvels[0] - jointvels[1];

  /* l3dot */
  jointvels[3] = -vel->v.z;

  return GO_RESULT_OK;
} 
