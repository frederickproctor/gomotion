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
#include <math.h>		/* sqrt */
#include "gotypes.h"		/* go_result, go_integer */
#include "gomath.h"		/* go_pose */
#include "tripointkins.h"

#define TRIPOINT_NUM_JOINTS 3

#define sq(x) ((x)*(x))
#define mag(x,y,z) sqrt(sq(x)+sq(y)+sq(z))

go_integer tripoint_kin_size(void)
{
  return (go_integer) sizeof(tripoint_kin_struct);
} 

/* default values are for a unit-side equilateral triangle */

#define SCALE 2.0
#define TRIPOINT_X1 (SCALE * 1.0)
#define TRIPOINT_X2 (SCALE * 0.5) /* cos 60 */
#define TRIPOINT_Y2 (SCALE * .866025403784439) /* sin 60 */

go_result tripoint_kin_init(void * kins)
{
  tripoint_kin_struct * tpk = (tripoint_kin_struct *) kins;

  tpk->x1 = TRIPOINT_X1;
  tpk->x2 = TRIPOINT_X2;
  tpk->y2 = TRIPOINT_Y2;
  tpk->fflags = 0;

  return GO_RESULT_OK;
} 

const char * tripoint_kin_get_name(void)
{
  return "tripointkins";
} 

go_integer tripoint_kin_num_joints(void * kins)
{
  return TRIPOINT_NUM_JOINTS;
}

go_result tripoint_kin_fwd(void * kins,
			   const go_real * joints,
			   go_pose * world)
{
  tripoint_kin_struct * tpk;
  go_real discr;

  tpk = (tripoint_kin_struct *) kins;

  world->tran.x =
    0.5 * (tpk->x1 - (sq(joints[1]) - sq(joints[0]))/tpk->x1);

  world->tran.y =
    (sq(joints[0]) - 2.0 * world->tran.x * tpk->x2 + sq(tpk->x2) +
     sq(tpk->y2) - sq(joints[2])) / (2.0 * tpk->y2);

  discr = sq(joints[0]) - sq(world->tran.x) - sq(world->tran.y);
  if (discr < -GO_REAL_EPSILON) {
    return GO_RESULT_DOMAIN_ERROR;
  }
  if (discr < 0.0) discr = 0.0;	/* make slightly negative numbers 0 */

  world->tran.z = sqrt(discr);
  if (tpk->fflags == TRIPOINT_Z_NEGATIVE) {
    world->tran.z = -world->tran.z;
  }

  /* no orientation control */
  world->rot.s = 1.0;
  world->rot.x = 0.0;
  world->rot.y = 0.0;
  world->rot.z = 0.0;

  return GO_RESULT_OK;
} 

go_result tripoint_kin_inv(void * kins,
			   const go_pose * world,
			   go_real * joints)
{
  tripoint_kin_struct * tpk;

  tpk = (tripoint_kin_struct *) kins;

  joints[0] = mag(world->tran.x, world->tran.y, world->tran.z);
  joints[1] = mag(world->tran.x - tpk->x1, world->tran.y, world->tran.z);
  joints[2] = mag(world->tran.x - tpk->x2, world->tran.y - tpk->y2, world->tran.z);

  if (world->tran.z >= 0.0) {
    tpk->fflags = TRIPOINT_Z_POSITIVE;
  } else {
    tpk->fflags = TRIPOINT_Z_NEGATIVE;
  }

  return GO_RESULT_OK;
} 

go_kin_type tripoint_kin_get_type(void * kins)
{
  return GO_KIN_BOTH;
} 

go_result tripoint_kin_set_parameters(void * kins, go_link * params, go_integer num)
{
  tripoint_kin_struct * tpk = (tripoint_kin_struct *) kins;
  go_dh dh;

  if (num > TRIPOINT_NUM_JOINTS) return GO_RESULT_ERROR;

  if (params[0].quantity != GO_QUANTITY_LENGTH) return GO_RESULT_ERROR;
  if (params[0].type == GO_LINK_DH) {
    tpk->x1 = params[0].u.dh.d;
  } else if (params[0].type == GO_LINK_PP) {
    go_pose_dh_convert(&params[0].u.pp.pose, &dh);
    tpk->x1 = dh.d;
  } else return GO_RESULT_ERROR;

  if (params[1].quantity != GO_QUANTITY_LENGTH) return GO_RESULT_ERROR;
  if (params[1].type == GO_LINK_DH) {
    tpk->x2 = params[1].u.dh.d;
  } else if (params[1].type == GO_LINK_PP) {
    go_pose_dh_convert(&params[1].u.pp.pose, &dh);
    tpk->x2 = dh.d;
  } else return GO_RESULT_ERROR;

  if (params[2].quantity != GO_QUANTITY_LENGTH) return GO_RESULT_ERROR;
  if (params[2].type == GO_LINK_DH) {
    tpk->y2 = params[2].u.dh.d;
  } else if (params[2].type == GO_LINK_PP) {
    go_pose_dh_convert(&params[2].u.pp.pose, &dh);
    tpk->y2 = dh.d;
  } else return GO_RESULT_ERROR;

  return GO_RESULT_OK;
} 

go_result tripoint_kin_get_parameters(void * kins, go_link * params, go_integer num)
{
  tripoint_kin_struct * tpk = (tripoint_kin_struct *) kins;

  if (num < TRIPOINT_NUM_JOINTS) return GO_RESULT_ERROR;

  params[0].type = GO_LINK_DH;
  params[0].quantity = GO_QUANTITY_LENGTH;
  params[0].u.dh.d = tpk->x1;

  params[1].type = GO_LINK_DH;
  params[1].quantity = GO_QUANTITY_LENGTH;
  params[1].u.dh.d = tpk->x2;

  params[2].type = GO_LINK_DH;
  params[2].quantity = GO_QUANTITY_LENGTH;
  params[2].u.dh.d = tpk->y2;

  return GO_RESULT_OK;
} 

go_result tripoint_kin_jac_inv(void * kins,
			       const go_pose * pos,
			       const go_vel * vel,
			       const go_real * joints, 
			       go_real * jointvels)
{
  /* FIXME-- unimplemented */
  return GO_RESULT_IMPL_ERROR;
} 

go_result tripoint_kin_jac_fwd(void * kins,
			       const go_real * joints,
			       const go_real * jointvels,
			       const go_pose * pos,
			       go_vel * vel)
{
  /* FIXME-- unimplemented */
  return GO_RESULT_IMPL_ERROR;
}
