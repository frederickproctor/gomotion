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
#include "gotypes.h"		/* go_result, go_integer */
#include "gomath.h"		/* go_pose */
#include "trivkins.h"

/* define USE_ZYZ to use Z-Y-Z Euler angles, else it's roll-pitch-yaw */
#define USE_ZYZ 1

/* how many joints we have */
#define TRIV_NUM_JOINTS 6

/* we don't need any record of the kinematics, so just return an int 
   so as to get something to point to */
go_integer triv_kin_size(void)
{
  return (go_integer) sizeof(int);
} 

go_result triv_kin_init(void *kins)
{
  return GO_RESULT_OK;
} 

const char *triv_kin_get_name(void)
{
  return "trivkins";
} 

go_integer triv_kin_num_joints(void *kins)
{
  return TRIV_NUM_JOINTS;
}

go_result triv_kin_fwd(void *kins,
		       const go_real *joint,
		       go_pose *world)
{
#ifdef USE_ZYZ
  go_zyz zyz;
#else
  go_rpy rpy;
#endif

  world->tran.x = joint[0];
  world->tran.y = joint[1];
  world->tran.z = joint[2];
#ifdef USE_ZYZ
  zyz.z = joint[3];
  zyz.y = joint[4];
  zyz.zp = joint[5];
  return go_zyz_quat_convert(&zyz, &world->rot);
#else
  rpy.r = joint[3];
  rpy.p = joint[4];
  rpy.y = joint[5];
  return go_rpy_quat_convert(&rpy, &world->rot);
#endif
} 

go_result triv_kin_inv(void *kins,
		       const go_pose *world,
		       go_real *joint)
{
#ifdef USE_ZYZ
  go_zyz zyz;
#else
  go_rpy rpy;
#endif

  joint[0] = world->tran.x;
  joint[1] = world->tran.y;
  joint[2] = world->tran.z;
#ifdef USE_ZYZ
  if (0 != go_quat_zyz_convert(&world->rot, &zyz)) return GO_RESULT_ERROR;
  joint[3] = zyz.z;
  joint[4] = zyz.y;
  joint[5] = zyz.zp;
#else
  if (0 != go_quat_rpy_convert(&world->rot, &rpy)) return GO_RESULT_ERROR;
  joint[3] = rpy.r;
  joint[4] = rpy.p;
  joint[5] = rpy.y;
#endif

  return GO_RESULT_OK;
} 

go_kin_type triv_kin_get_type(void *kins)
{
  return GO_KIN_BOTH;
} 

go_result triv_kin_set_parameters(void *kins, go_link *params, go_integer num)
{
  return GO_RESULT_OK;
} 

go_result triv_kin_get_parameters(void *kins, go_link *params, go_integer num)
{
  return GO_RESULT_OK;
} 

go_result triv_kin_jac_inv(void *kins,
			   const go_pose *pos,
			   const go_vel *vel,
			   const go_real *joints, 
			   go_real *jointvels)
{
  jointvels[0] = vel->v.x;
  jointvels[1] = vel->v.y;
  jointvels[2] = vel->v.z;
  jointvels[3] = vel->w.x;
  jointvels[4] = vel->w.y;
  jointvels[5] = vel->w.z;

  return GO_RESULT_OK;
} 

go_result triv_kin_jac_fwd(void *kins,
			   const go_real *joints,
			   const go_real *jointvels,
			   const go_pose *pos,
			   go_vel *vel)
{
  vel->v.x = jointvels[0];
  vel->v.y = jointvels[1];
  vel->v.z = jointvels[2];
  vel->w.x = jointvels[3];
  vel->w.y = jointvels[4];
  vel->w.z = jointvels[5];

  return GO_RESULT_OK;
}
