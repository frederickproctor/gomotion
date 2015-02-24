/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#include "gotypes.h"		/* go_result, go_integer */
#include "gomath.h"		/* go_pose */
#include "gokin.h"		/* go_kin_fwd_flags */

#include "roboch.h"

static go_integer strmatch(const char * a, const char * b)
{
  for (; 0 != *a; a++, b++) {
    if (*a != *b) return 0;
  }
  return *a == *b;
}

go_result go_kin_select(const char * name)
{
  if (strmatch(name, roboch_kin_get_name())) {
    return GO_RESULT_OK;
  }

  return GO_RESULT_ERROR;
}

go_integer go_kin_size(void)
{
  return roboch_kin_size();
}

go_result go_kin_init(void * kins) 
{
  return roboch_kin_init(kins);
}

const char * go_kin_get_name(void)
{
  return roboch_kin_get_name();
}

go_integer go_kin_num_joints(void * kins)
{
  return roboch_kin_num_joints(kins);
}

go_result go_kin_fwd(void * kins,
		     const go_real *joint,
		     go_pose * world)
{
  return roboch_kin_fwd(kins, joint, world);
}

go_result go_kin_inv(void * kins,
		     const go_pose * world,
		     go_real *joint)
{
  return roboch_kin_inv(kins, world, joint);
}

go_kin_type go_kin_get_type(void * kins)
{
  return roboch_kin_get_type(kins);
}

go_result go_kin_set_parameters(void * kins, go_link * params, go_integer num)
{
  return roboch_kin_set_parameters(kins, params, num);
}

go_result go_kin_get_parameters(void * kins, go_link * params, go_integer num)
{
  return roboch_kin_get_parameters(kins, params, num);
}

go_result go_kin_jac_inv(void * kins,
			 const go_pose * pos,
			 const go_vel * vel,
			 const go_real * joints,
			 go_real * jointvels)
{
  return roboch_kin_jac_inv(kins, pos, vel, joints, jointvels);
}


go_result go_kin_jac_fwd(void * kins,
			 const go_real * joints,
			 const go_real * jointvels,
			 const go_pose * pos,
			 go_vel * vel)
{
  return roboch_kin_jac_fwd(kins, joints, jointvels, pos, vel);
}
