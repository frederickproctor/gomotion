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
#include "genhexkins.h"
#include "genserkins.h"
#include "fanuckins.h"
#include "fanuc_lrmate200id_kins.h"
#include "pumakins.h"
#include "scarakins.h"
#include "trivkins.h"
#include "tripointkins.h"
#include "spheristkins.h"
#include "robochkins.h"

static enum {
  USE_TRIVKINS = 1,
  USE_GENHEXKINS,
  USE_GENSERKINS,
  USE_FANUCKINS,
  USE_FANUC_LRMATE200ID_KINS,
  USE_PUMAKINS,
  USE_SCARAKINS,
  USE_TRIPOINTKINS,
  USE_SPHERISTKINS,
  USE_THREE21KINS,
  USE_ROBOCHKINS
} go_kin_which = USE_TRIVKINS;

static go_integer strmatch(const char * a, const char * b)
{
  for (; 0 != *a; a++, b++) {
    if (*a != *b) return 0;
  }
  return *a == *b;
}

go_result go_kin_select(const char * name)
{
  if (strmatch(name, triv_kin_get_name())) {
    go_kin_which = USE_TRIVKINS;
    return GO_RESULT_OK;
  }
  if (strmatch(name, genhex_kin_get_name())) {
    go_kin_which = USE_GENHEXKINS;
    return GO_RESULT_OK;
  }
  if (strmatch(name, genser_kin_get_name())) {
    go_kin_which = USE_GENSERKINS;
    return GO_RESULT_OK;
  }
  if (strmatch(name, fanuc_kin_get_name())) {
    go_kin_which = USE_FANUCKINS;
    return GO_RESULT_OK;
  }
  if (strmatch(name, fanuc_lrmate200id_kin_get_name())) {
    go_kin_which = USE_FANUC_LRMATE200ID_KINS;
    return GO_RESULT_OK;
  }
  if (strmatch(name, puma_kin_get_name())) {
    go_kin_which = USE_PUMAKINS;
    return GO_RESULT_OK;
  }
  if (strmatch(name, scara_kin_get_name())) {
    go_kin_which = USE_SCARAKINS;
    return GO_RESULT_OK;
  }
  if (strmatch(name, tripoint_kin_get_name())) {
    go_kin_which = USE_TRIPOINTKINS;
    return GO_RESULT_OK;
  }
  if (strmatch(name, spherist_kin_get_name())) {
    go_kin_which = USE_SPHERISTKINS;
    return GO_RESULT_OK;
  }
  if (strmatch(name, three21_kin_get_name())) {
    go_kin_which = USE_THREE21KINS;
    return GO_RESULT_OK;
  }
  if (strmatch(name, roboch_kin_get_name())) {
    go_kin_which = USE_ROBOCHKINS;
    return GO_RESULT_OK;
  }

  go_kin_which = USE_TRIVKINS;
  return GO_RESULT_ERROR;
}

go_integer go_kin_size(void)
{
  if (go_kin_which == USE_GENHEXKINS)
    return genhex_kin_size();
  if (go_kin_which == USE_GENSERKINS)
    return genser_kin_size();
  if (go_kin_which == USE_FANUCKINS)
    return fanuc_kin_size();
  if (go_kin_which == USE_FANUC_LRMATE200ID_KINS)
    return fanuc_lrmate200id_kin_size();
  if (go_kin_which == USE_PUMAKINS)
    return puma_kin_size();
  if (go_kin_which == USE_SCARAKINS)
    return scara_kin_size();
  if (go_kin_which == USE_TRIPOINTKINS)
    return tripoint_kin_size();
  if (go_kin_which == USE_SPHERISTKINS)
    return spherist_kin_size();
  if (go_kin_which == USE_THREE21KINS)
    return three21_kin_size();
  if (go_kin_which == USE_ROBOCHKINS)
    return roboch_kin_size();

  return triv_kin_size();
}

go_result go_kin_init(void * kins) 
{
  if (go_kin_which == USE_GENHEXKINS)
    return genhex_kin_init(kins);
  if (go_kin_which == USE_GENSERKINS)
    return genser_kin_init(kins);
  if (go_kin_which == USE_FANUCKINS)
    return fanuc_kin_init(kins);
  if (go_kin_which == USE_FANUC_LRMATE200ID_KINS)
    return fanuc_lrmate200id_kin_init(kins);
  if (go_kin_which == USE_PUMAKINS)
    return puma_kin_init(kins);
  if (go_kin_which == USE_SCARAKINS)
    return scara_kin_init(kins);
  if (go_kin_which == USE_TRIPOINTKINS)
    return tripoint_kin_init(kins);
  if (go_kin_which == USE_SPHERISTKINS)
    return spherist_kin_init(kins);
  if (go_kin_which == USE_THREE21KINS)
    return three21_kin_init(kins);
  if (go_kin_which == USE_ROBOCHKINS)
    return roboch_kin_init(kins);

  return triv_kin_init(kins);
}

const char * go_kin_get_name(void)
{
  if (go_kin_which == USE_GENHEXKINS)
    return genhex_kin_get_name();
  if (go_kin_which == USE_GENSERKINS)
    return genser_kin_get_name();
  if (go_kin_which == USE_FANUCKINS)
    return fanuc_kin_get_name();
  if (go_kin_which == USE_FANUC_LRMATE200ID_KINS)
    return fanuc_lrmate200id_kin_get_name();
  if (go_kin_which == USE_PUMAKINS)
    return puma_kin_get_name();
  if (go_kin_which == USE_SCARAKINS)
    return scara_kin_get_name();
  if (go_kin_which == USE_TRIPOINTKINS)
    return tripoint_kin_get_name();
  if (go_kin_which == USE_SPHERISTKINS)
    return spherist_kin_get_name();
  if (go_kin_which == USE_THREE21KINS)
    return three21_kin_get_name();
  if (go_kin_which == USE_ROBOCHKINS)
    return roboch_kin_get_name();

  return triv_kin_get_name();
}

go_integer go_kin_num_joints(void * kins)
{
  if (go_kin_which == USE_GENHEXKINS)
    return genhex_kin_num_joints(kins);
  if (go_kin_which == USE_GENSERKINS)
    return genser_kin_num_joints(kins);
  if (go_kin_which == USE_FANUCKINS)
    return fanuc_kin_num_joints(kins);
  if (go_kin_which == USE_FANUC_LRMATE200ID_KINS)
    return fanuc_lrmate200id_kin_num_joints(kins);
  if (go_kin_which == USE_PUMAKINS)
    return puma_kin_num_joints(kins);
  if (go_kin_which == USE_SCARAKINS)
    return scara_kin_num_joints(kins);
  if (go_kin_which == USE_TRIPOINTKINS)
    return tripoint_kin_num_joints(kins);
  if (go_kin_which == USE_SPHERISTKINS)
    return spherist_kin_num_joints(kins);
  if (go_kin_which == USE_THREE21KINS)
    return three21_kin_num_joints(kins);
  if (go_kin_which == USE_ROBOCHKINS)
    return roboch_kin_num_joints(kins);

  return triv_kin_num_joints(kins);
}

go_result go_kin_fwd(void * kins,
		     const go_real *joint,
		     go_pose * world)
{
  if (go_kin_which == USE_GENHEXKINS)
    return genhex_kin_fwd(kins, joint, world);
  if (go_kin_which == USE_GENSERKINS)
    return genser_kin_fwd(kins, joint, world);
  if (go_kin_which == USE_FANUCKINS)
    return fanuc_kin_fwd(kins, joint, world);
  if (go_kin_which == USE_FANUC_LRMATE200ID_KINS)
    return fanuc_lrmate200id_kin_fwd(kins, joint, world);
  if (go_kin_which == USE_PUMAKINS)
    return puma_kin_fwd(kins, joint, world);
  if (go_kin_which == USE_SCARAKINS)
    return scara_kin_fwd(kins, joint, world);
  if (go_kin_which == USE_TRIPOINTKINS)
    return tripoint_kin_fwd(kins, joint, world);
  if (go_kin_which == USE_SPHERISTKINS)
    return spherist_kin_fwd(kins, joint, world);
  if (go_kin_which == USE_THREE21KINS)
    return three21_kin_fwd(kins, joint, world);
  if (go_kin_which == USE_ROBOCHKINS)
    return roboch_kin_fwd(kins, joint, world);

  return triv_kin_fwd(kins, joint, world);
}

go_result go_kin_inv(void * kins,
		     const go_pose * world,
		     go_real *joint)
{
  if (go_kin_which == USE_GENHEXKINS)
    return genhex_kin_inv(kins, world, joint);
  if (go_kin_which == USE_GENSERKINS)
    return genser_kin_inv(kins, world, joint);
  if (go_kin_which == USE_FANUCKINS)
    return fanuc_kin_inv(kins, world, joint);
  if (go_kin_which == USE_FANUC_LRMATE200ID_KINS)
    return fanuc_lrmate200id_kin_inv(kins, world, joint);
  if (go_kin_which == USE_PUMAKINS)
    return puma_kin_inv(kins, world, joint);
  if (go_kin_which == USE_SCARAKINS)
    return scara_kin_inv(kins, world, joint);
  if (go_kin_which == USE_TRIPOINTKINS)
    return tripoint_kin_inv(kins, world, joint);
  if (go_kin_which == USE_SPHERISTKINS)
    return spherist_kin_inv(kins, world, joint);
  if (go_kin_which == USE_THREE21KINS)
    return three21_kin_inv(kins, world, joint);
  if (go_kin_which == USE_ROBOCHKINS)
    return roboch_kin_inv(kins, world, joint);

  return triv_kin_inv(kins, world, joint);
}

go_kin_type go_kin_get_type(void * kins)
{
  if (go_kin_which == USE_GENHEXKINS)
    return genhex_kin_get_type(kins);
  if (go_kin_which == USE_GENSERKINS)
    return genser_kin_get_type(kins);
  if (go_kin_which == USE_FANUCKINS)
    return fanuc_kin_get_type(kins);
  if (go_kin_which == USE_FANUC_LRMATE200ID_KINS)
    return fanuc_lrmate200id_kin_get_type(kins);
  if (go_kin_which == USE_PUMAKINS)
    return puma_kin_get_type(kins);
  if (go_kin_which == USE_SCARAKINS)
    return scara_kin_get_type(kins);
  if (go_kin_which == USE_TRIPOINTKINS)
    return tripoint_kin_get_type(kins);
  if (go_kin_which == USE_SPHERISTKINS)
    return spherist_kin_get_type(kins);
  if (go_kin_which == USE_THREE21KINS)
    return three21_kin_get_type(kins);
  if (go_kin_which == USE_ROBOCHKINS)
    return roboch_kin_get_type(kins);

  return triv_kin_get_type(kins);
}

go_result go_kin_set_parameters(void * kins, go_link * params, go_integer num)
{
  if (go_kin_which == USE_GENHEXKINS)
    return genhex_kin_set_parameters(kins, params, num);
  if (go_kin_which == USE_GENSERKINS)
    return genser_kin_set_parameters(kins, params, num);
  if (go_kin_which == USE_FANUCKINS)
    return fanuc_kin_set_parameters(kins, params, num);
  if (go_kin_which == USE_FANUC_LRMATE200ID_KINS)
    return fanuc_lrmate200id_kin_set_parameters(kins, params, num);
  if (go_kin_which == USE_PUMAKINS)
    return puma_kin_set_parameters(kins, params, num);
  if (go_kin_which == USE_SCARAKINS)
    return scara_kin_set_parameters(kins, params, num);
  if (go_kin_which == USE_TRIPOINTKINS)
    return tripoint_kin_set_parameters(kins, params, num);
  if (go_kin_which == USE_SPHERISTKINS)
    return spherist_kin_set_parameters(kins, params, num);
  if (go_kin_which == USE_THREE21KINS)
    return three21_kin_set_parameters(kins, params, num);
  if (go_kin_which == USE_ROBOCHKINS)
    return roboch_kin_set_parameters(kins, params, num);

  return triv_kin_set_parameters(kins, params, num);
}

go_result go_kin_get_parameters(void * kins, go_link * params, go_integer num)
{
  if (go_kin_which == USE_GENHEXKINS)
    return genhex_kin_get_parameters(kins, params, num);
  if (go_kin_which == USE_GENSERKINS)
    return genser_kin_get_parameters(kins, params, num);
  if (go_kin_which == USE_FANUCKINS)
    return fanuc_kin_get_parameters(kins, params, num);
  if (go_kin_which == USE_FANUC_LRMATE200ID_KINS)
    return fanuc_lrmate200id_kin_get_parameters(kins, params, num);
  if (go_kin_which == USE_PUMAKINS)
    return puma_kin_get_parameters(kins, params, num);
  if (go_kin_which == USE_SCARAKINS)
    return scara_kin_get_parameters(kins, params, num);
  if (go_kin_which == USE_TRIPOINTKINS)
    return tripoint_kin_get_parameters(kins, params, num);
  if (go_kin_which == USE_SPHERISTKINS)
    return spherist_kin_get_parameters(kins, params, num);
  if (go_kin_which == USE_THREE21KINS)
    return three21_kin_get_parameters(kins, params, num);
  if (go_kin_which == USE_ROBOCHKINS)
    return roboch_kin_get_parameters(kins, params, num);

  return triv_kin_get_parameters(kins, params, num);
}

go_result go_kin_jac_inv(void * kins,
			 const go_pose * pos,
			 const go_vel * vel,
			 const go_real * joints, go_real * jointvels)
{
  if (go_kin_which == USE_GENHEXKINS)
    return genhex_kin_jac_inv(kins, pos, vel, joints, jointvels);
  if (go_kin_which == USE_GENSERKINS)
    return genser_kin_jac_inv(kins, pos, vel, joints, jointvels);
  if (go_kin_which == USE_FANUCKINS)
    return fanuc_kin_jac_inv(kins, pos, vel, joints, jointvels);
  if (go_kin_which == USE_FANUC_LRMATE200ID_KINS)
    return fanuc_lrmate200id_kin_jac_inv(kins, pos, vel, joints, jointvels);
  if (go_kin_which == USE_PUMAKINS)
    return puma_kin_jac_inv(kins, pos, vel, joints, jointvels);
  if (go_kin_which == USE_SCARAKINS)
    return scara_kin_jac_inv(kins, pos, vel, joints, jointvels);
  if (go_kin_which == USE_TRIPOINTKINS)
    return tripoint_kin_jac_inv(kins, pos, vel, joints, jointvels);
  if (go_kin_which == USE_SPHERISTKINS)
    return spherist_kin_jac_inv(kins, pos, vel, joints, jointvels);
  if (go_kin_which == USE_THREE21KINS)
    return three21_kin_jac_inv(kins, pos, vel, joints, jointvels);
  if (go_kin_which == USE_ROBOCHKINS)
    return roboch_kin_jac_inv(kins, pos, vel, joints, jointvels);

  return triv_kin_jac_inv(kins, pos, vel, joints, jointvels);
}


go_result go_kin_jac_fwd(void * kins,
			 const go_real * joints,
			 const go_real * jointvels,
			 const go_pose * pos,
			 go_vel * vel)
{
  if (go_kin_which == USE_GENHEXKINS)
    return genhex_kin_jac_fwd(kins, joints, jointvels, pos, vel);
  if (go_kin_which == USE_GENSERKINS)
    return genser_kin_jac_fwd(kins, joints, jointvels, pos, vel);
  if (go_kin_which == USE_FANUCKINS)
    return fanuc_kin_jac_fwd(kins, joints, jointvels, pos, vel);
  if (go_kin_which == USE_FANUC_LRMATE200ID_KINS)
    return fanuc_lrmate200id_kin_jac_fwd(kins, joints, jointvels, pos, vel);
  if (go_kin_which == USE_PUMAKINS)
    return puma_kin_jac_fwd(kins, joints, jointvels, pos, vel);
  if (go_kin_which == USE_SCARAKINS)
    return scara_kin_jac_fwd(kins, joints, jointvels, pos, vel);
  if (go_kin_which == USE_TRIPOINTKINS)
    return tripoint_kin_jac_fwd(kins, joints, jointvels, pos, vel);
  if (go_kin_which == USE_SPHERISTKINS)
    return spherist_kin_jac_fwd(kins, joints, jointvels, pos, vel);
  if (go_kin_which == USE_THREE21KINS)
    return three21_kin_jac_fwd(kins, joints, jointvels, pos, vel);
  if (go_kin_which == USE_ROBOCHKINS)
    return roboch_kin_jac_fwd(kins, joints, jointvels, pos, vel);

  return triv_kin_jac_fwd(kins, joints, jointvels, pos, vel);
}

go_result go_kin_set_flags(void *kins,
			   go_flag fflags,
			   go_flag iflags)
{
  if (go_kin_which == USE_FANUC_LRMATE200ID_KINS)
    return fanuc_lrmate200id_kin_set_flags(kins, fflags, iflags);

  /* others are not yet implemented -- to do */
  return GO_RESULT_IMPL_ERROR;
}

go_result go_kin_get_flags(void *kins,
			   go_flag *fflags,
			   go_flag *iflags)
{
  if (go_kin_which == USE_FANUC_LRMATE200ID_KINS)
    return fanuc_lrmate200id_kin_get_flags(kins, fflags, iflags);

  /* ditto */
  return GO_RESULT_IMPL_ERROR;
}

