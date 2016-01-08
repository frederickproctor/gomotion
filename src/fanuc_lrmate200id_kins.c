/*
  fanuc_lrmate200id_kins.c
*/

#include <math.h>
#include "gotypes.h"		/* go_result, go_integer */
#include "gomath.h"		/* go_pose */
#include "gokin.h"		/* go_kin_type */
#include "three21kins.h"	/* three21_xxx */
#include "fanuc_lrmate200id_kins.h" /* these decls */

#if FANUC_LRMATE200ID_KIN_NUM_JOINTS != 6
#error FANUC_LRMATE200ID_KIN_NUM_JOINTS must equal 6
#endif

#define WRIST_OFFSET 0.080  /* 80 mm nominal offset in -Z for wrist */

go_integer fanuc_lrmate200id_kin_size(void)
{
  return (go_integer) sizeof(fanuc_lrmate200id_kin_struct);
}

go_kin_type fanuc_lrmate200id_kin_get_type(fanuc_lrmate200id_kin_struct *kins)
{
  return GO_KIN_BOTH;
}

go_result fanuc_lrmate200id_kin_init(fanuc_lrmate200id_kin_struct *kins)
{
  go_result result;

  kins->t7.tran.x = 0;
  kins->t7.tran.y = 0;
  kins->t7.tran.z = WRIST_OFFSET;
  kins->t7.rot.s = 1;
  kins->t7.rot.x = 0;
  kins->t7.rot.y = 0;
  kins->t7.rot.z = 0;

  result = go_pose_inv(&kins->t7, &kins->t7_inv);
  if (GO_RESULT_OK != result) return result;

  return three21_kin_init(&kins->tk);
}

const char *fanuc_lrmate200id_kin_get_name(void)
{
  return "fanuc_lrmate200id_kins";
}

go_integer fanuc_lrmate200id_kin_num_joints(fanuc_lrmate200id_kin_struct *kins)
{
  return FANUC_LRMATE200ID_KIN_NUM_JOINTS;
}

go_result fanuc_lrmate200id_kin_set_parameters(fanuc_lrmate200id_kin_struct *kins, go_link *params, go_integer num)
{
  if (num < 6) return GO_RESULT_ERROR;

  if (GO_RESULT_OK != three21_kin_set_parameters(&kins->tk, params, 6)) {
    /* force them to be what the Fanuc is */
    kins->tk.a1 = 0.050;
    kins->tk.a2 = 0.330;
    kins->tk.a3 = 0.035;
    kins->tk.d2 = 0;
    kins->tk.d3 = 0;
    kins->tk.d4 = 0.335;
    kins->tk.iflags = 0;
  }

  return GO_RESULT_OK;
}

go_result fanuc_lrmate200id_kin_get_parameters(fanuc_lrmate200id_kin_struct *kins, go_link *params, go_integer num)
{
  if (num < 6) return GO_RESULT_ERROR;

  return three21_kin_get_parameters(&kins->tk, params, 6);
}

/*
  The 'motors' in this context are the Fanuc's reported J values. They
  are converted into DH-convention 'joints', then run through the
  3-2-1 kinematics that assume DH conventions. Then the final tool
  transform is appended.
*/
go_result fanuc_lrmate200id_kin_fwd(fanuc_lrmate200id_kin_struct *kins,
				    const go_real *motors,
				    go_pose *pos)
{
  go_real joints[6];
  go_result retval;

  /* gearing equations */
  joints[0] = motors[0];
  joints[1] = motors[1] - GO_PI_2;
  joints[2] = motors[1] + motors[2];
  joints[3] = -motors[3];
  joints[4] = -motors[4];
  joints[5] = -motors[5];

  retval = three21_kin_fwd(&kins->tk, joints, pos);
  if (GO_RESULT_OK != retval) return retval;

  /* add the end frame */
  go_pose_pose_mult(pos, &kins->t7, pos);

  return GO_RESULT_OK;
}

go_result fanuc_lrmate200id_kin_inv(fanuc_lrmate200id_kin_struct *kins,
				    const go_pose *pos,
				    go_real *motors)
{
  go_pose end_pos;
  go_real joints[6];
  go_result retval;
  go_integer t;

  /* take off the end frame */
  go_pose_pose_mult(pos, &kins->t7_inv, &end_pos);
  
  retval = three21_kin_inv(&kins->tk, &end_pos, joints);
  if (GO_RESULT_OK != retval) return retval;

  /* gearing equations */
  motors[0] = joints[0];
  motors[1] = joints[1] + GO_PI_2;
  motors[2] = joints[2] - motors[1];
  motors[3] = -joints[3];
  motors[4] = -joints[4];
  motors[5] = -joints[5];

  /* normalize motor angles to range [-180..180] as Fanuc does */
  for (t = 0; t < 6; t++) {
    if (motors[t] < -GO_PI) motors[t] += GO_2_PI;
    else if (motors[t] > GO_PI) motors[t] -= GO_2_PI;
  }

  return GO_RESULT_OK;
}

go_result fanuc_lrmate200id_kin_jac_fwd(fanuc_lrmate200id_kin_struct *kins,
			    const go_real *motors,
			    const go_real *motorvels,
			    const go_pose *pos,
			    go_vel *vw)
{
  go_real joints[6];
  go_real jointvels[6];
  go_result retval;

  /* gearing equations */
  joints[0] = motors[0];
  joints[1] = motors[1] - GO_PI_2;
  joints[2] = motors[1] + motors[2];
  joints[3] = -motors[3];
  joints[4] = -motors[4];
  joints[5] = -motors[5];
  /* and their speeds */
  jointvels[0] = motorvels[0];
  jointvels[1] = motorvels[1];
  jointvels[2] = motorvels[1] + motorvels[2];
  jointvels[3] = -motorvels[3];
  jointvels[4] = -motorvels[4];
  jointvels[5] = -motorvels[5];

  /* run 3-2-1 kins with DH-convention joints and vels */
  retval = three21_kin_jac_fwd(&kins->tk, joints, jointvels, pos, vw);
  if (GO_RESULT_OK != retval) return retval;

  return GO_RESULT_OK;
}

go_result fanuc_lrmate200id_kin_jac_inv(fanuc_lrmate200id_kin_struct *kins,
			    const go_pose *pos,
			    const go_vel *vw,
			    const go_real *motors,
			    go_real *motorvels)
{
  go_real joints[6];
  go_real jointvels[6];
  go_result retval;

  /* convert the motor positions to joint positions */
  joints[0] = motors[0];
  joints[1] = motors[1] - GO_PI_2;
  joints[2] = motors[1] + motors[2];
  joints[3] = -motors[3];
  joints[4] = -motors[4];
  joints[5] = -motors[5];
  
  /* now call the inverse Jacobian with the true joints */
  retval = three21_kin_jac_inv(&kins->tk, pos, vw, joints, jointvels);
  if (GO_RESULT_OK != retval) return retval;

  /* now convert the joint vels into motor vels */
  motorvels[0] = jointvels[0];
  motorvels[1] = jointvels[1];
  motorvels[2] = jointvels[2] - motorvels[1];
  motorvels[3] = -jointvels[3];
  motorvels[4] = -jointvels[4];
  motorvels[5] = -jointvels[5];

  return GO_RESULT_OK;
}

go_result fanuc_lrmate200id_kin_set_flags(fanuc_lrmate200id_kin_struct *kins,
					  go_flag fflags,
					  go_flag iflags)
{
  kins->tk.iflags = iflags;

  return GO_RESULT_OK;
}

go_result fanuc_lrmate200id_kin_get_flags(fanuc_lrmate200id_kin_struct *kins,
					  go_flag *fflags,
					  go_flag *iflags)
{
  *fflags = 0;			/* no forward flags */
  *iflags = kins->tk.iflags;

  return GO_RESULT_OK;
}
