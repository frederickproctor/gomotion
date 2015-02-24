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
  fanuckins.c

  Kinematics functions for a Fanuc-like robot on a gantry, using
  the three21kins functions for most of the work.
*/

#include <math.h>
#include "gotypes.h"		/* go_result, go_integer */
#include "gomath.h"		/* go_pose */
#include "gokin.h"		/* go_kin_type */
#include "three21kins.h"	/* three21_xxx */
#include "fanuckins.h"		/* these decls */

#if FANUC_KIN_NUM_JOINTS != 7
#error FANUC_KIN_NUM_JOINTS must equal 7
#endif

#define WRIST_OFFSET 0.100	/* 100 mm offset in Z for wrist */

go_integer fanuc_kin_size(void)
{
  return (go_integer) sizeof(fanuc_kin_struct);
}

go_kin_type fanuc_kin_get_type(fanuc_kin_struct *kins)
{
  return GO_KIN_BOTH;
}

go_result fanuc_kin_init(fanuc_kin_struct *kins)
{
  kins->gantry = 0;

  return three21_kin_init(&kins->tk);
}

const char *fanuc_kin_get_name(void)
{
  return "fanuckins";
}

go_integer fanuc_kin_num_joints(fanuc_kin_struct *kins)
{
  return 7;
}

go_result fanuc_kin_set_parameters(fanuc_kin_struct *kins, go_link *params, go_integer num)
{
  if (num < 6) return GO_RESULT_ERROR; /* OK to not set the gantry */

  if (GO_RESULT_OK != three21_kin_set_parameters(&kins->tk, params, 6)) {
    /* force them to be what the Fanuc is */
    kins->tk.a1 = 0.150;
    kins->tk.a2 = 0.770;
    kins->tk.a3 = 0.100;
    kins->tk.d2 = 0;
    kins->tk.d3 = 0;
    kins->tk.d4 = 0.740;
    kins->tk.iflags = 0;
  }

  return GO_RESULT_OK;
}

go_result fanuc_kin_get_parameters(fanuc_kin_struct *kins, go_link *params, go_integer num)
{
  if (num < 7) return GO_RESULT_ERROR;

  /* stuff zeros into the last joint, whose parameters we ignore */
  params[6].type = GO_LINK_DH;
  params[6].quantity = GO_QUANTITY_LENGTH;
  params[6].u.dh.a = 0;
  params[6].u.dh.alpha = 0;
  params[6].u.dh.d = 0;
  params[6].u.dh.theta = 0;

  return three21_kin_get_parameters(&kins->tk, params, 6);
}

/*
  Converts Fanuc motors to Puma-convention joints, so that the
  joints can be passed into the 3-2-1 forward kinematics to
  get the Cartesian position.

  Gearing equations from motors to Fanuc joints:

  J1 = M1
  J2 = M2 - 90
  J3 = M3 + M2
  J4 = M4
  J5 = M5
  J6 = M6

  Mapping from Fanuc joints to the Puma joint convention:

  P1 = J1 = M1
  P2 = J2 = M2 - 90
  P3 = -J3 = -(M3 + M2)
  P4 = -J4 = -M4
  P5 = -J5 = -M5
  P6 = -J6 = -M6
*/
static void fanuc_kin_gearing_fwd(fanuc_kin_struct *kins,
				  const go_real *motors,
				  go_real *joints)
{
  joints[0] = motors[0];
  joints[1] = motors[1] - GO_PI_2;
  joints[2] = -(motors[2] + motors[1]);
  joints[3] = -motors[3];
  joints[4] = -motors[4];
  joints[5] = -motors[5];

  return;
}

/*
  Converts Puma-convention joints to Fanuc motors, so that the joints
  coming from the 3-2-1 inverse kinematics will match Fanuc motors.

  Mapping from the Puma joint convention to Fanuc joints:

  J1 = P1
  J2 = P2
  J3 = -P3
  J4 = -P4
  J5 = -P5
  J6 = -P6

  Gearing equations from Fanuc joints to motors:

  M1 = J1 = P1
  M2 = J2 + 90 = P2 + 90
  M3 = J3 - M2 = -P3 - M2 = -P3 - P2 - 90
  M4 = J4 = -P4
  M5 = J5 = -P5
  M6 = J6 = =P6
*/
static void fanuc_kin_gearing_inv(fanuc_kin_struct *kins,
				  const go_real *joints,
				  go_real *motors)
{
  motors[0] = joints[0];
  motors[1] = joints[1] + GO_PI_2;
  motors[2] = -joints[2] - motors[1];
  motors[3] = -joints[3];
  motors[4] = -joints[4];
  motors[5] = -joints[5];

  return;
}

/*
  The forward kinematic function takes the arm joint values and
  calculates the position of the robot wrt its base, adds the wrist
  plate offset, then adds the gantry to the robot X to get the complete X.
*/
go_result fanuc_kin_fwd(fanuc_kin_struct *kins,
			const go_real *motors,
			go_pose *pos)
{
  go_pose wristoff = {{0,0,WRIST_OFFSET},{1,0,0,0}};
  go_real joints[6];
  go_result retval;

  fanuc_kin_gearing_fwd(kins, motors, joints);

  retval = three21_kin_fwd(&kins->tk, joints, pos);
  if (GO_RESULT_OK != retval) return retval;

  /* The end frame {7} is pushed out 100 mm */
  go_pose_pose_mult(pos, &wristoff, pos);

  /* almost - now we need to add the gantry */
  pos->tran.x += motors[6];

  return GO_RESULT_OK;
}

/*
  The Fanuc {7} frame follows the Puma {6} convention (Z away from
  plate, X away from base) but is moved out away from plate 100 mm.

  The inverse kinematics function takes the Cartesian position,
  subtracts off the X gantry value, moves in 100 mm in Z and computes the
  robot motors.  The last motor position 'motors[FANUC_KIN_NUM_JOINTS]'
  must be set by the caller to be the gantry position. It will be
  referenced here, but left alone.
*/
go_result fanuc_kin_inv(fanuc_kin_struct *kins,
			const go_pose *pos,
			go_real *motors)
{
  go_real joints[6];
  go_pose adj;
  go_pose wristoff = {{0,0,-WRIST_OFFSET},{1,0,0,0}};
  go_integer t;
  go_result retval;

  adj = *pos;
  /* take off the gantry X position */
  adj.tran.x -= motors[6];

  /* take off the wrist offset */
  go_pose_pose_mult(&adj, &wristoff, &adj);

  /* set joints as initial estimate, although we don't need it since
     the kinematics are closed-form */
  fanuc_kin_gearing_fwd(kins, motors, joints);

  retval = three21_kin_inv(&kins->tk, &adj, joints);
  if (GO_RESULT_OK != retval) return retval;

  fanuc_kin_gearing_inv(kins, joints, motors);

  /* normalize motor angles to range [-180..180] as Fanuc does */
  for (t = 0; t < 6; t++) {
    if (motors[t] < -GO_PI) motors[t] += GO_2_PI;
    else if (motors[t] > GO_PI) motors[t] -= GO_2_PI;
  }

  return GO_RESULT_OK;
}

/* the speed counterpart to the forward gearing function */
static void fanuc_kin_gearing_vel_fwd(fanuc_kin_struct *kins,
				      const go_real *motorvels,
				      go_real *jointvels)
{
  jointvels[0] = motorvels[0];
  /* the 90 degree joint 2 offset doesn't affect speeds */
  jointvels[1] = motorvels[1];
  /* since P3 = -(M2 + M3), P3 dot = -(M2 dot + M3 dot) */
  jointvels[2] = -(motorvels[1] + motorvels[2]);
  jointvels[3] = -motorvels[3];
  jointvels[4] = -motorvels[4];
  jointvels[5] = -motorvels[5];

  return;
}

/* the speed counterpart to the inverse gearing function */
static void fanuc_kin_gearing_vel_inv(fanuc_kin_struct *kins,
				      const go_real *jointvels,
				      go_real *motorvels)
{
  motorvels[0] = jointvels[0];
  /* the 90 degree offset doesn't affect speeds */
  motorvels[1] = jointvels[1];
  /* M3 = -P3-P2-90, so M3 dot = -P3 dot - P2 dot */
  motorvels[2] = -jointvels[2] - jointvels[1];
  motorvels[3] = -jointvels[3];
  motorvels[4] = -jointvels[4];
  motorvels[5] = -jointvels[5];

  return;
}

go_result fanuc_kin_jac_fwd(fanuc_kin_struct *kins,
			    const go_real *motors,
			    const go_real *motorvels,
			    const go_pose *pos,
			    go_vel *vw)
{
  go_real joints[FANUC_KIN_NUM_JOINTS + 1];
  go_real jointvels[FANUC_KIN_NUM_JOINTS + 1];
  go_result retval;

  /* convert the motor positions and speeds to joint positions and speeds */
  fanuc_kin_gearing_fwd(kins, motors, joints);
  fanuc_kin_gearing_vel_fwd(kins, motorvels, jointvels);

  /* run 3-2-1 kins with Puma convention joints and vels */
  retval = three21_kin_jac_fwd(&kins->tk, joints, jointvels, pos, vw);
  if (GO_RESULT_OK != retval) return retval;

  /* add on the gantry motor speed */
  vw->v.x += motorvels[6];

  return GO_RESULT_OK;
}

go_result fanuc_kin_jac_inv(fanuc_kin_struct *kins,
			    const go_pose *pos,
			    const go_vel *vw,
			    const go_real *motors,
			    go_real *motorvels)
{
  go_real joints[FANUC_KIN_NUM_JOINTS + 1];
  go_real jointvels[FANUC_KIN_NUM_JOINTS + 1];
  go_result retval;

  /*
    We need to apportion any desired Cartesian X speed into the part
    supplied by the robot and the part supplied by the gantry. 
    Our choice is to 'lock' the gantry during speed moves, and supply
    all motion by only the arm. So, we don't take off any speed from
    the input to the robot arm inverse Jacobian.
  */

  /*
    Convert the motor position to joint positions, which are needed in
    the input part. Calling the forward gearing here in the inverse
    Jacobian looks wrong, but it's OK.
  */
  fanuc_kin_gearing_fwd(kins, motors, joints); 
  
  /* now call the inverse Jacobian with the true joints */
  retval = three21_kin_jac_inv(&kins->tk, pos, vw, joints, jointvels);
  if (GO_RESULT_OK != retval) return retval;

  /* now convert the joint vels into motor vels */
  fanuc_kin_gearing_vel_inv(kins, jointvels, motorvels);

  /* make sure the gantry speed is zero */
  motorvels[6] = 0;

  return GO_RESULT_OK;
}
