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
  FIXME-- add these:

  active id (also in Go Motion)
  max tv, ta, tj, rv, ra, rj
*/

/*!
  \defgroup KINEMATICS How Kinematics Functions Work

  The kinematics functions relate the joints to the "kinematic control
  point" KCP. The forward kinematics calculate the KCP from the joint
  values, and the inverse kinematics calculate the joint values from
  the KCP. 

  Often one would like to add another transform from the KCP to an
  application "end control point" ECP. This is commonly called the
  "tool transform," with the KCP called the "wrist frame" and the ECP
  called the "tool frame". As tools are placed on the manipulator,
  the tool transform changes.

  <small><sup>0</sup><sub>K</sub></small>T is the KCP. This is the
  frame of the kinematics functions, where the 0 means the world
  coordinate system.

  <small><sup>0</sup><sub>E</sub></small>T is the ECP. This is the
  frame of motion control, program coordinates, position limits, home
  position and the display.

  <small><sup>K</sup><sub>E</sub></small>T is the tool transform.
  The tool transform is specified as the position and orientation
  of the tool's end control point, ECP, with respect to the kinematic
  control point, KCP,

  <small><sup>K</sup><sub>E</sub></small>T = | <small><sup>K</sup><sub>E</sub></small>R <sup><small>K</small></sup>P<sub><small>Eorg</small></sub> |

  i.e., the position and orientation of the tool tip expressed with
  respect to the kinematic control point (aka wrist frame).

  To go from the ECP to the KCP, we postmultiply the ECP by the
  inverse tool transform to get the KCP:

  <small><sup>0</sup><sub>K</sub></small>T = <small><sup>0</sup><sub>E</sub></small>T * <small><sup>E</sup><sub>K</sub></small>T

  that is, 
  
  ECP * inverse tool transform = KCP
  
  or in code as go_pose_pose_mult(&ECP, &tool_transform_inv, &KCP).

  Results from the forward kinematics functions are in the KCP, and
  must be transformed into the ECP when sent out as status. To go from the KCP to the ECP, we postmultiply the KCP by the tool transform to get the ECP:

  <small><sup>0</sup><sub>E</sub></small>T = <small><sup>0</sup><sub>K</sub></small>T * <small><sup>K</sup><sub>E</sub></small>T

  that is,

  KCP * tool transform = ECP

  or in code as go_pose_pose_mult(&KCP, &tool_transform, &ECP).

  Changing the tool transform is tricky. Even the motion queue is
  empty and the controller is holding the ECP constant, changing the
  tool transform will cause a jump in the KCP and a corresponding jump
  in the joint values coming out of the inverse kinematics. To handle
  this, we need to change the ECP when changing the tool transform:

  <small><sup>0</sup><sub>K</sub></small>T * <small><sup>K</sup><sub>Enew</sub></small>T = <small><sup>0</sup><sub>Enew</sub></small>T, KCP * new tool transform = new ECP

  When the tool is changed, you would see a jump in the displayed
  position, but no jump in actual position.

  To avoid inconsistencies between points on the motion queue in
  various ECPs, the tool transform can only be changed when the
  motion queue is empty. The configuration state table for changing
  the tool transform ensures this, then updates the queue position
  with the new ECP.

  The ECP is in the traj status buffer as 'cmd_position'.

  The KCP is in the traj status buffer as 'kcp'.

  The tool transform and inverse are in the traj settings buffer
  as 'tool_transform' and 'tool_transform_inv'.

  To convert from a pose A in the ECP to the pose in the KCP, premultiply
  by the tool transform:

  <sup><small>K</small></sup>A = <small><sup>K</sup><sub>E</sub></small>T * <sup><small>E</small></sup>A
*/

/*
  Note to coders using Emacs: you can use the shorthand TX(sup,sub,T)
  to denote transforms, then run this macro to convert them to the
  HTML superscript/subscript form: 

  (defalias 'TX (read-kbd-macro
"C-s TX( RET 3*<backspace> <small> <sup> C-s , RET <backspace> </sup> <sub> C-s , RET <backspace> </sub> </small> C-s ) RET <backspace>"))
*/

#include <stddef.h>		/* NULL */
#include <math.h>		/* fabs */
#include <rtapi.h>
#include "go.h"
#include "gorcs.h"
#include "gokin.h"		
#include "golog.h"		/* go_log_entry,add, ... */
#include "goio.h"		/* go_io_struct */
#include "trajintf.h"
#include "servointf.h"		/* servo_comm, servo_sem */

#define BN "trajloop"

/*
  Testing-- define USE_XINV to enable external Cartesian reference tracking
*/
#define USE_XINV

/*
  Testing-- define USE_ACTUAL to use actual joint values when calculating
  the next servo setpoint increment for teleoperation
*/
#undef USE_ACTUAL

#define DEFAULT_CYCLE_TIME 0.1

static go_pose DEFAULT_POSITION = {{0,0,0},{1,0,0,0}};
static go_pose DEFAULT_HOME = {{0,0,0},{1,0,0,0}};
#define DEFAULT_JOINT 0.0

traj_comm_struct * global_traj_comm_ptr = NULL;

static go_real traj_timestamp(void)
{
  rtapi_integer secs, nsecs;

  if (RTAPI_OK == rtapi_clock_get_time(&secs, &nsecs)) {
    return ((go_real) secs) + ((go_real) nsecs) * 1.0e-9;
  }

  return 0.0;
}

#define CMD_PRINT_1(x) if (set->debug & DEBUG_CMD) rtapi_print(x)
#define CMD_PRINT_2(x,y) if (set->debug & DEBUG_CMD) rtapi_print(x, y)
#define CMD_PRINT_3(x,y,z) if (set->debug & DEBUG_CMD) rtapi_print(x, y, z)
#define CMD_PRINT_4(x,y,z,u) if (set->debug & DEBUG_CMD) rtapi_print(x, y, z, u)

#define CFG_PRINT_1(x) if (set->debug & DEBUG_CFG) rtapi_print(x)
#define CFG_PRINT_2(x,y) if (set->debug & DEBUG_CFG) rtapi_print(x, y)
#define CFG_PRINT_3(x,y,z) if (set->debug & DEBUG_CFG) rtapi_print(x, y, z)
#define CFG_PRINT_4(x,y,z,u) if (set->debug & DEBUG_CFG) rtapi_print(x, y, z, u)

static void shift_joints(go_real *joints, go_real *last, go_integer num, void *kins)
{
  go_link link[SERVO_NUM];

  go_kin_get_parameters(kins, link, num);

  while (--num >= 0) {
    if (GO_QUANTITY_ANGLE != link[num].quantity) {
      continue;			/* no need to shift linear joints */
    }
    while (joints[num] - last[num] >= GO_PI) joints[num] -= GO_2_PI;
    while (joints[num] - last[num] <= -GO_PI) joints[num] += GO_2_PI;
  }
}

static void write_servo_cmd(servo_cmd_struct * servo_cmd, go_integer servo_num)
{
  servo_cmd[servo_num].tail = ++servo_cmd[servo_num].head;
  servo_cmd[servo_num].serial_number++;
  global_servo_comm_ptr[servo_num].servo_cmd = servo_cmd[servo_num];
}

static void write_servo_cfg(servo_cfg_struct * servo_cfg, go_integer servo_num)
{
  servo_cfg[servo_num].tail = ++servo_cfg[servo_num].head;
  servo_cfg[servo_num].serial_number++;
  global_servo_comm_ptr[servo_num].servo_cfg = servo_cfg[servo_num];
}

static void do_cmd_nop(traj_stat_struct * stat, traj_set_struct * set)
{
  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_1("traj: cmd nop\n");
    go_state_new(stat);
    go_status_next(stat, GO_RCS_STATUS_DONE);
    go_state_next(stat, GO_RCS_STATE_S0);
  } else {			/* S0 */
    go_state_default(stat);
  }
}

static void do_cmd_init(traj_stat_struct * stat, traj_set_struct * set, servo_cmd_struct * servo_cmd, servo_stat_struct * servo_stat, go_motion_queue * queue)
{
  go_position position;
  go_integer servo_num;
  go_integer num_done;
  go_flag is_error;

  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_1("traj: cmd init\n");
    go_state_new(stat);
    for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
      servo_cmd[servo_num].type = SERVO_CMD_INIT_TYPE;
      write_servo_cmd(servo_cmd, servo_num);
    }
    go_status_next(stat, GO_RCS_STATUS_EXEC);
    go_state_next(stat, GO_RCS_STATE_S1);
  } else if (go_state_match(stat, GO_RCS_STATE_S1)) {
    num_done = 0;
    is_error = 0;
    for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
      if (servo_stat[servo_num].command_type == SERVO_CMD_INIT_TYPE &&
	  servo_stat[servo_num].echo_serial_number == servo_cmd[servo_num].serial_number) {
	if (servo_stat[servo_num].status == GO_RCS_STATUS_DONE) {
	  num_done++;
	} else if (servo_stat[servo_num].status == GO_RCS_STATUS_ERROR) {
	  is_error = 1;
	  break;
	} /* else still executing */
      }	/* else didn't get there yet */
    } /* for (servo_num) */
    if (num_done == set->joint_num) {
      /* now put us in joint mode, at 'here' */
      stat->frame = TRAJ_JOINT_FRAME;
      go_motion_queue_reset(queue);
      go_motion_queue_set_type(queue, GO_MOTION_JOINT);
      for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
	position.u.joint[servo_num] = stat->joints_act[servo_num];
      }
      go_motion_queue_set_here(queue, &position);
      stat->admin_state = GO_RCS_ADMIN_STATE_INITIALIZED;
      go_status_next(stat, GO_RCS_STATUS_DONE);
      go_state_next(stat, GO_RCS_STATE_S0);
    } else if (is_error) {
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
    } /* else still working on it */
  } else {			/* S0 */
    go_state_default(stat);
  }
}

static void do_cmd_abort(traj_stat_struct * stat, traj_set_struct * set, servo_cmd_struct * servo_cmd, servo_stat_struct * servo_stat)
{
  go_integer servo_num;
  go_integer num_done;
  go_flag is_error;

  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_1("traj: cmd abort\n");
    go_state_new(stat);
    for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
      servo_cmd[servo_num].type = SERVO_CMD_ABORT_TYPE;
      write_servo_cmd(servo_cmd, servo_num);
    }
    go_status_next(stat, GO_RCS_STATUS_EXEC);
    go_state_next(stat, GO_RCS_STATE_S1);
  } else if (go_state_match(stat, GO_RCS_STATE_S1)) {
    num_done = 0;
    is_error = 0;
    for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
      if (servo_stat[servo_num].command_type == SERVO_CMD_ABORT_TYPE &&
	  servo_stat[servo_num].echo_serial_number == servo_cmd[servo_num].serial_number) {
	if (servo_stat[servo_num].status == GO_RCS_STATUS_DONE) {
	  num_done++;
	} else if (servo_stat[servo_num].status == GO_RCS_STATUS_ERROR) {
	  is_error = 1;
	  break;
	} /* else still executing */
      }	/* else didn't get there yet */
    } /* for (servo_num) */
    if (num_done == set->joint_num) {
      stat->admin_state = GO_RCS_ADMIN_STATE_UNINITIALIZED;
      go_status_next(stat, GO_RCS_STATUS_DONE);
      go_state_next(stat, GO_RCS_STATE_S0);
    } else if (is_error) {
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
    } /* else still working on it */
  } else {			/* S0 */
    go_state_default(stat);
  }
}

static void do_cmd_halt(traj_stat_struct * stat, traj_set_struct * set, servo_cmd_struct * servo_cmd, servo_stat_struct * servo_stat)
{
  go_integer servo_num;
  go_integer num_done;
  go_flag is_error;

  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_1("traj: cmd halt\n");
    go_state_new(stat);
    for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
      servo_cmd[servo_num].type = SERVO_CMD_HALT_TYPE;
      write_servo_cmd(servo_cmd, servo_num);
    }
    go_status_next(stat, GO_RCS_STATUS_EXEC);
    go_state_next(stat, GO_RCS_STATE_S1);
  } else if (go_state_match(stat, GO_RCS_STATE_S1)) {
    num_done = 0;
    is_error = 0;
    for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
      if (servo_stat[servo_num].command_type == SERVO_CMD_HALT_TYPE &&
	  servo_stat[servo_num].echo_serial_number == servo_cmd[servo_num].serial_number) {
	if (servo_stat[servo_num].status == GO_RCS_STATUS_DONE) {
	  num_done++;
	} else if (servo_stat[servo_num].status == GO_RCS_STATUS_ERROR) {
	  is_error = 1;
	  break;
	} /* else still executing */
      }	/* else didn't get there yet */
    } /* for (servo_num) */
    if (num_done == set->joint_num) {
      stat->admin_state = GO_RCS_ADMIN_STATE_UNINITIALIZED;
      go_status_next(stat, GO_RCS_STATUS_DONE);
      go_state_next(stat, GO_RCS_STATE_S0);
    } else if (is_error) {
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
    } /* else still working on it */
  } else {			/* S0 */
    go_state_default(stat);
  }
}

static void do_cmd_shutdown(traj_stat_struct * stat, traj_set_struct * set, servo_cmd_struct * servo_cmd, servo_stat_struct * servo_stat)
{
  go_integer servo_num;
  go_integer num_done;
  go_flag is_error;

  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_1("traj: cmd shutdown\n");
    go_state_new(stat);
    if (stat->admin_state == GO_RCS_ADMIN_STATE_UNINITIALIZED) {
      for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
	servo_cmd[servo_num].type = SERVO_CMD_SHUTDOWN_TYPE;
	write_servo_cmd(servo_cmd, servo_num);
      }
      go_status_next(stat, GO_RCS_STATUS_EXEC);
      go_state_next(stat, GO_RCS_STATE_S1);
    } else {
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
    }
  } else if (go_state_match(stat, GO_RCS_STATE_S1)) {
    num_done = 0;
    is_error = 0;
    for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
      if (servo_stat[servo_num].command_type == SERVO_CMD_SHUTDOWN_TYPE &&
	  servo_stat[servo_num].echo_serial_number == servo_cmd[servo_num].serial_number) {
	if (servo_stat[servo_num].status == GO_RCS_STATUS_DONE) {
	  num_done++;
	} else if (servo_stat[servo_num].status == GO_RCS_STATUS_ERROR) {
	  is_error = 1;
	  break;
	} /* else still executing */
      }	/* else didn't get there yet */
    } /* for (servo_num) */
    if (num_done == set->joint_num) {
      stat->admin_state = GO_RCS_ADMIN_STATE_SHUT_DOWN;
      go_status_next(stat, GO_RCS_STATUS_DONE);
      go_state_next(stat, GO_RCS_STATE_S0);
    } else if (is_error) {
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
    } /* else still working on it */
  } else {			/* S0 */
    go_state_default(stat);
    rtapi_exit();
  }
}

static go_pose walk_in(traj_stat_struct * stat, traj_set_struct * set, traj_ref_struct * ref)
{
  go_pose curinv, curinvinv, del;
  go_cart uvec;
  go_quat uquat;
  go_real mag;
  go_real tincr, tdel;
  go_real rincr, rdel;
#define WALK_IN_SCALE 0.1
  tincr = WALK_IN_SCALE * set->max_tvel * set->cycle_time;
  rincr = WALK_IN_SCALE * set->max_rvel * set->cycle_time;

  curinv = stat->xinv;
  go_pose_inv(&curinv, &curinvinv);
  go_pose_pose_mult(&curinvinv, &ref->xinv, &del);

  if (GO_RESULT_OK != go_cart_unit(&del.tran, &uvec)) {
    del.tran.x = del.tran.y = del.tran.z = 0.0;
  } else {
    (void) go_cart_mag(&del.tran, &mag);
    if (mag > tincr) tdel = tincr;
    else tdel = mag;
    go_cart_scale_mult(&uvec, tdel, &del.tran);
  }

  if (GO_RESULT_OK != go_quat_unit(&del.rot, &uquat)) {
    del.rot.s = 1.0, del.rot.x = del.rot.y = del.rot.z = 0.0;
  } else {
    (void) go_quat_mag(&del.rot, &mag);
    if (mag > rincr) rdel = rincr;
    else rdel = mag;
    go_quat_scale_mult(&uquat, rdel, &del.rot);
  }

  (void) go_pose_pose_mult(&curinv, &del, &curinv);

  return curinv;
}

static void do_cmd_stop(traj_stat_struct * stat, traj_set_struct * set, traj_ref_struct * ref, servo_cmd_struct * servo_cmd, void * kinematics, go_motion_queue * queue)
{
  go_position ecp;
  go_pose kcp;
  go_real joints[SERVO_NUM];
  go_integer servo_num;

  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_1("traj: cmd stop\n");
    go_state_new(stat);
    if (stat->admin_state != GO_RCS_ADMIN_STATE_INITIALIZED) {
      stat->inpos = 1;
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
    } else {
      if (GO_RESULT_OK != go_motion_queue_stop(queue)) {
	rtapi_print("trajloop: can't stop move\n");
	stat->inpos = 1;
	go_status_next(stat, GO_RCS_STATUS_ERROR);
	go_state_next(stat, GO_RCS_STATE_S0);
      } else {
	stat->inpos = 0;
	go_status_next(stat, GO_RCS_STATUS_EXEC);
	go_state_next(stat, GO_RCS_STATE_S1);
      }
    }
  }

  if (go_state_match(stat, GO_RCS_STATE_S1)) {
    if (GO_RESULT_OK != go_motion_queue_interp(queue, &ecp)) {
      rtapi_print("trajloop: can't interp\n");
      stat->inpos = 1;
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
    } else {
      if (go_motion_queue_is_empty(queue)) {
	stat->inpos = 1;
	go_status_next(stat, GO_RCS_STATUS_DONE);
      }
      /* even if we're done, drop through to here so that we continue
	 to send setpoints to the servos */
      if (queue->type == GO_MOTION_WORLD) {
	stat->ecp = ecp.u.pose;
#ifdef USE_XINV
	stat->xinv = walk_in(stat, set, ref);
	go_pose_pose_mult(&ecp.u.pose, &stat->xinv, &ecp.u.pose);
#endif
	/* convert from ECP to KCP to before using the kinematics */
	go_pose_pose_mult(&ecp.u.pose, &set->tool_transform_inv, &kcp);
	for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
	  joints[servo_num] = stat->joints[servo_num]; /* seed the estimate */
	}
	if (GO_RESULT_OK != go_kin_inv(kinematics, &kcp, joints)) {
	  rtapi_print("trajloop: do_cmd_stop: can't invert\n");
	  go_status_next(stat, GO_RCS_STATUS_ERROR);
	  go_state_next(stat, GO_RCS_STATE_S0);
	} else {
	  /* shift joints to nearest revolution */
	  shift_joints(joints, stat->joints, set->joint_num, kinematics);
	  for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
	    stat->joints[servo_num] = joints[servo_num];
	    servo_cmd[servo_num].type = SERVO_CMD_SERVO_TYPE;
	    servo_cmd[servo_num].u.servo.setpoint = 
	      joints[servo_num] + stat->joint_offsets[servo_num];
	    servo_cmd[servo_num].u.servo.home = 0;
	    write_servo_cmd(servo_cmd, servo_num);
	  }
	}
      } else if (queue->type == GO_MOTION_JOINT) {
	for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
	  stat->joints[servo_num] = ecp.u.joint[servo_num];
	  servo_cmd[servo_num].type = SERVO_CMD_SERVO_TYPE;
	  servo_cmd[servo_num].u.servo.setpoint = 
	    ecp.u.joint[servo_num] + stat->joint_offsets[servo_num];
	  servo_cmd[servo_num].u.servo.home = 0;
	  write_servo_cmd(servo_cmd, servo_num);
	}
      } else {
	for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
	  stat->joints[servo_num] = ecp.u.joint[servo_num];
	  servo_cmd[servo_num].type = SERVO_CMD_SERVO_TYPE;
	  servo_cmd[servo_num].u.servo.setpoint = 
	    ecp.u.joint[servo_num];
	  servo_cmd[servo_num].u.servo.home = 0;
	  write_servo_cmd(servo_cmd, servo_num);
	}
      }
    }
  } else {			/* S0 */
    go_state_default(stat);
  }
}

static go_flag any_need_home_clear(traj_cmd_struct * cmd, traj_set_struct * set, servo_stat_struct * servo_stat)
{
  go_integer t;

  for (t = 0; t < set->joint_num; t++) {
    if (cmd->u.move_ujoint.home[t] && servo_stat[t].homing) return 1;
  }

  return 0;
}

static go_flag all_home_clear(traj_set_struct * set, servo_stat_struct * servo_stat)
{
  go_integer t;

  for (t = 0; t < set->joint_num; t++) {
    if (servo_stat[t].homing) {
      return 0;
    }
  }

  return 1;
}

static void copy_joints(go_real * dst, go_real * src, go_integer howmany)
{
  go_integer servo_num;

  if (howmany > SERVO_NUM) howmany = SERVO_NUM;

  for (servo_num = 0; servo_num < howmany; servo_num++) {
    dst[servo_num] = src[servo_num];
  }

  return;
}

static void do_cmd_here(traj_cmd_struct * cmd, traj_stat_struct * stat, traj_set_struct * set, servo_cmd_struct * servo_cmd, servo_stat_struct * servo_stat, servo_cfg_struct * servo_cfg, servo_set_struct * servo_set, void * kinematics, go_motion_queue * queue)
{
  go_real joints[SERVO_NUM];
  go_position position;
  go_integer servo_num;
  go_integer joints_done;
  go_result retval;

  /* NEW_COMMAND */
  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_1("traj: cmd here\n");
    go_state_new(stat);
    if (stat->admin_state == GO_RCS_ADMIN_STATE_INITIALIZED) {
      stat->frame = TRAJ_JOINT_FRAME;
      stat->inpos = 0;
      if (go_motion_queue_get_type(queue) != GO_MOTION_UJOINT) {
	go_motion_queue_reset(queue);
	go_motion_queue_set_type(queue, GO_MOTION_UJOINT);
	for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
	  /* set 'here' to be in non-offset space */
	  position.u.joint[servo_num] = servo_stat[servo_num].input;
	  /* set 'joints' to be actual joints */
	  stat->joints[servo_num] = stat->joints_act[servo_num];
	}
	go_motion_queue_set_here(queue, &position);
      }
      go_motion_queue_set_id(queue, 0);
      go_status_next(stat, GO_RCS_STATUS_EXEC);
      go_state_next(stat, GO_RCS_STATE_S1);
    } else {
      stat->inpos = 1;
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
      return;
    }
  }
  /* drop through to GO_RCS_STATE_S1 */

  /* FIXME-- this is popular enough to merit factoring out */
#define HOLD_POSITION(HOME) \
  for (servo_num = 0; servo_num < set->joint_num; servo_num++) {	    \
    servo_cmd[servo_num].type = SERVO_CMD_SERVO_TYPE;			    \
    servo_cmd[servo_num].u.servo.setpoint = servo_stat[servo_num].input;    \
    servo_cmd[servo_num].u.servo.home = HOME;				    \
    write_servo_cmd(servo_cmd, servo_num);				    \
  }

  if (go_state_match(stat, GO_RCS_STATE_S1)) {
    /* compute the joint values associated with 'here' */
    retval = go_kin_inv(kinematics, &cmd->u.here.here, joints);
    if (GO_RESULT_OK != retval) {
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
    } else {
      /* shift joints to nearest revolution */
      shift_joints(joints, stat->joints, set->joint_num, kinematics);
      /* tell all the servos to have a new home position */
      for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
	servo_cfg[servo_num].type = SERVO_CFG_HOME_TYPE;
	servo_cfg[servo_num].u.home.home = joints[servo_num];
	write_servo_cfg(servo_cfg, servo_num);
      }
      go_state_next(stat, GO_RCS_STATE_S2);
    }
    HOLD_POSITION(0);
  } else if (go_state_match(stat, GO_RCS_STATE_S2)) {
    /* wait for all joints to register a new home position */
    joints_done = 0;
    for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
      if (servo_set[servo_num].echo_serial_number ==
	  servo_cfg[servo_num].serial_number) {
	if (servo_set[servo_num].status == GO_RCS_STATUS_ERROR) {
	  go_status_next(stat, GO_RCS_STATUS_ERROR);
	  go_state_next(stat, GO_RCS_STATE_S0);
	  break;
	} else if (servo_set[servo_num].status == GO_RCS_STATUS_DONE) {
	  joints_done++;
	  if (joints_done == set->joint_num) {
	    go_state_next(stat, GO_RCS_STATE_S3);
	    break;
	  }
	} else break;		/* this one is still executing */
      } else break;	    /* else command hasn't gotten there yet */
    }		     
    HOLD_POSITION(0);
  } else if (go_state_match(stat, GO_RCS_STATE_S3)) {
    /* all the joints have set up their new home positions, so
       tell them to home */
    HOLD_POSITION(1);
  } else {			
    /* S0 */
    go_state_default(stat);
    HOLD_POSITION(0);
  }

#undef HOLD_POSITION
  return;
}

static void do_cmd_move_ujoint(traj_cmd_struct * cmd, traj_stat_struct * stat, traj_set_struct * set, servo_cmd_struct * servo_cmd, servo_stat_struct * servo_stat, go_motion_queue * queue)
{
  go_integer servo_num;
  go_motion_spec gms;
  go_position position;

  /* NEW_COMMAND */
  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_1("traj: cmd move ujoint\n");
    go_state_new(stat);
    if (stat->admin_state == GO_RCS_ADMIN_STATE_INITIALIZED) {
      stat->frame = TRAJ_JOINT_FRAME;
      stat->inpos = 0;
      if (go_motion_queue_get_type(queue) != GO_MOTION_UJOINT) {
	go_motion_queue_reset(queue);
	go_motion_queue_set_type(queue, GO_MOTION_UJOINT);
	for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
	  /* set 'here' to be in non-offset space */
	  position.u.joint[servo_num] = servo_stat[servo_num].input;
	  /* set 'joints' to be actual joints */
	  stat->joints[servo_num] = stat->joints_act[servo_num];
	}
	go_motion_queue_set_here(queue, &position);
      }
      /* make sure this move registers as new */
      go_motion_queue_set_id(queue, cmd->u.move_ujoint.id - 1);
      go_status_next(stat, GO_RCS_STATUS_EXEC);
      go_state_next(stat, GO_RCS_STATE_S1);
    } else {
      stat->inpos = 1;
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
      return;
    }
  }
  /* drop through to GO_RCS_STATE_S1 */

  if (go_state_match(stat, GO_RCS_STATE_S1)) {
    if (cmd->u.move_ujoint.id !=
	go_motion_queue_last_id(queue)) {
      /* a new set to append */
      go_motion_spec_init(&gms);
      go_motion_spec_set_type(&gms, GO_MOTION_UJOINT);
      go_motion_spec_set_id(&gms, cmd->u.move_ujoint.id);
      for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
	/* FIXME-- could use a go_motion_spec_set_end_joint, or
	   something like that, for completeness */
	gms.end.u.joint[servo_num] = cmd->u.move_ujoint.d[servo_num];
	go_motion_spec_set_jpar(&gms, servo_num,
				cmd->u.move_ujoint.v[servo_num],
				cmd->u.move_ujoint.a[servo_num],
				cmd->u.move_ujoint.j[servo_num]);
      }
      if (GO_RESULT_OK != go_motion_queue_append(queue, &gms)) {
	rtapi_print("trajloop: can't append joint move\n");
	stat->inpos = 1;
	go_status_next(stat, GO_RCS_STATUS_ERROR);
	go_state_next(stat, GO_RCS_STATE_S0);
	return;
      }
      if (any_need_home_clear(cmd, set, servo_stat)) {
	/* drop home request low and wait for acknowledge, holding position */
	for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
	  servo_cmd[servo_num].type = SERVO_CMD_SERVO_TYPE;
	  servo_cmd[servo_num].u.servo.setpoint = servo_stat[servo_num].input;
	  servo_cmd[servo_num].u.servo.home = 0;
	  write_servo_cmd(servo_cmd, servo_num);
	}
	go_state_next(stat, GO_RCS_STATE_S2);
      } else {
	/* none need clearing, so run one interp to keep in step
	   and we'll go to steady state next cycle */
	if (GO_RESULT_OK != go_motion_queue_interp(queue, &position)) {
	  rtapi_print("trajloop: can't interp\n");
	  stat->inpos = 1;
	  go_status_next(stat, GO_RCS_STATUS_ERROR);
	  go_state_next(stat, GO_RCS_STATE_S0);
	  return;
	} else {
	  for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
	    stat->joints[servo_num] = position.u.joint[servo_num];
	    servo_cmd[servo_num].type = SERVO_CMD_SERVO_TYPE;
	    servo_cmd[servo_num].u.servo.setpoint = position.u.joint[servo_num];
	    servo_cmd[servo_num].u.servo.home = cmd->u.move_ujoint.home[servo_num];
	    write_servo_cmd(servo_cmd, servo_num);
	  }
	}
      }
    } else {
      /* This is the steady state, running motions on the queue. 
	 Still in GO_RCS_STATE_S1. */
      if (GO_RESULT_OK != go_motion_queue_interp(queue, &position)) {
	rtapi_print("trajloop: can't interp\n");
	stat->inpos = 1;
	go_status_next(stat, GO_RCS_STATUS_ERROR);
	go_state_next(stat, GO_RCS_STATE_S0);
	return;
      } else {
	stat->inpos = go_motion_queue_is_empty(queue);
	if (stat->inpos) {
	  go_status_next(stat, GO_RCS_STATUS_DONE);
	}
	for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
	  stat->joints[servo_num] = position.u.joint[servo_num];
	  servo_cmd[servo_num].type = SERVO_CMD_SERVO_TYPE;
	  servo_cmd[servo_num].u.servo.setpoint = position.u.joint[servo_num];
	  servo_cmd[servo_num].u.servo.home = cmd->u.move_ujoint.home[servo_num];
	  write_servo_cmd(servo_cmd, servo_num);
	  if (servo_stat[servo_num].homed &&
	      cmd->u.move_ujoint.home[servo_num]) {
	    /* FIXME-- this will stop all of them when any has homed.
	       What we should do is stop the one that is homed. */
	    go_motion_queue_stop(queue);
	  }
	}
      }
    }
  } else if (go_state_match(stat, GO_RCS_STATE_S2)) {
    /* S2 -- waiting for homing clear */
    if (all_home_clear(set, servo_stat)) {
      go_state_next(stat, GO_RCS_STATE_S1);
    }
    /* and hold position in the meantime */
    for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
      servo_cmd[servo_num].type = SERVO_CMD_SERVO_TYPE;
      servo_cmd[servo_num].u.servo.setpoint = servo_stat[servo_num].input;
      servo_cmd[servo_num].u.servo.home = 0;
      write_servo_cmd(servo_cmd, servo_num);
    }
  } else {			
    /* S0 */
    go_state_default(stat);
  }
}

static void do_cmd_move_joint(traj_cmd_struct * cmd, traj_stat_struct * stat, traj_set_struct * set, servo_cmd_struct * servo_cmd, servo_stat_struct * servo_stat, servo_set_struct * servo_set, go_motion_queue * queue)
{
  go_integer servo_num;
  go_motion_spec gms;
  go_position position;

  /* NEW_COMMAND */
  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_1("traj: cmd move joint\n");
    go_state_new(stat);
    if (stat->admin_state == GO_RCS_ADMIN_STATE_INITIALIZED) {
      stat->frame = TRAJ_JOINT_FRAME;
      stat->inpos = 0;
      if (go_motion_queue_get_type(queue) != GO_MOTION_JOINT) {
	go_motion_queue_reset(queue);
	go_motion_queue_set_type(queue, GO_MOTION_JOINT);
	for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
	  /* set 'here' to be in offset space */
	  position.u.joint[servo_num] = stat->joints_act[servo_num];
	  /* set 'joints' to be actual joints */
	  stat->joints[servo_num] = stat->joints_act[servo_num];
	}
	go_motion_queue_set_here(queue, &position);
      }
      /* make sure this move registers as new */
      go_motion_queue_set_id(queue, cmd->u.move_joint.id - 1);
      go_status_next(stat, GO_RCS_STATUS_EXEC);
      go_state_next(stat, GO_RCS_STATE_S1);
    } else {
      stat->inpos = 1;
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
      return;
    }
  }
  /* drop through to S1 */

  if (go_state_match(stat, GO_RCS_STATE_S1)) {
    if (cmd->u.move_joint.id !=
	go_motion_queue_last_id(queue)) {
      /* a new set to append */
      go_motion_spec_init(&gms);
      go_motion_spec_set_id(&gms, cmd->u.move_joint.id);
      go_motion_spec_set_type(&gms, GO_MOTION_JOINT);
      if (cmd->u.move_joint.time > GO_REAL_EPSILON) {
	go_motion_spec_set_time(&gms, cmd->u.move_joint.time);
      }
      for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
	/* clamp joint position to lie within limits, if this
	   joint is homed */
	if (servo_stat[servo_num].homed) {
	  if (cmd->u.move_joint.d[servo_num] > servo_set[servo_num].max_limit) {
	    gms.end.u.joint[servo_num] = servo_set[servo_num].max_limit;
	    CMD_PRINT_4("traj: clamping joint %d down from %f to %f\n", (int) servo_num, (double) cmd->u.move_joint.d[servo_num], (double) servo_set[servo_num].max_limit);
	  } else if (cmd->u.move_joint.d[servo_num] < servo_set[servo_num].min_limit) {
	    gms.end.u.joint[servo_num] = servo_set[servo_num].min_limit;
	    CMD_PRINT_4("traj: clamping joint %d up from %f to %f\n", (int) servo_num, (double) cmd->u.move_joint.d[servo_num], (double) servo_set[servo_num].min_limit);
	  } else {
	    gms.end.u.joint[servo_num] = cmd->u.move_joint.d[servo_num];
	  }
	} else {
	  gms.end.u.joint[servo_num] = cmd->u.move_joint.d[servo_num];
	}
	go_motion_spec_set_jpar(&gms, servo_num,
				cmd->u.move_joint.v[servo_num],
				cmd->u.move_joint.a[servo_num],
				cmd->u.move_joint.j[servo_num]);
      }
      if (GO_RESULT_OK != go_motion_queue_append(queue, &gms)) {
	rtapi_print("trajloop: can't append joint move\n");
	stat->inpos = 1;
	go_status_next(stat, GO_RCS_STATUS_ERROR);
	go_state_next(stat, GO_RCS_STATE_S0);
	return;
      }
    }
    /* This is the steady state, running motions on the queue. */
    if (GO_RESULT_OK != go_motion_queue_interp(queue, &position)) {
      rtapi_print("trajloop: can't interp\n");
      stat->inpos = 1;
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
    } else {
      stat->inpos = go_motion_queue_is_empty(queue);
      if (stat->inpos) {
	go_status_next(stat, GO_RCS_STATUS_DONE);
      }
      for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
	stat->joints[servo_num] = position.u.joint[servo_num];
	servo_cmd[servo_num].type = SERVO_CMD_SERVO_TYPE;
	servo_cmd[servo_num].u.servo.setpoint =
	  position.u.joint[servo_num] + stat->joint_offsets[servo_num];
	servo_cmd[servo_num].u.servo.home = 0;
	write_servo_cmd(servo_cmd, servo_num);
      }
    }
  } else {			
    /* S0 */
    go_state_default(stat);
  }
}

static go_result clamp_pose(go_pose * pose, const go_pose * min, const go_pose * max)
{
  go_rpy inrpy, minrpy, maxrpy;
  go_result retval;

  if (pose->tran.x < min->tran.x) pose->tran.x = min->tran.x;
  else if (pose->tran.x > max->tran.x) pose->tran.x = max->tran.x;
  if (pose->tran.y < min->tran.y) pose->tran.y = min->tran.y;
  else if (pose->tran.y > max->tran.y) pose->tran.y = max->tran.y;
  if (pose->tran.z < min->tran.z) pose->tran.z = min->tran.z;
  else if (pose->tran.z > max->tran.z) pose->tran.z = max->tran.z;

  /* FIXME -- the naive algorithm doesn't work due to RPY values going
   into and out of quaternions not being unique */
  return GO_RESULT_OK;

#if 0
  retval = go_quat_rpy_convert(&pose->rot, &inrpy);
  if (GO_RESULT_OK != retval) return retval;
  retval = go_quat_rpy_convert(&min->rot, &minrpy);
  if (GO_RESULT_OK != retval) return retval;
  retval = go_quat_rpy_convert(&max->rot, &maxrpy);
  if (GO_RESULT_OK != retval) return retval;

  if (inrpy.r < minrpy.r) inrpy.r = minrpy.r;
  else if (inrpy.r > maxrpy.r) inrpy.r = maxrpy.r;
  if (inrpy.p < minrpy.p) inrpy.p = minrpy.p;
  else if (inrpy.p > maxrpy.p) inrpy.p = maxrpy.p;
  if (inrpy.y < minrpy.y) inrpy.y = minrpy.y;
  else if (inrpy.y > maxrpy.y) inrpy.y = maxrpy.y;

  return go_rpy_quat_convert(&inrpy, &pose->rot);
#endif
}

static go_result clamp_vel(const go_pose * pose, go_vel * vel, const go_pose * min, const go_pose * max)
{
  go_rpy inrpy, minrpy, maxrpy;
  go_result retval;

  if (pose->tran.x < min->tran.x && vel->v.x < 0.0) vel->v.x = 0.0;
  else if (pose->tran.x > max->tran.x && vel->v.x > 0.0) vel->v.x = 0.0;
  if (pose->tran.y < min->tran.y && vel->v.y < 0.0) vel->v.y = 0.0;
  else if (pose->tran.y > max->tran.y && vel->v.y > 0.0) vel->v.y = 0.0;
  if (pose->tran.z < min->tran.z && vel->v.z < 0.0) vel->v.z = 0.0;
  else if (pose->tran.z > max->tran.z && vel->v.z > 0.0) vel->v.z = 0.0;

  retval = go_quat_rpy_convert(&pose->rot, &inrpy);
  if (GO_RESULT_OK != retval) return retval;
  retval = go_quat_rpy_convert(&min->rot, &minrpy);
  if (GO_RESULT_OK != retval) return retval;
  retval = go_quat_rpy_convert(&max->rot, &maxrpy);
  if (GO_RESULT_OK != retval) return retval;

  if (inrpy.r < minrpy.r && vel->w.x < 0.0) vel->w.x = 0.0;
  else if (inrpy.r > maxrpy.r && vel->w.x > 0.0) vel->w.x = 0.0;
  if (inrpy.p < minrpy.p && vel->w.y < 0.0) vel->w.y = 0.0;
  else if (inrpy.p > maxrpy.p && vel->w.y > 0.0) vel->w.y = 0.0;
  if (inrpy.y < minrpy.y && vel->w.z < 0.0) vel->w.z = 0.0;
  else if (inrpy.y > maxrpy.y && vel->w.z > 0.0) vel->w.z = 0.0;

  return GO_RESULT_OK;
}

static void do_cmd_move_world_or_tool(go_flag world, traj_cmd_struct * cmd, traj_stat_struct * stat, traj_set_struct * set, traj_ref_struct * ref, servo_cmd_struct * servo_cmd, servo_stat_struct * servo_stat, void * kinematics, go_motion_queue * queue)
{
  go_motion_spec gms;
  go_position ecp;
  go_pose end;
  go_cart center;
  go_cart normal;
  go_real joints[SERVO_NUM];
  go_real tv, ta, tj;
  go_real rv, ra, rj;
  go_real time;
  go_integer id;
  go_integer turns;
  go_integer servo_num;
  go_flag is_circular;

  id = world ? cmd->u.move_world.id : cmd->u.move_tool.id;
  is_circular = (cmd->u.move_tool.type == GO_MOTION_CIRCULAR);

  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_3("traj: cmd move %s (%f ...)\n", world ? "world" : "tool", (double) cmd->u.move_world.end.tran.x);
    go_state_new(stat);
    if (stat->admin_state == GO_RCS_ADMIN_STATE_INITIALIZED &&
	stat->homed) {
      stat->frame = TRAJ_WORLD_FRAME;
      stat->inpos = 0;
      if (go_motion_queue_get_type(queue) != GO_MOTION_WORLD) {
	go_motion_queue_reset(queue);
	go_motion_queue_set_type(queue, GO_MOTION_WORLD);
	/* set the queue 'here' to be the actual ECP */
	ecp.u.pose = stat->ecp_act;
	/* set the commanded ECP to be the actual ECP. This
	   will normally be overwritten below, but it can't hurt
	   to synchronize them now. */
	stat->ecp= stat->ecp_act;
	go_motion_queue_set_here(queue, &ecp);
      }
      /* make sure this move registers as new */
      go_motion_queue_set_id(queue, id - 1);
      go_status_next(stat, GO_RCS_STATUS_EXEC);
      go_state_next(stat, GO_RCS_STATE_S1);
    } else {
      stat->inpos = 1;
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
      return;
    }
  }

  if (go_state_match(stat, GO_RCS_STATE_S1)) {
    if (id != go_motion_queue_last_id(queue)) {
      if (world) {
	time = cmd->u.move_world.time;
	tv = cmd->u.move_world.tv;
	ta = cmd->u.move_world.ta;
	tj = cmd->u.move_world.tj;
	rv = cmd->u.move_world.rv;
	ra = cmd->u.move_world.ra;
	rj = cmd->u.move_world.rj;
	end = cmd->u.move_world.end;
	if (is_circular) {
	  center = cmd->u.move_world.center;
	  normal = cmd->u.move_world.normal;
	  turns = cmd->u.move_world.turns;
	}
      } else {
	time = cmd->u.move_tool.time;
	tv = cmd->u.move_tool.tv;
	ta = cmd->u.move_tool.ta;
	tj = cmd->u.move_tool.tj;
	rv = cmd->u.move_tool.rv;
	ra = cmd->u.move_tool.ra;
	rj = cmd->u.move_tool.rj;
	end = cmd->u.move_tool.end;
	/*
	  The 'end' pose is in the end frame, i.e.,

	  E
	  .end

	  We need to convert this to a pose in the world (0) frame:

	  0    E      0
	  .T *  end =  end
	  E
	*/
	/* get the ECP as the end of the queue */
	go_motion_queue_there(queue, &ecp);
	/* do the pose multiply to get 'end' in the world frame */
	go_pose_pose_mult(&ecp.u.pose, &cmd->u.move_tool.end, &end);
	if (is_circular) {
	  go_pose_cart_mult(&ecp.u.pose, &cmd->u.move_tool.center, &center);
	  go_pose_cart_mult(&ecp.u.pose, &cmd->u.move_tool.normal, &normal);
	  turns = cmd->u.move_tool.turns;
	}
      }

      /*
	Clamp the values to the limits. This doesn't check the whole
	circular move, just the end.
	FIXME-- check the whole circular move against limits.
      */
      if (GO_RESULT_OK != 
	  clamp_pose(&end,
		     &set->min_limit,
		     &set->max_limit)) {
	stat->inpos = 1;
	go_status_next(stat, GO_RCS_STATUS_ERROR);
	go_state_next(stat, GO_RCS_STATE_S0);
	return;
      } 
      /* now 'end' is in the world frame, clamped to be inside limits */

      go_motion_spec_init(&gms);
      go_motion_spec_set_id(&gms, id);
      go_motion_spec_set_end_pose(&gms, &end);

      if (is_circular) {
	go_motion_spec_set_type(&gms, GO_MOTION_CIRCULAR);
	go_motion_spec_set_cpar(&gms, &center, &normal, turns);
      } else {
	go_motion_spec_set_type(&gms, GO_MOTION_LINEAR);
      }

      if (time > GO_REAL_EPSILON) {
	go_motion_spec_set_tpar(&gms, set->max_tvel, set->max_tacc, set->max_tjerk);
	go_motion_spec_set_rpar(&gms, set->max_rvel, set->max_racc, set->max_rjerk);
	go_motion_spec_set_time(&gms, time);
      } else {
	go_motion_spec_set_tpar(&gms, tv, ta, tj);
	go_motion_spec_set_rpar(&gms, rv, ra, rj);
	/* no need to set time for move; it's handled automatically
	   by go_motion_spec_init() as the default */
      }
      if (GO_RESULT_OK != go_motion_queue_append(queue, &gms)) {
	rtapi_print("trajloop: can't append %s move\n", world ? "world" : "tool");
	stat->inpos = 1;
	go_status_next(stat, GO_RCS_STATUS_ERROR);
	go_state_next(stat, GO_RCS_STATE_S0);
      }
    }
    /* else the id hasn't changed, so ignore this move */

    /* move will be scaled to 'time' if provided by the interp function */
    if (GO_RESULT_OK != go_motion_queue_interp(queue, &ecp)) {
      rtapi_print("trajloop: can't interp\n");
      stat->inpos = 1;
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
    } else {
      stat->ecp = ecp.u.pose;
#ifdef USE_XINV
      /* walk in any Xinv inputs to avoid jumps */
      stat->xinv = walk_in(stat, set, ref);
      /* adjust the nominal ECP to get one that goes out */
      go_pose_pose_mult(&ecp.u.pose, &stat->xinv, &ecp.u.pose);
      /* FIXME-- should we keep track of the adjusted ECP separately?
	 We're overwriting it here. */
#endif
      /* convert from ECP to KCP to before using the kinematics */
      go_pose_pose_mult(&ecp.u.pose, &set->tool_transform_inv, &stat->kcp);
      for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
	joints[servo_num] = stat->joints[servo_num]; /* seed the estimate */
      }
      if (GO_RESULT_OK != go_kin_inv(kinematics, &stat->kcp, joints)) {
	rtapi_print("trajloop: do_cmd_move_world_or_tool: can't invert\n");
	stat->inpos = 1;
	go_status_next(stat, GO_RCS_STATUS_ERROR);
	go_state_next(stat, GO_RCS_STATE_S0);
      } else {
	/* shift joints to nearest revolution */
	shift_joints(joints, stat->joints, set->joint_num, kinematics);
	stat->inpos = go_motion_queue_is_empty(queue);
	if (stat->inpos) {
	  go_status_next(stat, GO_RCS_STATUS_DONE);
	}
	for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
	  stat->joints[servo_num] = joints[servo_num];
	  servo_cmd[servo_num].type = SERVO_CMD_SERVO_TYPE;
	  servo_cmd[servo_num].u.servo.setpoint = joints[servo_num] + stat->joint_offsets[servo_num];
	  write_servo_cmd(servo_cmd, servo_num);
	}
      }
    }
  } else {			/* S0 */
    go_state_default(stat);
  }
}

static void do_cmd_track_world(traj_cmd_struct * cmd, traj_stat_struct * stat, traj_set_struct * set, traj_ref_struct * ref, servo_cmd_struct * servo_cmd, void * kinematics)
{
  go_integer servo_num;
  go_pose ecp;
  go_pose kcp;
  go_real joints[SERVO_NUM];

  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_1("traj: cmd track world\n");
    go_state_new(stat);
    if (stat->admin_state == GO_RCS_ADMIN_STATE_INITIALIZED &&
	stat->homed) {
      stat->frame = TRAJ_WORLD_FRAME;
      stat->inpos = 0;
      go_status_next(stat, GO_RCS_STATUS_EXEC);
      go_state_next(stat, GO_RCS_STATE_S1);
    } else {
      stat->inpos = 1;
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
      return;
    }
  }

  if (go_state_match(stat, GO_RCS_STATE_S1)) {
    /* recall that the tracked position is in the ECP frame */
    ecp = cmd->u.track_world.position;
    /* clamp the values to the limits */
    if (GO_RESULT_OK != 
	clamp_pose(&ecp,
		   &set->min_limit,
		   &set->max_limit)) {
      stat->inpos = 1;
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
      return;
    } 
    /* keep the commanded position synchronized */
    stat->ecp = ecp;
#ifdef USE_XINV
    stat->xinv = walk_in(stat, set, ref);
    go_pose_pose_mult(&ecp, &stat->xinv, &ecp);
    /* FIXME-- as with moves, should we keep track of the adjusted ECP
       separately? */
#endif
    /* convert from ECP to KCP to before using the kinematics */
    go_pose_pose_mult(&ecp, &set->tool_transform_inv, &kcp);
    if (GO_RESULT_OK != go_kin_inv(kinematics, &kcp, joints)) {
      rtapi_print("trajloop: can't invert\n");
      stat->inpos = 1;
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
    } else {
      /* shift joints to nearest revolution */
      shift_joints(joints, stat->joints, set->joint_num, kinematics);
      for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
	stat->joints[servo_num] = joints[servo_num];
	servo_cmd[servo_num].type = SERVO_CMD_SERVO_TYPE;
	servo_cmd[servo_num].u.servo.setpoint = joints[servo_num] + stat->joint_offsets[servo_num];
	write_servo_cmd(servo_cmd, servo_num);
      }
    }
  } else {			/* S0 */
    go_state_default(stat);
  }
}

static void do_cmd_track_joint(traj_cmd_struct * cmd, traj_stat_struct * stat, traj_set_struct * set, servo_cmd_struct * servo_cmd, servo_stat_struct * servo_stat, servo_set_struct * servo_set)
{
  go_integer servo_num;
  go_position position;

  /* NEW_COMMAND */
  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_1("traj: cmd track joint\n");
    go_state_new(stat);
    if (stat->admin_state == GO_RCS_ADMIN_STATE_INITIALIZED) {
      stat->frame = TRAJ_JOINT_FRAME;
      stat->inpos = 0;
      go_status_next(stat, GO_RCS_STATUS_EXEC);
      go_state_next(stat, GO_RCS_STATE_S1);
    } else {
      stat->inpos = 1;
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
      return;
    }
  }
  /* drop through to S1 */

  if (go_state_match(stat, GO_RCS_STATE_S1)) {
    for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
      /* clamp joint position to lie within limits, if this joint is homed */
      if (servo_stat[servo_num].homed) {
	if (cmd->u.track_joint.joints[servo_num] > servo_set[servo_num].max_limit)
	  position.u.joint[servo_num] = servo_set[servo_num].max_limit;
	else if (cmd->u.track_joint.joints[servo_num] < servo_set[servo_num].min_limit)
	  position.u.joint[servo_num] = servo_set[servo_num].min_limit;
	else
	  position.u.joint[servo_num] = cmd->u.track_joint.joints[servo_num];
      } else {
	position.u.joint[servo_num] = cmd->u.track_joint.joints[servo_num];
      }
      /* write out this joint setpoint */
      stat->joints[servo_num] = position.u.joint[servo_num];
      servo_cmd[servo_num].type = SERVO_CMD_SERVO_TYPE;
      servo_cmd[servo_num].u.servo.setpoint =
	position.u.joint[servo_num] + stat->joint_offsets[servo_num];
      servo_cmd[servo_num].u.servo.home = 0;
      write_servo_cmd(servo_cmd, servo_num);
    }
  } else {			
    /* S0 */
    go_state_default(stat);
  }
}
 
static go_real filter(go_real current, go_real target, go_real incr)
{
  go_real delta;

  delta = target - current;
  if (delta > incr) delta = incr;
  else if (delta < -incr) delta = -incr;

  return current + delta;
}

static void do_cmd_teleop_joint(traj_cmd_struct * cmd, traj_stat_struct * stat, traj_set_struct * set, servo_cmd_struct * servo_cmd, servo_stat_struct * servo_stat, servo_set_struct * servo_set, go_motion_queue * queue, go_real * joint_teleop_speed)
{
  go_position position;
  go_integer servo_num;

  /* NEW_COMMAND */
  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_1("traj: cmd teleop joint\n");
    go_state_new(stat);
    if (stat->admin_state == GO_RCS_ADMIN_STATE_INITIALIZED) {
      stat->frame = TRAJ_JOINT_FRAME;
      stat->inpos = 0;
      if (go_motion_queue_get_type(queue) != GO_MOTION_JOINT) {
	go_motion_queue_reset(queue);
	go_motion_queue_set_type(queue, GO_MOTION_JOINT);
	for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
	  /* set 'here' to be in offset space */
	  position.u.joint[servo_num] = stat->joints_act[servo_num];
	  /* set 'joints' to be actual joints */
	  stat->joints[servo_num] = stat->joints_act[servo_num];
	}
	go_motion_queue_set_here(queue, &position);
      }
      go_status_next(stat, GO_RCS_STATUS_EXEC);
      go_state_next(stat, GO_RCS_STATE_S1);
    } else {
      stat->inpos = 1;
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
      return;
    }
  }
  /* drop through to S1 */

  if (go_state_match(stat, GO_RCS_STATE_S1)) {
    for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
      /*
	Calculate the servo setpoint as the current joint position
	plus the joint speed X cycle time, filtered through joint accel
      */
      joint_teleop_speed[servo_num] = filter(joint_teleop_speed[servo_num], cmd->u.teleop_joint.v[servo_num], cmd->u.teleop_joint.a[servo_num] * set->cycle_time);

      stat->joints[servo_num] = 
#ifdef USE_ACTUAL
	stat->joints_act[servo_num] + 
#else
	stat->joints[servo_num] + 
#endif
	(joint_teleop_speed[servo_num] * set->cycle_time);
      /* accumulate each joint into 'position' for keeping the
	 queue up to date */
      position.u.joint[servo_num] = stat->joints[servo_num];
      servo_cmd[servo_num].type = SERVO_CMD_SERVO_TYPE;
      servo_cmd[servo_num].u.servo.setpoint =
	stat->joints[servo_num] + stat->joint_offsets[servo_num];
      servo_cmd[servo_num].u.servo.home = 0;
      write_servo_cmd(servo_cmd, servo_num);
    }
    /* keep the queue position up to date */
    go_motion_queue_set_here(queue, &position);
  } else {			
    /* S0 */
    go_state_default(stat);
  }
}

static void do_cmd_teleop_world_or_tool(go_flag world, traj_cmd_struct * cmd, traj_stat_struct * stat, traj_set_struct * set, servo_cmd_struct * servo_cmd, void * kinematics, go_motion_queue * queue, go_vel * world_teleop_speed)
{
  go_position position;
  go_pose kcp_act;
  go_integer servo_num;
  go_real jointvels[SERVO_NUM];
  go_result retval;
  go_vel tv;
  go_vel tvk;
  go_real ta;
  go_real ra;

  if (world) {
    tv = cmd->u.teleop_world.tv;
    ta = cmd->u.teleop_world.ta;
    ra = cmd->u.teleop_world.ra;
  } else {
    tv = cmd->u.teleop_tool.tv;
    ta = cmd->u.teleop_tool.ta;
    ra = cmd->u.teleop_tool.ra;
  }
  if (ta > set->max_tacc) ta = set->max_tacc;
  if (ra > set->max_racc) ta = set->max_racc;
  /* we're ignoring jerks */

  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_2("traj: cmd teleop %s\n", world ? "world" : "tool");
    go_state_new(stat);
    if (stat->admin_state == GO_RCS_ADMIN_STATE_INITIALIZED &&
	stat->homed) {
      stat->frame = TRAJ_WORLD_FRAME;
      stat->inpos = 0;
      if (go_motion_queue_get_type(queue) != GO_MOTION_WORLD) {
	/* update our joints */
	copy_joints(stat->joints, stat->joints_act, set->joint_num);
	go_motion_queue_reset(queue);
	go_motion_queue_set_type(queue, GO_MOTION_WORLD);
	/* set the queue 'here' to be the actual ECP */
	position.u.pose = stat->ecp_act;
	/* set the commanded ECP to be the actual ECP; we'll be
	   overwriting this below each cycle anyway */
	stat->ecp= stat->ecp_act;
	go_motion_queue_set_here(queue, &position);
      }
      go_status_next(stat, GO_RCS_STATUS_EXEC);
      go_state_next(stat, GO_RCS_STATE_S1);
    } else {
      stat->inpos = 1;
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
    }
  }
  /* drop through */

  if (go_state_match(stat, GO_RCS_STATE_S1)) {
    if (! world) {
      /* rotate the tran part from the tool frame to the ECP frame */
      go_quat_cart_mult(&stat->ecp_act.rot, &tv.v, &tv.v);
      /* rotate the rot part likewise */
      go_quat_cart_mult(&stat->ecp_act.rot, &tv.w, &tv.w);
    }

    /* inhibit speeds that take the device outside its limits, using
       our actual position as where we are, which should be the
       best estimate */
    if (GO_RESULT_OK != 
	clamp_vel(&stat->ecp_act,
		  &tv,
		  &set->min_limit,
		  &set->max_limit)) {
      stat->inpos = 1;
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
      return;
    }

    /* filter the speed */
    world_teleop_speed->v.x = filter(world_teleop_speed->v.x, tv.v.x, ta);
    world_teleop_speed->v.y = filter(world_teleop_speed->v.y, tv.v.y, ta);
    world_teleop_speed->v.z = filter(world_teleop_speed->v.z, tv.v.z, ta);
    world_teleop_speed->w.x = filter(world_teleop_speed->w.x, tv.w.x, ra);
    world_teleop_speed->w.y = filter(world_teleop_speed->w.y, tv.w.y, ra);
    world_teleop_speed->w.z = filter(world_teleop_speed->w.z, tv.w.z, ra);

    /* convert velocity in ECP to velocity in KCP, recalling that to
       convert from poses in the ECP to poses in the KCP, premultiply
       by the tool transform */
    go_pose_vel_mult(&set->tool_transform, world_teleop_speed, &tvk);

    /* convert ECP to KCP for position estimate to inverse Jacobian */
    go_pose_pose_mult(&stat->ecp_act, &set->tool_transform_inv, &kcp_act);

    /* run the inverse Jacobian */
    retval = go_kin_jac_inv(kinematics,
			    &kcp_act,
			    &tvk,
			    stat->joints_act,
			    jointvels);

    if (GO_RESULT_OK != retval) {
      rtapi_print("trajloop: can't calculate jac inv\n");
      stat->inpos = 1;
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
    } else {
      /* keep the commanded position and queue up to date */
      stat->ecp = stat->ecp_act;
      position.u.pose = stat->ecp;
      go_motion_queue_set_here(queue, &position);
      for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
	/*
	  Calculate the servo setpoint as the current joint position
	  plus the joint speed X cycle time
	*/
	stat->joints[servo_num] = 
#ifdef USE_ACTUAL
	  stat->joints_act[servo_num] + 
#else
	  stat->joints[servo_num] + 
#endif
	  (jointvels[servo_num] * set->cycle_time);
	servo_cmd[servo_num].type = SERVO_CMD_SERVO_TYPE;
	servo_cmd[servo_num].u.servo.setpoint = 
	  stat->joints[servo_num] + stat->joint_offsets[servo_num];
	servo_cmd[servo_num].u.servo.home = 0;
	write_servo_cmd(servo_cmd, servo_num);
      }
    }
  } else {			/* S0 */
    go_state_default(stat);
  }
}

static void do_cmd_stub(traj_cmd_struct * cmd, traj_stat_struct * stat, traj_set_struct * set, servo_cmd_struct * servo_cmd, servo_stat_struct * servo_stat, servo_set_struct * servo_set)
{
  go_integer servo_num;
  go_integer num_done;
  go_flag is_error;

  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_2("traj: cmd stub %d\n", cmd->u.stub.arg);
    go_state_new(stat);
    if (stat->admin_state == GO_RCS_ADMIN_STATE_INITIALIZED) {
      for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
	servo_cmd[servo_num].type = SERVO_CMD_STUB_TYPE;
	servo_cmd[servo_num].u.stub.arg = cmd->u.stub.arg;
	write_servo_cmd(servo_cmd, servo_num);
      }
      go_status_next(stat, GO_RCS_STATUS_EXEC);
      go_state_next(stat, GO_RCS_STATE_S1);
    } else {
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
      return;
    }
  } else if (go_state_match(stat, GO_RCS_STATE_S1)) {
    num_done = 0;
    is_error = 0;
    for (servo_num = 0; servo_num < set->joint_num; servo_num++) {
      if (servo_stat[servo_num].command_type == SERVO_CMD_STUB_TYPE &&
	  servo_stat[servo_num].echo_serial_number == servo_cmd[servo_num].serial_number) {
	if (servo_stat[servo_num].status == GO_RCS_STATUS_DONE) {
	  num_done++;
	} else if (servo_stat[servo_num].status == GO_RCS_STATUS_ERROR) {
	  is_error = 1;
	  break;
	} /* else still executing */
      }	/* else didn't get there yet */
    } /* for (servo_num) */
    if (num_done == set->joint_num) {
      go_status_next(stat, GO_RCS_STATUS_DONE);
      go_state_next(stat, GO_RCS_STATE_S0);
    } else if (is_error) {
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
    } /* else still working on it */
  } else {			/* S0 */
    go_state_default(stat);
  }
}

static void do_cfg_nop(traj_set_struct * set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    CFG_PRINT_1("traj: cfg nop\n");
    go_state_new(set);
    go_status_next(set, GO_RCS_STATUS_DONE);
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

#define MYROUND(x) ((x) >= 0.0 ? (int) ((x) + 0.5) : (int) ((x) - 0.5))

static void do_cfg_cycle_time(traj_cfg_struct * cfg, traj_set_struct * set, servo_cfg_struct * servo_cfg, servo_set_struct * servo_set, go_motion_queue * queue)
{
  go_real frac;
  rtapi_integer period_nsec;

  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    CFG_PRINT_2("traj: cfg cycle time %f\n",
		(double) cfg->u.cycle_time.cycle_time);
    go_state_new(set);
    if (cfg->u.cycle_time.cycle_time <= 0.0) {
      go_status_next(set, GO_RCS_STATUS_ERROR);
      go_state_next(set, GO_RCS_STATE_S0);
    } else {
      /* reference Servo 0 as the basis, since that clocks us */
      servo_cfg[0].type = SERVO_CFG_CYCLE_MULT_TYPE;
      frac = ((go_real) cfg->u.cycle_time.cycle_time) /
	((go_real) servo_set[0].cycle_time);
      servo_cfg[0].u.cycle_mult.cycle_mult = (int) MYROUND(frac);
      write_servo_cfg(servo_cfg, 0);
      go_status_next(set, GO_RCS_STATUS_EXEC);
      go_state_next(set, GO_RCS_STATE_S1);
    }
  } else if (go_state_match(set, GO_RCS_STATE_S1)) {
      if (servo_set[0].command_type == SERVO_CFG_CYCLE_MULT_TYPE &&
	  servo_set[0].echo_serial_number == servo_cfg[0].serial_number) {
	if (servo_set[0].status == GO_RCS_STATUS_DONE) {
	  set->cycle_time = cfg->u.cycle_time.cycle_time;
	  period_nsec = (rtapi_integer) (set->cycle_time * 1.0e9);
	  rtapi_self_set_period(period_nsec);
	  go_motion_queue_set_cycle_time(queue, set->cycle_time);
	  go_status_next(set, GO_RCS_STATUS_DONE);
	  go_state_next(set, GO_RCS_STATE_S0);
	} else if (servo_set[0].status == GO_RCS_STATUS_ERROR) {
	  go_status_next(set, GO_RCS_STATUS_ERROR);
	  go_state_next(set, GO_RCS_STATE_S0);
	} /* else still executing */
      }	/* else didn't get there yet */
  } else {			/* S0 */
    go_state_default(set);
  }
}

static void do_cfg_debug(traj_cfg_struct * cfg, traj_set_struct * set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    CFG_PRINT_2("traj: cfg debug %x\n", (int) cfg->u.debug.debug);
    go_state_new(set);
    set->debug = cfg->u.debug.debug;
    go_status_next(set, GO_RCS_STATUS_DONE);
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_home(traj_stat_struct * stat, traj_cfg_struct * cfg, traj_set_struct * set, void * kinematics)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    CFG_PRINT_1("traj: cfg home\n");
    go_state_new(set);
    /* the home position is sent and stored in the ECP, so we can
       just leave it alone here */
    set->home = cfg->u.home.home;
    go_status_next(set, GO_RCS_STATUS_DONE);
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_limit(traj_cfg_struct * cfg, traj_set_struct * set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    CFG_PRINT_1("traj: cfg limit\n");
    go_state_new(set);
    /* the limits are sent and stored in the ECP, so we can just
       leave them alone */
    set->min_limit = cfg->u.limit.min_limit;
    set->max_limit = cfg->u.limit.max_limit;
    go_status_next(set, GO_RCS_STATUS_DONE);
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_profile(traj_cfg_struct * cfg, traj_set_struct * set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    CFG_PRINT_1("traj: cfg profile\n");
    go_state_new(set);
    set->max_tvel = cfg->u.profile.max_tvel;
    set->max_tacc = cfg->u.profile.max_tacc;
    set->max_tjerk = cfg->u.profile.max_tjerk;
    set->max_rvel = cfg->u.profile.max_rvel;
    set->max_racc = cfg->u.profile.max_racc;
    set->max_rjerk = cfg->u.profile.max_rjerk;
    go_status_next(set, GO_RCS_STATUS_DONE);
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_kinematics(traj_cfg_struct * cfg, traj_set_struct * set, void * kins)
{
  go_result retval;

  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    CFG_PRINT_1("traj: cfg kinematics\n");
    go_state_new(set);
    retval = go_kin_set_parameters(kins,
				   cfg->u.kinematics.parameters,
				   cfg->u.kinematics.num);
    if (GO_RESULT_OK != retval) {
      go_status_next(set, GO_RCS_STATUS_ERROR);
    } else {
      go_status_next(set, GO_RCS_STATUS_DONE);
    }
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

#define NONNEGATIVE(x) (x) < 0.0 ? 0.0 : (x)
#define POSITIVE(x) (x) < GO_REAL_EPSILON ? GO_REAL_EPSILON : (x)

static void do_cfg_scale(traj_cfg_struct * cfg, traj_set_struct * set, go_motion_queue * queue)
{
  go_real scale, scale_v, scale_a;
  go_result retval;

  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    CFG_PRINT_4("traj: cfg scale %f %f %f\n",
		(double) cfg->u.scale.scale,
		(double) cfg->u.scale.scale_v,
		(double) cfg->u.scale.scale_a);
    go_state_new(set);
    scale = NONNEGATIVE(cfg->u.scale.scale);
    scale_v = POSITIVE(cfg->u.scale.scale_v);
    scale_a = POSITIVE(cfg->u.scale.scale_a);
    if (scale > set->max_scale) scale = set->max_scale;
    if (scale_v > set->max_scale_v) scale_v = set->max_scale_v;
    if (scale_a > set->max_scale_a) scale_a = set->max_scale_a;
    retval = go_motion_queue_set_scale(queue, scale, scale_v, scale_a);
    if (GO_RESULT_OK == retval) {
      /* these don't change dynamically, as does the scale, so just
	 set them here instead of updating them cyclically in the loop */
      set->scale_v = scale_v;
      set->scale_a = scale_a;
      go_status_next(set, GO_RCS_STATUS_DONE);
    } else {
      go_status_next(set, GO_RCS_STATUS_ERROR);
    }
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_max_scale(traj_cfg_struct * cfg, traj_set_struct * set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    CFG_PRINT_4("traj: cfg max scale %f %f %f\n",
		(double) cfg->u.scale.scale,
		(double) cfg->u.scale.scale_v,
		(double) cfg->u.scale.scale_a);
    go_state_new(set);
    set->max_scale = POSITIVE(cfg->u.scale.scale);
    set->max_scale_v = POSITIVE(cfg->u.scale.scale_v);
    set->max_scale_a = POSITIVE(cfg->u.scale.scale_a);
    go_status_next(set, GO_RCS_STATUS_DONE);
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_log(traj_stat_struct * stat, traj_cfg_struct * cfg, traj_set_struct * set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    go_state_new(set);
    CFG_PRINT_4("traj: cfg log %d %d %d\n", (int) cfg->u.log.log_type, (int) cfg->u.log.log_which, (int) cfg->u.log.log_size);
    if (GO_RESULT_OK != go_log_init(global_go_log_ptr, cfg->u.log.log_type, cfg->u.log.log_which, cfg->u.log.log_size)) {
      go_status_next(set, GO_RCS_STATUS_ERROR);
    } else {
      set->log_type = cfg->u.log.log_type;
      set->log_which = cfg->u.log.log_which;
      set->log_logging = 0;
      go_status_next(set, GO_RCS_STATUS_DONE);
    }
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_log_start(traj_stat_struct * stat, traj_cfg_struct * cfg, traj_set_struct * set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    go_state_new(set);
    CFG_PRINT_1("traj: cfg log start\n");
    set->log_logging = 1;
    go_status_next(set, GO_RCS_STATUS_DONE);
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_log_stop(traj_stat_struct * stat, traj_cfg_struct * cfg, traj_set_struct * set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    go_state_new(set);
    CFG_PRINT_1("traj: cfg log stop\n");
    set->log_logging = 0;
    go_status_next(set, GO_RCS_STATUS_DONE);
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_tool_transform(traj_stat_struct * stat, traj_cfg_struct * cfg, traj_set_struct * set, go_motion_queue * queue)
{
  go_position ecp;
  go_pose inv;
  go_pose Q;

  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    go_state_new(set);
    CFG_PRINT_2("traj: cfg tool transform %d\n", (int) cfg->u.tool_transform.tool_transform.tran.x);
    go_status_next(set, GO_RCS_STATUS_EXEC);
    go_state_next(set, GO_RCS_STATE_S1);
  }

  /* wait for motion queue to be empty, which we really only need to
     do if it's in world mode, not joint mode */
  if (go_state_match(set, GO_RCS_STATE_S1)) {
    if (go_motion_queue_is_empty(queue)) {
      go_state_next(set, GO_RCS_STATE_S2);
    }
    /* else keep waiting */
  } else if (go_state_match(set, GO_RCS_STATE_S2)) {
    if (GO_RESULT_OK != go_pose_inv(&cfg->u.tool_transform.tool_transform, &inv)) {
      go_status_next(set, GO_RCS_STATUS_ERROR);
      go_state_next(set, GO_RCS_STATE_S0);
    } else {
      /*
	We need to convert the limits and home from the previous
	ECP to the new ECP:

	Enew       K    Eold      Enew
	.   T *     T *     lim =     lim
	.  K    Eold
	
	were Q = new tool transform inverse * old tool transform.
       */
      go_pose_pose_mult(&inv, &set->tool_transform, &Q);
      go_pose_pose_mult(&Q, &set->min_limit, &set->min_limit);
      go_pose_pose_mult(&Q, &set->max_limit, &set->max_limit);
      go_pose_pose_mult(&Q, &set->home, &set->home);

      /* now we can clobber the old tool transforms with the new ones */
      set->tool_transform = cfg->u.tool_transform.tool_transform;
      set->tool_transform_inv = inv;
      /*
	if the queue is in world mode, we need to reset its ECP
      */
      if (queue->type == GO_MOTION_WORLD) {
	go_pose_pose_mult(&stat->kcp, &set->tool_transform, &ecp.u.pose);
	go_motion_queue_set_here(queue, &ecp);
      }
      go_status_next(set, GO_RCS_STATUS_DONE);
      go_state_next(set, GO_RCS_STATE_S0);
    }
  } else {			/* S0 */
    go_state_default(set);
  }
}

static void do_cfg_stub(traj_stat_struct * stat, traj_cfg_struct * cfg, traj_set_struct * set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    go_state_new(set);
    CFG_PRINT_2("traj: cfg stub %d\n", (int) cfg->u.stub.arg);
    go_status_next(set, GO_RCS_STATUS_DONE);
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

#define PROG_PRINT_1(x) if (traj_set.debug & DEBUG_PROG) rtapi_print(x)
#define TASK_PRINT_1(x) if (traj_set.debug & DEBUG_TASK) rtapi_print(x)
#define HOME_PRINT_3(x,y,z) if (traj_set.debug & DEBUG_HOME) rtapi_print(x, y, z)

void traj_loop(void * arg)
{
  go_integer joint_num;
  void * kinematics;
  go_rpy rpy;
  enum { TRAJ_MOTION_QUEUE_SIZE = 10};
  go_position position;
  go_pose kcp_act;
  go_motion_spec traj_motion_queue_space[TRAJ_MOTION_QUEUE_SIZE];
  go_motion_queue traj_motion_queue;
  go_real deltat = DEFAULT_CYCLE_TIME;
  go_real calc_time;		/* actual time for traj calcs */
  traj_cmd_struct pp_traj_cmd[2], * traj_cmd_ptr, * traj_cmd_test;
  traj_stat_struct traj_stat;
  traj_cfg_struct pp_traj_cfg_struct[2], * traj_cfg_ptr, * traj_cfg_test;
  traj_set_struct traj_set;
  traj_ref_struct pp_traj_ref_struct[2], * traj_ref_ptr, * traj_ref_test;
  servo_cmd_struct servo_cmd[SERVO_NUM];
  servo_stat_struct pp_servo_stat[2][SERVO_NUM], * servo_stat_ptr[SERVO_NUM],
    * servo_stat_test[SERVO_NUM];
  servo_cfg_struct servo_cfg[SERVO_NUM];
  servo_set_struct pp_servo_set[2][SERVO_NUM], * servo_set_ptr[SERVO_NUM],
    * servo_set_test[SERVO_NUM];
  go_real joint_teleop_speed[SERVO_NUM];
  go_vel world_teleop_speed;
  rtapi_integer old_sec, old_nsec, sec, nsec;
  rtapi_integer start_sec, start_nsec, end_sec, end_nsec;
  rtapi_integer diff_sec, diff_nsec;
  void * tmp;
  go_integer servo_num;
  go_integer cmd_type, cfg_type;
  go_integer cmd_serial_number, cfg_serial_number;
  go_integer joints_active;
  go_integer joints_homed;
  go_flag homed_transition;
  go_log_entry entry;
  go_result retval;

  /* read out some 'arguments' from our status and settings */
  traj_stat = global_traj_comm_ptr->traj_stat;
  traj_set = global_traj_comm_ptr->traj_set;

  joint_num = ((traj_arg_struct *) arg)->joint_num;
  kinematics = ((traj_arg_struct *) arg)->kinematics;

  if (GO_RESULT_OK != go_init() ||
      GO_RESULT_OK != go_motion_queue_init(&traj_motion_queue,
					   traj_motion_queue_space, 
					   TRAJ_MOTION_QUEUE_SIZE,
					   deltat) ||
      GO_RESULT_OK != go_motion_queue_set_type(&traj_motion_queue,
					       GO_MOTION_JOINT)) {
    rtapi_print("trajloop: can't init traj motion queue\n");
    return;
  }

  /* set up ping-pong buffers */
  traj_cmd_ptr = &pp_traj_cmd[0];
  traj_cmd_test = &pp_traj_cmd[1];
  traj_cmd_ptr->head = traj_cmd_ptr->tail = 0;
  traj_cmd_ptr->type = TRAJ_CMD_NOP_TYPE;
  traj_cmd_ptr->serial_number = 0;
  global_traj_comm_ptr->traj_cmd = *traj_cmd_ptr; /* force a write into ourself */
  /*  */
  traj_cfg_ptr = &pp_traj_cfg_struct[0];
  traj_cfg_test = &pp_traj_cfg_struct[1];
  traj_cfg_ptr->head = traj_cfg_ptr->tail = 0;
  traj_cfg_ptr->type = TRAJ_CFG_NOP_TYPE;
  traj_cfg_ptr->serial_number = 0;
  global_traj_comm_ptr->traj_cfg = *traj_cfg_ptr; /* as above */
  /*  */
  traj_ref_ptr = &pp_traj_ref_struct[0];
  traj_ref_test = &pp_traj_ref_struct[1];
  traj_ref_ptr->head = traj_ref_ptr->tail = 0;
  traj_ref_ptr->xinv = go_pose_identity();
  global_traj_comm_ptr->traj_ref = *traj_ref_ptr; /* as above */
  /*  */
  for (servo_num = 0; servo_num < joint_num; servo_num++) {
    /* set the head and tail to be 0, so the first write will increment
       them to the conventional 1 */
    servo_cmd[servo_num].head = servo_cmd[servo_num].tail = 0;
    servo_cmd[servo_num].serial_number = 0;
    /*  */
    servo_stat_ptr[servo_num] = &pp_servo_stat[0][servo_num];
    servo_stat_test[servo_num] = &pp_servo_stat[1][servo_num];
    /*  */
    servo_cfg[servo_num].head = servo_cfg[servo_num].tail = 0;
    servo_cfg[servo_num].serial_number = 0;
    /*  */
    servo_set_ptr[servo_num] = &pp_servo_set[0][servo_num];
    servo_set_test[servo_num] = &pp_servo_set[1][servo_num];
  }

  /* get the first good servo reads */
  for (servo_num = 0; servo_num < joint_num; servo_num++) {
    *servo_stat_test[servo_num] = global_servo_comm_ptr[servo_num].servo_stat;
    if (servo_stat_test[servo_num]->head == servo_stat_test[servo_num]->tail) {
      tmp = servo_stat_ptr[servo_num];
      servo_stat_ptr[servo_num] = servo_stat_test[servo_num];
      servo_stat_test[servo_num] = tmp;
    }
    /*  */
    *servo_set_test[servo_num] = global_servo_comm_ptr[servo_num].servo_set;
    if (servo_set_test[servo_num]->head == servo_set_test[servo_num]->tail) {
      tmp = servo_set_ptr[servo_num];
      servo_set_ptr[servo_num] = servo_set_test[servo_num];
      servo_set_test[servo_num] = tmp;
    }
  } /* for (servo_num) */

  traj_stat.head = 0;
  traj_stat.type = TRAJ_STAT_TYPE;
  traj_stat.admin_state = GO_RCS_ADMIN_STATE_UNINITIALIZED;
  traj_stat.echo_serial_number = traj_cmd_ptr->serial_number - 1;
  traj_stat.heartbeat = 0;
  traj_stat.homed = 0;
  traj_stat.frame = TRAJ_JOINT_FRAME;
  traj_stat.inpos = 1;
  go_motion_queue_number(&traj_motion_queue, &traj_stat.queue_count);
  traj_stat.cycle_time = DEFAULT_CYCLE_TIME;
  traj_stat.ecp = DEFAULT_POSITION;
  traj_stat.ecp_act = traj_stat.ecp;
  traj_stat.xinv = traj_ref_ptr->xinv;
  for (servo_num = 0; servo_num < joint_num; servo_num++) {
    traj_stat.joints[servo_num] = DEFAULT_JOINT;
    traj_stat.joints_act[servo_num] = traj_stat.joints[servo_num];
    traj_stat.joints_ferror[servo_num] = 0.0;
    traj_stat.joint_offsets[servo_num] = 0.0;
  }
  go_mmavg_init(&traj_stat.mmavg, NULL, 0, traj_timestamp);
  traj_stat.tail = traj_stat.head;

  traj_set.head = 0;
  traj_set.type = TRAJ_SET_TYPE;
  traj_set.echo_serial_number = traj_cfg_ptr->serial_number - 1;
  traj_set.id = 0;
  traj_set.cycle_time = DEFAULT_CYCLE_TIME;
  traj_set.debug = 0x0;
  traj_set.joint_num = joint_num;
  traj_set.home = DEFAULT_HOME;
  traj_set.tool_transform = go_pose_identity();
  go_pose_inv(&traj_set.tool_transform, &traj_set.tool_transform_inv);
  traj_set.min_limit.tran.x =
    traj_set.min_limit.tran.y =
    traj_set.min_limit.tran.z = -10.0;
  rpy.r = GO_TO_RAD(-30), rpy.p = GO_TO_RAD(-30), rpy.y = GO_TO_RAD(-30);
  go_rpy_quat_convert(&rpy, &traj_set.min_limit.rot);
  traj_set.max_limit.tran.x =
    traj_set.max_limit.tran.y =
    traj_set.max_limit.tran.z = -10.0;
  rpy.r = GO_TO_RAD(30), rpy.p = GO_TO_RAD(30), rpy.y = GO_TO_RAD(30);
  go_rpy_quat_convert(&rpy, &traj_set.max_limit.rot);
  traj_set.max_tvel = 1.0;
  traj_set.max_tacc = 1.0;
  traj_set.max_tjerk = 1.0;
  traj_set.max_rvel = 1.0;
  traj_set.max_racc = 1.0;
  traj_set.max_rjerk = 1.0;
  traj_set.scale = 1.0;
  traj_set.scale_v = 1.0;
  traj_set.scale_a = 1.0;
  traj_set.max_scale = 1.0;
  traj_set.max_scale_v = 1.0;
  traj_set.max_scale_a = 1.0;
  traj_set.log_type = GO_LOG_NONE;
  traj_set.log_which = 0;
  traj_set.log_logging = 0;
  go_motion_queue_size(&traj_motion_queue, &traj_set.queue_size);
  traj_set.tail = traj_set.head;

  /* set the actual number of joints we're using */
  if (GO_RESULT_OK != go_motion_queue_set_joint_number(&traj_motion_queue, joint_num)) {
    rtapi_print("trajloop: can't set traj motion queue joint number to %d\n", joint_num);
    return;
  }

  /* make sure that KCP = ECP * inverse tool transform */
  go_pose_pose_mult(&traj_stat.ecp, &traj_set.tool_transform_inv, &traj_stat.kcp);

  go_position_zero_joints(&position);
  for (servo_num = 0; servo_num < joint_num; servo_num++) {
    position.u.joint[servo_num] = traj_stat.joints_act[servo_num];
    joint_teleop_speed[servo_num] = 0.0;
  }
  go_motion_queue_set_here(&traj_motion_queue, &position);

  world_teleop_speed.v.x = 0.0;
  world_teleop_speed.v.y = 0.0;
  world_teleop_speed.v.z = 0.0;
  world_teleop_speed.w.x = 0.0;
  world_teleop_speed.w.y = 0.0;
  world_teleop_speed.w.z = 0.0;

  rtapi_clock_get_time(&old_sec, &old_nsec);

  PROG_PRINT_1("started traj_loop\n");

  while (1) {
    /* record start time, for perf measures */
    rtapi_clock_get_time(&start_sec, &start_nsec);

    /* read in command buffer, ping-pong style */
    *traj_cmd_test = global_traj_comm_ptr->traj_cmd;
    if (traj_cmd_test->head == traj_cmd_test->tail) {
      tmp = traj_cmd_ptr;
      traj_cmd_ptr = traj_cmd_test;
      traj_cmd_test = tmp;
    }
    cmd_type = traj_cmd_ptr->type;
    cmd_serial_number = traj_cmd_ptr->serial_number;

    /* clear these and build a running count each cycle */
    joints_active = 0;
    joints_homed = 0;

    /* read in servo stat,set, ping-pong style */
    for (servo_num = 0; servo_num < joint_num; servo_num++) {
      *servo_stat_test[servo_num] = global_servo_comm_ptr[servo_num].servo_stat;
      if (servo_stat_test[servo_num]->head == servo_stat_test[servo_num]->tail) {
	tmp = servo_stat_ptr[servo_num];
	servo_stat_ptr[servo_num] = servo_stat_test[servo_num];
	servo_stat_test[servo_num] = tmp;
      }
      /*  */
      *servo_set_test[servo_num] = global_servo_comm_ptr[servo_num].servo_set;
      if (servo_set_test[servo_num]->head == servo_set_test[servo_num]->tail) {
	tmp = servo_set_ptr[servo_num];
	servo_set_ptr[servo_num] = servo_set_test[servo_num];
	servo_set_test[servo_num] = tmp;
      }

      /* check if we're homed, set our offsets accordingly, and increment
	 our count of the joints that are homed to see if they're all homed
	 and full kinematics calculations can proceed */
      if (servo_set_ptr[servo_num]->active) {
	joints_active++;
	if (servo_stat_ptr[servo_num]->homed) {
	  traj_stat.joint_offsets[servo_num] =
	    servo_stat_ptr[servo_num]->input_latch - 
	    servo_set_ptr[servo_num]->home;
	  joints_homed++;
	}
      }

      /* update the actual measurement of the servo joints */
      traj_stat.joints_act[servo_num] = servo_stat_ptr[servo_num]->input -
	traj_stat.joint_offsets[servo_num];
      /* update the joint following errors */
      traj_stat.joints_ferror[servo_num] = servo_stat_ptr[servo_num]->ferror;
      /* leave our actual joints alone-- these may have been set by
	 a control state table above, and the past values may be used
	 as the basis for future incremental moves */
    } /* for (servo_num) */

    homed_transition = 0;
    if (joints_active > 0 && joints_homed >= joints_active) {
      if (! traj_stat.homed) {
	traj_stat.homed = 1;
	homed_transition = 1;
	HOME_PRINT_3("just homed %d out of %d joints\n", joints_homed, joints_active);
      }
    } else {
      traj_stat.homed = 0;
    }
    
    /* read in the reference buffer, ping-pong style */
    *traj_ref_test = global_traj_comm_ptr->traj_ref;
    if (traj_ref_test->head == traj_ref_test->tail) {
      tmp = traj_ref_ptr;
      traj_ref_ptr = traj_ref_test;
      traj_ref_test = tmp;
    }
    /* now traj_ref_ptr is where we look for our reference */

    /* calculate actual world position, initially using the world
       position as an estimate */
    if (traj_stat.homed) {
      kcp_act = traj_stat.kcp;
      retval = go_kin_fwd(kinematics,
			  traj_stat.joints_act,
			  &kcp_act);
      if (0 != retval) {
	rtapi_print("trajloop: forward kinematics error\n");
      } else {
	go_pose_pose_mult(&kcp_act, &traj_set.tool_transform, &traj_stat.ecp_act);
	if (homed_transition) {
	  traj_stat.ecp = traj_stat.ecp_act;
	}
      }
    } else {
      traj_stat.ecp = traj_set.home;
      traj_stat.ecp_act = traj_stat.ecp;
      go_pose_pose_mult(&traj_stat.ecp, &traj_set.tool_transform_inv, &traj_stat.kcp);
    }

    switch (cmd_type) {
    case 0:
    case -1:
      break;

    case TRAJ_CMD_NOP_TYPE:
    case TRAJ_CMD_INIT_TYPE:
    case TRAJ_CMD_HALT_TYPE:
    case TRAJ_CMD_ABORT_TYPE:
    case TRAJ_CMD_SHUTDOWN_TYPE:
    case TRAJ_CMD_STOP_TYPE:
    case TRAJ_CMD_MOVE_WORLD_TYPE:
    case TRAJ_CMD_MOVE_TOOL_TYPE:
    case TRAJ_CMD_TRACK_WORLD_TYPE:
    case TRAJ_CMD_TRACK_JOINT_TYPE:
    case TRAJ_CMD_MOVE_JOINT_TYPE:
    case TRAJ_CMD_MOVE_UJOINT_TYPE:
    case TRAJ_CMD_TELEOP_JOINT_TYPE:
    case TRAJ_CMD_TELEOP_WORLD_TYPE:
    case TRAJ_CMD_TELEOP_TOOL_TYPE:
    case TRAJ_CMD_HERE_TYPE:
    case TRAJ_CMD_STUB_TYPE:
      traj_stat.command_type = cmd_type;
      if (cmd_serial_number != traj_stat.echo_serial_number) {
	traj_stat.echo_serial_number = cmd_serial_number;
	traj_stat.state = GO_RCS_STATE_NEW_COMMAND;
      }
      break;

    default:
      rtapi_print("trajloop: %s: unknown command %d\n", BN, cmd_type);
      break;
    }

    /* read in config buffer, ping-pong style */
    *traj_cfg_test = global_traj_comm_ptr->traj_cfg;
    if (traj_cfg_test->head == traj_cfg_test->tail) {
      tmp = traj_cfg_ptr;
      traj_cfg_ptr = traj_cfg_test;
      traj_cfg_test = tmp;
    }
    cfg_type = traj_cfg_ptr->type;
    cfg_serial_number = traj_cfg_ptr->serial_number;

    switch (cfg_type) {
    case 0:
    case -1:
      break;

    case TRAJ_CFG_NOP_TYPE:
    case TRAJ_CFG_CYCLE_TIME_TYPE:
    case TRAJ_CFG_DEBUG_TYPE:
    case TRAJ_CFG_HOME_TYPE:
    case TRAJ_CFG_LIMIT_TYPE:
    case TRAJ_CFG_PROFILE_TYPE:
    case TRAJ_CFG_KINEMATICS_TYPE:
    case TRAJ_CFG_SCALE_TYPE:
    case TRAJ_CFG_MAX_SCALE_TYPE:
    case TRAJ_CFG_LOG_TYPE:
    case TRAJ_CFG_LOG_START_TYPE:
    case TRAJ_CFG_LOG_STOP_TYPE:
    case TRAJ_CFG_TOOL_TRANSFORM_TYPE:
    case TRAJ_CFG_STUB_TYPE:
      traj_set.command_type = cfg_type;
      if (cfg_serial_number != traj_set.echo_serial_number) {
	traj_set.echo_serial_number = cfg_serial_number;
	traj_set.state = GO_RCS_STATE_NEW_COMMAND;
      }
      break;

    default:
      rtapi_print("trajloop: %s: unknown config %d\n",  BN, cfg_type);
      break;
    }

    switch (traj_stat.command_type) {
    case TRAJ_CMD_NOP_TYPE:
      do_cmd_nop(&traj_stat, &traj_set);
      break;

    case TRAJ_CMD_INIT_TYPE:
      do_cmd_init(&traj_stat, &traj_set, servo_cmd, servo_stat_ptr[0], &traj_motion_queue);
      break;

    case TRAJ_CMD_ABORT_TYPE:
      do_cmd_abort(&traj_stat, &traj_set, servo_cmd, servo_stat_ptr[0]);
      break;

    case TRAJ_CMD_HALT_TYPE:
      do_cmd_halt(&traj_stat, &traj_set, servo_cmd, servo_stat_ptr[0]);
      break;

    case TRAJ_CMD_SHUTDOWN_TYPE:
      do_cmd_shutdown(&traj_stat, &traj_set, servo_cmd, servo_stat_ptr[0]);
      break;

    case TRAJ_CMD_STOP_TYPE:
      do_cmd_stop(&traj_stat, &traj_set, traj_ref_ptr, servo_cmd, kinematics, &traj_motion_queue);
      break;

    case TRAJ_CMD_MOVE_JOINT_TYPE:
      do_cmd_move_joint(traj_cmd_ptr, &traj_stat, &traj_set, servo_cmd, servo_stat_ptr[0], servo_set_ptr[0], &traj_motion_queue);
      break;

    case TRAJ_CMD_MOVE_UJOINT_TYPE:
      do_cmd_move_ujoint(traj_cmd_ptr, &traj_stat, &traj_set, servo_cmd, servo_stat_ptr[0], &traj_motion_queue);
      break;

    case TRAJ_CMD_MOVE_WORLD_TYPE:
      do_cmd_move_world_or_tool(1, traj_cmd_ptr, &traj_stat, &traj_set, traj_ref_ptr, servo_cmd, servo_stat_ptr[0], kinematics, &traj_motion_queue);
      break;

    case TRAJ_CMD_MOVE_TOOL_TYPE:
      do_cmd_move_world_or_tool(0, traj_cmd_ptr, &traj_stat, &traj_set, traj_ref_ptr, servo_cmd, servo_stat_ptr[0], kinematics, &traj_motion_queue);
      break;

    case TRAJ_CMD_TRACK_WORLD_TYPE:
      do_cmd_track_world(traj_cmd_ptr, &traj_stat, &traj_set, traj_ref_ptr, servo_cmd, kinematics);
      break;

    case TRAJ_CMD_TRACK_JOINT_TYPE:
      do_cmd_track_joint(traj_cmd_ptr, &traj_stat, &traj_set, servo_cmd, servo_stat_ptr[0], servo_set_ptr[0]);
      break;

    case TRAJ_CMD_TELEOP_JOINT_TYPE:
      do_cmd_teleop_joint(traj_cmd_ptr, &traj_stat, &traj_set, servo_cmd, servo_stat_ptr[0], servo_set_ptr[0], &traj_motion_queue, joint_teleop_speed);
      break;

    case TRAJ_CMD_TELEOP_WORLD_TYPE:
      do_cmd_teleop_world_or_tool(1, traj_cmd_ptr, &traj_stat, &traj_set, servo_cmd, kinematics, &traj_motion_queue, &world_teleop_speed);
      break;

    case TRAJ_CMD_TELEOP_TOOL_TYPE:
      do_cmd_teleop_world_or_tool(0, traj_cmd_ptr, &traj_stat, &traj_set, servo_cmd, kinematics, &traj_motion_queue, &world_teleop_speed);
      break;

    case TRAJ_CMD_HERE_TYPE:
      do_cmd_here(traj_cmd_ptr, &traj_stat, &traj_set, servo_cmd, servo_stat_ptr[0], servo_cfg, servo_set_ptr[0], kinematics, &traj_motion_queue);
      break;

    case TRAJ_CMD_STUB_TYPE:
      do_cmd_stub(traj_cmd_ptr, &traj_stat, &traj_set, servo_cmd, servo_stat_ptr[0], servo_set_ptr[0]);
      break;

    default:
      break;
    }

    switch (traj_set.command_type) {
    case TRAJ_CFG_NOP_TYPE:
      do_cfg_nop(&traj_set);
      break;

    case TRAJ_CFG_CYCLE_TIME_TYPE:
      do_cfg_cycle_time(traj_cfg_ptr, &traj_set, servo_cfg, servo_set_ptr[0], &traj_motion_queue);
      break;

    case TRAJ_CFG_DEBUG_TYPE:
      do_cfg_debug(traj_cfg_ptr, &traj_set);
      break;

    case TRAJ_CFG_HOME_TYPE:
      do_cfg_home(&traj_stat, traj_cfg_ptr, &traj_set, kinematics);
      break;

    case TRAJ_CFG_LIMIT_TYPE:
      do_cfg_limit(traj_cfg_ptr, &traj_set);
      break;

    case TRAJ_CFG_PROFILE_TYPE:
      do_cfg_profile(traj_cfg_ptr, &traj_set);
      break;

    case TRAJ_CFG_KINEMATICS_TYPE:
      do_cfg_kinematics(traj_cfg_ptr, &traj_set, kinematics);
      break;

    case TRAJ_CFG_SCALE_TYPE:
      do_cfg_scale(traj_cfg_ptr, &traj_set, &traj_motion_queue);
      break;

    case TRAJ_CFG_MAX_SCALE_TYPE:
      do_cfg_max_scale(traj_cfg_ptr, &traj_set);
      break;

    case TRAJ_CFG_LOG_TYPE:
      do_cfg_log(&traj_stat, traj_cfg_ptr, &traj_set);
      break;

    case TRAJ_CFG_LOG_START_TYPE:
      do_cfg_log_start(&traj_stat, traj_cfg_ptr, &traj_set);
      break;

    case TRAJ_CFG_LOG_STOP_TYPE:
      do_cfg_log_stop(&traj_stat, traj_cfg_ptr, &traj_set);
      break;

    case TRAJ_CFG_TOOL_TRANSFORM_TYPE:
      do_cfg_tool_transform(&traj_stat, traj_cfg_ptr, &traj_set, &traj_motion_queue);
      break;

    case TRAJ_CFG_STUB_TYPE:
      do_cfg_stub(&traj_stat, traj_cfg_ptr, &traj_set);
      break;

    default:
      break;
    }

    /* update status */
    traj_stat.heartbeat++;
    go_motion_queue_number(&traj_motion_queue, &traj_stat.queue_count);
    rtapi_clock_get_time(&sec, &nsec);
    rtapi_clock_get_interval(old_sec, old_nsec,
			     sec, nsec,
			     &diff_sec, &diff_nsec);
    old_sec = sec, old_nsec = nsec;
    traj_stat.cycle_time = ((go_real) diff_sec) +
      ((go_real) diff_nsec) * 1.0e-9;

    /* update settings */
    traj_set.scale = traj_motion_queue.timescale.scale;

    /* writing of servo cmd, cfg is done in state tables */

    /* write out traj status and settings */
    traj_stat.tail = ++traj_stat.head;
    global_traj_comm_ptr->traj_stat = traj_stat;
    /*  */
    traj_set.tail = ++traj_set.head;
    global_traj_comm_ptr->traj_set = traj_set;

    if (traj_set.debug & DEBUG_POSITION) {
      for (servo_num = 0; servo_num < joint_num; servo_num++) {
	rtapi_print("trajloop: %f ", servo_stat_ptr[servo_num]->input);
      }
      rtapi_print("\n");
    }

    /* log any data requested that's not logged in the state tables */
    if (traj_set.log_logging) {
      if (traj_set.log_type == GO_LOG_ACT_POS) {
	entry.time = traj_timestamp();
	entry.u.act_pos.pos = traj_stat.ecp_act;
	go_log_add(global_go_log_ptr, &entry);
      } else if (traj_set.log_type == GO_LOG_CMD_POS) {
	entry.u.cmd_pos.pos = traj_stat.ecp;
	entry.time = traj_timestamp();
	go_log_add(global_go_log_ptr, &entry);
      } else if (traj_set.log_type == GO_LOG_XINV) {
	entry.u.xinv.xinv = traj_stat.xinv;
	entry.time = traj_timestamp();
	go_log_add(global_go_log_ptr, &entry);
      } else if (traj_set.log_type == GO_LOG_MAGXINV) {
	entry.u.magxinv.x = traj_stat.ecp_act.tran.x;
	entry.u.magxinv.y = traj_stat.ecp_act.tran.y;
	(void) go_cart_mag(&traj_stat.xinv.tran, &entry.u.magxinv.mag);
	entry.time = traj_timestamp();
	go_log_add(global_go_log_ptr, &entry);
      }	/* else don't log anything; logging may be active for something else */
    }

    /* record stop time, for perf measures */
    rtapi_clock_get_time(&end_sec, &end_nsec);
    rtapi_clock_get_interval(start_sec, start_nsec,
			     end_sec, end_nsec,
			     &diff_sec, &diff_nsec);
    calc_time = ((go_real) diff_sec) + ((go_real) diff_nsec) * 1.0e-9;
    go_mmavg_add(&traj_stat.mmavg, calc_time);

    if (traj_stat.admin_state == GO_RCS_ADMIN_STATE_SHUT_DOWN) {
      break;
    } else {
      rtapi_sem_take(servo_sem);
      TASK_PRINT_1("traj took semaphore\n");
    }
  } /* while (1) */

  PROG_PRINT_1("traj done\n");

  (void) rtapi_task_exit();

  return;
}
