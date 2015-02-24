/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

/*!
  \file ext_sim.c

  \brief External interface implementation for simulated motors.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <math.h>
#include "gotypes.h"
#include "extintf.h"
#include "dcmotor.h"

#define NUM_JOINTS 8

static dcmotor_params params[NUM_JOINTS];

static go_real old_pos[NUM_JOINTS];

static go_real joint_home_latch[NUM_JOINTS];
static go_flag joint_is_homing[NUM_JOINTS];
static go_flag joint_is_homed[NUM_JOINTS];
static go_flag joint_home_immediate;

enum {AIN_NUM = 8};
enum {AOUT_NUM = 8};
enum {DIN_NUM = 16};
enum {DOUT_NUM = 16};

static go_real ain_data[AIN_NUM];
static go_real ain_incr[AIN_NUM];
static go_flag din_data[DIN_NUM];

/*
  Inland Motor BM-3503, peak torque 109 N-m
  ---------------------------
  armature resistance Ra =           0.028 (ohms)
  armature inductance La =           0.00035 (henries)
  back emf constant Kb =             0.414 (volts/radian/sec, N-m/amp)
  rotor inertia Jm =                 0.00707 (N-m/rad/sec^2)
  damping friction coefficient Bm =  6.129 (N-m/rad/sec)
*/

/*
  The init string is a mix of different configurable flags for testing:

  init_string[0] = 'I' for immediate homing, all joints.
*/

go_result ext_init(char * init_string)
{
  go_integer i;

  joint_home_immediate = (init_string[0] == 'I');

  for (i = 0; i < AIN_NUM; i++) {
    ain_data[i] = 0.0;
    ain_incr[i] = (i + 1) * 0.001;
  }

  for (i = 0; i < DIN_NUM; i++) {
    din_data[i] = 0;
  }

  return GO_RESULT_OK;
}
 
go_result ext_quit(void)
{
  return GO_RESULT_OK;
}

go_result ext_joint_init(go_integer joint, go_real cycle_time)
{
  if (joint < 0 || joint >= NUM_JOINTS) return GO_RESULT_ERROR;

  dcmotor_init(&params[joint],
	      6.129,		/* Bm */
	      0.00035,		/* La */
	      0.028,		/* Ra */
	      0.00707,		/* Jm */
	      0.414,		/* Kb */
	      0.0,		/* Tl, load torque */
	      0.0,		/* Tk, static friction torque */
	      0.0,		/* Ts, sliding friction torque */
	      cycle_time	/* cycle time */
	      );

  /* set the starting position to be something random, in this case
     the joint number */
  dcmotor_set_theta(&params[joint], (go_real) joint);
  old_pos[joint] = (go_real) joint;

  /* clear our home flags */
  joint_is_homing[joint] = 0;
  joint_is_homed[joint] = 0;
  joint_home_latch[joint] = 0.0;

  return GO_RESULT_OK;
}

go_result ext_joint_enable(go_integer joint)
{
  return GO_RESULT_OK;
}

go_result ext_joint_disable(go_integer joint)
{
  return GO_RESULT_OK;
}

go_result ext_joint_quit(go_integer joint)
{
  return GO_RESULT_OK;
}

go_result ext_read_pos(go_integer joint, go_real * pos)
{
  go_real theta;			/* angular position */
  go_real dtheta;		/* angular velocity */
  go_real d2theta;		/* angular acceleration */

  if (joint < 0 || joint >= NUM_JOINTS) return GO_RESULT_ERROR;

  (void) dcmotor_get(&params[joint], &theta, &dtheta, &d2theta);

  *pos = theta;

  return GO_RESULT_OK;
}

go_result ext_write_pos(go_integer joint, go_real pos)
{
  return GO_RESULT_IMPL_ERROR;
}

go_result ext_write_vel(go_integer joint, go_real vel)
{
  if (joint < 0 || joint >= NUM_JOINTS) return GO_RESULT_ERROR;

  /* save our old position */
  (void) ext_read_pos(joint, &old_pos[joint]);

  /* clock the simulation to bring us to our new position */
  (void) dcmotor_run_current_cycle(&params[joint], vel);

  return GO_RESULT_OK;
}

go_result ext_joint_home(go_integer joint)
{
  if (joint < 0 || joint >= NUM_JOINTS) return GO_RESULT_ERROR;

  joint_is_homing[joint] = 1;
  joint_is_homed[joint] = 0;
  
  return GO_RESULT_OK;
}

#define ROLLOVER 0.1

go_flag ext_joint_is_home(go_integer joint)
{
  go_real m, old_bin, now_pos, now_bin;

  if (joint < 0 || joint >= NUM_JOINTS) return 1;

  if (joint_is_homed[joint]) return 1;

  if (! joint_is_homing[joint]) return 0;

  /* else we're homing */

  if (joint_home_immediate) return 1;

  m = fmod(old_pos[joint], ROLLOVER);
  if (m < 0.0) m += ROLLOVER;
  old_bin = old_pos[joint] - m;

  (void) ext_read_pos(joint, &now_pos);
  m = fmod(now_pos, ROLLOVER);
  if (m < 0.0) m += ROLLOVER;
  now_bin = now_pos - m;

  if (old_bin != now_bin) {
    /* we moved across a rollover, so call us homed */
    joint_is_homing[joint] = 0;
    joint_is_homed[joint] = 1;
    joint_home_latch[joint] = now_bin;
    return 1;
  }

  return 0;
}

go_result ext_joint_home_latch(go_integer joint, go_real * pos)
{
  if (joint < 0 || joint >= NUM_JOINTS) return GO_RESULT_ERROR;

  *pos = joint_home_latch[joint];

  return GO_RESULT_OK;
}

go_integer ext_num_ain(void)
{
  return AIN_NUM;
}

go_integer ext_num_aout(void)
{
  return AOUT_NUM;
}

go_integer ext_num_din(void)
{
  return DIN_NUM;
}

go_integer ext_num_dout(void)
{
  return DOUT_NUM;
}

go_result ext_trigger_in(void)
{
  go_integer i;

  /* run the AINs through some back-and-forth cycling */
  for (i = 0; i < AIN_NUM; i++) {
    ain_data[i] += ain_incr[i];
    if (ain_data[i] > 10.0) {
      ain_data[i] = 10.0;
      ain_incr[i] = -ain_incr[i];
    } else if (ain_data[i] < -10.0) {
      ain_data[i] = -10.0;
      ain_incr[i] = -ain_incr[i];
    }
  }

  /* run the DINs through some toggling */
  for (i = 0; i < DIN_NUM; i++) {
    if (i < AIN_NUM) {
      if (ain_incr[i] < 0.0) {
	din_data[i] = 0;
      } else {
	din_data[i] = 1;
      }
    }
    /* else we don't have any more AINs to reference, so leave at 0 */
  }

  return GO_RESULT_OK;
}

go_result ext_read_ain(go_integer index, go_real * val)
{
  if (index < 0 || index >= AIN_NUM) return GO_RESULT_RANGE_ERROR;

  *val = ain_data[index];

  return GO_RESULT_OK;
}

go_result ext_write_aout(go_integer index, go_real val)
{
  return GO_RESULT_OK;
}

go_result ext_read_din(go_integer index, go_flag * val)
{
  if (index < 0 || index >= DIN_NUM) return GO_RESULT_RANGE_ERROR;

  *val = din_data[index];

  return GO_RESULT_OK;
}

go_result ext_write_dout(go_integer index, go_flag val)
{
  return GO_RESULT_OK;
}

go_result ext_set_parameters(go_integer joint, go_real * values, go_integer number)
{
  /* nothing to do */
  return GO_RESULT_OK;
}
