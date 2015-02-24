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
  \file ext_profi.c

  \brief External interface functions for the Profibus I/O board, with
  simulated motors standing in for joints that are not to be controlled
  by the Profibus.
*/

/*
  FIXME--

  DONE: Make ext_init a single-time call, and change the current ext_init to be ext_joint_init; ditto for ext_quit

  Look at a per-cycle ext_cycle_start() function, called once before any servos run; ditto for ext_cycle_end().

  DONE: Add ext_joint_enable, ext_joint_disable calls to servoloop.

  Fix shutdown so that ext_quit is called.

  Look at rtlib/Makefile and figure out how to select different ext interfaces.

  Reload .ini file.

  Touched files: 
  ext_profi.c (new)
  robocrane.c,h (new)
  ProfibusIOInterface.c,h (new)
  bin/run
  etc/profi.ini (new)
  rtlib/Makefile
  rtlib/SETUP
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <math.h>
#include "gotypes.h"
#include "extintf.h"
#include "dcmotor.h"

#include "robocrane.h"    /* Defines RoboCrane's structures and initialization functions */
RoboCrane_Type RoboCrane; /* Main global RoboCrane structure */
#include "ProfibusIOInterface.h"

#define NUM_JOINTS 8

/* flags for when to use real Profibus board, otherwise DC motor simulation */
static int UseProfi(int joint)
{
  if (joint < 0 || joint > 5) return 0;

  return 1;
}

static dcmotor_params params[NUM_JOINTS];

static go_real old_pos[NUM_JOINTS];

static go_flag joint_is_homing[NUM_JOINTS];
static go_flag joint_is_homed[NUM_JOINTS];
static go_real joint_home_latch[NUM_JOINTS];

/*
  Inland Motor BM-3503, peak torque 109 N-m
  ---------------------------
  armature resistance Ra =           0.028 (ohms)
  armature inductance La =           0.00035 (henries)
  back emf constant Kb =             0.414 (volts/radian/sec, N-m/amp)
  rotor inertia Jm =                 0.00707 (N-m/rad/sec^2)
  damping friction coefficient Bm =  6.129 (N-m/rad/sec)
*/

go_result ext_init(char * init_string)
{
  RoboCraneInitialize(&RoboCrane); /* Initialize the RoboCrane structure */
  ProfiInit();	     /* Initialize the Profibus communications */

  return GO_RESULT_OK;
}
 
go_result ext_quit(void)
{
  ProfiCleanup();	       /* Close the Profibus communications */

  return GO_RESULT_OK;
}

go_result ext_joint_init(go_integer joint, go_real cycle_time)
{
  if (joint < 0 || joint >= NUM_JOINTS) return GO_RESULT_ERROR;

  if (! UseProfi(joint)) {
    dcmotor_init(&params[joint],
		 6.129,		/* Bm */
		 0.00035,	/* la */
		 0.028,		/* Ra */
		 0.00707,	/* Jm */
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
  }

  /* clear our home flags */
  joint_is_homing[joint] = 0;
  joint_is_homed[joint] = 0;
  joint_home_latch[joint] = 0.0;

  /* enable the Profi amps */
  ext_joint_enable(joint);

  return GO_RESULT_OK;
}

go_result ext_joint_enable(go_integer joint)
{
  if (UseProfi(joint)) {
    RoboCrane.cmd.jointCmd[joint].AmpEnable = 1;
    UpdateOutputDataBuffer();	/* Build Profibus output buffer */
    ProfiDataCommProcess(WRITE_MASK); /* Send Profibus output buffer */
  }

  return GO_RESULT_OK;
}

go_result ext_joint_disable(go_integer joint)
{
  if (UseProfi(joint)) {
    RoboCrane.cmd.jointCmd[joint].AmpEnable = 0;
    UpdateOutputDataBuffer();	/* Build Profibus output buffer */
    ProfiDataCommProcess(WRITE_MASK); /* Send Profibus output buffer */
  }

  return GO_RESULT_OK;
}

go_result ext_read_pos(go_integer joint, go_real * pos)
{
  go_real theta;			/* angular position */
  go_real dtheta;		/* angular velocity */
  go_real d2theta;		/* angular acceleration */

  if (joint < 0 || joint >= NUM_JOINTS) return GO_RESULT_ERROR;

  /* call any function that must be called each cycle, regardless of joint; put it
     in the 0th joint handler since that gets called first. */
  if (0 == joint) {
    ProfiDataCommProcess(READ_MASK); /* Read Profibus intput buffer (READ_MASK and WRITE_MASK are defined in ProfibusIOInterface.c) */
  }

  if (UseProfi(joint)) {
    UpdateEncoderCount(joint); /* Update encoder counts, where axis is 0 to 5 */
    theta = (go_real) RoboCrane.status.jointStatus[joint].EncoderCnt;
  } else {
    (void) dcmotor_get(&params[joint], &theta, &dtheta, &d2theta);
  }

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

  if (UseProfi(joint)) {
    RoboCrane.cmd.jointCmd[joint].CmdMotorVolt = (float) vel;
  } else {
    /* clock the simulation to bring us to our new position */
    (void) dcmotor_run_current_cycle(&params[joint], vel);
  }

  /* do this once after all joints are finished */
  if (5 == joint) {
    UpdateOutputDataBuffer();	/* Build Profibus output buffer */
    ProfiDataCommProcess(WRITE_MASK); /* Send Profibus output buffer */
  }

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

go_result ext_joint_quit(go_integer joint)
{
  ext_joint_disable(joint);

  return GO_RESULT_OK;
}

enum {AIN_NUM = 0};
enum {AOUT_NUM = 0};
enum {DIN_NUM = 0};
enum {DOUT_NUM = 0};

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
  return GO_RESULT_OK;
}

go_result ext_read_ain(go_integer index, go_real * val)
{
  *val = 0.0;

  return GO_RESULT_OK;
}

go_result ext_write_aout(go_integer index, go_real val)
{
  return GO_RESULT_OK;
}

go_result ext_read_din(go_integer index, go_flag * val)
{
  *val = 0;

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
