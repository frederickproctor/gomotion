/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#ifndef SERVOINTF_H
#define SERVOINTF_H

#include "go.h"			/* GO_MOTION_JOINT_NUM */
#include "gorcs.h"		/* GO_RCS_CMD,STAT_MSG */
#include "pid.h"		/* PidStruct */

#define DEFAULT_SERVO_SHM_KEY 101

enum {
  SERVO_CMD_NOP_TYPE = SERVO_CMD_BASE + 1,
  SERVO_CMD_INIT_TYPE,
  SERVO_CMD_ABORT_TYPE,
  SERVO_CMD_HALT_TYPE,
  SERVO_CMD_SHUTDOWN_TYPE,
  SERVO_CMD_SERVO_TYPE,
  SERVO_CMD_STUB_TYPE
};

#define servo_cmd_symbol(x) \
(x) == SERVO_CMD_NOP_TYPE ? "NOP" : \
(x) == SERVO_CMD_INIT_TYPE ? "Init" : \
(x) == SERVO_CMD_ABORT_TYPE ? "Abort" : \
(x) == SERVO_CMD_HALT_TYPE ? "Halt" : \
(x) == SERVO_CMD_SHUTDOWN_TYPE ? "Shutdown" : \
(x) == SERVO_CMD_SERVO_TYPE ? "Servo" : \
(x) == SERVO_CMD_STUB_TYPE ? "Stub" : "?"

enum {
  SERVO_STAT_TYPE = SERVO_STAT_BASE + 1
};

enum {
  SERVO_CFG_NOP_TYPE = SERVO_CFG_BASE + 1,
  SERVO_CFG_CYCLE_TIME_TYPE,
  SERVO_CFG_CYCLE_MULT_TYPE,
  SERVO_CFG_ACTIVE_TYPE,
  SERVO_CFG_PID_TYPE,
  SERVO_CFG_PARAMETERS_TYPE,
  SERVO_CFG_LINK_TYPE,
  SERVO_CFG_DEBUG_TYPE,
  SERVO_CFG_HOME_TYPE,
  SERVO_CFG_INPUT_SCALE_TYPE,
  SERVO_CFG_OUTPUT_SCALE_TYPE,
  SERVO_CFG_LIMIT_TYPE,
  SERVO_CFG_PROFILE_TYPE,
  SERVO_CFG_LOG_TYPE,
  SERVO_CFG_LOG_START_TYPE,
  SERVO_CFG_LOG_STOP_TYPE,
  SERVO_CFG_SERVO_TYPE_TYPE,
  SERVO_CFG_STUB_TYPE
};

#define servo_cfg_symbol(x) \
(x) == SERVO_CFG_NOP_TYPE ? "NOP" : \
(x) == SERVO_CFG_CYCLE_TIME_TYPE ? "CycleTime" : \
(x) == SERVO_CFG_CYCLE_MULT_TYPE ? "CycleMult" : \
(x) == SERVO_CFG_ACTIVE_TYPE ? "Active" : \
(x) == SERVO_CFG_PID_TYPE ? "Pid" : \
(x) == SERVO_CFG_PARAMETERS_TYPE ? "Parameters" : \
(x) == SERVO_CFG_LINK_TYPE ? "Link" : \
(x) == SERVO_CFG_DEBUG_TYPE ? "Debug" : \
(x) == SERVO_CFG_HOME_TYPE ? "Home" : \
(x) == SERVO_CFG_LIMIT_TYPE ? "Limit" : \
(x) == SERVO_CFG_PROFILE_TYPE ? "Profile" : \
(x) == SERVO_CFG_LIMIT_TYPE ? "Limit" : \
(x) == SERVO_CFG_INPUT_SCALE_TYPE ? "InputScale" : \
(x) == SERVO_CFG_OUTPUT_SCALE_TYPE ? "OutputScale" : \
(x) == SERVO_CFG_LOG_TYPE ? "LogCfg" : \
(x) == SERVO_CFG_LOG_START_TYPE ? "LogStart" : \
(x) == SERVO_CFG_LOG_STOP_TYPE ? "LogStop" : \
(x) == SERVO_CFG_SERVO_TYPE_TYPE ? "ServoType" : \
(x) == SERVO_CFG_STUB_TYPE ? "Stub" : "?"

enum {
  SERVO_SET_TYPE = SERVO_SET_BASE + 1
};

typedef struct {
  go_real setpoint;
  go_flag home;		/*!< non-zero means undertake homing  */
} servo_cmd_servo;

typedef struct {
  go_integer arg;	      /*!< an integer argument */
} servo_cmd_stub;

typedef struct {
  unsigned char head;
  GO_RCS_CMD_MSG;
  union {
    servo_cmd_servo servo;
    servo_cmd_stub stub;
  } u;
  unsigned char tail;
} servo_cmd_struct;

typedef struct {
  unsigned char head;
  GO_RCS_STAT_MSG;
  go_real setpoint;
  go_real raw_input;
  go_real raw_output;
  go_real input;		/*!< scaled input, but not offset */
  go_real input_latch;		/*!< position where homed happened */
  go_real input_vel;
  go_real output;
  go_real ferror;		/*!< following error */
  go_real cycle_time;		/*!< actual cycle time */
  go_integer heartbeat;		/*!< incremented each cycle */
  go_flag enable;		/*!< enable output */
  go_flag homing;		/*!< homing is happening */
  go_flag homed;		/*!< homing happened */
  unsigned char tail;
} servo_stat_struct;

typedef struct {
  go_real cycle_time;
} servo_cfg_cycle_time;

typedef struct {
  go_integer cycle_mult;
} servo_cfg_cycle_mult;

typedef struct {
  go_link link;
} servo_cfg_link;

typedef struct {
  go_integer debug;
} servo_cfg_debug;

typedef struct {
  go_flag active;
} servo_cfg_active;

enum {GO_SERVO_PARAMETER_MAX = 10};

typedef struct {
  go_real parameters[GO_SERVO_PARAMETER_MAX];
  go_integer number;
} servo_cfg_parameters;

typedef struct {
  go_real home;
} servo_cfg_home;

typedef struct {
  go_real scale;
} servo_cfg_scale;

typedef struct {
  go_real min_limit;
  go_real max_limit;
} servo_cfg_limit;

typedef struct {
  go_real max_vel;
  go_real max_acc;
  go_real max_jerk;
} servo_cfg_profile;

typedef struct {
  go_integer log_type;
  go_integer log_size;
} servo_cfg_log;

typedef struct {
  go_integer arg;
} servo_cfg_stub;

enum {
  GO_SERVO_TYPE_PID = 1,	/*!< for PID servos */
  GO_SERVO_TYPE_PASS		/*!< for passing through to external servos  */
};

typedef struct {
  go_flag servo_type;	 /*! < one of GO_SERVO_TYPE_PID, PASS, ...  */
} servo_cfg_servo_type;

typedef struct {
  unsigned char head;
  GO_RCS_CMD_MSG;
  union {
    servo_cfg_cycle_time cycle_time;
    servo_cfg_cycle_mult cycle_mult;
    servo_cfg_active active;
    pid_struct pid;
    servo_cfg_parameters parameters;
    servo_cfg_link link;
    servo_cfg_debug debug;
    servo_cfg_home home;
    servo_cfg_scale scale;
    servo_cfg_limit limit;
    servo_cfg_profile profile;
    servo_cfg_log log;
    servo_cfg_servo_type servo_type;
    servo_cfg_stub stub;
  } u;
  unsigned char tail;
} servo_cfg_struct;

typedef struct {
  unsigned char head;
  GO_RCS_STAT_MSG;
  go_link link;
  pid_struct pid;
  go_real cycle_time;		/* nominal cycle time */
  go_real cycle_mult_inv;
  go_real home;			/* nominal position when homed */
  go_real input_scale;
  go_real output_scale;
  go_real min_limit;
  go_real max_limit;
  go_real max_vel;
  go_real max_acc;
  go_real max_jerk;
  go_integer id;
  go_integer cycle_mult;
  go_integer debug;
  go_integer log_type;		/* what we're logging, also in the log */
  go_integer log_logging;	/* are we logging */
  go_flag active;
  go_flag servo_type;		/*!< the type of servo algorithm being used  */
  unsigned char tail;
} servo_set_struct;

#define SERVO_NUM 7

#if SERVO_NUM > GO_MOTION_JOINT_NUM
#error SERVO_NUM is greater than GO_MOTION_JOINT_NUM
#endif

/* there are SERVO_NUM of these servo_comm_structs in shared memory */
typedef struct {
  servo_cmd_struct servo_cmd;
  servo_stat_struct servo_stat;
  servo_cfg_struct servo_cfg;
  servo_set_struct servo_set;
} servo_comm_struct;

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

extern servo_comm_struct * global_servo_comm_ptr;

extern void * servo_sem;

extern void servo_loop(void *);

/*!
  Run one pass of the servo calculations in response to a \a
  servo_cmd_servo command.  It generates an output for the joint so that
  the joint tracks the given setpoint in the \a cmd sent by Prim. The
  output is stored in \a stat.  The settings \a set contain the gains
  to be used for the servo calculations.

  Typically, \a do_cmd_servo runs several cycles between each Prim
  cycle so interpolation is necessary. At the first cycle, \b
  NEW_COMMAND, it adds the given setpoint to the the interpolator \a
  interp, sets \a interp_s to 0 and evaluates the intepolation. \a
  interp_s is then incremented by the inverse of the cycle time for
  the next cycle, so that \a interp_s ranges from exactly 0 to
  something less than 1.

  Each time Prim sends a \a servo_cmd_servo command, it should increment
  the serial number to ensure that a new point is added to the
  interpolator. This way the interpolation will be finished when a new
  point arrives. Interpolation is robust, so that if Prim sends a new
  point before the interpolation has been fully carried out, it acts
  as if the preceding interpolation completed immediately. If Prim is
  late, i.e. if \a interp_s reaches 1, \a do_cmd_servo will add the
  current interpolated point onto the interpolator automatically and
  reset \a interp_s to 0. This results in Servo tracking the last
  setpoint indefinitely in the absence of further setpoints.
 */

extern void
do_cmd_servo(
	   servo_cmd_struct * cmd,    /*!< the input command, with setpoint. */
	   servo_stat_struct * stat,	/*!< the current status */
	   servo_set_struct * set,    /*!< the current settings, with gains */
	   go_interp * interp,	/*!< the interpolator used */
	   go_real * interp_s	/*!< the interpolation parameter */
	   );	

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif
