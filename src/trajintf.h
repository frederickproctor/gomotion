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
  \defgroup HOMING Homing a Joint

  The servo controllers work entirely in their initial frame. When
  they are homed, the position at which they homed is stored in a
  latch variable. The trajectory controller can then change to the
  homed frame, and adjust with an offset to maintain initial frame
  values for servo.
*/

#ifndef TRAJINTF_H
#define TRAJINTF_H

#include "go.h"			/* go_pose */
#include "gorcs.h"		/* GO_RCS_CMD,STAT_MSG */
#include "servointf.h"		/* SERVO_NUM */

#define DEFAULT_TRAJ_SHM_KEY 201

enum {
  TRAJ_CMD_NOP_TYPE = TRAJ_CMD_BASE + 1,
  TRAJ_CMD_INIT_TYPE,
  TRAJ_CMD_ABORT_TYPE,
  TRAJ_CMD_HALT_TYPE,
  TRAJ_CMD_SHUTDOWN_TYPE,
  TRAJ_CMD_STOP_TYPE,
  TRAJ_CMD_MOVE_WORLD_TYPE,
  TRAJ_CMD_MOVE_TOOL_TYPE,  
  TRAJ_CMD_MOVE_JOINT_TYPE,
  TRAJ_CMD_MOVE_UJOINT_TYPE,
  TRAJ_CMD_TRACK_WORLD_TYPE,
  TRAJ_CMD_TRACK_JOINT_TYPE,
  TRAJ_CMD_TELEOP_JOINT_TYPE,
  TRAJ_CMD_TELEOP_WORLD_TYPE,
  TRAJ_CMD_TELEOP_TOOL_TYPE,
  TRAJ_CMD_HERE_TYPE,
  TRAJ_CMD_STUB_TYPE
};

#define traj_cmd_symbol(x) \
(x) == TRAJ_CMD_NOP_TYPE ? "NOP" : \
(x) == TRAJ_CMD_INIT_TYPE ? "Init" : \
(x) == TRAJ_CMD_ABORT_TYPE ? "Abort" : \
(x) == TRAJ_CMD_HALT_TYPE ? "Halt" : \
(x) == TRAJ_CMD_SHUTDOWN_TYPE ? "Shutdown" : \
(x) == TRAJ_CMD_STOP_TYPE ? "Stop" : \
(x) == TRAJ_CMD_MOVE_WORLD_TYPE ? "Move World" : \
(x) == TRAJ_CMD_MOVE_TOOL_TYPE ? "Move Tool" : \
(x) == TRAJ_CMD_MOVE_JOINT_TYPE ? "Move Joint" : \
(x) == TRAJ_CMD_MOVE_UJOINT_TYPE ? "Move UJoint" : \
(x) == TRAJ_CMD_TRACK_WORLD_TYPE ? "Track World" : \
(x) == TRAJ_CMD_TRACK_JOINT_TYPE ? "Track Joint" : \
(x) == TRAJ_CMD_TELEOP_JOINT_TYPE ? "Teleop Joint" : \
(x) == TRAJ_CMD_TELEOP_WORLD_TYPE ? "Teleop World" : \
(x) == TRAJ_CMD_TELEOP_TOOL_TYPE ? "Teleop Tool" : \
(x) == TRAJ_CMD_HERE_TYPE ? "Here" : \
(x) == TRAJ_CMD_STUB_TYPE ? "Stub" : "?"

enum {
  TRAJ_STAT_TYPE = TRAJ_STAT_BASE + 1
};

enum {
  TRAJ_CFG_NOP_TYPE = TRAJ_CFG_BASE + 1,
  TRAJ_CFG_CYCLE_TIME_TYPE,
  TRAJ_CFG_DEBUG_TYPE,
  TRAJ_CFG_HOME_TYPE,
  TRAJ_CFG_LIMIT_TYPE,
  TRAJ_CFG_PROFILE_TYPE,
  TRAJ_CFG_KINEMATICS_TYPE,
  TRAJ_CFG_SCALE_TYPE,
  TRAJ_CFG_MAX_SCALE_TYPE,
  TRAJ_CFG_LOG_TYPE,
  TRAJ_CFG_LOG_START_TYPE,
  TRAJ_CFG_LOG_STOP_TYPE,
  TRAJ_CFG_TOOL_TRANSFORM_TYPE,
  TRAJ_CFG_STUB_TYPE
};

#define traj_cfg_symbol(x) \
(x) == TRAJ_CFG_NOP_TYPE ? "NOP" : \
(x) == TRAJ_CFG_CYCLE_TIME_TYPE ? "CycleTime" : \
(x) == TRAJ_CFG_DEBUG_TYPE ? "Debug" : \
(x) == TRAJ_CFG_HOME_TYPE ? "Home" : \
(x) == TRAJ_CFG_LIMIT_TYPE ? "Limit" : \
(x) == TRAJ_CFG_PROFILE_TYPE ? "Profile" : \
(x) == TRAJ_CFG_KINEMATICS_TYPE ? "Kinematics" : \
(x) == TRAJ_CFG_SCALE_TYPE ? "Scale" : \
(x) == TRAJ_CFG_MAX_SCALE_TYPE ? "MaxScale" : \
(x) == TRAJ_CFG_LOG_TYPE ? "LogCfg" : \
(x) == TRAJ_CFG_LOG_START_TYPE ? "LogStart" : \
(x) == TRAJ_CFG_LOG_STOP_TYPE ? "LogStop" : \
(x) == TRAJ_CFG_TOOL_TRANSFORM_TYPE ? "ToolTransform" : \
(x) == TRAJ_CFG_STUB_TYPE ? "Stub" : "?"

enum {
  TRAJ_SET_TYPE = TRAJ_SET_BASE + 1
};

/*!
  The \a traj_cmd_move_world moves the mechanism to the goal pose \a
  end, expressed in world coordinates. If \a time in seconds is
  positive, the move will be scaled to take that amount of time if
  possible, or longer if the system is constrained by max vel, accel
  and jerk. If \a time is not positive, then the motion parameters \a
  tv, \a ta, \a tj, \a rv, \a ra and \a rj are used for the
  translational and rotational vel, accel and jerk, respectively.
*/
typedef struct {
  go_integer id;		/*!< motion id */
  go_flag type;			/*!< GO_MOTION_LINEAR,CIRCULAR */
  go_real tv, ta, tj;		/*!< trans vel, acc and jerk */
  go_real rv, ra, rj;		/*!< rot vel, acc and jerk */
  go_real time;			/*!< time for move */
  go_pose end;			/*!< goal pose in world coordinates */
  go_cart center;		/*!< vector to center, for circles  */
  go_cart normal;		/*!< normal vector, for circles */
  go_integer turns;		/*!< how many turns, for circles  */
} traj_cmd_move_world;

/*!
  The \a traj_cmd_move_tool moves the mechanism to the goal pose \a
  end, expressed in tool coordinates. These will be incremental moves,
  since at the end of the move, the tool is at its own origin.  If \a
  time in seconds is positive, the move will be scaled to take that
  amount of time if possible, or longer if the system is constrained
  by max vel, accel and jerk. If \a time is not positive, then the
  motion parameters \a tv, \a ta, \a tj, \a rv, \a ra and \a rj are
  used for the translational and rotational vel, accel and jerk,
  respectively.
*/
typedef struct {
  go_integer id;		/*!< motion id */
  go_flag type;			/*!< GO_MOTION_LINEAR,CIRCULAR */
  go_real tv, ta, tj;		/*!< trans vel, acc and jerk */
  go_real rv, ra, rj;		/*!< rot vel, acc and jerk */
  go_real time;			/*!< time for move */
  go_pose end;			/*!< goal pose in tool coordinates */
  go_cart center;		/*!< vector to center, for circles  */
  go_cart normal;		/*!< normal vector, for circles */
  go_integer turns;		/*!< how many turns, for circles  */
} traj_cmd_move_tool;

typedef struct {
  go_integer id;		/* motion id for the joint moves */
  /* The following d, v, a, j are the distance, vel, acc and jerk for
     each joint, in [m] for translations, [rad] for rotational. 
     'd' is absolute, and can be positive or negative.
     'v', 'a' and 'j' must be positive. */
  go_real d[SERVO_NUM], v[SERVO_NUM], a[SERVO_NUM], j[SERVO_NUM];
  go_real time;			/*!< time for move */
} traj_cmd_move_joint;

typedef struct {
  go_integer id;		/* motion id for the joint moves */
  /* The following d, v, a, j are the distance, vel, acc and jerk for
     each joint, in [m] for translations, [rad] for rotational. 
     'd' is absolute, and can be positive or negative.
     'v', 'a' and 'j' must be positive. */
  go_real d[SERVO_NUM], v[SERVO_NUM], a[SERVO_NUM], j[SERVO_NUM];
  /* non-zero means this axis is homing */
  go_flag home[SERVO_NUM];
} traj_cmd_move_ujoint;

/*!
  traj_cmd_teleop_joint gives an array of joint speeds, accelerations
  for immediate following in joint space. The controller will clamp
  joint positions to lie within the joint limits.
 */
typedef struct {
  go_real v[SERVO_NUM];		/*!< pos or neg speed  */
  go_real a[SERVO_NUM];		/*!< pos accel  */
  go_real j[SERVO_NUM];		/*!< pos jerk */
} traj_cmd_teleop_joint;

/*!
  traj_cmd_teleop_world gives poses for speed, acceleration and jerk
  with respect to the world frame for immediate following in Cartesian
  space. The controller will clamp the position to lie within limits.
 */
typedef struct {
  go_vel tv;			/*!< both trans v and angular w */
  go_real ta, tj;		/*!< trans accel and jerk */
  go_real ra, rj;		/*!< rot accel and jerk */
} traj_cmd_teleop_world;

/*!
  traj_cmd_teleop_tool gives poses for speed, acceleration and jerk
  with respect to the tool frame for immediate following in Cartesian
  space. The controller will clamp the position to lie within limits.
 */
typedef struct {
  go_vel tv;			/*!< both trans v and angular w */
  go_real ta, tj;		/*!< trans accel and jerk */
  go_real ra, rj;		/*!< rot accel and jerk */
} traj_cmd_teleop_tool;

/*!
  traj_cmd_here sets the Cartesian position to be \a here and
  becomes homed.
 */
typedef struct {
  go_pose here;			/*!< the current pose */
} traj_cmd_here;

/*!
  traj_cmd_stub is a template for copying and changing to add
  a new command. This should never be executed, but if it is
  it has the effect of sending stub commands to each servo and
  waiting until they're done.
 */
typedef struct {
  go_integer arg;		/*!< an integer argument */
} traj_cmd_stub;

/*!
  Moves the device immediately to the target \a position, with no
  speed, acceleration or jerk profiling. The \a position is intended
  to vary slowly at human speeds, e.g., originating from a digitizing
  wand.
 */
typedef struct {
  go_pose position;		/*!< where to go, immediately  */
} traj_cmd_track_world;

/*!
  Moves the device immediately to the target \a joints, with no
  speed, acceleration or jerk profiling. The \a joints are intended
  to vary slowly at human speeds, e.g., originating from a master.
 */
typedef struct {
  go_real joints[SERVO_NUM];	/*!< where to go, immediately  */
} traj_cmd_track_joint;

typedef struct {
  unsigned char head;
  GO_RCS_CMD_MSG;
  union {
    traj_cmd_move_world move_world;
    traj_cmd_move_tool move_tool;
    traj_cmd_move_joint move_joint;
    traj_cmd_move_ujoint move_ujoint;
    traj_cmd_track_world track_world;
    traj_cmd_track_joint track_joint;
    traj_cmd_teleop_joint teleop_joint;
    traj_cmd_teleop_world teleop_world;
    traj_cmd_teleop_tool teleop_tool;
    traj_cmd_here here;
    traj_cmd_stub stub;
  } u;
  unsigned char tail;
} traj_cmd_struct;

enum {TRAJ_WORLD_FRAME = 1,
      TRAJ_JOINT_FRAME};

typedef struct {
  unsigned char head;
  GO_RCS_STAT_MSG;
  go_integer heartbeat;
  go_flag homed;
  go_flag frame;	 /*!<reference frame, TRAJ_WORLD,JOINT_FRAME */
  go_flag inpos;	   /*!<non-zero means in position, move done */
  go_real cycle_time;		/*!< actual cycle time */
  go_pose ecp;			/*!< the commanded end control point */
  go_pose ecp_act;		/*!< the actual end control point */
  go_pose kcp;			/*!< the kinematic control point */
  go_pose xinv;			/*!< inverse of nominal-to-actual transform */
  go_real joints[SERVO_NUM];	/*!< the commanded joints (offset) */
  go_real joints_act[SERVO_NUM]; /*!< the actual joints (offset) */
  go_real joints_ferror[SERVO_NUM]; /*!< the joint following errors */
  /*!
    The \a joint_offsets[] array holds the offsets from the servo home
    positions, \a input_latch[] in the arbitrary original coordinate
    system, to Traj's nominal home values. Coordinated joint and world
    moves are in Traj's homed coordinate system, and these offsets are
    added to interpolated points before sending to Servo, i.e.,

    nominal home + \a joint_offsets = \a input_latch, or

    \a joint_offsets = \a input_latch - nominal home, or

    \a joints = servo input - \a joint_offsets, or

    servo setpoints = \a joints + \a joint_offsets
  */
  go_real joint_offsets[SERVO_NUM];
  go_mmavg mmavg;
  go_integer queue_count;	/*<! how many moves on the motion queue  */
  unsigned char tail;
} traj_stat_struct;

typedef struct {
  go_real cycle_time;
} traj_cfg_cycle_time;

typedef struct {
  go_integer debug;
} traj_cfg_debug;

typedef struct {
  go_pose home;
} traj_cfg_home;

typedef struct {
  go_pose min_limit;
  go_pose max_limit;
} traj_cfg_limit;

typedef struct {
  go_real max_tvel;
  go_real max_tacc;
  go_real max_tjerk;
  go_real max_rvel;
  go_real max_racc;
  go_real max_rjerk;
} traj_cfg_profile;

/* The kinematics implementation will know what to do with these
   parameters. */
typedef struct {
  go_link parameters[SERVO_NUM];
  go_integer num;		/*!< how many there are */
} traj_cfg_kinematics;

typedef struct {
  go_real scale;
  /* these parameters affect how quickly the scale factor is
     "walked in", so that the time scale is not changed too abruptly */
  go_real scale_v;		/*!< d(scale)/dt */
  go_real scale_a;		/*!< d^2(scale)/dt^2 */
} traj_cfg_scale;

typedef struct {
  go_integer log_type;		/*!< TRAJ_CFG_LOG_ACT,CMD_POS, ... */
  go_integer log_which;		/*!< 0 for X, ... */
  go_integer log_size;
} traj_cfg_log;

/*!
  The \a tool_transform is specified as the position and orientation
  of the tool's end control point, ECP, with respect to the kinematic
  control point, KCP.
*/
typedef struct {
  go_pose tool_transform;    /*!< pose of tool origin in end frame */
} traj_cfg_tool_transform;

typedef struct {
  go_integer arg;		/*!< an integer argument */
} traj_cfg_stub;

typedef struct {
  unsigned char head;
  GO_RCS_CMD_MSG;
  union {
    traj_cfg_cycle_time cycle_time;
    traj_cfg_debug debug;
    traj_cfg_home home;
    traj_cfg_limit limit;
    traj_cfg_profile profile;
    traj_cfg_kinematics kinematics;
    traj_cfg_scale scale;
    traj_cfg_scale max_scale;
    traj_cfg_log log;
    traj_cfg_tool_transform tool_transform;
    traj_cfg_stub stub;
  } u;
  unsigned char tail;
} traj_cfg_struct;

/* FIXME-- consider adding go_link[] */
typedef struct {
  unsigned char head;
  GO_RCS_STAT_MSG;
  go_integer id;
  go_real cycle_time;		/*!< nominal cycle time */
  go_integer debug;
  go_integer joint_num;		/*!< number of joint subordinates */
  go_pose home;
  go_pose tool_transform;
  go_pose tool_transform_inv;
  go_pose min_limit;
  go_pose max_limit;
  go_real max_tvel;
  go_real max_tacc;
  go_real max_tjerk;
  go_real max_rvel;
  go_real max_racc;
  go_real max_rjerk;
  /*! these are the current values  */
  go_real scale;		/*!< speed scale factor */
  go_real scale_v;		/*!< d(scale)/dt */
  go_real scale_a;		/*!< d^2(scale)/dt^2 */
  /*! these are the max values */
  go_real max_scale;
  go_real max_scale_v;
  go_real max_scale_a;
  go_integer log_type;		/*!< what we're logging, also in the log */
  go_integer log_which;		/*!< which part we're logging, e.g, X */
  go_integer log_logging;	/*!< are we logging */
  go_integer queue_size;	/*!< how big the motion queue is */
  unsigned char tail;
} traj_set_struct;

/*!
  The reference structure is intended to be filled in with the actual
  pose of the device, as measured by some external system such as a
  set of laser trackers.
*/
typedef struct {
  unsigned char head;
  go_pose xinv;			/*!< set this to the new inverse transform */
  unsigned char tail;
} traj_ref_struct;

typedef struct {
  traj_cmd_struct traj_cmd;
  traj_stat_struct traj_stat;
  traj_cfg_struct traj_cfg;
  traj_set_struct traj_set;
  traj_ref_struct traj_ref;
} traj_comm_struct;

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

extern traj_comm_struct * global_traj_comm_ptr;

typedef struct {
  go_integer joint_num;		/*!< The number of joints, needed by the traj loop during initialization */
  void * kinematics;		/*!< Space for the kinematics calculations, allocated and set by gomain prior to starting the traj loop. */
} traj_arg_struct;

extern void traj_loop(void * arg);

/*!
  Setpoints to Servo are always in Servo's original startup coordinate
  system (CS). As an example, assume Servo starts out at 5.5. 

  Servo's \a input_latch is some point in the original CS, say 17. That
  is, the joint moved from 5.5 to 17 and there it saw a home
  condition. 

  After the stop, the joint will have moved some extra distance, say
  to 17.1.

  Traj has a nominal value for the joint home, say 29. Traj wants to
  call 29 what Servo calls 17.

  The difference between Servo's home in the original CS, here 17, and
  Traj's nominal home, here 29, is an offset, here 12. Traj needs to
  add 12 to Servo's position values to get values in Traj's homed CS. 
  
  When a joint is homed, Traj needs to add the offset to Servo
  positions when reading them, and subtract this offset to setpoints
  to Servo when writing them. 

  Servo will guarantee that \a inputOffset and \a homed are set
  simultaneously, so it's safe for Traj to check the \a homed flag and
  then add/subtract inputs/outputs based on that. 

  The homing sequence is as follows:

  Traj should begin by sending \a servo_cmd_servo commands with a
  constant setpoint to hold position, and the \a home flag
  cleared. Traj waits until the \a homing flag is cleared in the joint
  servo status. The \a homing flag is simply an echo of Traj's \home
  flag. Any homing action in Servo will be abandoned.

  When Servo sees the \a home flag set, and its \a homing flag clear,
  it requests a homing action if necessary from the external
  interface, and clears its \a homed flag.

  When Servo sees the homing condition met by the external interface,
  it saves the latched position to the \a input_latch in its status,
  sets the \a homed flag and leaves the \a homing flag set so that
  it won't re-initiate a home.

  When Traj sees the \a homed flag set for a joint, it stops the
  motion for that joint.
*/

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif
