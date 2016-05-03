/*
  canon_go.cc

  Canonical interface to Go Motion.
*/

#include <stddef.h>
#include <math.h>
#include <float.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "canon.h"
#include "rs274ngc.h"		// rs274ngc_sequence_number()
#include "taskintf.h"		// task_interplist
#include "interplist.h"

static bool dbflag = false;

static void dbprintf(const char * fmt, ...)
{
  va_list ap;

  if (dbflag) {
    va_start(ap, fmt);
    vprintf(fmt, ap);
    fflush(stdout);
    va_end(ap);
  }
}

/*
  The global interp list, written by the Go Motion canonical interface
  functions, read by do_cmd_execute. Defined in taskmain.c, but not
  declared in a header, so we need to declare it explicitly.
*/
extern "C" interplist_struct task_interplist;

/* set if you want to use ZYZ representation for ABC, else RPY */
#undef USE_ZYZ

enum {
  SPINDLE_ID = 1,
  FLOOD_ID = 2,
  MIST_ID = 3,
};

#define SET_TO =
#define IS ==
#define AND &&
#define OR ||

static CANON_PLANE       _active_plane = CANON_PLANE_XY;
static int               _active_slot = 1;
static int               _flood = 0;
static CANON_UNITS       _length_unit_type = CANON_UNITS_MM;
static int               _mist = 0;
static CANON_MOTION_MODE _motion_mode = CANON_CONTINUOUS;
static char              _parameter_file_name[PARAMETER_FILE_NAME_LENGTH] = ""; 
#ifdef AA
static double            _probe_position_a = 0; /*AA*/
#endif
#ifdef BB
static double            _probe_position_b = 0; /*BB*/
#endif
#ifdef CC
static double            _probe_position_c = 0; /*CC*/
#endif
static double            _probe_position_x = 0;
static double            _probe_position_y = 0;
static double            _probe_position_z = 0;
#ifdef AA
static double            _program_origin_a = 0; /*AA*/
#endif
#ifdef BB
static double            _program_origin_b = 0; /*BB*/
#endif
#ifdef CC
static double            _program_origin_c = 0; /*CC*/
#endif
static double            _program_origin_x = 0;
static double            _program_origin_y = 0;
static double            _program_origin_z = 0;
#ifdef AA
static double            _program_position_a = 0; /*AA*/
#endif
#ifdef BB
static double            _program_position_b = 0; /*BB*/
#endif
#ifdef CC
static double            _program_position_c = 0; /*CC*/
#endif
static double            _program_position_x = 0;
static double            _program_position_y = 0;
static double            _program_position_z = 0;
static double            _spindle_speed;
static CANON_DIRECTION   _spindle_turning;
static CANON_TOOL_TABLE  _tools[CANON_TOOL_MAX];
static int               _tool_max = sizeof(_tools)/sizeof(*_tools);
static double            _traverse_rate = 1.0;

static double _go_linear_feed_rate = 1.0;
static double _go_angular_feed_rate = 1.0;

// Go length units are always meters, angle units always rads

#define M_PER_MM 0.001
#define M_PER_INCH 0.0254
#define MM_PER_M 1000.0
#define RAD_PER_DEG 0.017453292519943
#define DEG_PER_RAD 57.2957795130823

static double go_per_interp_length = M_PER_MM;
static double interp_per_go_length = MM_PER_M;
static double go_per_interp_angle = RAD_PER_DEG;
static double interp_per_go_angle = DEG_PER_RAD;

#define GO_TO_INTERP_LENGTH(go) ((go)*interp_per_go_length)
#define INTERP_TO_GO_LENGTH(in) ((in)*go_per_interp_length)
#define GO_TO_INTERP_ANGLE(go) ((go)*interp_per_go_angle)
#define INTERP_TO_GO_ANGLE(in) ((in)*go_per_interp_angle)

/*
  Canonical "do it" functions.
*/

void SET_ORIGIN_OFFSETS(
 double x, double y, double z
#ifdef AA
 , double a  /*AA*/
#endif
#ifdef BB
 , double b  /*BB*/
#endif
#ifdef CC
 , double c  /*CC*/
#endif
)
{
  _program_position_x SET_TO _program_position_x + _program_origin_x - x;
  _program_position_y SET_TO _program_position_y + _program_origin_y - y;
  _program_position_z SET_TO _program_position_z + _program_origin_z - z;
#ifdef AA
  _program_position_a SET_TO _program_position_a + _program_origin_a - a;/*AA*/
#endif
#ifdef BB
  _program_position_b SET_TO _program_position_b + _program_origin_b - b;/*BB*/
#endif
#ifdef CC
  _program_position_c SET_TO _program_position_c + _program_origin_c - c;/*CC*/
#endif

  _program_origin_x SET_TO x;
  _program_origin_y SET_TO y;
  _program_origin_z SET_TO z;
#ifdef AA
  _program_origin_a SET_TO a;  /*AA*/
#endif
#ifdef BB
  _program_origin_b SET_TO b;  /*BB*/
#endif
#ifdef CC
  _program_origin_c SET_TO c;  /*CC*/
#endif
}

void USE_LENGTH_UNITS(CANON_UNITS in_unit)
{
  if (in_unit == CANON_UNITS_INCHES) {
    dbprintf("USE_LENGTH_UNITS(CANON_UNITS_INCHES)\n");
    if (_length_unit_type == CANON_UNITS_MM) {
      interp_per_go_length /= 25.4;
      go_per_interp_length *= 25.4;
      _length_unit_type SET_TO CANON_UNITS_INCHES;
      _program_origin_x SET_TO (_program_origin_x / 25.4);
      _program_origin_y SET_TO (_program_origin_y / 25.4);
      _program_origin_z SET_TO (_program_origin_z / 25.4);
      _program_position_x SET_TO (_program_position_x / 25.4);
      _program_position_y SET_TO (_program_position_y / 25.4);
      _program_position_z SET_TO (_program_position_z / 25.4);
    }
  } else if (in_unit == CANON_UNITS_MM) {
    dbprintf("USE_LENGTH_UNITS(CANON_UNITS_MM)\n");
    if (_length_unit_type == CANON_UNITS_INCHES) {
      interp_per_go_length *= 25.4;
      go_per_interp_length /= 25.4;
      _length_unit_type SET_TO CANON_UNITS_MM;
      _program_origin_x SET_TO (_program_origin_x * 25.4);
      _program_origin_y SET_TO (_program_origin_y * 25.4);
      _program_origin_z SET_TO (_program_origin_z * 25.4);
      _program_position_x SET_TO (_program_position_x * 25.4);
      _program_position_y SET_TO (_program_position_y * 25.4);
      _program_position_z SET_TO (_program_position_z * 25.4);
    }
  }
}

/* Free Space Motion */

void SET_TRAVERSE_RATE(double rate)
{
  _traverse_rate SET_TO rate;
}

void STRAIGHT_TRAVERSE
(
 double x, double y, double z
#ifdef AA
 , double a /*AA*/
#endif
#ifdef BB
 , double b /*BB*/
#endif
#ifdef CC
 , double c /*CC*/
#endif
 )
{
  double go_x, go_y, go_z;
#ifdef AA
  double go_a;
#endif
#ifdef BB
  double go_b;
#endif
#ifdef CC
  double go_c;
#endif
#ifdef USE_ZYZ
  go_zyz zyz;
#else
  go_rpy rpy;
#endif
  go_quat quat;
  interplist_type val;

  go_x SET_TO INTERP_TO_GO_LENGTH(x - _program_origin_x);
  go_y SET_TO INTERP_TO_GO_LENGTH(y - _program_origin_y);
  go_z SET_TO INTERP_TO_GO_LENGTH(z - _program_origin_z);
#ifdef AA
  go_a SET_TO INTERP_TO_GO_ANGLE(a - _program_origin_a);
#endif
#ifdef BB
  go_b SET_TO INTERP_TO_GO_ANGLE(b - _program_origin_b);
#endif
#ifdef CC
  go_c SET_TO INTERP_TO_GO_ANGLE(c - _program_origin_c);
#endif
  
  val.type = TASK_EXEC_WAIT_FOR_TOOL_TYPE;
  interplist_put(&task_interplist, val);

  val.type = val.u.traj_cmd.type = TRAJ_CMD_MOVE_WORLD_TYPE;
  val.u.traj_cmd.u.move_world.id = rs274ngc_sequence_number();
  val.u.traj_cmd.u.move_world.type = GO_MOTION_LINEAR;
  val.u.traj_cmd.u.move_world.tv = FLT_MAX;
  val.u.traj_cmd.u.move_world.rv = FLT_MAX;
  val.u.traj_cmd.u.move_world.end.tran.x = go_x;
  val.u.traj_cmd.u.move_world.end.tran.y = go_y;
  val.u.traj_cmd.u.move_world.end.tran.z = go_z;

#ifdef USE_ZYZ

#ifdef AA
  zyz.z = go_a;
#else
  zyz.z = 0;
#endif
#ifdef BB
  zyz.y = go_b;
#else
  zyz.y = 0;
#endif
#ifdef CC
  zyz.zp = go_c;
#else
  zyz.zp = 0;
#endif
  go_zyz_quat_convert(&zyz, &quat);

#else

#ifdef AA
  rpy.r = go_a;
#else
  rpy.r = 0;
#endif
#ifdef BB
  rpy.p = go_b;
#else
  rpy.p = 0;
#endif
#ifdef CC
  rpy.y = go_c;
#else
  rpy.y = 0;
#endif
  go_rpy_quat_convert(&rpy, &quat);

#endif

  val.u.traj_cmd.u.move_world.end.rot = quat;
  val.u.traj_cmd.u.move_world.time = -1;
  interplist_put(&task_interplist, val);

  _program_position_x SET_TO x;
  _program_position_y SET_TO y;
  _program_position_z SET_TO z;
#ifdef AA
  _program_position_a SET_TO a; /*AA*/
#endif
#ifdef BB
  _program_position_b SET_TO b; /*BB*/
#endif
#ifdef CC
  _program_position_c SET_TO c; /*CC*/
#endif

  dbprintf("STRAIGHT_TRAVERSE(%.4f, %.4f, %.4f"
#ifdef AA
         ", %.4f" /*AA*/
#endif
#ifdef BB
         ", %.4f" /*BB*/
#endif
#ifdef CC
         ", %.4f" /*CC*/
#endif
         ")\n", go_x, go_y, go_z
#ifdef AA
         , go_a /*AA*/
#endif
#ifdef BB
         , go_b /*BB*/
#endif
#ifdef CC
         , go_c /*CC*/
#endif
         );
}

/* Machining Attributes */

void SET_FEED_RATE(double rate)
{
  /*
    The feed rate applies first to linear moves, in length units per
    minute, and if no linear motion is being done, then to angular
    moves, in angle units per minute. Set the both here, and use
    them as indicated when handling STRAIGHT_FEED.
  */
  _go_linear_feed_rate SET_TO INTERP_TO_GO_LENGTH(rate) / 60.0;
  _go_angular_feed_rate SET_TO INTERP_TO_GO_ANGLE(rate) / 60.0;
}

void SET_FEED_REFERENCE(CANON_FEED_REFERENCE reference)
{
  dbprintf("SET_FEED_REFERENCE(%s)\n", reference == CANON_WORKPIECE ? "CANON_WORKPIECE" : "CANON_XYZ");
}

void SET_MOTION_CONTROL_MODE(CANON_MOTION_MODE mode)
{
  if (mode IS CANON_EXACT_STOP)
    {
      _motion_mode SET_TO CANON_EXACT_STOP;
    }
  else if (mode IS CANON_EXACT_PATH)
    {
      _motion_mode SET_TO CANON_EXACT_PATH;
    }
  else if (mode IS CANON_CONTINUOUS)
    {
      _motion_mode SET_TO CANON_CONTINUOUS;
    }
}

void SELECT_PLANE(CANON_PLANE in_plane)
{
  _active_plane SET_TO in_plane;
}

void SET_CUTTER_RADIUS_COMPENSATION(double radius)
{
  // handled by interpreter
}

void START_CUTTER_RADIUS_COMPENSATION(int side)
{
  // handled by interpreter
}

void STOP_CUTTER_RADIUS_COMPENSATION()
{
  // handled by interpreter
}

void START_SPEED_FEED_SYNCH()
{
  // FIXME -- do something with it
}

void STOP_SPEED_FEED_SYNCH()
{
  // FIXME -- do something with it
}

/* Machining Functions */

void ARC_FEED
(
 double first_end, double second_end, // ends
 double first_axis, double second_axis, // center
 int rotation,			     // turns
 double axis_end_point    // end
#ifdef AA
 , double a /*AA*/
#endif
#ifdef BB
 , double b /*BB*/
#endif
#ifdef CC
 , double c /*CC*/
#endif
 )
{
  double x, y, z;
  // Go Motion needs end XYZ, center XYZ, normal XYZ, and turns
  double go_end_x, go_end_y, go_end_z;
  double go_center_x, go_center_y, go_center_z;
  double go_normal_x, go_normal_y, go_normal_z;
  int go_turns;
#ifdef AA
  double go_a;
#endif
#ifdef BB
  double go_b;
#endif
#ifdef CC
  double go_c;
#endif
#ifdef USE_ZYZ
  go_zyz zyz;
#else
  go_rpy rpy;
#endif
  go_quat quat;
  interplist_type val;

  if (_active_plane IS CANON_PLANE_XY) {
    x SET_TO first_end;
    y SET_TO second_end;
    z SET_TO axis_end_point;
    go_end_x SET_TO INTERP_TO_GO_LENGTH(first_end - _program_origin_x);
    go_end_y SET_TO INTERP_TO_GO_LENGTH(second_end - _program_origin_y);
    go_end_z SET_TO INTERP_TO_GO_LENGTH(axis_end_point - _program_origin_z);
    go_center_x SET_TO INTERP_TO_GO_LENGTH(first_axis - _program_origin_x);
    go_center_y SET_TO INTERP_TO_GO_LENGTH(second_axis - _program_origin_y);
    go_center_z SET_TO go_end_z;	// Go Motion will normalize helixes
    go_normal_x SET_TO 0.0;
    go_normal_y SET_TO 0.0;
    if (rotation > 0) {
      go_turns SET_TO +rotation - 1;
      go_normal_z SET_TO +1.0;
    } else {
      go_turns SET_TO -rotation - 1;
      go_normal_z SET_TO -1.0;
    }
  } else if (_active_plane IS CANON_PLANE_YZ) {
    y SET_TO first_end;
    z SET_TO second_end;
    x SET_TO axis_end_point;
    go_end_y SET_TO INTERP_TO_GO_LENGTH(first_end - _program_origin_y);
    go_end_z SET_TO INTERP_TO_GO_LENGTH(second_end - _program_origin_z);
    go_end_x SET_TO INTERP_TO_GO_LENGTH(axis_end_point - _program_origin_x);
    go_center_y SET_TO INTERP_TO_GO_LENGTH(first_axis - _program_origin_y);
    go_center_z SET_TO INTERP_TO_GO_LENGTH(second_axis - _program_origin_z);
    go_center_x SET_TO go_end_x;
    go_normal_y SET_TO 0.0;
    go_normal_z SET_TO 0.0;
    if (rotation > 0) {
      go_turns SET_TO +rotation - 1;
      go_normal_x SET_TO +1.0;
    } else {
      go_turns SET_TO -rotation - 1;
      go_normal_x SET_TO -1.0;
    }
  } else { /* if (_active_plane IS CANON_PLANE_XZ) */
    z SET_TO first_end;
    x SET_TO second_end;
    y SET_TO axis_end_point;
    go_end_z SET_TO INTERP_TO_GO_LENGTH(first_end - _program_origin_z);
    go_end_x SET_TO INTERP_TO_GO_LENGTH(second_end - _program_origin_x);
    go_end_y SET_TO INTERP_TO_GO_LENGTH(axis_end_point - _program_origin_y);
    go_center_z SET_TO INTERP_TO_GO_LENGTH(first_axis - _program_origin_z);
    go_center_x SET_TO INTERP_TO_GO_LENGTH(second_axis - _program_origin_x);
    go_center_y SET_TO go_end_y;
    go_normal_z SET_TO 0.0;
    go_normal_x SET_TO 0.0;
    if (rotation > 0) {
      go_turns SET_TO +rotation - 1;
      go_normal_y SET_TO +1.0;
    } else {
      go_turns SET_TO -rotation - 1;
      go_normal_y SET_TO -1.0;
    }
  }
#ifdef AA
  go_a SET_TO INTERP_TO_GO_ANGLE(a - _program_origin_a);
#endif
#ifdef BB
  go_b SET_TO INTERP_TO_GO_ANGLE(b - _program_origin_b);
#endif
#ifdef CC
  go_c SET_TO INTERP_TO_GO_ANGLE(c - _program_origin_c);
#endif
  
  val.type = TASK_EXEC_WAIT_FOR_TOOL_TYPE;
  interplist_put(&task_interplist, val);

  val.type = val.u.traj_cmd.type = TRAJ_CMD_MOVE_WORLD_TYPE;
  val.u.traj_cmd.u.move_world.id = rs274ngc_sequence_number();
  val.u.traj_cmd.u.move_world.type = GO_MOTION_CIRCULAR;

  if (fabs(x - _program_position_x) < FLT_EPSILON &&
      fabs(y - _program_position_y) < FLT_EPSILON &&
      fabs(z - _program_position_z) < FLT_EPSILON) {
    // no translation, so let angular motion dictate speed
    val.u.traj_cmd.u.move_world.tv = FLT_MAX;
    val.u.traj_cmd.u.move_world.rv = _go_angular_feed_rate;
  } else {
    // some translation, so let that dictate speed
    val.u.traj_cmd.u.move_world.tv = _go_linear_feed_rate;
    val.u.traj_cmd.u.move_world.rv = FLT_MAX;
  }
  // the ta and tj params will be written by the task controller
  // ditto for the ra and rj params
  val.u.traj_cmd.u.move_world.end.tran.x = go_end_x;
  val.u.traj_cmd.u.move_world.end.tran.y = go_end_y;
  val.u.traj_cmd.u.move_world.end.tran.z = go_end_z;
  val.u.traj_cmd.u.move_world.center.x = go_center_x;
  val.u.traj_cmd.u.move_world.center.y = go_center_y;
  val.u.traj_cmd.u.move_world.center.z = go_center_z;
  val.u.traj_cmd.u.move_world.normal.x = go_normal_x;
  val.u.traj_cmd.u.move_world.normal.y = go_normal_y;
  val.u.traj_cmd.u.move_world.normal.z = go_normal_z;
  val.u.traj_cmd.u.move_world.turns = go_turns;

#ifdef USE_ZYZ

#ifdef AA
  zyz.z = go_a;
#else
  zyz.z = 0;
#endif
#ifdef BB
  zyz.y = go_b;
#else
  zyz.y = 0;
#endif
#ifdef CC
  zyz.zp = go_c;
#else
  zyz.zp = 0;
#endif
  go_zyz_quat_convert(&zyz, &quat);

#else

#ifdef AA
  rpy.r = go_a;
#else
  rpy.r = 0;
#endif
#ifdef BB
  rpy.p = go_b;
#else
  rpy.p = 0;
#endif
#ifdef CC
  rpy.y = go_c;
#else
  rpy.y = 0;
#endif
  go_rpy_quat_convert(&rpy, &quat);

#endif

  val.u.traj_cmd.u.move_world.end.rot = quat;
  val.u.traj_cmd.u.move_world.time = -1;
  interplist_put(&task_interplist, val);

  _program_position_x SET_TO x;
  _program_position_y SET_TO y;
  _program_position_z SET_TO z;
#ifdef AA
  _program_position_a SET_TO a; /*AA*/
#endif
#ifdef BB
  _program_position_b SET_TO b; /*BB*/
#endif
#ifdef CC
  _program_position_c SET_TO c; /*CC*/
#endif

  dbprintf("ARC_FEED(%.4f, %.4f, %.4f"
#ifdef AA
         ", %.4f" /*AA*/
#endif
#ifdef BB
         ", %.4f" /*BB*/
#endif
#ifdef CC
         ", %.4f" /*CC*/
#endif
         ")\n", go_end_x, go_end_y, go_end_z
#ifdef AA
         , go_a /*AA*/
#endif
#ifdef BB
         , go_b /*BB*/
#endif
#ifdef CC
         , go_c /*CC*/
#endif
         );
}

void STRAIGHT_FEED
(
 double x, double y, double z
#ifdef AA
 , double a /*AA*/
#endif
#ifdef BB
 , double b /*BB*/
#endif
#ifdef CC
 , double c /*CC*/
#endif
 )
{
  double go_x, go_y, go_z;
#ifdef AA
  double go_a;
#endif
#ifdef BB
  double go_b;
#endif
#ifdef CC
  double go_c;
#endif
#ifdef USE_ZYZ
  go_zyz zyz;
#else
  go_rpy rpy;
#endif
  go_quat quat;
  interplist_type val;

  go_x SET_TO INTERP_TO_GO_LENGTH(x - _program_origin_x);
  go_y SET_TO INTERP_TO_GO_LENGTH(y - _program_origin_y);
  go_z SET_TO INTERP_TO_GO_LENGTH(z - _program_origin_z);
#ifdef AA
  go_a SET_TO INTERP_TO_GO_ANGLE(a - _program_origin_a);
#endif
#ifdef BB
  go_b SET_TO INTERP_TO_GO_ANGLE(b - _program_origin_b);
#endif
#ifdef CC
  go_c SET_TO INTERP_TO_GO_ANGLE(c - _program_origin_c);
#endif
  
  val.type = TASK_EXEC_WAIT_FOR_TOOL_TYPE;
  interplist_put(&task_interplist, val);

  val.type = val.u.traj_cmd.type = TRAJ_CMD_MOVE_WORLD_TYPE;
  val.u.traj_cmd.u.move_world.id = rs274ngc_sequence_number();
  val.u.traj_cmd.u.move_world.type = GO_MOTION_LINEAR;

  if (fabs(x - _program_position_x) < FLT_EPSILON &&
      fabs(y - _program_position_y) < FLT_EPSILON &&
      fabs(z - _program_position_z) < FLT_EPSILON) {
    // no translation, so let angular motion dictate speed
    val.u.traj_cmd.u.move_world.tv = FLT_MAX;
    val.u.traj_cmd.u.move_world.rv = _go_angular_feed_rate;
  } else {
    // some translation, so let that dictate speed
    val.u.traj_cmd.u.move_world.tv = _go_linear_feed_rate;
    val.u.traj_cmd.u.move_world.rv = FLT_MAX;
  }
  // the ta and tj params will be written by the task controller
  // ditto for the ra and rj params
  val.u.traj_cmd.u.move_world.end.tran.x = go_x;
  val.u.traj_cmd.u.move_world.end.tran.y = go_y;
  val.u.traj_cmd.u.move_world.end.tran.z = go_z;

#ifdef USE_ZYZ

#ifdef AA
  zyz.z = go_a;
#else
  zyz.z = 0;
#endif
#ifdef BB
  zyz.y = go_b;
#else
  zyz.y = 0;
#endif
#ifdef CC
  zyz.zp = go_c;
#else
  zyz.zp = 0;
#endif
  go_zyz_quat_convert(&zyz, &quat);

#else

#ifdef AA
  rpy.r = go_a;
#else
  rpy.r = 0;
#endif
#ifdef BB
  rpy.p = go_b;
#else
  rpy.p = 0;
#endif
#ifdef CC
  rpy.y = go_c;
#else
  rpy.y = 0;
#endif
  go_rpy_quat_convert(&rpy, &quat);

#endif

  val.u.traj_cmd.u.move_world.end.rot = quat;
  val.u.traj_cmd.u.move_world.time = -1;
  interplist_put(&task_interplist, val);

  _program_position_x SET_TO x;
  _program_position_y SET_TO y;
  _program_position_z SET_TO z;
#ifdef AA
  _program_position_a SET_TO a; /*AA*/
#endif
#ifdef BB
  _program_position_b SET_TO b; /*BB*/
#endif
#ifdef CC
  _program_position_c SET_TO c; /*CC*/
#endif

  dbprintf("STRAIGHT_FEED(%.4f, %.4f, %.4f"
#ifdef AA
         ", %.4f" /*AA*/
#endif
#ifdef BB
         ", %.4f" /*BB*/
#endif
#ifdef CC
         ", %.4f" /*CC*/
#endif
         ")\n", go_x, go_y, go_z
#ifdef AA
         , go_a /*AA*/
#endif
#ifdef BB
         , go_b /*BB*/
#endif
#ifdef CC
         , go_c /*CC*/
#endif
         );
}

/*
  This models backing the probe off 0.01 inch or 0.254 mm from the probe
  point towards the previous location after the probing, if the probe
  point is not the same as the previous point -- which it should not be.

  Go Motion does not have a probe command, so this just does a move to the
  goal probe position.
*/

void STRAIGHT_PROBE
(
 double x, double y, double z
#ifdef AA
 , double a /*AA*/
#endif
#ifdef BB
 , double b /*BB*/
#endif
#ifdef CC
 , double c /*CC*/
#endif
 )
{
  double distance;
  double dx, dy, dz;
  double backoff;
  double go_x, go_y, go_z;
#ifdef AA
  double go_a;
#endif
#ifdef BB
  double go_b;
#endif
#ifdef CC
  double go_c;
#endif
#ifdef USE_ZYZ
  go_zyz zyz;
#else
  go_rpy rpy;
#endif
  go_quat quat;
  interplist_type val;

  go_x SET_TO INTERP_TO_GO_LENGTH(x - _program_origin_x);
  go_y SET_TO INTERP_TO_GO_LENGTH(y - _program_origin_y);
  go_z SET_TO INTERP_TO_GO_LENGTH(z - _program_origin_z);
#ifdef AA
  go_a SET_TO INTERP_TO_GO_ANGLE(a - _program_origin_a);
#endif
#ifdef BB
  go_b SET_TO INTERP_TO_GO_ANGLE(b - _program_origin_b);
#endif
#ifdef CC
  go_c SET_TO INTERP_TO_GO_ANGLE(c - _program_origin_c);
#endif
  
  val.type = TASK_EXEC_WAIT_FOR_TOOL_TYPE;
  interplist_put(&task_interplist, val);

  // this is the same as for a STRAIGHT_FEED

  val.type = val.u.traj_cmd.type = TRAJ_CMD_MOVE_WORLD_TYPE;
  val.u.traj_cmd.u.move_world.id = rs274ngc_sequence_number();
  val.u.traj_cmd.u.move_world.type = GO_MOTION_LINEAR;

  if (fabs(x - _program_position_x) < FLT_EPSILON &&
      fabs(y - _program_position_y) < FLT_EPSILON &&
      fabs(z - _program_position_z) < FLT_EPSILON) {
    val.u.traj_cmd.u.move_world.tv = FLT_MAX;
    val.u.traj_cmd.u.move_world.rv = _go_angular_feed_rate;
  } else {
    val.u.traj_cmd.u.move_world.tv = _go_linear_feed_rate;
    val.u.traj_cmd.u.move_world.rv = FLT_MAX;
  }
  val.u.traj_cmd.u.move_world.end.tran.x = go_x;
  val.u.traj_cmd.u.move_world.end.tran.y = go_y;
  val.u.traj_cmd.u.move_world.end.tran.z = go_z;

#ifdef USE_ZYZ

#ifdef AA
  zyz.z = go_a;
#else
  zyz.z = 0;
#endif
#ifdef BB
  zyz.y = go_b;
#else
  zyz.y = 0;
#endif
#ifdef CC
  zyz.zp = go_c;
#else
  zyz.zp = 0;
#endif
  go_zyz_quat_convert(&zyz, &quat);

#else

#ifdef AA
  rpy.r = go_a;
#else
  rpy.r = 0;
#endif
#ifdef BB
  rpy.p = go_b;
#else
  rpy.p = 0;
#endif
#ifdef CC
  rpy.y = go_c;
#else
  rpy.y = 0;
#endif
  go_rpy_quat_convert(&rpy, &quat);

#endif

  val.u.traj_cmd.u.move_world.end.rot = quat;
  val.u.traj_cmd.u.move_world.time = -1;
  interplist_put(&task_interplist, val);

  dx SET_TO (_program_position_x - x);
  dy SET_TO (_program_position_y - y);
  dz SET_TO (_program_position_z - z);
  distance SET_TO sqrt((dx * dx) + (dy * dy) + (dz * dz));

  if (distance IS 0) {
    _program_position_x SET_TO _program_position_x;
    _program_position_y SET_TO _program_position_y;
    _program_position_z SET_TO _program_position_z;
  } else {
    backoff SET_TO ((_length_unit_type IS CANON_UNITS_MM) ? 0.254 : 0.01);
    _program_position_x SET_TO (x + (backoff * (dx / distance)));
    _program_position_y SET_TO (y + (backoff * (dy / distance)));
    _program_position_z SET_TO (z + (backoff * (dz / distance)));
  }
  _probe_position_x SET_TO x;
  _probe_position_y SET_TO y;
  _probe_position_z SET_TO z;
#ifdef AA
  _program_position_a SET_TO a; /*AA*/
  _probe_position_a SET_TO a; /*AA*/
#endif
#ifdef BB
  _program_position_b SET_TO b; /*BB*/
  _probe_position_b SET_TO b; /*BB*/
#endif
#ifdef CC
  _program_position_c SET_TO c; /*CC*/
  _probe_position_c SET_TO c; /*CC*/
#endif

  dbprintf("STRAIGHT_PROBE(%.4f, %.4f, %.4f"
#ifdef AA
         ", %.4f" /*AA*/
#endif
#ifdef BB
         ", %.4f" /*BB*/
#endif
#ifdef CC
         ", %.4f" /*CC*/
#endif
         ")\n", go_x, go_y, go_z
#ifdef AA
         , go_a /*AA*/
#endif
#ifdef BB
         , go_b /*BB*/
#endif
#ifdef CC
         , go_c /*CC*/
#endif
         );
}

void DWELL(double seconds)
{
  interplist_type val;

  val.type = val.u.task_cmd.type = TASK_EXEC_DELAY_TYPE;
  val.u.task_cmd.u.delay.time = seconds;
  interplist_put(&task_interplist, val);
}

/* Spindle Functions */
void SPINDLE_RETRACT_TRAVERSE()
{
  // FIXME -- do something with it
}

void START_SPINDLE_CLOCKWISE()
{
  interplist_type val;
  
  val.type = TASK_EXEC_WAIT_FOR_MOTION_TYPE;
  interplist_put(&task_interplist, val);
  val.type = TASK_EXEC_WAIT_FOR_TOOL_TYPE;
  interplist_put(&task_interplist, val);

  val.type = val.u.traj_cmd.type = TOOL_CMD_ON_TYPE;
  val.u.tool_cmd.id = SPINDLE_ID;
  val.u.tool_cmd.u.on.value = +_spindle_speed;
  interplist_put(&task_interplist, val);

  _spindle_turning SET_TO ((_spindle_speed IS 0) ? CANON_STOPPED : CANON_CLOCKWISE);
}

void START_SPINDLE_COUNTERCLOCKWISE()
{
  interplist_type val;
  
  val.type = TASK_EXEC_WAIT_FOR_MOTION_TYPE;
  interplist_put(&task_interplist, val);
  val.type = TASK_EXEC_WAIT_FOR_TOOL_TYPE;
  interplist_put(&task_interplist, val);

  val.type = val.u.traj_cmd.type = TOOL_CMD_ON_TYPE;
  val.u.tool_cmd.id = SPINDLE_ID;
  val.u.tool_cmd.u.on.value = -_spindle_speed;
  interplist_put(&task_interplist, val);

  _spindle_turning SET_TO ((_spindle_speed IS 0) ? CANON_STOPPED : CANON_COUNTERCLOCKWISE);
}

void SET_SPINDLE_SPEED(double rpm)
{
  _spindle_speed SET_TO rpm;
}

void STOP_SPINDLE_TURNING()
{
  interplist_type val;
  
  val.type = TASK_EXEC_WAIT_FOR_MOTION_TYPE;
  interplist_put(&task_interplist, val);
  val.type = TASK_EXEC_WAIT_FOR_TOOL_TYPE;
  interplist_put(&task_interplist, val);

  val.type = val.u.traj_cmd.type = TOOL_CMD_OFF_TYPE;
  val.u.tool_cmd.id = SPINDLE_ID;
  interplist_put(&task_interplist, val);

  _spindle_turning SET_TO CANON_STOPPED;
}

void SPINDLE_RETRACT()
{
  // FIXME -- do something with it
}

void ORIENT_SPINDLE(double orientation, CANON_DIRECTION direction)
{
  // FIXME -- do something with it
}

void USE_NO_SPINDLE_FORCE()
{
  // FIXME -- do something with it
}

/* Tool Functions */

void USE_TOOL_LENGTH_OFFSET(double length)
{
  // FIXME -- do something with it
}

void CHANGE_TOOL(int slot)
{
  CANON_TOOL_TABLE tool;
  interplist_type val;

  // This is an M6
  
  val.type = TASK_EXEC_WAIT_FOR_MOTION_TYPE;
  interplist_put(&task_interplist, val);
  val.type = TASK_EXEC_WAIT_FOR_TOOL_TYPE;
  interplist_put(&task_interplist, val);

  dbprintf("CHANGE_TOOL(%d)", (int) slot);
  if (slot < 0 || slot >= sizeof(_tools)/sizeof(*_tools)) {
    dbprintf(" (out of range)\n");
    return;
  }
  tool = GET_EXTERNAL_TOOL_TABLE(slot);
  dbprintf(" [%d] [%.4f] [%.4f]\n", tool.id, tool.length, tool.diameter);

  _active_slot SET_TO slot;
}

void SELECT_TOOL(int slot)
{
  // This is a T code
}

/* Misc Functions */

void CLAMP_AXIS(CANON_AXIS axis)
{
  // FIXME -- do something with it
}

void COMMENT(const char *s)
{
  /* Rogue extensibility via hot comments - comments that do something */
#define HOT(STR) (!strncmp(s, STR, strlen(STR)))

  if (HOT("DEBUGON")) {
    dbflag = true;
  } else if (HOT("DEBUGOFF")) {
    dbflag = false;
  }

  dbprintf("COMMENT(%s)\n", s);
}

void DISABLE_FEED_OVERRIDE()
{
  // FIXME -- do something with it
}

void DISABLE_SPEED_OVERRIDE()
{
  // FIXME -- do something with it
}

void ENABLE_FEED_OVERRIDE()
{
  // FIXME -- do something with it
}

void ENABLE_SPEED_OVERRIDE()
{
  // FIXME -- do something with it
}

void FLOOD_OFF()
{
  interplist_type val;
  
  val.type = TASK_EXEC_WAIT_FOR_MOTION_TYPE;
  interplist_put(&task_interplist, val);
  val.type = TASK_EXEC_WAIT_FOR_TOOL_TYPE;
  interplist_put(&task_interplist, val);

  val.type = val.u.tool_cmd.type = TOOL_CMD_OFF_TYPE;
  val.u.tool_cmd.id = FLOOD_ID;
  interplist_put(&task_interplist, val);

  _flood SET_TO 0;
}

void FLOOD_ON()
{
  interplist_type val;
  
  val.type = TASK_EXEC_WAIT_FOR_MOTION_TYPE;
  interplist_put(&task_interplist, val);
  val.type = TASK_EXEC_WAIT_FOR_TOOL_TYPE;
  interplist_put(&task_interplist, val);

  val.type = val.u.traj_cmd.type = TOOL_CMD_ON_TYPE;
  val.u.tool_cmd.id = FLOOD_ID;
  val.u.tool_cmd.u.on.value = 1;
  interplist_put(&task_interplist, val);

  _flood SET_TO 1;
}

void INIT_CANON()
{
  if (_length_unit_type == CANON_UNITS_MM) {
    go_per_interp_length = M_PER_MM;
  } else {
    go_per_interp_length = M_PER_INCH;
  }
  interp_per_go_length = 1.0 / go_per_interp_length;

  go_per_interp_angle = RAD_PER_DEG;
  interp_per_go_angle = 1.0 / go_per_interp_angle;
}

void MESSAGE(char *s)
{
  // FIXME -- do something with it
}

void MIST_OFF()
{
  interplist_type val;
  
  val.type = TASK_EXEC_WAIT_FOR_MOTION_TYPE;
  interplist_put(&task_interplist, val);
  val.type = TASK_EXEC_WAIT_FOR_TOOL_TYPE;
  interplist_put(&task_interplist, val);

  val.type = val.u.tool_cmd.type = TOOL_CMD_OFF_TYPE;
  val.u.tool_cmd.id = MIST_ID;
  interplist_put(&task_interplist, val);

  _mist SET_TO 0;
}

void MIST_ON()
{
  interplist_type val;
  
  val.type = TASK_EXEC_WAIT_FOR_MOTION_TYPE;
  interplist_put(&task_interplist, val);
  val.type = TASK_EXEC_WAIT_FOR_TOOL_TYPE;
  interplist_put(&task_interplist, val);

  val.type = val.u.tool_cmd.type = TOOL_CMD_ON_TYPE;
  val.u.tool_cmd.id = MIST_ID;
  val.u.tool_cmd.u.on.value = 1;
  interplist_put(&task_interplist, val);

  _mist SET_TO 1;
}

void PALLET_SHUTTLE()
{
  // FIXME -- do something with it
}

void TURN_PROBE_OFF()
{
  // FIXME -- do something with it
}

void TURN_PROBE_ON()
{
  // FIXME -- do something with it
}

void UNCLAMP_AXIS(CANON_AXIS axis)
{
  // FIXME -- do something with it
}

/* Program Functions */

void PROGRAM_STOP()
{
  // FIXME -- do something with it
}

void OPTIONAL_PROGRAM_STOP()
{
  // FIXME -- do something with it
}

void PROGRAM_END()
{
  // FIXME -- do something with it
}

/*
  Canonical "Give me information" functions.
*/

/* Returns the system feed rate */
double GET_EXTERNAL_FEED_RATE()
{
  // the Go Motion speed is units per second, so convert to per minute
  return GO_TO_INTERP_LENGTH(_go_linear_feed_rate) * 60.0;
  // and ignore angular speed
}

/* Returns the system flood coolant setting zero = off, non-zero = on */
int GET_EXTERNAL_FLOOD()
{
  return _flood;
}

// Returns or sets the system length unit factor, in units / mm

CANON_UNITS GET_EXTERNAL_LENGTH_UNIT_TYPE()
{
  return _length_unit_type;
}

/* Returns the system mist coolant setting zero = off, non-zero = on */
int GET_EXTERNAL_MIST()
{
  return _mist;
}

// Returns the current motion control mode
CANON_MOTION_MODE GET_EXTERNAL_MOTION_CONTROL_MODE()
{
  return _motion_mode;
}

void GET_EXTERNAL_PARAMETER_FILE_NAME(
 char *file_name,       /* string: to copy file name into       */
 int max_size)           /* maximum number of characters to copy */
{
  if (strlen(_parameter_file_name) < max_size) {
    strcpy(file_name, _parameter_file_name);
  } else {
    file_name[0] SET_TO 0;
  }
}

CANON_PLANE GET_EXTERNAL_PLANE()
{
  return _active_plane;
}

#ifdef AA
/* returns the current a-axis position */
double GET_EXTERNAL_POSITION_A()
{
  return _program_position_a;
}
#endif

#ifdef BB
/* returns the current b-axis position */
double GET_EXTERNAL_POSITION_B()
{
  return _program_position_b;
}
#endif

#ifdef CC
/* returns the current c-axis position */
double GET_EXTERNAL_POSITION_C()
{
  return _program_position_c;
}
#endif

/* returns the current x-axis position */
double GET_EXTERNAL_POSITION_X()
{
  return _program_position_x;
}

/* returns the current y-axis position */
double GET_EXTERNAL_POSITION_Y()
{
  return _program_position_y;
}

/* returns the current z-axis position */
double GET_EXTERNAL_POSITION_Z()
{
  return _program_position_z;
}

#ifdef AA
/* returns the a-axis position at the last probe trip. This is only valid
   once the probe command has executed to completion. */
double GET_EXTERNAL_PROBE_POSITION_A()
{
  return _probe_position_a;
}
#endif

#ifdef BB
/* returns the b-axis position at the last probe trip. This is only valid
   once the probe command has executed to completion. */
double GET_EXTERNAL_PROBE_POSITION_B()
{
  return _probe_position_b;
}
#endif

#ifdef CC
/* returns the c-axis position at the last probe trip. This is only valid
   once the probe command has executed to completion. */
double GET_EXTERNAL_PROBE_POSITION_C()
{
  return _probe_position_c;
}
#endif

/* returns the x-axis position at the last probe trip. This is only valid
   once the probe command has executed to completion. */
double GET_EXTERNAL_PROBE_POSITION_X()
{
  return _probe_position_x;
}

/* returns the y-axis position at the last probe trip. This is only valid
   once the probe command has executed to completion. */
double GET_EXTERNAL_PROBE_POSITION_Y()
{
  return _probe_position_y;
}

/* returns the z-axis position at the last probe trip. This is only valid
   once the probe command has executed to completion. */
double GET_EXTERNAL_PROBE_POSITION_Z()
{
  return _probe_position_z;
}

/* Returns the value for any analog non-contact probing. */
/* This is a dummy of a dummy, returning a useless value. */
/* It is not expected this will ever be called. */
double GET_EXTERNAL_PROBE_VALUE()
{
  return 1.0;
}

/* Returns zero if queue is not empty, non-zero if the queue is empty */
/* In the stand-alone interpreter, there is no queue, so it is always empty */
int GET_EXTERNAL_QUEUE_EMPTY()
{
  return 1;
}

/* Returns the system value for spindle speed in rpm */
double GET_EXTERNAL_SPEED()
{
  return _spindle_speed;
}

/* Returns the system value for direction of spindle turning */
CANON_DIRECTION GET_EXTERNAL_SPINDLE()
{
  return _spindle_turning;
}

/* Returns the system value for the carousel slot in which the tool
currently in the spindle belongs. Return value zero means there is no
tool in the spindle. */
int GET_EXTERNAL_TOOL_SLOT()
{
  return _active_slot;
}

/* Returns maximum number of tools */
int GET_EXTERNAL_TOOL_MAX()
{
  return _tool_max;
}

/* Returns the CANON_TOOL_TABLE structure associated with the tool
   in the given pocket */
CANON_TOOL_TABLE GET_EXTERNAL_TOOL_TABLE(int pocket)
{
  return _tools[pocket];
}

/* Returns the system traverse rate */
double GET_EXTERNAL_TRAVERSE_RATE()
{
  return _traverse_rate;
}

/* Sets the name of the parameter file */
void SET_EXTERNAL_PARAMETER_FILE_NAME(const char *file_name)
{
  if (strlen(file_name) >= sizeof(_parameter_file_name)) {
    _parameter_file_name[0] = 0;
  } else {
    strncpy(_parameter_file_name, file_name, sizeof(_parameter_file_name));
  }
}

/* Sets the CANON_TOOL_TABLE structure associated with the tool
   in the given pocket. Returns 0 if success, otherwise a non-zero
   value if the pocket is out of range. */
int SET_EXTERNAL_TOOL_TABLE(int pocket, CANON_TOOL_TABLE tool)
{
  if (pocket < 0 || pocket > sizeof(_tools)/sizeof(*_tools)) {
    return 1;
  }

  _tools[pocket] = tool;

  return 0;
}
