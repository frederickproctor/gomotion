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
  \file gosh.c

  \brief Interactive shell for interfacing to a Go Motion controller,
  sending commands and configuration requests, and viewing status and
  settings. \c gosh is intended for testing, not normal use, although one
  could write \c gosh scripts. The more full-featured script interpreter
  intended for normal use is \c gotk, as developed in \c gotk.c.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>		/* printf, sscanf, feof, stdin */
#include <string.h>		/* strlen, strcmp */
#include <stdlib.h>		/* strtol */
#include <ctype.h>		/* isspace */
#include <inifile.h>
#include <ulapi.h>		/* ulapi_time */
#include "go.h"			/* go_pose */
#include "goio.h"		/* go_io_struct */
#include "gorcs.h"		/* NEW_COMMAND, RCS_DONE, ... */
#include "gorcsutil.h"		/* rcs_state_to_string */
#include "servointf.h"		/* servo_cmd_struct ... */
#include "trajintf.h"		/* traj_cmd_struct ... */
#include "taskintf.h"		/* task_cmd_struct ... */
#include "toolintf.h"		/* tool_cmd_struct, ... */

#ifdef HAVE_READLINE_READLINE_H
#include <readline/readline.h>	/* readline */
#include <readline/history.h>	/* using_history */
#endif

#define SAFECPY(dst,src) strncpy((dst),(src),sizeof(dst)); (dst)[sizeof(dst)-1] = 0

/* how many seconds to wait to connect to subsystems */
#define CONNECT_WAIT_TIME 3.0

/* Serial number offset from what's read out to what we'll send. Making this
   large means we can jump in and stuff commands and be less likely to
   conflict with the official supervisor. */
#define SERIAL_NUMBER_OFFSET 100

static int SERVO_HOWMANY = 0;

static go_real home_vel[SERVO_NUM] = {0.0};
static go_real m_per_length_units, rad_per_angle_units;
static double length_units_per_m, angle_units_per_rad;
static go_integer joint_quantity[SERVO_NUM];

/* these are "To Go Length, Angle, Quantity" conversions into Go units */
#define TGL(x) (((go_real) (x)) * m_per_length_units)
#define TGA(x) (((go_real) (x)) * rad_per_angle_units)
#define TGQ(x,i)							\
  (joint_quantity[i] == GO_QUANTITY_LENGTH ? (((go_real) (x)) * m_per_length_units) : \
   joint_quantity[i] == GO_QUANTITY_ANGLE ? (((go_real) (x)) * rad_per_angle_units) : ((go_real) (x)))

/* these are "From Go Length, Angle, Quantity" conversions into user units */
#define FGL(x) (((double) (x)) * length_units_per_m)
#define FGA(x) (((double) (x)) * angle_units_per_rad)
#define FGQ(x,i)							\
  (joint_quantity[i] == GO_QUANTITY_LENGTH ? (((double) (x)) * length_units_per_m) : \
   joint_quantity[i] == GO_QUANTITY_ANGLE ? (((double) (x)) * angle_units_per_rad) : ((double) (x)))

/* copy of previously-run task script, for convenience */
static char old_program[TASK_CMD_PROGRAM_LEN] = "";

static int ini_load(char *inifile_name,
		    int *task_shm_key,
		    int *tool_shm_key,
		    int *traj_shm_key,
		    int *servo_shm_key,
		    int *go_io_shm_key)
{
  FILE *fp;
  const char *section;
  const char *key;
  const char *inistring;
  int servo_num;
  char servo_section[INIFILE_MAX_LINELEN];
  double d1;

  if (NULL == (fp = fopen(inifile_name, "r"))) {
    fprintf(stderr, "gosh: can't open %s\n", inifile_name);
    return 1;
  }

#define CLOSE_AND_RETURN \
  fclose(fp);		 \
  return 1

  section = "TASK";
  key = "SHM_KEY";
  inistring = ini_find(fp, key, section);
  /* the task controller is optional */
  if (NULL != inistring) {
    if (1 != sscanf(inistring, "%i", task_shm_key)) {
      fprintf(stderr, "gosh: bad entry: [%s] %s = %s\n", section, key, inistring);
      CLOSE_AND_RETURN;
    }
  }

  section = "TOOL";
  key = "SHM_KEY";
  inistring = ini_find(fp, key, section);
  /* the tool controller is optional */
  if (NULL != inistring) {
    if (1 != sscanf(inistring, "%i", tool_shm_key)) {
      fprintf(stderr, "gosh: bad entry: [%s] %s = %s\n", section, key, inistring);
      CLOSE_AND_RETURN;
    }
  }

  section = "TRAJ";
  key = "SHM_KEY";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    fprintf(stderr, "gosh: missing entry: [%s] %s\n", section, key);
    CLOSE_AND_RETURN;
  } else if (1 != sscanf(inistring, "%i", traj_shm_key)) {
    fprintf(stderr, "gosh: bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN;
  }

  section = "SERVO";
  key = "HOWMANY";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    for (servo_num = 0; servo_num < SERVO_NUM; servo_num++) {
      sprintf(servo_section, "SERVO_%d", servo_num + 1);
      if (NULL == ini_find(fp, "CYCLE_TIME", servo_section)) break;
    }
    SERVO_HOWMANY = servo_num;
    fprintf(stderr, "gosh: missing entry: [%s] %s, found and using %d\n", section, key, SERVO_HOWMANY);
  } else if (1 != sscanf(inistring, "%i", &SERVO_HOWMANY)) {
    fprintf(stderr, "gosh: bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN;
  }

  section = "SERVO";
  key = "SHM_KEY";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    fprintf(stderr, "gosh: missing entry: [%s] %s\n", section, key);
    CLOSE_AND_RETURN;
  } else if (1 != sscanf(inistring, "%i", servo_shm_key)) {
    fprintf(stderr, "gosh: bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN;
  }

  section = "GO_IO";
  key = "SHM_KEY";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    fprintf(stderr, "gosh: missing entry: [%s] %s\n", section, key);
    CLOSE_AND_RETURN;
  } else if (1 != sscanf(inistring, "%i", go_io_shm_key)) {
    fprintf(stderr, "gosh: bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN;
  }

  length_units_per_m = 1.0;
  section = "GOMOTION";
  key = "LENGTH_UNITS_PER_M";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    fprintf(stderr, "gosh: missing entry: [%s] %s, using default %f\n", section, key, length_units_per_m);
  } else if (1 != sscanf(inistring, "%lf", &d1)) {
    fprintf(stderr, "gosh: bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN;
  } else if (d1 <= 0.0) {
    fprintf(stderr, "gosh: invalid entry: [%s] %s = %s must be positive\n", section, key, inistring);
    CLOSE_AND_RETURN;
  } else {
    length_units_per_m = d1;
  }
  m_per_length_units = (go_real) (1.0 / length_units_per_m);

  angle_units_per_rad = 1.0;
  section = "GOMOTION";
  key = "ANGLE_UNITS_PER_RAD";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    fprintf(stderr, "gosh: missing entry: [%s] %s, using default %f\n", section, key, angle_units_per_rad);
  } else if (1 != sscanf(inistring, "%lf", &d1)) {
    fprintf(stderr, "gosh: bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN;
  } else if (d1 <= 0.0) {
    fprintf(stderr, "gosh: invalid entry: [%s] %s = %s must be positive\n", section, key, inistring);
    CLOSE_AND_RETURN;
  } else {
    angle_units_per_rad = d1;
  }
  rad_per_angle_units = (go_real) (1.0 / angle_units_per_rad);

  for (servo_num = 0; servo_num < SERVO_HOWMANY; servo_num++) {
    sprintf(servo_section, "SERVO_%d", servo_num + 1);
    section = servo_section;

    /*
      The joint quantity is present in the servo settings buffer, and
      we could read it from either there or the .ini file. We'll get it
      from the .ini file, since the servo loop may not yet be
      configured.
    */
    joint_quantity[servo_num] = GO_QUANTITY_NONE;
    key = "QUANTITY";
    inistring = ini_find(fp, key, section);
    if (NULL == inistring) {
      fprintf(stderr, "gosh: missing entry: [%s] %s, using default %s\n", section, key, go_quantity_to_string(joint_quantity[servo_num]));
    } else {
      if (ini_match(inistring, "ANGLE")) {
	joint_quantity[servo_num] = GO_QUANTITY_ANGLE;
      } else if (ini_match(inistring, "LENGTH")) {
	joint_quantity[servo_num] = GO_QUANTITY_LENGTH;
      } else {
	fprintf(stderr, "gosh: bad entry: [%s] %s = %s\n", section, key, inistring);
	CLOSE_AND_RETURN;
      }
    }

    home_vel[servo_num] = 0.0;
    key = "HOME_VEL";
    inistring = ini_find(fp, key, section);
    if (NULL == inistring) {
      fprintf(stderr, "gosh: missing entry: [%s] %s, using default %f\n", section, key, (double) home_vel[servo_num]);
    } else {
      if (1 == sscanf(inistring, "%lf", &d1)) {
	home_vel[servo_num] = TGQ(d1, servo_num);
      } else {
	fprintf(stderr, "gosh: bad entry: [%s] %s = %s\n", section, key, inistring);
	CLOSE_AND_RETURN;
      }
    }
  }

  fclose(fp);
  return 0;
}

int scan_ints(int *array, char *string, int len)
{
  char *nptr = string;
  char *endptr;
  int i;

  for (i = 0; i < len; i++) {
    array[i] = strtol(nptr, &endptr, 10);
    if (nptr == endptr)
      break;			/* nothing was converted */
    nptr = endptr;
  }

  /* if isgraph(*endptr) we have extra numbers; ignore */

  return i;
}

static void print_servo_stat(servo_stat_struct *stat, int which)
{
  printf("command_type:       %s\n", servo_cmd_symbol(stat->command_type));
  printf("echo_serial_number: %d\n", (int) stat->echo_serial_number);
  printf("status:             %s\n", rcs_status_to_string(stat->status));
  printf("state:              %s\n", rcs_state_to_string(stat->state));
  printf("admin_state:        %s\n",
	 rcs_admin_state_to_string(stat->admin_state));
  printf("line:               %d\n", (int) stat->line);
  printf("source_line:        %d\n", (int) stat->source_line);
  printf("source_file:        %s\n", (char *) stat->source_file);
  printf("heartbeat:          %d\n", (int) stat->heartbeat);
  printf("cycle_time:         %f\n", (double) stat->cycle_time);
  printf("setpoint:           %f\n", FGQ(stat->setpoint, which));
  /* leave raw input alone, no units are relevant */
  printf("raw_input:          %f\n", (double) stat->raw_input);
  printf("input:              %f\n", FGQ(stat->input, which));
  printf("input_latch:        %f\n", FGQ(stat->input_latch, which));
  printf("input_vel:          %f\n", FGQ(stat->input_vel, which));
  printf("output:             %f\n", FGQ(stat->output, which));
  /* similarly, leave raw output alone */
  printf("raw_output:         %f\n", (double) stat->raw_output);
  printf("homing:             %d\n", (int) stat->homing);
  printf("homed:              %d\n", (int) stat->homed);
}

static void print_servo_set(servo_set_struct *set, int which)
{
  go_rpy rpy;

  printf("id:                  %d\n", (int) set->id);
  printf("command_type:        %s\n", servo_cfg_symbol(set->command_type));
  printf("echo_serial_number:  %d\n", (int) set->echo_serial_number);
  printf("status:              %s\n", rcs_status_to_string(set->status));
  printf("state:               %s\n", rcs_state_to_string(set->state));
  printf("line:                %d\n", (int) set->line);
  printf("source_line:         %d\n", (int) set->source_line);
  printf("source_file:         %s\n", (char *) set->source_file);
  printf("link quantity:       %s\n", go_quantity_to_string(set->link.quantity));
  switch (set->link.type) {
  case GO_LINK_DH:
    printf("DH params:           %f %f %f %f\n",
	   FGL(set->link.u.dh.a),
	   FGA(set->link.u.dh.alpha),
	   FGL(set->link.u.dh.d),
	   FGA(set->link.u.dh.theta));
    break;
  case GO_LINK_PK:
    printf("PK params:           %f %f %f, %f %f %f\n",
	   FGL(set->link.u.pk.base.x),
	   FGL(set->link.u.pk.base.y),
	   FGL(set->link.u.pk.base.z),
	   FGL(set->link.u.pk.platform.x),
	   FGL(set->link.u.pk.platform.y),
	   FGL(set->link.u.pk.platform.z));
    break;
  case GO_LINK_PP:
    go_quat_rpy_convert(&set->link.u.pp.pose.rot, &rpy);
    printf("PP params:           %f %f %f %f %f %f\n",
	   FGL(set->link.u.pp.pose.tran.x),
	   FGL(set->link.u.pp.pose.tran.y),
	   FGL(set->link.u.pp.pose.tran.z),
	   FGA(rpy.r),
	   FGA(rpy.r),
	   FGA(rpy.r));
    break;
  }
  printf("cycle_time:          %f\n", (double) set->cycle_time);
  printf("debug:               0x%X\n", (int) set->debug);
  printf("active:              %s\n", set->active ? "active" : "inactive");
  printf("servo_type:          %s\n", set->servo_type == GO_SERVO_TYPE_PID ? "PID" : set->servo_type == GO_SERVO_TYPE_PASS ? "Pass" : "?");
  switch (set->servo_type) {
  case GO_SERVO_TYPE_PID:
    printf("p,i,d:               %f %f %f\n", FGQ(set->pid.p, which), FGQ(set->pid.i, which), FGQ(set->pid.d, which));
    printf("v,aff:               %f %f\n", FGQ(set->pid.vff, which), FGQ(set->pid.aff, which));
    printf("min,max_output:      %f %f\n", FGQ(set->pid.min_output, which), FGQ(set->pid.max_output, which));
    printf("pos,neg_bias:        %f %f\n", FGQ(set->pid.pos_bias, which), FGQ(set->pid.neg_bias, which));
    printf("deadband:            %f\n", FGQ(set->pid.deadband, which));
    break;
  case GO_SERVO_TYPE_PASS:
    break;
  default:
    printf("unknown servo type %d\n", (int) set->servo_type);
    break;
  }
  printf("input_scale:         %f\n", (double) set->input_scale);
  printf("output_scale         %f\n", (double) set->output_scale);
  printf("home:                %f\n", FGQ(set->home, which));
  printf("min,max_limit:       %f %f\n", (double) set->min_limit, (double) set->max_limit);
  printf("max_vel:             %f\n", FGQ(set->max_vel, which));
  printf("max_acc:             %f\n", FGQ(set->max_acc, which));
  printf("max_jerk:            %f\n", FGQ(set->max_jerk, which));

  printf("logging:            %d\n", (int) set->log_logging);
  printf("log_type:           %d\n", (int) set->log_type);
}

static void print_traj_stat(traj_stat_struct *stat)
{
  go_rpy rpy;
  int i;

  printf("command_type:       %s\n", traj_cmd_symbol(stat->command_type));
  printf("echo_serial_number: %d\n", (int) stat->echo_serial_number);
  printf("status:             %s\n", rcs_status_to_string(stat->status));
  printf("state:              %s\n", rcs_state_to_string(stat->state));
  printf("admin_state:        %s\n",
	 rcs_admin_state_to_string(stat->admin_state));
  printf("line:               %d\n", (int) stat->line);
  printf("source_line:        %d\n", (int) stat->source_line);
  printf("source_file:        %s\n", (char *) stat->source_file);
  printf("heartbeat:          %d\n", (int) stat->heartbeat);
  printf("cycle_time:         %f\n", (double) stat->cycle_time);
  printf("m/m/avg:            %f %f %f\n", 
	 (double) go_mmavg_min(&stat->mmavg),
	 (double) go_mmavg_max(&stat->mmavg),
	 (double) go_mmavg_avg(&stat->mmavg));
  printf("homed:              %s\n", stat->homed ? "HOMED" : "NOT HOMED");
  printf("frame:              %s\n",
	 stat->frame == TRAJ_WORLD_FRAME ? "World" : stat->frame ==
	 TRAJ_JOINT_FRAME ? "Joint" : "?");
  printf("inpos:              %s\n", stat->inpos ? "INPOS" : "NOT INPOS");
  printf("in queue:           %d\n", (int) stat->queue_count);
  go_quat_rpy_convert(&stat->ecp.rot, &rpy);
  printf("ecp:                %f %f %f %f %f %f\n",
	 FGL(stat->ecp.tran.x), FGL(stat->ecp.tran.y), FGL(stat->ecp.tran.z),
	 FGA(rpy.r), FGA(rpy.p), FGA(rpy.y));
  go_quat_rpy_convert(&stat->ecp_act.rot, &rpy);
  printf("ecp_act:            %f %f %f %f %f %f\n", 
	 FGL(stat->ecp_act.tran.x), FGL(stat->ecp_act.tran.y), FGL(stat->ecp_act.tran.z),
	 FGA(rpy.r), FGA(rpy.p), FGA(rpy.y));
  go_quat_rpy_convert(&stat->kcp.rot, &rpy);
  printf("kcp:                %f %f %f %f %f %f\n", 
	 FGL(stat->kcp.tran.x), FGL(stat->kcp.tran.y), FGL(stat->kcp.tran.z),
	 FGA(rpy.r), FGA(rpy.p), FGA(rpy.y));
  printf("joints:             ");
  for (i = 0; i < SERVO_HOWMANY; i++) {
    printf("%f ", FGQ(stat->joints_act[i], i));
  }
  printf("\n");
  printf("cmd_joints:         ");
  for (i = 0; i < SERVO_HOWMANY; i++) {
    printf("%f ", FGQ(stat->joints[i], i));
  }
  printf("\n");
  printf("joint_offsets:      ");
  for (i = 0; i < SERVO_HOWMANY; i++) {
    printf("%f ", FGQ(stat->joint_offsets[i], i));
  }
  printf("\n");
  printf("joints_ferror:      ");
  for (i = 0; i < SERVO_HOWMANY; i++) {
    printf("%f ", FGQ(stat->joints_ferror[i], i));
  }
  printf("\n");
}

static void print_traj_set(traj_set_struct *set)
{
  go_cart tran;
  go_rpy rpy;

  printf("command_type:       %s\n", traj_cfg_symbol(set->command_type));
  printf("echo_serial_number: %d\n", (int) set->echo_serial_number);
  printf("status:             %s\n", rcs_status_to_string(set->status));
  printf("state:              %s\n", rcs_state_to_string(set->state));
  printf("line:               %d\n", (int) set->line);
  printf("source_line:        %d\n", (int) set->source_line);
  printf("source_file:        %s\n", (char *) set->source_file);
  printf("cycle_time:         %f\n", (double) set->cycle_time);
  printf("debug:              0x%X\n", (int) set->debug);
  printf("joint_num:          %d\n", (int) set->joint_num);

  tran = set->home.tran;
  go_quat_rpy_convert(&set->home.rot, &rpy);
  rpy.r = GO_TO_DEG(rpy.r), rpy.p = GO_TO_DEG(rpy.p), rpy.y =
    GO_TO_DEG(rpy.y);

  printf("home:               %f %f %f %f %f %f\n",
	 (double) tran.x, (double) tran.y, (double) tran.z, (double) rpy.r, (double) rpy.p, (double) rpy.y);

  tran = set->tool_transform.tran;
  go_quat_rpy_convert(&set->tool_transform.rot, &rpy);
  rpy.r = GO_TO_DEG(rpy.r), rpy.p = GO_TO_DEG(rpy.p), rpy.y =
    GO_TO_DEG(rpy.y);

  printf("tool transform:     %f %f %f %f %f %f\n",
	 (double) tran.x, (double) tran.y, (double) tran.z, (double) rpy.r, (double) rpy.p, (double) rpy.y);

  tran = set->min_limit.tran;
  go_quat_rpy_convert(&set->min_limit.rot, &rpy);
  rpy.r = GO_TO_DEG(rpy.r), rpy.p = GO_TO_DEG(rpy.p), rpy.y =
    GO_TO_DEG(rpy.y);

  printf("min_limit:          %f %f %f %f %f %f\n",
	 (double) tran.x, (double) tran.y, (double) tran.z, (double) rpy.r, (double) rpy.p, (double) rpy.y);

  tran = set->max_limit.tran;
  go_quat_rpy_convert(&set->max_limit.rot, &rpy);
  rpy.r = GO_TO_DEG(rpy.r), rpy.p = GO_TO_DEG(rpy.p), rpy.y =
    GO_TO_DEG(rpy.y);

  printf("max_limit:          %f %f %f %f %f %f\n",
	 (double) tran.x, (double) tran.y, (double) tran.z, (double) rpy.r, (double) rpy.p, (double) rpy.y);

  printf("max_tvel:           %f\n", (double) set->max_tvel);
  printf("max_tacc:           %f\n", (double) set->max_tacc);
  printf("max_tjerk:          %f\n", (double) set->max_tjerk);
  printf("max_rvel:           %f\n", (double) set->max_rvel);
  printf("max_racc:           %f\n", (double) set->max_racc);
  printf("max_rjerk:          %f\n", (double) set->max_rjerk);

  printf("scale:              %f\n", (double) set->scale);
  printf("scale_v:            %f\n", (double) set->scale_v);
  printf("scale_a:            %f\n", (double) set->scale_a);
  printf("max_scale:          %f\n", (double) set->max_scale);
  printf("max_scale_v:        %f\n", (double) set->max_scale_v);
  printf("max_scale_a:        %f\n", (double) set->max_scale_a);

  printf("logging:            %d\n", (int) set->log_logging);
  printf("log_type:           %d\n", (int) set->log_type);
  printf("log_which:          %d\n", (int) set->log_which);

  printf("queue size:         %d\n", (int) set->queue_size);
}

static void print_traj_ref(traj_ref_struct *ref)
{
  go_rpy rpy;

  go_quat_rpy_convert(&ref->xinv.rot, &rpy);
  rpy.r = GO_TO_DEG(rpy.r);
  rpy.p = GO_TO_DEG(rpy.p);
  rpy.y = GO_TO_DEG(rpy.y);

  printf("xinv:               %f %f %f %f %f %f\n",
	 (double) ref->xinv.tran.x,
	 (double) ref->xinv.tran.y,
	 (double) ref->xinv.tran.z,
	 (double) rpy.r, (double) rpy.p, (double) rpy.y);
}

static void print_task_stat(task_stat_struct *stat)
{
  printf("command_type:       %s\n", task_cmd_symbol(stat->command_type));
  printf("echo_serial_number: %d\n", (int) stat->echo_serial_number);
  printf("status:             %s\n", rcs_status_to_string(stat->status));
  printf("state:              %s\n", rcs_state_to_string(stat->state));
  printf("admin_state:        %s\n",
	 rcs_admin_state_to_string(stat->admin_state));
  printf("line:               %d\n", (int) stat->line);
  printf("source_line:        %d\n", (int) stat->source_line);
  printf("source_file:        %s\n", (char *) stat->source_file);
  printf("heartbeat:          %d\n", (int) stat->heartbeat);
  printf("cycle_time:         %f\n", (double) stat->cycle_time);
  printf("state model:        %s\n", task_state_model_symbol(stat->state_model));
  if (stat->state_model == TASK_STATE_EXECUTE) {
    printf("program:            %s\n", stat->program);
  } else {
    printf("program:            (%s)\n", old_program);
  }
}

static void print_task_set(task_set_struct *set)
{
  printf("command_type:       %s\n", task_cfg_symbol(set->command_type));
  printf("echo_serial_number: %d\n", (int) set->echo_serial_number);
  printf("status:             %s\n", rcs_status_to_string(set->status));
  printf("state:              %s\n", rcs_state_to_string(set->state));
  printf("line:               %d\n", (int) set->line);
  printf("source_line:        %d\n", (int) set->source_line);
  printf("source_file:        %s\n", (char *) set->source_file);
  printf("cycle_time:         %f\n", (double) set->cycle_time);
  printf("debug:              0x%X\n", (int) set->debug);
  printf("strict:             %s\n", set->strict ? "on" : "off");
  printf("program directory:  %s\n", set->prog_dir);
}

static void print_task_errors(task_stat_struct *stat)
{
  int t;

  /* print them out oldest to newest, which will take two chunks */
  for (t = stat->error_index; t < sizeof(stat->error) / sizeof(stat->error[0]); t++) {
    printf("[%f]\t%s\n", (double) stat->error[t].timestamp, task_error_symbol(stat->error[t].code));
  }
  for (t = 0; t < stat->error_index; t++) {
    printf("[%f]\t%s\n", (double) stat->error[t].timestamp, task_error_symbol(stat->error[t].code));
  }
}

static void print_tool_stat(tool_cmd_struct *cmd, tool_stat_struct *stat)
{
  int t;

  printf("type:               %s\n", tool_cmd_symbol(cmd->type));
  printf("serial_number:      %d\n", (int) cmd->serial_number);
  printf("command_type:       %s\n", tool_cmd_symbol(stat->command_type));
  printf("echo_serial_number: %d\n", (int) stat->echo_serial_number);
  printf("status:             %s\n", rcs_status_to_string(stat->status));
  printf("state:              %s\n", rcs_state_to_string(stat->state));
  printf("admin_state:        %s\n",
	 rcs_admin_state_to_string(stat->admin_state));
  printf("line:               %d\n", (int) stat->line);
  printf("source_line:        %d\n", (int) stat->source_line);
  printf("source_file:        %s\n", (char *) stat->source_file);
  printf("heartbeat:          %d\n", (int) stat->heartbeat);
  printf("cycle_time:         %f\n", (double) stat->cycle_time);
  printf("values:\n");
  for (t = 0; t < GO_ARRAYELS(stat->value); t++) {
    printf("\t%d\t%f\n", (int) t, (double) stat->value[t]);
  }
}

static void print_tool_set(tool_set_struct *set)
{
  printf("command_type:       %s\n", tool_cfg_symbol(set->command_type));
  printf("echo_serial_number: %d\n", (int) set->echo_serial_number);
  printf("status:             %s\n", rcs_status_to_string(set->status));
  printf("state:              %s\n", rcs_state_to_string(set->state));
  printf("line:               %d\n", (int) set->line);
  printf("source_line:        %d\n", (int) set->source_line);
  printf("source_file:        %s\n", (char *) set->source_file);
  printf("cycle_time:         %f\n", (double) set->cycle_time);
  printf("debug:              0x%X\n", (int) set->debug);
}

static void print_io(go_io_struct *io, go_input_struct *in)
{
  int t;

  printf("%d AIN:\n", io->num_ain);
  for (t = 0; t < io->num_ain; t++) {
    printf("%d\t%f\n", t, (double) in->ain[t]);
  }
  printf("\n");
  printf("%d DIN:\n", io->num_din);
  for (t = 0; t < io->num_din; t++) {
    printf("%d\t%d\n", t, in->din[t]);
  }
}

/* find all of 'a' in 'b', and following chars of 'b' are space or 0 */
static int strwcmp(char *a, char *b)
{
  int i;

  i = strlen(a);
  if (0 != strncmp(a, b, i))
    return 1;
  if (0 == b[i] || isspace(b[i]))
    return 0;

  return 1;
}

/*
  Options:

  -i <inifile>   : use <inifile>
  -u unix | rtai : select a ULAPI implementation
  -t             : connect to optional task controller
  -l             : connect to optional tool controller
*/

int main(int argc, char *argv[])
{
  enum { BUFFERLEN = 80 };
  int option;
  char inifile_name[BUFFERLEN] = "gomotion.ini";
  char prompt[BUFFERLEN];
#ifndef HAVE_READLINE_READLINE_H
  char buffer[BUFFERLEN];
  int len;
#endif
  char *line;
  char *ptr;
  int SERVO_SHM_KEY, TRAJ_SHM_KEY, GO_IO_SHM_KEY, TASK_SHM_KEY, TOOL_SHM_KEY;

  servo_comm_struct *servo_comm_ptr;
  servo_cmd_struct servo_cmd[SERVO_NUM];
  servo_cfg_struct servo_cfg[SERVO_NUM];
  servo_stat_struct pp_servo_stat[2][SERVO_NUM], *servo_stat_ptr[SERVO_NUM],
    *servo_stat_test[SERVO_NUM];
  servo_set_struct pp_servo_set_struct[2][SERVO_NUM],
    *servo_set_ptr[SERVO_NUM], *servo_set_test[SERVO_NUM];

  traj_comm_struct *traj_comm_ptr;
  traj_cmd_struct traj_cmd;
  traj_cfg_struct traj_cfg;
  traj_stat_struct pp_traj_stat[2], *traj_stat_ptr, *traj_stat_test;
  traj_set_struct pp_traj_set[2], *traj_set_ptr, *traj_set_test;
  traj_ref_struct pp_traj_ref[2], *traj_ref_ptr, *traj_ref_test;

  task_comm_struct *task_comm_ptr;
  task_cmd_struct task_cmd;
  task_cfg_struct task_cfg;
  task_stat_struct pp_task_stat[2], *task_stat_ptr, *task_stat_test;
  task_set_struct pp_task_set[2], *task_set_ptr, *task_set_test;
  task_comm_struct task_comm_dummy;

  tool_comm_struct *tool_comm_ptr;
  tool_cmd_struct tool_cmd;
  tool_cfg_struct tool_cfg;
  tool_stat_struct pp_tool_stat[2], *tool_stat_ptr, *tool_stat_test;
  tool_set_struct pp_tool_set[2], *tool_set_ptr, *tool_set_test;
  tool_comm_struct tool_comm_dummy;

  go_io_struct *go_io_ptr;
  go_output_struct go_output;
  go_input_struct pp_go_input[2], *go_input_ptr, *go_input_test;
  int servo_num;
  int which_servo = 0;
  void *tmp;
  void *servo_shm = NULL;
  void *traj_shm = NULL;
  void *go_io_shm = NULL;
  void *task_shm = NULL;
  void *tool_shm = NULL;
  int use_task = 0;
  int use_tool = 0;
  enum { WHICH_STAT = 1, WHICH_SET };
  int which_print = WHICH_STAT;
  enum { WHICH_SERVO = 1, WHICH_TRAJ, WHICH_IO, WHICH_TASK, WHICH_TOOL };
  int which_point = WHICH_TRAJ;
  /* locals are all in user units */
  double local_v[SERVO_NUM];
  double local_a[SERVO_NUM];
  double local_j[SERVO_NUM];
  double local_tv;
  double local_ta;
  double local_tj;
  double local_rv;
  double local_ra;
  double local_rj;
  double local_scale_v;
  double local_scale_a;
  double end;
  double movetime;
  int start_it;
  int got_it;
  int heartbeat;
  int connect_task;
  int connect_tool;
  int retval = 0;

  connect_task = 0;
  connect_tool = 0;

#define MYRETURN(x) retval = (x); goto CLOSE

  opterr = 0;
  while (1) {
    option = getopt(argc, argv, ":i:u:tl");
    if (option == -1)
      break;

    switch (option) {
    case 'i':
      strncpy(inifile_name, optarg, BUFFERLEN);
      inifile_name[BUFFERLEN - 1] = 0;
      break;

    case 't':
      connect_task = 1;
      break;

    case 'l':
      connect_tool = 1;
      break;

    case ':':
      fprintf(stderr, "gosh: missing value for -%c\n", optopt);
      return 1;
      break;

    default:			/* '?' */
      fprintf (stderr, "gosh: unrecognized option -%c\n", optopt);
      return 1;
      break;
    }
  }
  if (optind < argc) {
    fprintf(stderr, "gosh: extra non-option characters: %s\n", argv[optind]);
    return 1;
  }

  if (ULAPI_OK != ulapi_init()) {
    return 1;
  } 

  if (0 != ini_load(inifile_name, &TASK_SHM_KEY, &TOOL_SHM_KEY, &TRAJ_SHM_KEY, &SERVO_SHM_KEY, &GO_IO_SHM_KEY)) {
    return 1;
  }

  if (go_init()) {
    fprintf(stderr, "gosh: go_init error\n");
    return 1;
  }

  /* get servo shared memory buffers */
  servo_shm = ulapi_rtm_new(SERVO_SHM_KEY, SERVO_NUM * sizeof(servo_comm_struct));
  if (NULL == servo_shm) {
    printf("can't get servo comm shm\n");
    MYRETURN(1);
  }
  servo_comm_ptr = ulapi_rtm_addr(servo_shm);
  /* set up ping-pong buffers for buffers we have */
  for (servo_num = 0; servo_num < SERVO_HOWMANY; servo_num++) {
    servo_stat_ptr[servo_num] = &pp_servo_stat[0][servo_num];
    servo_stat_test[servo_num] = &pp_servo_stat[1][servo_num];
    /*  */
    servo_set_ptr[servo_num] = &pp_servo_set_struct[0][servo_num];
    servo_set_test[servo_num] = &pp_servo_set_struct[1][servo_num];
    /* check for running servos */
    for (start_it = 0, got_it = 0, end = ulapi_time() + CONNECT_WAIT_TIME;
	 ulapi_time() < end;
	 ulapi_sleep(0.1)) {
      *servo_stat_ptr[servo_num] = servo_comm_ptr[servo_num].servo_stat;
      if (servo_stat_ptr[servo_num]->head == servo_stat_ptr[servo_num]->tail &&
	  servo_stat_ptr[servo_num]->type == SERVO_STAT_TYPE) {
	if (! start_it) {
	  start_it = 1;
	  heartbeat = servo_stat_ptr[servo_num]->heartbeat;
	}
	if (heartbeat != servo_stat_ptr[servo_num]->heartbeat) {
	  got_it = 1;
	  break;
	}
      }
    }
    if (! got_it) {
      printf("timed out connecting to servo\n");
      MYRETURN(1);
    }
    servo_cmd[servo_num].serial_number =
      servo_stat_ptr[servo_num]->echo_serial_number + SERIAL_NUMBER_OFFSET;
    /* now get settings */
    *servo_set_ptr[servo_num] = servo_comm_ptr[servo_num].servo_set;
    servo_cfg[servo_num].serial_number = servo_set_ptr[servo_num]->echo_serial_number + SERIAL_NUMBER_OFFSET;
  } /* for (servo_num) */

  /* get traj shared memory buffers */
  traj_shm = ulapi_rtm_new(TRAJ_SHM_KEY, sizeof(traj_comm_struct));
  if (NULL == traj_shm) {
    printf("can't get traj comm shm\n");
    MYRETURN(1);
  }
  traj_comm_ptr = ulapi_rtm_addr(traj_shm);
  /* set up ping-pong buffers */
  traj_stat_ptr = &pp_traj_stat[0];
  traj_stat_test = &pp_traj_stat[1];
  /*  */
  traj_set_ptr = &pp_traj_set[0];
  traj_set_test = &pp_traj_set[1];
  /*  */
  traj_ref_ptr = &pp_traj_ref[0];
  traj_ref_test = &pp_traj_ref[1];
  /* check for running traj */
  for (start_it = 0, got_it = 0, end = ulapi_time() + CONNECT_WAIT_TIME;
       ulapi_time() < end;
       ulapi_sleep(0.1)) {
    *traj_stat_ptr = traj_comm_ptr->traj_stat;
    if (traj_stat_ptr->head == traj_stat_ptr->tail &&
	traj_stat_ptr->type == TRAJ_STAT_TYPE) {
      if (! start_it) {
	start_it = 1;
	heartbeat = traj_stat_ptr->heartbeat;
      }
      if (heartbeat != traj_stat_ptr->heartbeat) {
	got_it = 1;
	break;
      }
    }
  }
  if (! got_it) {
    printf("timed out connecting to traj status\n");
    MYRETURN(1);
  }
  traj_cmd.serial_number = traj_stat_ptr->echo_serial_number + SERIAL_NUMBER_OFFSET;
  /* now get settings */
  *traj_set_ptr = traj_comm_ptr->traj_set;
  traj_cfg.serial_number = traj_set_ptr->echo_serial_number + SERIAL_NUMBER_OFFSET;
  /* and reference */
  *traj_ref_ptr = traj_comm_ptr->traj_ref;

  /* get task shared memory buffers */
  task_stat_ptr = &pp_task_stat[0];
  task_stat_test = &pp_task_stat[1];
  task_set_ptr = &pp_task_set[0];
  task_set_test = &pp_task_set[1];
#define GET_TASK							\
  task_shm = ulapi_rtm_new(TASK_SHM_KEY, sizeof(task_comm_struct));	\
  if (NULL == task_shm) {						\
    fprintf(stderr, "gosh: can't get task comm shm\n");			\
    use_task = 0;							\
    task_comm_ptr = &task_comm_dummy; /* guard against null pointers */	\
  } else {								\
    task_comm_ptr = ulapi_rtm_addr(task_shm);				\
    /* check for running task */					\
    for (start_it = 0, got_it = 0, end = ulapi_time() + CONNECT_WAIT_TIME; \
	 ulapi_time() < end;						\
	 ulapi_sleep(0.1)) {						\
      *task_stat_ptr = task_comm_ptr->task_stat;			\
      if (task_stat_ptr->head == task_stat_ptr->tail &&			\
	  task_stat_ptr->type == TASK_STAT_TYPE) {			\
	if (! start_it) {						\
	  start_it = 1;							\
	  heartbeat = task_stat_ptr->heartbeat;				\
	}								\
	if (heartbeat != task_stat_ptr->heartbeat) {			\
	  got_it = 1;							\
	  break;							\
	}								\
      }									\
    }									\
    if (! got_it) {							\
      printf("timed out connecting to task status\n");			\
      use_task = 0;							\
    } else {								\
      task_cmd.serial_number = task_stat_ptr->echo_serial_number + SERIAL_NUMBER_OFFSET;	\
      /* now get settings */						\
      *task_set_ptr = task_comm_ptr->task_set;				\
      task_cfg.serial_number = task_set_ptr->echo_serial_number + SERIAL_NUMBER_OFFSET;	\
      use_task = 1;							\
      which_point = WHICH_TASK;						\
    }									\
  }
  if (connect_task) {
    GET_TASK;
  } else {
    use_task = 0;
    task_comm_ptr = &task_comm_dummy;
  }

  /* get tool shared memory buffers */
  tool_stat_ptr = &pp_tool_stat[0];
  tool_stat_test = &pp_tool_stat[1];
  tool_set_ptr = &pp_tool_set[0];
  tool_set_test = &pp_tool_set[1];
#define GET_TOOL							\
  tool_shm = ulapi_rtm_new(TOOL_SHM_KEY, sizeof(tool_comm_struct));	\
  if (NULL == tool_shm) {						\
    fprintf(stderr, "gosh: can't get tool comm shm\n");			\
    use_tool = 0;							\
    tool_comm_ptr = &tool_comm_dummy; /* guard against null pointers */	\
  } else {								\
    tool_comm_ptr = ulapi_rtm_addr(tool_shm);				\
    /* check for running tool */					\
    for (start_it = 0, got_it = 0, end = ulapi_time() + CONNECT_WAIT_TIME; \
	 ulapi_time() < end;						\
	 ulapi_sleep(0.1)) {						\
      *tool_stat_ptr = tool_comm_ptr->tool_stat;			\
      if (tool_stat_ptr->head == tool_stat_ptr->tail &&			\
	  tool_stat_ptr->type == TOOL_STAT_TYPE) {			\
	if (! start_it) {						\
	  start_it = 1;							\
	  heartbeat = tool_stat_ptr->heartbeat;				\
	}								\
	if (heartbeat != tool_stat_ptr->heartbeat) {			\
	  got_it = 1;							\
	  break;							\
	}								\
      }									\
    }									\
    if (! got_it) {							\
      fprintf(stderr, "gosh: timed out connecting to tool status\n");			\
      use_tool = 0;							\
    } else {								\
      tool_cmd.serial_number = tool_stat_ptr->echo_serial_number + SERIAL_NUMBER_OFFSET;	\
      /* now get settings */						\
      *tool_set_ptr = tool_comm_ptr->tool_set;				\
      tool_cfg.serial_number = tool_set_ptr->echo_serial_number + SERIAL_NUMBER_OFFSET;	\
      use_tool = 1;							\
      which_point = WHICH_TOOL;						\
    }									\
  }
  if (connect_tool) {
    GET_TOOL;
  } else {
    use_tool = 0;
    tool_comm_ptr = &tool_comm_dummy;
  }

  /* set up IO buffers */
  go_io_shm = ulapi_rtm_new(GO_IO_SHM_KEY, sizeof(go_io_struct));
  if (NULL == go_io_shm) {
    fprintf(stderr, "gosh: can't get IO shm\n");
    MYRETURN(1);
  }
  go_io_ptr = ulapi_rtm_addr(go_io_shm);
  go_input_ptr = &pp_go_input[0];
  go_input_test = &pp_go_input[1];
  go_input_ptr->head = go_input_ptr->tail = 0;
  go_output.head = go_output.tail = 0;

  /* set move time to an invalid value, signifying speed-based moves */
  movetime = -1.0;

  /* set up default values for local v, a, j */
  for (servo_num = 0; servo_num < SERVO_HOWMANY; servo_num++) {
    local_v[servo_num] = 1.0;
    local_a[servo_num] = 10.0;
    local_j[servo_num] = 100.0;
  }
  local_tv = 1.0;
  local_ta = 10.0;
  local_tj = 100.0;
  local_rv = 1.0;
  local_ra = 10.0;
  local_rj = 100.0;
  local_scale_v = 1.0;
  local_scale_a = 10.0;

#ifdef HAVE_READLINE_READLINE_H
  using_history();
#endif

  while (!feof(stdin)) {
    go_rpy rpy;
    double da[12];
    int i1, i2;
    int ints[SERVO_NUM];
    int bad;

    if (which_point == WHICH_SERVO) {
      sprintf(prompt, "servo %d> ", which_servo + 1);
    } else if (which_point == WHICH_TRAJ) {
      sprintf(prompt, "traj> ");
    } else if (which_point == WHICH_TASK) {
      sprintf(prompt, "task> ");
    } else if (which_point == WHICH_TOOL) {
      sprintf(prompt, "tool> ");
    } else if (which_point == WHICH_IO) {
      sprintf(prompt, "io> ");
    } else {
      sprintf(prompt, "?> ");
    }

#ifdef HAVE_READLINE_READLINE_H
    line = readline(prompt);
    if (NULL == line) {
      break;
    }
    if (*line) {
      add_history(line);
    }
#else
    printf(prompt);
    fflush(stdout);
    if (NULL == fgets(buffer, BUFFERLEN, stdin)) {
      break;
    }
    len = strlen(buffer);
    if (len > 0)
      buffer[len - 1] = 0;	/* take off newline */
    line = buffer;
#endif

    /* read in servo status and settings, ping-pong style */
    for (servo_num = 0; servo_num < SERVO_HOWMANY; servo_num++) {
      *servo_stat_test[servo_num] = servo_comm_ptr[servo_num].servo_stat;
      if (servo_stat_test[servo_num]->head ==
	  servo_stat_test[servo_num]->tail) {
	tmp = servo_stat_ptr[servo_num];
	servo_stat_ptr[servo_num] = servo_stat_test[servo_num];
	servo_stat_test[servo_num] = tmp;
      }
      /*  */
      *servo_set_test[servo_num] = servo_comm_ptr[servo_num].servo_set;
      if (servo_set_test[servo_num]->head == servo_set_test[servo_num]->tail) {
	tmp = servo_set_ptr[servo_num];
	servo_set_ptr[servo_num] = servo_set_test[servo_num];
	servo_set_test[servo_num] = tmp;
      }
    }

    /* read in traj status and settings, ping-pong style */
    *traj_stat_test = traj_comm_ptr->traj_stat;
    if (traj_stat_test->head == traj_stat_test->tail) {
      tmp = traj_stat_ptr;
      traj_stat_ptr = traj_stat_test;
      traj_stat_test = tmp;
    }
    /*  */
    *traj_set_test = traj_comm_ptr->traj_set;
    if (traj_set_test->head == traj_set_test->tail) {
      tmp = traj_set_ptr;
      traj_set_ptr = traj_set_test;
      traj_set_test = tmp;
    }
    /*  */
    *traj_ref_test = traj_comm_ptr->traj_ref;
    if (traj_ref_test->head == traj_ref_test->tail) {
      tmp = traj_ref_ptr;
      traj_ref_ptr = traj_ref_test;
      traj_ref_test = tmp;
    }

    if (use_task) {
      /* read in task status and settings, ping-pong style */
      *task_stat_test = task_comm_ptr->task_stat;
      if (task_stat_test->head == task_stat_test->tail) {
	tmp = task_stat_ptr;
	task_stat_ptr = task_stat_test;
	task_stat_test = tmp;
      }
      /*  */
      *task_set_test = task_comm_ptr->task_set;
      if (task_set_test->head == task_set_test->tail) {
	tmp = task_set_ptr;
	task_set_ptr = task_set_test;
	task_set_test = tmp;
      }
    }

    if (use_tool) {
      /* read in tool status and settings, ping-pong style */
      *tool_stat_test = tool_comm_ptr->tool_stat;
      if (tool_stat_test->head == tool_stat_test->tail) {
	tmp = tool_stat_ptr;
	tool_stat_ptr = tool_stat_test;
	tool_stat_test = tmp;
      }
      /*  */
      *tool_set_test = tool_comm_ptr->tool_set;
      if (tool_set_test->head == tool_set_test->tail) {
	tmp = tool_set_ptr;
	tool_set_ptr = tool_set_test;
	tool_set_test = tmp;
      }
    }

    /* read in IO inputs, ping-pong style */
    *go_input_test = go_io_ptr->input;
    if (go_input_test->head == go_input_test->tail) {
      tmp = go_input_ptr;
      go_input_ptr = go_input_test;
      go_input_test = tmp;
    }

    ptr = &line[strlen(line)] - 1;
    while (ptr > line && isspace(*ptr))
      *ptr-- = 0;		/* zero trailing white space */

    ptr = line;
    while (isspace(*ptr))
      ptr++;			/* skip leading white space */

#define TRY(s) if (! strwcmp(s, ptr))
#define TRYEM(s1,s2) if (! strwcmp(s1, ptr) || ! strwcmp(s2, ptr))

    if (*ptr == 0) {
#define PRINT_EM 						\
      if (which_print == WHICH_STAT) {				\
	if (which_point == WHICH_SERVO) {			\
	  print_servo_stat(servo_stat_ptr[which_servo], which_servo); \
	} else if (which_point == WHICH_TRAJ) {			\
	  print_traj_stat(traj_stat_ptr);			\
	  print_traj_ref(traj_ref_ptr);				\
	} else if (which_point == WHICH_TASK) {			\
	  print_task_stat(task_stat_ptr);			\
	} else if (which_point == WHICH_TOOL) {			\
	  print_tool_stat(&tool_cmd, tool_stat_ptr);		\
	} else if (which_point == WHICH_IO) {			\
	  print_io(go_io_ptr, go_input_ptr);			\
	} else {						\
	  printf("specify servo #, 'traj', 'task', 'tool' or 'io'\n");  \
	}							\
      } else {							\
	if (which_point == WHICH_SERVO) {			\
	  print_servo_set(servo_set_ptr[which_servo], which_servo); \
	} else if (which_point == WHICH_TRAJ) {			\
	  print_traj_set(traj_set_ptr);				\
	} else if (which_point == WHICH_TASK) {			\
	  print_task_set(task_set_ptr);				\
	} else if (which_point == WHICH_TOOL) {			\
	  print_tool_set(tool_set_ptr);				\
	} else if (which_point == WHICH_IO) {			\
	  print_io(go_io_ptr, go_input_ptr);			\
	} else {						\
	  printf("specify servo #, 'traj', 'task', 'tool' or 'io'\n");		\
	}							\
      }
      PRINT_EM;
    } else if (1 == sscanf(ptr, "%d", &i1)) {
      if (i1 < 1 || i1 > SERVO_HOWMANY) {
	printf("need servo id 1 to %d\n", SERVO_HOWMANY);
      } else {
	which_point = WHICH_SERVO;
	which_servo = i1 - 1;
      }
    } else if (*ptr == '#') {
      continue;
#if 0
      )				/* workaround for C indenting problem */
#endif
      } else TRYEM("q", "quit") {
      break;
    } else TRY("exit") {
      if (1 == sscanf(ptr, "%*s %i", &i1)) {
	retval = i1;
      }
      break;
    } else TRY("sleep") {
      if (1 == sscanf(ptr, "%*s %lf", da)) {
	ulapi_sleep(*da);
      }
    } else TRY("servo") {
      which_point = WHICH_SERVO;
      which_print = WHICH_STAT;
      PRINT_EM;
    } else TRY("traj") {
      which_point = WHICH_TRAJ;
      which_print = WHICH_STAT;
      PRINT_EM;
    } else TRY("task") {
      if (! use_task) {
	GET_TASK;
      }
      if (use_task) {
	which_point = WHICH_TASK;
	which_print = WHICH_STAT;
	PRINT_EM;
      }
    } else TRY("tool") {
      if (! use_tool) {
	  GET_TOOL;
      }
      if (use_tool) {
	which_point = WHICH_TOOL;
	which_print = WHICH_STAT;
	PRINT_EM;
      }
    } else TRY("io") {
      which_point = WHICH_IO;
      PRINT_EM;
    } else TRY("stat") {
      which_print = WHICH_STAT;
      PRINT_EM;
    } else TRY("set") {
      which_print = WHICH_SET;
      PRINT_EM;
    } else TRYEM("init", "reset") {
      which_print = WHICH_STAT;
      if (which_point == WHICH_TASK) {
	task_cmd.type = TASK_CMD_RESET_TYPE;
#define DO_TASK_CMD \
	task_cmd.serial_number++; \
	task_cmd.tail = ++task_cmd.head; \
	task_comm_ptr->task_cmd = task_cmd; \
	which_print = WHICH_STAT
	DO_TASK_CMD;
      } else if (which_point == WHICH_TOOL) {
	tool_cmd.type = TOOL_CMD_INIT_TYPE;
#define DO_TOOL_CMD \
	tool_cmd.serial_number++; \
	tool_cmd.tail = ++tool_cmd.head; \
	tool_comm_ptr->tool_cmd = tool_cmd; \
	which_print = WHICH_STAT
	DO_TOOL_CMD;
      } else if (which_point == WHICH_TRAJ) {
	traj_cmd.type = TRAJ_CMD_INIT_TYPE;
#define DO_TRAJ_CMD \
	traj_cmd.serial_number++; \
	traj_cmd.tail = ++traj_cmd.head; \
	traj_comm_ptr->traj_cmd = traj_cmd; \
	which_print = WHICH_STAT
	DO_TRAJ_CMD;
      } else {
	servo_cmd[which_servo].type = SERVO_CMD_INIT_TYPE;
#define DO_SERVO_CMD \
	servo_cmd[which_servo].serial_number++;				\
	servo_cmd[which_servo].tail = ++servo_cmd[which_servo].head;	\
	servo_comm_ptr[which_servo].servo_cmd = servo_cmd[which_servo]; \
	which_print = WHICH_STAT
	DO_SERVO_CMD;
      }
    } else TRY("errors") {
      if (which_point == WHICH_TASK) {
	print_task_errors(task_stat_ptr);
      } else {
	printf("task controller not running\n");
      }
    } else TRY("hold") {
      if (which_point == WHICH_TASK) {
	which_print = WHICH_STAT;
	task_cmd.type = TASK_CMD_HOLD_TYPE;
	DO_TASK_CMD;
      }  else {
	printf("task controller not running\n");
      }
    } else TRY("unhold") {
      if (which_point == WHICH_TASK) {
	which_print = WHICH_STAT;
	task_cmd.type = TASK_CMD_UNHOLD_TYPE;
	DO_TASK_CMD;
      }  else {
	printf("task controller not running\n");
      }
    } else TRY("suspend") {
      if (which_point == WHICH_TASK) {
	which_print = WHICH_STAT;
	task_cmd.type = TASK_CMD_SUSPEND_TYPE;
	DO_TASK_CMD;
      }  else {
	printf("task controller not running\n");
      }
    } else TRY("unsuspend") {
      if (which_point == WHICH_TASK) {
	which_print = WHICH_STAT;
	task_cmd.type = TASK_CMD_UNSUSPEND_TYPE;
	DO_TASK_CMD;
      }  else {
	printf("task controller not running\n");
      }
    } else TRY("clear") {
      if (which_point == WHICH_TASK) {
	which_print = WHICH_STAT;
	task_cmd.type = TASK_CMD_CLEAR_TYPE;
	DO_TASK_CMD;
      }  else {
	printf("task controller not running\n");
      }
    } else TRY("abort") {
      which_print = WHICH_STAT;
      if (which_point == WHICH_TASK) {
	task_cmd.type = TASK_CMD_ABORT_TYPE;
	DO_TASK_CMD;
      } else if (which_point == WHICH_TRAJ) {
	traj_cmd.type = TRAJ_CMD_ABORT_TYPE;
	DO_TRAJ_CMD;
      } else {
	servo_cmd[which_servo].type = SERVO_CMD_ABORT_TYPE;
	DO_SERVO_CMD;
      }
    } else TRY("on") {
      if (which_point == WHICH_TOOL) {
	which_print = WHICH_STAT;
	if (2 == sscanf(ptr, "%*s %i %lf", &i1, &da[0])) {
	  tool_cmd.type = TOOL_CMD_ON_TYPE;
	  tool_cmd.id = i1;
	  tool_cmd.u.on.value = da[0];
	  DO_TOOL_CMD;
	} else {
	  printf("need tool id and output value\n");
	}
      } else {
	printf("tool controller not running\n");
      }
    } else TRY("off") {
      if (which_point == WHICH_TOOL) {
	which_print = WHICH_STAT;
	if (1 == sscanf(ptr, "%*s %i", &i1)) {
	  tool_cmd.type = TOOL_CMD_OFF_TYPE;
	  tool_cmd.id = i1;
	  DO_TOOL_CMD;
	} else {
	  printf("need tool id\n");
	}
      } else {
	printf("tool controller not running\n");
      }
    } else TRY("halt") {
      which_print = WHICH_STAT;
      if (which_point == WHICH_SERVO) {
	servo_cmd[which_servo].type = SERVO_CMD_HALT_TYPE;
	DO_SERVO_CMD;
      } else {
	traj_cmd.type = TRAJ_CMD_HALT_TYPE;
	DO_TRAJ_CMD;
      }
    } else TRY("shutdown") {
      which_print = WHICH_STAT;
      if (which_point == WHICH_TASK) {
	task_cmd.type = TASK_CMD_SHUTDOWN_TYPE;
	DO_TASK_CMD;
      } else if (which_point == WHICH_TOOL) {
	tool_cmd.type = TOOL_CMD_SHUTDOWN_TYPE;
	DO_TOOL_CMD;
      } else if (which_point == WHICH_TRAJ) {
	traj_cmd.type = TRAJ_CMD_SHUTDOWN_TYPE;
	DO_TRAJ_CMD;
      } else {
	servo_cmd[which_servo].type = SERVO_CMD_SHUTDOWN_TYPE;
	DO_SERVO_CMD;
      }
    } else TRY("nop") {
      which_print = WHICH_STAT;
      if (which_point == WHICH_SERVO) {
	servo_cmd[which_servo].type = SERVO_CMD_NOP_TYPE;
	DO_SERVO_CMD;
      } else if (which_point == WHICH_TOOL) {
	tool_cmd.type = TOOL_CMD_NOP_TYPE;
	DO_TOOL_CMD;
      } else {
	traj_cmd.type = TRAJ_CMD_NOP_TYPE;
	DO_TRAJ_CMD;
      }
    } else TRY("stub") {
      which_print = WHICH_STAT;
      if (which_point == WHICH_SERVO) {
	servo_cmd[which_servo].type = SERVO_CMD_STUB_TYPE;
	DO_SERVO_CMD;
      } else {
	traj_cmd.type = TRAJ_CMD_STUB_TYPE;
	DO_TRAJ_CMD;
      }
    } else TRY("sp") {
      which_point = WHICH_SERVO;
      which_print = WHICH_STAT;
      if (1 == sscanf(ptr, "%*s %lf", &da[0])) {
	servo_cmd[which_servo].type = SERVO_CMD_SERVO_TYPE;
	servo_cmd[which_servo].u.servo.setpoint = TGQ(da[0], which_servo);
	servo_cmd[which_servo].u.servo.home = 0;
	DO_SERVO_CMD;
      } else {
	printf("need setpoint\n");
      }
    } else TRYEM("start", "run") {
      if (use_task) {
	which_point = WHICH_TASK;
	which_print = WHICH_STAT;
	task_cmd.type = TASK_CMD_START_TYPE;
	while ((! isspace(*ptr)) && (0 != *ptr)) ptr++;
	while (isspace(*ptr)) ptr++;
	if (0 == *ptr) {
	  if (0 == *old_program) continue; /* nothing to run */
	  SAFECPY(task_cmd.u.start.program, old_program); /* run the old one */
	} else {
	  SAFECPY(task_cmd.u.start.program, ptr);
	  SAFECPY(old_program, ptr);
	}
	DO_TASK_CMD;
      } else {
	printf("task controller not running\n");
      }
    } else TRY("stop") {
      which_print = WHICH_STAT;
      if (which_point == WHICH_TASK) {
	task_cmd.type = TASK_CMD_STOP_TYPE;
	DO_TASK_CMD;
      } else {
	traj_cmd.type = TRAJ_CMD_STOP_TYPE;
	DO_TRAJ_CMD;
      }
    } else TRY("par") {
      if (which_point == WHICH_SERVO) {
	printf("%f %f %f\n", local_v[which_servo], local_a[which_servo],
	       local_j[which_servo]);
      } else {
	which_point = WHICH_TRAJ;
	printf("%f %f %f %f %f %f\n", local_tv, local_ta, local_tj, local_rv,
	       local_ra, local_rj);
      }
    } else TRY("jpar") {
      which_point = WHICH_SERVO;
      which_print = WHICH_SET;
      if (3 == sscanf(ptr, "%*s %lf %lf %lf", &da[0], &da[1], &da[2])) {
	local_v[which_servo] = da[0];
	local_a[which_servo] = da[1];
	local_j[which_servo] = da[2];
      } else {
	printf("need vaj\n");
      }
    } else TRY("tpar") {
      which_point = WHICH_TRAJ;
      which_print = WHICH_SET;
      if (3 == sscanf(ptr, "%*s %lf %lf %lf", &da[0], &da[1], &da[2])) {
	local_tv = da[0];
	local_ta = da[1];
	local_tj = da[2];
      } else {
	printf("need vaj\n");
      }
    } else TRY("rpar") {
      which_point = WHICH_TRAJ;
      which_print = WHICH_SET;
      if (3 == sscanf(ptr, "%*s %lf %lf %lf", &da[0], &da[1], &da[2])) {
	local_rv = da[0];
	local_ra = da[1];
	local_rj = da[2];
      } else {
	printf("need vaj\n");
      }
    } else TRY("time") {
      which_point = WHICH_TRAJ;
      which_print = WHICH_STAT;
      if (1 == sscanf(ptr, "%*s %lf", &da[0])) {
	movetime = da[0];
      } else {
	printf("need a time for subsequent moves\n");
      }
    } else TRY("notime") {
      which_point = WHICH_TRAJ;
      which_print = WHICH_STAT;
      movetime = -1.0;
    } else TRY("here") {
      which_point = WHICH_TRAJ;
      which_print = WHICH_STAT;
      if (6 == sscanf(ptr, "%*s %lf %lf %lf %lf %lf %lf",
		      &da[0], &da[1], &da[2], &da[3], &da[4], &da[5])) {
	traj_cmd.type = TRAJ_CMD_HERE_TYPE;
	traj_cmd.u.here.here.tran.x = TGL(da[0]);
	traj_cmd.u.here.here.tran.y = TGL(da[1]);
	traj_cmd.u.here.here.tran.z = TGL(da[2]);
	rpy.r = TGA(da[3]);
	rpy.p = TGA(da[4]);
	rpy.y = TGA(da[5]);
	go_rpy_quat_convert(&rpy, &traj_cmd.u.here.here.rot);
	DO_TRAJ_CMD;
      } else {
	printf("need x y z r p w\n");
      }
    } else TRY("moveuj") {
      which_point = WHICH_TRAJ;
      which_print = WHICH_STAT;
      if (6 == sscanf(ptr, "%*s %lf %lf %lf %lf %lf %lf",
		      &da[0], &da[1], &da[2], &da[3], &da[4], &da[5])) {
	traj_cmd.type = TRAJ_CMD_MOVE_UJOINT_TYPE;
	for (servo_num = 0; servo_num < SERVO_HOWMANY; servo_num++) {
	  traj_cmd.u.move_ujoint.d[servo_num] = TGQ(da[servo_num], servo_num);
	  traj_cmd.u.move_ujoint.v[servo_num] = TGQ(local_v[servo_num], servo_num);
	  traj_cmd.u.move_ujoint.a[servo_num] = TGQ(local_a[servo_num], servo_num);
	  traj_cmd.u.move_ujoint.j[servo_num] = TGQ(local_j[servo_num], servo_num);
	  traj_cmd.u.move_ujoint.home[servo_num] = 0;
	}
	traj_cmd.u.move_ujoint.id = traj_cmd.serial_number;
	DO_TRAJ_CMD;
      } else {
	printf("need 6 ujoint values\n");
      }
    } else TRY("movej") {
      which_point = WHICH_TRAJ;
      which_print = WHICH_STAT;
      if (6 == sscanf(ptr, "%*s %lf %lf %lf %lf %lf %lf",
		      &da[0], &da[1], &da[2], &da[3], &da[4], &da[5])) {
	traj_cmd.type = TRAJ_CMD_MOVE_JOINT_TYPE;
	for (servo_num = 0; servo_num < SERVO_HOWMANY; servo_num++) {
	  traj_cmd.u.move_joint.d[servo_num] = TGQ(da[servo_num], servo_num);
	  traj_cmd.u.move_joint.v[servo_num] = TGQ(local_v[servo_num], servo_num);
	  traj_cmd.u.move_joint.a[servo_num] = TGQ(local_a[servo_num], servo_num);
	  traj_cmd.u.move_joint.j[servo_num] = TGQ(local_j[servo_num], servo_num);
	}
	traj_cmd.u.move_joint.id = traj_cmd.serial_number;
	traj_cmd.u.move_joint.time = movetime;
	DO_TRAJ_CMD;
      } else {
	printf("need 6 joint values\n");
      }
    } else TRY("movew") {
      which_point = WHICH_TRAJ;
      which_print = WHICH_STAT;
      if (6 == sscanf(ptr, "%*s %lf %lf %lf %lf %lf %lf",
		      &da[0], &da[1], &da[2], &da[3], &da[4], &da[5])) {
	traj_cmd.type = TRAJ_CMD_MOVE_WORLD_TYPE;
	traj_cmd.u.move_world.id = traj_cmd.serial_number;
	traj_cmd.u.move_world.type = GO_MOTION_LINEAR;
	traj_cmd.u.move_world.time = movetime;
	traj_cmd.u.move_world.end.tran.x = TGL(da[0]);
	traj_cmd.u.move_world.end.tran.y = TGL(da[1]);
	traj_cmd.u.move_world.end.tran.z = TGL(da[2]);
	rpy.r = TGA(da[3]);
	rpy.p = TGA(da[4]);
	rpy.y = TGA(da[5]);
	go_rpy_quat_convert(&rpy, &traj_cmd.u.move_world.end.rot);
	traj_cmd.u.move_world.tv = TGL(local_tv);
	traj_cmd.u.move_world.ta = TGL(local_ta);
	traj_cmd.u.move_world.tj = TGL(local_tj);
	traj_cmd.u.move_world.rv = TGA(local_rv);
	traj_cmd.u.move_world.ra = TGA(local_ra);
	traj_cmd.u.move_world.rj = TGA(local_rj);
	DO_TRAJ_CMD;
      } else {
	printf("need x y z r p w\n");
      }
    } else TRY("movec") {
      which_point = WHICH_TRAJ;
      which_print = WHICH_STAT;
      if (13 ==
	  sscanf(ptr,
		 "%*s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %i",
		 &da[0], &da[1], &da[2], &da[3], &da[4], &da[5],
		 &da[6], &da[7], &da[8],
		 &da[9], &da[10], &da[11],
		 &i1)) {
	traj_cmd.type = TRAJ_CMD_MOVE_WORLD_TYPE;
	traj_cmd.u.move_world.id = traj_cmd.serial_number;
	traj_cmd.u.move_world.type = GO_MOTION_CIRCULAR;
	traj_cmd.u.move_world.time = movetime;
	traj_cmd.u.move_world.end.tran.x = TGL(da[0]);
	traj_cmd.u.move_world.end.tran.y = TGL(da[1]);
	traj_cmd.u.move_world.end.tran.z = TGL(da[2]);
	rpy.r = TGA(da[3]);
	rpy.p = TGA(da[4]);
	rpy.y = TGA(da[5]);
	go_rpy_quat_convert(&rpy, &traj_cmd.u.move_world.end.rot);
	traj_cmd.u.move_world.center.x = TGL(da[6]);
	traj_cmd.u.move_world.center.y = TGL(da[7]);
	traj_cmd.u.move_world.center.z = TGL(da[8]);
	traj_cmd.u.move_world.normal.x = TGL(da[9]);
	traj_cmd.u.move_world.normal.y = TGL(da[10]);
	traj_cmd.u.move_world.normal.z = TGL(da[11]);
	traj_cmd.u.move_world.turns = (go_integer) i1;
	traj_cmd.u.move_world.tv = TGL(local_tv);
	traj_cmd.u.move_world.ta = TGL(local_ta);
	traj_cmd.u.move_world.tj = TGL(local_tj);
	traj_cmd.u.move_world.rv = TGA(local_rv);
	traj_cmd.u.move_world.ra = TGA(local_ra);
	traj_cmd.u.move_world.rj = TGA(local_rj);
	DO_TRAJ_CMD;
      } else {
	printf("need end-xyzrpw, center-xyz, normal-xyz and turns\n");
      }
    } else TRY("movet") {
      which_point = WHICH_TRAJ;
      which_print = WHICH_STAT;
      if (6 == sscanf(ptr, "%*s %lf %lf %lf %lf %lf %lf",
		      &da[0], &da[1], &da[2], &da[3], &da[4], &da[5])) {
	traj_cmd.type = TRAJ_CMD_MOVE_TOOL_TYPE;
	traj_cmd.u.move_tool.id = traj_cmd.serial_number;
	traj_cmd.u.move_tool.type = GO_MOTION_LINEAR;
	traj_cmd.u.move_tool.time = movetime;
	traj_cmd.u.move_tool.end.tran.x = TGL(da[0]);
	traj_cmd.u.move_tool.end.tran.y = TGL(da[1]);
	traj_cmd.u.move_tool.end.tran.z = TGL(da[2]);
	rpy.r = TGA(da[3]);
	rpy.p = TGA(da[4]);
	rpy.y = TGA(da[5]);
	go_rpy_quat_convert(&rpy, &traj_cmd.u.move_tool.end.rot);
	traj_cmd.u.move_tool.tv = TGL(local_tv);
	traj_cmd.u.move_tool.ta = TGL(local_ta);
	traj_cmd.u.move_tool.tj = TGL(local_tj);
	traj_cmd.u.move_tool.rv = TGA(local_rv);
	traj_cmd.u.move_tool.ra = TGA(local_ra);
	traj_cmd.u.move_tool.rj = TGA(local_rj);
	DO_TRAJ_CMD;
      } else {
	printf("need x y z r p w\n");
      }
    } else TRY("trackw") {
      which_point = WHICH_TRAJ;
      which_print = WHICH_STAT;
      if (6 == sscanf(ptr, "%*s %lf %lf %lf %lf %lf %lf",
		      &da[0], &da[1], &da[2], &da[3], &da[4], &da[5])) {
	traj_cmd.type = TRAJ_CMD_TRACK_WORLD_TYPE;
	traj_cmd.u.track_world.position.tran.x = TGL(da[0]);
	traj_cmd.u.track_world.position.tran.y = TGL(da[1]);
	traj_cmd.u.track_world.position.tran.z = TGL(da[2]);
	rpy.r = TGA(da[3]);
	rpy.p = TGA(da[4]);
	rpy.y = TGA(da[5]);
	go_rpy_quat_convert(&rpy, &traj_cmd.u.track_world.position.rot);
	DO_TRAJ_CMD;
      } else {
	printf("need x y z r p w\n");
      }
    } else TRY("trackj") {
      which_point = WHICH_TRAJ;
      which_print = WHICH_STAT;
      if (6 == sscanf(ptr, "%*s %lf %lf %lf %lf %lf %lf",
		      &da[0], &da[1], &da[2], &da[3], &da[4], &da[5])) {
	traj_cmd.type = TRAJ_CMD_TRACK_JOINT_TYPE;
	for (servo_num = 0; servo_num < SERVO_HOWMANY; servo_num++) {
	  traj_cmd.u.track_joint.joints[servo_num] = TGQ(da[servo_num], servo_num);
	}
	DO_TRAJ_CMD;
      } else {
	printf("need 6 joint values\n");
      }
    } else TRY("teleopw") {
      which_point = WHICH_TRAJ;
      which_print = WHICH_STAT;
      if (6 == sscanf(ptr, "%*s %lf %lf %lf %lf %lf %lf",
		      &da[0], &da[1], &da[2], &da[3], &da[4], &da[5])) {
	traj_cmd.type = TRAJ_CMD_TELEOP_WORLD_TYPE;
	traj_cmd.u.teleop_world.tv.v.x = TGL(da[0]);
	traj_cmd.u.teleop_world.tv.v.y = TGL(da[1]);
	traj_cmd.u.teleop_world.tv.v.z = TGL(da[2]);
	traj_cmd.u.teleop_world.tv.w.x = TGA(da[3]);
	traj_cmd.u.teleop_world.tv.w.y = TGA(da[4]);
	traj_cmd.u.teleop_world.tv.w.z = TGA(da[5]);
	traj_cmd.u.teleop_world.ta = TGL(local_ta);
	traj_cmd.u.teleop_world.tj = TGL(local_tj);
	traj_cmd.u.teleop_world.ra = TGA(local_ra);
	traj_cmd.u.teleop_world.rj = TGA(local_rj);
	DO_TRAJ_CMD;
      } else {
	printf("need x y z r p w\n");
      }
    } else TRY("home") {	/* e.g., home 1 5 6 */
      which_point = WHICH_TRAJ;
      which_print = WHICH_STAT;
      ptr += strlen("home");	/* move ptr to args */
      traj_cmd.type = TRAJ_CMD_MOVE_UJOINT_TYPE;
      for (servo_num = 0; servo_num < SERVO_HOWMANY; servo_num++) {
	traj_cmd.u.move_ujoint.d[servo_num] = servo_stat_ptr[servo_num]->input;
	traj_cmd.u.move_ujoint.v[servo_num] = TGQ(local_v[servo_num], servo_num);
	traj_cmd.u.move_ujoint.a[servo_num] = TGQ(local_a[servo_num], servo_num);
	traj_cmd.u.move_ujoint.j[servo_num] = TGQ(local_j[servo_num], servo_num);
	traj_cmd.u.move_ujoint.home[servo_num] = 0;
      }
      i1 = scan_ints(ints, ptr, SERVO_HOWMANY) - 1;	/* i1 is the max index */
      if (i1 == -1) {
	printf("need some of 1..%d\n", SERVO_HOWMANY);
	continue;
      }
      bad = 0;
      while (i1 >= 0 && i1 < SERVO_HOWMANY) {
	i2 = ints[i1] - 1;	/* i2 is now the which[] index */
	if (i2 < 0 || i2 >= SERVO_HOWMANY) {
	  bad = 1;
	  break;
	}
	/* move this joint incrementally to 1 more than its max range,
	   in positive direction, to guarantee that it will see home */
	if (home_vel[i2] < -GO_REAL_EPSILON) {
	traj_cmd.u.move_ujoint.d[i2] +=
	  (servo_set_ptr[i2]->max_limit - servo_set_ptr[i2]->min_limit + 1.0);
	traj_cmd.u.move_ujoint.v[i2] = -home_vel[i2];
	} else if (home_vel[i2] > GO_REAL_EPSILON) {
	  traj_cmd.u.move_ujoint.d[i2] -=
	    (servo_set_ptr[i2]->max_limit - servo_set_ptr[i2]->min_limit + 1.0);
	  traj_cmd.u.move_ujoint.v[i2] = home_vel[i2];
	}
	/* else leave it here, at default speed */
	traj_cmd.u.move_ujoint.home[i2] = 1;
	printf("homing joint %d to %f\n", i2 + 1, (double) traj_cmd.u.move_ujoint.d[i2]);
	i1--;
      }
      if (bad) {
	printf("joint out of range\n");
      } else {
	traj_cmd.u.move_ujoint.id = traj_cmd.serial_number;
	DO_TRAJ_CMD;
      }
    } else TRY("ct") {
      which_print = WHICH_SET;
      if (1 == sscanf(ptr, "%*s %lf", &da[0]) && da[0] > 0.0) {
	if (which_point == WHICH_SERVO) {
	  servo_cfg[which_servo].type = SERVO_CFG_CYCLE_TIME_TYPE;
	  servo_cfg[which_servo].u.cycle_time.cycle_time = da[0];
#define DO_SERVO_CFG \
	  servo_cfg[which_servo].serial_number++; \
	  servo_cfg[which_servo].tail = ++servo_cfg[which_servo].head; \
	  servo_comm_ptr[which_servo].servo_cfg = servo_cfg[which_servo]; \
	  which_point = WHICH_SERVO, which_print = WHICH_SET
	  DO_SERVO_CFG;
	} else if (which_point == WHICH_TASK) {
	  task_cfg.type = TASK_CFG_CYCLE_TIME_TYPE;
	  task_cfg.u.cycle_time.cycle_time = da[0];
#define DO_TASK_CFG \
	  task_cfg.serial_number++; \
	  task_cfg.tail = ++task_cfg.head; \
	  task_comm_ptr->task_cfg = task_cfg; \
	  which_point = WHICH_TASK, which_print = WHICH_SET
	  DO_TASK_CFG;
	} else {
	  which_point = WHICH_TRAJ;
	  traj_cfg.type = TRAJ_CFG_CYCLE_TIME_TYPE;
	  traj_cfg.u.cycle_time.cycle_time = da[0];
#define DO_TRAJ_CFG \
	  traj_cfg.serial_number++; \
	  traj_cfg.tail = ++traj_cfg.head; \
	  traj_comm_ptr->traj_cfg = traj_cfg; \
	  which_point = WHICH_TRAJ, which_print = WHICH_SET
	  DO_TRAJ_CFG;
	}
      } else {
	printf("need positive cycle time\n");
      }
    } else TRY("debug") {
      which_print = WHICH_SET;
      if (1 == sscanf(ptr, "%*s %i", &i1)) {
	if (which_point == WHICH_SERVO) {
	  servo_cfg[which_servo].type = SERVO_CFG_DEBUG_TYPE;
	  servo_cfg[which_servo].u.debug.debug = i1;
	  DO_SERVO_CFG;
	} else if (which_point == WHICH_TRAJ) {
	  traj_cfg.type = TRAJ_CFG_DEBUG_TYPE;
	  traj_cfg.u.debug.debug = i1;
	  DO_TRAJ_CFG;
	} else {
	  which_point = WHICH_TASK;
	  task_cfg.type = TASK_CFG_DEBUG_TYPE;
	  task_cfg.u.debug.debug = i1;
	  DO_TASK_CFG;
	}
      } else {
	printf("need integer\n");
      }
    } else TRY("strict") {
      if (which_point == WHICH_TASK) {
	which_print = WHICH_SET;
	while ((! isspace(*ptr)) && (0 != *ptr)) ptr++;
	while (isspace(*ptr)) ptr++;
	if (! strcmp(ptr, "on")) {
	  task_cfg.type = TASK_CFG_STRICT_TYPE;
	  task_cfg.u.strict.strict = 1;
	  DO_TASK_CFG;
	} else if (! strcmp(ptr, "off")) {
	  task_cfg.type = TASK_CFG_STRICT_TYPE;
	  task_cfg.u.strict.strict = 0;
	  DO_TASK_CFG;
	} else {
	  printf("need 'on' or 'off'\n");
	}
      } else {
	printf("task controller not running\n");
      }
    } else TRY("tooltrans") {
      which_point = WHICH_TRAJ;
      which_print = WHICH_SET;
      if (6 == sscanf(ptr, "%*s %lf %lf %lf %lf %lf %lf",
		      &da[0], &da[1], &da[2], &da[3], &da[4], &da[5])) {
	traj_cfg.type = TRAJ_CFG_TOOL_TRANSFORM_TYPE;
	traj_cfg.u.tool_transform.tool_transform.tran.x = TGL(da[0]);
	traj_cfg.u.tool_transform.tool_transform.tran.y = TGL(da[1]);
	traj_cfg.u.tool_transform.tool_transform.tran.z = TGL(da[2]);
	rpy.r = TGA(da[3]);
	rpy.p = TGA(da[4]);
	rpy.y = TGA(da[5]);
	go_rpy_quat_convert(&rpy, &traj_cfg.u.tool_transform.tool_transform.rot);
	DO_TRAJ_CFG;
      } else {
	printf("need tool pose\n");
      }
    } else TRY("scale") {
      which_point = WHICH_TRAJ;
      which_print = WHICH_SET;
      if (3 == sscanf(ptr, "%*s %lf %lf %lf", &da[0], &da[1], &da[2])) {
	traj_cfg.type = TRAJ_CFG_SCALE_TYPE;
	traj_cfg.u.scale.scale = da[0];
	traj_cfg.u.scale.scale_v = local_scale_v = da[1];
	traj_cfg.u.scale.scale_a = local_scale_a = da[2];
	DO_TRAJ_CFG;
      } else if (1 == sscanf(ptr, "%*s %lf", &da[0])) {
	traj_cfg.type = TRAJ_CFG_SCALE_TYPE;
	traj_cfg.u.scale.scale = da[0];
	traj_cfg.u.scale.scale_v = local_scale_v;
	traj_cfg.u.scale.scale_a = local_scale_a;
	DO_TRAJ_CFG;
      } else {
	printf("need scale factor, optional v a\n");
      }
    } else TRY("profile") {
      which_point = WHICH_TRAJ;
      which_print = WHICH_SET;
      if (6 == sscanf(ptr, "%*s %lf %lf %lf %lf %lf %lf", &da[0], &da[1], &da[2], &da[3], &da[4], &da[5])) {
	traj_cfg.type = TRAJ_CFG_PROFILE_TYPE;
	traj_cfg.u.profile.max_tvel = da[0];
	traj_cfg.u.profile.max_tacc = da[1];
	traj_cfg.u.profile.max_tjerk = da[2];
	traj_cfg.u.profile.max_rvel = da[3];
	traj_cfg.u.profile.max_racc = da[4];
	traj_cfg.u.profile.max_rjerk = da[5];
	DO_TRAJ_CFG;
      } else {
	printf("need tv ta tj rv ra rj\n");
      }
    } else TRY("cfghome") {
      which_point = WHICH_SERVO;
      which_print = WHICH_SET;
      if (1 == sscanf(ptr, "%*s %lf", &da[0])) {
	servo_cfg[which_servo].type = SERVO_CFG_HOME_TYPE;
	servo_cfg[which_servo].u.home.home = TGQ(da[0], which_servo);
	DO_SERVO_CFG;
      } else {
	printf("need home\n");
      }
    } else TRY("pid") {
      which_point = WHICH_SERVO;
      which_print = WHICH_SET;
      if (3 == sscanf(ptr, "%*s %lf %lf %lf", &da[0], &da[1], &da[2])) {
	servo_cfg[which_servo].type = SERVO_CFG_PID_TYPE;
	/* fill in everything with what's current */
	servo_cfg[which_servo].u.pid = servo_set_ptr[which_servo]->pid;
	/* and just overwrite what we want to set */
	servo_cfg[which_servo].u.pid.p = TGQ(da[0], which_servo);
	servo_cfg[which_servo].u.pid.i = TGQ(da[1], which_servo);
	servo_cfg[which_servo].u.pid.d = TGQ(da[2], which_servo);
	DO_SERVO_CFG;
      } else {
	printf("need P I D\n");
      }
    } else TRY("bias") {
      which_point = WHICH_SERVO;
      which_print = WHICH_SET;
      if (2 == sscanf(ptr, "%*s %lf %lf", &da[0], &da[1])) {
	servo_cfg[which_servo].type = SERVO_CFG_PID_TYPE;
	/* fill in everything with what's current */
	servo_cfg[which_servo].u.pid = servo_set_ptr[which_servo]->pid;
	/* and just overwrite what we want to set */
	/* these don't need to be unit converted */
	servo_cfg[which_servo].u.pid.pos_bias = da[0];
	servo_cfg[which_servo].u.pid.neg_bias = da[1];
	DO_SERVO_CFG;
      } else {
	printf("need pos, neg biases\n");
      }
    } else TRY("log") {
      /* FIXME-- we assume servo logging, add something for traj */
      which_point = WHICH_SERVO;
      which_print = WHICH_SET;
      if (2 == sscanf(ptr, "%*s %d %d", &i1, &i2)) {
	servo_cfg[which_servo].type = SERVO_CFG_LOG_TYPE;
	servo_cfg[which_servo].u.log.log_type = (go_integer) i1;
	servo_cfg[which_servo].u.log.log_size = (go_integer) i2;
	DO_SERVO_CFG;
      } else {
	printf("need <log type> <log size>\n");
      }
    } else TRY("aout") {
      which_point = WHICH_IO;
      if (2 == sscanf(ptr, "%*s %i %lf", &i1, &da[0])) {
	if (i1 < 0 || i1 >= GO_IO_NUM_AOUT) {
	  printf("need <index> between 0 and %d\n", GO_IO_NUM_AOUT);
	} else {
	  go_output.aout[i1] = da[0];
	}
      } else {
	printf("need <index> <output>\n");
      }
    } else TRY("dout") {
      which_point = WHICH_IO;
      if (2 == sscanf(ptr, "%*s %i %i", &i1, &i2)) {
	if (i1 < 0 || i1 >= GO_IO_NUM_DOUT) {
	  printf("need <index> between 0 and %d\n", GO_IO_NUM_DOUT);
	} else {
	  go_output.dout[i1] = (i2 != 0);
	}
      } else {
	printf("need <index> <output>\n");
      }
    } else TRY("sizes") {
      printf("sizeof(traj_cmd_struct) = %d\n", (int) sizeof(traj_cmd_struct));
      printf("sizeof(traj_stat_struct) = %d\n", (int) sizeof(traj_stat_struct));
      printf("sizeof(traj_cfg_struct) = %d\n", (int) sizeof(traj_cfg_struct));
      printf("sizeof(traj_set_struct) = %d\n", (int) sizeof(traj_set_struct));
      printf("sizeof(traj_comm_struct) = %d\n", (int) sizeof(traj_comm_struct));
      printf("SERVO_NUM = %d\n", SERVO_NUM);
      printf("sizeof(servo_cmd_struct) = %d\n", (int) sizeof(servo_cmd_struct));
      printf("sizeof(servo_stat_struct) = %d\n", (int) sizeof(servo_stat_struct));
      printf("sizeof(servo_cfg_struct) = %d\n", (int) sizeof(servo_cfg_struct));
      printf("sizeof(servo_set_struct) = %d\n", (int) sizeof(servo_set_struct));
      printf("sizeof(servo_comm_struct) = %d\n", (int) sizeof(servo_comm_struct));
      printf("sizeof(go_io_struct) = %d\n", (int) sizeof(go_io_struct));
    } else {
      printf("?\n");
    }

#ifdef HAVE_READLINE_READLINE_H
    free(line);
#endif

  }				/* while (! feof(stdin)) */

CLOSE:
  if (NULL != servo_shm) {
    ulapi_rtm_delete(servo_shm);
    servo_shm = NULL;
  }
  if (NULL != traj_shm) {
    ulapi_rtm_delete(traj_shm);
    traj_shm = NULL;
  }
  if (NULL != task_shm) {
    ulapi_rtm_delete(task_shm);
    task_shm = NULL;
  }
  if (NULL != tool_shm) {
    ulapi_rtm_delete(tool_shm);
    tool_shm = NULL;
  }
  if (NULL != go_io_shm) {
    ulapi_rtm_delete(go_io_shm);
    go_io_shm = NULL;
  }

  (void) ulapi_exit();

  return retval;
}
