/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <inifile.h>
#include <ulapi.h>
#include <signal.h>
#include "go.h"
#include "gorcs.h"
#include "taskintf.h"
#include "trajintf.h"
#include "toolintf.h"
#include "variates.h"
#include "rs274ngc.h"
#include "rs274ngc_return.h"
#include "interplist.h"

#define CONNECT_WAIT_TIME 10.0

#define DEFAULT_CYCLE_TIME 0.1

#define CMD_PRINT_1(x) if (set->debug & DEBUG_CMD) ulapi_print(x)
#define CMD_PRINT_2(x,y) if (set->debug & DEBUG_CMD) ulapi_print(x, y)
#define CMD_PRINT_3(x,y,z) if (set->debug & DEBUG_CMD) ulapi_print(x, y, z)
#define CMD_PRINT_4(x,y,z,u) if (set->debug & DEBUG_CMD) ulapi_print(x, y, z, u)

#define CFG_PRINT_1(x) if (set->debug & DEBUG_CFG) ulapi_print(x)
#define CFG_PRINT_2(x,y) if (set->debug & DEBUG_CFG) ulapi_print(x, y)
#define CFG_PRINT_3(x,y,z) if (set->debug & DEBUG_CFG) ulapi_print(x, y, z)
#define CFG_PRINT_4(x,y,z,u) if (set->debug & DEBUG_CFG) ulapi_print(x, y, z, u)

#define SAFECPY(dst,src) strncpy(dst,src,sizeof(dst)); (dst)[sizeof(dst)-1] = 0

enum { BUFFERLEN = 256 };
#define DEFAULT_INI_FILE "gomotion.ini"
#define DEFAULT_PROG_DIR "scripts"
#define DEFAULT_PARAMETER_FILE_NAME ""
#define DEFAULT_TOOL_FILE_NAME ""
static char inifile_name[BUFFERLEN] = DEFAULT_INI_FILE;
static char prog_dir[BUFFERLEN] = DEFAULT_PROG_DIR;
static char parameter_file_name[BUFFERLEN] = DEFAULT_PARAMETER_FILE_NAME;
static char tool_file_name[BUFFERLEN] = DEFAULT_TOOL_FILE_NAME;

/*
 This flag is set when the task controller has generated a simulated
 failure, based on the statistical distributions parameterized by the
 MTTF and MTTR entries in the TASK section of the initialization file.
 'in_failure' triggers the failure of the control state tables and
 prevents future control actions until it is cleared by the simulator.
*/
static int in_failure = 0;

static int do_exit = 0;
static void quit(int sig)
{
  do_exit = 1;
}

static traj_comm_struct *traj_comm_ptr = NULL;
static tool_comm_struct *tool_comm_ptr = NULL;

static void *runproc = NULL;
static double old_scale = 1.0;

/*
  The global interp list, written by the Go Motion canonical interface
  functions, read by do_cmd_execute.
*/
interplist_struct task_interplist;
/*
  The global Go units, written here in ini_load and referenced in
  the Go canonical interface.
*/
double length_units_per_m = 1.0;
double m_per_length_units = 1.0;
double angle_units_per_rad = 1.0;
double rad_per_angle_units = 1.0;

/* the time in seconds we make it take for transitions from x-ing to x-ed */
#define TRANSITION_TIME 1.0

static void write_traj_cmd(traj_cmd_struct *traj_cmd)
{
  traj_cmd->tail = ++traj_cmd->head;
  traj_cmd->serial_number++;
  traj_comm_ptr->traj_cmd = *traj_cmd;
}

static void resend_traj_cmd(traj_cmd_struct *traj_cmd)
{
  traj_comm_ptr->traj_cmd = *traj_cmd;
}

static void write_traj_cfg(traj_cfg_struct *traj_cfg)
{
  traj_cfg->tail = ++traj_cfg->head;
  traj_cfg->serial_number++;
  traj_comm_ptr->traj_cfg = *traj_cfg;
}

static void resend_traj_cfg(traj_cfg_struct *traj_cfg)
{
  traj_comm_ptr->traj_cfg = *traj_cfg;
}

static void write_tool_cmd(tool_cmd_struct *tool_cmd)
{
  tool_cmd->tail = ++tool_cmd->head;
  tool_cmd->serial_number++;
  tool_comm_ptr->tool_cmd = *tool_cmd;
}

static void resend_tool_cmd(tool_cmd_struct *tool_cmd)
{
  tool_comm_ptr->tool_cmd = *tool_cmd;
}

static void add_task_error(task_stat_struct *stat, task_set_struct *set, task_error_code code)
{
  ulapi_real timestamp;

  /* normalize the error index in case it's bad */
  if (stat->error_index < 0) stat->error_index = 0;
  else if (stat->error_index >= TASK_ERROR_MAX) stat->error_index = TASK_ERROR_MAX - 1;
  timestamp = ulapi_time();
  stat->error[stat->error_index].timestamp = timestamp;
  stat->error[stat->error_index].code = code;
  stat->error_index++;
  if (stat->error_index >= TASK_ERROR_MAX) stat->error_index = 0;

  CMD_PRINT_3("task: %f\t%s\n", (double) timestamp, task_error_symbol(code));

  return;
}

static task_error_code get_task_error(task_stat_struct *stat)
{
  ulapi_integer index;

  index = stat->error_index - 1;
  if (index < 0) index = TASK_ERROR_MAX - 1;

  return stat->error[index].code;
}

static void do_cmd_nop(task_stat_struct *stat, task_set_struct *set)
{
  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_1("task: cmd nop\n");
    go_state_new(stat);
    go_status_next(stat, GO_RCS_STATUS_DONE);
    go_state_next(stat, GO_RCS_STATE_S0);
  } else {			/* S0 */
    go_state_default(stat);
  }
}

static void do_cmd_clear(task_stat_struct *stat, task_set_struct *set)
{
  static ulapi_real dclock;

  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_1("task: cmd clear\n");
    go_state_new(stat);
    if (stat->state_model != TASK_STATE_ABORTED && set->strict) {
      add_task_error(stat, set, TASK_ERROR_IMPROPER_COMMAND);
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
    } else {
      stat->state_model = TASK_STATE_CLEARING;
      dclock = TRANSITION_TIME;
      go_status_next(stat, GO_RCS_STATUS_EXEC);
      go_state_next(stat, GO_RCS_STATE_S1);
    }
  } else if (go_state_match(stat, GO_RCS_STATE_S1)) {
    dclock -= set->cycle_time;
    if (dclock <= 0.0) {
      stat->state_model = TASK_STATE_STOPPED;
      go_status_next(stat, GO_RCS_STATUS_DONE);
      go_state_next(stat, GO_RCS_STATE_S0);
    }
  } else {			/* S0 */
    go_state_default(stat);
  }
}

static void do_cmd_hold(task_stat_struct *stat, task_set_struct *set, traj_cfg_struct *traj_cfg, traj_set_struct *traj_set)
{
  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_1("task: cmd hold\n");
    go_state_new(stat);
    if (stat->state_model != TASK_STATE_EXECUTE && set->strict) {
      add_task_error(stat, set, TASK_ERROR_IMPROPER_COMMAND);
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
    } else {
      traj_cfg->type = TRAJ_CFG_SCALE_TYPE;
      old_scale = traj_set->scale;
      traj_cfg->u.scale.scale = 0;
      traj_cfg->u.scale.scale_v = traj_set->scale_v;
      traj_cfg->u.scale.scale_a = traj_set->scale_a;
      write_traj_cfg(traj_cfg);
      stat->state_model = TASK_STATE_HOLDING;
      go_status_next(stat, GO_RCS_STATUS_EXEC);
      go_state_next(stat, GO_RCS_STATE_S1);
    }
  } else if (go_state_match(stat, GO_RCS_STATE_S1)) {
    if (traj_set->command_type == TRAJ_CFG_SCALE_TYPE &&
	traj_set->echo_serial_number == traj_cfg->serial_number) {
      if (traj_set->status == GO_RCS_STATUS_DONE) {
	stat->state_model = TASK_STATE_HELD;
	go_status_next(stat, GO_RCS_STATUS_DONE);
	go_state_next(stat, GO_RCS_STATE_S0);
      } else if (traj_set->status == GO_RCS_STATUS_ERROR) {
	stat->state_model = TASK_STATE_HELD;
	add_task_error(stat, set, TASK_ERROR_MOTION);
	go_status_next(stat, GO_RCS_STATUS_ERROR);
	go_state_next(stat, GO_RCS_STATE_S0);
      } /* else still executing */
    } else {
      /*
	General comment on resending commands: the command didn't get
	there yet -- it may be due to the script still executing for a
	short period of time and overwriting our command. Resend the
	command.
      */
      resend_traj_cfg(traj_cfg);
    }
  } else {			/* S0 */
    go_state_default(stat);
  }
}

static void do_cmd_unhold(task_stat_struct *stat, task_set_struct *set, traj_cfg_struct *traj_cfg, traj_set_struct *traj_set)
{
  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_1("task: cmd unhold\n");
    go_state_new(stat);
    if (stat->state_model != TASK_STATE_HELD && set->strict) {
      add_task_error(stat, set, TASK_ERROR_IMPROPER_COMMAND);
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
    } else {
      traj_cfg->type = TRAJ_CFG_SCALE_TYPE;
      traj_cfg->u.scale.scale = old_scale;
      traj_cfg->u.scale.scale_v = traj_set->scale_v;
      traj_cfg->u.scale.scale_a = traj_set->scale_a;
      write_traj_cfg(traj_cfg);
      stat->state_model = TASK_STATE_UNHOLDING;
      go_status_next(stat, GO_RCS_STATUS_EXEC);
      go_state_next(stat, GO_RCS_STATE_S1);
    }
  } else if (go_state_match(stat, GO_RCS_STATE_S1)) {
    if (traj_set->command_type == TRAJ_CFG_SCALE_TYPE &&
	traj_set->echo_serial_number == traj_cfg->serial_number) {
      if (traj_set->status == GO_RCS_STATUS_DONE) {
	stat->state_model = TASK_STATE_EXECUTE;
	go_status_next(stat, GO_RCS_STATUS_DONE);
	go_state_next(stat, GO_RCS_STATE_S0);
      } else if (traj_set->status == GO_RCS_STATUS_ERROR) {
	stat->state_model = TASK_STATE_EXECUTE;
	add_task_error(stat, set, TASK_ERROR_MOTION);
	go_status_next(stat, GO_RCS_STATUS_ERROR);
	go_state_next(stat, GO_RCS_STATE_S0);
      } /* else still executing */
    } else {
      /* resend, per earlier comment on resending */
      resend_traj_cfg(traj_cfg);
    }
  } else {			/* S0 */
    go_state_default(stat);
  }
}

static void do_cmd_suspend(task_stat_struct *stat, task_set_struct *set, traj_cfg_struct *traj_cfg, traj_set_struct *traj_set)
{
  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_1("task: cmd suspend\n");
    go_state_new(stat);
    if (stat->state_model != TASK_STATE_EXECUTE && set->strict) {
      add_task_error(stat, set, TASK_ERROR_IMPROPER_COMMAND);
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
    } else {
      traj_cfg->type = TRAJ_CFG_SCALE_TYPE;
      old_scale = traj_set->scale;
      traj_cfg->u.scale.scale = 0;
      traj_cfg->u.scale.scale_v = traj_set->scale_v;
      traj_cfg->u.scale.scale_a = traj_set->scale_a;
      write_traj_cfg(traj_cfg);
      stat->state_model = TASK_STATE_SUSPENDING;
      go_status_next(stat, GO_RCS_STATUS_EXEC);
      go_state_next(stat, GO_RCS_STATE_S1);
    }
  } else if (go_state_match(stat, GO_RCS_STATE_S1)) {
    if (traj_set->command_type == TRAJ_CFG_SCALE_TYPE &&
	traj_set->echo_serial_number == traj_cfg->serial_number) {
      if (traj_set->status == GO_RCS_STATUS_DONE) {
	stat->state_model = TASK_STATE_SUSPENDED;
	go_status_next(stat, GO_RCS_STATUS_DONE);
	go_state_next(stat, GO_RCS_STATE_S0);
      } else if (traj_set->status == GO_RCS_STATUS_ERROR) {
	stat->state_model = TASK_STATE_SUSPENDED;
	add_task_error(stat, set, TASK_ERROR_MOTION);
	go_status_next(stat, GO_RCS_STATUS_ERROR);
	go_state_next(stat, GO_RCS_STATE_S0);
      } /* else still executing */
    } else {
      /* resend, per earlier comment on resending */
      resend_traj_cfg(traj_cfg);
    }
  } else {			/* S0 */
    go_state_default(stat);
  }
}

static void do_cmd_unsuspend(task_stat_struct *stat, task_set_struct *set, traj_cfg_struct *traj_cfg, traj_set_struct *traj_set)
{
  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_1("task: cmd unsuspend\n");
    go_state_new(stat);
    if (stat->state_model != TASK_STATE_SUSPENDED && set->strict) {
      add_task_error(stat, set, TASK_ERROR_IMPROPER_COMMAND);
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
    } else {
      traj_cfg->type = TRAJ_CFG_SCALE_TYPE;
      traj_cfg->u.scale.scale = old_scale;
      traj_cfg->u.scale.scale_v = traj_set->scale_v;
      traj_cfg->u.scale.scale_a = traj_set->scale_a;
      write_traj_cfg(traj_cfg);
      stat->state_model = TASK_STATE_UNSUSPENDING;
      go_status_next(stat, GO_RCS_STATUS_EXEC);
      go_state_next(stat, GO_RCS_STATE_S1);
    }
  } else if (go_state_match(stat, GO_RCS_STATE_S1)) {
    if (traj_set->command_type == TRAJ_CFG_SCALE_TYPE &&
	traj_set->echo_serial_number == traj_cfg->serial_number) {
      if (traj_set->status == GO_RCS_STATUS_DONE) {
	stat->state_model = TASK_STATE_EXECUTE;
	go_status_next(stat, GO_RCS_STATUS_DONE);
	go_state_next(stat, GO_RCS_STATE_S0);
      } else if (traj_set->status == GO_RCS_STATUS_ERROR) {
	stat->state_model = TASK_STATE_EXECUTE;
	add_task_error(stat, set, TASK_ERROR_MOTION);
	go_status_next(stat, GO_RCS_STATUS_ERROR);
	go_state_next(stat, GO_RCS_STATE_S0);
      } /* else still executing */
    } else {
      /* resend, per earlier comment on resending */
      resend_traj_cfg(traj_cfg);
    }
  } else {			/* S0 */
    go_state_default(stat);
  }
}

static void do_cmd_reset(task_stat_struct *stat, task_set_struct *set, traj_cmd_struct *traj_cmd, traj_stat_struct *traj_stat, tool_cmd_struct *tool_cmd, tool_stat_struct *tool_stat)
{
  static ulapi_real dclock;

  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_1("task: cmd reset\n");
    go_state_new(stat);
    if (NULL != runproc) {
      ulapi_process_stop(runproc);
      ulapi_process_delete(runproc);
      runproc = NULL;
    }
    if (stat->state_model != TASK_STATE_COMPLETE &&
	stat->state_model != TASK_STATE_STOPPED && set->strict) {
      add_task_error(stat, set, TASK_ERROR_IMPROPER_COMMAND);
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
    } else {
      traj_cmd->type = TRAJ_CMD_INIT_TYPE;
      write_traj_cmd(traj_cmd);
      tool_cmd->type = TOOL_CMD_INIT_TYPE;
      write_tool_cmd(tool_cmd);
      stat->state_model = TASK_STATE_RESETTING;
      go_status_next(stat, GO_RCS_STATUS_EXEC);
      go_state_next(stat, GO_RCS_STATE_S1);
    }
  } else if (go_state_match(stat, GO_RCS_STATE_S1)) {
    /* first check traj, then tool */
    if (traj_stat->command_type == TRAJ_CMD_INIT_TYPE &&
	traj_stat->echo_serial_number == traj_cmd->serial_number) {
      if (traj_stat->status == GO_RCS_STATUS_DONE ||
	  traj_stat->status == GO_RCS_STATUS_ERROR) {
	/* finished somehow, now check tool */
	go_state_next(stat, GO_RCS_STATE_S2);
      } /* else still executing */
    } else {
      resend_traj_cmd(traj_cmd);
    }
  } else if (go_state_match(stat, GO_RCS_STATE_S2)) {
    /* now check tool */
    if (tool_stat->command_type == TOOL_CMD_INIT_TYPE &&
	tool_stat->echo_serial_number == tool_cmd->serial_number) {
      if (tool_stat->status == GO_RCS_STATUS_DONE ||
	  tool_stat->status == GO_RCS_STATUS_ERROR) {
	/* finished somehow, now finish up */
	dclock = TRANSITION_TIME;
	go_state_next(stat, GO_RCS_STATE_S3);
      } /* else still executing */
    } else {
      resend_tool_cmd(tool_cmd);
    }
  } else if (go_state_match(stat, GO_RCS_STATE_S3)) {
    dclock -= set->cycle_time;
    if (dclock <= 0) {
      if (traj_stat->status == GO_RCS_STATUS_DONE &&
	  tool_stat->status == GO_RCS_STATUS_DONE) {
	stat->admin_state = GO_RCS_ADMIN_STATE_INITIALIZED;
	go_status_next(stat, GO_RCS_STATUS_DONE);
      } else {
	if (traj_stat->status == GO_RCS_STATUS_ERROR) {
	  add_task_error(stat, set, TASK_ERROR_MOTION);
	}
	if (tool_stat->status == GO_RCS_STATUS_ERROR) {
	  add_task_error(stat, set, TASK_ERROR_TOOL);
	}
	go_status_next(stat, GO_RCS_STATUS_ERROR);
      }
      stat->state_model = TASK_STATE_IDLE;
      go_state_next(stat, GO_RCS_STATE_S0);
    }
  } else {			/* S0 */
    go_state_default(stat);
  }
}

static void do_cmd_start(char *path, task_stat_struct *stat, task_set_struct *set, traj_cmd_struct *traj_cmd, traj_stat_struct *traj_stat, traj_set_struct *traj_set, tool_cmd_struct *tool_cmd, tool_stat_struct *tool_stat)
{
  static ulapi_real dclock;
  static ulapi_integer isdone;
  static ulapi_integer result;

  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    go_state_new(stat);
    CMD_PRINT_2("task: cmd start %s\n", path);
    if ((stat->state_model != TASK_STATE_IDLE && set->strict) || in_failure) {
      add_task_error(stat, set, TASK_ERROR_IMPROPER_COMMAND);
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
    } else {
      if (NULL != runproc) {
	/* stop a running process */
	ulapi_process_stop(runproc);
	ulapi_process_delete(runproc);
      }
      runproc = ulapi_process_new();
      if (NULL == runproc) {
	add_task_error(stat, set, TASK_ERROR_OUT_OF_MEMORY);
	go_status_next(stat, GO_RCS_STATUS_ERROR);
	go_state_next(stat, GO_RCS_STATE_S0);
      } else {
	if (ULAPI_OK != ulapi_process_start(runproc, path)) {
	  ulapi_process_delete(runproc);
	  runproc = NULL;
	  add_task_error(stat, set, TASK_ERROR_PROGRAM_NOT_FOUND);
	  go_status_next(stat, GO_RCS_STATUS_ERROR);
	  go_state_next(stat, GO_RCS_STATE_S0);
	} else {
	  isdone = 0;
	  ulapi_strncpy(stat->program, path, sizeof(stat->program));
	  stat->program[sizeof(stat->program)-1] = 0;
	  dclock = TRANSITION_TIME;
	  stat->state_model = TASK_STATE_STARTING;
	  go_status_next(stat, GO_RCS_STATUS_EXEC);
	  go_state_next(stat, GO_RCS_STATE_S1);
	}
      }
    }
  } else if (go_state_match(stat, GO_RCS_STATE_S1)) {
    dclock -= set->cycle_time;
    if (dclock <= 0.0) {
      stat->state_model = TASK_STATE_EXECUTE;
      go_state_next(stat, GO_RCS_STATE_S2);
    }
  } else if (go_state_match(stat, GO_RCS_STATE_S2)) {
    if (in_failure) {
      /*
	Abort if the in_failure flag was set, as if we completed the
	process with an error
      */
      if (NULL != runproc) {
	ulapi_process_stop(runproc);
	ulapi_process_delete(runproc);
	runproc = NULL;
      }
      result = 1;
      dclock = TRANSITION_TIME;
      stat->state_model = TASK_STATE_COMPLETING;
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S3);
    } else {
      /*
	Move into the task completion state if we're done and there's
	no control error. Control errors may be forced, in which case
	we don't want to prematurely report that the program is done.
      */
      if (! isdone) isdone = ulapi_process_done(runproc, &result);
      if (isdone && TASK_ERROR_CONTROL != get_task_error(stat)) {
	dclock = TRANSITION_TIME;
	stat->state_model = TASK_STATE_COMPLETING;
	go_state_next(stat, GO_RCS_STATE_S3);
      }
    }
  } else if (go_state_match(stat, GO_RCS_STATE_S3)) {
    dclock -= set->cycle_time;
    if (dclock <= 0.0) {
      stat->program[0] = 0;
      if (result == 0) {
	go_status_next(stat, GO_RCS_STATUS_DONE);
      } else {
	add_task_error(stat, set, TASK_ERROR_PROGRAM_ERROR);
	CMD_PRINT_2("task: cmd run result %d\n", (int) result);
	go_status_next(stat, GO_RCS_STATUS_ERROR);
      }
      /*
	Since the running program may have sent commands to the traj
	and tool subordinates, and changed their command serial numbers,
	we need to synchronize ours to match.
      */
      traj_cmd->serial_number = traj_stat->echo_serial_number;
      tool_cmd->serial_number = tool_stat->echo_serial_number;
      stat->state_model = TASK_STATE_COMPLETE;
      go_state_next(stat, GO_RCS_STATE_S0);
    }
  } else {			/* S0 */
    go_state_default(stat);
  }
}

static void do_cmd_execute(char *path, task_stat_struct *stat, task_set_struct *set, traj_cmd_struct *traj_cmd, traj_stat_struct *traj_stat, traj_set_struct *traj_set, tool_cmd_struct *tool_cmd, tool_stat_struct *tool_stat)
{
  static ulapi_real dclock;
  char full_program[TASK_CMD_PROGRAM_LEN];
  int retval;
  interplist_type val;
  enum {
    GO_RCS_STATE_WAITING_FOR_DELAY = GO_RCS_STATE_LAST,
    GO_RCS_STATE_WAITING_FOR_MOTION,
    GO_RCS_STATE_WAITING_FOR_TOOL,
    GO_RCS_STATE_WAITING_FOR_MOTION_QUEUE,
    GO_RCS_STATE_FAILURE,
  };

  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    go_state_new(stat);
    CMD_PRINT_2("task: cmd execute %s\n", path);
    if ((stat->state_model != TASK_STATE_IDLE && set->strict) || in_failure) {
      add_task_error(stat, set, TASK_ERROR_IMPROPER_COMMAND);
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
    } else {
      interplist_clear(&task_interplist);
      if (ulapi_ispath(path)) {
	/* it probably came from a file selector */
	ulapi_snprintf(full_program, sizeof(full_program), "%s", path);
	full_program[sizeof(full_program) - 1] = 0;
      } else {
	/* it probably came as a short name, implicitly in the prog_dir */
	ulapi_snprintf(full_program, sizeof(full_program), "%s%s%s", set->prog_dir, ulapi_pathsep, path);
	full_program[sizeof(full_program) - 1] = 0;
      }
      CMD_PRINT_2("task: cmd execute running %s\n", full_program);
      rs274ngc_c_close();
      if (RS274NGC_OK != rs274ngc_c_open(full_program)) {
	add_task_error(stat, set, TASK_ERROR_PROGRAM_NOT_FOUND);
	go_status_next(stat, GO_RCS_STATUS_ERROR);
	go_state_next(stat, GO_RCS_STATE_S0);
      } else {
	ulapi_strncpy(stat->program, path, sizeof(stat->program));
	stat->program[sizeof(stat->program)-1] = 0;
	dclock = TRANSITION_TIME;
	stat->state_model = TASK_STATE_STARTING;
	go_status_next(stat, GO_RCS_STATUS_EXEC);
	go_state_next(stat, GO_RCS_STATE_S1);
      }
    }
  } else if (go_state_match(stat, GO_RCS_STATE_S1)) {
    dclock -= set->cycle_time;
    if (dclock <= 0.0) {
      stat->state_model = TASK_STATE_EXECUTE;
      go_state_next(stat, GO_RCS_STATE_S5); /* inserting S5 here for S2 */
    }
  } else if (go_state_match(stat, GO_RCS_STATE_S5)) {
    if (in_failure) {
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_FAILURE);
    } else {
      /* check subordinates for errors, and stop everything if so */
      if (traj_stat->echo_serial_number == traj_cmd->serial_number &&
	  traj_stat->status == GO_RCS_STATUS_ERROR) {
	tool_cmd->type = TOOL_CMD_ABORT_TYPE;
	write_tool_cmd(tool_cmd);
	go_state_next(stat, GO_RCS_STATE_S6);
      } else if (tool_stat->echo_serial_number == tool_cmd->serial_number &&
		 tool_stat->status == GO_RCS_STATUS_ERROR) {
	traj_cmd->type = TRAJ_CMD_STOP_TYPE;
	write_traj_cmd(traj_cmd);
	go_state_next(stat, GO_RCS_STATE_S6);
      } else {
	/* handle the interp list */
	if ((-1 != interplist_peek(&task_interplist, &val)) &&
	    (TRAJ_CMD_MOVE_WORLD_TYPE == val.type &&
	     traj_stat->queue_count > traj_set->queue_size / 2)) {
	  go_state_next(stat, GO_RCS_STATE_WAITING_FOR_MOTION_QUEUE);
	} else {
	  if (-1 != interplist_get(&task_interplist, &val)) {
	    switch (val.type) {
	    case TASK_EXEC_DELAY_TYPE:
	      CMD_PRINT_3("task: %s %f\n", task_cmd_symbol(val.type), (double) val.u.task_cmd.u.delay.time);
	      dclock = val.u.task_cmd.u.delay.time;
	      go_state_next(stat, GO_RCS_STATE_WAITING_FOR_DELAY);
	      break;
	    case TASK_EXEC_WAIT_FOR_MOTION_TYPE:
	      CMD_PRINT_2("task: %s\n", task_cmd_symbol(val.type));
	      go_state_next(stat, GO_RCS_STATE_WAITING_FOR_MOTION);
	      break;
	    case TASK_EXEC_WAIT_FOR_TOOL_TYPE:
	      CMD_PRINT_2("task: %s\n", task_cmd_symbol(val.type));
	      go_state_next(stat, GO_RCS_STATE_WAITING_FOR_TOOL);
	      break;
	    case TRAJ_CMD_MOVE_WORLD_TYPE:
	      CMD_PRINT_3("task: %s %f ...\n", traj_cmd_symbol(val.type), (double) val.u.traj_cmd.u.move_world.end.tran.x);
	      /* leave serial number, head and tail alone, and just
		 copy over the type and data */
	      traj_cmd->type = val.type;
	      traj_cmd->u = val.u.traj_cmd.u;
	      /* and set the move accel, jerk with our settings */
	      traj_cmd->u.move_world.ta = traj_set->max_tacc;
	      traj_cmd->u.move_world.tj = traj_set->max_tjerk;
	      traj_cmd->u.move_world.ra = traj_set->max_racc;
	      traj_cmd->u.move_world.rj = traj_set->max_rjerk;
	      write_traj_cmd(traj_cmd);
	      break;
	    case TOOL_CMD_ON_TYPE:
	      CMD_PRINT_4("task: %s [%d] %f\n", tool_cmd_symbol(val.type), (int) val.u.tool_cmd.id, (double) val.u.tool_cmd.u.on.value);
	      tool_cmd->type = val.type;
	      tool_cmd->id = val.u.tool_cmd.id;
	      tool_cmd->u = val.u.tool_cmd.u;
	      write_tool_cmd(tool_cmd);
	      break;
	    case TOOL_CMD_OFF_TYPE:
	      CMD_PRINT_3("task: %s [%d]\n", tool_cmd_symbol(val.type), (int) val.u.tool_cmd.id);
	      tool_cmd->type = val.type;
	      tool_cmd->id = val.u.tool_cmd.id;
	      tool_cmd->u = val.u.tool_cmd.u;
	      write_tool_cmd(tool_cmd);
	      break;
	    default:
	      CMD_PRINT_2("task: unknown type %d\n", (int) val.type);
	    } /* switch (val.type) */
	  } else {
	    retval = rs274ngc_c_read();
	    if (RS274NGC_ENDFILE == retval ||
		RS274NGC_EXECUTE_FINISH == retval) {
	      rs274ngc_c_close();
	      go_status_next(stat, GO_RCS_STATUS_DONE);
	      go_state_next(stat, GO_RCS_STATE_S6);
	    } else if (RS274NGC_OK != retval) {
	      add_task_error(stat, set, TASK_ERROR_PROGRAM_ERROR);
	      CMD_PRINT_2("task: %s\n", rs274ngc_c_error(retval));
	      rs274ngc_c_close();
	      go_status_next(stat, GO_RCS_STATUS_ERROR);
	      go_state_next(stat, GO_RCS_STATE_S6);
	    } else {
	      retval = rs274ngc_c_execute(NULL);
	      if (RS274NGC_ENDFILE == retval ||
		  RS274NGC_EXECUTE_FINISH == retval ||
		  RS274NGC_EXIT == retval) {
		rs274ngc_c_close();
		go_status_next(stat, GO_RCS_STATUS_DONE);
		go_state_next(stat, GO_RCS_STATE_S6);
	      } else if (RS274NGC_OK != retval) {
		add_task_error(stat, set, TASK_ERROR_PROGRAM_ERROR);
		CMD_PRINT_2("task: %s\n", rs274ngc_c_error(retval));
		rs274ngc_c_close();
		go_status_next(stat, GO_RCS_STATUS_ERROR);
		go_state_next(stat, GO_RCS_STATE_S6);
	      }
	    }
	  }
	}
      }
    }
  } else if (go_state_match(stat, GO_RCS_STATE_WAITING_FOR_DELAY)) {
    if (in_failure) {
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_FAILURE);
    } else {
      dclock -= set->cycle_time;
      if (dclock <= 0.0) {
	go_state_next(stat, GO_RCS_STATE_S5);
      }
    }
  } else if (go_state_match(stat, GO_RCS_STATE_WAITING_FOR_MOTION)) {
    if (in_failure) {
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_FAILURE);
    } else {
      if (traj_stat->echo_serial_number == traj_cmd->serial_number &&
	  traj_stat->status != GO_RCS_STATUS_EXEC) {
	go_state_next(stat, GO_RCS_STATE_S5);
      }
    }
  } else if (go_state_match(stat, GO_RCS_STATE_WAITING_FOR_MOTION_QUEUE)) {
    if (in_failure) {
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_FAILURE);
    } else {
      if (traj_stat->queue_count < traj_set->queue_size / 2) {
	go_state_next(stat, GO_RCS_STATE_S5);
      }
    }
  } else if (go_state_match(stat, GO_RCS_STATE_WAITING_FOR_TOOL)) {
    if (in_failure) {
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_FAILURE);
    } else {
      if (tool_stat->echo_serial_number == tool_cmd->serial_number &&
	  tool_stat->status != GO_RCS_STATUS_EXEC) {
	go_state_next(stat, GO_RCS_STATE_S5);
      }
    }
  } else if (go_state_match(stat, GO_RCS_STATE_FAILURE)) {
    traj_cmd->type = TRAJ_CMD_STOP_TYPE;
    write_traj_cmd(traj_cmd);
    tool_cmd->type = TOOL_CMD_ABORT_TYPE;
    write_tool_cmd(tool_cmd);
    add_task_error(stat, set, TASK_ERROR_PROGRAM_ERROR);
    go_status_next(stat, GO_RCS_STATUS_ERROR);
    go_state_next(stat, GO_RCS_STATE_S2);
  } else if (go_state_match(stat, GO_RCS_STATE_S6)) {
    /* wait for traj and tool to finish */
    if (traj_stat->echo_serial_number == traj_cmd->serial_number &&
	traj_stat->status != GO_RCS_STATUS_EXEC &&
	tool_stat->echo_serial_number == tool_cmd->serial_number &&
	tool_stat->status != GO_RCS_STATUS_EXEC) {
      go_state_next(stat, GO_RCS_STATE_S2);
    }
    /* else keep waiting */
  } else if (go_state_match(stat, GO_RCS_STATE_S2)) {
    dclock = TRANSITION_TIME;
    stat->state_model = TASK_STATE_COMPLETING;
    go_state_next(stat, GO_RCS_STATE_S3);
  } else if (go_state_match(stat, GO_RCS_STATE_S3)) {
    dclock -= set->cycle_time;
    if (dclock <= 0.0) {
      stat->program[0] = 0;
      stat->state_model = TASK_STATE_COMPLETE;
      go_state_next(stat, GO_RCS_STATE_S0);
    }
  } else {			/* S0 */
    go_state_default(stat);
  }
}

/*
  If the program is an NC file, ending in .ngc or .NGC, then it's passed
  to 'do_cmd_execute'. Absolute paths are run without modification. Relative
  paths are 

  Otherwise, it's passed to 'do_cmd_start'.

  
*/

static void do_cmd_start_or_execute(task_cmd_struct *cmd, task_stat_struct *stat, task_set_struct *set, traj_cmd_struct *traj_cmd, traj_stat_struct *traj_stat, traj_set_struct *traj_set, tool_cmd_struct *tool_cmd, tool_stat_struct *tool_stat)
{
  /* switch on the suffix to either run an external program, or use
     the native RS274-NGC code interpreter */
  static enum {DO_START, DO_EXECUTE} which = DO_START;
  char path[TASK_CMD_PROGRAM_LEN];
  char suffix[TASK_CMD_PROGRAM_LEN];
  char *ptr, *sptr;

  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    which = DO_START;
    ulapi_fixpath(cmd->u.start.program, path, sizeof(path));
    ptr = strrchr(path, '.');
    if (NULL != ptr) {
      sptr = suffix;
      while (0 != *ptr) {
	*sptr++ = tolower(*ptr++);
      }
      *sptr = 0;
      if ((! strcmp(suffix, ".ngc")) ||
	  (! strcmp(suffix, ".nc"))) {
	which = DO_EXECUTE;
      }
    }
  }

  if (DO_START == which) {
    do_cmd_start(path, stat, set, traj_cmd, traj_stat, traj_set, tool_cmd, tool_stat);
  } else {
    do_cmd_execute(path, stat, set, traj_cmd, traj_stat, traj_set, tool_cmd, tool_stat);
  }
}

static void do_cmd_abort(task_stat_struct *stat, task_set_struct *set, traj_cmd_struct *traj_cmd, traj_stat_struct *traj_stat, tool_cmd_struct *tool_cmd, tool_stat_struct *tool_stat)
{
  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_1("task: cmd abort\n");
    go_state_new(stat);
    if (NULL != runproc) {
      ulapi_process_stop(runproc);
      ulapi_process_delete(runproc);
      runproc = NULL;
    }
    traj_cmd->type = TRAJ_CMD_ABORT_TYPE;
    write_traj_cmd(traj_cmd);
    tool_cmd->type = TOOL_CMD_ABORT_TYPE;
    write_tool_cmd(tool_cmd);
    stat->state_model = TASK_STATE_ABORTING;
    go_status_next(stat, GO_RCS_STATUS_EXEC);
    go_state_next(stat, GO_RCS_STATE_S1);
  } else if (go_state_match(stat, GO_RCS_STATE_S1)) {
    /* first check traj */
    if (traj_stat->command_type == TRAJ_CMD_ABORT_TYPE &&
	traj_stat->echo_serial_number == traj_cmd->serial_number) {
      if (traj_stat->status == GO_RCS_STATUS_DONE || 
	  traj_stat->status == GO_RCS_STATUS_ERROR) {
	/* traj is done, now check tool */
	go_state_next(stat, GO_RCS_STATE_S2);
      } /* else still executing */
    } else {
      resend_traj_cmd(traj_cmd);
    }
  } else if (go_state_match(stat, GO_RCS_STATE_S2)) {
    /* now check tool */
    if (tool_stat->command_type == TOOL_CMD_ABORT_TYPE &&
	tool_stat->echo_serial_number == tool_cmd->serial_number) {
      if (tool_stat->status == GO_RCS_STATUS_DONE || 
	  tool_stat->status == GO_RCS_STATUS_ERROR) {
	/* tool is done */
	/* this state doesn't require a transition time, so don't
	   set the dclock */
	go_state_next(stat, GO_RCS_STATE_S3);
      } /* else still executing */
    } else {
      resend_tool_cmd(tool_cmd);
    }
  } else if (go_state_match(stat, GO_RCS_STATE_S3)) {
    if (traj_stat->status == GO_RCS_STATUS_DONE &&
	tool_stat->status == GO_RCS_STATUS_DONE) {
      go_status_next(stat, GO_RCS_STATUS_DONE);
    } else {
      if (traj_stat->status == GO_RCS_STATUS_ERROR) {
	add_task_error(stat, set, TASK_ERROR_MOTION);
      }
      if (tool_stat->status == GO_RCS_STATUS_ERROR) {
	add_task_error(stat, set, TASK_ERROR_TOOL);
      }
      go_status_next(stat, GO_RCS_STATUS_ERROR);
    }
    /* make our state 'aborted' regardless of task and tool */
    stat->state_model = TASK_STATE_ABORTED;
    go_state_next(stat, GO_RCS_STATE_S0);
  } else {			/* S0 */
    go_state_default(stat);
  }
}

static void do_cmd_stop(task_stat_struct *stat, task_set_struct *set, traj_cmd_struct *traj_cmd, traj_stat_struct *traj_stat, tool_cmd_struct *tool_cmd, tool_stat_struct *tool_stat)
{
  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_1("task: cmd stop\n");
    go_state_new(stat);
    if (NULL != runproc) {
      ulapi_process_stop(runproc);
      ulapi_process_delete(runproc);
      runproc = NULL;
    }
    traj_cmd->type = TRAJ_CMD_STOP_TYPE;
    write_traj_cmd(traj_cmd);
    tool_cmd->type = TOOL_CMD_OFF_TYPE;
    write_tool_cmd(tool_cmd);
    stat->state_model = TASK_STATE_STOPPING;
    go_status_next(stat, GO_RCS_STATUS_EXEC);
    go_state_next(stat, GO_RCS_STATE_S1);
  } else if (go_state_match(stat, GO_RCS_STATE_S1)) {
    if (traj_stat->command_type == TRAJ_CMD_STOP_TYPE &&
	traj_stat->echo_serial_number == traj_cmd->serial_number) {
      if (traj_stat->status == GO_RCS_STATUS_DONE || 
	  traj_stat->status == GO_RCS_STATUS_ERROR) {
	go_state_next(stat, GO_RCS_STATE_S2);
      }
    } else {
      resend_traj_cmd(traj_cmd);
    }
  } else if (go_state_match(stat, GO_RCS_STATE_S2)) {
    if (tool_stat->command_type == TOOL_CMD_OFF_TYPE &&
	tool_stat->echo_serial_number == tool_cmd->serial_number) {
      if (tool_stat->status == GO_RCS_STATUS_DONE || 
	  tool_stat->status == GO_RCS_STATUS_ERROR) {
	go_state_next(stat, GO_RCS_STATE_S3);
      }
    } else {
      resend_tool_cmd(tool_cmd);
    }
  } else if (go_state_match(stat, GO_RCS_STATE_S3)) {
    if (traj_stat->status == GO_RCS_STATUS_DONE &&
	tool_stat->status == GO_RCS_STATUS_DONE) {
      go_status_next(stat, GO_RCS_STATUS_DONE);
    } else {
      if (traj_stat->status == GO_RCS_STATUS_ERROR) {
	add_task_error(stat, set, TASK_ERROR_MOTION);
      }
      if (tool_stat->status == GO_RCS_STATUS_ERROR) {
	add_task_error(stat, set, TASK_ERROR_TOOL);
      }
      go_status_next(stat, GO_RCS_STATUS_ERROR);
    }
    stat->state_model = TASK_STATE_STOPPED;
    go_state_next(stat, GO_RCS_STATE_S0);
  } else {			/* S0 */
    go_state_default(stat);
  }
}

static void do_cmd_shutdown(task_stat_struct *stat, task_set_struct *set, traj_cmd_struct *traj_cmd, traj_stat_struct *traj_stat, tool_cmd_struct *tool_cmd, tool_stat_struct *tool_stat)
{
  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_1("task: cmd shutdown\n");
    go_state_new(stat);
    traj_cmd->type = TRAJ_CMD_SHUTDOWN_TYPE;
    write_traj_cmd(traj_cmd);
    tool_cmd->type = TOOL_CMD_SHUTDOWN_TYPE;
    write_tool_cmd(tool_cmd);
    go_status_next(stat, GO_RCS_STATUS_EXEC);
    go_state_next(stat, GO_RCS_STATE_S1);
  } else if (go_state_match(stat, GO_RCS_STATE_S1)) {
    if (traj_stat->command_type == TRAJ_CMD_SHUTDOWN_TYPE &&
	traj_stat->echo_serial_number == traj_cmd->serial_number) {
      if (traj_stat->status == GO_RCS_STATUS_DONE || 
	  traj_stat->status == GO_RCS_STATUS_ERROR) {
	go_state_next(stat, GO_RCS_STATE_S2);
      }
    } else {
      resend_traj_cmd(traj_cmd);
    }
  } else if (go_state_match(stat, GO_RCS_STATE_S2)) {
    if (tool_stat->command_type == TOOL_CMD_SHUTDOWN_TYPE &&
	tool_stat->echo_serial_number == tool_cmd->serial_number) {
      if (tool_stat->status == GO_RCS_STATUS_DONE || 
	  tool_stat->status == GO_RCS_STATUS_ERROR) {
	go_state_next(stat, GO_RCS_STATE_S3);
      }
    } else {
      resend_tool_cmd(tool_cmd);
    }
  } else if (go_state_match(stat, GO_RCS_STATE_S3)) {
    if (traj_stat->status == GO_RCS_STATUS_DONE &&
	tool_stat->status == GO_RCS_STATUS_DONE) {
      go_status_next(stat, GO_RCS_STATUS_DONE);
    } else {
      if (traj_stat->status == GO_RCS_STATUS_ERROR) {
	add_task_error(stat, set, TASK_ERROR_MOTION);
      }
      if (tool_stat->status == GO_RCS_STATUS_ERROR) {
	add_task_error(stat, set, TASK_ERROR_TOOL);
      }
      go_status_next(stat, GO_RCS_STATUS_ERROR);
    }
    stat->state_model = TASK_STATE_STOPPED; /* same as our initial state */
    go_state_next(stat, GO_RCS_STATE_S0);
  } else {			/* S0 */
    go_state_default(stat);
    do_exit = 1;
  }
}

static void do_cfg_nop(task_set_struct *set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    CFG_PRINT_1("task: cfg nop\n");
    go_state_new(set);
    go_status_next(set, GO_RCS_STATUS_DONE);
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_cycle_time(task_cfg_struct *cfg, task_set_struct *set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    CFG_PRINT_2("task: cfg cycle time %f\n", (double) cfg->u.cycle_time.cycle_time);
    go_state_new(set);
    if (cfg->u.cycle_time.cycle_time > 0) {
      set->cycle_time = cfg->u.cycle_time.cycle_time;
      go_status_next(set, GO_RCS_STATUS_DONE);
    } else {
      go_status_next(set, GO_RCS_STATUS_ERROR);
    }
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_debug(task_cfg_struct *cfg, task_set_struct *set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    CFG_PRINT_2("task: cfg debug %x\n", (int) cfg->u.debug.debug);
    go_state_new(set);
    set->debug = cfg->u.debug.debug;
    go_status_next(set, GO_RCS_STATUS_DONE);
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_strict(task_cfg_struct *cfg, task_set_struct *set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    CFG_PRINT_2("task: cfg strict %d\n", (int) cfg->u.strict.strict);
    go_state_new(set);
    set->strict = cfg->u.strict.strict;
    go_status_next(set, GO_RCS_STATUS_DONE);
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_prog_dir(task_cfg_struct *cfg, task_set_struct *set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    CFG_PRINT_2("task: cfg prog dir '%s'\n", cfg->u.prog_dir.prog_dir);
    go_state_new(set);
    strcpy(set->prog_dir, cfg->u.prog_dir.prog_dir);
    go_status_next(set, GO_RCS_STATUS_DONE);
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

#define TASK_PRINT_1(x) if (task_set.debug & DEBUG_TASK) ulapi_print(x)
#define TASK_PRINT_2(x,y) if (task_set.debug & DEBUG_TASK) ulapi_print(x,y)

static int ini_load(char *inifile_name,
		    int *task_shm_key,
		    double *task_cycle_time,
		    int *task_debug,
		    int *task_strict,
		    char *prog_dir, size_t prog_dir_len,
		    char *parameter_file_name, size_t parameter_file_name_len,
		    char *tool_file_name, size_t tool_file_name_len,
		    double *mttf,
		    double *mttr,
		    int *traj_shm_key,
		    int *tool_shm_key)
{
  FILE *fp;
  const char *section;
  const char *key;
  const char *inistring;
  double d1;

  if (NULL == (fp = fopen(inifile_name, "r"))) {
    fprintf(stderr, "task: can't open %s\n", inifile_name);
    return 1;
  }

#define CLOSE_AND_RETURN \
  fclose(fp);		 \
  return 1

  section = "GOMOTION";

  key = "LENGTH_UNITS_PER_M";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    fprintf(stderr, "task: missing entry: [%s] %s\n", section, key);
    CLOSE_AND_RETURN;
  } else if (1 != sscanf(inistring, "%lf", &d1) || d1 <= 0.0) {
    fprintf(stderr, "task: bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN;
  }
  length_units_per_m = d1;
  m_per_length_units = 1.0 / length_units_per_m;

  key = "ANGLE_UNITS_PER_RAD";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    fprintf(stderr, "task: missing entry: [%s] %s\n", section, key);
    CLOSE_AND_RETURN;
  } else if (1 != sscanf(inistring, "%lf", &d1) || d1 <= 0.0) {
    fprintf(stderr, "task: bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN;
  }
  angle_units_per_rad = d1;
  rad_per_angle_units = 1.0 / angle_units_per_rad;

  section = "TASK";

  key = "SHM_KEY";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    fprintf(stderr, "task: missing entry: [%s] %s\n", section, key);
    CLOSE_AND_RETURN;
  } else if (1 != sscanf(inistring, "%i", task_shm_key)) {
    fprintf(stderr, "task: bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN;
  }

  key = "CYCLE_TIME";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    fprintf(stderr, "task: missing entry: [%s] %s\n", section, key);
    CLOSE_AND_RETURN;
  } else if (1 != sscanf(inistring, "%lf", task_cycle_time)) {
    fprintf(stderr, "task: bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN;
  }

  key = "DEBUG";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    fprintf(stderr, "task: missing entry: [%s] %s\n", section, key);
    CLOSE_AND_RETURN;
  } else if (1 != sscanf(inistring, "%i", task_debug)) {
    fprintf(stderr, "task: bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN;
  }

  key = "STRICT";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    /* optional, set to zero */
    *task_strict = 0;
  } else if (1 != sscanf(inistring, "%i", task_strict)) {
    fprintf(stderr, "task: bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN;
  }

  key = "PROG_DIR";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    /* optional, leave as default */
  } else {
    strncpy(prog_dir, inistring, prog_dir_len);
    prog_dir[prog_dir_len - 1] = 0;
  }

  key = "PARAMETER_FILE_NAME";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    /* optional, leave as default */
  } else {
    strncpy(parameter_file_name, inistring, parameter_file_name_len);
    parameter_file_name[parameter_file_name_len - 1] = 0;
  }

  key = "TOOL_FILE_NAME";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    /* optional, leave as default */
  } else {
    strncpy(tool_file_name, inistring, tool_file_name_len);
    tool_file_name[tool_file_name_len - 1] = 0;
  }

  key = "MTTF";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    /* optional, leave as default */
  } else if (1 != sscanf(inistring, "%lf", mttf)) {
    fprintf(stderr, "task: bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN;
  }

  key = "MTTR";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    /* also optional */
  } else if (1 != sscanf(inistring, "%lf", mttr)) {
    fprintf(stderr, "task: bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN;
  }

  section = "TRAJ";

  key = "SHM_KEY";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    fprintf(stderr, "task: missing entry: [%s] %s\n", section, key);
    CLOSE_AND_RETURN;
  } else if (1 != sscanf(inistring, "%i", traj_shm_key)) {
    fprintf(stderr, "task: bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN;
  }

  section = "TOOL";

  key = "SHM_KEY";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    fprintf(stderr, "task: missing entry: [%s] %s\n", section, key);
    CLOSE_AND_RETURN;
  } else if (1 != sscanf(inistring, "%i", tool_shm_key)) {
    fprintf(stderr, "task: bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN;
  }

  fclose(fp);
  return 0;
}

static void print_help(void)
{
  printf("-i <file> : use initialization file <file>, default %s\n", DEFAULT_INI_FILE);
  printf("-s        : follow strict state transitions\n");
  printf("-d        : turn debug on\n");
  printf("-?        : print this help message\n");
}

int main(int argc, char *argv[])
{
  int option;

  int task_shm_key;
  double task_cycle_time = 1;
  int task_debug = 0;
  int task_strict = 0;
  int traj_shm_key;
  int tool_shm_key;

  void *task_shm;
  void *traj_shm;
  void *tool_shm;
  task_comm_struct *task_comm_ptr;
  task_stat_struct task_stat;
  task_set_struct task_set;
  task_cmd_struct pp_task_cmd[2], *task_cmd_ptr, *task_cmd_test;
  task_cfg_struct pp_task_cfg_struct[2], *task_cfg_ptr, *task_cfg_test;

  /* traj_comm_ptr and tool_comm_ptr are global */
  traj_cmd_struct traj_cmd;
  traj_cfg_struct traj_cfg;
  traj_stat_struct pp_traj_stat[2], *traj_stat_ptr, *traj_stat_test;
  traj_set_struct pp_traj_set[2], *traj_set_ptr, *traj_set_test;
  tool_cmd_struct tool_cmd;
  tool_cfg_struct tool_cfg;
  tool_stat_struct pp_tool_stat[2], *tool_stat_ptr, *tool_stat_test;
  tool_set_struct pp_tool_set[2], *tool_set_ptr, *tool_set_test;

  void *tmp;
  go_integer cmd_type, cfg_type;
  go_integer cmd_serial_number, cfg_serial_number;

  exponential_random_struct mttf_rand;
  exponential_random_struct mttr_rand;
  double mttf = -1;
  double mttr = -1;
  double next_time;
  int do_failures = 0;

  ulapi_real end;
  ulapi_real start_time;

  int debug_arg;
  int strict_arg;
  int start_it;
  int got_it;
  int heartbeat;
  int t;
  int retval;

  debug_arg = 0;
  strict_arg = 0;
  opterr = 0;
  while (1) {
    option = ulapi_getopt(argc, argv, ":i:u:sd?");
    if (option == -1)
      break;

    switch (option) {
    case 'i':
      strncpy(inifile_name, ulapi_optarg, BUFFERLEN);
      inifile_name[BUFFERLEN - 1] = 0;
      break;

    case 's':
      strict_arg = 1;
      break;

    case 'd':
      debug_arg = 0xFFFFFFFF;
      break;

    case '?':
      print_help();
      return 0;
      break;

    case ':':
      fprintf(stderr, "task: missing value for -%c\n", ulapi_optopt);
      return 1;
      break;

    default:			/* '?' */
      fprintf (stderr, "task: unrecognized option -%c\n", ulapi_optopt);
      return 1;
      break;
    }
  }
  if (ulapi_optind < argc) {
    fprintf(stderr, "task: extra non-option characters: %s\n", argv[ulapi_optind]);
    return 1;
  }

  if (ULAPI_OK != ulapi_init()) {
    return 1;
  } 

  if (0 != ini_load(inifile_name, &task_shm_key, &task_cycle_time, &task_debug, &task_strict, prog_dir, sizeof(prog_dir), parameter_file_name, sizeof(parameter_file_name), tool_file_name, sizeof(tool_file_name), &mttf, &mttr, &traj_shm_key, &tool_shm_key)) {
    return 1;
  }

  if (GO_RESULT_OK != go_init()) {
    fprintf(stderr, "task: can't init go motion\n");
    exit(0);
  }

  /* get my task shared memory buffers */
  task_shm = ulapi_rtm_new(task_shm_key, sizeof(task_comm_struct));
  if (NULL == task_shm) {
    fprintf(stderr, "task: can't get task comm shm\n");
    exit(0);
  }
  task_comm_ptr = ulapi_rtm_addr(task_shm);
  /* set up my ping-pong buffers for things I read */
  task_cmd_ptr = &pp_task_cmd[0];
  task_cmd_test = &pp_task_cmd[1];
  task_cmd_ptr->head = task_cmd_ptr->tail = 0;
  task_cmd_ptr->type = TASK_CMD_NOP_TYPE;
  task_cmd_ptr->serial_number = 0;
  task_comm_ptr->task_cmd = *task_cmd_ptr; /* force a write into ourself */
  /*  */
  task_cfg_ptr = &pp_task_cfg_struct[0];
  task_cfg_test = &pp_task_cfg_struct[1];
  task_cfg_ptr->head = task_cfg_ptr->tail = 0;
  task_cfg_ptr->type = TASK_CFG_NOP_TYPE;
  task_cfg_ptr->serial_number = 0;
  task_comm_ptr->task_cfg = *task_cfg_ptr; /* as above */
  /*  */

  /* get the traj shared memory buffers */

  traj_shm = ulapi_rtm_new(traj_shm_key, sizeof(traj_comm_struct));
  if (NULL == traj_shm) {
    fprintf(stderr, "task: can't get traj comm shm\n");
    exit(0);
  }
  traj_comm_ptr = ulapi_rtm_addr(traj_shm);
  /* set up ping-pong buffers for reading traj */
  traj_stat_ptr = &pp_traj_stat[0];
  traj_stat_test = &pp_traj_stat[1];
  /*  */
  traj_set_ptr = &pp_traj_set[0];
  traj_set_test = &pp_traj_set[1];
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
    fprintf(stderr, "task: timed out connecting to traj status\n");
    exit(0);
  }

  /* set the head and tail to be 0, so the first write will increment
     them to the conventional 1 */
  traj_cmd.head = traj_cmd.tail = 0;
  traj_cmd.serial_number = traj_stat_ptr->echo_serial_number;
  /*  */
  traj_stat_ptr = &pp_traj_stat[0];
  traj_stat_test = &pp_traj_stat[1];
  *traj_set_ptr = traj_comm_ptr->traj_set;
  /*  */
  traj_cfg.head = traj_cfg.tail = 0;
  traj_cfg.serial_number = traj_set_ptr->echo_serial_number;
  /*  */
  traj_set_ptr = &pp_traj_set[0];
  traj_set_test = &pp_traj_set[1];

  /* ditto for the tool communication buffers */

  tool_shm = ulapi_rtm_new(tool_shm_key, sizeof(tool_comm_struct));
  if (NULL == tool_shm) {
    fprintf(stderr, "task: can't get tool comm shm\n");
    exit(0);
  }
  tool_comm_ptr = ulapi_rtm_addr(tool_shm);
  /* set up ping-pong buffers for reading tool */
  tool_stat_ptr = &pp_tool_stat[0];
  tool_stat_test = &pp_tool_stat[1];
  /*  */
  tool_set_ptr = &pp_tool_set[0];
  tool_set_test = &pp_tool_set[1];
  /* check for running tool */
  for (start_it = 0, got_it = 0, end = ulapi_time() + CONNECT_WAIT_TIME;
       ulapi_time() < end;
       ulapi_sleep(0.1)) {
    *tool_stat_ptr = tool_comm_ptr->tool_stat;
    if (tool_stat_ptr->head == tool_stat_ptr->tail &&
	tool_stat_ptr->type == TOOL_STAT_TYPE) {
      if (! start_it) {
	start_it = 1;
	heartbeat = tool_stat_ptr->heartbeat;
      }
      if (heartbeat != tool_stat_ptr->heartbeat) {
	got_it = 1;
	break;
      }
    }
  }
  if (! got_it) {
    fprintf(stderr, "task: timed out connecting to tool status\n");
    exit(0);
  }

  /* set the head and tail to be 0, so the first write will increment
     them to the conventional 1 */
  tool_cmd.head = tool_cmd.tail = 0;
  tool_cmd.serial_number = tool_stat_ptr->echo_serial_number;
  /*  */
  tool_stat_ptr = &pp_tool_stat[0];
  tool_stat_test = &pp_tool_stat[1];
  /*  */
  tool_cfg.head = tool_cfg.tail = 0;
  tool_cfg.serial_number = tool_set_ptr->echo_serial_number;
  /*  */
  tool_set_ptr = &pp_tool_set[0];
  tool_set_test = &pp_tool_set[1];
  *tool_set_ptr = tool_comm_ptr->tool_set;

  task_stat.head = 0;
  task_stat.type = TASK_STAT_TYPE;
  task_stat.admin_state = GO_RCS_ADMIN_STATE_UNINITIALIZED;
  task_stat.echo_serial_number = task_cmd_ptr->serial_number - 1;
  task_stat.heartbeat = 0;
  task_stat.cycle_time = task_cycle_time;
  task_stat.program[0] = 0;
  task_stat.state_model = TASK_STATE_STOPPED;
  for (t = 0; t < TASK_ERROR_MAX; t++) {
    task_stat.error[t].timestamp = ulapi_time();
    task_stat.error[t].code = TASK_ERROR_NONE;
  }
  task_stat.error_index = 0;
  task_stat.tail = task_stat.head;

  task_set.head = 0;
  task_set.type = TASK_SET_TYPE;
  task_set.echo_serial_number = task_cfg_ptr->serial_number - 1;
  task_set.cycle_time = DEFAULT_CYCLE_TIME;
  task_set.debug = task_debug | debug_arg;
  task_set.strict = (task_strict | strict_arg) ? 0 : 1;
  SAFECPY(task_set.prog_dir, DEFAULT_PROG_DIR);
  task_set.tail = task_set.head;

  /* initialize the NC code interpreter */
  interplist_init(&task_interplist);
  if (0 != *parameter_file_name) {
    retval = rs274ngc_c_restore_parameters(parameter_file_name);
    if (RS274NGC_OK != retval ) {
      TASK_PRINT_2("task: can't restore parameter file '%s'\n", parameter_file_name);
    }
  }
  if (0 != *tool_file_name) {
    retval = rs274ngc_c_restore_tool_table(tool_file_name);
    if (RS274NGC_OK != retval ) {
      TASK_PRINT_2("task: can't load tool table file '%s'\n", tool_file_name);
    }
  }
  retval = rs274ngc_c_init();
  if (RS274NGC_OK != retval ) {
    TASK_PRINT_1("task: can't initialize NC code interpreter\n");
  }

  /* set up failure and repair statistical distributions */
  if (mttf > 0 && mttr > 0) {
    do_failures = 1;
    in_failure = 0;
    /* inject offsets between the distros, based on the shared mem key */
    exponential_random_init(&mttf_rand, mttf);
    exponential_random_seed(&mttf_rand, get_random_seed(task_shm_key));
    exponential_random_init(&mttr_rand, mttr);
    exponential_random_seed(&mttr_rand, get_random_seed(task_shm_key + get_random_bins() / 2));
    next_time = ulapi_time() + exponential_random_real(&mttf_rand);
  }

  TASK_PRINT_1("task: started task loop\n");

  signal(SIGINT, quit);

  while (! do_exit) {
    start_time = ulapi_time();

    /* read in command buffer, ping-pong style */
    *task_cmd_test = task_comm_ptr->task_cmd;
    if (task_cmd_test->head == task_cmd_test->tail) {
      tmp = task_cmd_ptr;
      task_cmd_ptr = task_cmd_test;
      task_cmd_test = tmp;
    }
    cmd_type = task_cmd_ptr->type;
    cmd_serial_number = task_cmd_ptr->serial_number;

    /* read in traj stat,set, ping-pong style */
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

    /* ditto for tool */
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

    switch (cmd_type) {
    case 0:
    case -1:
      break;

    case TASK_CMD_NOP_TYPE:
    case TASK_CMD_STOP_TYPE:
    case TASK_CMD_START_TYPE:
    case TASK_CMD_HOLD_TYPE:
    case TASK_CMD_UNHOLD_TYPE:
    case TASK_CMD_SUSPEND_TYPE:
    case TASK_CMD_UNSUSPEND_TYPE:
    case TASK_CMD_RESET_TYPE:
    case TASK_CMD_ABORT_TYPE:
    case TASK_CMD_CLEAR_TYPE:
    case TASK_CMD_SHUTDOWN_TYPE:
      task_stat.command_type = cmd_type;
      if (cmd_serial_number != task_stat.echo_serial_number) {
	task_stat.echo_serial_number = cmd_serial_number;
	task_stat.state = GO_RCS_STATE_NEW_COMMAND;
      }
      break;

    default:
      TASK_PRINT_2("task: unknown command %d\n", cmd_type);
      break;
    }

    /* read in config buffer, ping-pong style */
    *task_cfg_test = task_comm_ptr->task_cfg;
    if (task_cfg_test->head == task_cfg_test->tail) {
      tmp = task_cfg_ptr;
      task_cfg_ptr = task_cfg_test;
      task_cfg_test = tmp;
    }
    cfg_type = task_cfg_ptr->type;
    cfg_serial_number = task_cfg_ptr->serial_number;

    switch (cfg_type) {
    case 0:
    case -1:
      break;

    case TASK_CFG_NOP_TYPE:
    case TASK_CFG_CYCLE_TIME_TYPE:
    case TASK_CFG_DEBUG_TYPE:
    case TASK_CFG_STRICT_TYPE:
    case TASK_CFG_PROG_DIR_TYPE:
      task_set.command_type = cfg_type;
      if (cfg_serial_number != task_set.echo_serial_number) {
	task_set.echo_serial_number = cfg_serial_number;
	task_set.state = GO_RCS_STATE_NEW_COMMAND;
      }
      break;

    default:
      fprintf(stderr, "task: unknown config %d\n", cfg_type);
      break;
    }

    switch (task_stat.command_type) {

    case TASK_CMD_NOP_TYPE:
      do_cmd_nop(&task_stat, &task_set);
      break;
    case TASK_CMD_STOP_TYPE:
      do_cmd_stop(&task_stat, &task_set, &traj_cmd, traj_stat_ptr, &tool_cmd, tool_stat_ptr);
      break;
    case TASK_CMD_START_TYPE:
      do_cmd_start_or_execute(task_cmd_ptr, &task_stat, &task_set, &traj_cmd, traj_stat_ptr, traj_set_ptr, &tool_cmd, tool_stat_ptr);
      break;
    case TASK_CMD_HOLD_TYPE:
      do_cmd_hold(&task_stat, &task_set, &traj_cfg, traj_set_ptr);
      break;
    case TASK_CMD_UNHOLD_TYPE:
      do_cmd_unhold(&task_stat, &task_set, &traj_cfg, traj_set_ptr);
      break;
    case TASK_CMD_SUSPEND_TYPE:
      do_cmd_suspend(&task_stat, &task_set, &traj_cfg, traj_set_ptr);
      break;
    case TASK_CMD_UNSUSPEND_TYPE:
      do_cmd_unsuspend(&task_stat, &task_set, &traj_cfg, traj_set_ptr);
      break;
    case TASK_CMD_RESET_TYPE:
      do_cmd_reset(&task_stat, &task_set, &traj_cmd, traj_stat_ptr, &tool_cmd, tool_stat_ptr);
      break;
    case TASK_CMD_ABORT_TYPE:
      do_cmd_abort(&task_stat, &task_set, &traj_cmd, traj_stat_ptr, &tool_cmd, tool_stat_ptr);
      break;
    case TASK_CMD_CLEAR_TYPE:
      do_cmd_clear(&task_stat, &task_set);
      break;
    case TASK_CMD_SHUTDOWN_TYPE:
      do_cmd_shutdown(&task_stat, &task_set, &traj_cmd, traj_stat_ptr, &tool_cmd, tool_stat_ptr);
      break;
    default:
      break;
    }

    switch (task_set.command_type) {
    case TASK_CFG_NOP_TYPE:
      do_cfg_nop(&task_set);
      break;

    case TASK_CFG_CYCLE_TIME_TYPE:
      do_cfg_cycle_time(task_cfg_ptr, &task_set);
      break;

    case TASK_CFG_DEBUG_TYPE:
      do_cfg_debug(task_cfg_ptr, &task_set);
      break;

    case TASK_CFG_STRICT_TYPE:
      do_cfg_strict(task_cfg_ptr, &task_set);
      break;

    case TASK_CFG_PROG_DIR_TYPE:
      do_cfg_prog_dir(task_cfg_ptr, &task_set);
      break;

    default:
      break;
    }

    /* update status */
    task_stat.heartbeat++;
    switch (task_stat.state_model) {
    case TASK_STATE_ABORTING:
    case TASK_STATE_ABORTED:
    case TASK_STATE_CLEARING:
    case TASK_STATE_STOPPING:
    case TASK_STATE_STOPPED:
      task_stat.admin_state = GO_RCS_ADMIN_STATE_UNINITIALIZED;
      break;
    default:
      task_stat.admin_state = GO_RCS_ADMIN_STATE_INITIALIZED;
      break;
    }

    /* run failure statistics */
    if (do_failures) {
      if (start_time >= next_time) {
	if (in_failure) {
	  in_failure = 0;
	  next_time += exponential_random_real(&mttf_rand);
	  add_task_error(&task_stat, &task_set, TASK_ERROR_NONE);
	} else {
	  in_failure = 1;
	  next_time += exponential_random_real(&mttr_rand);
	  add_task_error(&task_stat, &task_set, TASK_ERROR_CONTROL);
	}
      }
    }

    /* FIXME -- affect the state model, and pause execution */

    /* writing of traj cmd, cfg is done in state tables */

    /* write out task status and settings */
    task_stat.tail = ++task_stat.head;
    task_comm_ptr->task_stat = task_stat;
    /*  */
    task_set.tail = ++task_set.head;
    task_comm_ptr->task_set = task_set;

    ulapi_sleep(task_set.cycle_time);
    task_stat.cycle_time = ulapi_time() - start_time;
  } /* while (1) */

  TASK_PRINT_1("task: done\n");

  exit(0);
}
