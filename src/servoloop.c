/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#include <stddef.h>		/* NULL */
#include <rtapi.h>
#include "go.h"			/* go_interp_, go_timestamp */
#include "gorcs.h"
#include "golog.h"		/* go_log_entry,add, ... */
#include "goio.h"		/* go_io_struct */
#include "servointf.h"
#include "extintf.h"
#include "pid.h"

#define BN "servoloop"		/* the base name of this executable */

#define DEFAULT_CYCLE_TIME 0.01	/* must be non-zero */
#define DEFAULT_CYCLE_MULT 10	/* must be non-zero */

servo_comm_struct * global_servo_comm_ptr = NULL;

void * servo_sem = NULL;

static go_real servo_timestamp(void)
{
  rtapi_integer secs, nsecs;

  if (RTAPI_OK == rtapi_clock_get_time(&secs, &nsecs)) {
    return ((go_real) secs) + ((go_real) nsecs) * 1.0e-9;
  }

  return 0.0;
}

#define CMD_PRINT_2(x,y) if (set->debug & DEBUG_CMD) rtapi_print(x, y)
#define CMD_PRINT_3(x,y,z) if (set->debug & DEBUG_CMD) rtapi_print(x, y, z)
#define CFG_PRINT_2(x,y) if (set->debug & DEBUG_CFG) rtapi_print(x, y)
#define CFG_PRINT_3(x,y,z) if (set->debug & DEBUG_CFG) rtapi_print(x, y, z)
#define CFG_PRINT_4(x,y,z,u) if (set->debug & DEBUG_CFG) rtapi_print(x, y, z, u)
#define CFG_PRINT_5(x,y,z,u,v) if (set->debug & DEBUG_CFG) rtapi_print(x, y, z, u,v)
#define PERF_PRINT_3(x,y,z) if (set->debug & DEBUG_PERF) rtapi_print(x, y, z)
#define HOME_PRINT_2(x,y) if (set->debug & DEBUG_HOME) rtapi_print(x, y)

static void do_cmd_nop(servo_stat_struct * stat, servo_set_struct * set)
{
  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_2("servo %d cmd nop\n", (int) set->id);
    go_state_new(stat);
    go_status_next(stat, GO_RCS_STATUS_DONE);
    go_state_next(stat, GO_RCS_STATE_S0);
  } else {			/* GO_RCS_STATE_S0 */
    go_state_default(stat);
  }
}

static void do_cmd_init(servo_stat_struct * stat, servo_set_struct * set)
{
  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_2("servo %d cmd init\n", (int) set->id);
    go_state_new(stat);
    stat->admin_state = GO_RCS_ADMIN_STATE_INITIALIZED;
    stat->enable = 0;
    go_status_next(stat, GO_RCS_STATUS_DONE);
    go_state_next(stat, GO_RCS_STATE_S0);
  } else {			/* GO_RCS_STATE_S0 */
    go_state_default(stat);
  }
}

static void do_cmd_abort(servo_stat_struct * stat, servo_set_struct * set)
{
  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_2("servo %d cmd abort\n", (int) set->id);
    go_state_new(stat);
    stat->admin_state = GO_RCS_ADMIN_STATE_UNINITIALIZED;
    stat->enable = 0;
    go_status_next(stat, GO_RCS_STATUS_DONE);
    go_state_next(stat, GO_RCS_STATE_S0);
  } else {			/* GO_RCS_STATE_S0 */
    go_state_default(stat);
  }
}

static void do_cmd_halt(servo_stat_struct * stat, servo_set_struct * set)
{
  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_2("servo %d cmd halt\n", (int) set->id);
    go_state_new(stat);
    stat->admin_state = GO_RCS_ADMIN_STATE_UNINITIALIZED;
    stat->enable = 0;
    go_status_next(stat, GO_RCS_STATUS_DONE);
    go_state_next(stat, GO_RCS_STATE_S0);
  } else {			/* GO_RCS_STATE_S0 */
    go_state_default(stat);
  }
}

static void do_cmd_shutdown(servo_stat_struct * stat, servo_set_struct * set)
{
  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_2("servo %d cmd shutdown\n", (int) set->id);
    go_state_new(stat);
    if (stat->admin_state == GO_RCS_ADMIN_STATE_UNINITIALIZED) {
      stat->admin_state = GO_RCS_ADMIN_STATE_SHUT_DOWN;
      stat->enable = 0;
      go_status_next(stat, GO_RCS_STATUS_DONE);
    } else {
      go_status_next(stat, GO_RCS_STATUS_ERROR);
    }
    go_state_next(stat, GO_RCS_STATE_S0);
  } else {			/* GO_RCS_STATE_S0 */
    go_state_default(stat);
  }
}

#define USE_LINEAR

#if defined(USE_LINEAR)
static go_interp_add_func go_interp_add = go_interp_add_linear;
static go_interp_eval_func go_interp_eval = go_interp_eval_linear;

#elif defined(USE_CUBIC)
static go_interp_add_func go_interp_add = go_interp_add_cubic_pdv;
static go_interp_eval_func go_interp_eval = go_interp_eval_cubic;

#else
static go_interp_add_func go_interp_add = go_interp_add_quintic_pdva;
static go_interp_eval_func go_interp_eval = go_interp_eval_quintic;
#endif

void do_cmd_servo(servo_cmd_struct * cmd, servo_stat_struct * stat, servo_set_struct * set, go_interp * interp, go_real * interp_s)
{
  go_log_entry entry;

  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_3("servo %d cmd servo %f\n", 
		(int) set->id, (double) cmd->u.servo.setpoint);
    go_state_new(stat);
    if (stat->admin_state == GO_RCS_ADMIN_STATE_INITIALIZED) {
      if (set->active) {
	stat->enable = 1;
	stat->setpoint = cmd->u.servo.setpoint;
	go_interp_add(interp, stat->setpoint);
	*interp_s = 0.0;
	go_status_next(stat, GO_RCS_STATUS_EXEC);
	go_state_next(stat, GO_RCS_STATE_S1);
      } else {
	/* not active, so set status to executing but state to S0
	   so we don't really do anything */
	stat->enable = 0;
	go_status_next(stat, GO_RCS_STATUS_EXEC);
	go_state_next(stat, GO_RCS_STATE_S0);
      }
    } else {
      stat->enable = 0;
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
    }
  }
  /* fall through to here */
  if (go_state_match(stat, GO_RCS_STATE_S1)) {
    stat->setpoint = go_interp_eval(interp, *interp_s);
    *interp_s += set->cycle_mult_inv;
    if (*interp_s > 1.0) {
      *interp_s = 1.0;
    }
    CMD_PRINT_3("servo %d cmd servo interp %f\n",
		(int) set->id, (double) stat->setpoint);

    switch (set->servo_type) {
    case GO_SERVO_TYPE_PID:
      /*
	For PIDs, we run the PID calculation and set the output. This
	will be converted to raw units later and sent using
	ext_write_vel assuming an integrating output.
       */
      pid_run_cycle(&set->pid, stat->setpoint, stat->input, &stat->output);
      break;
    case GO_SERVO_TYPE_PASS:
      /*
	For pass-through servos, the raw output to them will equal the
	raw input from them in the steady state. We also want the
	output to equal the input in the steady state. These
	constraints are

	raw output = raw input
	output = input

	We have these relationships:

	raw output = output * output scale
	input = raw input * input scale

	So output scale and input scale must be inverses. Make sure
	this is the case in the .ini file.
      */
      stat->output = stat->setpoint;
      break;
    default:
      /*
	Without a servo type being set, we'll just output nothing.
      */
      stat->output = 0.0;
      break;
    }

    stat->ferror = stat->setpoint - stat->input;
    PERF_PRINT_3("servo %d ferror %f\n",
		 (int) set->id, (double) stat->ferror);
    if (set->log_logging) {
      entry.time = servo_timestamp();
      if (set->log_type == GO_LOG_FERROR) {
	entry.u.ferror.ferror = stat->ferror;
	go_log_add(global_go_log_ptr, &entry);
      } else if (set->log_type == GO_LOG_SETPOINT) {
	entry.u.setpoint.setpoint = stat->setpoint;
	go_log_add(global_go_log_ptr, &entry);
      } else if (set->log_type == GO_LOG_SPEED) {
	entry.u.speed.speed = stat->input_vel;
	go_log_add(global_go_log_ptr, &entry);
      }
    }

    if (cmd->u.servo.home) {
      if (! stat->homing) {
	/* we're not homing, and were just asked to home, so
	   clear our homed flag and initiate a home */
	stat->homed = 0;
	ext_joint_home(set->id);
	HOME_PRINT_2("servo %d initiating home\n", (int) set->id);
      } else if (! stat->homed) {
	/* look for homing from the external interface */
	if (ext_joint_is_home(set->id)) {
	  ext_joint_home_latch(set->id, &stat->input_latch);
	  /* input_latch is raw, so we need to convert to scaled units */
	  stat->input_latch = stat->raw_input * set->input_scale;
	  stat->homed = 1;
	  /* leave the homing flag set so that we don't re-initiate
	     a homing action */
	  HOME_PRINT_2("servo %d finished home\n", (int) set->id);
	}
      }
    }
    /* our homing flag is an echo of traj's home request */
    stat->homing = cmd->u.servo.home;
  } else {			/* GO_RCS_STATE_S0 */
    go_state_default(stat);
  }
}

static void do_cmd_stub(servo_cmd_struct * cmd, servo_stat_struct * stat, servo_set_struct * set)
{
  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_3("servo %d cmd stub %d\n", 
		(int) set->id, (int) cmd->u.stub.arg);
    go_state_new(stat);
    if (stat->admin_state == GO_RCS_ADMIN_STATE_INITIALIZED) {
      go_status_next(stat, GO_RCS_STATUS_DONE);
      go_state_next(stat, GO_RCS_STATE_S0);
    } else {
      go_status_next(stat, GO_RCS_STATUS_ERROR);
      go_state_next(stat, GO_RCS_STATE_S0);
    }
  } else {			/* S0 */
    go_state_default(stat);
  }
}

static void do_cfg_nop(servo_set_struct * set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    CFG_PRINT_2("servo %d cfg nop\n", (int) set->id);
    go_state_new(set);
    go_status_next(set, GO_RCS_STATUS_DONE);
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_cycle_time(servo_cfg_struct * cfg, servo_set_struct * set, go_real * cycle_time_inv, rtapi_integer * period_nsec)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    CFG_PRINT_3("servo %d cfg cycle time %f\n",
		(int) set->id, (double) cfg->u.cycle_time.cycle_time);
    go_state_new(set);
    if (cfg->u.cycle_time.cycle_time <= 0.0) {
      go_status_next(set, GO_RCS_STATUS_ERROR);
    } else {
      set->cycle_time = cfg->u.cycle_time.cycle_time;
      *cycle_time_inv = 1.0 / set->cycle_time;
      ext_joint_init(set->id, set->cycle_time);
      *period_nsec = (unsigned long int) (set->cycle_time * 1.0e9);
      rtapi_self_set_period(*period_nsec);
      go_status_next(set, GO_RCS_STATUS_DONE);
    }
    go_state_next(set, GO_RCS_STATE_S0);
  } else {			/* GO_RCS_STATE_S0 */
    go_state_default(set);
  }
}

static void do_cfg_cycle_mult(servo_cfg_struct * cfg, servo_set_struct * set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    go_state_new(set);
    CFG_PRINT_3("servo %d cfg cycle mult %d\n",
		(int) set->id, (double) cfg->u.cycle_mult.cycle_mult);
    if (cfg->u.cycle_mult.cycle_mult <= 0) {
      go_status_next(set, GO_RCS_STATUS_ERROR);
    } else {
      set->cycle_mult = cfg->u.cycle_mult.cycle_mult;
      set->cycle_mult_inv = 1.0 / set->cycle_mult;
      go_status_next(set, GO_RCS_STATUS_DONE);
    }
    go_state_next(set, GO_RCS_STATE_S0);
  } else {			/* GO_RCS_STATE_S0 */
    go_state_default(set);
  }
}

static void do_cfg_pid(servo_cfg_struct * cfg, servo_set_struct * set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    CFG_PRINT_2("servo %d cfg pid\n", (int) set->id);
    go_state_new(set);
    pid_reset(&set->pid);
    pid_copy_gains(&set->pid, &cfg->u.pid);
    go_status_next(set, GO_RCS_STATUS_DONE);
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_parameters(servo_cfg_struct * cfg, servo_set_struct * set)
{
  go_integer number;

  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    CFG_PRINT_3("servo %d cfg %d parameters\n", (int) set->id, (int) cfg->u.parameters.number);
    go_state_new(set);
    number = cfg->u.parameters.number;
    if (number > GO_SERVO_PARAMETER_MAX) number = GO_SERVO_PARAMETER_MAX;
    ext_set_parameters(set->id, cfg->u.parameters.parameters, cfg->u.parameters.number);
    go_status_next(set, GO_RCS_STATUS_DONE);
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_link(servo_cfg_struct * cfg, servo_set_struct * set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    CFG_PRINT_2("servo %d cfg link\n", (int) set->id);
    go_state_new(set);
    set->link = cfg->u.link.link;
    go_status_next(set, GO_RCS_STATUS_DONE);
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_debug(servo_cfg_struct * cfg, servo_set_struct * set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    CFG_PRINT_3("servo %d cfg debug %X\n", 
		(int) set->id, (int) cfg->u.debug.debug);
    go_state_new(set);
    set->debug = cfg->u.debug.debug;
    go_status_next(set, GO_RCS_STATUS_DONE);
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_active(servo_stat_struct * stat, servo_cfg_struct * cfg, servo_set_struct * set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    CFG_PRINT_3("servo %d cfg active %d\n",
		(int) set->id, (int) cfg->u.active.active);
    go_state_new(set);
    if (cfg->u.active.active) {
      /* we want to go active */
      if (! set->active) {
	/* we weren't active, so clear our homed flag */
	stat->homed = 0;
      }
      /* else we're setting an active joint active, so just
	 leave everything alone */
    } else {
      /* we want to go inactive, so turn off our homed flag */
      stat->homed = 0;
    }
    set->active = cfg->u.active.active;
    go_status_next(set, GO_RCS_STATUS_DONE);
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_home(servo_stat_struct * stat, servo_cfg_struct * cfg, servo_set_struct * set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    go_state_new(set);
    CFG_PRINT_3("servo %d cfg home %f\n",
		(int) set->id, (double) cfg->u.home.home);
    /*
      If a joint is homed, the traj loop will continually calculate
      its offset for that joint as

      offset = input latch - home

      To avoid a jump, we need to adjust both the input latch and
      the home so that their difference remains the same as
      before. If we're not homed, this can't hurt.
    */
    stat->input_latch += (cfg->u.home.home - set->home);
    set->home = cfg->u.home.home;
    go_status_next(set, GO_RCS_STATUS_DONE);
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_input_scale(servo_stat_struct * stat, servo_cfg_struct * cfg, servo_set_struct * set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    go_state_new(set);
    CFG_PRINT_3("servo %d cfg input scale %f\n",
		(int) set->id, (double) cfg->u.scale.scale);
    if (stat->admin_state == GO_RCS_ADMIN_STATE_UNINITIALIZED) {
      set->input_scale = cfg->u.scale.scale;
      go_status_next(set, GO_RCS_STATUS_DONE);
    } else {
      go_status_next(set, GO_RCS_STATUS_ERROR);
    }
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_output_scale(servo_stat_struct * stat, servo_cfg_struct * cfg, servo_set_struct * set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    go_state_new(set);
    CFG_PRINT_3("servo %d cfg output scale %f\n", 
		(int) set->id, (double) cfg->u.scale.scale);
    if (stat->admin_state == GO_RCS_ADMIN_STATE_UNINITIALIZED) {
      set->output_scale = cfg->u.scale.scale;
      go_status_next(set, GO_RCS_STATUS_DONE);
    } else {
      go_status_next(set, GO_RCS_STATUS_ERROR);
    }
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_limit(servo_stat_struct * stat, servo_cfg_struct * cfg, servo_set_struct * set, go_interp * interp)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    go_state_new(set);
    CFG_PRINT_4("servo %d cfg limit %f %f\n",
		(int) set->id, (double) cfg->u.limit.min_limit,
		(double) cfg->u.limit.max_limit);
    if (stat->admin_state == GO_RCS_ADMIN_STATE_UNINITIALIZED) {
      set->min_limit = cfg->u.limit.min_limit;
      set->max_limit = cfg->u.limit.max_limit;
      go_interp_set_here(interp, stat->input);
      go_status_next(set, GO_RCS_STATUS_DONE);
    } else {
      go_status_next(set, GO_RCS_STATUS_ERROR);
    }
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_profile(servo_stat_struct * stat, servo_cfg_struct * cfg, servo_set_struct * set, go_interp * interp)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    go_state_new(set);
    CFG_PRINT_5("servo %d cfg profile %f %f %f\n",
		(int) set->id, (double) cfg->u.profile.max_vel,
		(double) cfg->u.profile.max_acc,
		(double) cfg->u.profile.max_jerk);
    if (stat->admin_state == GO_RCS_ADMIN_STATE_UNINITIALIZED) {
      set->max_vel = cfg->u.profile.max_vel;
      set->max_acc = cfg->u.profile.max_acc;
      set->max_jerk = cfg->u.profile.max_jerk;
      go_status_next(set, GO_RCS_STATUS_DONE);
    } else {
      go_status_next(set, GO_RCS_STATUS_ERROR);
    }
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_log(servo_stat_struct * stat, servo_cfg_struct * cfg, servo_set_struct * set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    go_state_new(set);
    CFG_PRINT_4("servo %d cfg log %d %d %d\n", (int) set->id, (int) cfg->u.log.log_type, (int) cfg->u.log.log_size);
    /* we don't have our own 'which' field, so we'll use our id */
    if (GO_RESULT_OK != go_log_init(global_go_log_ptr, cfg->u.log.log_type, set->id, cfg->u.log.log_size)) {
      go_status_next(set, GO_RCS_STATUS_ERROR);
    } else {
      set->log_type = cfg->u.log.log_type;
      set->log_logging = 0;
      /* ignore the 'which' field, since ours is implicitly our id */
      go_status_next(set, GO_RCS_STATUS_DONE);
    }
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_log_start(servo_stat_struct * stat, servo_cfg_struct * cfg, servo_set_struct * set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    go_state_new(set);
    CFG_PRINT_2("servo %d cfg log start\n", (int) set->id);
    set->log_logging = 1;
    go_status_next(set, GO_RCS_STATUS_DONE);
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_log_stop(servo_stat_struct * stat, servo_cfg_struct * cfg, servo_set_struct * set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    go_state_new(set);
    CFG_PRINT_2("servo %d cfg log stop\n", (int) set->id);
    set->log_logging = 0;
    go_status_next(set, GO_RCS_STATUS_DONE);
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_servo_type(servo_stat_struct * stat, servo_cfg_struct * cfg, servo_set_struct * set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    go_state_new(set);
    CFG_PRINT_3("servo %d cfg type %d\n", (int) set->id, (int) cfg->u.servo_type.servo_type);
    set->servo_type = cfg->u.servo_type.servo_type;
    /*
      When we change our servo type, we need to adjust our output to
      avoid jumps.

      For PID servos, the output should be zeroed.
      For pass-through servos, the output should be the input.
    */
    switch (set->servo_type) {
    case GO_SERVO_TYPE_PID:
      stat->output = 0.0;
      break;
    case GO_SERVO_TYPE_PASS:
      stat->output = stat->input;
      break;
    }
    go_status_next(set, GO_RCS_STATUS_DONE);
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_stub(servo_stat_struct * stat, servo_cfg_struct * cfg, servo_set_struct * set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    go_state_new(set);
    CFG_PRINT_3("servo %d cfg stub %d\n", (int) set->id, (int) cfg->u.stub.arg);
    go_status_next(set, GO_RCS_STATUS_DONE);
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

#define PROG_PRINT_2(x,y) if (servo_set.debug & DEBUG_PROG) rtapi_print(x, y)
#define TASK_PRINT_1(x) if (servo_set.debug & DEBUG_TASK) rtapi_print(x)

#define MIN(a,b) ((a) < (b) ? (a) : (b))

void servo_loop(void * arg)
{
  servo_cmd_struct pp_servo_cmd[2], * servo_cmd_ptr, * servo_cmd_test;
  servo_stat_struct servo_stat;
  servo_cfg_struct pp_servo_cfg[2], * servo_cfg_ptr, * servo_cfg_test;
  servo_set_struct servo_set;
  /* only servo 0 deals with these, so the others will have extra stack */
  go_output_struct pp_go_output[2], * go_output_ptr, * go_output_test;
  void * tmp;
  go_input_struct go_input;
  rtapi_integer num_ain, num_aout, num_din, num_dout;
  rtapi_integer period_nsec;
  rtapi_integer old_sec, old_nsec, sec, nsec, diff_sec, diff_nsec;
  go_integer id;
  go_integer cmd_type, cfg_type;
  go_integer cmd_serial_number, cfg_serial_number;
  go_integer dclock;
  go_interp interp;
  go_real interp_s;
  go_real cycle_time_inv;
  go_real old_input;
  go_log_entry entry;
  go_integer t;

  id = (go_integer) arg;
  if (id < 0) id = 0;
  else if (id >= SERVO_NUM) id = SERVO_NUM-1;

  if (GO_RESULT_OK != go_interp_init(&interp)) {
    rtapi_print("servoloop: can't init interp %d\n", id);
    return;
  }

  /* set up ping-pong buffers */
  servo_cmd_ptr = &pp_servo_cmd[0];
  servo_cmd_test = &pp_servo_cmd[1];
  servo_cmd_ptr->head = servo_cmd_ptr->tail = 0;
  servo_cmd_ptr->type = SERVO_CMD_NOP_TYPE;
  servo_cmd_ptr->serial_number = 0;
  global_servo_comm_ptr[id].servo_cmd = *servo_cmd_ptr; /* force a write into ourself */
  /*  */
  servo_cfg_ptr = &pp_servo_cfg[0];
  servo_cfg_test = &pp_servo_cfg[1];
  servo_cfg_ptr->head = servo_cfg_ptr->tail = 0;
  servo_cfg_ptr->type = SERVO_CFG_NOP_TYPE;
  servo_cfg_ptr->serial_number = 0;
  global_servo_comm_ptr[id].servo_cfg = *servo_cfg_ptr; /* force a write into ourself */
  /*  */
  go_output_ptr = &pp_go_output[0];
  go_output_test = &pp_go_output[1];
  go_output_ptr->head = go_output_ptr->tail = 0;
  go_input.head = go_input.tail = 0;

  servo_stat.head = 0;
  servo_stat.type = SERVO_STAT_TYPE;
  servo_stat.admin_state = GO_RCS_ADMIN_STATE_UNINITIALIZED;
  servo_stat.echo_serial_number = servo_cmd_ptr->serial_number - 1;
  servo_stat.setpoint = 0.0;
  servo_stat.raw_input = 0.0;	/* set later, can't hurt to do it here */
  servo_stat.raw_output = 0.0;
  servo_stat.input = 0.0;	/* ditto */
  servo_stat.input_latch = 0.0;
  servo_stat.input_vel = 0.0;
  servo_stat.output = 0.0;
  servo_stat.ferror = 0.0;
  servo_stat.cycle_time = DEFAULT_CYCLE_TIME;
  cycle_time_inv = 1.0 / servo_stat.cycle_time;
  servo_stat.heartbeat = 0;
  servo_stat.enable = 0;
  servo_stat.homing = 0;
  servo_stat.homed = 0;
  servo_stat.tail = servo_stat.head;

  servo_set.head = 0;
  servo_set.type = SERVO_SET_TYPE;
  servo_set.admin_state = GO_RCS_ADMIN_STATE_UNINITIALIZED;
  servo_set.echo_serial_number = servo_cfg_ptr->serial_number - 1;
  servo_set.id = id;
  servo_set.cycle_time = DEFAULT_CYCLE_TIME;
  servo_set.link.type = GO_LINK_DH;
  servo_set.link.quantity = GO_QUANTITY_NONE;
  servo_set.link.u.dh.a = 0.0;
  servo_set.link.u.dh.alpha = 0.0;
  servo_set.link.u.dh.d = 0.0;
  servo_set.link.u.dh.theta = 0.0;
  servo_set.servo_type = GO_SERVO_TYPE_PID;
  servo_set.debug = 0x0;
  servo_set.active = 0;
  servo_set.home = 0.0;
  servo_set.input_scale = 1.0;
  servo_set.output_scale = 1.0;
  servo_set.min_limit = -1.0;
  servo_set.max_limit = 1.0;
  servo_set.max_vel = 1.0;
  servo_set.max_acc = 1.0;
  servo_set.max_jerk = 1.0;
  servo_set.log_type = GO_LOG_NONE;
  servo_set.log_logging = 0;
  servo_set.tail = servo_set.head;

  dclock = servo_set.cycle_mult = DEFAULT_CYCLE_MULT;
  servo_set.cycle_mult_inv = 1.0 / servo_set.cycle_mult;
  pid_init(&servo_set.pid);
  pid_set_cycle_time(&servo_set.pid, servo_set.cycle_time);
  pid_set_gains(&servo_set.pid, 
		1, 0, 0, /* p,i,d */
		0, 0,		/* vff,aff */
		-1, 1,		/* min,max_output */
		0, 0,		/* neg,posBias */
		0);		/* deadband */

  period_nsec = (rtapi_integer) (servo_set.cycle_time * 1.0e9);
  rtapi_self_set_period(period_nsec);
  rtapi_clock_get_time(&old_sec, &old_nsec);

  if (id == 0) {
    num_ain = MIN(GO_IO_NUM_AIN, ext_num_ain());
    num_aout = MIN(GO_IO_NUM_AOUT, ext_num_aout());
    num_din = MIN(GO_IO_NUM_DIN, ext_num_din());
    num_dout = MIN(GO_IO_NUM_DOUT, ext_num_dout());
    global_go_io_ptr->num_ain = num_ain;
    global_go_io_ptr->num_aout = num_aout;
    global_go_io_ptr->num_din = num_din;
    global_go_io_ptr->num_dout = num_dout;
  }

  /* read raw input the first time and set the offset such that the
     scaled input is halfway between the default min,max_limits.
     Note that this may be outside the range established
     by the min,max_limits later. We will check this in do_cfg_limit. */
  ext_joint_init(id, servo_set.cycle_time);
  ext_joint_enable(id);
  ext_read_pos(servo_set.id, &servo_stat.raw_input);
  servo_stat.input = servo_stat.raw_input * servo_set.input_scale;
  old_input = servo_stat.input;
  /* set the initial input_latch to be our starting position */
  servo_stat.input_latch = servo_stat.input;
  /* fill up our interpolator with this starting position */
  go_interp_set_here(&interp, servo_stat.input);

  PROG_PRINT_2("started servo_loop %d\n", (int) id);

  while (1) {
    /* if we're the first, deal with the IO interface */
    if (id == 0) {
      ext_trigger_in();
      /* read inputs from the external interface */
      for (t = 0; t < num_ain; t++) {
	ext_read_ain(t, &go_input.ain[t]);
      }
      for (t = 0; t < num_din; t++) {
	ext_read_din(t, &go_input.din[t]);
      }
      /* and write to shared memory */
      go_input.head++;
      go_input.tail = go_input.head;
      global_go_io_ptr->input = go_input;
    }

    /* read in command buffer, ping-pong style */
    *servo_cmd_test = global_servo_comm_ptr[id].servo_cmd;
    if (servo_cmd_test->head == servo_cmd_test->tail) {
      tmp = servo_cmd_ptr;
      servo_cmd_ptr = servo_cmd_test;
      servo_cmd_test = tmp;
    }
    cmd_type = servo_cmd_ptr->type;
    cmd_serial_number = servo_cmd_ptr->serial_number;

    switch (cmd_type) {
    case 0:
    case -1:
      break;

    case SERVO_CMD_NOP_TYPE:
    case SERVO_CMD_INIT_TYPE:
    case SERVO_CMD_HALT_TYPE:
    case SERVO_CMD_ABORT_TYPE:
    case SERVO_CMD_SHUTDOWN_TYPE:
    case SERVO_CMD_SERVO_TYPE:
    case SERVO_CMD_STUB_TYPE:
      servo_stat.command_type = cmd_type;
      if (cmd_serial_number != servo_stat.echo_serial_number) {
	servo_stat.echo_serial_number = cmd_serial_number;
	servo_stat.state = GO_RCS_STATE_NEW_COMMAND;
      }
      break;

    default:
      rtapi_print("servoloop: %s: unknown command %d\n", BN, cmd_type);
      break;
    }

    /* read in config buffer, ping-pong style */
    *servo_cfg_test = global_servo_comm_ptr[id].servo_cfg;
    if (servo_cfg_test->head == servo_cfg_test->tail) {
      tmp = servo_cfg_ptr;
      servo_cfg_ptr = servo_cfg_test;
      servo_cfg_test = tmp;
    }
    cfg_type = servo_cfg_ptr->type;
    cfg_serial_number = servo_cfg_ptr->serial_number;

    switch (cfg_type) {
    case 0:
    case -1:
      break;

    case SERVO_CFG_NOP_TYPE:
    case SERVO_CFG_CYCLE_TIME_TYPE:
    case SERVO_CFG_CYCLE_MULT_TYPE:
    case SERVO_CFG_PID_TYPE:
    case SERVO_CFG_PARAMETERS_TYPE:
    case SERVO_CFG_LINK_TYPE:
    case SERVO_CFG_DEBUG_TYPE:
    case SERVO_CFG_ACTIVE_TYPE:
    case SERVO_CFG_HOME_TYPE:
    case SERVO_CFG_INPUT_SCALE_TYPE:
    case SERVO_CFG_OUTPUT_SCALE_TYPE:
    case SERVO_CFG_LIMIT_TYPE:
    case SERVO_CFG_PROFILE_TYPE:
    case SERVO_CFG_LOG_TYPE:
    case SERVO_CFG_LOG_START_TYPE:
    case SERVO_CFG_LOG_STOP_TYPE:
    case SERVO_CFG_SERVO_TYPE_TYPE:
    case SERVO_CFG_STUB_TYPE:
      servo_set.command_type = cfg_type;
      if (cfg_serial_number != servo_set.echo_serial_number) {
	servo_set.echo_serial_number = cfg_serial_number;
	servo_set.state = GO_RCS_STATE_NEW_COMMAND;
      }
      break;

    default:
      rtapi_print("servoloop: %s: unknown config %d\n",  BN, cfg_type);
      break;
    }

    /* read inputs */
    ext_read_pos(servo_set.id, &servo_stat.raw_input);
    old_input = servo_stat.input;
    servo_stat.input = servo_stat.raw_input * servo_set.input_scale;
    servo_stat.input_vel = (servo_stat.input - old_input) * cycle_time_inv;
    if (servo_set.log_logging && servo_set.log_type == GO_LOG_INPUT) {
      entry.time = servo_timestamp();
      entry.u.input.input = servo_stat.input;
      go_log_add(global_go_log_ptr, &entry);
    }

    /* run command */
    switch (servo_stat.command_type) {
    case SERVO_CMD_NOP_TYPE:
      do_cmd_nop(&servo_stat, &servo_set);
      break;

    case SERVO_CMD_INIT_TYPE:
      do_cmd_init(&servo_stat, &servo_set);
      break;

    case SERVO_CMD_ABORT_TYPE:
      do_cmd_abort(&servo_stat, &servo_set);
      break;

    case SERVO_CMD_HALT_TYPE:
      do_cmd_halt(&servo_stat, &servo_set);
      break;

    case SERVO_CMD_SHUTDOWN_TYPE:
      do_cmd_shutdown(&servo_stat, &servo_set);
      break;

    case SERVO_CMD_SERVO_TYPE:
      do_cmd_servo(servo_cmd_ptr, &servo_stat, &servo_set, &interp, &interp_s);
      break;

    case SERVO_CMD_STUB_TYPE:
      do_cmd_stub(servo_cmd_ptr, &servo_stat, &servo_set);
      break;

    default:
      break;
    }

    /* write outputs */
    servo_stat.raw_output = servo_stat.output * servo_set.output_scale;
    if (servo_stat.enable) {
      switch (servo_set.servo_type) {
      case GO_SERVO_TYPE_PID:
	/* PID assumes an integrating system, with output meaning velocity */
	ext_write_vel(servo_set.id, servo_stat.raw_output);
	break;
      case GO_SERVO_TYPE_PASS:
	/* pass-through assumes a downstream position servo */
	ext_write_pos(servo_set.id, servo_stat.raw_output);
	break;
      default:
	/* No servo type? Don't write anything out. */
	break;
      }
    }

    /* run config */
    switch (servo_set.command_type) {
    case SERVO_CFG_NOP_TYPE:
      do_cfg_nop(&servo_set);
      break;

    case SERVO_CFG_CYCLE_TIME_TYPE:
      do_cfg_cycle_time(servo_cfg_ptr, &servo_set, &cycle_time_inv, &period_nsec);
      break;

    case SERVO_CFG_CYCLE_MULT_TYPE:
      do_cfg_cycle_mult(servo_cfg_ptr, &servo_set);
      break;

    case SERVO_CFG_PID_TYPE:
      do_cfg_pid(servo_cfg_ptr, &servo_set);
      break;

    case SERVO_CFG_PARAMETERS_TYPE:
      do_cfg_parameters(servo_cfg_ptr, &servo_set);
      break;

    case SERVO_CFG_LINK_TYPE:
      do_cfg_link(servo_cfg_ptr, &servo_set);
      break;

    case SERVO_CFG_DEBUG_TYPE:
      do_cfg_debug(servo_cfg_ptr, &servo_set);
      break;

    case SERVO_CFG_ACTIVE_TYPE:
      do_cfg_active(&servo_stat, servo_cfg_ptr, &servo_set);
      break;

    case SERVO_CFG_HOME_TYPE:
      do_cfg_home(&servo_stat, servo_cfg_ptr, &servo_set);
      break;

    case SERVO_CFG_INPUT_SCALE_TYPE:
      do_cfg_input_scale(&servo_stat, servo_cfg_ptr, &servo_set);
      break;

    case SERVO_CFG_OUTPUT_SCALE_TYPE:
      do_cfg_output_scale(&servo_stat, servo_cfg_ptr, &servo_set);
      break;

    case SERVO_CFG_LIMIT_TYPE:
      do_cfg_limit(&servo_stat, servo_cfg_ptr, &servo_set, &interp);
      break;

    case SERVO_CFG_PROFILE_TYPE:
      do_cfg_profile(&servo_stat, servo_cfg_ptr, &servo_set, &interp);
      break;

    case SERVO_CFG_LOG_TYPE:
      do_cfg_log(&servo_stat, servo_cfg_ptr, &servo_set);
      break;

    case SERVO_CFG_LOG_START_TYPE:
      do_cfg_log_start(&servo_stat, servo_cfg_ptr, &servo_set);
      break;

    case SERVO_CFG_LOG_STOP_TYPE:
      do_cfg_log_stop(&servo_stat, servo_cfg_ptr, &servo_set);
      break;

    case SERVO_CFG_SERVO_TYPE_TYPE:
      do_cfg_servo_type(&servo_stat, servo_cfg_ptr, &servo_set);
      break;

    case SERVO_CFG_STUB_TYPE:
      do_cfg_stub(&servo_stat, servo_cfg_ptr, &servo_set);
      break;

    default:
      break;
    }

    servo_stat.heartbeat++;
    rtapi_clock_get_time(&sec, &nsec);
    rtapi_clock_get_interval(old_sec, old_nsec,
			     sec, nsec,
			     &diff_sec, &diff_nsec);
    old_sec = sec, old_nsec = nsec;
    servo_stat.cycle_time = ((go_real) diff_sec) +
      ((go_real) diff_nsec) * 1.0e-9;

    /* write status and settings */
    servo_stat.tail = ++servo_stat.head;
    global_servo_comm_ptr[id].servo_stat = servo_stat;
    /*  */
    servo_set.tail = ++servo_set.head;
    global_servo_comm_ptr[id].servo_set = servo_set;

    /* release the task semaphore to clock traj's execution */
    if (id == 0) {
      /* read outputs ping-pong style from shared memory */
      *go_output_test = global_go_io_ptr->output;
      if (go_output_test->head == go_output_test->tail) {
	tmp = go_output_ptr;
	go_output_ptr = go_output_test;
	go_output_test = tmp;
      }
      /* and write them to the external interface */
      for (t = 0; t < num_aout; t++) {
	ext_write_aout(t, go_output_ptr->aout[t]);
      }
      for (t = 0; t < num_dout; t++) {
	ext_write_dout(t, go_output_ptr->dout[t]);
      }

      if (--dclock <= 0) {
	rtapi_sem_give(servo_sem);
	TASK_PRINT_1("servo gave semaphore\n");
	dclock = servo_set.cycle_mult;
      }
    }

    if (servo_stat.admin_state == GO_RCS_ADMIN_STATE_SHUT_DOWN) {
      break;
    } else {
      rtapi_wait(period_nsec);
    }
  } /* while (1) */

  /* free up traj to safely shut down */
  if (id == 0) {
    rtapi_sem_give(servo_sem);
  }

  /* disable the joint hardware */
  (void) ext_joint_disable(id);

  /* exit external interface */
  (void) ext_joint_quit(id);

  PROG_PRINT_2("servo %d done\n", (int) id);

  (void) rtapi_task_exit();

  return;
}
