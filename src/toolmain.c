/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <stddef.h>		/* NULL */
#include <rtapi.h>
#include <rtapi_app.h>
#include "extintf.h"		/* ext_init,quit */
#include "go.h"
#include "gorcs.h"
#include "toolintf.h"

#define DEFAULT_CYCLE_TIME 0.1

#define CMD_PRINT_1(x) if (set->debug & DEBUG_CMD) rtapi_print(x)
#define CMD_PRINT_2(x,y) if (set->debug & DEBUG_CMD) rtapi_print(x, y)
#define CMD_PRINT_3(x,y,z) if (set->debug & DEBUG_CMD) rtapi_print(x, y, z)

#define CFG_PRINT_1(x) if (set->debug & DEBUG_CFG) rtapi_print(x)
#define CFG_PRINT_2(x,y) if (set->debug & DEBUG_CFG) rtapi_print(x, y)
#define CFG_PRINT_3(x,y,z) if (set->debug & DEBUG_CFG) rtapi_print(x, y, z)
#define CFG_PRINT_4(x,y,z,u) if (set->debug & DEBUG_CFG) rtapi_print(x, y, z, u)

static void do_cmd_nop(tool_stat_struct *stat, tool_set_struct *set)
{
  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_1("tool: cmd nop\n");
    go_state_new(stat);
    go_status_next(stat, GO_RCS_STATUS_DONE);
    go_state_next(stat, GO_RCS_STATE_S0);
  } else {			/* S0 */
    go_state_default(stat);
  }
}

static void do_cmd_init(tool_stat_struct *stat, tool_set_struct *set)
{
  go_integer t;

  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_1("tool: cmd init\n");
    go_state_new(stat);
    stat->admin_state = GO_RCS_ADMIN_STATE_INITIALIZED;
    for (t = 0; t < GO_ARRAYELS(stat->value); t++) {
      stat->value[t] = 0;
    }
    go_status_next(stat, GO_RCS_STATUS_DONE);
    go_state_next(stat, GO_RCS_STATE_S0);
  } else {			/* S0 */
    go_state_default(stat);
  }
}

static void do_cmd_abort(tool_stat_struct *stat, tool_set_struct *set)
{
  go_integer t;

  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_1("tool: cmd abort\n");
    go_state_new(stat);
    for (t = 0; t < GO_ARRAYELS(stat->value); t++) {
      stat->value[t] = 0;
    }
    go_status_next(stat, GO_RCS_STATUS_DONE);
    go_state_next(stat, GO_RCS_STATE_S0);
  } else {			/* S0 */
    go_state_default(stat);
  }
}

static void do_cmd_on(tool_cmd_struct *cmd, tool_stat_struct *stat, tool_set_struct *set)
{
  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_3("tool: cmd [%d] on %f\n", (int) cmd->id, (double) cmd->u.on.value);
    go_state_new(stat);
    if (GO_ARRAYBAD(stat->value, cmd->id)) {
      go_status_next(stat, GO_RCS_STATUS_ERROR);
    } else {
      stat->value[cmd->id] = cmd->u.on.value;
      go_status_next(stat, GO_RCS_STATUS_DONE);
    }
    go_state_next(stat, GO_RCS_STATE_S0);
  } else {			/* S0 */
    go_state_default(stat);
  }
}

static void do_cmd_off(tool_cmd_struct *cmd, tool_stat_struct *stat, tool_set_struct *set)
{
  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_2("tool: cmd [%d] off\n", (int) cmd->id);
    go_state_new(stat);
    if (GO_ARRAYBAD(stat->value, cmd->id)) {
      go_status_next(stat, GO_RCS_STATUS_ERROR);
    } else {
      stat->value[cmd->id] = 0.0;
      go_status_next(stat, GO_RCS_STATUS_DONE);
    }
    go_state_next(stat, GO_RCS_STATE_S0);
  } else {			/* S0 */
    go_state_default(stat);
  }
}

static rtapi_integer exit_me = 0;

static void do_cmd_shutdown(tool_stat_struct *stat, tool_set_struct *set)
{
  go_integer t;

  if (go_state_match(stat, GO_RCS_STATE_NEW_COMMAND)) {
    CMD_PRINT_1("tool: cmd shutdown\n");
    for (t = 0; t < GO_ARRAYELS(stat->value); t++) {
      stat->value[t] = 0;
    }
    exit_me = 1;
    go_state_new(stat);
    go_status_next(stat, GO_RCS_STATUS_DONE);
    go_state_next(stat, GO_RCS_STATE_S0);
  } else {			/* S0 */
    go_state_default(stat);
  }
}

static void do_cfg_nop(tool_set_struct * set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    CFG_PRINT_1("tool: cfg nop\n");
    go_state_new(set);
    go_status_next(set, GO_RCS_STATUS_DONE);
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

static void do_cfg_cycle_time(tool_cfg_struct *cfg, tool_set_struct *set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    CFG_PRINT_2("tool: cfg cycle time %f\n", (double) cfg->u.cycle_time.cycle_time);
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

static void do_cfg_debug(tool_cfg_struct * cfg, tool_set_struct * set)
{
  if (go_state_match(set, GO_RCS_STATE_NEW_COMMAND)) {
    go_state_new(set);
    set->debug = cfg->u.debug.debug;
    /* put the print here since it references the debug value for printing */
    CFG_PRINT_2("tool: cfg debug %x\n", (int) cfg->u.debug.debug);
    go_status_next(set, GO_RCS_STATUS_DONE);
    go_state_next(set, GO_RCS_STATE_S0);
  } else {
    go_state_default(set);
  }
}

#define PROG_PRINT_1(x) if (tool_set.debug & DEBUG_PROG) rtapi_print(x)
#define PROG_PRINT_2(x,y) if (tool_set.debug & DEBUG_PROG) rtapi_print(x, y)

static tool_comm_struct *global_tool_comm_ptr = NULL;

void tool_loop(void *arg)
{
  tool_cmd_struct pp_tool_cmd[2], * tool_cmd_ptr, * tool_cmd_test;
  tool_stat_struct tool_stat;
  tool_cfg_struct pp_tool_cfg_struct[2], * tool_cfg_ptr, * tool_cfg_test;
  tool_set_struct tool_set;
  rtapi_integer old_sec = 0, old_nsec = 0, sec, nsec;
  rtapi_integer diff_sec, diff_nsec;
  void *tmp;
  go_integer cmd_type, cfg_type;
  go_integer cmd_serial_number, cfg_serial_number;
  rtapi_integer t;

  if (GO_RESULT_OK != go_init()) {
    rtapi_print("tool: can't init tool motion queue\n");
    return;
  }

  /* set up ping-pong buffers */
  tool_cmd_ptr = &pp_tool_cmd[0];
  tool_cmd_test = &pp_tool_cmd[1];
  tool_cmd_ptr->head = tool_cmd_ptr->tail = 0;
  tool_cmd_ptr->type = TOOL_CMD_NOP_TYPE;
  tool_cmd_ptr->serial_number = 1;
  global_tool_comm_ptr->tool_cmd = *tool_cmd_ptr; /* force a write into ourself */
  /*  */
  tool_cfg_ptr = &pp_tool_cfg_struct[0];
  tool_cfg_test = &pp_tool_cfg_struct[1];
  tool_cfg_ptr->head = tool_cfg_ptr->tail = 0;
  tool_cfg_ptr->type = TOOL_CFG_NOP_TYPE;
  tool_cfg_ptr->serial_number = 1;
  global_tool_comm_ptr->tool_cfg = *tool_cfg_ptr; /* as above */

  tool_stat.head = 0;
  tool_stat.type = TOOL_STAT_TYPE;
  tool_stat.admin_state = GO_RCS_ADMIN_STATE_UNINITIALIZED;
  tool_stat.echo_serial_number = tool_cmd_ptr->serial_number - 1;
  tool_stat.heartbeat = 0;
  tool_stat.cycle_time = DEFAULT_CYCLE_TIME;
  for (t = 0; t < GO_ARRAYELS(tool_stat.value); t++) {
  }
  tool_stat.tail = tool_stat.head;

  tool_set.head = 0;
  tool_set.type = TOOL_SET_TYPE;
  tool_set.echo_serial_number = tool_cfg_ptr->serial_number - 1;
  tool_set.cycle_time = DEFAULT_CYCLE_TIME;
  tool_set.debug = 0x0;
  tool_set.tail = tool_set.head;

  PROG_PRINT_1("tool: started tool loop\n");

  while (1) {
    /* read in command buffer, ping-pong style */
    *tool_cmd_test = global_tool_comm_ptr->tool_cmd;
    if (tool_cmd_test->head == tool_cmd_test->tail) {
      tmp = tool_cmd_ptr;
      tool_cmd_ptr = tool_cmd_test;
      tool_cmd_test = tmp;
    }
    cmd_type = tool_cmd_ptr->type;
    cmd_serial_number = tool_cmd_ptr->serial_number;

    switch (cmd_type) {
    case 0:
    case -1:
      break;

    case TOOL_CMD_NOP_TYPE:
    case TOOL_CMD_INIT_TYPE:
    case TOOL_CMD_ABORT_TYPE:
    case TOOL_CMD_SHUTDOWN_TYPE:
    case TOOL_CMD_ON_TYPE:
    case TOOL_CMD_OFF_TYPE:
      tool_stat.command_type = cmd_type;
      if (cmd_serial_number != tool_stat.echo_serial_number) {
	tool_stat.echo_serial_number = cmd_serial_number;
	tool_stat.state = GO_RCS_STATE_NEW_COMMAND;
      }
      break;

    default:
      rtapi_print("tool: unknown command %d\n", cmd_type);
      break;
    }

    /* read in config buffer, ping-pong style */
    *tool_cfg_test = global_tool_comm_ptr->tool_cfg;
    if (tool_cfg_test->head == tool_cfg_test->tail) {
      tmp = tool_cfg_ptr;
      tool_cfg_ptr = tool_cfg_test;
      tool_cfg_test = tmp;
    }
    cfg_type = tool_cfg_ptr->type;
    cfg_serial_number = tool_cfg_ptr->serial_number;

    switch (cfg_type) {
    case 0:
    case -1:
      break;

    case TOOL_CFG_NOP_TYPE:
    case TOOL_CFG_CYCLE_TIME_TYPE:
    case TOOL_CFG_DEBUG_TYPE:
      tool_set.command_type = cfg_type;
      if (cfg_serial_number != tool_set.echo_serial_number) {
	tool_set.echo_serial_number = cfg_serial_number;
	tool_set.state = GO_RCS_STATE_NEW_COMMAND;
      }
      break;

    default:
      rtapi_print("tool: unknown config %d\n",  cfg_type);
      break;
    }

    switch (tool_stat.command_type) {
    case TOOL_CMD_NOP_TYPE:
      do_cmd_nop(&tool_stat, &tool_set);
      break;

    case TOOL_CMD_INIT_TYPE:
      do_cmd_init(&tool_stat, &tool_set);
      break;

    case TOOL_CMD_ABORT_TYPE:
      do_cmd_abort(&tool_stat, &tool_set);
      break;

    case TOOL_CMD_SHUTDOWN_TYPE:
      do_cmd_shutdown(&tool_stat, &tool_set);
      break;

    case TOOL_CMD_ON_TYPE:
      do_cmd_on(tool_cmd_ptr, &tool_stat, &tool_set);
      break;

    case TOOL_CMD_OFF_TYPE:
      do_cmd_off(tool_cmd_ptr, &tool_stat, &tool_set);
      break;

    default:
      break;
    }

    switch (tool_set.command_type) {
    case TOOL_CFG_NOP_TYPE:
      do_cfg_nop(&tool_set);
      break;

    case TOOL_CFG_CYCLE_TIME_TYPE:
      do_cfg_cycle_time(tool_cfg_ptr, &tool_set);
      break;

    case TOOL_CFG_DEBUG_TYPE:
      do_cfg_debug(tool_cfg_ptr, &tool_set);
      break;

    default:
      break;
    }

    /* update status */
    tool_stat.heartbeat++;
    rtapi_clock_get_time(&sec, &nsec);
    rtapi_clock_get_interval(old_sec, old_nsec,
			     sec, nsec,
			     &diff_sec, &diff_nsec);
    old_sec = sec, old_nsec = nsec;
    tool_stat.cycle_time = ((go_real) diff_sec) +
      ((go_real) diff_nsec) * 1.0e-9;

    /* write out tool status and settings */
    tool_stat.tail = ++tool_stat.head;
    global_tool_comm_ptr->tool_stat = tool_stat;
    /*  */
    tool_set.tail = ++tool_set.head;
    global_tool_comm_ptr->tool_set = tool_set;

    if (exit_me) {
      break;
    }

    rtapi_wait(tool_set.cycle_time * 1.0e9);
  } /* while (1) */

  PROG_PRINT_1("tool_loop done\n");

  (void) rtapi_task_exit();

  return;
}

/*!
  NOMINAL_PERIOD_NSEC is the nominal period of the servo and tool
  control tasks. It will be set to be longer than this based on
  the cycle times from the .ini file later during the config process.
 */
#define NOMINAL_PERIOD_NSEC 1000000

static void *tool_task = NULL;
#define TOOL_STACKSIZE 8000

static void *tool_shm = NULL;

/* declare comm params that aren't set via the config process later */
RTAPI_DECL_INT(DEBUG, 0);
RTAPI_DECL_INT(TOOL_SHM_KEY, 201);
RTAPI_DECL_STRING(EXT_INIT_STRING, "");

rtapi_integer rtapi_app_main(RTAPI_APP_ARGS_DECL)
{
  rtapi_integer tool_prio;

  if (0 != rtapi_app_init(RTAPI_APP_ARGS)) {
    return 1;
  }

  /* get command line args */
  (void) rtapi_arg_get_int(&DEBUG, "DEBUG");
  if (DEBUG) rtapi_print("tool: using DEBUG = %d\n", DEBUG);
  (void) rtapi_arg_get_int(&TOOL_SHM_KEY, "TOOL_SHM_KEY");
  if (DEBUG) rtapi_print("tool: using TOOL_SHM_KEY = %d\n", TOOL_SHM_KEY);

  if (DEBUG) rtapi_print("tool: main running off base clock period %d\n", rtapi_clock_period);

  /* allocate the tool comm buffer */
  tool_shm = rtapi_rtm_new(TOOL_SHM_KEY, sizeof(tool_comm_struct));
  if (NULL == tool_shm) {
    rtapi_print("tool: can't get tool comm shm\n");
    return 1;
  }
  global_tool_comm_ptr = rtapi_rtm_addr(tool_shm);

  /* set prios as servo, then tool */
  tool_prio = rtapi_prio_lowest();

  /* initialize the external interface */
  ext_init(EXT_INIT_STRING);

  /* launch the tool task */

  /* first, fill in some things we know that tool wants */

  tool_task = rtapi_task_new();
  if (NULL == tool_task) {
    rtapi_print("tool: can't allocate tool task\n");
    return 1;
  }
  if (DEBUG) rtapi_print("tool: allocated tool task\n");

  if (0 != rtapi_task_start(tool_task,
			    tool_loop,
			    NULL,
			    tool_prio, 
			    TOOL_STACKSIZE,
			    NOMINAL_PERIOD_NSEC,
			    1)) { /* 1 = floating point */
    rtapi_print("tool: can't start tool task\n");
    return 1;
  }
  if (DEBUG) rtapi_print("tool: started tool task %x\n", tool_task);

  if (DEBUG) rtapi_print("tool: main started\n");

  return rtapi_app_wait();
}

void rtapi_app_exit(void)
{
  rtapi_integer unused;

  if (NULL != tool_task) {
    unused = rtapi_task_stack_check(tool_task);
    if (0 == unused) {
      rtapi_print("tool: stack overwritten\n");
    } else if (unused > 0) {
      if (DEBUG) rtapi_print("tool: %d unused tool stack words\n", unused);
    } /* else the stack check is irrelevant */
    (void) rtapi_task_stop(tool_task);
    (void) rtapi_task_delete(tool_task);
    tool_task = 0;
  }

  if (NULL != tool_shm) {
    rtapi_rtm_delete(tool_shm);
    tool_shm = NULL;
  }
  global_tool_comm_ptr = NULL;

  if (DEBUG) rtapi_print("tool: toolmain done\n");

  ext_quit();

  return;
}
