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
  \file gotk.c

  \brief C code for Tcl/Tk interface to Go Motion.

  It expects a Tk script as the first argument, but it can be '-' for
  stdin, so to run interactively you do:

  gotk - -- -i \<ini file\>

  where the second '--' keeps Tk from parsing our command line args.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>		/* fprintf, stderr, FILE */
#include <stddef.h>		/* sizeof, NULL */
#include <string.h>		/* strcmp */
#include <stdlib.h>		/* exit, atexit */
#include <stdarg.h>		/* va_list */
#include <ctype.h>		/* toupper, isspace */
#include <signal.h>		/* signal, SIGINT */
#include <tk.h>			/* Tk_XXX */
#include <inifile.h>
#include <ulapi.h>		/* ulapi_time */
#include "gotypes.h"		/* GO_INF */
#include "gomath.h"		/* go_quat_rot_convert */
#include "gorcsutil.h"		/* ulapi_time, ulapi_sleep, rcs_state_to_string */
#include "servointf.h"		/* servo_cmd_structXXX,Stat,CfgXXX,Set */
#include "trajintf.h"		/* traj_cmd_structXXX,Stat,CfgXXX,Set */
#include "toolintf.h"		/* tool_cmd_struct, etc */
#include "taskintf.h"		/* task_cmd_struct, etc */

#ifdef BUILD_GOTCL
#undef HAVE_SDL
#endif

#ifdef HAVE_SDL
#include "SDL.h"
#endif

#define CONNECT_WAIT_TIME 1.0

#define SAFECPY(dst,src) strncpy((dst),(src),sizeof(dst)); (dst)[sizeof(dst)-1] = 0

/*
  'dbprintf' is debug printf that shows what's going on during init
  and shutdown. Fatal errors are printed regardless.
*/
static int dbflag = 0;
static int killflag = 0;
static int peekflag = 0;
static void dbprintf(const char * fmt, ...)
{
  va_list ap;

  if (dbflag) {
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    fflush(stderr);
    va_end(ap);
  }
}

/* simple tokens to be passed as ClientData to switch between cases */
#define CD1 ((ClientData *) 1)
#define CD2 ((ClientData *) 2)
#define CD3 ((ClientData *) 3)
#define CD4 ((ClientData *) 4)
#define CD5 ((ClientData *) 5)
#define CD6 ((ClientData *) 6)

static servo_cmd_struct servo_cmd[SERVO_NUM];
static servo_cfg_struct servo_cfg[SERVO_NUM];
static servo_stat_struct pp_servo_stat[2][SERVO_NUM], *servo_stat_ptr[SERVO_NUM], *servo_stat_test[SERVO_NUM];
static servo_set_struct pp_servo_set_struct[2][SERVO_NUM],
  *servo_set_ptr[SERVO_NUM], *servo_set_test[SERVO_NUM];
servo_comm_struct servo_comm_dummy[SERVO_NUM];
servo_comm_struct *servo_comm_ptr = servo_comm_dummy;
static int servo_howmany;
static int servo_shm_key;
static void *servo_shm;

static traj_cmd_struct traj_cmd;
static traj_cfg_struct traj_cfg;
static traj_stat_struct pp_traj_stat[2], *traj_stat_ptr, *traj_stat_test;
static traj_set_struct pp_traj_set[2], *traj_set_ptr, *traj_set_test;
static traj_comm_struct traj_comm_dummy;
static traj_comm_struct *traj_comm_ptr = &traj_comm_dummy;
static int traj_shm_key;
static void *traj_shm;

static tool_cmd_struct tool_cmd;
static tool_cfg_struct tool_cfg;
static tool_stat_struct pp_tool_stat[2], *tool_stat_ptr, *tool_stat_test;
static tool_set_struct pp_tool_set[2], *tool_set_ptr, *tool_set_test;
static tool_comm_struct tool_comm_dummy;
static tool_comm_struct *tool_comm_ptr = &tool_comm_dummy;
static int tool_shm_key;
static void *tool_shm;

static task_cmd_struct task_cmd;
static task_cfg_struct task_cfg;
static task_stat_struct pp_task_stat[2], *task_stat_ptr, *task_stat_test;
static task_set_struct pp_task_set[2], *task_set_ptr, *task_set_test;
static task_comm_struct task_comm_dummy;
static task_comm_struct *task_comm_ptr = &task_comm_dummy;
static int task_shm_key;
static void *task_shm;

static go_log_struct go_log_dummy;
static go_log_struct *go_log_ptr = &go_log_dummy;
static int go_log_shm_key;
static void *go_log_shm;

static go_input_struct pp_go_input[2], *go_input_ptr, *go_input_test;
static go_io_struct go_io_dummy;
static go_io_struct *go_io_ptr = &go_io_dummy;
static int go_io_shm_key;
static void *go_io_shm;

static go_real local_jvel[SERVO_NUM];
static go_real local_jacc[SERVO_NUM];
static go_real local_jjerk[SERVO_NUM];

static go_real local_tvel;
static go_real local_tacc;
static go_real local_tjerk;

static go_real local_rvel;
static go_real local_racc;
static go_real local_rjerk;

static go_real local_move_time;

static go_real m_per_length_units, rad_per_angle_units;
static double length_units_per_m, angle_units_per_rad;
static go_real home_vel[SERVO_NUM];

static go_integer joint_quantity[SERVO_NUM];

#define ESTOP_COMMAND_LEN 256
static char estop_command[ESTOP_COMMAND_LEN] = "";

/*
  Mutex for shared resources between scripts and the code here, e.g.,
  the coordinate system 'csys_select'
*/
enum {MUTEX_KEY = 101};
static void *mutex = NULL;
#define LOCK if (NULL != mutex) ulapi_mutex_take(mutex)
#define UNLOCK if (NULL != mutex) ulapi_mutex_give(mutex)

/*
  'csys_select' is the coordinate system selected, either joint, world or
  tool. The value can be set and read by both Tcl scripts and this C
  code here. This allows Tcl buttons or the joystick to affect it.
*/
typedef enum {CSYS_JOINT = 1, CSYS_WORLD, CSYS_TOOL} csys_select_t;
static csys_select_t csys_select = CSYS_JOINT;

/*
  'joint_select' is the index of which joint is selected, starting at
  0 and running through the number of joints minus one.
*/
static go_integer joint_select = 0;

/*
  'cart_select' is the Cartesian axis selected, one of CART_X,Y,Z,R,P,W.
*/
typedef enum {CART_X = 1,CART_Y, CART_Z, CART_R, CART_P, CART_W} cart_select_t;
static cart_select_t cart_select = CART_X;

/*
  'input_control' is non-zero if the Tk GUI has control, zero if
  the Tk GUI has relinquished control, for example to the joystick.
*/
static go_flag input_control = 1;

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

#define DO_TRAJ_CMD				\
  traj_cmd.serial_number++;			\
  traj_cmd.tail = ++traj_cmd.head;		\
  traj_comm_ptr->traj_cmd = traj_cmd;

#define DO_TRAJ_CMD_IF_NEEDED				\
  if (traj_comm_ptr->traj_stat.command_type != traj_cmd.type) {	\
    traj_cmd.serial_number++;				\
    traj_cmd.tail = ++traj_cmd.head;			\
    traj_comm_ptr->traj_cmd = traj_cmd;			\
  }

#define DO_TRAJ_CFG				\
  traj_cfg.serial_number++;			\
  traj_cfg.tail = ++traj_cfg.head;		\
  traj_comm_ptr->traj_cfg = traj_cfg;

#define DO_TOOL_CMD				\
  tool_cmd.serial_number++;			\
  tool_cmd.tail = ++tool_cmd.head;		\
  tool_comm_ptr->tool_cmd = tool_cmd;

#define DO_TOOL_CMD_IF_NEEDED				\
  if (tool_comm_ptr->tool_stat.command_type != tool_cmd.type) {	\
    tool_cmd.serial_number++;				\
    tool_cmd.tail = ++tool_cmd.head;			\
    tool_comm_ptr->tool_cmd = tool_cmd;			\
  }

#define DO_TASK_CMD				\
  task_cmd.serial_number++;			\
  task_cmd.tail = ++task_cmd.head;		\
  task_comm_ptr->task_cmd = task_cmd;

#define DO_TASK_CMD_IF_NEEDED				\
  if (task_comm_ptr->task_stat.command_type != task_cmd.type) {	\
    task_cmd.serial_number++;				\
    task_cmd.tail = ++task_cmd.head;			\
    task_comm_ptr->task_cmd = task_cmd;			\
  }

#define DO_SERVO_CFG(servo_num)					\
  servo_cfg[servo_num].serial_number++;				\
  servo_cfg[servo_num].tail = ++servo_cfg[servo_num].head;	\
  servo_comm_ptr[servo_num].servo_cfg = servo_cfg[servo_num];

/* set by gotk_set_timeout <time in secs>, this is initially
   negative to signify no timeout */
static double timeout = GO_INF;

static int
gotk_set_timeout(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  double d1;

  if (objc != 2) {
    Tcl_WrongNumArgs(interp, 1, objv, "<timeout in secs>");
    return TCL_ERROR;
  }

  if (!strcmp("none", Tcl_GetString(objv[1]))) {
    d1 = -1;
  } else if (!strcmp("forever", Tcl_GetString(objv[1]))) {
    d1 = GO_INF;
  } else if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[1], &d1)) {
    return TCL_ERROR;
  }

  timeout = d1;

  return TCL_OK;
}

static int
have_task_comm_buffers(void)
{
  return task_comm_ptr != &task_comm_dummy;
}

static int
gotk_have_task(ClientData clientData, Tcl_Interp *interp, int objc, Tcl_Obj *CONST objv[])
{
  Tcl_Obj *resultPtr;

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetIntObj(resultPtr, have_task_comm_buffers());

  return TCL_OK;
}

static int
get_task_comm_buffers(void)
{
  ulapi_real end;
  int start_it;
  int got_it;
  int heartbeat;

  task_shm = ulapi_rtm_new(task_shm_key, sizeof(task_comm_struct));
  if (NULL == task_shm) {
    task_comm_ptr = &task_comm_dummy;
  } else {
    task_comm_ptr = ulapi_rtm_addr(task_shm);
    /* set up ping-pong buffers */
    task_stat_ptr = &pp_task_stat[0];
    task_stat_test = &pp_task_stat[1];
    /*  */
    task_set_ptr = &pp_task_set[0];
    task_set_test = &pp_task_set[1];
    /* check for task life */
    for (start_it = 0, got_it = 0, end = ulapi_time() + CONNECT_WAIT_TIME;
	 ulapi_time() < end;
	 ulapi_sleep(0.1)) {
      *task_stat_ptr = task_comm_ptr->task_stat;
      if (task_stat_ptr->head == task_stat_ptr->tail &&
	  task_stat_ptr->type == TASK_STAT_TYPE) {
	if (! start_it) {
	  start_it = 1;
	  heartbeat = task_stat_ptr->heartbeat;
	}
	if (heartbeat != task_stat_ptr->heartbeat) {
	  got_it = 1;
	  break;
	}
      }
    }
    if (! got_it) {
      task_comm_ptr = &task_comm_dummy;
    }
    task_cmd.serial_number = task_stat_ptr->echo_serial_number + 1;
    *task_set_ptr = task_comm_ptr->task_set;
    task_cfg.serial_number = task_set_ptr->echo_serial_number + 1;
  }

  return have_task_comm_buffers() ? 0 : 1;
}

static void
free_task_comm_buffers(void)
{
  if (NULL != task_shm) {
    ulapi_rtm_delete(task_shm);
    task_shm = NULL;
  }
  task_comm_ptr = &task_comm_dummy;
}

static int
update_task_comm_buffers(void)
{
  void *tmp;

  if (! have_task_comm_buffers()) return 1;

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

  return 0;
}

static int task_wait_done(void)
{
  double end;
  int got_it;

  if (timeout <= 0.0) {
    return GO_RCS_STATUS_DONE;
  }

  for (got_it = 0, end = ulapi_time() + timeout; ulapi_time() < end;) {
    (void) update_task_comm_buffers();
    if ((task_comm_ptr->task_stat.command_type == task_cmd.type) &&
	(task_comm_ptr->task_stat.echo_serial_number == task_cmd.serial_number)) {
      got_it = 1;
      if (task_comm_ptr->task_stat.status != GO_RCS_STATUS_EXEC) {
	return task_comm_ptr->task_stat.status;
      }
    } else {
      if (got_it) {
	/* something else is there, and it got ours already, so we
	   must have been overridden by something else */
	return task_comm_ptr->task_stat.status;
      }
    }
    /* else keep waiting */
    ulapi_sleep(0.001);
  }

  /* else we timed out */
  return GO_RCS_STATUS_EXEC;
}

static int
gotk_task(ClientData clientData, Tcl_Interp *interp, int objc, Tcl_Obj *CONST objv[])
{
  char *string;
  char *program;
  Tcl_Obj *resultPtr;

  if (objc < 2) {
    Tcl_WrongNumArgs(interp, 1, objv, "<command> {<other args>}");
    return TCL_ERROR;
  }

  string = Tcl_GetString(objv[1]);

#define ARGS(NUM,STR)				\
    if (objc != NUM) {				\
      Tcl_WrongNumArgs(interp, 1, objv, STR);	\
      return TCL_ERROR;				\
    }

  /* first process queries, and exit so we don't send a command */
  if (! strcmp("cmd", string)) {
    ARGS(2, "cmd");
    update_task_comm_buffers();
    Tcl_SetResult(interp, (char *) task_cmd_symbol(task_stat_ptr->command_type), TCL_STATIC);
    return TCL_OK;
  } else if (! strcmp("status", string)) {
    ARGS(2, "status");
    update_task_comm_buffers();
    Tcl_SetResult(interp, rcs_status_to_string(task_stat_ptr->status), TCL_STATIC);
    return TCL_OK;
  }
  
  if (! strcmp("reset", string) || ! strcmp("init", string)) {
    ARGS(2, "reset | init");
    task_cmd.type = TASK_CMD_RESET_TYPE;
  } else if (! strcmp("start", string) || ! strcmp("run", string)) {
    ARGS(3, "start | run <program>");
    program = Tcl_GetString(objv[2]);
    task_cmd.type = TASK_CMD_START_TYPE;
    SAFECPY(task_cmd.u.start.program, program);
  } else if (! strcmp("stop", string)) {
    ARGS(2, "stop");
    task_cmd.type = TASK_CMD_STOP_TYPE;
  } else if (! strcmp("hold", string)) {
    ARGS(2, "hold");
    task_cmd.type = TASK_CMD_HOLD_TYPE;
  } else if (! strcmp("unhold", string)) {
    ARGS(2, "unhold");
    task_cmd.type = TASK_CMD_UNHOLD_TYPE;
  } else if (! strcmp("suspend", string)) {
    ARGS(2, "suspend");
    task_cmd.type = TASK_CMD_SUSPEND_TYPE;
  } else if (! strcmp("unsuspend", string)) {
    ARGS(2, "unsuspend");
    task_cmd.type = TASK_CMD_UNSUSPEND_TYPE;
  } else if (! strcmp("abort", string)) {
    ARGS(2, "abort");
    task_cmd.type = TASK_CMD_ABORT_TYPE;
  } else if (! strcmp("clear", string)) {
    ARGS(2, "clear");
    task_cmd.type = TASK_CMD_CLEAR_TYPE;
  } else {
    Tcl_SetResult(interp, "bad args", TCL_STATIC);
    return TCL_ERROR;
  }

  LOCK;
  DO_TASK_CMD;
  UNLOCK;

  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetIntObj(resultPtr, task_wait_done());

  return TCL_OK;
}

static int
have_tool_comm_buffers(void)
{
  return tool_comm_ptr != &tool_comm_dummy;
}

static int
gotk_have_tool(ClientData clientData, Tcl_Interp *interp, int objc, Tcl_Obj *CONST objv[])
{
  Tcl_Obj *resultPtr;

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetIntObj(resultPtr, have_tool_comm_buffers());

  return TCL_OK;
}

static int
get_tool_comm_buffers(void)
{
  ulapi_real end;
  int start_it;
  int got_it;
  int heartbeat;

  tool_shm = ulapi_rtm_new(tool_shm_key, sizeof(tool_comm_struct));
  if (NULL == tool_shm) {
    tool_comm_ptr = &tool_comm_dummy;
  } else {
    tool_comm_ptr = ulapi_rtm_addr(tool_shm);
    /* set up ping-pong buffers */
    tool_stat_ptr = &pp_tool_stat[0];
    tool_stat_test = &pp_tool_stat[1];
    /*  */
    tool_set_ptr = &pp_tool_set[0];
    tool_set_test = &pp_tool_set[1];
    /* check for tool life */
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
      tool_comm_ptr = &tool_comm_dummy;
    }
    tool_cmd.serial_number = tool_stat_ptr->echo_serial_number + 1;
    *tool_set_ptr = tool_comm_ptr->tool_set;
    tool_cfg.serial_number = tool_set_ptr->echo_serial_number + 1;
  }

  return have_tool_comm_buffers() ? 0 : 1;
}

static void
free_tool_comm_buffers(void)
{
  if (NULL != tool_shm) {
    ulapi_rtm_delete(tool_shm);
    tool_shm = NULL;
  }
  tool_comm_ptr = &tool_comm_dummy;
}

static int
update_tool_comm_buffers(void)
{
  void *tmp;

  if (! have_tool_comm_buffers()) return 1;

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

  return 0;
}

static int
have_traj_comm_buffers(void)
{
  return traj_comm_ptr != &traj_comm_dummy;
}

static int
get_traj_comm_buffers(void)
{
  ulapi_real end;
  int start_it;
  int got_it;
  int heartbeat;

  traj_shm = ulapi_rtm_new(traj_shm_key, sizeof(traj_comm_struct));
  if (NULL == traj_shm) {
    traj_comm_ptr = &traj_comm_dummy;
  } else {
    traj_comm_ptr = ulapi_rtm_addr(traj_shm);
    /* set up ping-pong buffers */
    traj_stat_ptr = &pp_traj_stat[0];
    traj_stat_test = &pp_traj_stat[1];
    /*  */
    traj_set_ptr = &pp_traj_set[0];
    traj_set_test = &pp_traj_set[1];
    /* check for traj life */
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
      traj_comm_ptr = &traj_comm_dummy;
    }
    traj_cmd.serial_number = traj_stat_ptr->echo_serial_number + 1;
    *traj_set_ptr = traj_comm_ptr->traj_set;
    traj_cfg.serial_number = traj_set_ptr->echo_serial_number + 1;
  }

  return have_traj_comm_buffers() ? 0 : 1;
}

static void
free_traj_comm_buffers(void)
{
  if (NULL != traj_shm) {
    ulapi_rtm_delete(traj_shm);
    traj_shm = NULL;
  }
  traj_comm_ptr = &traj_comm_dummy;
}

static int
update_traj_comm_buffers(void)
{
  void *tmp;

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

  return 0;
}

static int
have_servo_comm_buffers(void)
{
  return servo_comm_ptr != servo_comm_dummy;
}

static int
get_servo_comm_buffers(void)
{
  ulapi_real end;
  int start_it;
  int got_it;
  int heartbeat;
  int servo_num;

  /* get servo shared memory buffers */
  servo_shm = ulapi_rtm_new(servo_shm_key, SERVO_NUM * sizeof(servo_comm_struct));
  if (NULL == servo_shm) {
    servo_comm_ptr = servo_comm_dummy;
  } else {
    servo_comm_ptr = ulapi_rtm_addr(servo_shm);
    /* set up ping-pong buffers */
    for (servo_num = 0; servo_num < SERVO_NUM; servo_num++) {
      servo_stat_ptr[servo_num] = &pp_servo_stat[0][servo_num];
      servo_stat_test[servo_num] = &pp_servo_stat[1][servo_num];
      /*  */
      servo_set_ptr[servo_num] = &pp_servo_set_struct[0][servo_num];
      servo_set_test[servo_num] = &pp_servo_set_struct[1][servo_num];
      /* check for servo life */
      if (servo_num < servo_howmany) {
	for (start_it = 0, got_it = 0, end = ulapi_time() + CONNECT_WAIT_TIME;
	     ulapi_time() < end;
	     ulapi_sleep(0.1)) {
	  *servo_stat_ptr[servo_num] = servo_comm_ptr[servo_num].servo_stat;
	  if (servo_stat_ptr[servo_num]->head == servo_stat_ptr[servo_num]->tail && servo_stat_ptr[servo_num]->type == SERVO_STAT_TYPE) {
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
	  servo_comm_ptr = servo_comm_dummy;
	  /* keep going so that the full array gets set */
	}
      }
      servo_cmd[servo_num].serial_number = servo_stat_ptr[servo_num]->echo_serial_number + 1;
      *servo_set_ptr[servo_num] = servo_comm_ptr[servo_num].servo_set;
      servo_cfg[servo_num].serial_number = servo_set_ptr[servo_num]->echo_serial_number + 1;
    } /* for (servo_num) */
  }

  return have_servo_comm_buffers() ? 0 : 1;
}

static void
free_servo_comm_buffers(void)
{
  if (NULL != servo_shm) {
    ulapi_rtm_delete(servo_shm);
    servo_shm = NULL;
  }
  servo_comm_ptr = servo_comm_dummy;
}

static int
update_servo_comm_buffers(void)
{
  int servo_num;
  void *tmp;

  /* read in servo status and settings, ping-pong style */
  for (servo_num = 0; servo_num < SERVO_NUM; servo_num++) {
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

  return 0;
}

static int
get_comm_buffers(void)
{
  int retval = 0;

  (void) get_servo_comm_buffers();
  if (! have_servo_comm_buffers()) {
    retval = 1;
  }

  (void) get_traj_comm_buffers();
  if (! have_traj_comm_buffers()) {
    retval = 1;
  }

  /* try for the optional tool and task comm buffers, and ignore
     failure here since we can check with have_xxx_comm_buffers() */
  (void) get_tool_comm_buffers();
  (void) get_task_comm_buffers();

  /* get the log buffer */
  go_log_shm = ulapi_rtm_new(go_log_shm_key, sizeof(go_log_struct));
  if (NULL == go_log_shm) {
    return 1;
  }
  go_log_ptr = ulapi_rtm_addr(go_log_shm);

  /* get the io buffer */
  go_io_shm = ulapi_rtm_new(go_io_shm_key, sizeof(go_io_struct));
  if (NULL == go_io_shm) {
    return 1;
  }
  go_io_ptr = ulapi_rtm_addr(go_io_shm);
  /* set up ping-pong buffers */
  go_input_ptr = &pp_go_input[0];
  go_input_test = &pp_go_input[1];

  return retval;
}

static void
free_comm_buffers(void)
{
  free_traj_comm_buffers();
  free_servo_comm_buffers();

  free_tool_comm_buffers();
  free_task_comm_buffers();

  if (NULL != go_log_shm) {
    ulapi_rtm_delete(go_log_shm);
    go_log_shm = NULL;
  }
  go_log_ptr = NULL;

  if (NULL != go_io_shm) {
    ulapi_rtm_delete(go_io_shm);
    go_io_shm = NULL;
  }
  go_io_ptr = NULL;
}

static int
update_comm_buffers(void)
{
  void *tmp;

  if (NULL == go_log_ptr ||
      NULL == go_io_ptr) return 1;

  LOCK;

  (void) update_servo_comm_buffers();
  (void) update_traj_comm_buffers();
  (void) update_tool_comm_buffers();
  (void) update_task_comm_buffers();

  /* read io input, ping-pong style */
  *go_input_test = go_io_ptr->input;
  if (go_input_test->head == go_input_test->tail) {
    tmp = go_input_ptr;
    go_input_ptr = go_input_test;
    go_input_test = tmp;
  }

  UNLOCK;

  return 0;
}

static int
gotk_update(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  static int wasbad = 0;
  ulapi_result status;
  int retval;

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  (void) update_comm_buffers();

  if (! wasbad) {
    if (NULL != estop_command && 0 != estop_command[0]) {
      status = ulapi_system(estop_command, &retval);
      if (ULAPI_OK != status) {
	wasbad = 1;
      } else if (1 == retval) {
	/* only send these if needed, otherwise there could be a continuous
	   flood of commands while the estop condition is true */
	traj_cmd.type = TRAJ_CMD_ABORT_TYPE;
	LOCK;
	DO_TRAJ_CMD_IF_NEEDED;
	UNLOCK;
	tool_cmd.type = TOOL_CMD_ABORT_TYPE;
	LOCK;
	DO_TOOL_CMD_IF_NEEDED;
	UNLOCK;
      }
    }
  }

  return TCL_OK;
}

static int
gotk_get_csys_select(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  LOCK;
  Tcl_SetResult(interp,
		csys_select == CSYS_JOINT ? "joint" :
		csys_select == CSYS_WORLD ? "world" : "tool",
		TCL_STATIC);
  UNLOCK;

  return TCL_OK;
}

static int
gotk_set_csys_select(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  char * string;
  int retval = TCL_OK;

  if (objc != 2) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  string = Tcl_GetString(objv[1]);

  LOCK;
  if (! strcmp("joint", string)) csys_select = CSYS_JOINT;
  else if (! strcmp("world", string)) csys_select = CSYS_WORLD;
  else if (! strcmp("tool", string)) csys_select = CSYS_TOOL;
  else retval = TCL_ERROR;
  UNLOCK;

  return retval;
}

static int
gotk_get_joint_select(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  Tcl_Obj * resultPtr;

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }

  resultPtr = Tcl_GetObjResult(interp);
  
  /* the script index starts at 1, while Go's index starts at 0 */
  LOCK;
  Tcl_SetIntObj(resultPtr, (int) joint_select + 1);
  UNLOCK;

  return TCL_OK;
}

static int
gotk_set_joint_select(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  int joint;

  if (objc != 2) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetIntFromObj(interp, objv[1], &joint)) {
    return TCL_ERROR;
  }

  /* the script index starts at 1, while Go's index starts at 0 */
  joint--;
  if (joint < 0) joint = 0;
  else if (joint >= SERVO_NUM) joint = SERVO_NUM - 1;
  LOCK;
  joint_select = (go_integer) joint;
  UNLOCK;

  return TCL_OK;
}

static int
gotk_get_cart_select(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  LOCK;
  Tcl_SetResult(interp,
		cart_select == CART_X ? "X" :
		cart_select == CART_Y ? "Y" :
		cart_select == CART_Z ? "Z" :
		cart_select == CART_R ? "R" :
		cart_select == CART_P ? "P" : "W",
		TCL_STATIC);
  UNLOCK;

  return TCL_OK;
}

static int
gotk_set_cart_select(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  char * string;
  int retval = TCL_OK;

  if (objc != 2) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  string = Tcl_GetString(objv[1]);

  LOCK;
  if (! strcmp("X", string)) cart_select = CART_X;
  else if (! strcmp("Y", string)) cart_select = CART_Y;
  else if (! strcmp("Z", string)) cart_select = CART_Z;
  else if (! strcmp("R", string)) cart_select = CART_R;
  else if (! strcmp("P", string)) cart_select = CART_P;
  else if (! strcmp("W", string)) cart_select = CART_W;
  else {
    retval = TCL_ERROR;
  }
  UNLOCK;

  return retval;
}

static int
gotk_get_input_control(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  Tcl_Obj * resultPtr;

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }

  resultPtr = Tcl_GetObjResult(interp);
  
  LOCK;
  Tcl_SetIntObj(resultPtr, (int) (input_control != 0));
  UNLOCK;

  return TCL_OK;
}

static int
gotk_set_input_control(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  int flag;

  if (objc != 2) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetIntFromObj(interp, objv[1], &flag)) {
    return TCL_ERROR;
  }

  LOCK;
  input_control = (go_flag) (flag != 0);
  UNLOCK;

  return TCL_OK;
}

static int
gotk_traj_cmd(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  Tcl_SetResult(interp, traj_cmd_symbol(traj_stat_ptr->command_type), TCL_STATIC);

  return TCL_OK;
}

static int
gotk_traj_admin_state(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  Tcl_SetResult(interp, rcs_admin_state_to_string(traj_stat_ptr->admin_state), TCL_STATIC);

  return TCL_OK;
}

static int
gotk_traj_state(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  Tcl_SetResult(interp, rcs_state_to_string(traj_stat_ptr->state), TCL_STATIC);

  return TCL_OK;
}

static int
gotk_traj_status(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  Tcl_SetResult(interp, rcs_status_to_string(traj_stat_ptr->status), TCL_STATIC);

  return TCL_OK;
}

static int
gotk_traj_heartbeat(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  Tcl_Obj * resultPtr;

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetIntObj(resultPtr, traj_stat_ptr->heartbeat);

  return TCL_OK;
}

static int
gotk_inpos(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  Tcl_Obj * resultPtr;

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetIntObj(resultPtr, traj_stat_ptr->inpos);

  return TCL_OK;
}

static int
gotk_joint_pos(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  int joint;
  Tcl_Obj * resultPtr;

  if (objc != 2) {
    Tcl_WrongNumArgs(interp, 1, objv, "<joint to query>");
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetIntFromObj(interp, objv[1], &joint)) {
    return TCL_ERROR;
  }

  joint--;			/* user has 1..SERVO_NUM, our index 
				   is 0..SERVO_NUM-1 */
  if (joint < 0) joint = 0;
  else if (joint >= SERVO_NUM) joint = SERVO_NUM - 1;

  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetDoubleObj(resultPtr, FGQ(traj_stat_ptr->joints_act[joint], joint));

  return TCL_OK;
}

static int
gotk_joint_cmd_pos(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  int joint;
  Tcl_Obj * resultPtr;

  if (objc != 2) {
    Tcl_WrongNumArgs(interp, 1, objv, "<joint to query>");
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetIntFromObj(interp, objv[1], &joint)) {
    return TCL_ERROR;
  }

  joint--;			/* user has 1..SERVO_NUM, our index 
				   is 0..SERVO_NUM-1 */
  if (joint < 0) joint = 0;
  else if (joint >= SERVO_NUM) joint = SERVO_NUM - 1;

  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetDoubleObj(resultPtr, FGQ(traj_stat_ptr->joints[joint], joint));

  return TCL_OK;
}

static int
gotk_world_homed(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  Tcl_Obj * resultPtr;

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetIntObj(resultPtr, traj_stat_ptr->homed);

  return TCL_OK;
}

static int
gotk_joint_homed(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  int joint;
  Tcl_Obj * resultPtr;

  if (objc != 2) {
    Tcl_WrongNumArgs(interp, 1, objv, "<joint to query>");
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetIntFromObj(interp, objv[1], &joint)) {
    return TCL_ERROR;
  }

  joint--;			/* user has 1..SERVO_NUM, our index 
				   is 0..SERVO_NUM-1 */
  if (joint < 0) joint = 0;
  else if (joint >= SERVO_NUM) joint = SERVO_NUM - 1;

  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetIntObj(resultPtr, (int) servo_stat_ptr[joint]->homed);

  return TCL_OK;
}

static int
gotk_joint_active(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  int joint;
  Tcl_Obj * resultPtr;

  if (objc != 2) {
    Tcl_WrongNumArgs(interp, 1, objv, "<joint to query>");
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetIntFromObj(interp, objv[1], &joint)) {
    return TCL_ERROR;
  }

  joint--;			/* user has 1..SERVO_NUM, our index 
				   is 0..SERVO_NUM-1 */
  if (joint < 0) joint = 0;
  else if (joint >= SERVO_NUM) joint = SERVO_NUM - 1;

  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetIntObj(resultPtr, (int) servo_set_ptr[joint]->active);

  return TCL_OK;
}

static int
gotk_world_pos(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  char * string;
  Tcl_Obj * resultPtr;
  go_pose pose;
  go_rpy rpy;

  if (objc != 2) {
    Tcl_WrongNumArgs(interp, 1, objv, "<coordinate to query>");
    return TCL_ERROR;
  }
  string = Tcl_GetString(objv[1]);

  pose = traj_stat_ptr->ecp_act;
  go_quat_rpy_convert(&pose.rot, &rpy);
  
  if (!strcmp("X", string)) {
    resultPtr = Tcl_GetObjResult(interp);
    Tcl_SetDoubleObj(resultPtr, FGL(pose.tran.x));
    return TCL_OK;
  } else if (!strcmp("Y", string)) {
    resultPtr = Tcl_GetObjResult(interp);
    Tcl_SetDoubleObj(resultPtr, FGL(pose.tran.y));
    return TCL_OK;
  } else if (!strcmp("Z", string)) {
    resultPtr = Tcl_GetObjResult(interp);
    Tcl_SetDoubleObj(resultPtr, FGL(pose.tran.z));
    return TCL_OK;
  } else if (!strcmp("R", string)) {
    resultPtr = Tcl_GetObjResult(interp);
    Tcl_SetDoubleObj(resultPtr, FGA(rpy.r));
    return TCL_OK;
  } else if (!strcmp("P", string)) {
    resultPtr = Tcl_GetObjResult(interp);
    Tcl_SetDoubleObj(resultPtr, FGA(rpy.p));
    return TCL_OK;
  } else if (!strcmp("W", string)) {
    resultPtr = Tcl_GetObjResult(interp);
    Tcl_SetDoubleObj(resultPtr, FGA(rpy.y));
    return TCL_OK;
  } else {
    return TCL_ERROR;
  }
}

static int
gotk_world_cmd_pos(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  char * string;
  Tcl_Obj * resultPtr;
  go_pose pose;
  go_rpy rpy;

  if (objc != 2) {
    Tcl_WrongNumArgs(interp, 1, objv, "<coordinate to query>");
    return TCL_ERROR;
  }
  string = Tcl_GetString(objv[1]);

  pose = traj_stat_ptr->ecp;
  go_quat_rpy_convert(&pose.rot, &rpy);
  
  if (!strcmp("X", string)) {
    resultPtr = Tcl_GetObjResult(interp);
    Tcl_SetDoubleObj(resultPtr, FGL(pose.tran.x));
    return TCL_OK;
  } else if (!strcmp("Y", string)) {
    resultPtr = Tcl_GetObjResult(interp);
    Tcl_SetDoubleObj(resultPtr, FGL(pose.tran.y));
    return TCL_OK;
  } else if (!strcmp("Z", string)) {
    resultPtr = Tcl_GetObjResult(interp);
    Tcl_SetDoubleObj(resultPtr, FGL(pose.tran.z));
    return TCL_OK;
  } else if (!strcmp("R", string)) {
    resultPtr = Tcl_GetObjResult(interp);
    Tcl_SetDoubleObj(resultPtr, FGA(rpy.r));
    return TCL_OK;
  } else if (!strcmp("P", string)) {
    resultPtr = Tcl_GetObjResult(interp);
    Tcl_SetDoubleObj(resultPtr, FGA(rpy.p));
    return TCL_OK;
  } else if (!strcmp("W", string)) {
    resultPtr = Tcl_GetObjResult(interp);
    Tcl_SetDoubleObj(resultPtr, FGA(rpy.y));
    return TCL_OK;
  } else {
    return TCL_ERROR;
  }
}

static int
gotk_joint_profile(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  int joint;
  double val;
  Tcl_Obj * resultPtr;

  if (objc != 2) {
    Tcl_WrongNumArgs(interp, 1, objv, "<joint to query>");
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetIntFromObj(interp, objv[1], &joint)) {
    return TCL_ERROR;
  }

  joint--;			/* user has 1..SERVO_NUM, our index 
				   is 0..SERVO_NUM-1 */
  if (joint < 0) joint = 0;
  else if (joint >= SERVO_NUM) joint = SERVO_NUM - 1;

  resultPtr = Tcl_GetObjResult(interp);
  if (CD1 == clientData) {
    val = FGQ(servo_set_ptr[joint]->max_vel, joint);
  } else if (CD2 == clientData) {
    val = FGQ(servo_set_ptr[joint]->max_acc, joint);
  } else {
    val = FGQ(servo_set_ptr[joint]->max_jerk, joint);
  }
  Tcl_SetDoubleObj(resultPtr, val);

  return TCL_OK;
}

static int
gotk_world_profile(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  double val;
  Tcl_Obj * resultPtr;

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  resultPtr = Tcl_GetObjResult(interp);
  if (CD1 == clientData) {
    val = FGL(traj_set_ptr->max_tvel);
  } else if (CD2 == clientData) {
    val = FGL(traj_set_ptr->max_tacc);
  } else if (CD3 == clientData) {
    val = FGL(traj_set_ptr->max_tjerk);
  } else if (CD4 == clientData) {
    val = FGA(traj_set_ptr->max_rvel);
  } else if (CD5 == clientData) {
    val = FGA(traj_set_ptr->max_racc);
  } else {
    val = FGA(traj_set_ptr->max_rjerk);
  }
  Tcl_SetDoubleObj(resultPtr, val);

  return TCL_OK;
}

/* FIXME-- consider adding gotk_scale,v,a to get the current values */

static int
gotk_max_scale(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  Tcl_Obj * resultPtr;

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetDoubleObj(resultPtr, traj_set_ptr->max_scale);

  return TCL_OK;
}

static int
gotk_max_scale_v(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  Tcl_Obj * resultPtr;

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetDoubleObj(resultPtr, traj_set_ptr->max_scale_v);

  return TCL_OK;
}

static int
gotk_max_scale_a(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  Tcl_Obj * resultPtr;

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetDoubleObj(resultPtr, traj_set_ptr->max_scale_a);

  return TCL_OK;
}

static int traj_wait_done(void)
{
  double end;
  int got_it;

  if (timeout <= 0.0) {
    return GO_RCS_STATUS_DONE;
  }

  for (got_it = 0, end = ulapi_time() + timeout; ulapi_time() < end;) {
    (void) update_traj_comm_buffers();
    if ((traj_comm_ptr->traj_stat.command_type == traj_cmd.type) &&
	(traj_comm_ptr->traj_stat.echo_serial_number == traj_cmd.serial_number)) {
      got_it = 1;
      if (traj_comm_ptr->traj_stat.status != GO_RCS_STATUS_EXEC) {
	return traj_comm_ptr->traj_stat.status;
      }
    } else {
      if (got_it) {
	/* something else is there, and it got ours already, so we
	   must have been overridden by something else */
	return traj_comm_ptr->traj_stat.status;
      }
    }
    /* else keep waiting */
    ulapi_sleep(0.001);
  }

  /* else we timed out */
  return GO_RCS_STATUS_EXEC;
}

static int tool_wait_done(void)
{
  double end;
  int got_it;

  if (timeout <= 0.0) {
    return GO_RCS_STATUS_DONE;
  }

  for (got_it = 0, end = ulapi_time() + timeout; ulapi_time() < end;) {
    (void) update_tool_comm_buffers();
    if ((tool_comm_ptr->tool_stat.command_type == tool_cmd.type) &&
	(tool_comm_ptr->tool_stat.echo_serial_number == tool_cmd.serial_number)) {
      got_it = 1;
      if (tool_comm_ptr->tool_stat.status != GO_RCS_STATUS_EXEC) {
	return tool_comm_ptr->tool_stat.status;
      }
    } else {
      if (got_it) {
	/* something else is there, and it got ours already, so we
	   must have been overridden by something else */
	return tool_comm_ptr->tool_stat.status;
      }
    }
    /* else keep waiting */
    ulapi_sleep(0.001);
  }

  /* else we timed out */
  return GO_RCS_STATUS_EXEC;
}

/* to use this, make sure that timeout is positive, probably large */
static int traj_wait_inpos(void)
{
  double end;
  int got_it;

  for (got_it = 0, end = ulapi_time() + timeout; ulapi_time() < end;) {
    if ((0 == update_comm_buffers()) &&
	(traj_comm_ptr->traj_stat.echo_serial_number ==
	 traj_cmd.serial_number) &&
	(traj_comm_ptr->traj_stat.inpos)) {
      got_it = 1;
      break;
    }
    ulapi_sleep(0.001);
  }

  if (got_it) {
    return traj_comm_ptr->traj_stat.status == GO_RCS_STATUS_ERROR ? GO_RCS_STATUS_ERROR : GO_RCS_STATUS_DONE;
  }
  return GO_RCS_STATUS_EXEC;		/* timed out */
}

static int
gotk_traj_init(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  Tcl_Obj * resultPtr;

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }

  LOCK;
  traj_cmd.type = TRAJ_CMD_INIT_TYPE;
  DO_TRAJ_CMD;
  UNLOCK;

  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetIntObj(resultPtr, traj_wait_done());

  return TCL_OK;
}

static int
gotk_traj_halt(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  Tcl_Obj * resultPtr;

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }

  LOCK;
  traj_cmd.type = TRAJ_CMD_HALT_TYPE;
  DO_TRAJ_CMD;
  UNLOCK;

  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetIntObj(resultPtr, traj_wait_done());

  return TCL_OK;
}

static int
gotk_traj_abort(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  Tcl_Obj * resultPtr;

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }

  LOCK;
  traj_cmd.type = TRAJ_CMD_ABORT_TYPE;
  DO_TRAJ_CMD_IF_NEEDED;
  UNLOCK;

  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetIntObj(resultPtr, traj_wait_done());

  return TCL_OK;
}

static int
gotk_traj_shutdown(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }

  LOCK;
  traj_cmd.type = TRAJ_CMD_SHUTDOWN_TYPE;
  DO_TRAJ_CMD;
  UNLOCK;

  return TCL_OK;
}

static int
gotk_traj_stop(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  Tcl_Obj * resultPtr;

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }

  LOCK;
  traj_cmd.type = TRAJ_CMD_STOP_TYPE;
  DO_TRAJ_CMD;
  UNLOCK;

  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetIntObj(resultPtr, traj_wait_done());

  return TCL_OK;
}

static int
gotk_traj_set_tpar(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  double d1, d2, d3;

  if (objc != 4) {
    Tcl_WrongNumArgs(interp, 1, objv, "<v> <a> <j>");
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[1], &d1)) {
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[2], &d2)) {
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[3], &d3)) {
    return TCL_ERROR;
  }

  if (d1 <= 0.0 || d2 <= 0.0 || d3 <= 0.0) {
    Tcl_SetResult(interp, "args must be positive", TCL_STATIC);
    return TCL_OK;
  }

  local_tvel = TGL(d1);
  local_tacc = TGL(d2);
  local_tjerk = TGL(d3);

  return TCL_OK;
}

static int
gotk_traj_set_rpar(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  double d1, d2, d3;

  if (objc != 4) {
    Tcl_WrongNumArgs(interp, 1, objv, "<v> <a> <j>");
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[1], &d1)) {
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[2], &d2)) {
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[3], &d3)) {
    return TCL_ERROR;
  }

  if (d1 <= 0.0 || d2 <= 0.0 || d3 <= 0.0) {
    Tcl_SetResult(interp, "args must be positive", TCL_STATIC);
    return TCL_OK;
  }

  local_rvel = TGA(d1);
  local_racc = TGA(d2);
  local_rjerk = TGA(d3);

  return TCL_OK;
}

static int
gotk_traj_set_jpar(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  double d1, d2, d3;
  int joint;

  if (objc != 5) {
    Tcl_WrongNumArgs(interp, 1, objv, "<joint #> <v> <a> <j>");
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetIntFromObj(interp, objv[1], &joint)) {
    return TCL_ERROR;
  }

  joint--;
  if (joint < 0) joint = 0;
  else if (joint >= SERVO_NUM) joint = SERVO_NUM - 1;

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[2], &d1)) {
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[3], &d2)) {
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[4], &d3)) {
    return TCL_ERROR;
  }

  if (d1 <= 0.0 || d2 <= 0.0 || d3 <= 0.0) {
    Tcl_SetResult(interp, "args must be positive", TCL_STATIC);
    return TCL_OK;
  }

  local_jvel[joint] = TGQ(d1, joint);
  local_jacc[joint] = TGQ(d2, joint);
  local_jjerk[joint] = TGQ(d3, joint);

  return TCL_OK;
}

static int
gotk_traj_set_move_time(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  double d1;

  if (objc != 2) {
    Tcl_WrongNumArgs(interp, 1, objv, "<time for move>");
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[1], &d1)) {
    return TCL_ERROR;
  }

  local_move_time = (go_real) d1;

  return TCL_OK;
}

static int
gotk_traj_home(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  int joint;
  int servo_num;
  Tcl_Obj * resultPtr;

  if (objc != 2) {
    Tcl_WrongNumArgs(interp, 1, objv, "<joint to home>");
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetIntFromObj(interp, objv[1], &joint)) {
    return TCL_ERROR;
  }

  joint--;			/* user has 1..SERVO_NUM, our index 
				   is 0..SERVO_NUM-1 */
  if (joint < 0) joint = 0;
  else if (joint >= SERVO_NUM) joint = SERVO_NUM - 1;

  LOCK;

  /* set the destination for all joints to be their current
     absolute position, since we're doing a UJOINT motion */
  for (servo_num = 0; servo_num < SERVO_NUM; servo_num++) {
    traj_cmd.u.move_ujoint.d[servo_num] = servo_stat_ptr[servo_num]->input;
    /* we'll override the max vel with the homing vel if necessary */
    traj_cmd.u.move_ujoint.v[servo_num] = servo_set_ptr[servo_num]->max_vel;
    traj_cmd.u.move_ujoint.a[servo_num] = servo_set_ptr[servo_num]->max_acc;
    traj_cmd.u.move_ujoint.j[servo_num] = servo_set_ptr[servo_num]->max_jerk;
    traj_cmd.u.move_ujoint.home[servo_num] = 0;
  }

  /*
    there are three directions to home: negative, positive and stay here
   */
  traj_cmd.type = TRAJ_CMD_MOVE_UJOINT_TYPE;

  if (home_vel[joint] > GO_REAL_EPSILON) {
    /* home in the positive direction */
    traj_cmd.u.move_ujoint.v[joint] = home_vel[joint];
    traj_cmd.u.move_ujoint.d[joint] =
      servo_stat_ptr[joint]->input +
      (servo_set_ptr[joint]->max_limit -
       servo_set_ptr[joint]->min_limit + 1.0);
  } else  if (home_vel[joint] < -GO_REAL_EPSILON) {
    /* home in the negative direction */
    traj_cmd.u.move_ujoint.v[joint] = -home_vel[joint];	/* needs to be pos */
    traj_cmd.u.move_ujoint.d[joint] = 
      servo_stat_ptr[joint]->input -
      (servo_set_ptr[joint]->max_limit -
       servo_set_ptr[joint]->min_limit + 1.0);
  } else {
    /* else stay here, leave v at vmax */
    traj_cmd.u.move_ujoint.d[joint] = 
      servo_stat_ptr[joint]->input;
  }
  
  traj_cmd.u.move_ujoint.home[joint] = 1;
  traj_cmd.u.move_ujoint.id = traj_cmd.serial_number + 1;
  DO_TRAJ_CMD;

  UNLOCK;

  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetIntObj(resultPtr, traj_wait_inpos());

  return TCL_OK;
}

static int
gotk_traj_hold(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  int servo_num;
  Tcl_Obj * resultPtr;

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }

  /* For a ujoint motion, the current position is the abs position,
     i.e., servo_stat.input. The v,a,j can be the max since we're
     not moving anywhere, just holding position here. */

  LOCK;

  for (servo_num = 0; servo_num < SERVO_NUM; servo_num++) {
    traj_cmd.u.move_ujoint.d[servo_num] = servo_stat_ptr[servo_num]->input;
    traj_cmd.u.move_ujoint.v[servo_num] = servo_set_ptr[servo_num]->max_vel;
    traj_cmd.u.move_ujoint.a[servo_num] = servo_set_ptr[servo_num]->max_acc;
    traj_cmd.u.move_ujoint.j[servo_num] = servo_set_ptr[servo_num]->max_jerk;
    traj_cmd.u.move_ujoint.home[servo_num] = 0;
  }
  traj_cmd.type = TRAJ_CMD_MOVE_UJOINT_TYPE;
  traj_cmd.u.move_ujoint.id = traj_cmd.serial_number + 1;
  DO_TRAJ_CMD;

  UNLOCK;

  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetIntObj(resultPtr, traj_wait_inpos());

  return TCL_OK;
}

static int
gotk_traj_jog_joint(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  double d1;
  int joint;
  int servo_num;
  Tcl_Obj * resultPtr;
  go_real vel;

  if (objc != 3) {
    Tcl_WrongNumArgs(interp, 1, objv, "<joint to jog> <+/- speed>");
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetIntFromObj(interp, objv[1], &joint)) {
    return TCL_ERROR;
  }
  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[2], &d1)) {
    return TCL_ERROR;
  }

  joint--;
  if (joint < 0) joint = 0;
  else if (joint >= SERVO_NUM) joint = SERVO_NUM - 1;

  vel = TGQ(d1, joint);

  LOCK;

  if (servo_stat_ptr[joint]->homed) {
    /* If the joint is homed, we'll do a coordinated joint move
       with the other joint destinations set to their current relative
       positions. Even if these other joints are not homed, they
       will remain stationary. */

    /* set the destination for all joints to be their current
       relative position, since we're doing a JOINT motion */
    for (servo_num = 0; servo_num < SERVO_NUM; servo_num++) {
      traj_cmd.u.move_joint.d[servo_num] = traj_stat_ptr->joints_act[servo_num];
      traj_cmd.u.move_joint.v[servo_num] = servo_set_ptr[servo_num]->max_vel;
      traj_cmd.u.move_joint.a[servo_num] = servo_set_ptr[servo_num]->max_acc;
      traj_cmd.u.move_joint.j[servo_num] = servo_set_ptr[servo_num]->max_jerk;
    }

    /* now set the jogging joint's vel, and its destination to be the
       min or max limit */
    if (vel < 0) {
      traj_cmd.u.move_joint.d[joint] = servo_set_ptr[joint]->min_limit;
      traj_cmd.u.move_joint.v[joint] = -vel;
    } else {
      traj_cmd.u.move_joint.d[joint] = servo_set_ptr[joint]->max_limit;
      traj_cmd.u.move_joint.v[joint] = vel;
    }
    traj_cmd.type = TRAJ_CMD_MOVE_JOINT_TYPE;
    /* set id to match the serial number of what will go out */
    traj_cmd.u.move_joint.id = traj_cmd.serial_number + 1;
    DO_TRAJ_CMD;

  } else {
    /* If the joint is not homed, we'll do an uncoordinated joint move
       to more than the joint's max range, with the other joint
       destinations set to their current absolute positions. They will
       remain stationary regardless of whether or not they are
       homed. The jogged joint won't stop automatically at any limit,
       so the operator must be careful. */

    /* set the destination for all joints to be their current
       absolute position, since we're doing a UJOINT motion */
    for (servo_num = 0; servo_num < SERVO_NUM; servo_num++) {
      traj_cmd.u.move_ujoint.d[servo_num] = servo_stat_ptr[servo_num]->input;
      traj_cmd.u.move_ujoint.v[servo_num] = servo_set_ptr[servo_num]->max_vel;
      traj_cmd.u.move_ujoint.a[servo_num] = servo_set_ptr[servo_num]->max_acc;
      traj_cmd.u.move_ujoint.j[servo_num] = servo_set_ptr[servo_num]->max_jerk;
      traj_cmd.u.move_ujoint.home[servo_num] = 0;
    }

    /* now set the speed, and add +/- the joint range of motion
       to the jogging joint for its destination */
    if (vel < 0) {
      traj_cmd.u.move_ujoint.d[joint] -=
	(servo_set_ptr[joint]->max_limit -
	 servo_set_ptr[joint]->min_limit + 1);
      traj_cmd.u.move_joint.v[joint] = -vel;
    } else {
      traj_cmd.u.move_ujoint.d[joint] +=
	(servo_set_ptr[joint]->max_limit -
	 servo_set_ptr[joint]->min_limit + 1);
      traj_cmd.u.move_joint.v[joint] = vel;
    }
    traj_cmd.type = TRAJ_CMD_MOVE_UJOINT_TYPE;
    /* set id to match the serial number of what will go out */
    traj_cmd.u.move_ujoint.id = traj_cmd.serial_number + 1;
    DO_TRAJ_CMD;
  }

  UNLOCK;

  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetIntObj(resultPtr, traj_wait_inpos());

  return TCL_OK;
}

static int
gotk_traj_move_ujoint(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  int joint;
  double d1;
  Tcl_Obj * resultPtr;

  if (objc != SERVO_NUM + 1) {
    Tcl_WrongNumArgs(interp, 1, objv, "<target joint values>");
    return TCL_ERROR;
  }

  resultPtr = Tcl_GetObjResult(interp);

  LOCK;

  for (joint = 0; joint < SERVO_NUM; joint++) {
    if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[joint + 1], &d1)) {
      return TCL_ERROR;
    }
    traj_cmd.u.move_ujoint.d[joint] = TGQ(d1, joint);
    traj_cmd.u.move_ujoint.v[joint] = servo_set_ptr[joint]->max_vel;
    traj_cmd.u.move_ujoint.a[joint] = servo_set_ptr[joint]->max_acc;
    traj_cmd.u.move_ujoint.j[joint] = servo_set_ptr[joint]->max_jerk;
  }

  traj_cmd.type = TRAJ_CMD_MOVE_UJOINT_TYPE;
  /* set id to match the serial number of what will go out */
  traj_cmd.u.move_ujoint.id = traj_cmd.serial_number + 1;
  DO_TRAJ_CMD;

  UNLOCK;

  Tcl_SetIntObj(resultPtr, traj_wait_inpos());

  return TCL_OK;
}

static int
gotk_traj_move_joint(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  int joint;
  double d1;
  Tcl_Obj * resultPtr;

  if (objc != SERVO_NUM + 1) {
    Tcl_WrongNumArgs(interp, 1, objv, "<target joint values>");
    return TCL_ERROR;
  }

  resultPtr = Tcl_GetObjResult(interp);

  /* We're assuming joints are homed, and the args are relative
     to home position. */
  if (! traj_stat_ptr->homed) {
    Tcl_SetIntObj(resultPtr, GO_RCS_STATUS_ERROR);
  }

  LOCK;

  for (joint = 0; joint < SERVO_NUM; joint++) {
    if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[joint + 1], &d1)) {
      return TCL_ERROR;
    }
    traj_cmd.u.move_joint.d[joint] = TGQ(d1, joint);
    traj_cmd.u.move_joint.v[joint] = servo_set_ptr[joint]->max_vel;
    traj_cmd.u.move_joint.a[joint] = servo_set_ptr[joint]->max_acc;
    traj_cmd.u.move_joint.j[joint] = servo_set_ptr[joint]->max_jerk;
  }

  traj_cmd.type = TRAJ_CMD_MOVE_JOINT_TYPE;
  /* set id to match the serial number of what will go out */
  traj_cmd.u.move_joint.id = traj_cmd.serial_number + 1;
  DO_TRAJ_CMD;

  UNLOCK;

  Tcl_SetIntObj(resultPtr, traj_wait_inpos());

  return TCL_OK;
}

static int
gotk_traj_jog_world_or_tool(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  char * string;
  double d1;
  go_pose pose;
  go_rpy rpy_now, rpy_min, rpy_max;
  go_real vel, tvel, rvel;
  Tcl_Obj * resultPtr;

  if (objc != 3) {
    Tcl_WrongNumArgs(interp, 1, objv, "<one of XYZRPW> <+/- speed>");
    return TCL_ERROR;
  }
  string = Tcl_GetString(objv[1]);

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[2], &d1)) {
    return TCL_ERROR;
  }

  /* FIXME-- this does world jogging. To do tool jogging, we need
     to transform the limits into tool space, meaning find the
     distances from the rotated jog direction to each of the
     limit planes, and take the minimum, filling in zero for the
     other directions. */

  /* fill in the target pose with the current pose */
  pose = traj_stat_ptr->ecp_act;
  /* get the current orientation and limits in terms of RPY */
  go_quat_rpy_convert(&pose.rot, &rpy_now);
  go_quat_rpy_convert(&traj_set_ptr->min_limit.rot, &rpy_min);
  go_quat_rpy_convert(&traj_set_ptr->max_limit.rot, &rpy_max);
  /* set t,rvel to their max in case they aren't set below */
  tvel = (go_real) traj_set_ptr->max_tvel;
  rvel = (go_real) traj_set_ptr->max_rvel;

  if (!strcmp("X", string)) {
    vel = TGL(d1);
    if (vel < 0) {
      tvel = -vel;
      pose.tran.x = traj_set_ptr->min_limit.tran.x;
    } else {
      tvel = vel;
      pose.tran.x = traj_set_ptr->max_limit.tran.x;
    }
  } else if (!strcmp("Y", string)) {
    vel = TGL(d1);
    if (vel < 0) {
      tvel = -vel;
      pose.tran.y = traj_set_ptr->min_limit.tran.y;
    } else {
      tvel = vel;
      pose.tran.y = traj_set_ptr->max_limit.tran.y;
    }
  } else if (!strcmp("Z", string)) {
    vel = TGL(d1);
    if (vel < 0) {
      tvel = -vel;
      pose.tran.z = traj_set_ptr->min_limit.tran.z;
    } else {
      tvel = vel;
      pose.tran.z = traj_set_ptr->max_limit.tran.z;
    }
  } else if (!strcmp("R", string)) {
    vel = TGA(d1);
    if (vel < 0) {
      rvel = -vel;
      rpy_now.r = rpy_min.r;
    } else {
      rvel = vel;
      rpy_now.r = rpy_max.r;
    }
    go_rpy_quat_convert(&rpy_now, &pose.rot);
  } else if (!strcmp("P", string)) {
    vel = TGA(d1);
    if (vel < 0) {
      rvel = -vel;
      rpy_now.p = rpy_min.p;
    } else {
      rvel = vel;
      rpy_now.p = rpy_max.p;
    }
    go_rpy_quat_convert(&rpy_now, &pose.rot);
  } else if (!strcmp("W", string)) {
    vel = TGA(d1);
    if (vel < 0) {
      rvel = -vel;
      rpy_now.y = rpy_min.y;
    } else {
      rvel = vel;
      rpy_now.y = rpy_max.y;
    }
    go_rpy_quat_convert(&rpy_now, &pose.rot);
  } else {
    return TCL_ERROR;
  }

  /* clamp the speeds, which have already been forced positive */
  if (tvel > traj_set_ptr->max_tvel) tvel = traj_set_ptr->max_tvel;
  if (rvel > traj_set_ptr->max_rvel) rvel = traj_set_ptr->max_rvel;

  LOCK;

  traj_cmd.type = TRAJ_CMD_MOVE_WORLD_TYPE;
  /* set id to match the serial number of what will go out */
  traj_cmd.u.move_world.id = traj_cmd.serial_number + 1;
  traj_cmd.u.move_world.type = GO_MOTION_LINEAR;
  traj_cmd.u.move_world.end = pose;
  traj_cmd.u.move_world.tv = tvel;
  traj_cmd.u.move_world.ta = local_tacc;
  traj_cmd.u.move_world.tj = local_tjerk;
  traj_cmd.u.move_world.rv = rvel;
  traj_cmd.u.move_world.ra = local_racc;
  traj_cmd.u.move_world.rj = local_rjerk;
  traj_cmd.u.move_world.time = 0.0;
  DO_TRAJ_CMD;

  UNLOCK;

  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetIntObj(resultPtr, traj_wait_inpos());

  return TCL_OK;
}

static int
gotk_traj_move_world_or_tool(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  double d1;
  int i1;
  int is_circular;
  go_pose pose;
  go_rpy rpy;
  go_cart center;
  go_cart normal;
  go_integer turns;
  Tcl_Obj * resultPtr;

  /*
    We'll have one of these two arg lists, given linear/circular:

    7 args, proc end-x y z r p w
    14 args, proc end-x y z r p w center-x y z normal-x y z turns
  */

  if (objc != 7 && objc != 14) {
    Tcl_WrongNumArgs(interp, 1, objv, "<X Y Z R P W>");
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[1], &d1)) {
    return TCL_ERROR;
  }
  pose.tran.x = TGL(d1);
  
  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[2], &d1)) {
    return TCL_ERROR;
  }
  pose.tran.y = TGL(d1);
  
  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[3], &d1)) {
    return TCL_ERROR;
  }
  pose.tran.z = TGL(d1);
  
  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[4], &d1)) {
    return TCL_ERROR;
  }
  rpy.r = TGA(d1);
  
  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[5], &d1)) {
    return TCL_ERROR;
  }
  rpy.p = TGA(d1);
  
  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[6], &d1)) {
    return TCL_ERROR;
  }
  rpy.y = TGA(d1);

  go_rpy_quat_convert(&rpy, &pose.rot);

  if (objc > 7) {
    is_circular = 1;

    if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[7], &d1)) {
      return TCL_ERROR;
    }
    center.x = TGL(d1);
  
    if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[8], &d1)) {
      return TCL_ERROR;
    }
    center.y = TGL(d1);
  
    if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[9], &d1)) {
      return TCL_ERROR;
    }
    center.z = TGL(d1);

    if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[10], &d1)) {
      return TCL_ERROR;
    }
    normal.x = TGL(d1);
  
    if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[11], &d1)) {
      return TCL_ERROR;
    }
    normal.y = TGL(d1);
  
    if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[12], &d1)) {
      return TCL_ERROR;
    }
    normal.z = TGL(d1);
  
    if (TCL_OK != Tcl_GetIntFromObj(interp, objv[13], &i1)) {
      return TCL_ERROR;
    }
    turns = (go_integer) i1;

  } else {
    is_circular = 0;
  }

  LOCK;

  if (CD1 == clientData) {
    traj_cmd.type = TRAJ_CMD_MOVE_WORLD_TYPE;
    traj_cmd.u.move_world.id = traj_cmd.serial_number + 1;
    traj_cmd.u.move_world.end = pose;
    if (is_circular) {
      traj_cmd.u.move_world.type = GO_MOTION_CIRCULAR;
      traj_cmd.u.move_world.center = center;
      traj_cmd.u.move_world.normal = normal;
      traj_cmd.u.move_world.turns = turns;
    } else {
      traj_cmd.u.move_world.type = GO_MOTION_LINEAR;
    }
    if (local_move_time > GO_REAL_EPSILON) {
      /* set the pars to be their max, and the non-zero time
	 tells the traj controller to scale the move */
      traj_cmd.u.move_world.tv = traj_set_ptr->max_tvel;
      traj_cmd.u.move_world.ta = traj_set_ptr->max_tacc;
      traj_cmd.u.move_world.tj = traj_set_ptr->max_tjerk;
      traj_cmd.u.move_world.rv = traj_set_ptr->max_rvel;
      traj_cmd.u.move_world.ra = traj_set_ptr->max_racc;
      traj_cmd.u.move_world.rj = traj_set_ptr->max_rjerk;
      traj_cmd.u.move_world.time = local_move_time;
    } else {
      /* set the pars to be their local values, and the non-positive
	 time tells the traj controller to max the move */
      traj_cmd.u.move_world.tv = local_tvel;
      traj_cmd.u.move_world.ta = local_tacc;
      traj_cmd.u.move_world.tj = local_tjerk;
      traj_cmd.u.move_world.rv = local_rvel;
      traj_cmd.u.move_world.ra = local_racc;
      traj_cmd.u.move_world.rj = local_rjerk;
      traj_cmd.u.move_world.time = -1.0;
    }
  } else {
    traj_cmd.type = TRAJ_CMD_MOVE_TOOL_TYPE;
    traj_cmd.u.move_tool.id = traj_cmd.serial_number + 1;
    traj_cmd.u.move_tool.end = pose;
    if (is_circular) {
      traj_cmd.u.move_tool.type = GO_MOTION_CIRCULAR;
      traj_cmd.u.move_tool.center = center;
      traj_cmd.u.move_tool.normal = normal;
      traj_cmd.u.move_tool.turns = turns;
    } else {
      traj_cmd.u.move_tool.type = GO_MOTION_LINEAR;
    }
    if (local_move_time > GO_REAL_EPSILON) {
      /* set the pars to be their max, and the non-zero time
	 tells the traj controller to scale the move */
      traj_cmd.u.move_tool.tv = traj_set_ptr->max_tvel;
      traj_cmd.u.move_tool.ta = traj_set_ptr->max_tacc;
      traj_cmd.u.move_tool.tj = traj_set_ptr->max_tjerk;
      traj_cmd.u.move_tool.rv = traj_set_ptr->max_rvel;
      traj_cmd.u.move_tool.ra = traj_set_ptr->max_racc;
      traj_cmd.u.move_tool.rj = traj_set_ptr->max_rjerk;
      traj_cmd.u.move_tool.time = local_move_time;
    } else {
      /* set the pars to be their local values, and the non-positive
	 time tells the traj controller to max the move */
      traj_cmd.u.move_tool.tv = local_tvel;
      traj_cmd.u.move_tool.ta = local_tacc;
      traj_cmd.u.move_tool.tj = local_tjerk;
      traj_cmd.u.move_tool.rv = local_rvel;
      traj_cmd.u.move_tool.ra = local_racc;
      traj_cmd.u.move_tool.rj = local_rjerk;
      traj_cmd.u.move_tool.time = -1.0;
    }
  }
  DO_TRAJ_CMD;

  UNLOCK;

  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetIntObj(resultPtr, traj_wait_inpos());

  return TCL_OK;
}

static int
gotk_traj_here(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  go_pose pose;
  go_rpy rpy;
  double d1;

  if (objc != 7) {
    Tcl_WrongNumArgs(interp, 1, objv, "<X Y Z R P W>");
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[1], &d1)) {
    return TCL_ERROR;
  }
  pose.tran.x = TGL(d1);
  
  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[2], &d1)) {
    return TCL_ERROR;
  }
  pose.tran.y = TGL(d1);
  
  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[3], &d1)) {
    return TCL_ERROR;
  }
  pose.tran.z = TGL(d1);
  
  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[4], &d1)) {
    return TCL_ERROR;
  }
  rpy.r = TGA(d1);
  
  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[5], &d1)) {
    return TCL_ERROR;
  }
  rpy.p = TGA(d1);
  
  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[6], &d1)) {
    return TCL_ERROR;
  }
  rpy.y = TGA(d1);

  go_rpy_quat_convert(&rpy, &pose.rot);

  LOCK;
  traj_cmd.type = TRAJ_CMD_HERE_TYPE;
  traj_cmd.u.here.here = pose;
  DO_TRAJ_CMD;
  UNLOCK;

  return TCL_OK;
}

static int
gotk_traj_cfg_scale(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  double d1, d2, d3;

  if (objc != 4) {
    Tcl_WrongNumArgs(interp, 1, objv, "<scale factor> <v> <a>");
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[1], &d1)) {
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[2], &d2)) {
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[3], &d3)) {
    return TCL_ERROR;
  }

  LOCK;
  traj_cfg.type = TRAJ_CFG_SCALE_TYPE;
  /* set id to match the serial number of what will go out */
  traj_cfg.u.scale.scale = (go_real) d1;
  traj_cfg.u.scale.scale_v = (go_real) d2;
  traj_cfg.u.scale.scale_a = (go_real) d3;
  DO_TRAJ_CFG;
  UNLOCK;

  return TCL_OK;
}

static int
gotk_traj_cfg_home(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  double d1, d2, d3, d4, d5, d6;
  go_rpy rpy;

  if (objc != 7) {
    Tcl_WrongNumArgs(interp, 1, objv, "<x> <y> <z> <r> <p> <w>");
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[1], &d1)) {
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[2], &d2)) {
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[3], &d3)) {
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[4], &d4)) {
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[5], &d5)) {
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[6], &d6)) {
    return TCL_ERROR;
  }

  LOCK;
  traj_cfg.type = TRAJ_CFG_HOME_TYPE;
  traj_cfg.u.home.home.tran.x = TGL(d1);
  traj_cfg.u.home.home.tran.y = TGL(d2);
  traj_cfg.u.home.home.tran.z = TGL(d3);
  rpy.r = TGA(d4);
  rpy.p = TGA(d5);
  rpy.y = TGA(d6);
  go_rpy_quat_convert(&rpy, &traj_cfg.u.home.home.rot);
  DO_TRAJ_CFG;
  UNLOCK;

  return TCL_OK;
}

static int
gotk_traj_cfg_tool_transform(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  double d1, d2, d3, d4, d5, d6;
  go_rpy rpy;

  if (objc != 7) {
    Tcl_WrongNumArgs(interp, 1, objv, "<x> <y> <z> <r> <p> <w>");
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[1], &d1)) {
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[2], &d2)) {
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[3], &d3)) {
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[4], &d4)) {
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[5], &d5)) {
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[6], &d6)) {
    return TCL_ERROR;
  }

  LOCK;
  traj_cfg.type = TRAJ_CFG_TOOL_TRANSFORM_TYPE;
  traj_cfg.u.tool_transform.tool_transform.tran.x = TGL(d1);
  traj_cfg.u.tool_transform.tool_transform.tran.y = TGL(d2);
  traj_cfg.u.tool_transform.tool_transform.tran.z = TGL(d3);
  rpy.r = TGA(d4);
  rpy.p = TGA(d5);
  rpy.y = TGA(d6);
  go_rpy_quat_convert(&rpy, &traj_cfg.u.tool_transform.tool_transform.rot);
  DO_TRAJ_CFG;
  UNLOCK;

  return TCL_OK;
}

static int
gotk_traj_get_tool_transform(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  go_rpy rpy;
  enum {STRLEN = 256};
  char str[STRLEN];

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }

  (void) go_quat_rpy_convert(&traj_set_ptr->tool_transform.rot, &rpy);

  /* "%.4g" prints something as large as -1.235e-112, or 4 chars for the
     precision plus up to 7 other chars. Each "%.4g " counts as 12 chars.
     This helps us against buffer overflow. 
     Also, convert some of these from Go units to user units */
  sprintf(str, "%.4g %.4g %.4g %.4g %.4g %.4g", 
	  FGL(traj_set_ptr->tool_transform.tran.x),
	  FGL(traj_set_ptr->tool_transform.tran.y),
	  FGL(traj_set_ptr->tool_transform.tran.z),
	  FGA(rpy.r), FGA(rpy.p), FGA(rpy.y));

  Tcl_SetResult(interp, str, TCL_VOLATILE);

  return TCL_OK;
}

static int
gotk_tool_cmd(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  Tcl_SetResult(interp, tool_cmd_symbol(tool_stat_ptr->command_type), TCL_STATIC);

  return TCL_OK;
}

static int
gotk_tool_admin_state(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  Tcl_SetResult(interp, rcs_admin_state_to_string(tool_stat_ptr->admin_state), TCL_STATIC);

  return TCL_OK;
}

static int
gotk_tool_state(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  Tcl_SetResult(interp, rcs_state_to_string(tool_stat_ptr->state), TCL_STATIC);

  return TCL_OK;
}

static int
gotk_tool_status(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  Tcl_SetResult(interp, rcs_status_to_string(tool_stat_ptr->status), TCL_STATIC);

  return TCL_OK;
}

static int
gotk_tool_heartbeat(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  Tcl_Obj * resultPtr;

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetIntObj(resultPtr, tool_stat_ptr->heartbeat);

  return TCL_OK;
}

static int
gotk_tool_init(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  Tcl_Obj * resultPtr;

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }

  LOCK;
  tool_cmd.type = TOOL_CMD_INIT_TYPE;
  DO_TOOL_CMD;
  UNLOCK;

  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetIntObj(resultPtr, tool_wait_done());

  return TCL_OK;
}

static int
gotk_tool_abort(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  Tcl_Obj * resultPtr;

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }

  LOCK;
  tool_cmd.type = TOOL_CMD_ABORT_TYPE;
  DO_TOOL_CMD_IF_NEEDED;
  UNLOCK;

  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetIntObj(resultPtr, tool_wait_done());

  return TCL_OK;
}

static int
gotk_tool_shutdown(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }

  LOCK;
  tool_cmd.type = TOOL_CMD_SHUTDOWN_TYPE;
  DO_TOOL_CMD;
  UNLOCK;

  return TCL_OK;
}

static int
gotk_tool_on(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  Tcl_Obj * resultPtr;
  double d1;

  if (objc != 2) {
    Tcl_WrongNumArgs(interp, 1, objv, "<tool output value>");
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[1], &d1)) {
    return TCL_ERROR;
  }

  LOCK;
  tool_cmd.type = TOOL_CMD_ON_TYPE;
  tool_cmd.u.on.value = d1;
  DO_TOOL_CMD;
  UNLOCK;

  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetIntObj(resultPtr, tool_wait_done());

  return TCL_OK;
}

static int
gotk_tool_off(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }

  LOCK;
  tool_cmd.type = TOOL_CMD_OFF_TYPE;
  DO_TOOL_CMD;
  UNLOCK;

  return TCL_OK;
}

/*
  Returns the maximum number of servo controllers (aka joints), typically
  used as an upper limit on 'for' loops in scripts.
*/
static int
gotk_servo_num(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  Tcl_Obj *resultPtr;

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }

  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetIntObj(resultPtr, SERVO_NUM);

  return TCL_OK;
}

static int
gotk_servo_get_pid(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  enum {STRLEN = 256};
  int servo_num;
  char str[STRLEN];

  if (objc != 2) {
    Tcl_WrongNumArgs(interp, 1, objv, "<joint servo to query>");
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetIntFromObj(interp, objv[1], &servo_num)) {
    return TCL_ERROR;
  }

  servo_num--;			/* user has 1..SERVO_NUM, our index 
				   is 0..SERVO_NUM-1 */
  if (servo_num < 0) servo_num = 0;
  else if (servo_num >= SERVO_NUM) servo_num = SERVO_NUM - 1;

  sprintf(str, "%.4g %.4g %.4g %.4g %.4g %.4g %.4g %.4g %.4g %.4g", 
	  FGQ(servo_set_ptr[servo_num]->pid.p, servo_num),
	  FGQ(servo_set_ptr[servo_num]->pid.i, servo_num),
	  FGQ(servo_set_ptr[servo_num]->pid.d, servo_num),
	  FGQ(servo_set_ptr[servo_num]->pid.vff, servo_num),
	  FGQ(servo_set_ptr[servo_num]->pid.aff, servo_num),
	  (double) servo_set_ptr[servo_num]->pid.min_output,
	  (double) servo_set_ptr[servo_num]->pid.max_output,
	  (double) servo_set_ptr[servo_num]->pid.pos_bias,
	  (double) servo_set_ptr[servo_num]->pid.neg_bias,
	  FGQ(servo_set_ptr[servo_num]->pid.deadband, servo_num));

  Tcl_SetResult(interp, str, TCL_VOLATILE);

  return TCL_OK;
}

static int
gotk_servo_set_pid(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  int servo_num;
  double p, i, d, vff, aff, min_output, max_output, pos_bias, neg_bias, deadband;
  pid_struct pid;

  if (objc != 12) {
    Tcl_WrongNumArgs(interp, 1, objv, "<joint servo> <11 gains>");
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetIntFromObj(interp, objv[1], &servo_num)) {
    return TCL_ERROR;
  }
  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[2], &p)) {
    return TCL_ERROR;
  }
  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[3], &i)) {
    return TCL_ERROR;
  }
  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[4], &d)) {
    return TCL_ERROR;
  }
  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[5], &vff)) {
    return TCL_ERROR;
  }
  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[6], &aff)) {
    return TCL_ERROR;
  }
  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[7], &min_output)) {
    return TCL_ERROR;
  }
  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[8], &max_output)) {
    return TCL_ERROR;
  }
  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[9], &pos_bias)) {
    return TCL_ERROR;
  }
  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[10], &neg_bias)) {
    return TCL_ERROR;
  }
  if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[11], &deadband)) {
    return TCL_ERROR;
  }

  servo_num--;
  if (servo_num < 0) servo_num = 0;
  else if (servo_num >= SERVO_NUM) servo_num = SERVO_NUM - 1;

  /* read out the current settings */
  pid = servo_set_ptr[servo_num]->pid;
  /* overwrite with our args */
  pid.p = TGQ(p, servo_num);
  pid.i = TGQ(i, servo_num);
  pid.d = TGQ(d, servo_num);
  pid.vff = TGQ(vff, servo_num);
  pid.aff = TGQ(aff, servo_num);
  pid.min_output = (double) min_output;
  pid.max_output = (double) max_output;
  pid.pos_bias = (double) pos_bias;
  pid.neg_bias = (double) neg_bias;
  pid.deadband = TGQ(deadband, servo_num);

  /* send them */
  LOCK;
  servo_cfg[servo_num].type = SERVO_CFG_PID_TYPE;
  servo_cfg[servo_num].u.pid = pid;
  DO_SERVO_CFG(servo_num);
  UNLOCK;

  return TCL_OK;
}

static int
gotk_ini(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  FILE * fp;
  char * variable;
  char * section;
  char * inifile;
  char * inistring;

  if (objc != 4) {
    Tcl_WrongNumArgs(interp, 1, objv, "<variable> <section> <inifile>");
    return TCL_ERROR;
  }

  variable = Tcl_GetString(objv[1]);
  section = Tcl_GetString(objv[2]);
  inifile = Tcl_GetString(objv[3]);

  if (NULL == (fp = fopen(inifile, "r"))) {
    return TCL_OK;		/* and the empty string */
  }

  inistring = (char *) ini_find(fp, variable, section);
  
  if (NULL != inistring) {
    Tcl_SetResult(interp, inistring, TCL_VOLATILE);
  }
  
  return TCL_OK;
}

static int
gotk_log_init(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  int servo_num;
  char * str;
  int i1;
  go_integer log_type;
  go_integer log_size;

  if (objc < 3 || objc > 4) {
    Tcl_WrongNumArgs(interp, 1, objv, "<log type> <log size> {<which>}");
    return TCL_ERROR;
  }

  str = Tcl_GetString(objv[1]);
  if (!strcmp(str, "Ferror")) {
    log_type = GO_LOG_FERROR;
  } else if (!strcmp(str, "Input")) {
    log_type = GO_LOG_INPUT;
  } else if (!strcmp(str, "ActPos")) {
    log_type = GO_LOG_ACT_POS;
  } else if (!strcmp(str, "CmdPos")) {
    log_type = GO_LOG_CMD_POS;
  } else if (!strcmp(str, "Setpoint")) {
    log_type = GO_LOG_SETPOINT;
  } else if (!strcmp(str, "Speed")) {
    log_type = GO_LOG_SPEED;
  } else if (!strcmp(str, "Xinv")) {
    log_type = GO_LOG_XINV;
  } else if (!strcmp(str, "MagXinv")) {
    log_type = GO_LOG_MAGXINV;
  } else {
    log_type = GO_LOG_NONE;
  }

  if (TCL_OK != Tcl_GetIntFromObj(interp, objv[2], &i1)) {
    return TCL_ERROR;
  }
  log_size = (go_integer) i1;

  switch (log_type) {
  case GO_LOG_ACT_POS:
  case GO_LOG_CMD_POS:
  case GO_LOG_XINV:
  case GO_LOG_MAGXINV:
    if (objc != 3) {
      Tcl_WrongNumArgs(interp, 1, objv, "<log type> <log size>");
      return TCL_ERROR;
    }
    LOCK;
    traj_cfg.type = TRAJ_CFG_LOG_TYPE;
    traj_cfg.u.log.log_type = log_type;
    /* no 'which' field applies */
    traj_cfg.u.log.log_size = log_size;
    DO_TRAJ_CFG;
    UNLOCK;
    break;

  case GO_LOG_FERROR:
  case GO_LOG_INPUT:
  case GO_LOG_SETPOINT:
  case GO_LOG_SPEED:
    if (objc != 4) {
      Tcl_WrongNumArgs(interp, 1, objv, "<log type> <log size> <joint>");
      return TCL_ERROR;
    }
    if (TCL_OK != Tcl_GetIntFromObj(interp, objv[3], &i1)) {
      return TCL_ERROR;
    }
    servo_num = i1 - 1;		/* script index starts at 1, C index at 0 */
    if (servo_num < 0) servo_num = 0;
    else if (servo_num >= SERVO_NUM) servo_num = SERVO_NUM - 1;

    LOCK;
    servo_cfg[servo_num].type = SERVO_CFG_LOG_TYPE;
    servo_cfg[servo_num].u.log.log_type = log_type;
    servo_cfg[servo_num].u.log.log_size = log_size;
    /* servo doesn't have a log_which setting, it's implicit */
    DO_SERVO_CFG(servo_num);
    UNLOCK;
    break;

  default:
    break;
  }

  return TCL_OK;
}

static int
gotk_log_start(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  int servo_num;
  int i1;

  if (objc == 1) {
    LOCK;
    traj_cfg.type = TRAJ_CFG_LOG_START_TYPE;
    DO_TRAJ_CFG;
    UNLOCK;
  } else if (objc == 2) {
    if (TCL_OK != Tcl_GetIntFromObj(interp, objv[1], &i1)) {
      return TCL_ERROR;
    }
    servo_num = i1 - 1;
    if (servo_num < 0) servo_num = 0;
    else if (servo_num >= SERVO_NUM) servo_num = SERVO_NUM - 1;
    LOCK;
    servo_cfg[servo_num].type = SERVO_CFG_LOG_START_TYPE;
    DO_SERVO_CFG(servo_num);
    UNLOCK;
  } else {
    Tcl_WrongNumArgs(interp, 1, objv, "{<which>}");
    return TCL_ERROR;
  }

  return TCL_OK;
}

static int
gotk_log_stop(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  int servo_num;
  int i1;

  if (objc == 1) {
    LOCK;
    traj_cfg.type = TRAJ_CFG_LOG_STOP_TYPE;
    DO_TRAJ_CFG;
    UNLOCK;
  } else if (objc == 2) {
    if (TCL_OK != Tcl_GetIntFromObj(interp, objv[1], &i1)) {
      return TCL_ERROR;
    }
    servo_num = i1 - 1;
    if (servo_num < 0) servo_num = 0;
    else if (servo_num >= SERVO_NUM) servo_num = SERVO_NUM - 1;
    LOCK;
    servo_cfg[servo_num].type = SERVO_CFG_LOG_STOP_TYPE;
    DO_SERVO_CFG(servo_num);
    UNLOCK;
  } else {
    Tcl_WrongNumArgs(interp, 1, objv, "{<which>}");
    return TCL_ERROR;
  }

  return TCL_OK;
}

static int
gotk_log_logging(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  int servo_num;
  int i1;
  Tcl_Obj * resultPtr;

  if (objc == 1) {
    resultPtr = Tcl_GetObjResult(interp);
    Tcl_SetIntObj(resultPtr, (int) traj_set_ptr->log_logging);
  } else if (objc == 2) {
    if (TCL_OK != Tcl_GetIntFromObj(interp, objv[1], &i1)) {
      return TCL_ERROR;
    }
    servo_num = i1 - 1;
    if (servo_num < 0) servo_num = 0;
    else if (servo_num >= SERVO_NUM) servo_num = SERVO_NUM - 1;
    resultPtr = Tcl_GetObjResult(interp);
    Tcl_SetIntObj(resultPtr, (int) servo_set_ptr[servo_num]->log_logging);
  } else {
    Tcl_WrongNumArgs(interp, 1, objv, "{<which>}");
    return TCL_ERROR;
  }

  return TCL_OK;
}

static int
gotk_log_type(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  Tcl_SetResult(interp, go_log_symbol(go_log_ptr->type), TCL_STATIC);

  return TCL_OK;
}

static int
gotk_log_which(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  Tcl_Obj * resultPtr;

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  resultPtr = Tcl_GetObjResult(interp);
  /* internally, joint numbering starts at 0; in the gui, it starts at 1 */
  Tcl_SetIntObj(resultPtr, (int) go_log_ptr->which + 1);

  return TCL_OK;
}

static int
gotk_log_size(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  Tcl_Obj * resultPtr;

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetIntObj(resultPtr, (int) go_log_ptr->size);

  return TCL_OK;
}

static int
gotk_log_howmany(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  Tcl_Obj * resultPtr;

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetIntObj(resultPtr, (int) go_log_ptr->howmany);

  return TCL_OK;
}

static int
gotk_log_dump(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  char * filename;
  FILE * fp;
  go_log_entry entry;
  int type;
  int which;
  go_rpy rpy;

  if (objc != 2) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }

  if (NULL == go_log_ptr) {
    fprintf(stderr, "gotk: null log pointer\n");
    return TCL_OK;
  }

  filename = Tcl_GetString(objv[1]);

  if (NULL == (fp = fopen(filename, "w"))) {
    fprintf(stderr, "gotk: can't open %s\n", filename);
    return TCL_OK;
  }

  type = go_log_type(go_log_ptr);
  which = go_log_which(go_log_ptr);

  /* prefix log with a comment */
  switch (type) {
  case GO_LOG_FERROR:
  case GO_LOG_INPUT:
  case GO_LOG_SETPOINT:
  case GO_LOG_SPEED:
    fprintf(fp, "# %s %d\n", go_log_symbol(go_log_type(go_log_ptr)), (int) go_log_which(go_log_ptr));
    break;

  case GO_LOG_ACT_POS:
  case GO_LOG_CMD_POS:
  case GO_LOG_XINV:
  case GO_LOG_MAGXINV:
    fprintf(fp, "# %s\n", go_log_symbol(go_log_type(go_log_ptr)));
      break;

  default:
    fprintf(fp, "# (Unknown log type %d)\n", type);
    break;
  }

  while (go_log_howmany(go_log_ptr) > 0) {
    if (GO_RESULT_ERROR == go_log_get(go_log_ptr, &entry)) {
      fprintf(stderr, "gotk: can't get log entries\n");
      break;
    }
    switch (type) {
    case GO_LOG_FERROR:
      fprintf(fp, "%f %f\n", (double) entry.time, FGQ(entry.u.ferror.ferror, which));
      break;
    case GO_LOG_INPUT:
      fprintf(fp, "%f %f\n", (double) entry.time, FGQ(entry.u.input.input, which));
      break;
    case GO_LOG_ACT_POS:
      go_quat_rpy_convert(&entry.u.act_pos.pos.rot, &rpy);
      fprintf(fp, "%f %f %f %f %f %f %f\n", (double) entry.time, 
	      FGL(entry.u.act_pos.pos.tran.x),
	      FGL(entry.u.act_pos.pos.tran.y),
	      FGL(entry.u.act_pos.pos.tran.z),
	      FGA(rpy.r), FGA(rpy.p), FGA(rpy.y));
      break;
    case GO_LOG_CMD_POS:
      go_quat_rpy_convert(&entry.u.cmd_pos.pos.rot, &rpy);
      fprintf(fp, "%f %f %f %f %f %f %f\n", (double) entry.time, 
	      FGL(entry.u.cmd_pos.pos.tran.x),
	      FGL(entry.u.cmd_pos.pos.tran.y),
	      FGL(entry.u.cmd_pos.pos.tran.z),
	      FGA(rpy.r), FGA(rpy.p), FGA(rpy.y));
      break;
    case GO_LOG_XINV:
      go_quat_rpy_convert(&entry.u.xinv.xinv.rot, &rpy);
      fprintf(fp, "%f %f %f %f %f %f %f\n", (double) entry.time, 
	      FGL(entry.u.xinv.xinv.tran.x),
	      FGL(entry.u.xinv.xinv.tran.y),
	      FGL(entry.u.xinv.xinv.tran.z),
	      FGA(rpy.r), FGA(rpy.p), FGA(rpy.y));
      break;
    case GO_LOG_MAGXINV:
      fprintf(fp, "%f %f %f %f\n", (double) entry.time, 
	      FGL(entry.u.magxinv.x),
	      FGL(entry.u.magxinv.y),
	      FGL(entry.u.magxinv.mag));
      break;
    case GO_LOG_SETPOINT:
	      fprintf(fp, "%f %f\n", (double) entry.time, FGQ(entry.u.setpoint.setpoint, which));
      break;
    case GO_LOG_SPEED:
	      fprintf(fp, "%f %f\n", (double) entry.time, FGQ(entry.u.speed.speed, which));
      break;
    default:
      fprintf(fp, "%f 0 # unknown log type\n", (double) entry.time);
      break;
    }
  }
  fclose(fp);

  return TCL_OK;
}

static int
gotk_io_num(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  Tcl_Obj * resultPtr;

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  resultPtr = Tcl_GetObjResult(interp);

  if (CD1 == clientData) {
    Tcl_SetIntObj(resultPtr, (int) go_io_ptr->num_ain);
  } else if (CD2 == clientData) {
    Tcl_SetIntObj(resultPtr, (int) go_io_ptr->num_aout);
  } else if (CD3 == clientData) {
    Tcl_SetIntObj(resultPtr, (int) go_io_ptr->num_din);
  } else {
    Tcl_SetIntObj(resultPtr, (int) go_io_ptr->num_dout);
  }

  return TCL_OK;
}

static int
gotk_io_in(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  int index;			/* 1..max for the user */
  Tcl_Obj * resultPtr;

  if (objc != 2) {
    Tcl_WrongNumArgs(interp, 1, objv, "<IO index>");
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetIntFromObj(interp, objv[1], &index)) {
    return TCL_ERROR;
  }

  index--;			/* 0..max-1 for the controller */

  resultPtr = Tcl_GetObjResult(interp);

  if (CD1 == clientData) {
    /* get ain */
    if (index < 0) index = 0;
    else if (index >= GO_IO_NUM_AIN) index = GO_IO_NUM_AIN - 1;
    Tcl_SetDoubleObj(resultPtr, (double) go_input_ptr->ain[index]);
  } else {
    /* get din */
    if (index < 0) index = 0;
    else if (index >= GO_IO_NUM_DIN) index = GO_IO_NUM_DIN - 1;
    Tcl_SetIntObj(resultPtr, (int) (go_input_ptr->din[index] != 0));
  }

  return TCL_OK;
}

static int
gotk_io_out(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  int index;			/* 1..max for the user */
  int ival;
  double dval;

  if (objc != 3) {
    Tcl_WrongNumArgs(interp, 1, objv, "<IO index> <value>");
    return TCL_ERROR;
  }

  if (TCL_OK != Tcl_GetIntFromObj(interp, objv[1], &index)) {
    return TCL_ERROR;
  }

  index--;			/* 0..max-1 for the controller */

  if (CD1 == clientData) {
    /* set aout */
    if (index < 0) index = 0;
    else if (index >= GO_IO_NUM_AOUT) index = GO_IO_NUM_AOUT - 1;
    if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[2], &dval)) {
      return TCL_ERROR;
    }
    /* We'll write this one value directly to shared memory, setting
       the head and tail flags around it. This saves us from having to
       copy in a whole structure. This won't guard against
       inconsistencies caused by other writers, so if you have
       multiple writers, you need to handshake that yourself. */
    go_io_ptr->output.head++;
    go_io_ptr->output.aout[index] = dval;
    go_io_ptr->output.tail = go_io_ptr->output.head;
  } else {
    /* set dout */
    if (index < 0) index = 0;
    else if (index >= GO_IO_NUM_DOUT) index = GO_IO_NUM_DOUT - 1;
    if (TCL_OK != Tcl_GetIntFromObj(interp, objv[2], &ival)) {
      return TCL_ERROR;
    }
    /* see shared memory comment above */
    go_io_ptr->output.head++;
    go_io_ptr->output.dout[index] = ival;
    go_io_ptr->output.tail = go_io_ptr->output.head;
  }

  return TCL_OK;
}

static int ini_load(char * inifile)
{
  FILE * fp;
  const char * inistring;
  int servo_num;
  char section[INIFILE_MAX_LINELEN];
  double d1;

  if (NULL == (fp = fopen(inifile, "r"))) {
    fprintf(stderr, "gotk: can't open %s\n", inifile);
    return 1;
  }

#undef CLOSE_AND_RETURN
#define CLOSE_AND_RETURN			\
  fclose(fp);					\
  return 1

  inistring = ini_find(fp, "ESTOP_COMMAND", "GOMOTION");
  if (NULL != inistring) {
    strncpy(estop_command, inistring, sizeof(estop_command));
    estop_command[sizeof(estop_command) - 1] = 0;
  }

  inistring = ini_find(fp, "SHM_KEY", "TRAJ");
  if (NULL == inistring) {
    fprintf(stderr, "gotk: [TRAJ] SHM_KEY not found in %s\n", inifile);
    CLOSE_AND_RETURN;
  } else if (1 != sscanf(inistring, "%i", &traj_shm_key)) {
    fprintf(stderr, "gotk: bad entry: [TRAJ] SHM_KEY = %s\n", inistring);
    CLOSE_AND_RETURN;
  }

  inistring = ini_find(fp, "SHM_KEY", "TOOL");
  /* it's optional */
  if (NULL != inistring) {
    if (1 != sscanf(inistring, "%i", &tool_shm_key)) {
      fprintf(stderr, "gotk: bad entry: [TOOL] SHM_KEY = %s\n", inistring);
      CLOSE_AND_RETURN;
    }
  }

  inistring = ini_find(fp, "SHM_KEY", "TASK");
  /* it's optional */
  if (NULL != inistring) {
    if (1 != sscanf(inistring, "%i", &task_shm_key)) {
      fprintf(stderr, "gotk: bad entry: [TASK] SHM_KEY = %s\n", inistring);
      CLOSE_AND_RETURN;
    }
  }

  inistring = ini_find(fp, "HOWMANY", "SERVO");
  if (NULL == inistring) {
    for (servo_num = 0; servo_num < SERVO_NUM; servo_num++) {
      sprintf(section, "SERVO_%d", servo_num + 1);
      if (NULL == ini_find(fp, "CYCLE_TIME", section)) break;
    }
    servo_howmany = servo_num;
  } else if (1 != sscanf(inistring, "%i", &servo_howmany)) {
    fprintf(stderr, "gotk: bad entry: [SERVO] HOWMANY = %s\n", inistring);
    CLOSE_AND_RETURN;
  }

  inistring = ini_find(fp, "SHM_KEY", "SERVO");
  if (NULL == inistring) {
    fprintf(stderr, "gotk: [SERVO] SHM_KEY not found in %s\n", inifile);
    CLOSE_AND_RETURN;
  } else if (1 != sscanf(inistring, "%i", &servo_shm_key)) {
    fprintf(stderr, "gotk: bad entry: [SERVO] SHM_KEY = %s\n", inistring);
    CLOSE_AND_RETURN;
  }

  inistring = ini_find(fp, "SHM_KEY", "GO_LOG");
  if (NULL == inistring) {
    fprintf(stderr, "gotk: [GO_LOG] SHM_KEY not found in %s\n", inifile);
    CLOSE_AND_RETURN;
  } else if (1 != sscanf(inistring, "%i", &go_log_shm_key)) {
    fprintf(stderr, "gotk: bad entry: [GO_LOG] SHM_KEY = %s\n", inistring);
    CLOSE_AND_RETURN;
  }

  inistring = ini_find(fp, "SHM_KEY", "GO_IO");
  if (NULL == inistring) {
    fprintf(stderr, "gotk: [GO_IO] SHM_KEY not found in %s\n", inifile);
    CLOSE_AND_RETURN;
  } else if (1 != sscanf(inistring, "%i", &go_io_shm_key)) {
    fprintf(stderr, "gotk: bad entry: [GO_IO] SHM_KEY = %s\n", inistring);
    CLOSE_AND_RETURN;
  }

  length_units_per_m = 1.0;
  inistring = ini_find(fp, "LENGTH_UNITS_PER_M", "GOMOTION");
  if (NULL == inistring) {
    fprintf(stderr, "gotk: [GOMOTION] LENGTH_UNITS_PER_M not found, using %f\n", length_units_per_m);
  } else if (1 != sscanf(inistring, "%lf", &d1)) {
    fprintf(stderr, "gotk: bad entry: [GOMOTION] LENGTH_UNITS_PER_M = %s\n", inistring);
    CLOSE_AND_RETURN;
  } else if (d1 <= 0.0) {
    fprintf(stderr, "gotk: invalid entry: [GOMOTION] LENGTH_UNITS_PER_M = %s must be positive\n", inistring);
    CLOSE_AND_RETURN;
  } else {
    length_units_per_m = d1;
  }
  m_per_length_units = (go_real) (1.0 / length_units_per_m);

  angle_units_per_rad = 1.0;
  inistring = ini_find(fp, "ANGLE_UNITS_PER_RAD", "GOMOTION");
  if (NULL == inistring) {
    fprintf(stderr, "gotk: [GOMOTION] ANGLE_UNITS_PER_RAD not found, using %f\n", angle_units_per_rad);
  } else if (1 != sscanf(inistring, "%lf", &d1)) {
    fprintf(stderr, "gotk: bad entry: [GOMOTION] ANGLE_UNITS_PER_RAD = %s\n", inistring);
    CLOSE_AND_RETURN;
  } else if (d1 <= 0.0) {
    fprintf(stderr, "gotk: invalid entry: [GOMOTION] ANGLE_UNITS_PER_RAD = %s must be positive\n", inistring);
    CLOSE_AND_RETURN;
  } else {
    angle_units_per_rad = d1;
  }
  rad_per_angle_units = (go_real) (1.0 / angle_units_per_rad);

  for (servo_num = 0; servo_num < servo_howmany; servo_num++) {
    sprintf(section, "SERVO_%d", servo_num + 1);

    joint_quantity[servo_num] = GO_QUANTITY_LENGTH;
    inistring = ini_find(fp, "QUANTITY", section);
    if (NULL == inistring) {
      fprintf(stderr, "gotk: [%s] QUANTITY not found, using linear\n", section);
    } else {
      if (ini_match(inistring, "ANGLE")) {
	joint_quantity[servo_num] = GO_QUANTITY_ANGLE;
      } else if (ini_match(inistring, "LENGTH")) {
	joint_quantity[servo_num] = GO_QUANTITY_LENGTH;
      } else {
	fprintf(stderr, "gotk: bad entry: [%s] QUANTITY = %s\n", section, inistring);
	CLOSE_AND_RETURN;
      }
    }

    home_vel[servo_num] = 0.0;
    inistring = ini_find(fp, "HOME_VEL", section);
    if (NULL == inistring) {
      fprintf(stderr, "gotk: [%s] HOME_VEL not found, using 0.0\n", section);
    } else {
      if (1 == sscanf(inistring, "%lf", &d1)) {
	home_vel[servo_num] = TGQ(d1, servo_num);
      } else {
	fprintf(stderr, "gotk: bad entry: [%s] HOME_VEL = %s\n", section, inistring);
	CLOSE_AND_RETURN;
      }
    }
  }

  fclose(fp);
  return 0;
}

#ifdef HAVE_SDL

static int num_joysticks = 0;
static SDL_Joystick ** joysticks = NULL;
static void *task = NULL;
enum {WATCH_PERIOD_NSEC = 100000000};

static void
watch_joystick(void *arg)
{
  int done;
  int axis;
  int servo_num;
  SDL_Event event;
  csys_select_t csys_cpy;
  go_vel vw;
  go_cart raw_v = {0, 0, 0}, new_v;
  go_cart raw_w = {0, 0, 0}, new_w;
  go_real scale = 1.0/32768.0;
  go_real mag;
  go_real jointvel;
  go_flag dorot = 0;
  go_flag input_control_cpy;

  /* loop, getting joystick events */
  for (done = 0; done == 0; ulapi_wait(WATCH_PERIOD_NSEC)) {
    while (SDL_PollEvent(&event)) {
      /* got something, so update comm buffers */
      switch (event.type) {

      case SDL_JOYAXISMOTION:
	axis = event.jaxis.axis;
	dbprintf("Joystick %d axis %d value: %d\n",
	       event.jaxis.which, event.jaxis.axis, event.jaxis.value);
	LOCK;
	csys_cpy = csys_select;
	input_control_cpy = input_control;
	UNLOCK;

	/* inhibit any joystick control here if Tk GUI has input control */
	if (input_control_cpy) {
	  break;
	}

	if (CSYS_JOINT == csys_cpy) {
	  if (3 == event.jaxis.axis) {
	    jointvel = -event.jaxis.value * scale;
	  }
	  dbprintf("Jointvel = %f\n", (double) jointvel);
	  LOCK;
	  /* zero all the speeds */
	  for (servo_num = 0; servo_num < SERVO_NUM; servo_num++) {
	    traj_cmd.u.teleop_joint.v[servo_num] = 0.0;
	    traj_cmd.u.teleop_joint.a[servo_num] = local_jacc[servo_num];
	    traj_cmd.u.teleop_joint.j[servo_num] = local_jjerk[servo_num];
	  }
	  traj_cmd.type = TRAJ_CMD_TELEOP_JOINT_TYPE;
	  /* overwrite the zero speed for this joint with our scaled value */
	  traj_cmd.u.teleop_joint.v[joint_select] = local_jvel[joint_select] * jointvel;
	  DO_TRAJ_CMD;
	  UNLOCK;
	} else {		/* CSYS_WORLD || CSYS_TOOL */
	  /*
	    Indexes are

	    0 ->  left joystick left-right
	    1 ->  left joystick down-up
	    2 -> right joystick left-right
	    3 -> right joystick down-up

	    The mapping is

	    0 -> minux X or minus roll
	    1 -> minus Y or minus pitch
	    2 -> unused
	    3 -> minus Z or minus yaw

	    The odd indexes are all negated so that the joystick
	    feels right, up = positive direction.
	  */
	  if (dorot) {
	    /* null out the outputed trans motion */
	    vw.v.x = vw.v.y = vw.v.z = 0;
	    /* update the new field of the saved rot motion */
	    if (0 == event.jaxis.axis) raw_w.x = -event.jaxis.value * scale;
	    else if (1 == event.jaxis.axis) raw_w.y = -event.jaxis.value * scale;
	    else if (3 == event.jaxis.axis) raw_w.z = -event.jaxis.value * scale;
	    new_w = raw_w;
	    /* scale to keep within max trans vel */
	    go_cart_mag(&new_w, &mag);
	    if (mag > 1.0) (void) go_cart_unit(&new_w, &new_w);
	    /* "output" it to 'pose' */
	    go_cart_scale_mult(&new_w, local_rvel, &vw.w);
	  } else {
	    /* null out the outputed rot motions */
	    vw.w.x = vw.w.y = vw.w.z = 0;
	    /* update the new field of the saved trans motion */
	    if (0 == event.jaxis.axis) raw_v.x = -event.jaxis.value * scale;
	    else if (1 == event.jaxis.axis) raw_v.y = -event.jaxis.value * scale;
	    else if (3 == event.jaxis.axis) raw_v.z = -event.jaxis.value * scale;
	    new_v = raw_v;
	    /* scale to keep within max trans vel */
	    go_cart_mag(&new_v, &mag);
	    if (mag > 1.0) (void) go_cart_unit(&new_v, &new_v);
	    /* "output" it to 'pose' */
	    go_cart_scale_mult(&new_v, local_tvel, &vw.v);
	  }
	  dbprintf("Velocity = %f %f %f %f %f %f\n",
		 (double) vw.v.x,
		 (double) vw.v.y,
		 (double) vw.v.z,
		 (double) vw.w.x,
		 (double) vw.w.y,
		 (double) vw.w.z
		 );
	  if (CSYS_WORLD == csys_cpy) {
	    LOCK;
	    traj_cmd.type = TRAJ_CMD_TELEOP_WORLD_TYPE;
	    traj_cmd.u.teleop_world.tv = vw;
	    traj_cmd.u.teleop_world.ta = local_tacc;
	    traj_cmd.u.teleop_world.tj = local_tjerk;
	    traj_cmd.u.teleop_world.ra = local_racc;
	    traj_cmd.u.teleop_world.rj = local_rjerk;
	    DO_TRAJ_CMD;
	    UNLOCK;
	  } else if (CSYS_TOOL == csys_cpy) {
	    LOCK;
	    traj_cmd.type = TRAJ_CMD_TELEOP_TOOL_TYPE;
	    traj_cmd.u.teleop_tool.tv = vw;
	    traj_cmd.u.teleop_tool.ta = local_tacc;
	    traj_cmd.u.teleop_tool.tj = local_tjerk;
	    traj_cmd.u.teleop_tool.ra = local_racc;
	    traj_cmd.u.teleop_tool.rj = local_rjerk;
	    DO_TRAJ_CMD;
	    UNLOCK;
	  } /* else something else, ignore */
	}
	break;

      case SDL_JOYBUTTONDOWN:
	dbprintf("Joystick %d button %d down\n",
	       event.jbutton.which, event.jbutton.button);
	switch (event.jbutton.button) {
	case 0:
	  /* pressing button #1 takes joystick control */
	  LOCK;
	  input_control = 0;
	  joint_select = (go_integer) event.jbutton.button;
	  UNLOCK;
	  break;

	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	  LOCK;
	  joint_select = (go_integer) event.jbutton.button;
	  UNLOCK;
	  break;

	case 7:
	  dorot = 1;
	  break;
	case 8:
	  LOCK;
	  if (CSYS_JOINT == csys_select) csys_select = CSYS_WORLD;
	  else if (CSYS_WORLD == csys_select) csys_select = CSYS_TOOL;
	  else csys_select = CSYS_JOINT;
	  UNLOCK;
	  break;
	case 9:
	  /* home the selected joint, if we're in joint mode */
	  /* FIXME-- this was pasted from above, with [joint]
	     changed to [joint_select]. Factor this out. */
	  if (CSYS_JOINT == csys_select) {
	    LOCK;

	    /* set the destination for all joints to be their current
	       absolute position, since we're doing a UJOINT motion */
	    for (servo_num = 0; servo_num < SERVO_NUM; servo_num++) {
	      traj_cmd.u.move_ujoint.d[servo_num] = servo_stat_ptr[servo_num]->input;
	      /* we'll override the max vel with the homing vel if necessary */
	      traj_cmd.u.move_ujoint.v[servo_num] = servo_set_ptr[servo_num]->max_vel;
	      traj_cmd.u.move_ujoint.a[servo_num] = servo_set_ptr[servo_num]->max_acc;
	      traj_cmd.u.move_ujoint.j[servo_num] = servo_set_ptr[servo_num]->max_jerk;
	      traj_cmd.u.move_ujoint.home[servo_num] = 0;
	    }

	    /*
	      there are three directions to home: negative, positive and stay here
	    */
	    traj_cmd.type = TRAJ_CMD_MOVE_UJOINT_TYPE;

	    if (home_vel[joint_select] > GO_REAL_EPSILON) {
	      /* home in the positive direction */
	      traj_cmd.u.move_ujoint.v[joint_select] = home_vel[joint_select];
	      traj_cmd.u.move_ujoint.d[joint_select] =
		servo_stat_ptr[joint_select]->input +
		(servo_set_ptr[joint_select]->max_limit -
		 servo_set_ptr[joint_select]->min_limit + 1.0);
	    } else  if (home_vel[joint_select] < -GO_REAL_EPSILON) {
	      /* home in the negative direction */
	      traj_cmd.u.move_ujoint.v[joint_select] = -home_vel[joint_select];	/* needs to be pos */
	      traj_cmd.u.move_ujoint.d[joint_select] = 
		servo_stat_ptr[joint_select]->input -
		(servo_set_ptr[joint_select]->max_limit -
		 servo_set_ptr[joint_select]->min_limit + 1.0);
	    } else {
	      /* else stay here, leave v at vmax */
	      traj_cmd.u.move_ujoint.d[joint_select] = 
		servo_stat_ptr[joint_select]->input;
	    }
  
	    traj_cmd.u.move_ujoint.home[joint_select] = 1;
	    traj_cmd.u.move_ujoint.id = traj_cmd.serial_number + 1;
	    DO_TRAJ_CMD;

	    UNLOCK;
	  }
	  break;
	}
	break;

      case SDL_JOYBUTTONUP:
	dbprintf("Joystick %d button %d up\n",
	       event.jbutton.which, event.jbutton.button);
	switch (event.jbutton.button) {
	case 7:
	  dorot = 0;
	  break;
	}
	break;

      case SDL_JOYHATMOTION:
	dbprintf("Joystick %d hat %d value:", event.jhat.which, event.jhat.hat);
	if (event.jhat.value == SDL_HAT_CENTERED)
	  dbprintf(" centered");
	if (event.jhat.value & SDL_HAT_UP)
	  dbprintf(" up");
	if (event.jhat.value & SDL_HAT_RIGHT)
	  dbprintf(" right");
	if (event.jhat.value & SDL_HAT_DOWN)
	  dbprintf(" down");
	if (event.jhat.value & SDL_HAT_LEFT)
	  dbprintf(" left");
	dbprintf("\n");
	break;

      case SDL_JOYBALLMOTION:
	dbprintf("Joystick %d ball %d delta: (%d,%d)\n",
	       event.jball.which,
	       event.jball.ball, event.jball.xrel, event.jball.yrel);
	break;

      case SDL_KEYDOWN:
	if (event.key.keysym.sym != SDLK_ESCAPE) {
	  break;
	}
	/* fall through to signal quit */
      case SDL_QUIT:
	done = 1;
	break;

      default:
	break;
      }
    }
  } /* while (!done) */
}

static int
init_joystick(void)
{
  const char * name;
  int i;

  /* initialize SDL (Note: video is required to start event loop) */
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK) < 0) {
    fprintf(stderr, "gotk: couldn't initialize SDL: %s\n", SDL_GetError());
    return 1;
  }

  /* print information about the joysticks */
  num_joysticks =  SDL_NumJoysticks();
  if (num_joysticks < 1) {
    dbprintf("There are no joysticks attached\n");
    return 1;
  }
  dbprintf("There are %d joysticks attached\n", num_joysticks);

  /* get array of joystick pointers */
  joysticks = malloc(num_joysticks * sizeof(SDL_Joystick *));
  /* get each joystick */
  for (i = 0; i < num_joysticks; i++) {
    name = SDL_JoystickName(i);
    dbprintf("Joystick %d: %s\n", i, name ? name : "Unknown");
    if (NULL == (joysticks[i] = SDL_JoystickOpen(i))) {
      fprintf(stderr, "gotk: couldn't open joystick %d: %s\n", i, SDL_GetError());
      return 1;
    }
  }

  task = ulapi_task_new();
  if (NULL != task) {
    ulapi_task_start(task, 
		     watch_joystick,
		     NULL,
		     ulapi_prio_lowest(),
		     WATCH_PERIOD_NSEC);
  }
  
  return 0;
}

static int
quit_joystick(void)
{
  int i;

  if (NULL != task) {
    ulapi_task_stop(task);
    task = NULL;
  }

  if (NULL != joysticks) {
    for (i = 0; i < num_joysticks; i++) {
      if (NULL != joysticks[i]) {
	SDL_JoystickClose(joysticks[i]);
	joysticks[i] = NULL;
      }
    }
    free(joysticks);
  }

  SDL_QuitSubSystem(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK);

  return 0;
}

#endif /* HAVE_SDL */

static int
gotk_have_joystick(ClientData clientData, Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
  Tcl_Obj * resultPtr;

  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }
  
  resultPtr = Tcl_GetObjResult(interp);
  Tcl_SetIntObj(resultPtr,
#ifdef HAVE_SDL
		num_joysticks
#else
		0
#endif
		);

  return TCL_OK;
}

/* ------- Random Variates ------ */

#include "variates.h"

static int
do_rand(ClientData clientData, Tcl_Interp *interp, int objc, Tcl_Obj *CONST objv[])
{
#define DO_RAND_DECL(NAME)					\
  static NAME##_random_struct *NAME##_random_array = NULL;	\
  static int NAME##_random_num = 0;				\
  NAME##_random_struct *NAME##_random_struct_ptr

  DO_RAND_DECL(uniform);
  DO_RAND_DECL(normal);
  DO_RAND_DECL(exponential);
  DO_RAND_DECL(weibull);
  DO_RAND_DECL(gamma);
  DO_RAND_DECL(pearson_v);

  Tcl_Obj *resultPtr;
  double alpha, beta;
  int num;

  if (objc < 2) {
    Tcl_WrongNumArgs(interp, 1, objv, "init | get {<other options>}");
    return TCL_ERROR;
  }

  if (CD1 == clientData) {
#define DO_RAND_2(NAME)							\
    if (! strcmp("init", Tcl_GetString(objv[1]))) {			\
      if (objc != 4) {							\
	Tcl_WrongNumArgs(interp, 1, objv, "init <alpha/shape> <beta/scale>"); \
	return TCL_ERROR;						\
      }									\
      if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[2], &alpha) ||	\
	  TCL_OK != Tcl_GetDoubleFromObj(interp, objv[3], &beta)) {	\
	Tcl_SetResult(interp, "bad args", TCL_STATIC);			\
	return TCL_ERROR;						\
      }									\
      resultPtr = Tcl_GetObjResult(interp);				\
      NAME##_random_struct_ptr = realloc(NAME##_random_array, (NAME##_random_num + 1) * sizeof(*NAME##_random_array)); \
      if (NAME##_random_struct_ptr == NAME##_random_array) {		\
	/* can't grow it */						\
	Tcl_SetIntObj(resultPtr, -1);					\
	return TCL_OK;							\
      } else {								\
	NAME##_random_array = NAME##_random_struct_ptr;			\
	NAME##_random_init(&NAME##_random_array[NAME##_random_num], alpha, beta); \
	Tcl_SetIntObj(resultPtr, NAME##_random_num);			\
	NAME##_random_num++;						\
	return TCL_OK;							\
      }									\
    } else if (! strcmp("get", Tcl_GetString(objv[1]))) {		\
      if (objc != 3) {							\
	Tcl_WrongNumArgs(interp, 1, objv, "get <NAME##_random instance>"); \
	return TCL_ERROR;						\
      }									\
      if (TCL_OK != Tcl_GetIntFromObj(interp, objv[2], &num)) {		\
	Tcl_SetResult(interp, "bad args", TCL_STATIC);			\
	return TCL_ERROR;						\
      }									\
      if (num < 0 || num >= NAME##_random_num) {			\
	Tcl_SetResult(interp, "bad NAME##_random instance", TCL_STATIC); \
	return TCL_ERROR;						\
      }									\
      resultPtr = Tcl_GetObjResult(interp);				\
      Tcl_SetDoubleObj(resultPtr, NAME##_random_real(&NAME##_random_array[num])); \
      return TCL_OK;							\
    } else {								\
      Tcl_SetResult(interp, "bad args", TCL_STATIC);			\
      return TCL_ERROR;							\
    }									\
    return TCL_OK
    DO_RAND_2(uniform);
  } else if (CD2 == clientData) {
    DO_RAND_2(normal);
  } else if (CD3 == clientData) {
#define DO_RAND_1(NAME)							\
    if (! strcmp("init", Tcl_GetString(objv[1]))) {			\
      if (objc != 3) {							\
	Tcl_WrongNumArgs(interp, 1, objv, "init <alpha>");		\
	return TCL_ERROR;						\
      }									\
      if (TCL_OK != Tcl_GetDoubleFromObj(interp, objv[2], &alpha)) {	\
	Tcl_SetResult(interp, "bad args", TCL_STATIC);			\
	return TCL_ERROR;						\
      }									\
      resultPtr = Tcl_GetObjResult(interp);				\
      NAME##_random_struct_ptr = realloc(NAME##_random_array, (NAME##_random_num + 1) * sizeof(*NAME##_random_array)); \
      if (NAME##_random_struct_ptr == NAME##_random_array) {		\
	/* can't grow it */						\
	Tcl_SetIntObj(resultPtr, -1);					\
	return TCL_OK;							\
      } else {								\
	NAME##_random_array = NAME##_random_struct_ptr;			\
	NAME##_random_init(&NAME##_random_array[NAME##_random_num], alpha); \
	Tcl_SetIntObj(resultPtr, NAME##_random_num);			\
	NAME##_random_num++;						\
	return TCL_OK;							\
      }									\
    } else if (! strcmp("get", Tcl_GetString(objv[1]))) {		\
      if (objc != 3) {							\
	Tcl_WrongNumArgs(interp, 1, objv, "get <NAME##_random instance>"); \
	return TCL_ERROR;						\
      }									\
      if (TCL_OK != Tcl_GetIntFromObj(interp, objv[2], &num)) {		\
	Tcl_SetResult(interp, "bad args", TCL_STATIC);			\
	return TCL_ERROR;						\
      }									\
      if (num < 0 || num >= NAME##_random_num) {			\
	Tcl_SetResult(interp, "bad NAME##_random instance", TCL_STATIC); \
	return TCL_ERROR;						\
      }									\
      resultPtr = Tcl_GetObjResult(interp);				\
      Tcl_SetDoubleObj(resultPtr, NAME##_random_real(&NAME##_random_array[num])); \
      return TCL_OK;							\
    } else {								\
      Tcl_SetResult(interp, "bad args", TCL_STATIC);			\
      return TCL_ERROR;							\
    }									\
    return TCL_OK
    DO_RAND_1(exponential);
  } else if (CD4 == clientData) {
    DO_RAND_2(weibull);
  } else if (CD5 == clientData) {
    DO_RAND_2(gamma);
  } else {
    DO_RAND_2(pearson_v);
  }
}

/* --- end of Random Variates --- */

enum { INIFILE_NAME_LEN = 80 };
static char inifile_name[INIFILE_NAME_LEN] = "gomotion.ini";

static int
gotk_inifile(ClientData clientData, Tcl_Interp *interp, int objc, Tcl_Obj *CONST objv[])
{
  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }

  Tcl_SetResult(interp, (char *) inifile_name, TCL_STATIC);

  return TCL_OK;
}

static const char *ulapi_name = "";

static int
gotk_ulapi(ClientData clientData, Tcl_Interp *interp, int objc, Tcl_Obj *CONST objv[])
{
  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, NULL);
    return TCL_ERROR;
  }

  Tcl_SetResult(interp, (char *) ulapi_name, TCL_STATIC);

  return TCL_OK;
}

/* variables to link to the Tcl/Tk interpreter */
static const int go_rcs_status_done = GO_RCS_STATUS_DONE;
static const int go_rcs_status_exec = GO_RCS_STATUS_EXEC;
static const int go_rcs_status_error = GO_RCS_STATUS_ERROR;

#ifdef BUILD_GOTCL
int
Tcl_AppInit(Tcl_Interp * interp)
#else
static int
Tk_AppInit(Tcl_Interp * interp)
#endif
{
  if (Tcl_Init(interp) == TCL_ERROR) {
    return TCL_ERROR;
  }
#ifndef BUILD_GOTCL
  if (Tk_Init(interp) == TCL_ERROR) {
    return TCL_ERROR;
  }
#endif

  Tcl_CreateObjCommand(interp, "gotk_update", gotk_update, NULL, NULL);

  Tcl_CreateObjCommand(interp, "gotk_get_csys_select", gotk_get_csys_select, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_set_csys_select", gotk_set_csys_select, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_get_joint_select", gotk_get_joint_select, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_set_joint_select", gotk_set_joint_select, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_get_cart_select", gotk_get_cart_select, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_set_cart_select", gotk_set_cart_select, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_get_input_control", gotk_get_input_control, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_set_input_control", gotk_set_input_control, NULL, NULL);

  Tcl_CreateObjCommand(interp, "gotk_set_timeout", gotk_set_timeout, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_traj_cmd", gotk_traj_cmd, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_traj_admin_state", gotk_traj_admin_state, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_traj_state", gotk_traj_state, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_traj_status", gotk_traj_status, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_traj_heartbeat", gotk_traj_heartbeat, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_world_homed", gotk_world_homed, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_inpos", gotk_inpos, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_joint_pos", gotk_joint_pos, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_joint_cmd_pos", gotk_joint_cmd_pos, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_joint_homed", gotk_joint_homed, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_joint_active", gotk_joint_active, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_world_pos", gotk_world_pos, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_world_cmd_pos", gotk_world_cmd_pos, NULL, NULL);

  Tcl_CreateObjCommand(interp, "gotk_traj_set_tpar", gotk_traj_set_tpar, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_traj_set_rpar", gotk_traj_set_rpar, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_traj_set_jpar", gotk_traj_set_jpar, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_traj_set_move_time", gotk_traj_set_move_time, NULL, NULL);

  Tcl_CreateObjCommand(interp, "gotk_max_vel", gotk_joint_profile, CD1, NULL);
  Tcl_CreateObjCommand(interp, "gotk_max_acc", gotk_joint_profile, CD2, NULL);
  Tcl_CreateObjCommand(interp, "gotk_max_jerk", gotk_joint_profile, CD3, NULL);
  Tcl_CreateObjCommand(interp, "gotk_max_tvel", gotk_world_profile, CD1, NULL);
  Tcl_CreateObjCommand(interp, "gotk_max_tacc", gotk_world_profile, CD2, NULL);
  Tcl_CreateObjCommand(interp, "gotk_max_tjerk", gotk_world_profile, CD3, NULL);
  Tcl_CreateObjCommand(interp, "gotk_max_rvel", gotk_world_profile, CD4, NULL);
  Tcl_CreateObjCommand(interp, "gotk_max_racc", gotk_world_profile, CD5, NULL);
  Tcl_CreateObjCommand(interp, "gotk_max_rjerk", gotk_world_profile, CD6, NULL);

  Tcl_CreateObjCommand(interp, "gotk_max_scale", gotk_max_scale, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_max_scale_v", gotk_max_scale_v, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_max_scale_a", gotk_max_scale_a, NULL, NULL);

  Tcl_CreateObjCommand(interp, "gotk_traj_init", gotk_traj_init, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_traj_halt", gotk_traj_halt, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_traj_abort", gotk_traj_abort, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_traj_shutdown", gotk_traj_shutdown, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_traj_stop", gotk_traj_stop, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_traj_home", gotk_traj_home, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_traj_hold", gotk_traj_hold, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_traj_jog_joint", gotk_traj_jog_joint, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_traj_jog_world", gotk_traj_jog_world_or_tool, CD1, NULL);
  Tcl_CreateObjCommand(interp, "gotk_traj_jog_tool", gotk_traj_jog_world_or_tool, CD2, NULL);
  Tcl_CreateObjCommand(interp, "gotk_traj_move_ujoint", gotk_traj_move_ujoint, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_traj_move_joint", gotk_traj_move_joint, NULL, NULL);

  Tcl_CreateObjCommand(interp, "gotk_traj_move_world", gotk_traj_move_world_or_tool, CD1, NULL);
  Tcl_CreateObjCommand(interp, "gotk_traj_move_tool", gotk_traj_move_world_or_tool, CD2, NULL);
  Tcl_CreateObjCommand(interp, "gotk_traj_here", gotk_traj_here, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_traj_cfg_scale", gotk_traj_cfg_scale, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_traj_cfg_home", gotk_traj_cfg_home, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_traj_cfg_tool_transform", gotk_traj_cfg_tool_transform, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_traj_get_tool_transform", gotk_traj_get_tool_transform, NULL, NULL);

  Tcl_CreateObjCommand(interp, "gotk_tool_cmd", gotk_tool_cmd, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_tool_admin_state", gotk_tool_admin_state, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_tool_state", gotk_tool_state, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_tool_status", gotk_tool_status, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_tool_heartbeat", gotk_tool_heartbeat, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_tool_init", gotk_tool_init, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_tool_abort", gotk_tool_abort, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_tool_shutdown", gotk_tool_shutdown, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_tool_on", gotk_tool_on, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_tool_off", gotk_tool_off, NULL, NULL);

  Tcl_CreateObjCommand(interp, "gotk_servo_num", gotk_servo_num, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_servo_get_pid", gotk_servo_get_pid, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_servo_set_pid", gotk_servo_set_pid, NULL, NULL);

  Tcl_CreateObjCommand(interp, "gotk_ini", gotk_ini, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_log_init", gotk_log_init, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_log_start", gotk_log_start, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_log_stop", gotk_log_stop, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_log_type", gotk_log_type, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_log_which", gotk_log_which, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_log_size", gotk_log_size, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_log_logging", gotk_log_logging, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_log_howmany", gotk_log_howmany, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_log_dump", gotk_log_dump, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_io_num_ain", gotk_io_num, CD1, NULL);
  Tcl_CreateObjCommand(interp, "gotk_io_num_aout", gotk_io_num, CD2, NULL);
  Tcl_CreateObjCommand(interp, "gotk_io_num_din", gotk_io_num, CD3, NULL);
  Tcl_CreateObjCommand(interp, "gotk_io_num_dout", gotk_io_num, CD4, NULL);
  Tcl_CreateObjCommand(interp, "gotk_io_ain", gotk_io_in, CD1, NULL);
  Tcl_CreateObjCommand(interp, "gotk_io_din", gotk_io_in, CD2, NULL);
  Tcl_CreateObjCommand(interp, "gotk_io_aout", gotk_io_out, CD1, NULL);
  Tcl_CreateObjCommand(interp, "gotk_io_dout", gotk_io_out, CD2, NULL);

  /* the single function for all task-related commands */
  Tcl_CreateObjCommand(interp, "gotk_task", gotk_task, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_have_joystick", gotk_have_joystick, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_have_tool", gotk_have_tool, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_have_task", gotk_have_task, NULL, NULL);

  Tcl_CreateObjCommand(interp, "gotk_inifile", gotk_inifile, NULL, NULL);
  Tcl_CreateObjCommand(interp, "gotk_ulapi", gotk_ulapi, NULL, NULL);

  Tcl_CreateObjCommand(interp, "uniform", do_rand, CD1, NULL);
  Tcl_CreateObjCommand(interp, "normal", do_rand, CD2, NULL);
  Tcl_CreateObjCommand(interp, "exponential", do_rand, CD3, NULL);
  Tcl_CreateObjCommand(interp, "weibull", do_rand, CD4, NULL);
  Tcl_CreateObjCommand(interp, "gamma", do_rand, CD5, NULL);
  Tcl_CreateObjCommand(interp, "pearson_v", do_rand, CD6, NULL);

  /* link some variables */
  Tcl_LinkVar(interp, "GO_RCS_STATUS_DONE", (char *) &go_rcs_status_done, TCL_LINK_INT | TCL_LINK_READ_ONLY);
  Tcl_LinkVar(interp, "GO_RCS_STATUS_EXEC", (char *) &go_rcs_status_exec, TCL_LINK_INT | TCL_LINK_READ_ONLY);
  Tcl_LinkVar(interp, "GO_RCS_STATUS_ERROR", (char *) &go_rcs_status_error, TCL_LINK_INT | TCL_LINK_READ_ONLY);

  return TCL_OK;
}

static void
myexit(void)
{
  static int ran = 0;

  if (ran) return;
  ran = 1;

  if (! peekflag) {
    LOCK;
    if (killflag) {
      traj_cmd.type = TRAJ_CMD_SHUTDOWN_TYPE;
    } else {
      traj_cmd.type = TRAJ_CMD_STOP_TYPE;
    }
    DO_TRAJ_CMD;
    UNLOCK;
  }

#ifdef HAVE_SDL
  quit_joystick();
#endif

  if (NULL != mutex) {
    ulapi_mutex_delete(mutex);
    mutex = NULL;
  }

  free_comm_buffers();

  ulapi_exit();

  exit(0);
}

static void sigquit(int sig)
{
  exit(0);
}

/*
  Built as either 'gotk' or 'gotcl', with or without windowing support,
  respectively.

  We will be called with a script as something like

  gotk pendant.tcl {args to Tcl} -- {args for us}

  Interactively, call with a script '-' to represent stdin, e.g.,

  gotcl - -- -i gomotion.ini

  Args for us are:
  -i <inifile> , specify a initialization file
  -u unix | rtai , specify either unix or real-time unix
  -d , turn debug on
  -k , kill controllers on shutdown
  -x , ignore failures to connect, useful for testing
  -p , peek mode -- don't do any inits, stops, shutdowns
*/

int
main(int argc, char *argv[])
{
  int my_argc;
  char **my_argv;
  int option;
  int servo_num;
  int ignoreflag = 0;

  /* we need at least "gotk <script.tk>" */
  if (argc < 2) {
    fprintf(stderr, "gotk: need a script to run\n");
    return 1;
  }

  /* steps over gotk <script> so that -- is effectively the prog name */
#define ARGV_OFFSET 2

  /* if we have any args, -- and possibly more, process them */
  if (argc > 2) {
    my_argc = argc - ARGV_OFFSET;
    my_argv = &argv[ARGV_OFFSET];

    opterr = 0;
    while (1) {
      option = getopt(my_argc, my_argv, ":i:u:dkpx");
      if (option == -1)
	break;

      switch (option) {
      case 'i':
	strncpy(inifile_name, optarg, INIFILE_NAME_LEN);
	inifile_name[INIFILE_NAME_LEN - 1] = 0;
	break;

      case 'd':
	dbflag = 1;
	break;

      case 'k':
	killflag = 1;
	break;

      case 'p':
	peekflag = 1;
	break;

      case 'x':
	ignoreflag = 1;
	break;

      case ':':
	fprintf(stderr, "gotk: missing value for -%c\n", optopt);
	return 1;
	break;

      default:			/* '?' */
	fprintf (stderr, "gotk: unrecognized option -%c\n", optopt);
	return 1;
	break;
      }
    }
    if (optind < my_argc) {
      fprintf(stderr, "gotk: extra non-option characters: %s\n", argv[optind + 1]);
      return 1;
    }
  }

  if (ULAPI_OK != ulapi_init()) {
    fprintf(stderr, "gotk: can't init ulapi\n");
    return 1;
  } 

  if (go_init()) {
    fprintf(stderr, "gotk: go_init error\n");
    return 1;
  }

  mutex = ulapi_mutex_new(MUTEX_KEY);
  if (NULL == mutex) {
    fprintf(stderr, "gotk: can't allocate mutex\n");
    return 1;
  }

  /* get comm params from ini file */
  if (0 != ini_load(inifile_name)) {
    fprintf(stderr, "gotk: can't load ini file %s\n", inifile_name);
    return 1;
  }

  if (0 != get_comm_buffers() ||
      0 != update_comm_buffers()) {
    fprintf(stderr, "gotk: can't get comm buffers\n");
    if (! ignoreflag) {
      return 1;
    }
  }

  local_tvel = traj_set_ptr->max_tvel;
  local_tacc = traj_set_ptr->max_tacc;
  local_tjerk = traj_set_ptr->max_tjerk;

  local_rvel = traj_set_ptr->max_rvel;
  local_racc = traj_set_ptr->max_racc;
  local_rjerk = traj_set_ptr->max_rjerk;

  for (servo_num = 0; servo_num < SERVO_NUM; servo_num++) {
    local_jvel[servo_num] = servo_set_ptr[servo_num]->max_vel;
    local_jacc[servo_num] = servo_set_ptr[servo_num]->max_acc;
    local_jjerk[servo_num] = servo_set_ptr[servo_num]->max_jerk;
  }

  local_move_time = 0.0;

#ifdef HAVE_SDL
  init_joystick();
#endif

  /* Tcl,Tk_Main never return, so register all cleanup code with 'atexit' */
  signal(SIGINT, sigquit);
  atexit(myexit);
#ifdef BUILD_GOTCL
  Tcl_Main(argc, argv, Tcl_AppInit);
#else
  Tk_Main(argc, argv, Tk_AppInit);
#endif
  return 0;
}
