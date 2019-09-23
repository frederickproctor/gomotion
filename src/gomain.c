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
  \file gomain.c

  \brief Starts up the servo and traj tasks.
*/

/*!
  \defgroup GOMAIN Starting a Go Motion Controller

  A Go Motion controller is comprised of the trajectory loop and one
  or more servo loops. Depending on the platform, the controller may
  be a single executable (e.g., \c gomain or \c gomain.exe for Unix or
  Windows) or a kernel modules (e.g., \c gomain_mod.o or \c
  gomain_mod.ko for Real-Time Linux). These will be referred to as
  <b>Go Main</b>. As much as possible, the configuration of the controller
  is deferred until after it begins running, and is done by the \ref
  INICFG <b>Go
  Configurator</b>, e.g., \c gocfg.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <stddef.h>		/* NULL, sizeof */
#include <rtapi.h>
#include <rtapi_app.h>
#include "extintf.h"		/* ext_init,quit */
#include "golog.h"		/* go_log_struct */
#include "goio.h"		/* go_io_struct, global_go_io_ptr */
#include "servointf.h"		/* servoLoop, servoComm */
#include "trajintf.h"

/*!
  NOMINAL_PERIOD_NSEC is the nominal period of the servo and traj
  control tasks. It will be set to be longer than this based on
  the cycle times from the .ini file later during the config process.
 */
#define NOMINAL_PERIOD_NSEC 1000000

static void * servo_task[SERVO_NUM] = {NULL};
#define SERVO_STACKSIZE 4000	/* 1424 measured, + 640 for IO */

static traj_arg_struct traj_args;
static void * traj_task = NULL;
#define TRAJ_STACKSIZE 57000	/* 28416 measured */

static void * servo_shm = NULL;
static void * traj_shm = NULL;
static void * go_log_shm = NULL;
static void * go_io_shm = NULL;

/* the global log */
go_log_struct * global_go_log_ptr = NULL;

/* the global IO structure */
go_io_struct * global_go_io_ptr = NULL;

/* declare comm params that aren't set via the config process later */
RTAPI_DECL_INT(DEBUG, 0);
RTAPI_DECL_INT(TRAJ_SHM_KEY, 201);
RTAPI_DECL_INT(SERVO_HOWMANY, SERVO_NUM);
RTAPI_DECL_INT(SERVO_SHM_KEY, 101);
RTAPI_DECL_INT(SERVO_SEM_KEY, 101);
RTAPI_DECL_STRING(EXT_INIT_STRING, "");
RTAPI_DECL_STRING(KINEMATICS, "trivkins");
RTAPI_DECL_INT(GO_LOG_SHM_KEY, 1001);
RTAPI_DECL_INT(GO_IO_SHM_KEY, 1002);

int rtapi_app_main(RTAPI_APP_ARGS_DECL)
{
  int servo_num;
  int servo_prio;
  int traj_prio;
  int t;

  if (0 != rtapi_app_init(RTAPI_APP_ARGS)) {
    rtapi_print("gomain: can't initialize\n");
    return 1;
  }

  /* get command line args */

  (void) rtapi_arg_get_int(&DEBUG, "DEBUG");
  if (DEBUG) rtapi_print("gomain: using DEBUG = %d\n", DEBUG);

  if (DEBUG) {
    rtapi_print("gomain: %d arguments:\n", rtapi_argc);
    for (t = 0; t < rtapi_argc; t++) {
      rtapi_print("gomain:   %s\n", rtapi_argv[t]);
    }
  }

  (void) rtapi_arg_get_int(&TRAJ_SHM_KEY, "TRAJ_SHM_KEY");
  if (DEBUG) rtapi_print("gomain: using TRAJ_SHM_KEY = %d\n", TRAJ_SHM_KEY);
  (void) rtapi_arg_get_int(&SERVO_HOWMANY, "SERVO_HOWMANY");
  if (DEBUG) rtapi_print("gomain: using SERVO_HOWMANY = %d\n", SERVO_HOWMANY);
  (void) rtapi_arg_get_int(&SERVO_SHM_KEY, "SERVO_SHM_KEY");
  if (DEBUG) rtapi_print("gomain: using SERVO_SHM_KEY = %d\n", SERVO_SHM_KEY);
  (void) rtapi_arg_get_int(&SERVO_SEM_KEY, "SERVO_SEM_KEY");
  if (DEBUG) rtapi_print("gomain: using SERVO_SEM_KEY = %d\n", SERVO_SEM_KEY);
  (void) rtapi_arg_get_string(&EXT_INIT_STRING, "EXT_INIT_STRING");
  if (DEBUG) rtapi_print("gomain: using EXT_INIT_STRING = %s\n", EXT_INIT_STRING);
  (void) rtapi_arg_get_string(&KINEMATICS, "KINEMATICS");
  if (DEBUG) rtapi_print("gomain: using KINEMATICS = %s\n", KINEMATICS);
  (void) rtapi_arg_get_int(&GO_LOG_SHM_KEY, "GO_LOG_SHM_KEY");
  if (DEBUG) rtapi_print("gomain: using GO_LOG_SHM_KEY = %d\n", GO_LOG_SHM_KEY);
  (void) rtapi_arg_get_int(&GO_IO_SHM_KEY, "GO_IO_SHM_KEY");
  if (DEBUG) rtapi_print("gomain: using GO_IO_SHM_KEY = %d\n", GO_IO_SHM_KEY);

  /* need at least the first servo task to clock the semaphore */
  if (SERVO_HOWMANY < 1) SERVO_HOWMANY = 1;
  else if (SERVO_HOWMANY > SERVO_NUM) SERVO_HOWMANY = SERVO_NUM;

  if (DEBUG) rtapi_print("gomain running off base clock period %d\n", rtapi_clock_period);

  /* allocate the servo comm buffers */
  servo_shm = rtapi_rtm_new(SERVO_SHM_KEY, SERVO_NUM * sizeof(servo_comm_struct));
  if (NULL == servo_shm) {
    rtapi_print("can't get servo comm shm\n");
    return 1;
  }
  global_servo_comm_ptr = rtapi_rtm_addr(servo_shm);
  for (servo_num = 0; servo_num < SERVO_NUM; servo_num++) {
    /* set them to be different, so we can tell when they're running */
    global_servo_comm_ptr[servo_num].servo_cmd.head = 1;
    global_servo_comm_ptr[servo_num].servo_cmd.tail = 2;
    global_servo_comm_ptr[servo_num].servo_stat.head = 1;
    global_servo_comm_ptr[servo_num].servo_stat.tail = 2;
    global_servo_comm_ptr[servo_num].servo_cfg.head = 1;
    global_servo_comm_ptr[servo_num].servo_cfg.tail = 2;
    global_servo_comm_ptr[servo_num].servo_set.head = 1;
    global_servo_comm_ptr[servo_num].servo_set.tail = 2;
  }

  /* allocate the traj comm buffer */
  traj_shm = rtapi_rtm_new(TRAJ_SHM_KEY, sizeof(traj_comm_struct));
  if (NULL == traj_shm) {
    rtapi_print("can't get traj comm shm\n");
    return 1;
  }
  global_traj_comm_ptr = rtapi_rtm_addr(traj_shm);

  /* allocate the log buffer */
  go_log_shm = rtapi_rtm_new(GO_LOG_SHM_KEY, sizeof(go_log_struct));
  if (NULL == go_log_shm) {
    rtapi_print("can't get go log shm\n");
    return 1;
  }
  global_go_log_ptr = rtapi_rtm_addr(go_log_shm);
  go_log_init(global_go_log_ptr, GO_LOG_NONE, 0, 1);

  /* allocate the IO buffer */
  go_io_shm = rtapi_rtm_new(GO_IO_SHM_KEY, sizeof(go_io_struct));
  if (NULL == go_io_shm) {
    rtapi_print("can't get go io shm\n");
    return 1;
  }
  global_go_io_ptr = rtapi_rtm_addr(go_io_shm);

  /* initialize the servo task semaphore used to clock traj */
  if (NULL == (servo_sem = rtapi_sem_new((rtapi_id) SERVO_SEM_KEY))) {
    rtapi_print("can't get servo task semaphore\n");
    return 1;
  }
  rtapi_sem_give(servo_sem);

  /* we'll rely on the HAL to set the base timer period. */

  /* set prios as servo, then traj */
  servo_prio = rtapi_prio_next_lower(rtapi_prio_highest());
  traj_prio = rtapi_prio_next_lower(servo_prio);

  /* initialize the external interface */
  ext_init(EXT_INIT_STRING);

  /* launch just the servo tasks we need */
  for (servo_num = 0; servo_num < SERVO_NUM; servo_num++) {
    if (servo_num < SERVO_HOWMANY) {
      servo_task[servo_num] = rtapi_task_new();
      if (0 == servo_task[servo_num]) {
	rtapi_print("can't allocate servo task %d\n", servo_num + 1);
	return 1;
      }
      if (0 != rtapi_task_start(servo_task[servo_num], 
				servo_loop, 
				(void *) servo_num, 
				servo_prio,
				SERVO_STACKSIZE,
				NOMINAL_PERIOD_NSEC, 
				1)) { /* 1 = floating point */
	rtapi_print("can't start servo task %d\n", servo_num + 1);
	return 1;
      }
      if (DEBUG) rtapi_print("started servo %d task %x\n", servo_num + 1, servo_task[servo_num]);
    } else {
      servo_task[servo_num] = NULL;
      if (DEBUG) rtapi_print("not starting unused servo task %d\n", servo_num + 1);
    }
  }

  /* launch the traj task */

  /* first, fill in some things we know that traj wants */

  traj_task = rtapi_task_new();
  if (NULL == traj_task) {
    rtapi_print("can't allocate traj task\n");
    return 1;
  }
  if (DEBUG) rtapi_print("allocated traj task\n");

  traj_args.joint_num = SERVO_HOWMANY;

  /* get the kinematics */
  if (GO_RESULT_OK != go_kin_select(KINEMATICS)) {
    rtapi_print("can't select kinematics ``%s''\n", KINEMATICS);
    return 1;
  }
  /* now that we've selected the kinematics, we can get their size, etc */
  traj_args.kinematics = rtapi_new(go_kin_size());
  if (NULL == traj_args.kinematics) {
    rtapi_print("can't allocate kinematics\n");
    return 1;
  }
  if (DEBUG) rtapi_print("allocated kinematics\n");
  if (GO_RESULT_OK != go_kin_init(traj_args.kinematics)) {
    rtapi_print("can't initialize kinematics\n");
    return 1;
  }
  if (DEBUG) rtapi_print("initialized kinematics\n");

  if (0 != rtapi_task_start(traj_task,
			    traj_loop,
			    (void *) &traj_args,
			    traj_prio, 
			    TRAJ_STACKSIZE,
			    NOMINAL_PERIOD_NSEC,
			    1)) { /* 1 = floating point */
    rtapi_print("can't start traj task\n");
    return 1;
  }
  if (DEBUG) rtapi_print("started traj task %x\n", traj_task);

  if (DEBUG) rtapi_print("gomain started\n");

  return rtapi_app_wait();
}

void rtapi_app_exit(void)
{
  int unused;
  int servo_num;

  if (NULL != traj_task) {
    unused = rtapi_task_stack_check(traj_task);
    if (0 == unused) {
      rtapi_print("traj stack overwritten\n");
    } else if (unused > 0) {
      if (DEBUG) rtapi_print("%d unused traj stack words\n", unused);
    } /* else the stack check is irrelevant */
    (void) rtapi_task_stop(traj_task);
    (void) rtapi_task_delete(traj_task);
    traj_task = 0;
  }
  rtapi_free(traj_args.kinematics);

  for (servo_num = 0; servo_num < SERVO_NUM; servo_num++) {
    if (NULL != servo_task[servo_num]) {
      unused = rtapi_task_stack_check(servo_task[servo_num]);
      if (0 == unused) {
	rtapi_print("servo %d stack overwritten\n", servo_num + 1);
      } else if (unused > 0) {
	if (DEBUG) rtapi_print("%d unused servo %d stack words\n", unused, servo_num + 1);
      }	/* else the stack check is irrelevant */
      if (DEBUG) rtapi_print("servo task %d stopped\n", servo_num + 1);
      (void) rtapi_task_stop(servo_task[servo_num]);
      (void) rtapi_task_delete(servo_task[servo_num]);
      servo_task[servo_num] = 0;
    }
  }

  if (NULL != traj_shm) {
    rtapi_rtm_delete(traj_shm);
    traj_shm = NULL;
  }
  global_traj_comm_ptr = NULL;

  if (NULL != servo_shm) {
    rtapi_rtm_delete(servo_shm);
    servo_shm = NULL;
  }
  global_servo_comm_ptr = NULL;

  if (NULL != go_log_shm) {
    rtapi_rtm_delete(go_log_shm);
    go_log_shm = NULL;
  }
  global_go_log_ptr = NULL;

  if (NULL != go_io_shm) {
    rtapi_rtm_delete(go_io_shm);
    go_io_shm = NULL;
  }
  global_go_io_ptr = NULL;

  rtapi_sem_delete(servo_sem);

  if (DEBUG) rtapi_print("gomain done\n");

  ext_quit();

  return;
}
