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
  \defgroup TASK Task Controller

  The Task Controller coordinates motion, tooling, program execution.

  The Task Controller follows the PackML state model shown in Figure 1.

  \image html packml_state_model.png
  Figure 1. PackML State Model.
*/

#ifndef TASKINTF_H
#define TASKINTF_H

#include <ulapi.h>		/* ulapi_real */
#include "go.h"			/* go_pose */
#include "gorcs.h"		/* GO_RCS_CMD,STAT_MSG */

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

#define DEFAULT_TASK_SHM_KEY 501
#define DEFAULT_TASK_TCP_PORT 8121

typedef enum {
  TASK_CMD_NOP_TYPE = TASK_CMD_BASE + 1,
  TASK_CMD_STOP_TYPE,
  TASK_CMD_START_TYPE,
  TASK_CMD_HOLD_TYPE,
  TASK_CMD_UNHOLD_TYPE,
  TASK_CMD_SUSPEND_TYPE,
  TASK_CMD_UNSUSPEND_TYPE,
  TASK_CMD_RESET_TYPE,
  TASK_CMD_ABORT_TYPE,
  TASK_CMD_CLEAR_TYPE,
  TASK_CMD_SHUTDOWN_TYPE,
  /*
    the following are not commands to tasks, but instructions to task
    that may be placed on the interp list
  */
  TASK_EXEC_DELAY_TYPE,
  TASK_EXEC_WAIT_FOR_MOTION_TYPE,
  TASK_EXEC_WAIT_FOR_TOOL_TYPE,
} task_cmd_type;

typedef enum {
  TASK_STAT_TYPE = TASK_STAT_BASE + 1
} task_stat_type;

typedef enum {
  TASK_CFG_NOP_TYPE = TASK_CFG_BASE + 1,
  TASK_CFG_CYCLE_TIME_TYPE,
  TASK_CFG_DEBUG_TYPE,
  TASK_CFG_STRICT_TYPE,
  TASK_CFG_PROG_DIR_TYPE,
} task_cfg_type;

typedef enum {
  TASK_SET_TYPE = TASK_SET_BASE + 1
} task_set_type;

enum {
  TASK_CMD_PROGRAM_LEN = 256
};

typedef struct {
  /*! a string containing the script name and any args */
  char program[TASK_CMD_PROGRAM_LEN];
} task_cmd_start;

typedef struct {
  /*! time, in seconds, to delay before executing the next item */
  ulapi_real time;
} task_exec_delay;

typedef struct {
  unsigned char head;
  GO_RCS_CMD_MSG;
  union {
    task_cmd_start start;
    task_exec_delay delay;
  } u;
  unsigned char tail;
} task_cmd_struct;

typedef enum {
  TASK_STATE_IDLE,
  TASK_STATE_STARTING,
  TASK_STATE_EXECUTE,
  TASK_STATE_HOLDING,
  TASK_STATE_HELD,
  TASK_STATE_UNHOLDING,
  TASK_STATE_SUSPENDING,
  TASK_STATE_SUSPENDED,
  TASK_STATE_UNSUSPENDING,
  TASK_STATE_COMPLETING,
  TASK_STATE_COMPLETE,
  TASK_STATE_ABORTING,
  TASK_STATE_ABORTED,
  TASK_STATE_CLEARING,
  TASK_STATE_STOPPING,
  TASK_STATE_STOPPED,
  TASK_STATE_RESETTING,
} task_state_model_type;

typedef enum {
  TASK_ERROR_NONE,
  TASK_ERROR_UNKNOWN_COMMAND,
  TASK_ERROR_IMPROPER_COMMAND,
  TASK_ERROR_INVALID_COMMAND,
  TASK_ERROR_MOTION,
  TASK_ERROR_PROGRAM_NOT_FOUND,
  TASK_ERROR_OUT_OF_MEMORY,
  TASK_ERROR_PROGRAM_ERROR,
  TASK_ERROR_CONTROL,
  TASK_ERROR_TOOL,
} task_error_code;

typedef struct {
  ulapi_real timestamp;
  task_error_code code;
} task_error;

#define TASK_ERROR_MAX 10

typedef struct {
  unsigned char head;
  GO_RCS_STAT_MSG;
  go_integer heartbeat;
  go_real cycle_time;		/*< actual cycle time */
  char program[TASK_CMD_PROGRAM_LEN];
  task_state_model_type state_model;
  task_error error[TASK_ERROR_MAX];
  go_integer error_index;		/*< index of oldest error */
  unsigned char tail;
} task_stat_struct;

typedef struct {
  go_real cycle_time;
} task_cfg_cycle_time;

typedef struct {
  go_integer debug;
} task_cfg_debug;

typedef struct {
  go_flag strict;
} task_cfg_strict;

typedef struct {
  char prog_dir[TASK_CMD_PROGRAM_LEN];
} task_cfg_prog_dir;

typedef struct {
  unsigned char head;
  GO_RCS_CMD_MSG;
  union {
    task_cfg_cycle_time cycle_time;
    task_cfg_debug debug;
    task_cfg_strict strict;
    task_cfg_prog_dir prog_dir;
  } u;
  unsigned char tail;
} task_cfg_struct;

typedef struct {
  unsigned char head;
  GO_RCS_STAT_MSG;
  go_real cycle_time;		/*< nominal cycle time */
  go_integer debug;
  go_flag strict;		/*< non-zero if strict state model transitions are enforced  */
  char prog_dir[TASK_CMD_PROGRAM_LEN];
  unsigned char tail;
} task_set_struct;

typedef struct {
  task_cmd_struct task_cmd;
  task_stat_struct task_stat;
  task_cfg_struct task_cfg;
  task_set_struct task_set;
} task_comm_struct;

extern const char *task_cmd_symbol(task_cmd_type tc);
extern const char *task_cfg_symbol(task_cfg_type tc);
extern const char *task_state_model_symbol(task_state_model_type tsm);
extern const char *task_error_symbol(task_error_code te);

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif
