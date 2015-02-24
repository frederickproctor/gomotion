/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#ifndef GORCS_H
#define GORCS_H

#include <stddef.h>		/* sizeof */
#include "gotypes.h"		/* go_integer */
#include "goutil.h"		/* go_strncpy */

enum GO_RCS_STATE {
  GO_RCS_STATE_UNINITIALIZED = 0,
  GO_RCS_STATE_NEW_COMMAND = 1,
  GO_RCS_STATE_S0 = 10,
  GO_RCS_STATE_S1,
  GO_RCS_STATE_S2,
  GO_RCS_STATE_S3,
  GO_RCS_STATE_S4,
  GO_RCS_STATE_S5,
  GO_RCS_STATE_S6,
  GO_RCS_STATE_S7,
  GO_RCS_STATE_S8,
  GO_RCS_STATE_S9,
  GO_RCS_STATE_LAST, /* set your named states to begin at least here */
};

enum GO_RCS_ADMIN_STATE {
  GO_RCS_ADMIN_STATE_UNINITIALIZED = 1,
  GO_RCS_ADMIN_STATE_INITIALIZED = 2,
  GO_RCS_ADMIN_STATE_SHUT_DOWN = 3
};

enum GO_RCS_STATUS {
  GO_RCS_STATUS_UNINITIALIZED = 0,
  GO_RCS_STATUS_DONE,
  GO_RCS_STATUS_EXEC,
  GO_RCS_STATUS_ERROR
};

#define GO_RCS_CMD_MSG \
  go_integer type; \
  go_integer serial_number

#define GO_RCS_STAT_SOURCE_FILE_LEN 64

#define GO_RCS_STAT_MSG \
  go_integer type; \
  go_integer command_type; \
  go_integer echo_serial_number; \
  go_integer status; \
  go_integer state; \
  go_integer admin_state; \
  go_integer line; \
  go_integer source_line; \
  char source_file[GO_RCS_STAT_SOURCE_FILE_LEN]

#define go_state_match(s,a) (s)->line = (s)->source_line = __LINE__, (s)->state == (a)
/* FIXME-- need a portable (kernel) strncpy */
#define go_state_new(s) go_strncpy((s)->source_file, __FILE__, GO_RCS_STAT_SOURCE_FILE_LEN), (s)->source_file[GO_RCS_STAT_SOURCE_FILE_LEN - 1] = 0
#define go_state_next(s,a) (s)->state = (a)
#define go_status_next(s,a) (s)->status = (a)
#define go_state_default(s) (s)->line = (s)->source_line = __LINE__

#define COMM_BASE 1000

#define SERVO_BASE (COMM_BASE + 1000)
#define TRAJ_BASE (COMM_BASE + 2000)
#define TASK_BASE (COMM_BASE + 3000)
#define TOOL_BASE (COMM_BASE + 4000)

#define TASK_CMD_BASE (TASK_BASE + 100)
#define TASK_STAT_BASE (TASK_BASE + 200)
#define TASK_CFG_BASE (TASK_BASE + 300)
#define TASK_SET_BASE (TASK_BASE + 400)

#define TOOL_CMD_BASE (TOOL_BASE + 100)
#define TOOL_STAT_BASE (TOOL_BASE + 200)
#define TOOL_CFG_BASE (TOOL_BASE + 300)
#define TOOL_SET_BASE (TOOL_BASE + 400)

#define TRAJ_CMD_BASE (TRAJ_BASE + 100)
#define TRAJ_STAT_BASE (TRAJ_BASE + 200)
#define TRAJ_CFG_BASE (TRAJ_BASE + 300)
#define TRAJ_SET_BASE (TRAJ_BASE + 400)

#define SERVO_CMD_BASE (SERVO_BASE + 100)
#define SERVO_STAT_BASE (SERVO_BASE + 200)
#define SERVO_CFG_BASE (SERVO_BASE + 300)
#define SERVO_SET_BASE (SERVO_BASE + 400)

#define DEBUG_NONE     0x00000000
#define DEBUG_CMD      0x00000001
#define DEBUG_CFG      0x00000002
#define DEBUG_POSITION 0x00000004
#define DEBUG_PROG     0x00000008
#define DEBUG_HOME     0x00000010
#define DEBUG_PERF     0x00000020
#define DEBUG_TASK     0x00000040
#define DEBUG_TOOL     0x00000080

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif
