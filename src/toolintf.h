/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#ifndef TOOLINTF_H
#define TOOLINTF_H

#include "go.h"			/* go_pose */
#include "gorcs.h"		/* GO_RCS_CMD,STAT_MSG */

#define DEFAULT_TOOL_SHM_KEY 601

/* how many tools we support */
#define TOOL_MAX 16

typedef enum {
  TOOL_CMD_NOP_TYPE = TOOL_CMD_BASE + 1,
  TOOL_CMD_INIT_TYPE,
  TOOL_CMD_ABORT_TYPE,
  TOOL_CMD_SHUTDOWN_TYPE,
  TOOL_CMD_ON_TYPE,
  TOOL_CMD_OFF_TYPE,
} tool_cmd_type;

#define tool_cmd_symbol(x) \
(x) == TOOL_CMD_NOP_TYPE ? "NOP" : \
(x) == TOOL_CMD_INIT_TYPE ? "Init" : \
(x) == TOOL_CMD_ABORT_TYPE ? "Abort" : \
(x) == TOOL_CMD_SHUTDOWN_TYPE ? "Shutdown" : \
(x) == TOOL_CMD_ON_TYPE ? "On" : \
(x) == TOOL_CMD_OFF_TYPE ? "Off" : "?"

typedef enum {
  TOOL_STAT_TYPE = TOOL_STAT_BASE + 1
} tool_stat_type;

typedef enum {
  TOOL_CFG_NOP_TYPE = TOOL_CFG_BASE + 1,
  TOOL_CFG_CYCLE_TIME_TYPE,
  TOOL_CFG_DEBUG_TYPE
} tool_cfg_type;

#define tool_cfg_symbol(x) \
(x) == TOOL_CFG_NOP_TYPE ? "NOP" : \
(x) == TOOL_CFG_CYCLE_TIME_TYPE ? "CycleTime" : \
(x) == TOOL_CFG_DEBUG_TYPE ? "Debug" : "?"

typedef enum {
  TOOL_SET_TYPE = TOOL_SET_BASE + 1
} tool_set_type;

typedef struct {
  go_real value;		/*< the tool's output value */
} tool_cmd_on;

typedef struct {
  unsigned char head;
  GO_RCS_CMD_MSG;
  go_integer id;		/*< which tool type is targeted  */
  union {
    tool_cmd_on on;
  } u;
  unsigned char tail;
} tool_cmd_struct;

typedef struct {
  unsigned char head;
  GO_RCS_STAT_MSG;
  go_integer heartbeat;
  go_real cycle_time;		/*< actual cycle time */
  go_real value[TOOL_MAX];	/*< actual output values */
  unsigned char tail;
} tool_stat_struct;

typedef struct {
  go_real cycle_time;
} tool_cfg_cycle_time;

typedef struct {
  go_integer debug;
} tool_cfg_debug;

typedef struct {
  unsigned char head;
  GO_RCS_CMD_MSG;
  union {
    tool_cfg_cycle_time cycle_time;
    tool_cfg_debug debug;
  } u;
  unsigned char tail;
} tool_cfg_struct;

typedef struct {
  unsigned char head;
  GO_RCS_STAT_MSG;
  go_real cycle_time;		/*< nominal cycle time */
  go_integer debug;
  unsigned char tail;
} tool_set_struct;

typedef struct {
  tool_cmd_struct tool_cmd;
  tool_stat_struct tool_stat;
  tool_cfg_struct tool_cfg;
  tool_set_struct tool_set;
} tool_comm_struct;

#endif
