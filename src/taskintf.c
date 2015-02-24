/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#include "taskintf.h"

const char *task_cmd_symbol(task_cmd_type tc)
{
  switch (tc) {
  case TASK_CMD_NOP_TYPE:
    return "NOP";
    break;
  case TASK_CMD_STOP_TYPE:
    return "Stop";
    break;
  case TASK_CMD_START_TYPE:
    return "Start";
    break;
  case TASK_CMD_HOLD_TYPE:
    return "Hold";
    break;
  case TASK_CMD_UNHOLD_TYPE:
    return "Unhold";
    break;
  case TASK_CMD_SUSPEND_TYPE:
    return "Suspend";
    break;
  case TASK_CMD_UNSUSPEND_TYPE:
    return "Unsuspend";
    break;
  case TASK_CMD_RESET_TYPE:
    return "Reset";
    break;
  case TASK_CMD_ABORT_TYPE:
    return "Abort";
    break;
  case TASK_CMD_CLEAR_TYPE:
    return "Clear";
    break;
  case TASK_CMD_SHUTDOWN_TYPE:
    return "Shutdown";
    break;
  case TASK_EXEC_DELAY_TYPE:
    return "Delay";
    break;
  case TASK_EXEC_WAIT_FOR_MOTION_TYPE:
    return "Wait For Motion";
    break;
  case TASK_EXEC_WAIT_FOR_TOOL_TYPE:
    return "Wait For Tool";
    break;
  default:
    return "?";
    break;
  }

  return "?";
}

const char *task_cfg_symbol(task_cfg_type tc)
{
  switch (tc) {
  case TASK_CFG_NOP_TYPE:
    return "Nop";
    break;
  case TASK_CFG_CYCLE_TIME_TYPE:
    return "CycleTime";
    break;
  case TASK_CFG_DEBUG_TYPE:
    return "Debug";
    break;
  case TASK_CFG_STRICT_TYPE:
    return "Strict";
    break;
  case TASK_CFG_PROG_DIR_TYPE:
    return "ProgramDirectory";
    break;
  default:
    return "?";
    break;
  }

  return "?";
}

const char *task_state_model_symbol(task_state_model_type tsm)
{
  switch (tsm) {
  case TASK_STATE_IDLE:
    return "Idle";
    break;
  case TASK_STATE_STARTING:
    return "Starting";
    break;
  case TASK_STATE_EXECUTE:
    return "Execute";
    break;
  case TASK_STATE_HOLDING:
    return "Holding";
    break;
  case TASK_STATE_HELD:
    return "Held";
    break;
  case TASK_STATE_UNHOLDING:
    return "Unholding";
    break;
  case TASK_STATE_SUSPENDING:
    return "Suspending";
    break;
  case TASK_STATE_SUSPENDED:
    return "Suspended";
    break;
  case TASK_STATE_UNSUSPENDING:
    return "Unsuspending";
    break;
  case TASK_STATE_COMPLETING:
    return "Completing";
    break;
  case TASK_STATE_COMPLETE:
    return "Complete";
    break;
  case TASK_STATE_ABORTING:
    return "Aborting";
    break;
  case TASK_STATE_ABORTED:
    return "Aborted";
    break;
  case TASK_STATE_CLEARING:
    return "Clearing";
    break;
  case TASK_STATE_STOPPING:
    return "Stopping";
    break;
  case TASK_STATE_STOPPED:
    return "Stopped";
    break;
  case TASK_STATE_RESETTING:
    return "Resetting";
    break;
  default:
    return "?";
    break;
  }

  return "?";
}

const char *task_error_symbol(task_error_code te)
{
  switch (te) {
  case TASK_ERROR_NONE:
    return "None";
    break;
  case TASK_ERROR_UNKNOWN_COMMAND:
    return "Unknown command";
    break;
  case TASK_ERROR_IMPROPER_COMMAND:
    return "Improper command";
    break;
  case TASK_ERROR_MOTION:
    return "Motion controller error";
    break;
  case TASK_ERROR_PROGRAM_NOT_FOUND:
    return "Program not found";
    break;
  case TASK_ERROR_OUT_OF_MEMORY:
    return "Out of memory";
    break;
  case TASK_ERROR_PROGRAM_ERROR:
    return "Program error";
    break;
  case TASK_ERROR_CONTROL:
    return "Control error";
    break;
  default:
    return "?";
    break;
  }

  return "?";
}

