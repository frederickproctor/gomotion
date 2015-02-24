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
  \file ext_stub.c

  \brief Stubbed external interface implementation that does nothing. 
  Fill this in with the relevant code for your particular board as
  a starting point.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <math.h>
#include "gotypes.h"
#include "extintf.h"

#define NUM_JOINTS 8

go_result ext_init(char * init_string)
{
  return GO_RESULT_OK;
}
 
go_result ext_quit(void)
{
  return GO_RESULT_OK;
}

go_result ext_joint_init(go_integer joint, go_real cycle_time)
{
  if (joint < 0 || joint >= NUM_JOINTS) return GO_RESULT_ERROR;

  return GO_RESULT_OK;
}

go_result ext_joint_enable(go_integer joint)
{
  return GO_RESULT_OK;
}

go_result ext_joint_disable(go_integer joint)
{
  return GO_RESULT_OK;
}

go_result ext_joint_quit(go_integer joint)
{
  return GO_RESULT_OK;
}

go_result ext_read_pos(go_integer joint, go_real * pos)
{
  if (joint < 0 || joint >= NUM_JOINTS) return GO_RESULT_ERROR;

  *pos = 0.0;

  return GO_RESULT_OK;
}

go_result ext_write_pos(go_integer joint, go_real pos)
{
  return GO_RESULT_IMPL_ERROR;
}

go_result ext_write_vel(go_integer joint, go_real vel)
{
  if (joint < 0 || joint >= NUM_JOINTS) return GO_RESULT_ERROR;

  return GO_RESULT_OK;
}

go_result ext_joint_home(go_integer joint)
{
  if (joint < 0 || joint >= NUM_JOINTS) return GO_RESULT_ERROR;
  
  return GO_RESULT_OK;
}

go_flag ext_joint_is_home(go_integer joint)
{
  if (joint < 0 || joint >= NUM_JOINTS) return 1; /* always homed */

  return 1;			/* always homed */
}

go_result ext_joint_home_latch(go_integer joint, go_real * pos)
{
  if (joint < 0 || joint >= NUM_JOINTS) return GO_RESULT_ERROR;

  *pos = 0.0;

  return GO_RESULT_OK;
}

enum {AIN_NUM = 0};
enum {AOUT_NUM = 0};
enum {DIN_NUM = 0};
enum {DOUT_NUM = 0};

go_integer ext_num_ain(void)
{
  return AIN_NUM;
}

go_integer ext_num_aout(void)
{
  return AOUT_NUM;
}

go_integer ext_num_din(void)
{
  return DIN_NUM;
}

go_integer ext_num_dout(void)
{
  return DOUT_NUM;
}

go_result ext_trigger_in(void)
{
  return GO_RESULT_OK;
}

go_result ext_read_ain(go_integer index, go_real * val)
{
  *val = 0.0;

  return GO_RESULT_OK;
}

go_result ext_write_aout(go_integer index, go_real val)
{
  return GO_RESULT_OK;
}

go_result ext_read_din(go_integer index, go_flag * val)
{
  *val = 0;

  return GO_RESULT_OK;
}

go_result ext_write_dout(go_integer index, go_flag val)
{
  return GO_RESULT_OK;
}

go_result ext_set_parameters(go_integer joint, go_real * values, go_integer number)
{
  /* nothing to do */
  return GO_RESULT_OK;
}
