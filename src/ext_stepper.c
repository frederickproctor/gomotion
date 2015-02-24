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
  \file ext_stepper.c

  \brief External interface implementation for stepper motors.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stddef.h>		/* NULL */
#include <math.h>		/* fabs */
#include <rtapi.h>
#include "gotypes.h"
#include "extintf.h"
#include "gostepper.h"

static int gss_shm_key;		/* corresponds to GO_STEPPER_SHM_KEY */
static void * gss_shm;
static go_stepper_struct dummy;	/* so it's never null, avoiding checks */
static go_stepper_struct * gss_ptr = &dummy;

/*
  Normally the stepper returns a home condition right away, at
  the current position. There is no index pulse that needs to
  be latched. For testing, we can simulate an index pulse that
  occurs every so many counts.

  Defining SIMULATE_INDEX_PULSE will do this simulation, with index
  pulses occurring every INDEX_PULSE_COUNTS counts.
*/
#undef SIMULATE_INDEX_PULSE

#ifdef SIMULATE_INDEX_PULSE
/* how many counts between simulated index pulses */
#define INDEX_PULSE_COUNT 100
/* converts counts into which rev they lie within */
#define CALC_REV(i) ((i) < 0 ? (((i) + 1 - INDEX_PULSE_COUNT) / INDEX_PULSE_COUNT) : ((i) / INDEX_PULSE_COUNT))
/* saved rev at which the homing was initiated */
static go_integer joint_home_start_rev[GO_STEPPER_NUM];
/* where the latched home took place, in counts */
static go_integer joint_latch[GO_STEPPER_NUM];
#endif

static go_flag joint_is_homed[GO_STEPPER_NUM];
static go_real joint_cycle_time[GO_STEPPER_NUM];

go_result ext_init(char * init_string)
{
  go_integer joint;

  /* parse the init string for our shared memory buffer number */
  rtapi_print("ext_init(%s)\n", init_string);
  if (RTAPI_OK != rtapi_string_to_integer(init_string, &gss_shm_key)) {
    /* use default key */
    gss_shm_key = GO_STEPPER_DEFAULT_SHM_KEY;
  }
  rtapi_print("ext_init using go_stepper shm key %d\n", gss_shm_key);

  /* allocate the shared memory buffer */
  gss_shm = rtapi_rtm_new(gss_shm_key, sizeof(go_stepper_struct));
  if (NULL == gss_shm) {
    rtapi_print("can't get stepper shm\n");
    return GO_RESULT_ERROR;
  }
  gss_ptr = rtapi_rtm_addrr(gss_shm);

  for (joint = 0; joint < GO_STEPPER_NUM; joint++) {
    /* clear the frequency but leave gss_ptr->count[] alone, since
       it's set by stepper task */
    gss_ptr->freq[joint] = 0;
    /* store the starting count as the old count */
    joint_is_homed[joint] = 0;
    joint_cycle_time[joint] = 1.0;
#ifdef SIMULATE_INDEX_PULSE
    joint_home_start_rev[joint] = CALC_REV(gss_ptr->count[joint]);
    joint_latch[joint] = joint_home_start_rev[joint] * INDEX_PULSE_COUNT;
#endif
  }

  return GO_RESULT_OK;
}
 
go_result ext_quit(void)
{
  if (NULL != gss_shm) {
    rtapi_rtm_delete(gss_shm);
    gss_shm = NULL;
  }
  gss_ptr = &dummy;

  return GO_RESULT_OK;
}

go_result ext_joint_init(go_integer joint, go_real cycle_time)
{
  if (joint < 0 || joint >= GO_STEPPER_NUM ||
      cycle_time <= 0.0) return GO_RESULT_ERROR;

  joint_is_homed[joint] = 0;
  joint_cycle_time[joint] = cycle_time;

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
  if (joint < 0 || joint >= GO_STEPPER_NUM) return GO_RESULT_ERROR;

  *pos = (go_real) gss_ptr->count[joint];

  return GO_RESULT_OK;
}

#define IS_SMALL(x) fabs(x) < GO_REAL_EPSILON
#define PERIOD 50000

go_result ext_write_pos(go_integer joint, go_real pos)
{
  return GO_RESULT_IMPL_ERROR;
}

/*
  'vel' is the raw output in raw units per second
*/

go_result ext_write_vel(go_integer joint, go_real vel)
{
  if (joint < 0 || joint >= GO_STEPPER_NUM) return GO_RESULT_ERROR;

  gss_ptr->freq[joint] = (rtapi_integer) vel;

  return GO_RESULT_OK;
}

go_result ext_joint_home(go_integer joint)
{
  if (joint < 0 || joint >= GO_STEPPER_NUM) return GO_RESULT_ERROR;

#ifdef SIMULATE_INDEX_PULSE
    joint_is_homed[joint] = 0;
    joint_home_start_rev[joint] = CALC_REV(gss_ptr->count[joint]);
#else
    joint_is_homed[joint] = 1;
#endif
  
  return GO_RESULT_OK;
}

go_flag ext_joint_is_home(go_integer joint)
{
#ifdef SIMULATE_INDEX_PULSE
  go_integer rev;
#endif

  if (joint < 0 || joint >= GO_STEPPER_NUM) return 0;

#ifdef SIMULATE_INDEX_PULSE
  rev = CALC_REV(gss_ptr->count[joint]);
  if (rev > joint_home_start_rev[joint]) {
    /* we passed the index pulse going in the positive direction */
    joint_latch[joint] = (joint_home_start_rev[joint] + 1) * INDEX_PULSE_COUNT;
    joint_is_homed[joint] = 1;
  } else if (rev < joint_home_start_rev[joint]) {
    /* we passed going negative */
    joint_latch[joint] = joint_home_start_rev[joint] * INDEX_PULSE_COUNT;
    joint_is_homed[joint] = 1;
  } /* else we haven't passed it yet */
#endif

  return joint_is_homed[joint];
}

go_result ext_joint_home_latch(go_integer joint, go_real * pos)
{
  if (joint < 0 || joint >= GO_STEPPER_NUM) return GO_RESULT_ERROR;

#ifdef SIMULATE_INDEX_PULSE
  *pos = (go_real) joint_latch[joint];
#else
  *pos = (go_real) gss_ptr->count[joint]; /* return "here" */
#endif
  
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
