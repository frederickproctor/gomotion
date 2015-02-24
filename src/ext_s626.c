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
  \file ext_s626.c

  \brief External interface functions for the Sensoray 626 board, with
  simulated motors standing in for joints that are not to be controlled
  by the S626.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <math.h>
#include <rtapi.h>
#include "gotypes.h"
#include "extintf.h"
#include "dcmotor.h"

#define NUM_BOARDS 1

#ifdef HAVE_S626

#include "s626drv.h"
#include "App626.h"
#include "s626mod.h"
#include "s626core.h"
#include "s626api.h"

/* flags for when to use real S626 board, otherwise DC motor simulation */
static int UseS626(int joint)
{
  if (joint < 0 ||
      joint > 5) return 0;

  return 1;
}

static void InterruptNullISR(DWORD board)
{
  return;
}

/* print error code */

static void ErrorFunctionNull(DWORD ErrFlags)
{
  return;
}

#define COUNTER_OFFSET 0x800000

static void CreateEncoderCounter(HBD hbd, WORD Counter)
{
  S626_CounterModeSet(hbd, Counter,
		      ( LOADSRC_INDX << BF_LOADSRC ) |
		      ( INDXSRC_SOFT << BF_INDXSRC ) |
		      ( CLKSRC_COUNTER << BF_CLKSRC ) |
		      ( CLKPOL_POS  << BF_CLKPOL ) |
		      ( CLKMULT_1X   << BF_CLKMULT ) |
		      ( CLKENAB_INDEX << BF_CLKENAB ) );

  S626_CounterPreload( hbd, Counter, COUNTER_OFFSET );
  S626_CounterSoftIndex( hbd, Counter );
  S626_CounterLatchSourceSet( hbd, Counter, LATCHSRC_AB_READ );
  S626_CounterEnableSet( hbd, Counter, CLKENAB_ALWAYS );
}

static int IndexToCounter(int index)
{
  if (index <= 0) return CNTR_0A;
  else if (index == 1) return CNTR_1A;
  else if (index == 2) return CNTR_2A;
  else if (index == 3) return CNTR_0B;
  else if (index == 4) return CNTR_1B;
  else return CNTR_2B;
}

#endif
/* HAVE_S626 */

#define NUM_JOINTS 8

static dcmotor_params params[NUM_JOINTS];

static go_real old_pos[NUM_JOINTS];

static go_flag joint_is_homing[NUM_JOINTS];
static go_flag joint_is_homed[NUM_JOINTS];
static go_real joint_home_latch[NUM_JOINTS];

/* the index of the first joint on the second board, e.g., if
   joints 0-2 are on the first board, and 3-5 are on the second,
   make this 3 */
static int joint_index_divider = 3;

/*
  Inland Motor BM-3503, peak torque 109 N-m
  ---------------------------
  armature resistance Ra =           0.028 (ohms)
  armature inductance La =           0.00035 (henries)
  back emf constant Kb =             0.414 (volts/radian/sec, N-m/amp)
  rotor inertia Jm =                 0.00707 (N-m/rad/sec^2)
  damping friction coefficient Bm =  6.129 (N-m/rad/sec)
*/

go_result ext_init(char * init_string)
{
#ifdef HAVE_S626
  unsigned int board;
  unsigned long errFlags;
  BYTE poll_list[16];
  int gain;
  int chan;
#endif

  /* put here what needs to be done once, not per-joint */
#ifdef HAVE_S626
  if (0 != init_string) {
    joint_index_divider = init_string[0] - '0' - 1;
  }
  rtapi_print("setting joint index divider at %d\n", (int) joint_index_divider);

  for (board = 0; board < NUM_BOARDS; board++) {
    S626_OpenBoard(board, 0, InterruptNullISR, 1);
    S626_InterruptEnable(board, FALSE);
    S626_SetErrCallback(board, ErrorFunctionNull);
    errFlags = S626_GetErrors(board);
    if (errFlags != 0x0) {
      rtapi_print("can't open board %d\n", board);
      return GO_RESULT_ERROR;
    }

    /* set up ADC conversions */
    for (chan = 0; chan < 16; chan++) {
      poll_list[chan] = (gain << 4);
      poll_list[chan] += chan;
    }
    /*
      It looks like polling more than 6 ADCs interferes with the
      timing, and the platform runs rough.

      From the S626 demo, the ADC throughput test on one channel reports:

      time_spent = 3833 (ms) to get 10000 samples

      for about 0.4 milliseconds per channel. At 16 channels, this is
      6.4 msec, for two boards is about 13 msec, and our servos are
      running at 8 msec.
    */
#define MAX_ADC_INDEX 0
    poll_list[MAX_ADC_INDEX] += ADC_EOPL;
    S626_ResetADC(board, poll_list);
  }
#endif /* HAVE_S626 */

  return GO_RESULT_OK;
}
 
go_result ext_quit(void)
{
#ifdef HAVE_S626
  unsigned int board;
  unsigned int index;
#endif

  /* put here what needs to be done once, not per-joint */
#ifdef HAVE_S626
  for (board = 0; board < NUM_BOARDS; board++) {
    for (index = 0; index < 4; index++) {
      S626_WriteDAC(board, index, 0);
    }
#if 0
    /* FIXME-- closing the second board hangs */
    S626_CloseBoard(board);
#endif
  }
#endif

  return GO_RESULT_OK;
}

go_result ext_joint_init(go_integer joint, go_real cycle_time)
{
#ifdef HAVE_S626
  unsigned int board;
  unsigned int index;
  WORD Counter;
#endif

  if (joint < 0 || joint >= NUM_JOINTS) return GO_RESULT_ERROR;

#ifdef HAVE_S626
  if (! UseS626(joint)) {
#endif
    dcmotor_init(&params[joint],
		 6.129,		/* Bm */
		 0.00035,		/* La */
		 0.028,		/* Ra */
		 0.00707,		/* Jm */
		 0.414,		/* Kb */
		 0.0,		/* Tl, load torque */
		 0.0,		/* Tk, static friction torque */
		 0.0,		/* Ts, sliding friction torque */
		 cycle_time		/* cycle time */
		 );
    /* set the starting position to be something random, in this case
       the joint number */
    dcmotor_set_theta(&params[joint], (go_real) joint);
    old_pos[joint] = (go_real) joint;
#ifdef HAVE_S626
  }
#endif

  /* clear our home flags */
  joint_is_homing[joint] = 0;
  joint_is_homed[joint] = 0;
  joint_home_latch[joint] = 0.0;

#ifdef HAVE_S626
  if (UseS626(joint)) {
    board = joint < joint_index_divider ? 0 : 1;
    index = joint < joint_index_divider ? joint : joint - joint_index_divider;
    Counter = IndexToCounter(index);
    if (board < NUM_BOARDS) {
      S626_CounterCapFlagsReset(board, Counter);
      CreateEncoderCounter(board, Counter);
    }
  }
#endif /* HAVE_S626 */

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

go_result ext_read_pos(go_integer joint, go_real * pos)
{
  go_real theta;			/* angular position */
  go_real dtheta;		/* angular velocity */
  go_real d2theta;		/* angular acceleration */
#ifdef HAVE_S626
  unsigned int board;
  unsigned int index;
  WORD Counter;
  int signed_latch;	   /* we need to convert unsigned to signed */
#endif

  if (joint < 0 || joint >= NUM_JOINTS) return GO_RESULT_ERROR;

#ifdef HAVE_S626
  if (UseS626(joint)) {
    board = joint < joint_index_divider ? 0 : 1;
    index = joint < joint_index_divider ? joint : joint - joint_index_divider;
    Counter = IndexToCounter(index);
    if (board < NUM_BOARDS) {
      signed_latch = (int) (S626_CounterReadLatch(board, Counter) - COUNTER_OFFSET);
      theta = (go_real) signed_latch;
    } else {
      theta = 0;
    }
  } else {
    (void) dcmotor_get(&params[joint], &theta, &dtheta, &d2theta);
  }
#else
  (void) dcmotor_get(&params[joint], &theta, &dtheta, &d2theta);
#endif

  *pos = theta;

  return GO_RESULT_OK;
}

go_result ext_write_pos(go_integer joint, go_real pos)
{
  return GO_RESULT_IMPL_ERROR;
}

go_result ext_write_vel(go_integer joint, go_real vel)
{
#ifdef HAVE_S626
  unsigned int board;
  unsigned int index;
#endif

  if (joint < 0 || joint >= NUM_JOINTS) return GO_RESULT_ERROR;

  /* save our old position */
  (void) ext_read_pos(joint, &old_pos[joint]);

#ifdef HAVE_S626
  if (UseS626(joint)) {
    board = joint < joint_index_divider ? 0 : 1;
    index = joint < joint_index_divider ? joint : joint - joint_index_divider;
    if (board < NUM_BOARDS) {
      S626_WriteDAC(board, index, (int) vel);
    }
  } else {
    /* clock the simulation to bring us to our new position */
    (void) dcmotor_run_current_cycle(&params[joint], vel);
  }
#else
  /* clock the simulation to bring us to our new position */
  (void) dcmotor_run_current_cycle(&params[joint], vel);
#endif

  return GO_RESULT_OK;
}

go_result ext_joint_home(go_integer joint)
{
  if (joint < 0 || joint >= NUM_JOINTS) return GO_RESULT_ERROR;

  joint_is_homing[joint] = 1;
  joint_is_homed[joint] = 0;
  
  return GO_RESULT_OK;
}

go_flag ext_joint_is_home(go_integer joint)
{
  go_real pos;

  if (joint < 0 || joint >= NUM_JOINTS) return 1;
  if (joint_is_homed[joint]) return 1;
  if (! joint_is_homing[joint]) return 0;

  /* else home us immediately */
  joint_is_homing[joint] = 0;
  joint_is_homed[joint] = 1;
  (void) ext_read_pos(joint, &pos);
  joint_home_latch[joint] = pos;
  return 1;
}

go_result ext_joint_home_latch(go_integer joint, go_real * pos)
{
  if (joint < 0 || joint >= NUM_JOINTS) return GO_RESULT_ERROR;

  *pos = joint_home_latch[joint];

  return GO_RESULT_OK;
}

go_result ext_joint_quit(go_integer joint)
{
  return GO_RESULT_OK;
}

/*
  The S626 has 48 digital IO channels, 16 ADCs and 4 DACs.
*/

static WORD ain_data[2][16];

go_integer ext_num_ain(void)
{
  return 32;			/* for the two boards */
}

go_integer ext_num_aout(void)
{
  return 2;			/* for the one spare on each of two boards */
}

go_integer ext_num_din(void)
{
  return 24;			/* for half of the DIO */
}

go_integer ext_num_dout(void)
{
  return 24;			/* for the other half of the DIO */
}

go_result ext_trigger_in(void)
{
  DWORD board;

  for (board = 0; board < NUM_BOARDS; board++) {
    S626_ReadADC(board, ain_data[board]);
  }

  return GO_RESULT_OK;
}

/* this relies on a call to ext_trigger_ain() */
go_result ext_read_ain(go_integer index, go_real * val)
{
  if (index < 0) return GO_RESULT_RANGE_ERROR;

  if (index < 16) {
    *val = ((double) (ain_data[0][index])) / 3276.7;
    return GO_RESULT_OK;
  }

  if (index < 32) {
    *val = ((double) (ain_data[1][index - 16])) / 3276.7;
    return GO_RESULT_OK;
  }

  return GO_RESULT_RANGE_ERROR;
}

/*
  We need to skip over the DACs used by ext_write_vel. These are board
  0 channels 0,1,2 and board 1 channels 0,1,2, leaving channel 3 on
  each board for a total of 2 DACs we can use.  Index 0 is mapped to
  board 0 DAC 3, index 1 is mapped to board 1 DAC 3.  */
go_result ext_write_aout(go_integer index, go_real val)
{
  int i;
  int board;

  if (index < 0 || index > 1) return GO_RESULT_RANGE_ERROR;

  /* 10V is 8191 */
  if (val < 0) {
    i = (int) (val * 819.1 - 0.5);
  } else {
    i = (int) (val * 819.1 + 0.5);
  }

  board = index;
  if (board < NUM_BOARDS) {
    S626_WriteDAC(index, 3, i);
  }

  return GO_RESULT_OK;
}

go_result ext_read_din(go_integer index, go_flag * val)
{
  /* FIXME-- do these */
  return GO_RESULT_OK;
}

go_result ext_write_dout(go_integer index, go_flag val)
{
  /* FIXME-- do these */
  return GO_RESULT_OK;
}

go_result ext_set_parameters(go_integer joint, go_real * values, go_integer number)
{
  /* nothing to do */
  return GO_RESULT_OK;
}
