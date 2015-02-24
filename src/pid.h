/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#ifndef PID_H
#define PID_H

#include "gotypes.h"

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

typedef struct {
  go_real p;			/* proportional gain */
  go_real i;			/* integral gain */
  go_real d;			/* derivative gain */
  go_real vff;			/* first-order feedforward */
  go_real aff;			/* seond-order feedforward */
  go_real min_output;		/* output clamped between this... */
  go_real max_output;		/* ...and this */
  go_real pos_bias;		/* positive outputs add this */
  go_real neg_bias;		/* negative outputs subtract this */
  go_real deadband;		/* errors inside this make output zero */
  /* internal variables */
  go_real lasterr;
  go_real lastsp;
  go_real lastspdot;
  go_real cumerr;
  go_real t;
  go_real t_inv;
  /* extra variables not handled in basic 'pid_set_gains' command */
  go_real pff;			/* zeroth-order feedforward */
} pid_struct;

/*
  Typical sequence is

  pid_init
  pid_set_cycle_time
  pid_set_gains

  pid_run_cycle
  ...
  pid_run_cycle

  pid_set_gains

  pid_run_cycle
  ...
  pid_run_cycle

  pid_reset

  pid_run_cycle
  ...
  pid_run_cycle
 */

/* pid_init sets the pid_struct to a workable default state */
extern go_result pid_init(pid_struct * pid);

/* pid_set_cycle_time sets the cycle time and related internal parameters,
   leaving unrelated gains and internal parameters alone */
extern go_result pid_set_cycle_time(pid_struct * pid, go_real cycle_time);

/* pid_set_gains sets the gains, and related internal parameters,
   leaving the cycle time and related internal parameters alone */
extern go_result pid_set_gains(pid_struct * pid,
			       go_real p,
			       go_real i,
			       go_real d,
			       go_real vff,
			       go_real aff, 
			       go_real min_output,
			       go_real max_output,
			       go_real neg_bias,
			       go_real pos_bias,
			       go_real deadband);

typedef enum {
  PID_GAIN_P = 1,
  PID_GAIN_I,
  PID_GAIN_D,
  PID_GAIN_VFF,
  PID_GAIN_AFF,
  PID_GAIN_MIN_OUTPUT,
  PID_GAIN_MAX_OUTPUT,
  PID_GAIN_NEG_BIAS,
  PID_GAIN_POS_BIAS,
  PID_GAIN_DEADBAND,
  PID_GAIN_PFF
} pid_gain_type;

/* pid_set_gain sets an individual gain given from one of the enumerated
   'pid_gain_type' types */
extern go_result pid_set_gain(pid_struct * pid,
			      pid_gain_type type,
			      go_real gain);

extern go_result pid_get_gain(pid_struct * pid, pid_gain_type type, go_real *gain);

/* pid_copy_gains just copies the gains from 'src' to 'dst', leaving
   internal parameters in 'dst' alone */
extern go_result pid_copy_gains(pid_struct * dst, pid_struct * src);

/* pid_run_cycle runs a PID execution cycle, with setpoint 'sp', input
   'in' and putting result in pointer to output 'out' */
extern go_result pid_run_cycle(pid_struct * pid,
			       go_real sp,
			       go_real in,
			       go_real * out);

/* pid_reset clears internal history parameters, leaving cycle time
   and gains alone */
extern go_result pid_reset(pid_struct * pid);

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif
