/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#include "go.h"
#include "pid.h"

go_result pid_init(pid_struct * pid)
{
  pid->t = 1.0;
  pid->t_inv = 1.0;

  if (GO_RESULT_OK != pid_set_gains(pid,
				    1, 0, 0, /* p,i,d */
				    0, 0,	/* vff,aff */
				    -1, 1,	/* min_output,max_output */
				    0, 0,	/* neg_bias, pos_bias */
				    0)) {	/* deadband */
    return GO_RESULT_ERROR;
  }
  /* do the ones not handled in the basic set */
  if (GO_RESULT_OK != pid_set_gain(pid, PID_GAIN_PFF, 0)) return GO_RESULT_ERROR;

  return pid_reset(pid);
}

go_result pid_set_cycle_time(pid_struct * pid, go_real cycle_time)
{
  if (cycle_time <= 0.0) return GO_RESULT_ERROR;

  pid->t = cycle_time;
  pid->t_inv = 1.0 / cycle_time;

  return GO_RESULT_OK;
}

go_result pid_set_gains(pid_struct * pid,
		  go_real p,
		  go_real i,
		  go_real d, 
		  go_real vff,
		  go_real aff,
		  go_real min_output,
		  go_real max_output,
		  go_real neg_bias,
		  go_real pos_bias,
		  go_real deadband)
{
  pid->p = p;
  pid->i = i;
  pid->d = d;
  pid->vff = vff;
  pid->aff = aff;
  pid->min_output = min_output;
  pid->max_output = max_output;
  pid->neg_bias = neg_bias;
  pid->pos_bias = pos_bias;
  pid->deadband = deadband;

  return GO_RESULT_OK;
}

go_result pid_set_gain(pid_struct * pid,
		       pid_gain_type type,
		       go_real gain)
{
  go_result retval = GO_RESULT_OK;

  switch (type) {
  case PID_GAIN_P:
    pid->p = gain;
    break;
  case PID_GAIN_I:
    pid->i = gain;
    break;
  case PID_GAIN_D:
    pid->d = gain;
    break;
  case PID_GAIN_VFF:
    pid->vff = gain;
    break;
  case PID_GAIN_AFF:
    pid->aff = gain;
    break;
  case PID_GAIN_MIN_OUTPUT:
    pid->min_output = gain;
    break;
  case PID_GAIN_MAX_OUTPUT:
    pid->max_output = gain;
    break;
  case PID_GAIN_NEG_BIAS:
    pid->neg_bias = gain;
    break;
  case PID_GAIN_POS_BIAS:
    pid->pos_bias = gain;
    break;
  case PID_GAIN_DEADBAND:
    pid->deadband = gain;
    break;
  case PID_GAIN_PFF:
    pid->pff = gain;
    break;
  default:
    retval = GO_RESULT_ERROR;
    break;
  }

  return retval;
}

go_result pid_get_gain(pid_struct * pid,
		       pid_gain_type type,
		       go_real *gain)
{
  go_result retval = GO_RESULT_OK;

  switch (type) {
  case PID_GAIN_P:
    *gain = pid->p;
    break;
  case PID_GAIN_I:
    *gain = pid->i;
    break;
  case PID_GAIN_D:
    *gain = pid->d;
    break;
  case PID_GAIN_VFF:
    *gain = pid->vff;
    break;
  case PID_GAIN_AFF:
    *gain = pid->aff;
    break;
  case PID_GAIN_MIN_OUTPUT:
    *gain = pid->min_output;
    break;
  case PID_GAIN_MAX_OUTPUT:
    *gain = pid->max_output;
    break;
  case PID_GAIN_NEG_BIAS:
    *gain = pid->neg_bias;
    break;
  case PID_GAIN_POS_BIAS:
    *gain = pid->pos_bias;
    break;
  case PID_GAIN_DEADBAND:
    *gain = pid->deadband;
    break;
  case PID_GAIN_PFF:
    *gain = pid->pff;
    break;
  default:
    retval = GO_RESULT_ERROR;
    break;
  }

  return retval;
}

go_result pid_copy_gains(pid_struct * dst, pid_struct * src)
{
  return pid_set_gains(dst,
		       src->p,
		       src->i,
		       src->d,
		       src->vff,
		       src->aff,
		       src->min_output,
		       src->max_output,
		       src->neg_bias,
		       src->pos_bias,
		       src->deadband);
}

#undef SPERR

go_result pid_run_cycle(pid_struct * pid, go_real sp, go_real in, go_real * out)
{
  go_real err, werr;
  go_real spdot, spdbldot;
  go_real up, ui, ud;
  go_real pff, vff, aff;
  go_real raw;

  err = sp - in;

  if (err > pid->deadband) {
    err = err - pid->deadband;
  } else if (err < -pid->deadband) {
    err = err + pid->deadband;
  } else {
    err = 0.0;
  }

  werr = err * pid->t;
  spdot = (sp - pid->lastsp) * pid->t_inv;
  spdbldot = (spdot - pid->lastspdot) * pid->t_inv;

  up = pid->p * err;
  ui = pid->i * (pid->cumerr + werr);
#ifdef SPERR
  ud = pid->d * (sp - pid->lastsp) * pid->t_inv;
#else
  ud = pid->d * (err - pid->lasterr) * pid->t_inv;
#endif
  pff = pid->pff * sp;
  vff = pid->vff * spdot;
  aff = pid->aff * spdbldot;

  raw = up + ui + ud + pff + vff + aff;
  if (raw < pid->min_output) {
    raw = pid->min_output;
    /* inhibit cumulative error for anti-windup */
  } else if (raw > pid->max_output) {
    raw = pid->max_output;
    /* inhibit cumulative error for anti-windup */
  } else {
    /* allow cumulative error */
    pid->cumerr += werr;
  }

  if (err >= 0.0) {
    *out = raw + pid->pos_bias;
  } else {
    *out = raw - pid->neg_bias;
  }
  pid->lasterr = err;
  pid->lastsp = sp;
  pid->lastspdot = spdot;

  return GO_RESULT_OK;
}

go_result pid_reset(pid_struct * pid)
{
  pid->lasterr = 0.0;
  pid->lastsp = 0.0;
  pid->lastspdot = 0.0;
  pid->cumerr = 0.0;

  return GO_RESULT_OK;
}
