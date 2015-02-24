/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#include "gotypes.h"
#include "dcmotor.h"
#include <math.h>		/* exp(), sqrt() */

/*
  Simulation of a separately excited DC motor, from Benjamin C. Kuo,
  "Automatic Control Systems," Fourth Edition, pp. 176-186.

  Analysis shows that:

  di/dt = -Ra/La i - Kb/La dth/dt + 1/La v      [1]
  d2th/dt2 = Ki/Jm i - Bm/Jm dth/dt - 1/Jm Tl   [2]

  where

  Ra, La, Kb = Ki, Jm, Bm are motor parameters;
  Tl is the constant torque load;
  v is applied voltage;
  i, di/dt are armature current, time derivative;
  th, dth/dt, d2th/dt2 are shaft angular position theta, time derivatives

  Current Input
  -------------
  If current is the input to the motor, as is the case with an amplifier
  in current mode, we can solve equation [2], rewritten with y
  representing theta, as

  Jm y''[t] + Bm y'[t] = I Ki - Tl

  which gives

  y = (I Ki - Tl)/Bm t - C1 Jm/Bm exp(-Bm/Jm t) + C2
  y' = (I Ki - Tl)/Bm + C1 exp(-Bm/Jm t)
  y'' = -C1 Bm/Jm exp(-Bm/Jm t)

  From this, we have initial conditions

  y'(0) = (I Ki - Tl)/Bm + C1 -> C1 = y'(0) - (I Ki - Tl)/Bm
  y(0) = -C1 Jm/Bm + C2 -> C2 = y(0) + C1 Jm/Bm

  Each cycle, we compute C1 and C2 from the final conditions of
  the previous cycle, compute y, y', and y'', and then set these
  as the initial conditions for the next cycle.

  From y' we see that the steady-state velocity is (I Ki - Tl)/Bm.
  Assuming current is proportional to voltage from the D/A converter
  through the gain of the amplifier Ka, I = Ka V, we have

  y'(Inf) = (Ka Ki V - Tl)/Bm

  The velocity feedforward term vff is the ratio of the voltage to the
  velocity with no load torque, or

  vff = Bm/(Ka Ki)

  Voltage Input
  -------------
  We need a single equation relating output theta to input voltage.
  Solving [2] for i, differenting to get di/dt, and substituting into [1],
  we get

  LaJm d3th/dt3 + (BmLa + RaJm) d2th/dt2 + (RaBm + Kb^2) dth/dt = Kb v - RaTl

  which is a third-order linear ordinary differential equation of the form

  a y''' + b y'' + c y' = d

  for the time interval over which the applied voltage is constant.
  Mathematica gives the solution with

  DSolve[a y'''[t] + b y''[t] + c y'[t] == d, y[t], t]

  involving exponentials of sqrt(b^2 - 4ac) and integration constants
  C[1], C[2] and C[3], as

  root = sqrt(b^2 - 4ac),
  y = dt/c - 2a exp((root + b)/2a t) C1 / (root + b) +
  2a exp((root - b)/2a t) C2 / (root - b) + C3

  Using initial conditions y[0], y'[0] and y''[0], we get expressions
  for C[] in terms of a, b, c and d and can evaluate y[t], y'[t] and
  y''[t] each cycle to step through the simulation.

  In the case where b^2 - 4ac is negative, the exponentials are converted
  to real and imaginary cos/sin terms, and the imaginary terms are discarded.

  In the case where b^2 - 4ac is identically zero, the expression for y
  reduces to

  y = dt/c + C -> C = y(0)
  y' = d/c
  y'' = 0

  That is, the motor is so precisely balanced that any voltage induces
  a simple constant speed and the transients are infinitesimal. This
  happens when

  2 Kb sqrt(La Jm) = RaJm +/- BmLa

  Steady State
  ------------
  From Kuo, the transfer function is

  Theta(s) = Ki/s * 1/(LaJm s^2 + (RaJm+BmLa s) + KbKi+RaBm) * Ea(s)

  where the 1/s shows that the angular position is an integral of the
  applied voltage Ea. The velocity is the above expression with the
  integrating 1/s term removed.

  If the applied voltage is a step of V volts, with Laplace transform
  V/s, then the steady-state velocity is

  lim(s->0) s Ki * 1/(LaJm s^2 + (RaJm+BmLa s) + KbKi+RaBm) * V/s
  = Ki/(KbKi + RaBm) * V

  So, the velocity feedforward term vff (K = Kb = Ki) is
 
  vff = (K^2 + RaBm)/K

  Arbitrary Time Sampling
  -----------------------
  In dcmotorInit() we initialize some state values that make computation
  more efficient, such as sqrt(b^2 - 4ac) and the two exponentials involving
  the root term, a, b, c and d, and t. Since t is passed at initialization,
  it can't be changed during calls to dcmotorRunVoltageCycle() arbitrarily.
  This is a disadvantage that is offset by the efficiency gained by 
  precomputing the exponentials at init time. dcmotorRunVoltageCycle() only
  uses arithmetic and is therefore quite fast.

  If arbitrary time periods are needed, call like this:

  dcmotorInit(..., t1);
  dcmotorRunVoltageCycle();
  dcmotorGet(&params);
  dcmotorInit(..., t2);
  dcmotorSet(&params);
  dcmotorRunVoltageCycle();
  ...

  Friction Modeling
  -----------------
  We can adapt the current and voltage equations to handle friction by
  changing the right hand side driving function to be reduced by the
  frictional torque Tf. The current and voltage equations are then

  Jm y'' + Bm y' = I Ki - Tl - Tf
  LaJm y'' + (BmLa + RaJm) y'' + (RaBm + Kb^2) y' = Kb v - Ra Tl - Ra Tf

  where Tf is the frictional torque, either static friction Tk or
  sliding friction Ts depending on the initial speed. Tk and Ts are
  positive in this analysis, and Tk > Ts.

  The Tf term is nonlinear in that its direction depends on the
  direction of motion, y'. To adapt the current and voltage simulations
  it is simply a matter of subtracting (or adding) the appropriate
  friction torque from driving torque in the positive (or negative)
  direction.
*/

/* value of discriminant to root such that it will be called 0 */
#define ROOT_FUZZ 1.e-20

/* value of speed such that it will be called 0, for friction modeling */
#define SPEED_FUZZ 1.e-6

go_result dcmotor_set_parameter(dcmotor_params * p,
				dcmotor_parameter_type type,
				go_real value)
{
  go_result retval = GO_RESULT_OK;

  switch (type) {
  case DCMOTOR_PARAMETER_BM:
    p->bm = value;
    break;
  case DCMOTOR_PARAMETER_LA:
    p->la = value;
    break;
  case DCMOTOR_PARAMETER_RA:
    p->ra = value;
    break;
  case DCMOTOR_PARAMETER_JM:
    p->jm = value;
    break;
  case DCMOTOR_PARAMETER_K:
  case DCMOTOR_PARAMETER_KA:
  case DCMOTOR_PARAMETER_KB:
    p->k = value;
    break;
  case DCMOTOR_PARAMETER_TL:
    p->tl = value;
    break;
  case DCMOTOR_PARAMETER_TK:
    p->tk = value;
    break;
  case DCMOTOR_PARAMETER_TS:
    p->ts = value;
    break;
  default:
    retval = GO_RESULT_ERROR;
    break;
  }

  return retval;
}

go_result dcmotor_get_parameter(dcmotor_params * p,
				dcmotor_parameter_type type,
				go_real *value)
{
  go_result retval = GO_RESULT_OK;

  switch (type) {
  case DCMOTOR_PARAMETER_BM:
    *value = p->bm;
    break;
  case DCMOTOR_PARAMETER_LA:
    *value = p->la;
    break;
  case DCMOTOR_PARAMETER_RA:
    *value = p->ra;
    break;
  case DCMOTOR_PARAMETER_JM:
    *value = p->jm;
    break;
  case DCMOTOR_PARAMETER_K:
  case DCMOTOR_PARAMETER_KA:
  case DCMOTOR_PARAMETER_KB:
    *value = p->k;
    break;
  case DCMOTOR_PARAMETER_TL:
    *value = p->tl;
    break;
  case DCMOTOR_PARAMETER_TK:
    *value = p->tk;
    break;
  case DCMOTOR_PARAMETER_TS:
    *value = p->ts;
    break;
  default:
    retval = GO_RESULT_ERROR;
    break;
  }

  return retval;
}

go_result dcmotor_init(dcmotor_params * p, /* params to init */
		       go_real Bm,	/* viscous frictional coeff */
		       go_real La,	/* armature inductance */
		       go_real Ra,	/* armature resistance */
		       go_real Jm,	/* inertia of rotor + load */
		       go_real Kb, /* torque constant = back EMF constant */
		       go_real Tl, /* load torque */
		       go_real Tk, /* static friction torque */
		       go_real Ts, /* sliding friction torque */
		       go_real t   /* cycle time for simulation */
  )
{
  if (Bm < GO_REAL_EPSILON || Jm < GO_REAL_EPSILON) {
    return GO_RESULT_ERROR;
  }

  /* stuff for both modes */
  p->bm = Bm;
  p->la = La;
  p->ra = Ra;
  p->jm = Jm;
  p->k = Kb;
  p->tl = Tl;
  p->tk = Tk;
  p->ts = Ts;
  p->t = t;

  /* current input stuff */
  p->bm_inv = 1. / Bm;
  p->bm_jm = Bm / Jm;
  p->jm_bm = Jm / Bm;
  p->embm_jmt = exp(-p->bm_jm * t);

  /* voltage input stuff */
  p->a = La * Jm;
  p->b = Bm * La + Ra * Jm;
  p->c = Ra * Bm + Kb * Kb;
  p->d = Ra * Tl;

  p->root = p->b * p->b - 4. * p->a * p->c;
  if (p->c == 0. || p->a == 0.) {
    return GO_RESULT_ERROR;
  }

  if (p->root < -ROOT_FUZZ) {
    /* imaginary root stuff */
    p->flag = 1;
    p->root = sqrt(-p->root);
    p->c_inv = 1. / p->c;
    p->mb_2a = -p->b / (2. * p->a);
    p->embt_2a = exp(p->t * p->mb_2a);
    p->cos_root = cos(p->root * p->t);
    p->sin_root = sin(p->root * p->t);
  } else if (p->root > ROOT_FUZZ) {
    /* real root stuff */
    p->flag = 0;
    p->root = sqrt(p->root);
    p->eb = exp(-(p->b + p->root) * t / (2. * p->a));
    p->emb = exp((-p->b + p->root) * t / (2. * p->a));
    p->c_inv = 1. / p->c;
    p->root2_inv = 1. / (2. * p->root);
    p->rootpb_inv = 1. / (p->root + p->b);
    p->rootmb_inv = 1. / (p->root - p->b);
    p->a2_inv = 1. / (2. * p->a);
  } else {
    /* root is identically zero */
    p->flag = 2;
    p->c_inv = 1. / p->c;
  }
  p->theta = p->dtheta = p->d2theta = 0.;

  return GO_RESULT_OK;
}

go_result dcmotor_set_theta(dcmotor_params * p, go_real theta)
{
  p->theta = theta;

  return GO_RESULT_OK;
}

go_result dcmotor_run_voltage_cycle(dcmotor_params * p,	/* params for motor */
				    go_real v	/* applied voltage */
  )
{
  go_real rhs;
  go_real frictorq;
  go_real c1, c2, c3;
  go_flag stopped;

  rhs = v * p->k - p->d;

  /* figure out which frictional torque applies */
  if (p->dtheta < SPEED_FUZZ && p->dtheta > -SPEED_FUZZ) {
    /* system stopped, so apply static friction Tk */
    frictorq = p->ra * p->tk;
    stopped = 1;
  } else {
    frictorq = p->ra * p->ts;
    stopped = 0;
  }

  /* apply frictional torque */
  if (rhs > frictorq) {
    rhs = rhs - frictorq;
  } else if (rhs < -frictorq) {
    rhs = rhs + frictorq;
  } else {
    /* no net torque */
    rhs = 0.;
    if (stopped) {
      p->dtheta = 0.;
      p->d2theta = 0;
      return GO_RESULT_OK;
    }
  }

  if (p->flag == 1) {
    /* imaginary root stuff */
    c1 = p->dtheta - rhs * p->c_inv;	/* this is C1 + C2 */
    c3 = p->theta + c1 * p->b * 0.5 * p->c_inv;

    /* compute y's at next delta t, call it 0 */
    p->theta = rhs * p->t * p->c_inv + p->a * p->c_inv * c1 * p->embt_2a *
      (p->mb_2a * p->cos_root + p->root * p->sin_root) + c3;
    p->dtheta = rhs * p->c_inv + p->embt_2a * c1 * p->cos_root;
    p->d2theta =
      c1 * p->embt_2a * (p->mb_2a * p->cos_root - p->root * p->sin_root);
  } else if (p->flag == 0) {
    /* real root stuff */
    c2 =
      ((p->b + p->root) * (rhs * p->c_inv - p->dtheta) -
       2. * p->a * p->d2theta) * p->root2_inv;
    c1 = -(rhs * p->c_inv) + p->dtheta - c2;
    c3 =
      p->theta + (2. * p->a * c1) * p->rootpb_inv -
      (2. * p->a * c2) * p->rootmb_inv;

    /* compute y's at next delta t, call it 0 */
    p->theta =
      rhs * p->t * p->c_inv - (2. * p->a * p->eb * c1) * p->rootpb_inv +
      (2. * p->a * p->emb * c2) * p->rootmb_inv + c3;
    p->dtheta = rhs * p->c_inv + p->eb * c1 + p->emb * c2;
    p->d2theta =
      (-(p->b + p->root) * p->eb * c1 +
       (-p->b + p->root) * p->emb * c2) * p->a2_inv;
  } else {
    /* zero root stuff */
    p->dtheta = rhs * p->c_inv;
    p->theta = p->dtheta * p->t + p->theta;
    /* leave d2theta 0 */
  }

  return GO_RESULT_OK;
}

go_result dcmotor_run_current_cycle(dcmotor_params * p,	/* params for motor */
				    go_real i	/* applied current */
  )
{
  go_real rhs;
  go_real frictorq;
  go_real c1, c2;
  go_flag stopped;

  rhs = i * p->k - p->tl;

  /* figure out which frictional torque applies */
  if (p->dtheta < SPEED_FUZZ && p->dtheta > -SPEED_FUZZ) {
    /* system stopped, so apply static friction Tk */
    frictorq = p->tk;
    stopped = 1;
  } else {
    frictorq = p->ts;
    stopped = 0;
  }

  /* apply frictional torque */
  if (rhs > frictorq) {
    rhs = rhs - frictorq;
  } else if (rhs < -frictorq) {
    rhs = rhs + frictorq;
  } else {
    /* no net torque */
    rhs = 0.;
    if (stopped) {
      p->dtheta = 0.;
      p->d2theta = 0.;
      return GO_RESULT_OK;
    }
  }

  rhs = rhs * p->bm_inv;
  c1 = p->dtheta - rhs;
  c2 = p->theta + p->jm_bm * c1;

  p->theta = rhs * p->t - c1 * p->jm_bm * p->embm_jmt + c2;
  p->dtheta = rhs + c1 * p->embm_jmt;
  p->d2theta = -c1 * p->bm_jm * p->embm_jmt;

  return GO_RESULT_OK;
}

go_result dcmotor_get(dcmotor_params * p, /* params from which to get */
		      go_real *theta,	/* angular position */
		      go_real *dtheta,	/* angular velocity */
		      go_real *d2theta	/* angular acceleration */
		      )
{
  *theta = p->theta;
  *dtheta = p->dtheta;
  *d2theta = p->d2theta;

  return GO_RESULT_OK;
}

go_result dcmotor_set(dcmotor_params * p, /* params to set */
		      go_real theta,	/* angular position */
		      go_real dtheta,	/* angular velocity */
		      go_real d2theta	/* angular acceleration */
		      )
{
  p->theta = theta;
  p->dtheta = dtheta;
  p->d2theta = d2theta;

  return GO_RESULT_OK;
}
