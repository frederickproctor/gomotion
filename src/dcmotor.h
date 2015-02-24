/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#ifndef DCMOTOR_H
#define DCMOTOR_H

#include "gotypes.h"

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

/*
  Simulation of a separately excited DC motor, from Benjamin C. Kuo,
  "Automatic Control Systems," Fourth Edition, pp. 176-186.

  Units are SI for all quantities. Note that back EMF constant Kb is
  identically equal to torque constant Ki in SI units, so Ki is omitted
  from interface and replaced with Kb in calculations.

  Example parameters:

  Inland Motor BMHS-0701, peak torque 0.312 N-m
  ---------------------------
  armature resistance Ra =           1.10 (ohms)
  armature inductance La =           0.0006 (henries)
  back emf constant Kb =             0.0254 (volts/radian/sec, N-m/amp)
  rotor inertia Jm =                 0.00000368 (N-m/rad/sec^2)
  damping friction coefficient Bm =  0.000588 (N-m/rad/sec)

  Inland Motor BMHS-1401, peak torque 8.56 N-m
  ---------------------------
  armature resistance Ra =           1.22 (ohms)
  armature inductance La =           0.00085 (henries)
  back emf constant Kb =             0.237 (volts/radian/sec, N-m/amp)
  rotor inertia Jm =                 0.0000584 (N-m/rad/sec^2)
  damping friction coefficient Bm =  0.0462 (N-m/rad/sec)

  Inland Motor BM-3202, peak torque 37 N-m
  ---------------------------
  armature resistance Ra =           1.57 (ohms)
  armature inductance La =           0.017 (henries)
  back emf constant Kb =             1.72 (volts/radian/sec, N-m/amp)
  rotor inertia Jm =                 0.00433 (N-m/rad/sec^2)
  damping friction coefficient Bm =  1.90 (N-m/rad/sec)

  Inland Motor BM-3503, peak torque 109 N-m
  ---------------------------
  armature resistance Ra =           0.028 (ohms)
  armature inductance La =           0.00035 (henries)
  back emf constant Kb =             0.414 (volts/radian/sec, N-m/amp)
  rotor inertia Jm =                 0.00707 (N-m/rad/sec^2)
  damping friction coefficient Bm =  6.129 (N-m/rad/sec)

  Inland Motor BMHS-0701 with Ra adjusted so that root == 0
  ---------------------------
  armature resistance Ra =           0.744527 (ohms)
  armature inductance La =           0.0006 (henries)
  back emf constant Kb =             0.0254 (volts/radian/sec, N-m/amp)
  rotor inertia Jm =                 0.00000368 (N-m/rad/sec^2)
  damping friction coefficient Bm =  0.000588 (N-m/rad/sec)

  Handy conversion factors:

  oz-in * 0.00708 = newton-meters
  foot-pounds * 1.359 = newton-meters

  To make a motor "bigger", increase Lm and Jm by some factor.
*/

typedef struct {
  /* stuff for both current and voltage mode */
  go_real bm, la, ra, jm, k;	/* motor params */
  go_real tl, tk, ts;		/* load and friction torques */
  go_real t;			/* sim time interval */

  /* current input stuff */
  go_real bm_inv;
  go_real bm_jm;
  go_real jm_bm;
  go_real embm_jmt;

  /* voltage input stuff */
  go_flag flag;		/* 0 = real, 1 = imaginary */
  go_real a, b, c, d;
  go_real root;
  go_real c_inv;
  /* real root stuff */
  go_real eb, emb;
  go_real root2_inv;
  go_real rootpb_inv;
  go_real rootmb_inv;
  go_real a2_inv;
  /* imaginary root stuff */
  go_real mb_2a, embt_2a;
  go_real cos_root, sin_root;

  /* output variables, set by dcmotor_run_current,voltage_cycle() */
  go_real theta;		/* shaft position, radians */
  go_real dtheta;		/* shaft angular velocity, rad/sec */
  go_real d2theta;		/* shaft angular acceleration, rad/sec^2 */
} dcmotor_params;

typedef enum {
  DCMOTOR_PARAMETER_BM = 1,
  DCMOTOR_PARAMETER_LA,
  DCMOTOR_PARAMETER_RA,
  DCMOTOR_PARAMETER_JM,
  DCMOTOR_PARAMETER_K,		/* K, KA and KB will be treated the same */
  DCMOTOR_PARAMETER_KA,
  DCMOTOR_PARAMETER_KB,
  DCMOTOR_PARAMETER_TL,
  DCMOTOR_PARAMETER_TK,
  DCMOTOR_PARAMETER_TS,
} dcmotor_parameter_type;

extern go_result dcmotor_set_parameter(dcmotor_params * p, dcmotor_parameter_type type, go_real value);

extern go_result dcmotor_get_parameter(dcmotor_params * p, dcmotor_parameter_type type, go_real *value);

extern go_result dcmotor_init(dcmotor_params * p,	/* params to init */
		       go_real Bm,	/* viscous frictional coeff */
		       go_real La,	/* armature inductance */
		       go_real Ra,	/* armature resistance */
		       go_real Jm,	/* inertia of rotor + load */
		       go_real Kb,	/* torque const = back EMF const */
		       go_real Tl,	/* load torque */
		       go_real Tk,	/* static friction torque */
		       go_real Ts,	/* sliding friction torque */
		       go_real t	/* cycle time for simulation */
		       );

extern go_result dcmotor_set_theta(dcmotor_params * p, go_real theta);

/* 
   call dcmotor_set() with the theta, dtheta and d2theta from the last
   call to dcmotor_run_current,voltage_cycle, when changing time
   intervals with dcmotor_init(), e.g.,

   dcmotor_init(..., t1);
   dcmotor_run_current,voltage_cycle();
   ...
   dcmotor_run_current,voltage_cycle();

   dcmotor_get(, &t, &dt, &d2t);
   dcmotor_init(..., t2);
   dcmotor_set(, t, dt, d2t);

   dcmotor_run_current,voltage_cycle();
   ...
*/

extern go_result dcmotor_set_cycle_time(dcmotor_params *p, go_real cycle_time);

extern go_result dcmotor_run_current_cycle(dcmotor_params * p,	/* params for motor */
					   go_real i	/* applied current */
					   );

extern go_result dcmotor_run_voltage_cycle(dcmotor_params * p,	/* params for motor */
				  go_real v	/* applied voltage */
				  );

extern go_result dcmotor_get(dcmotor_params * p,	/* params from which to get */
		      go_real *theta,	/* angular position */
		      go_real *dtheta,	/* angular velocity */
		      go_real *d2theta	/* angular acceleration */
		      );

extern go_result dcmotor_set(dcmotor_params * p,	/* params to set */
		      go_real theta,	/* angular position */
		      go_real dtheta,	/* angular velocity */
		      go_real d2theta	/* angular acceleration */
		      );

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif /* DCMOTOR_H */
