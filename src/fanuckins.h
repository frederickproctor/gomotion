/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

/*
  fanuckins.h

  Kinematics functions for a Fanuc-like robot on a gantry, with
  six revolute joints carried by a gantry. Uses genserkins due to
  the 'a1' DH parameter that is assumed 0 in the Puma kinematics,
  so too bad we can't simply adapt the Puma kinematics.
*/

#ifndef FANUCKINS_H
#define FANUCKINS_H

#include "gotypes.h"		/* go_result, go_integer */
#include "gomath.h"		/* go_pose */
#include "gokin.h"		/* go_kin_type */
#include "three21kins.h"	/* three21_kin_struct */

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

/* six robot joints plus the gantry joint */
#define FANUC_KIN_NUM_JOINTS 7

typedef struct {
  three21_kin_struct tk;
  go_real gantry;
} fanuc_kin_struct;

extern go_integer fanuc_kin_size(void); 

extern go_result fanuc_kin_init(fanuc_kin_struct *kins); 

extern const char *fanuc_kin_get_name(void); 

extern go_integer fanuc_kin_num_joints(fanuc_kin_struct *kins);

extern go_result fanuc_kin_fwd(fanuc_kin_struct *kins,
			       const go_real *motors,
			       go_pose *world);

extern go_result fanuc_kin_inv(fanuc_kin_struct *kins,
			       const go_pose *world,
			       go_real *motors);

extern go_kin_type fanuc_kin_get_type(fanuc_kin_struct *kins); 

extern go_result fanuc_kin_set_parameters(fanuc_kin_struct *kins, go_link *params, go_integer num);

extern go_result fanuc_kin_get_parameters(fanuc_kin_struct *kins, go_link *params, go_integer num); 

extern go_result fanuc_kin_jac_inv(fanuc_kin_struct *kins,
				   const go_pose *pos,
				   const go_vel *vel,
				   const go_real *motors, 
				   go_real *motorvels); 


extern go_result fanuc_kin_jac_fwd(fanuc_kin_struct *kins,
				   const go_real *motors,
				   const go_real *motorvels,
				   const go_pose *pos, 
				   go_vel *vel); 

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif
