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
  three21kins.h

  Kinematics functions for a Three21-like robot on a gantry, with
  six revolute joints carried by a gantry. Uses the PUMA kinematics
  for the six revolute joints, adjusted for the a1 offset.
*/

#ifndef THREE21KINS_H
#define THREE21KINS_H

#include "gotypes.h"		/* go_result, go_integer */
#include "gomath.h"		/* go_pose */
#include "gokin.h"		/* go_kin_type */
#include "genserkins.h"		/* genser_kin_struct */

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

#define THREE21_KIN_NUM_JOINTS 6

#define THREE21_SHOULDER_RIGHT 0x01
#define THREE21_ELBOW_DOWN     0x02
#define THREE21_WRIST_FLIP     0x04
#define THREE21_SINGULAR       0x08

typedef struct {
  go_real a1;
  go_real a2;
  go_real a3;
  go_real d2;
  go_real d3;
  go_real d4;
  go_flag iflags;
  genser_struct gk;
} three21_kin_struct;

extern go_integer three21_kin_size(void); 

extern go_result three21_kin_init(three21_kin_struct *kins); 

extern const char *three21_kin_get_name(void); 

extern go_integer three21_kin_num_joints(three21_kin_struct *kins);

extern go_result three21_kin_fwd(three21_kin_struct *kins,
				 const go_real *joints,
				 go_pose *world);

extern go_result three21_kin_inv(three21_kin_struct *kins,
				 const go_pose *world,
				 go_real *joints);

extern go_kin_type three21_kin_get_type(three21_kin_struct *kins); 

extern go_result three21_kin_set_parameters(three21_kin_struct *kins, go_link *params, go_integer num);

extern go_result three21_kin_get_parameters(three21_kin_struct *kins, go_link *params, go_integer num); 

extern go_result three21_kin_jac_inv(three21_kin_struct *kins,
				     const go_pose *pos,
				     const go_vel *vel,
				     const go_real *joints, 
				     go_real *jointvels); 


extern go_result three21_kin_jac_fwd(three21_kin_struct *kins,
				     const go_real *joints,
				     const go_real *jointvels,
				     const go_pose *pos, 
				     go_vel *vel); 

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif
