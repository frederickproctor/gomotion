/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#ifndef SPHERISTKINS_H
#define SPHERISTKINS_H

#include "gotypes.h"		/* go_result, go_integer */
#include "gomath.h"		/* go_pose */
#include "gokin.h"		/* go_kin_type */

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

#define SPHERIST_NUM_JOINTS 6

typedef struct {
  /* a0, alpha0 = 0 since {0} and {1} are coincident */
  /* d1 = 0 since {0} and {1} are coincident */

  go_real a1;
  go_real sa1;			/* sin(alpha1) */
  go_real ca1;			/* cos(alpha1) */
  go_real d2;

  go_real a2;
  go_real sa2;
  go_real ca2;
  go_real d3;

  go_real a3;
  go_real sa3;
  go_real ca3;
  go_real d4;

  go_real sa4;
  go_real ca4;

  go_real sa5;
  go_real ca5;

  /* a4 and a5 = 0, d5 and d6 = 0 since {4}, {5} and {6} are coincident */

  go_flag iflags;		/* bits for flags SPHERIST_SHOULDER, ... */
} spherist_kin_struct;

/* flags for inverse kinematics */
#define SPHERIST_SHOULDER_RIGHT 0x01
#define SPHERIST_ELBOW_DOWN     0x02
#define SPHERIST_WRIST_FLIP     0x04
#define SPHERIST_SINGULAR       0x08  /* joints at a singularity */

extern go_integer spherist_kin_size(void); 

extern go_result spherist_kin_init(void *kins); 

extern const char *spherist_kin_get_name(void); 

extern go_integer spherist_kin_num_joints(void *kins);

extern go_result spherist_kin_fwd(void *kins,
				  const go_real *joint,
				  go_pose *world);

extern go_result spherist_kin_inv(void *kins,
				  const go_pose *world,
				  go_real *joint);

extern go_kin_type spherist_kin_get_type(void *kins); 

extern go_result spherist_kin_set_parameters(void *kins, go_link *params, go_integer num); 

extern go_result spherist_kin_get_parameters(void *kins, go_link *params, go_integer num); 

extern go_result spherist_kin_jac_inv(void *kins,
				      const go_pose *pos,
				      const go_vel *vel,
				      const go_real *joints, 
				      go_real *jointvels); 


extern go_result spherist_kin_jac_fwd(void *kins,
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
