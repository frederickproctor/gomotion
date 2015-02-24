/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#ifndef SCARAKINS_H
#define SCARAKINS_H

#include "gotypes.h"		/* go_result, go_integer */
#include "gomath.h"		/* go_pose */
#include "gokin.h"		/* go_kin_type */

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

typedef struct {
  go_real L1;
  go_real L2;
  go_real inv_2L1L2;
  go_flag iflags;
} scarakin_struct;

/* flags for inverse kinematics */
#define SCARA_ELBOW_DOWN     0x01
#define SCARA_SINGULAR       0x02  /* joints at a singularity */

extern go_integer scara_kin_size(void); 

extern go_result scara_kin_init(void * kins); 

extern const char * scara_kin_get_name(void); 

extern go_integer scara_kin_num_joints(void * kins);

extern go_result scara_kin_fwd(void * kins,
			       const go_real *joint,
			       go_pose * world);

extern go_result scara_kin_inv(void * kins,
			       const go_pose * world,
			       go_real *joint);

extern go_kin_type scara_kin_get_type(void * kins); 

extern go_result scara_kin_set_parameters(void * kins, go_link * params, go_integer num); 

extern go_result scara_kin_get_parameters(void * kins, go_link * params, go_integer num); 

extern go_result scara_kin_jac_inv(void * kins,
				   const go_pose * pos,
				   const go_vel * vel,
				   const go_real * joints, 
				   go_real * jointvels); 


extern go_result scara_kin_jac_fwd(void * kins,
				   const go_real * joints,
				   const go_real * jointvels,
				   const go_pose * pos,
				   go_vel * vel);

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif
