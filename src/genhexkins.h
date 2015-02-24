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
  genhexkins.h
  Originally written by: R. Brian Register, 1999

  This is the header file to accompany genhexkins.c.  This header file is used
  to configure genhexkins.c to solve the kinematics for a particular Stewart
  Platform configuration.
*/

#ifndef GENHEXKINS_H
#define GENHEXKINS_H

#include "gotypes.h"		/* go_result, go_integer */
#include "gomath.h"		/* go_pose */
#include "gokin.h"		/* go_kin_type */

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

#define GENHEX_NUM_JOINTS 6

typedef struct {
  go_cart base[GENHEX_NUM_JOINTS]; /* coords of base vertices wrt base origin */
  go_cart platform[GENHEX_NUM_JOINTS]; /* coords of platform vertices wrt platform origin */
  go_integer iteration;
} genhex_struct;

extern go_integer genhex_kin_size(void); 

extern go_result genhex_kin_init(void * kins); 

extern const char * genhex_kin_get_name(void); 

extern go_integer genhex_kin_num_joints(void * kins);

extern go_result genhex_kin_fwd(void * kins,
				const go_real *joint,
				go_pose * world);

extern go_result genhex_kin_inv(void * kins,
				const go_pose * world,
				go_real *joint);

extern go_kin_type genhex_kin_get_type(void * kins); 

extern go_result genhex_kin_set_parameters(void * kins, go_link * params, go_integer num); 

extern go_result genhex_kin_get_parameters(void * kins, go_link * params, go_integer num); 

extern go_result genhex_kin_jac_inv(void * kins,
				    const go_pose * pos,
				    const go_vel * vel,
				    const go_real * joints, 
				    go_real * jointvels); 


extern go_result genhex_kin_jac_fwd(void * kins,
				    const go_real * joints,
				    const go_real * jointvels,
				    const go_pose * pos, 
				    go_vel * vel); 

extern go_integer genhex_kin_fwd_interations(genhex_struct * genhex);

/* extras */

extern go_integer genhex_kin_fwd_iterations(genhex_struct * genhex);

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif
