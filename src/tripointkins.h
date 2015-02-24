/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#ifndef TRIPOINTKINS_H
#define TRIPOINTKINS_H

#include "gotypes.h"		/* go_result, go_integer */
#include "gomath.h"		/* go_pose */
#include "gokin.h"		/* go_kin_type */

/*
  The Tripoint is a tripod-like structure with all three struts
  intersecting at a point. The origin is defined to be the base of one
  of the struts, and the x-axis is defined to be pointing to an
  adjacent strut as shown in the overhead view:

  *       2
  *      / \
  * ^   /   \
  * |  /     \
  * y /       \
  *  /         \
  * 0 --------- 1  x->
  
  The controlled point lies at some point (x, y, z) above or below
  the base, z positive above, z negative below.

  The coordinates of the base points 0, 1 and 2 are 
  (0, 0), (x1, 0) and (x2, y2) respectively. This give three
  defining parameters for the tripoint, x1, x2 and y2. The
  strut lengths l0, l1 and l2 are the free joint values,
  with associated Cartesian position x, y and z.
*/

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

typedef struct {
  go_real x1;			/* x position of base point 1 */
  go_real x2;			/* x position of base point 2 */
  go_real y2;			/* y position of base point 2 */
  go_flag fflags;
} tripoint_kin_struct;

/* flags for forward kinematics */
enum {
  TRIPOINT_Z_POSITIVE,
  TRIPOINT_Z_NEGATIVE
};

extern go_integer tripoint_kin_size(void); 

extern go_result tripoint_kin_init(void * kins); 

extern const char * tripoint_kin_get_name(void); 

extern go_integer tripoint_kin_num_joints(void * kins);

extern go_result tripoint_kin_fwd(void * kins,
				  const go_real * joints,
				  go_pose * world);

extern go_result tripoint_kin_inv(void * kins,
				  const go_pose * world,
				  go_real * joints);

extern go_kin_type tripoint_kin_get_type(void * kins); 

extern go_result tripoint_kin_set_parameters(void * kins, go_link * params, go_integer num); 

extern go_result tripoint_kin_get_parameters(void * kins, go_link * params, go_integer num); 

extern go_result tripoint_kin_jac_inv(void * kins,
				      const go_pose * pos,
				      const go_vel * vel,
				      const go_real * joints, 
				      go_real * jointvels); 


extern go_result tripoint_kin_jac_fwd(void * kins,
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

#endif /* TRIPOINTKINS_H */
