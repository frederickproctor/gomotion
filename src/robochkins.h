/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#ifndef ROBOCHKINS_H
#define ROBOCHKINS_H

#include "gotypes.h"		/* go_result, go_integer */
#include "gomath.h"		/* go_pose */
#include "gokin.h"		/* go_kin_fwd_flags */

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

/*
  The Robocrane-H is a 6-DOF structure consisting of a horizontal
  T-shaped bar centered in a vertical triangular frame. Each end of
  the bar is connected to each of the three triangle vertices, as
  shown in the figures below, with connections

  A-D*
  A-E
  A-F
  B-D
  B-P
  C-D
  C-P

  *Cable A-D is not position controlled (its length is dependent on
  that of the others) but is included to provide upward tension on
  the crossbar of the T.

  Front view of the vertical triangle, looking in -z direction:

  *  Base Frame {B}
  *
  *        A
  *       / \
  *  ^   /   \
  *  |  /     \
  *  y /       \
  *   /         \
  *  B --------- C  x->

  The origin is at point B, giving three parameters necessary to
  characterize the frame given the convenient alignment of the
  coordinate frame: Cx, Ax and Ay.

  Top view of the horizontal twist bar, looking in the -y direction:

  *  Platform Frame {P}
  *
  *       D
  *       |
  *       |
  *       |
  *       |
  *  E----P----F x->
  *
  *       |
  *       v z

  The origin is at point P, giving three parameters necessary to
  characterize the T bar given the convenient alignment of the
  coordinate frame: Ex, Fx and Dz.

  The controlled point P is at the cross point of the T. The inverse
  kinematics problem is to find the cable lengths LAE, LAF, LBD, LBP,
  LCD and LCF given the coordinates of P in the {P} frame and the
  transform from platform to base TPB. The forward kinematics problem
  is to find TPB given the lengths LXX.

  The mapping from cables to joints is

  joints[0] = length A-E
  joints[1] = length A-F
  joints[2] = length B-D
  joints[3] = length B-P
  joints[4] = length C-D
  joints[5] = length C-P
  joints[6] = length A-D, for completeness
*/

typedef struct {
  go_real Cx;			/* x position of base point C */
  go_real Ax;			/* x position of base point A */
  go_real Ay;			/* y position of base point A */
  go_real Dz;			/* z position of bar point D */
  go_real Ex;			/* x position of bar point E */
  go_real Fx;			/* x position of bar point F */
  go_real LDE;			/* calculated length from D to E */
  go_flag fflags;		/* bits set from ROBOCH_D,E,P_NEGATIVE */
  /* no inverse flags necessary */
} roboch_kin_struct;

/*
  Bit flags for forward kinematics. The usual configuration
  is ROBOCH_P_POSITIVE only.
*/
enum {
  ROBOCH_D_POSITIVE = 1 << 0,
  ROBOCH_E_POSITIVE = 1 << 1,
  ROBOCH_P_POSITIVE = 1 << 2
};

extern go_integer roboch_kin_size(void); 

extern go_result roboch_kin_init(void * kins); 

extern const char * roboch_kin_get_name(void); 

extern go_integer roboch_kin_num_joints(void * kins);

extern go_result roboch_kin_fwd(void * kins,
				const go_real * joints,
				go_pose * world);

extern go_result roboch_kin_inv(void * kins,
				const go_pose * world,
				go_real * joints);

extern go_kin_type roboch_kin_get_type(void * kins); 

extern go_result roboch_kin_set_parameters(void * kins, go_link * params, go_integer num); 

extern go_result roboch_kin_get_parameters(void * kins, go_link * params, go_integer num); 

extern go_result roboch_kin_jac_inv(void * kins,
				    const go_pose * pos,
				    const go_pose * vel,
				    const go_real * joints, 
				    go_real * jointvels); 


extern go_result roboch_kin_jac_fwd(void * kins,
				    const go_real * joints,
				    const go_real * jointvels,
				    const go_pose * pos,
				    go_pose * vel);

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif /* ROBOCHKINS_H */
