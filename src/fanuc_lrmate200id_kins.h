/*
  fanuc_lrmate200id_kins.h
*/

#ifndef FANUC_LRMATE200ID_KINS_H
#define FANUC_LRMATE200ID_KINS_H

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

#define FANUC_LRMATE200ID_KIN_NUM_JOINTS 6

#define FANUC_LRMATE200ID_KIN_A1 0.050
#define FANUC_LRMATE200ID_KIN_A2 0.330
#define FANUC_LRMATE200ID_KIN_A3 0.035
#define FANUC_LRMATE200ID_KIN_D2 0.000
#define FANUC_LRMATE200ID_KIN_D3 0.000
#define FANUC_LRMATE200ID_KIN_D4 0.335
#define FANUC_LRMATE200ID_KIN_IFLAGS 0

typedef struct {
  three21_kin_struct tk;
  go_pose t7;			/* final tool transform */
  go_pose t7_inv;		/* its inverse */
} fanuc_lrmate200id_kin_struct;

extern go_integer fanuc_lrmate200id_kin_size(void); 

extern go_result fanuc_lrmate200id_kin_init(fanuc_lrmate200id_kin_struct *kins); 

extern const char *fanuc_lrmate200id_kin_get_name(void); 

extern go_integer fanuc_lrmate200id_kin_num_joints(fanuc_lrmate200id_kin_struct *kins);

extern go_result fanuc_lrmate200id_kin_fwd(fanuc_lrmate200id_kin_struct *kins,
					   const go_real *motors,
					   go_pose *world);

extern go_result fanuc_lrmate200id_kin_inv(fanuc_lrmate200id_kin_struct *kins,
					   const go_pose *world,
					   go_real *motors);

extern go_kin_type fanuc_lrmate200id_kin_get_type(fanuc_lrmate200id_kin_struct *kins); 

extern go_result fanuc_lrmate200id_kin_set_parameters(fanuc_lrmate200id_kin_struct *kins, go_link *params, go_integer num);

extern go_result fanuc_lrmate200id_kin_get_parameters(fanuc_lrmate200id_kin_struct *kins, go_link *params, go_integer num); 

extern go_result fanuc_lrmate200id_kin_jac_inv(fanuc_lrmate200id_kin_struct *kins,
					       const go_pose *pos,
					       const go_vel *vel,
					       const go_real *motors, 
					       go_real *motorvels); 


extern go_result fanuc_lrmate200id_kin_jac_fwd(fanuc_lrmate200id_kin_struct *kins,
					       const go_real *motors,
					       const go_real *motorvels,
					       const go_pose *pos, 
					       go_vel *vel); 

extern go_result fanuc_lrmate200id_kin_set_flags(fanuc_lrmate200id_kin_struct *kins,
						 go_flag fflags,
						 go_flag iflags);

extern go_result fanuc_lrmate200id_kin_get_flags(fanuc_lrmate200id_kin_struct *kins,
						 go_flag *fflags,
						 go_flag *iflags);

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif
