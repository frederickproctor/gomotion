/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#include <stddef.h>		/* sizeof */
#include <math.h>		/* sqrt */
#include "gotypes.h"		/* go_result, go_integer */
#include "gomath.h"		/* go_pose */
#include "roboch.h"		/* these decls */

/* despite these defs, the values are assumed to be 7 and 6 and can't
   simply be changed here */
#define ROBOCH_NUM_JOINTS 6 	/* does not include non-actuated AD */

go_integer roboch_kin_size(void)
{
  return (go_integer) sizeof(roboch_kin_struct);
} 

/* default values are for a unit-side equilateral triangle for the frame
   and unit length bar ends */

#define SCALE 1.0
#define ROBOCH_CX (SCALE)
#define ROBOCH_AX (SCALE * 0.5) /* cos 60 */
#define ROBOCH_AY (SCALE * 0.866025403784439) /* sin 60 */
#define ROBOCH_DZ (-SCALE)
#define ROBOCH_EX (-SCALE)
#define ROBOCH_FX (SCALE)

go_result roboch_kin_init(void * kins)
{
  roboch_kin_struct * roboch = (roboch_kin_struct *) kins;

  roboch->Cx = ROBOCH_CX;
  roboch->Ax = ROBOCH_AX;
  roboch->Ay = ROBOCH_AY;
  roboch->Dz = ROBOCH_DZ;
  roboch->Ex = ROBOCH_EX;
  roboch->Fx = ROBOCH_FX;
  roboch->LDE = sqrt(go_sq(ROBOCH_DZ) + go_sq(ROBOCH_EX));

  return GO_RESULT_OK;
} 

const char * roboch_kin_get_name(void)
{
  return "robochkins";
} 

go_integer roboch_kin_num_joints(void * kins)
{
  return ROBOCH_NUM_JOINTS;
}

/*
  The mapping from cables to joints is


  joints[0] = length A-E
  joints[1] = length A-F
  joints[2] = length B-D
  joints[3] = length B-P
  joints[4] = length C-D
  joints[5] = length C-P
*/

/* use these as array indexes for joints[] */
enum {AE = 0, AF, BD, BP, CD, CP};

go_result roboch_kin_fwd(void * kins,
			 const go_real * joints,
			 go_pose * world)
{
  roboch_kin_struct * roboch;
  go_cart A_in_B, B_in_B, C_in_B;
  go_cart D_in_B, E_in_B;
  go_real Dz, Ex, Fx;		/* abs vals */
  go_real LAP;
  go_cart tripos, trineg;
  go_mat mat;			/* rotation matrix of {P} in {B} */
  go_result retval;

  roboch = (roboch_kin_struct *) kins;

  /* set up our trivial base points in {B} from kin params */
  A_in_B.x = roboch->Ax, A_in_B.y = roboch->Ay, A_in_B.z = 0;
  B_in_B.x = B_in_B.y = B_in_B.z = 0;
  C_in_B.x = roboch->Cx, C_in_B.y = 0, C_in_B.z = 0;

  /* get kin params as positive numbers */
  Dz = fabs(roboch->Dz);
  Ex = fabs(roboch->Ex);
  Fx = fabs(roboch->Fx);

  /* get LAP */
  LAP = sqrt(((go_sq(joints[AE]) - go_sq(Ex)) * Fx +
	      (go_sq(joints[AF]) - go_sq(Fx)) * Ex) / 
	     (Ex + Fx));	

  /* trilaterate from BCA to get point P in {B} */
  retval = 
    go_cart_trilaterate(&B_in_B, &C_in_B, &A_in_B,
			joints[BP], joints[CP], LAP,
			&tripos, &trineg);
  if (GO_RESULT_OK != retval) return retval;
  if (roboch->fflags & ROBOCH_P_POSITIVE) world->tran = tripos;
  else world->tran = trineg;

  /* trilaterate from BCP to get point D in {B} */
  retval = 
    go_cart_trilaterate(&B_in_B, &C_in_B, &world->tran,
			joints[BD], joints[CD], Dz,
			&tripos, &trineg);
  if (GO_RESULT_OK != retval) return retval;
  if (roboch->fflags & ROBOCH_D_POSITIVE) D_in_B = tripos;
  else D_in_B = trineg;

  /* trilaterate from ADP to get point E in {B} */
  retval = 
    go_cart_trilaterate(&A_in_B, &D_in_B, &world->tran,
			joints[AE], roboch->LDE, Ex,
			&tripos, &trineg);
  if (GO_RESULT_OK != retval) return retval;
  if (roboch->fflags & ROBOCH_E_POSITIVE) E_in_B = tripos;
  else E_in_B = trineg;

  /* 
     now for the orientation-- get x as P-E, z as P-D
     and y as z cross x
  */
  go_cart_cart_sub(&world->tran, &E_in_B, &mat.x);
  go_cart_unit(&mat.x, &mat.x);
  go_cart_cart_sub(&world->tran, &D_in_B, &mat.z);
  go_cart_unit(&mat.z, &mat.z);
  go_cart_cart_cross(&mat.z, &mat.x, &mat.y);
  go_mat_norm(&mat, &mat);
  go_mat_quat_convert(&mat, &world->rot);

  return GO_RESULT_OK;
} 

go_result roboch_kin_inv(void * kins,
			 const go_pose * world,
			 go_real * joints)
{
  roboch_kin_struct * roboch;
  go_cart A_in_B, B_in_B, C_in_B; /* points ABC in their base frame */
  go_cart D_in_P, E_in_P, F_in_P; /* points DEF in their platform frame */
  go_cart D_in_B, E_in_B, F_in_B; /* points DEF in the base frame */
  go_cart diff;
  go_cart x, xy, z;
  go_real dot;

  roboch = (roboch_kin_struct *) kins;

  A_in_B.x = roboch->Ax, A_in_B.y = roboch->Ay, A_in_B.z = 0;
  B_in_B.x = 0, B_in_B.y = 0, B_in_B.z = 0;
  C_in_B.x = roboch->Cx, C_in_B.y = 0, C_in_B.z = 0;

  D_in_P.x = 0, D_in_P.y = 0, D_in_P.z = roboch->Dz;
  E_in_P.x = roboch->Ex, E_in_P.y = 0, E_in_P.z = 0;
  F_in_P.x = roboch->Fx, F_in_P.y = 0, F_in_P.z = 0;

  /* transform {P} frame points to {B} frame */
  go_pose_cart_mult(world, &D_in_P, &D_in_B);
  go_pose_cart_mult(world, &E_in_P, &E_in_B);
  go_pose_cart_mult(world, &F_in_P, &F_in_B);

  /* LAE */
  go_cart_cart_sub(&A_in_B, &E_in_B, &diff);
  go_cart_mag(&diff, &joints[AE]);

  /* LAF */
  go_cart_cart_sub(&A_in_B, &F_in_B, &diff);
  go_cart_mag(&diff, &joints[AF]);

  /* LBD */
  go_cart_cart_sub(&B_in_B, &D_in_B, &diff);
  go_cart_mag(&diff, &joints[BD]);

  /* LBP */
  go_cart_cart_sub(&B_in_B, &world->tran, &diff);
  go_cart_mag(&diff, &joints[BP]);

  /* LCD */
  go_cart_cart_sub(&C_in_B, &D_in_B, &diff);
  go_cart_mag(&diff, &joints[CD]);

  /* LCP */
  go_cart_cart_sub(&C_in_B, &world->tran, &diff);
  go_cart_mag(&diff, &joints[CP]);

  /*
    set flags by dotting points with their trilateration triangle z
    vectors
  */

  roboch->fflags = 0;

  /* we get D from BCP */
  go_cart_cart_sub(&C_in_B, &B_in_B, &x);
  go_cart_cart_sub(&world->tran, &B_in_B, &xy);
  go_cart_cart_cross(&x, &xy, &z);
  go_cart_cart_sub(&D_in_B, &B_in_B, &diff);
  go_cart_cart_dot(&diff, &z, &dot);
  if (dot >= 0.0) roboch->fflags |= ROBOCH_D_POSITIVE;

  /* we get E from ADP */
  go_cart_cart_sub(&D_in_B, &A_in_B, &x);
  go_cart_cart_sub(&world->tran, &A_in_B, &xy);
  go_cart_cart_cross(&x, &xy, &z);
  go_cart_cart_sub(&E_in_B, &A_in_B, &diff);
  go_cart_cart_dot(&diff, &z, &dot);
  if (dot >= 0.0) roboch->fflags |= ROBOCH_E_POSITIVE;

  /* we get P, aka world->tran, from BCA */
  go_cart_cart_sub(&C_in_B, &B_in_B, &x);
  go_cart_cart_sub(&A_in_B, &B_in_B, &xy);
  go_cart_cart_cross(&x, &xy, &z);
  go_cart_cart_sub(&world->tran, &B_in_B, &diff);
  go_cart_cart_dot(&diff, &z, &dot);
  if (dot >= 0.0) roboch->fflags |= ROBOCH_P_POSITIVE;

  return GO_RESULT_OK;
} 

go_kin_type roboch_kin_get_type(void * kins)
{
  return GO_KIN_BOTH;
} 

go_result roboch_kin_set_parameters(void * kins, go_link * params, go_integer num)
{
  roboch_kin_struct * roboch = (roboch_kin_struct *) kins;

  if (num < ROBOCH_NUM_JOINTS) return GO_RESULT_ERROR;

  roboch->Cx = params[0].u.dh.d;
  roboch->Ax = params[1].u.dh.d;
  roboch->Ay = params[2].u.dh.d;
  roboch->Dz = params[3].u.dh.d;
  roboch->Ex = params[4].u.dh.d;
  roboch->Fx = params[5].u.dh.d;
  roboch->LDE = sqrt(go_sq(roboch->Dz) + go_sq(roboch->Ex));

  return GO_RESULT_OK;
} 

go_result roboch_kin_get_parameters(void * kins, go_link * params, go_integer num)
{
  roboch_kin_struct * roboch = (roboch_kin_struct *) kins;

  if (num > ROBOCH_NUM_JOINTS) return GO_RESULT_ERROR;

  params[0].u.dh.d = roboch->Cx;
  params[1].u.dh.d = roboch->Ax;
  params[2].u.dh.d = roboch->Ay;
  params[3].u.dh.d = roboch->Dz;
  params[4].u.dh.d = roboch->Ex;
  params[5].u.dh.d = roboch->Fx;

  return GO_RESULT_OK;
} 

/*
  Build the inverse Jacobian matrix, 6x6, such that

  [v1 v2 v3 v4 v5 v6]T = J[6][6] * [vx vy vz vr vp vw]T

  where vi are the cable speeds (ignoring the seventh unactuated cable),
  and the input vector is the Cartesian speed and RPY speed.
*/
static go_result jac_inv_mat(roboch_kin_struct * roboch,
			     const go_pose * pos,
			     go_real mat[6][6])
{
  go_cart A_in_B, B_in_B, C_in_B; /* A,B,C points in {B} */
  go_cart D_in_P, E_in_P, F_in_P; /* D,E,F points in {P} */
  go_cart DV_in_B, EV_in_B, FV_in_B; /* D,E,F vectors rotated to {B} */
  go_cart D_in_B, E_in_B, F_in_B; /* D,E,F points in {B} */
  go_cart VAD, VAE, VAF, VBD, VBE, VCD;	/* struct vectors */
  go_cart DXAD, EXAE, FXAF, DXBD, EXBE, DXCD; /* strut cross products */
  go_result retval;

  A_in_B.x = roboch->Ax, A_in_B.y = roboch->Ay, A_in_B.z = 0;
  B_in_B.x = B_in_B.y = B_in_B.z = 0;
  C_in_B.x = roboch->Cx, C_in_B.y = 0, C_in_B.z = 0;

  D_in_P.x = 0, D_in_P.y = 0, D_in_P.z = roboch->Dz;
  E_in_P.x = roboch->Ex, E_in_P.y = 0, E_in_P.z = 0;
  F_in_P.x = roboch->Fx, F_in_P.y = 0, F_in_P.z = 0;

  go_quat_cart_mult(&pos->rot, &D_in_P, &DV_in_B);
  go_quat_cart_mult(&pos->rot, &E_in_P, &EV_in_B);
  go_quat_cart_mult(&pos->rot, &F_in_P, &FV_in_B);

  go_cart_cart_add(&pos->tran, &DV_in_B, &D_in_B);
  go_cart_cart_add(&pos->tran, &EV_in_B, &E_in_B);
  go_cart_cart_add(&pos->tran, &FV_in_B, &F_in_B);

  go_cart_cart_sub(&D_in_B, &A_in_B, &VAD);
  go_cart_cart_sub(&E_in_B, &A_in_B, &VAE);
  go_cart_cart_sub(&F_in_B, &A_in_B, &VAF);
  go_cart_cart_sub(&D_in_B, &B_in_B, &VBD);
  go_cart_cart_sub(&E_in_B, &B_in_B, &VBE);
  go_cart_cart_sub(&D_in_B, &C_in_B, &VCD);

  retval = go_cart_unit(&VAD, &VAD);
  if (GO_RESULT_OK != retval) return retval;
  retval = go_cart_unit(&VAE, &VAE);
  if (GO_RESULT_OK != retval) return retval;
  retval = go_cart_unit(&VAF, &VAF);
  if (GO_RESULT_OK != retval) return retval;
  retval = go_cart_unit(&VBD, &VBD);
  if (GO_RESULT_OK != retval) return retval;
  retval = go_cart_unit(&VBE, &VBE);
  if (GO_RESULT_OK != retval) return retval;
  retval = go_cart_unit(&VCD, &VCD);
  if (GO_RESULT_OK != retval) return retval;

  go_cart_cart_cross(&DV_in_B, &VAD, &DXAD);
  go_cart_cart_cross(&EV_in_B, &VAE, &EXAE);
  go_cart_cart_cross(&FV_in_B, &VAF, &FXAF);
  go_cart_cart_cross(&DV_in_B, &VBD, &DXBD);
  go_cart_cart_cross(&EV_in_B, &VBE, &EXBE);
  go_cart_cart_cross(&DV_in_B, &VCD, &DXCD);

  mat[0][0] = VAD.x, mat[0][1] = VAD.y, mat[0][2] = VAD.z,
    mat[0][3] = DXAD.x, mat[0][4] = DXAD.y, mat[0][5] = DXAD.z;

  mat[1][0] = VAE.x, mat[1][1] = VAE.y, mat[1][2] = VAE.z,
    mat[1][3] = EXAE.x, mat[1][4] = EXAE.y, mat[1][5] = EXAE.z;

  mat[2][0] = VAF.x, mat[2][1] = VAF.y, mat[2][2] = VAF.z,
    mat[2][3] = FXAF.x, mat[2][4] = FXAF.y, mat[2][5] = FXAF.z;

  mat[3][0] = VBD.x, mat[3][1] = VBD.y, mat[3][2] = VBD.z,
    mat[3][3] = DXBD.x, mat[3][4] = DXBD.y, mat[3][5] = DXBD.z;

  mat[4][0] = VBE.x, mat[4][1] = VBE.y, mat[4][2] = VBE.z,
    mat[4][3] = EXBE.x, mat[4][4] = EXBE.y, mat[4][5] = EXBE.z;

  mat[5][0] = VCD.x, mat[5][1] = VCD.y, mat[5][2] = VCD.z,
    mat[5][3] = DXCD.x, mat[5][4] = DXCD.y, mat[5][5] = DXCD.z;

  return GO_RESULT_OK;
}

go_result roboch_kin_jac_inv(void * kins,
			     const go_pose * pos,
			     const go_pose * vel,
			     const go_real * joints, 
			     go_real * jointvels)
{
  go_real jac_inv[6][6];
  go_real vel_vec[6];		/* vx vx vz vr vp vw */
  go_rpy rpy;
  go_result retval;

  retval = jac_inv_mat(kins, pos, jac_inv);
  if (GO_RESULT_OK != retval) return retval;

  go_quat_rpy_convert(&vel->rot, &rpy);
  vel_vec[0] = vel->tran.x;
  vel_vec[1] = vel->tran.y;
  vel_vec[2] = vel->tran.z;
  vel_vec[3] = rpy.r;
  vel_vec[4] = rpy.p;
  vel_vec[5] = rpy.y;

  (void) go_mat6_vec6_mult(jac_inv, vel_vec, jointvels);

  return GO_RESULT_OK;
} 

go_result roboch_kin_jac_fwd(void * kins,
			     const go_real * joints,
			     const go_real * jointvels,
			     const go_pose * pos,
			     go_pose * vel)
{
  go_real jac[6][6];
  go_real jac_inv[6][6];
  go_real vel_vec[6];
  go_rpy rpy;
  go_result retval;

  retval = jac_inv_mat(kins, pos, jac_inv);
  if (GO_RESULT_OK != retval) return retval;

  retval = go_mat6_inv(jac_inv, jac);
  if (GO_RESULT_OK != retval) return retval;

  (void) go_mat6_vec6_mult(jac, jointvels, vel_vec);

  vel->tran.x = vel_vec[0];
  vel->tran.y = vel_vec[1];
  vel->tran.z = vel_vec[2];
  rpy.r = vel_vec[3];
  rpy.p = vel_vec[4];
  rpy.y = vel_vec[5];
  go_rpy_quat_convert(&rpy, &vel->rot);

  return GO_RESULT_OK;
}
