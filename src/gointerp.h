#ifndef GOINTERP_H
#define GOINTERP_H

#include "gotypes.h"		/* go_real */

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

/*
  How many coefficients we support, one more than max polynomial degree 
*/
enum { GO_INTERP_COEFF_MAX = 6 };

/*
  This structure holds the points to be interpolated. See notes below for
  how to fill the p[] array for the particular type of interpolation.
*/
typedef struct {
  go_real p[GO_INTERP_COEFF_MAX];
} go_interp_points;

/*
  This structure holds the polynomial coefficients. In the derivations
  below, coefficients a,b,c,d,... are used. Here, they correspond to the
  a[] array like this:

  a[0] = 0th order coeff, e.g., 'd' for cubic
  a[1] = 1st order coeff, e.g., 'c' for cubic
  ...
*/
typedef struct {
  go_real a[GO_INTERP_COEFF_MAX];
} go_interp_coeff;

/*
  The interpolator structure

  For constant- and linear interpolation, exact-fit and boundary interpolation
  is the same, so we only have one type for these. Here we pass single points
  in succession, and interpolation is done at/between them. 

  For higher-order interpolation, we have two types, exact-fit and boundary.
  For exact-fit, we pass single points in succession, and interpolation is
  done in the middle interval. For boundary, we pass n-tuples of pos, vel,
  accel, etc. and interpolation is done between each tuple. 

  A variation on boundary interpolation is possible if the derivative values
  are not known but are estimated by differencing. The functions below
  suffixed _est are used for this.
*/

typedef struct {
  go_interp_coeff a;
  go_interp_points p;
} go_interp;

extern go_result go_interp_init(go_interp * i);

extern go_result go_interp_set_here(go_interp * i, go_real here);

extern go_result go_interp_calc_coeff_constant(const go_interp_points * p,
					       go_interp_coeff * a);
extern go_result go_interp_calc_coeff_linear(const go_interp_points * p,
					     go_interp_coeff * a);
extern go_result go_interp_calc_coeff_cubic_bc(const go_interp_points * p,
					       go_interp_coeff * a);
extern go_result go_interp_calc_coeff_cubic_pf(const go_interp_points * p,
					       go_interp_coeff * a);
extern go_result go_interp_calc_coeff_quintic_bc(const go_interp_points * p,
						 go_interp_coeff * a);
extern go_result go_interp_calc_coeff_quintic_pf(const go_interp_points * p,
						 go_interp_coeff * a);
extern go_result go_interp_calc_coeff_bc(go_integer order,
					 const go_interp_points * p,
					 go_interp_coeff * a);
extern go_result go_interp_calc_coeff_pf(go_integer order,
					 const go_interp_points * p,
					 go_interp_coeff * a);

extern go_real go_interp_eval_constant(const go_interp * i, go_real t);
extern go_real go_interp_eval_linear(const go_interp * i, go_real t);
extern go_real go_interp_eval_cubic(const go_interp * i, go_real t);
extern go_real go_interp_eval_quintic(const go_interp * i, go_real t);

extern go_result go_interp_add_constant(go_interp * i, go_real pos);
extern go_result go_interp_add_linear(go_interp * i, go_real pos);
extern go_result go_interp_add_cubic_pv(go_interp * i, go_real pos,
					go_real vel);
extern go_result go_interp_add_cubic_pdv(go_interp * i, go_real pos);
extern go_result go_interp_add_cubic_pf(go_interp * i, go_real pos);
extern go_result go_interp_add_quintic_pva(go_interp * i, go_real pos,
					   go_real vel, go_real acc);
extern go_result go_interp_add_quintic_pvda(go_interp * i, go_real pos,
					    go_real vel);
extern go_result go_interp_add_quintic_pdva(go_interp * i, go_real pos);
extern go_result go_interp_add_quintic_pf(go_interp * i, go_real pos);

typedef go_result (*go_interp_add_func)(go_interp * i, go_real pos);
typedef go_real (*go_interp_eval_func)(const go_interp * i, go_real t);

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif /* GOINTERP_H */
