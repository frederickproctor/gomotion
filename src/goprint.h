/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

/*!
  \file goprint.h

  \brief Declarations for printing go data structures
*/

#ifndef GO_PRINT_H
#define GO_PRINT_H

#include "gotypes.h"		/* go_integer,real */
#include "gomath.h"

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

extern void go_cart_print(const go_cart *cart);
extern void go_rpy_print(const go_rpy *rpy);
extern void go_mat_print(const go_mat *m);
extern void go_rvec_print(const go_rvec *rvec);
extern void go_quat_print(const go_quat *quat);
extern void go_matrix_print(const go_matrix *m);

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif /* GO_PRINT_H */
