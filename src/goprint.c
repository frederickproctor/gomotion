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
  \file goprint.c

  \brief Definitions of go data structure printing functions
*/

#include <stdio.h>
#include "gotypes.h"		/* go_integer,real */
#include "gomath.h"

void go_cart_print(const go_cart *cart)
{
  printf("%f %f %f\n", (double) cart->x, (double) cart->y, (double) cart->z);
}

void go_rpy_print(const go_rpy *rpy)
{
  printf("%f %f %f\n", (double) rpy->r, (double) rpy->p, (double) rpy->y);
}

void go_mat_print(const go_mat *m)
{
  printf("%f %f %f\n", (double) m->x.x, (double) m->y.x, (double) m->z.x);
  printf("%f %f %f\n", (double) m->x.y, (double) m->y.y, (double) m->z.y);
  printf("%f %f %f\n", (double) m->x.z, (double) m->y.z, (double) m->z.z);

  return;
}

void go_rvec_print(const go_rvec *rvec)
{
  printf("%f %f %f\n", (double) rvec->x, (double) rvec->y, (double) rvec->z);

  return;
}

void go_quat_print(const go_quat *quat)
{
  printf("%f %f %f %f\n", (double) quat->s, (double) quat->x, (double) quat->y, (double) quat->z);

  return;
}
