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
  \file go.h

  \brief Application include file. This includes all the Go headers
  you'll need, and the Go initialization macro.
*/

/*!
  \mainpage Go Motion

  Go Motion is a multi-axis motion controller.
*/

#ifndef GO_H
#define GO_H

#define GO_HEADER_VERSION 1

#include "gotypes.h"
#include "gomath.h"
#include "goutil.h"
#include "gotraj.h"
#include "gokin.h"
#include "gointerp.h"
#include "gomotion.h"
#include "golog.h"
#include "goio.h"
#include "goprint.h"

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

/* the library's compiled version, must match yours */
extern int go_lib_version;

/* this does any needed initialization */
extern int _go_init(void);

/*!
  Applications using Go Motion must call \a go_init before using any 
  other functions. \a go_init checks that the library against which
  the application was linked matches the headers against which the
  application was compiled. It also initializes any static data that
  may be referenced later.
*/
#define go_init() (GO_RESULT == 0 || GO_REAL == 0 || GO_INTEGER == 0 || GO_FLAG == 0 || GO_HEADER_VERSION != go_lib_version || _go_init())

/*!
  Applications should call \a go_exit when finished using any other
  functions. This lets \a Go clean up nicely.
*/
extern int go_exit(void);

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif /* GO_H */

/*!
  \defgroup EXTENDING Extending Go Motion

  Since Go Motion is available as source code, it can be extended to
  any degree desired depending on the effort willing to be
  expended. Here we describe step-by-step how a new feature was added.

  Edit \c trajintf.h and look for occurrences of "STUB" and
  "stub". Clone to suit your purposes.
*/
