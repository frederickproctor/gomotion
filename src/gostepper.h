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
  \file gostepper.h

  \brief Declarations for stepper motor interface.
*/

/*!
  \defgroup GOSTEPPER The Stepper Motor Driver

  The stepper motor driver runs at the underlying timer base period
  set in the hardware abstraction layer.  It provides a shared memory
  interface into which applications can write desired frequencies for
  the stepper motor square wave outputs. The driver maintains an
  accumulated count of steps that have occurred, which can be read out
  of the shared memory interface. Configuration settings such as ports
  and fine-tuning are also settable.
*/

#ifndef GOSTEPPER_H
#define GOSTEPPER_H

#include <rtapi.h>		/* rtapi_integer */

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

/*! The default shared memory key */
#define GO_STEPPER_DEFAULT_SHM_KEY 301

/*! How many joints are supported in the data structure, make at least 6 */
#define GO_STEPPER_NUM 6

enum {
  /*! Direction and step, with direction bit 0 and step bit 1,
    e.g., for Microkinetics Mighty Drive */
  GO_STEPPER_DIRSTEP = 1,
  /*! Step and direction, with step bit 0 and direction bit 1, 
    e.g. for Xylotex default wiring */
  GO_STEPPER_STEPDIR,
  /*! Two-bit Gray coding  */
  GO_STEPPER_GRAYCODE_2BIT,
  /*! Four-bit Gray coding  */
  GO_STEPPER_GRAYCODE_4BIT
};

/*
  FIXME-- consider head-tail mutexing these. They're all atomic
  and don't really need it now.
*/

typedef struct {
  /* INPUTS to stepper motor task */

  /*! IO port address for low-index outputs 1, ... */
  rtapi_integer lo_port;
  /*! IO port address for high-index outputs ..., GO_STEPPER_NUM */
  rtapi_integer hi_port;
  /*! Signed frequency, outputs per second */
  rtapi_integer freq[GO_STEPPER_NUM];
  /*! Minimum number of counts to hold output before changing,
    applies to up portion for step/dir */
  rtapi_integer min_up_count[GO_STEPPER_NUM];
  /*! Minimum number of counts for down portion of output for step/dir */
  rtapi_integer min_down_count[GO_STEPPER_NUM];
  /*! Accumulate a count on the up transition, else on the down transition,
   if doing step/dir */
  rtapi_flag count_on_up[GO_STEPPER_NUM];

  /* OUTPUTS from stepper motor task */

  /*! Heartbeat, for detecting that the stepper controller is running */
  rtapi_integer heartbeat;
  /*! Signed accumulated position, counts */
  rtapi_integer count[GO_STEPPER_NUM];
} go_stepper_struct;

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif /* GOSTEPPER_H */
