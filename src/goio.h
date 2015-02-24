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
  \file goio.h

  \brief Declarations for the input/output data structure
*/

#ifndef GOIO_H
#define GOIO_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

/*
  Max number of input/output types we support; make this larger
   than any that Go will likely encounter, and the unused ones
   will just be 0.
*/
enum {GO_IO_NUM_AIN = 32};
enum {GO_IO_NUM_AOUT = 32};
enum {GO_IO_NUM_DIN = 64};
enum {GO_IO_NUM_DOUT = 64};

/*
  The shared memory for the go_io_struct is allocated by gomain, using
  the SHM_KEY in the GOIO section of the .ini file.

  The first servo's job is to read the outputs and write them to the
  external interface, and read the external interface and write the 
  inputs.
*/
typedef struct {
  unsigned char head;
  go_real ain[GO_IO_NUM_AIN];
  go_flag din[GO_IO_NUM_DIN];
  unsigned char tail;
} go_input_struct;

typedef struct {
  unsigned char head;
  go_real aout[GO_IO_NUM_AOUT];
  go_flag dout[GO_IO_NUM_DOUT];
  unsigned char tail;
} go_output_struct;

typedef struct {
  go_integer num_ain;		/* how many there actually are, could
				   be smaller than GO_IO_NUM_AIN if
				   the external interface has fewer */
  go_integer num_aout;		/* ditto */
  go_integer num_din;
  go_integer num_dout;
  go_input_struct input;
  go_output_struct output;
} go_io_struct;

/* declaration for the global go_io_struct set up by gomain */
extern go_io_struct * global_go_io_ptr;

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif
