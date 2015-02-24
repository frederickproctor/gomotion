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
  \file extintf.h

  \brief Declarations for functions that interface to the sensors
  and actuators of the external world.
*/

/*!
  \defgroup EXTINF The External Interface

  The external interface defines function names, such as \a
  ext_read_pos, that are called throughout \b Go when it interfaces to
  the sensors and actuators of the external world. These are declared
  in \ref extintf.h.  The implementation of these functions is done in
  various files conventionally prefixed with "ext_", such as \ref
  ext_sim.c for the simulation. \ref ext_stub.c is a starting point
  for adding support for a new interface board. \ref ext_s626.c is an
  example of one for the Sensoray 626 board.
*/

#ifndef EXTINTF_H
#define EXTINTF_H

#include "gotypes.h"

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

/*!
  How long an external interface initialization string can be.
*/
#define EXT_INIT_STRING_LENGTH 256

extern go_result ext_init(char * init_string);

extern go_result ext_quit(void);

extern go_result ext_joint_init(go_integer joint, go_real cycle_time);

extern go_result ext_joint_enable(go_integer joint);

extern go_result ext_joint_disable(go_integer joint);

extern go_result ext_joint_quit(go_integer joint);

/*!
  Reads the position out of the external device.
 */
extern go_result
ext_read_pos(go_integer joint,	/*!< The joint to read.  */
	     go_real * pos	/*!< Where the position is stored. */
	     );

/*!
  Writes a position setpoint to the external device. This is used
  in "pass through" servos, where the Go Motion servo control just
  passes setpoints out to an external system that does the actual
  servoing.
 */
extern go_result
ext_write_pos(go_integer joint,	/*!< The joint to set. */
	      go_real pos	/*!< The position setpoint.  */
	      );

/*!
  Writes a speed setpoint to the external device. Used to send
  outputs to a device under closed-loop control. 
 */
extern go_result
ext_write_vel(go_integer joint,	/*!< The joint to set. */
	      go_real vel	/*!< The speed or output value. */
	      );

/*! Request that the joint's home position be latched. */
extern go_result ext_joint_home(go_integer joint);

/*! Query if the joint has met its homed condition. */
extern go_flag ext_joint_is_home(go_integer joint);

/*! Read the joint's homed position into \a pos. */
extern go_result ext_joint_home_latch(go_integer joint, go_real * pos);

/*!
  The generic input/output model is based on integer indices into
  ADC, DAC and DIO devices. The values read or written are in user
  units, e.g., volts, and are converted to raw units by the underlying
  external interface implementation.
*/

extern go_integer ext_num_ain(void);
extern go_integer ext_num_aout(void);
extern go_integer ext_num_din(void);
extern go_integer ext_num_dout(void);

extern go_result ext_trigger_in(void);

extern go_result ext_read_ain(go_integer index, go_real * val);

extern go_result ext_write_aout(go_integer index, go_real val);

extern go_result ext_read_din(go_integer index, go_flag * val);

extern go_result ext_write_dout(go_integer index, go_flag val);

extern go_result ext_set_parameters(go_integer joint, go_real * values, go_integer number);

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif
