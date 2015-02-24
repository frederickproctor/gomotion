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
  \file ext_galil.c

  \brief External interface to a Galil-based system.
*/

/*
  Principles of Operation

  This interface supposes a serial connection to each of the
  motors under Galil control. Socket emulation is used for testing.
  There can be up to SERVO_NUM motors. A data structure for motor
  status is set up for each motor.

  Each task is passed the index into its shared memory.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>		/* sprintf */
#include <string.h>		/* strlen */
#include <stddef.h>		/* NULL */
#include <stdlib.h>		/* atoi */
#include <ctype.h>		/* isspace */
#include <rtapi.h>		/* rtapi_string_to_integer */
#include "gotypes.h"
#include "extintf.h"
#include "servointf.h"		/* SERVO_NUM */

typedef struct {
  void * task;
  void * mutex;
  rtapi_integer socket_id;
  rtapi_integer period_nsec;
  go_real position;
} galil_struct;

static galil_struct galils[SERVO_NUM];

void taskcode(void * args)
{
  void * task;
  void * mutex;
  rtapi_integer socket_id;
  rtapi_integer * perptr;
  go_real * posptr;
  enum {BUFFERLEN = 256};
  char buffer[BUFFERLEN];
  rtapi_integer nchars;
  rtapi_integer period_nsec;

#define ARGIT(s) s = ((galil_struct *) args)->s
  ARGIT(task);
  ARGIT(mutex);
  ARGIT(socket_id);
  perptr = &(((galil_struct *) args)->period_nsec);
  posptr = &(((galil_struct *) args)->position);

  if (socket_id < 0) {
    rtapi_print("ext_galil: invalid socket\n");
    return;
  }

  for (;;) {
    strcpy(buffer, "TP\r");
    rtapi_mutex_take(mutex);
    (void) rtapi_socket_write(socket_id, buffer, strlen(buffer) + 1);
    rtapi_mutex_give(mutex);
    nchars = rtapi_socket_read(socket_id, buffer, BUFFERLEN);
    if (nchars > 0) {
      rtapi_mutex_take(mutex);
      *posptr = (go_real) atoi(buffer);
      rtapi_mutex_give(mutex);
    }
    /*
      This update task should not run so fast as to overwhelm the
      external Galil system with TP messages. It should run about
      as fast as the trajectory loop, and the Galil system should
      be faster than that. If the Galil system is slower than the
      trajectory loop, we have a design problem. 

      For the socket test, the Galil emulation in emu_galil.c builds
      in a delay of 10 milliseconds, nominally. This is faster than
      the 80 millisecond typical trajectory loop time. 
    */
    rtapi_mutex_take(mutex);
    period_nsec = *perptr;	/* ext_joint_init also shares this */
    rtapi_mutex_give(mutex);
    rtapi_wait(period_nsec);
  }
}

/*
  If we get a number, then we're supposed to open a socket to the
  server and we'll be running in debug testing mode using the Galil
  emulation on the socket server side.

  Otherwise, we'll just loop position writes to position reads
  immediately as simple emulation locally. 
*/
go_result ext_init(char * init_string)
{
  galil_struct * args;
  const char * ptr;
  rtapi_integer i, servo_num;
  rtapi_result retval;

  if (NULL == init_string) {
    return GO_RESULT_OK;
  }

  for (servo_num = 0, ptr = init_string;
       servo_num < SERVO_NUM;
       servo_num++, ptr = rtapi_string_skipone(ptr)) {
    args = &galils[servo_num];
    args->socket_id = -1;
    args->period_nsec = 1000000000;
    if (RTAPI_OK == rtapi_string_to_integer(ptr, &i)) {
      args->socket_id = rtapi_socket_client(i, "localhost");
      if (args->socket_id < 0) {
	rtapi_print("ext_galil: can't connect to %d\n", (int) i);
      } else {
	args->task = rtapi_task_new();
	if (NULL == args->task) {
	  rtapi_print("ext_galil: can't allocate task\n");
	  args->socket_id = -1;
	} else {
	  args->mutex = rtapi_mutex_new(i);
	  if (NULL == args->mutex) {
	    rtapi_print("ext_galil: can't allocate mutex\n");
	    args->socket_id = -1;
	  } else {
	    (void) rtapi_mutex_give(args->mutex);
	    retval = rtapi_task_start(args->task,
				      taskcode,
				      args,
				      rtapi_prio_highest(),
				      1024,
				      args->period_nsec,
				      1);
	    if (RTAPI_OK != retval) {
	      rtapi_print("ext_galil: can't start task\n");
	      rtapi_task_delete(args->task);
	      args->socket_id = -1;
	    } else {
	      rtapi_print("ext_galil: got port %d\n", (int) i);
	    }
	  }
	}
      }
    }
  }

  return GO_RESULT_OK;
}
 
go_result ext_quit(void)
{
  return GO_RESULT_OK;
}

go_result ext_joint_init(go_integer joint, go_real cycle_time)
{
  if (joint < 0 || joint >= SERVO_NUM) return GO_RESULT_ERROR;

  if (galils[joint].socket_id < 0) {
    galils[joint].position = 0.0;
  } else {
    rtapi_mutex_take(galils[joint].mutex);
    /* taskcode also shares these */
    galils[joint].position = 0.0;
    galils[joint].period_nsec = (rtapi_integer) (cycle_time * 1.0e9 + 0.5);
    rtapi_mutex_give(galils[joint].mutex);
  }

  return GO_RESULT_OK;
}

go_result ext_joint_enable(go_integer joint)
{
  return GO_RESULT_OK;
}

go_result ext_joint_disable(go_integer joint)
{
  return GO_RESULT_OK;
}

go_result ext_joint_quit(go_integer joint)
{
  return GO_RESULT_OK;
}

go_result ext_read_pos(go_integer joint, go_real * pos)
{
  if (joint < 0 || joint >= SERVO_NUM) return GO_RESULT_ERROR;

  if (galils[joint].socket_id < 0) {
    *pos = galils[joint].position;
  } else {
    rtapi_mutex_take(galils[joint].mutex);
    *pos = galils[joint].position;
    rtapi_mutex_give(galils[joint].mutex);
  }

  return GO_RESULT_OK;
}

go_result ext_write_pos(go_integer joint, go_real pos)
{
  enum {BUFFERLEN = 256};
  char buffer[BUFFERLEN];

  if (joint < 0 || joint >= SERVO_NUM) return GO_RESULT_ERROR;

  if (galils[joint].socket_id < 0) {
    galils[joint].position = pos;
  } else {
    sprintf(buffer, "PA%d;BG\r", (int) pos);
    rtapi_mutex_take(galils[joint].mutex);
    (void) rtapi_socket_write(galils[joint].socket_id, buffer, strlen(buffer) + 1);
    rtapi_mutex_give(galils[joint].mutex);
  }

  return GO_RESULT_OK;
}

go_result ext_write_vel(go_integer joint, go_real vel)
{
  if (joint < 0 || joint >= SERVO_NUM) return GO_RESULT_ERROR;

  return GO_RESULT_OK;
}

go_result ext_joint_home(go_integer joint)
{
  if (joint < 0 || joint >= SERVO_NUM) return GO_RESULT_ERROR;
  
  return GO_RESULT_OK;
}

go_flag ext_joint_is_home(go_integer joint)
{
  if (joint < 0 || joint >= SERVO_NUM) return 1; /* always homed */

  return 1;			/* always homed */
}

go_result ext_joint_home_latch(go_integer joint, go_real * pos)
{
  if (joint < 0 || joint >= SERVO_NUM) return GO_RESULT_ERROR;

  *pos = 0.0;

  return GO_RESULT_OK;
}

enum {AIN_NUM = 0};
enum {AOUT_NUM = 0};
enum {DIN_NUM = 0};
enum {DOUT_NUM = 0};

go_integer ext_num_ain(void)
{
  return AIN_NUM;
}

go_integer ext_num_aout(void)
{
  return AOUT_NUM;
}

go_integer ext_num_din(void)
{
  return DIN_NUM;
}

go_integer ext_num_dout(void)
{
  return DOUT_NUM;
}

go_result ext_trigger_in(void)
{
  return GO_RESULT_OK;
}

go_result ext_read_ain(go_integer index, go_real * val)
{
  *val = 0.0;

  return GO_RESULT_OK;
}

go_result ext_write_aout(go_integer index, go_real val)
{
  return GO_RESULT_OK;
}

go_result ext_read_din(go_integer index, go_flag * val)
{
  *val = 0;

  return GO_RESULT_OK;
}

go_result ext_write_dout(go_integer index, go_flag val)
{
  return GO_RESULT_OK;
}

go_result ext_set_parameters(go_integer joint, go_real * values, go_integer number)
{
  /* nothing to do */
  return GO_RESULT_OK;
}
