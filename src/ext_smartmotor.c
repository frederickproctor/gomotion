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
  \file ext_smartmotor.c

  \brief External interface to a Smart Motor-based system.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>		/* snprintf */
#include <string.h>		/* strlen */
#include <stddef.h>		/* NULL */
#include <stdlib.h>		/* atoi */
#include <rtapi.h>		/* rtapi_string_to_integer */
#include "gotypes.h"
#include "extintf.h"
#include "servointf.h"		/* SERVO_NUM */

#define SEPARATE_RP_QUERY 1

#define DEFAULT_VEL 200000
#define DEFAULT_ACC  20000
#define MIN_VEL      20000

typedef struct {
  void *task;
  void *mutex;
  void *serial_id;
  go_real position;
  go_real old_position;
  go_real scale_vel;
  rtapi_integer period_nsec;
  go_real inverse_cycle_time;
  go_flag valid;
} smartmotor_struct;

static smartmotor_struct smartmotors[SERVO_NUM];

void taskcode(void *args)
{
  void *task;
  void *mutex;
  void *serial_id;
  go_real *posptr;
  go_real *oldptr;
  go_real *scaleptr;
  rtapi_integer *perptr;
  go_flag *validptr;
  enum {BUFFERLEN = 256};
  char buffer[BUFFERLEN];
  rtapi_integer nchars;
  rtapi_integer period_nsec;

#define ARGIT(s) s = ((smartmotor_struct *) args)->s
  ARGIT(task);
  ARGIT(mutex);
  ARGIT(serial_id);
  perptr = &(((smartmotor_struct *) args)->period_nsec);
  posptr = &(((smartmotor_struct *) args)->position);
  oldptr = &(((smartmotor_struct *) args)->old_position);
  scaleptr = &(((smartmotor_struct *) args)->scale_vel);
  validptr = &(((smartmotor_struct *) args)->valid);

  if (NULL != serial_id) {
    rtapi_serial_set_nonblocking(serial_id);
  }

  if (NULL != serial_id) {
    /* send some good values for V and A */
    rtapi_snprintf(buffer, sizeof(buffer) - 1, "V=%d A=%d G\r", DEFAULT_VEL, DEFAULT_ACC);
    buffer[sizeof(buffer) - 1] = 0;
    rtapi_mutex_take(mutex);
    (void) rtapi_serial_write(serial_id, buffer, strlen(buffer));
    rtapi_mutex_give(mutex);
  } else {
    *validptr = 1;
  }

  for (;;) {
#ifdef SEPARATE_RP_QUERY
    if (NULL != serial_id) {
      strcpy(buffer, "RP\r");
      rtapi_mutex_take(mutex);
      (void) rtapi_serial_write(serial_id, buffer, 3);
      rtapi_mutex_give(mutex);
      nchars = rtapi_serial_read(serial_id, buffer, BUFFERLEN);
      if (nchars > 0) {
	rtapi_mutex_take(mutex);
	*posptr = (go_real) atoi(buffer);
	if (0 == *validptr) *oldptr = *posptr;
	*validptr = 1;
	rtapi_mutex_give(mutex);
      }
    }
#else
    *validptr = 1;
#endif
    /*
      This update task should not run so fast as to overwhelm the
      external Smart Motor system with RP messages. It should run
      about as fast as the trajectory loop, and the Smart Motor system
      should be faster than that. If the Smart Motor system is slower
      than the trajectory loop, we have a design problem.
    */
    rtapi_mutex_take(mutex);
    period_nsec = *perptr;	/* ext_joint_init also shares this */
    rtapi_mutex_give(mutex);
    rtapi_wait(period_nsec);
  }
}

/*
  We need a series of serial port names to open.
*/
go_result ext_init(char *init_string)
{
  char port[EXT_INIT_STRING_LENGTH];
  smartmotor_struct *args;
  const char *ptr;
  rtapi_integer servo_num;
  rtapi_result retval;
  rtapi_integer count_nsec;

  if (NULL == init_string) {
    return GO_RESULT_OK;
  }

  for (servo_num = 0, ptr = rtapi_string_skipwhite(init_string);
       servo_num < SERVO_NUM;
       servo_num++, ptr = rtapi_string_skipone(ptr)) {
    /* init all the task args */
    args = &smartmotors[servo_num];
    args->task = NULL;
    args->mutex = NULL;
    args->serial_id = NULL;
    args->position = 12345.0;
    args->old_position = args->position;
    args->period_nsec = 100000000;
    args->inverse_cycle_time = 1.0e9 / ((go_real) args->period_nsec);
    args->valid = 0;

    (void) rtapi_string_copyone(port, ptr);
    if (*port != 0) {
      args->task = rtapi_task_new();
      args->mutex = rtapi_mutex_new(servo_num);
      (void) rtapi_mutex_give(args->mutex);
      /* ports named "-" are stubbed without error */
      if (strcmp(port, "-")) {
	args->serial_id = rtapi_serial_new();
	retval = rtapi_serial_open(port, args->serial_id);
	if (RTAPI_OK != retval) {
	  rtapi_print("ext_smartmotor: can't open %s\n", port);
	  rtapi_serial_delete(args->serial_id);
	  args->serial_id = NULL;
	  /* setting serial_id to NULL flags that this port should be stubbed */
	}
      }
      retval = rtapi_task_start(args->task,
				taskcode,
				args,
				rtapi_prio_highest(),
				1024,
				args->period_nsec,
				1);

      for (count_nsec = 1000000000; ! args->valid && count_nsec > 0; count_nsec -= args->period_nsec) {
	rtapi_wait(args->period_nsec);
      }
      if (! args->valid) {
	rtapi_print("ext_smartmotor: timed out waiting for position on port %s\n", port);
      }
    }
  }
  
  return GO_RESULT_OK;
}
 
go_result ext_quit(void)
{
  rtapi_integer servo_num;

  for (servo_num = 0; servo_num < SERVO_NUM; servo_num++) {
    if (NULL != smartmotors[servo_num].serial_id) 
      rtapi_serial_delete(smartmotors[servo_num].serial_id);
    if (NULL != smartmotors[servo_num].mutex) 
      rtapi_mutex_delete(smartmotors[servo_num].mutex);
    if (NULL != smartmotors[servo_num].task) 
      rtapi_task_delete(smartmotors[servo_num].task);
  }

  return GO_RESULT_OK;
}

go_result ext_joint_init(go_integer joint, go_real cycle_time)
{
  if (joint < 0 || joint >= SERVO_NUM) return GO_RESULT_ERROR;

  rtapi_mutex_take(smartmotors[joint].mutex);
  /* taskcode also shares these */
  smartmotors[joint].period_nsec = (rtapi_integer) (cycle_time * 1.0e9 + 0.5);
  smartmotors[joint].inverse_cycle_time = 1.0 / cycle_time;
  rtapi_mutex_give(smartmotors[joint].mutex);

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

go_result ext_read_pos
(go_integer joint, go_real *pos)
{
  if (joint < 0 || joint >= SERVO_NUM) return GO_RESULT_ERROR;

  rtapi_mutex_take(smartmotors[joint].mutex);
  *pos = smartmotors[joint].position;
  rtapi_mutex_give(smartmotors[joint].mutex);

  return GO_RESULT_OK;
}

go_result ext_write_pos(go_integer joint, go_real pos)
{
  enum {BUFFERLEN = 256};
  char buffer[BUFFERLEN];
  go_real vel;

  if (joint < 0 || joint >= SERVO_NUM) return GO_RESULT_ERROR;

  rtapi_mutex_take(smartmotors[joint].mutex);

  if (smartmotors[joint].valid) {
    vel = (pos - smartmotors[joint].old_position) * smartmotors[joint].inverse_cycle_time;
    if (vel < 0) vel = -vel;
  } else {
    vel = DEFAULT_VEL;
  }

  /*
    we have to keep at least some speed, otherwise the speed may be
    set to zero before the motor gets there and will be prematurely
    stopped
  */
  if (vel < MIN_VEL) vel = MIN_VEL;

  /*
    we need to scale this up by this magic number to minimize lag
  */
  vel *= smartmotors[joint].scale_vel;

#define INCLUDE_SPEED 1
#ifdef INCLUDE_SPEED
  rtapi_snprintf(buffer, sizeof(buffer) - 1, "V=%d P=%d G\r", (int) vel, (int) pos);
  buffer[sizeof(buffer) - 1] = 0;
#else
  rtapi_snprintf(buffer, sizeof(buffer) - 1, "P=%d G\r", (int) pos);
  buffer[sizeof(buffer) - 1] = 0;
#endif

  if (NULL == smartmotors[joint].serial_id) {
    smartmotors[joint].position = pos;
  } else {
    (void) rtapi_serial_write(smartmotors[joint].serial_id, buffer, strlen(buffer));
  }
  smartmotors[joint].old_position = pos;

  rtapi_mutex_give(smartmotors[joint].mutex);

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

go_result ext_joint_home_latch(go_integer joint, go_real *pos)
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

go_result ext_read_ain(go_integer index, go_real *val)
{
  *val = 0.0;

  return GO_RESULT_OK;
}

go_result ext_write_aout(go_integer index, go_real val)
{
  return GO_RESULT_OK;
}

go_result ext_read_din(go_integer index, go_flag *val)
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
  enum {BUFFERLEN = 256};
  char buffer[BUFFERLEN];

  if (joint < 0 || joint >= SERVO_NUM) return GO_RESULT_ERROR;
  if (number < 4) return GO_RESULT_ERROR; /* need at least Scale P I D */

  smartmotors[joint].scale_vel = values[0];

  if (smartmotors[joint].valid) {
    rtapi_mutex_take(smartmotors[joint].mutex);
    rtapi_snprintf(buffer, sizeof(buffer) - 1, "KP=%d KD=%d KI=%d G\r",
		   (int) values[1], (int) values[2], (int) values[3]);
    buffer[sizeof(buffer) - 1] = 0;
    (void) rtapi_serial_write(smartmotors[joint].serial_id, buffer, strlen(buffer));
    rtapi_mutex_give(smartmotors[joint].mutex);
  }

  return GO_RESULT_OK;
}