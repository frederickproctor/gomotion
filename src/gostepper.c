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
  \file gostepper.c

  \brief Starts up the stepper motor driver task.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <stddef.h>		/* NULL, sizeof */
#include <rtapi.h>
#include <rtapi_app.h>
#include "gotypes.h"
#include "gostepper.h"

RTAPI_DECL_INT(DEBUG, 0);
RTAPI_DECL_INT(GO_STEPPER_SHM_KEY, GO_STEPPER_DEFAULT_SHM_KEY);
RTAPI_DECL_INT(GO_STEPPER_TYPE, GO_STEPPER_DIRSTEP);

static void * stepper_task;
#define STEPPER_STACKSIZE 2048

static void * gss_shm;
static go_stepper_struct * gss_ptr;

/* define if you want binary string outputs */
#undef PRINT_STR

#ifdef PRINT_STR
static char * btostr(char b, char * str)
{
  str[0] = b & 0x80 ? '1' : '0';
  str[1] = b & 0x40 ? '1' : '0';
  str[2] = b & 0x20 ? '1' : '0';
  str[3] = b & 0x10 ? '1' : '0';
  str[4] = b & 0x08 ? '1' : '0';
  str[5] = b & 0x04 ? '1' : '0';
  str[6] = b & 0x02 ? '1' : '0';
  str[7] = b & 0x01 ? '1' : '0';
  str[8] = 0;

  return str;
}
#endif

/*!
  The stepper_loop works by taking the desired frequency of the square
  wave, the \a go_stepper_struct \a freq in outputs per second, and
  this task's frequency in task cycles per second, and computing the
  number of task cycles needed for the up- and down portions of the
  full output period.

  h = task frequency [task cycles / sec, Hz]
  f = output frequency [outputs / sec, Hz]
  h/f = task cycles / output

  h/f will be evenly split between the up- and down portions of the
  output.

  e.g. if h = 20,000 Hz (50 usec period), and f = 1,000 Hz,
  h/f = 20, so we have 10 up portions and 10 down portions.

  Maximum frequency is for 1 up and 1 down, or half the task frequency,
  in this case 10,000 Hz.
*/

static void stepdir_loop(void * arg)
{
  rtapi_integer step_bit;
  rtapi_integer dir_bit;
  rtapi_integer nsecs_per_task_cycle;
  rtapi_integer task_cycles_per_sec;
  rtapi_integer joint = 0;
  rtapi_integer up_count[GO_STEPPER_NUM];
  rtapi_integer down_count[GO_STEPPER_NUM];
  rtapi_integer max_count;	/* 'h/f' per description above */
  rtapi_integer max_up_count, max_down_count;
  rtapi_flag old_dir[GO_STEPPER_NUM];
  rtapi_flag dir;
  char loByte = 0;
  char hiByte = 0;
  char oldLoByte = ! loByte;	/* this forces an initial output */
  char oldHiByte = ! hiByte;
#ifdef PRINT_STR
  char loStr[9], hiStr[9];
#endif
 
  if (arg == (void *) 0) {
    /* for dir-step */
    dir_bit = 1;
    step_bit = 2;
  } else {
    /* for step-dir */
    step_bit = 1;
    dir_bit = 2;
  }

  /* set our parameters to defaults, to disable control and await sender */
  gss_ptr->lo_port = 0, gss_ptr->hi_port = 0;
  for (joint = 0; joint < GO_STEPPER_NUM; joint++) {
    gss_ptr->freq[joint] = 0;
    gss_ptr->min_up_count[joint] = 1;
    gss_ptr->min_down_count[joint] = 1;
    gss_ptr->count_on_up[joint] = 1;
    up_count[joint] = 0;
    down_count[joint] = 0;
    old_dir[joint] = 0;
  }

  nsecs_per_task_cycle = rtapi_clock_period;
  /* task_cycles_per_sec is 'h' per description above */
  task_cycles_per_sec = 1000000000 / nsecs_per_task_cycle;
  /* gss_ptr->freq[] is 'f' per description above */

  rtapi_print("gostepper: starting gostepper loop\n");

  while (1) {
    for (joint = 0; joint < GO_STEPPER_NUM; joint++) {
      /* compute maximum counts for full period, 'h/f' */
      if (gss_ptr->freq[joint] > 0) {
	dir = 1;
	max_count = task_cycles_per_sec / (+gss_ptr->freq[joint]);
      } else if (gss_ptr->freq[joint] < 0) {
	dir = 0;
	max_count = task_cycles_per_sec / (-gss_ptr->freq[joint]);
      } else {
	/* not moving this joint */
	continue;		/* next for (joint) */
      }

      /* check for direction reversal, and delay other outputs */
      if (dir != old_dir[joint]) {
	old_dir[joint] = dir;
      } else {
	/* split max_count into equal up, down portions */
	max_up_count = max_count >> 1; /* half */
	max_down_count = max_count - max_up_count;
	/* clamp them to be at or above their minimums */
	if (max_up_count < gss_ptr->min_up_count[joint]) {
	  max_up_count = gss_ptr->min_up_count[joint];
	}
	if (max_down_count < gss_ptr->min_down_count[joint]) {
	  max_down_count = gss_ptr->min_down_count[joint];
	}

	/* we could have sped up, lowering our max count, so to be
	   responsive we should cut down our counts to be less than max */
	if (up_count[joint] > max_up_count) {
	  up_count[joint] = max_up_count;
	}
	if (down_count[joint] > max_down_count) {
	  down_count[joint] = max_down_count;
	}

	if (up_count[joint] > 0) {
	  /* we're in the up-count part */
	  up_count[joint]--;
	  if (up_count[joint] <= 0) {
	    /* finished, so clear output, accumulate position and set down count */
	    if (joint < 4) {
	      loByte &= ~(step_bit << (joint << 1));
	    } else {
	      hiByte &= ~(step_bit << ((joint - 4) << 1));
	    }
	    if (! gss_ptr->count_on_up[joint]) {
	      if (dir) {
		gss_ptr->count[joint]++;
	      } else {
		gss_ptr->count[joint]--;
	      }
	    }
	    down_count[joint] = max_down_count;
	  }
	} else {
	  /* we're in the down-count part */
	  down_count[joint]--;
	  if (down_count[joint] <= 0) {
	    /* finished, so set output, accumulate position and set up count */
	    if (joint < 4) {
	      loByte |= (step_bit << (joint << 1));
	    } else {
	      hiByte |= (step_bit << ((joint - 4) << 1));
	    }
	    if (gss_ptr->count_on_up[joint]) {
	      if (dir) {
		gss_ptr->count[joint]++;
	      } else {
		gss_ptr->count[joint]--;
	      }
	    }
	    up_count[joint] = max_up_count;
	  }
	}
      }

      /* now set direction bit */
      if (dir) {
	if (joint < 4) {
	  loByte |= (dir_bit << (joint << 1));
	} else {
	  hiByte |= (dir_bit << ((joint - 4) << 1));
	}
      } else {
	if (joint < 4) {
	  loByte &= ~(dir_bit << (joint << 1));
	} else {
	  hiByte &= ~(dir_bit << ((joint - 4) << 1));
	}
      }
    } /* end of for (joint) loop */

#ifdef PRINT_STR
    if (DEBUG) {
      if (oldLoByte != loByte ||
	  oldHiByte != hiByte) {
	rtapi_print("gostepper: %s %s\n", btostr(loByte, loStr), btostr(hiByte, hiStr));
      }
    }
#endif

    /* write the output */
    if (oldLoByte != loByte) {
      oldLoByte = loByte;
      if (gss_ptr->lo_port != 0) rtapi_outb(loByte, gss_ptr->lo_port);
    }
    if (oldHiByte != hiByte) {
      oldHiByte = hiByte;
      /* un-invert bits 0, 1 and 3 */
      if (gss_ptr->hi_port != 0) rtapi_outb(hiByte ^ 0x0B, gss_ptr->hi_port);
    }

    gss_ptr->heartbeat++;

    rtapi_wait(nsecs_per_task_cycle);
  } /* while (1) */
}

/*!
  The \a graycode_loop handles either two-bit or four-bit Gray code
  stepping. Two-bit Gray codes cycle through a two-bit pattern that
  changes by only one bit at a time, a technique also known as
  quadrature and used by incremental encoders for position
  feedback. This eliminates the race condition exhibited by
  step-direction coding, in which two bits can change at a time, and
  the direction bit may not register before the step bit.

  Four-bit Gray codes work the same way. Although only two bits are
  needed to represent a step in the positive or negative direction,
  four-bit coding can be directly mapped to the four wires on stepper
  motors to avoid some electronics.

  Three-, five- and and more-bit Gray codes are possible but not
  implemented here since no real systems using these are known.
*/

static void graycode_loop(void * arg)
{
  rtapi_integer bits_per_tuple;
  rtapi_integer tuples;		/* how many pairs or quads we support */
  rtapi_integer tuples_per_byte;
  rtapi_integer nsecs_per_task_cycle;
  rtapi_integer task_cycles_per_sec;
  rtapi_integer joint = 0;
  rtapi_integer max_count; /* 'h/f' per description above */
  rtapi_integer count[GO_STEPPER_NUM]; /* this decrements each cycle */
  rtapi_integer index[GO_STEPPER_NUM];
  rtapi_integer max_index;
  /* set code array to hold enough for 4-bit Gray code, although only
     the first four will be used if we're doing 2-bit Gray code */
  rtapi_integer code[] = {0, 1, 3, 2, 6, 7, 5, 4,
			  0xC, 0xD, 0xF, 0xE, 0xA, 0xB, 9, 8};
  rtapi_flag dir;
  char loByte = 0;
  char hiByte = 0;
  char oldLoByte = ! loByte;	/* this forces an initial output */
  char oldHiByte = ! hiByte;
#ifdef PRINT_STR
  char loStr[9], hiStr[9];
#endif

  if (arg == (void *) 0) {
    /* two-bit Gray code */
    bits_per_tuple = 2;
  } else {
    /* four-bit Gray code */
    bits_per_tuple = 4;
  }
  tuples = 12 / bits_per_tuple;
  tuples_per_byte = 8 / bits_per_tuple;
  max_index = (1 << bits_per_tuple) - 1;

  /* set our parameters to defaults, to disable control and await sender */
  gss_ptr->lo_port = 0, gss_ptr->hi_port = 0;
  for (joint = 0; joint < GO_STEPPER_NUM; joint++) {
    gss_ptr->freq[joint] = 0;
    gss_ptr->min_up_count[joint] = 1;
    count[joint] = 0;
    index[joint] = 0;
  }

  nsecs_per_task_cycle = rtapi_clock_period;
  /* task_cycles_per_sec is 'h' per description above */
  task_cycles_per_sec = 1000000000 / nsecs_per_task_cycle;
  /* gss_ptr->freq[] is 'f' per description above */

  while (1) {
    loByte = 0, hiByte = 0;
    /* we count backward since we shift output bytes up each time,
       so this leaves joint 0 near the low-order bits */
    for (joint = tuples - 1; joint >= 0; joint--) {
      if (gss_ptr->freq[joint] != 0) {
	/* compute maximum counts for full period, 'h/f' */
	if (gss_ptr->freq[joint] > 0) {
	  dir = 1;
	  max_count = task_cycles_per_sec / (+gss_ptr->freq[joint]);
	} else {
	  dir = 0;
	  max_count = task_cycles_per_sec / (-gss_ptr->freq[joint]);
	}

	/* clamp the count to be at or above the minimum */
	if (max_count < gss_ptr->min_up_count[joint]) {
	  max_count = gss_ptr->min_up_count[joint];
	}

	/* we could have sped up, lowering our max count, so to be
	   responsive we should cut down our counts to be less than max */
	if (count[joint] > max_count) {
	  count[joint] = max_count;
	}

	count[joint]--;
	if (count[joint] <= 0) {
	  /* move to next code */
	  if (dir) {
	    if (index[joint] == max_index) index[joint] = 0;
	    else index[joint]++;
	    gss_ptr->count[joint]++;
	  } else {
	    if (index[joint] == 0) index[joint] = max_index;
	    else index[joint]--;
	    gss_ptr->count[joint]--;
	  }
	  count[joint] = max_count;
	}
      }	/* else we're not moving, so leave index as-is */

      /* shift the byte down and add this code */
      if (joint < tuples_per_byte) {
	loByte <<= bits_per_tuple;
	loByte += code[index[joint]];
      } else {
	hiByte <<= bits_per_tuple;
	hiByte += code[index[joint]];
      }
    } /* end of for (joint) loop */

#ifdef PRINT_STR
    if (DEBUG) {
      if (oldLoByte != loByte ||
	  oldHiByte != hiByte) {
	rtapi_print("gostepper: %s %s\n", btostr(loByte, loStr), btostr(hiByte, hiStr));
      }
    }
#endif

    /* write the output */
    if (oldLoByte != loByte) {
      oldLoByte = loByte;
      if (gss_ptr->lo_port != 0) rtapi_outb(loByte, gss_ptr->lo_port);
    }
    if (oldHiByte != hiByte) {
      oldHiByte = hiByte;
      /* un-invert bits 0, 1 and 3 */
      if (gss_ptr->hi_port != 0) rtapi_outb(hiByte ^ 0x0B, gss_ptr->hi_port);
    }

    gss_ptr->heartbeat++;

    rtapi_wait(nsecs_per_task_cycle);
  } /* while (1) */
}

void rtapi_app_exit(void)
{
  int count = gss_ptr->heartbeat;
  
  if (NULL != stepper_task) {
    if (DEBUG) rtapi_print("gostepper: %d unused stepper stack bytes\n",
		rtapi_task_stack_check(stepper_task));
    (void) rtapi_task_stop(stepper_task);
    (void) rtapi_task_delete(stepper_task);
    stepper_task = 0;
  }

  if (NULL != gss_shm) {
    rtapi_rtm_delete(gss_shm);
    gss_shm = NULL;
  }

  if (DEBUG) rtapi_print("gostepper: gostepper done, count = %d\n", count);
  return;
}

int rtapi_app_main(RTAPI_APP_ARGS_DECL)
{
  void (*stepper_loop)(void * arg);
  void * stepper_arg;
  int stepper_prio;
  rtapi_integer nsecs_per_period;
  rtapi_integer nsecs_per_task_cycle;

  if (0 != rtapi_app_init(RTAPI_APP_ARGS)) {
    rtapi_print("gostepper: can't init rtapi\n");
    return -1;
  }

  rtapi_app_atexit(rtapi_app_exit);

  /* get command line args */
  (void) rtapi_arg_get_int(&DEBUG, "DEBUG");
  if (DEBUG) rtapi_print("gostepper: using DEBUG = %d\n", DEBUG);
  (void) rtapi_arg_get_int(&GO_STEPPER_SHM_KEY, "GO_STEPPER_SHM_KEY");
  if (DEBUG) rtapi_print("gostepper: using GO_STEPPER_SHM_KEY = %d\n", GO_STEPPER_SHM_KEY);
  (void) rtapi_arg_get_int(&GO_STEPPER_TYPE, "GO_STEPPER_TYPE");
  if (DEBUG) rtapi_print("gostepper: using GO_STEPPER_TYPE = %d\n", GO_STEPPER_TYPE);
  (void) rtapi_arg_get_int(&nsecs_per_period, "NSECS_PER_PERIOD");
  if (0 < nsecs_per_period) rtapi_clock_set_period(nsecs_per_period);
  if (DEBUG) rtapi_print("gostepper: using period = %d\n", rtapi_clock_period);

  /* allocate the shared memory buffer */
  gss_shm = rtapi_rtm_new(GO_STEPPER_SHM_KEY, sizeof(go_stepper_struct));
  if (NULL == gss_shm) {
    rtapi_print("gostepper: can't get stepper shm\n");
    return -1;
  }

  gss_ptr = rtapi_rtm_addr(gss_shm);
  gss_ptr->heartbeat = 0;

  /* set prio to be highest */
  stepper_prio = rtapi_prio_highest();

  /* set the period to be the global base period, as fast as possible */
  nsecs_per_task_cycle = rtapi_clock_period;

  /* select which stepper type to run */
  switch (GO_STEPPER_TYPE) {
  case GO_STEPPER_DIRSTEP:
    stepper_loop = stepdir_loop;
    stepper_arg = (void *) 0;
    break;
  case GO_STEPPER_STEPDIR:
    stepper_loop = stepdir_loop;
    stepper_arg = (void *) 1;
    break;
  case GO_STEPPER_GRAYCODE_2BIT:
    stepper_loop = graycode_loop;
    stepper_arg = (void *) 0;
    break;
  case GO_STEPPER_GRAYCODE_4BIT:
    stepper_loop = graycode_loop;
    stepper_arg = (void *) 1;
    break;
  default:
    stepper_loop = stepdir_loop;
    stepper_arg = (void *) 0;
    break;
  }

  /* launch the stepper task */
  stepper_task = rtapi_task_new();
  if (NULL == stepper_task) {
    rtapi_print("gostepper: can't allocate stepper task\n");
    return -1;
  }
  if (0 != rtapi_task_start(stepper_task,
			    stepper_loop,
			    stepper_arg,
			    stepper_prio, 
			    STEPPER_STACKSIZE,
			    nsecs_per_task_cycle,
			    0)) { /* no floating point */
    rtapi_print("gostepper: can't start stepper task with period %d\n", nsecs_per_task_cycle);
    return -1;
  }

  rtapi_print("gostepper: gostepper started with period %d nsec\n", nsecs_per_task_cycle);

  return rtapi_app_wait();
}
