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
  \file emu_galil.c

  \brief Emulates a Galil motion controller via a socket interface.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <stdio.h>		/* sprintf */
#include <stddef.h>		/* NULL, sizeof */
#include <string.h>		/* strcmp */
#include <rtapi.h>
#include <rtapi_app.h>
#include "gotypes.h"

static void * galil_task;
#define GALIL_STACKSIZE 1024

typedef struct {
  rtapi_integer port;
} galil_args;

static void galil_loop(void * args)
{
  rtapi_integer port;
  rtapi_integer socket_id;
  rtapi_integer client_id;
  rtapi_integer nchars;
  enum {BUFFERLEN = 256};
  char buffer[BUFFERLEN];
  go_integer position = 0;

#define ARGIT(s) s = ((galil_args *) args)->s
  ARGIT(port);

  rtapi_print("galil_loop: using port %d\n", (int) port);

  socket_id = rtapi_socket_server(port);
  if (socket_id < 0) {
    rtapi_print("galil_loop: can't serve port %d\n", (int) port);
    (void) rtapi_task_exit();
    return;
  }

  for (;;) {
    rtapi_print("galil_loop: waiting for client connection...\n");
    client_id = rtapi_socket_get_client(socket_id);
    if (client_id < 0) {
      rtapi_print("galil_loop: can't get client connection on %d\n", (int) socket_id);
      break;
    }
    rtapi_print("galil_loop: got client connection on %d\n", (int) client_id);

    for (;;) {
      nchars = rtapi_socket_read(client_id, buffer, sizeof(buffer));
      if (nchars <= 0) break;
      rtapi_print("%s\n", buffer);	/* FIXME-- testing */
      if (! strncmp(buffer, "TP", 2)) {
	sprintf(buffer, "%d\n", (int) position);
	rtapi_socket_write(client_id, buffer, strlen(buffer) + 1);
      } else if (! strncmp(buffer, "PA", 2)) {
	position = atoi(&buffer[2]);
      }
      /*
	Here we add on some delay typical of a real Galil system, so
	that we are prepared to handle it. We'll estimate 10 msec.
       */
      rtapi_wait(10000000);
    }
    rtapi_socket_close(client_id);
  }
}

RTAPI_DECL_INT(DEBUG, 0);
RTAPI_DECL_INT(GALIL_SOCKET_PORT, 17101);

int rtapi_app_main(RTAPI_APP_ARGS_DECL)
{
  galil_args galil_args;
  rtapi_integer galil_prio;

  if (0 != rtapi_app_init(RTAPI_APP_ARGS)) {
    rtapi_print("can't init rtapi\n");
    return -1;
  }

  /* get command line args */
  (void) rtapi_arg_get_int(&DEBUG, "DEBUG");
  if (DEBUG) rtapi_print("using DEBUG = %d\n", DEBUG);
  (void) rtapi_arg_get_int(&GALIL_SOCKET_PORT, "GALIL_SOCKET_PORT");
  if (DEBUG) rtapi_print("using GALIL_SOCKET_PORT = %d\n", GALIL_SOCKET_PORT);

  /* set prio to be highest */
  galil_prio = rtapi_prio_highest();

  /* set the task args */
  galil_args.port = (rtapi_integer) GALIL_SOCKET_PORT;

  /* launch the stepper task */
  galil_task = rtapi_task_new();
  if (NULL == galil_task) {
    rtapi_print("can't allocate galil task\n");
    return -1;
  }
  if (0 != rtapi_task_start(galil_task,
			    galil_loop,
			    &galil_args,
			    galil_prio, 
			    GALIL_STACKSIZE,
			    1,
			    1)) {
    rtapi_print("can't start galil task\n");
    return -1;
  }

  if (DEBUG) rtapi_print("galil task started\n");

  return rtapi_app_wait();
}

void rtapi_app_exit(void)
{
  if (NULL != galil_task) {
    if (DEBUG) rtapi_print("%d unused galil stack bytes\n",
		rtapi_task_stack_check(galil_task));
    (void) rtapi_task_stop(galil_task);
    (void) rtapi_task_delete(galil_task);
    galil_task = 0;
  }

  if (DEBUG) rtapi_print("galil done\n");
  return;
}
