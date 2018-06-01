#include <stdio.h>
#include <stdlib.h>
#include "ulapi.h"

/*
  Time is in units of servo samples, where 128 samples is 32 ms.

  WHEREWASI - you are sending pos-time pairs in write_code, but still
  getting the 249 32 0 0 0 response indicating too much data (?)
*/

static void read_code(void *serial)
{
  enum {BUFFERLEN = 80};
  char buffer[BUFFERLEN];
  int nchars;
  int t;
  unsigned char ival;

  for (;;) {
    nchars = ulapi_serial_read(serial, buffer, sizeof(buffer)-1);
    if (nchars < 0) break;
    if (nchars == 0) continue;
    for (t = 0; t < nchars; t++) {
      ival = buffer[t];
      printf("%X (%u)\n", ival, ival);
    }
  }

  printf("done\n");

  return;
}

static ulapi_mutex_struct mutex;

enum {PERIOD_NSECS = 32000000};

static void write_code(void *serial)
{
  char position_buffer[] = {250, 0, 0, 0, 0, 13};
  char time_buffer[] = {251, 0, 0, 0, 128, 13}; /* 128 samples = 32 msec */

  ulapi_mutex_take(&mutex);
  ulapi_serial_write(serial, position_buffer, sizeof(position_buffer));
  time_buffer[4] = 0;
  ulapi_serial_write(serial, time_buffer, sizeof(time_buffer));
  ulapi_mutex_give(&mutex);
  ulapi_wait(PERIOD_NSECS);

  ulapi_mutex_take(&mutex);
  ulapi_serial_write(serial, position_buffer, sizeof(position_buffer));
  time_buffer[4] = 128;
  ulapi_serial_write(serial, time_buffer, sizeof(time_buffer));
  ulapi_mutex_give(&mutex);
  ulapi_wait(PERIOD_NSECS);
  
  for (;;) {
    ulapi_mutex_take(&mutex);
    ulapi_serial_write(serial, position_buffer, sizeof(position_buffer));
    ulapi_serial_write(serial, time_buffer, sizeof(time_buffer));
    ulapi_mutex_give(&mutex);
    ulapi_wait(PERIOD_NSECS);
  }

  printf("done\n");

  return;
}

/* syntax: smartmotor_md <port> <baud> */

int main(int argc, char * argv[])
{
#define BUFFERLEN 256
  char buffer[BUFFERLEN];
  char *port;
  int baud;
  void *serial;
  void *read_task;
  void *write_task;

  if (argc < 3) {
    fprintf(stderr, "syntax: smartmotor_md <port> <baud>\n");
    return 1;
  }

  port = argv[1];
  baud = atoi(argv[2]);

  if (ULAPI_OK != ulapi_init()) {
    fprintf(stderr, "ulapi_init error\n");
    return 1;
  }

  ulapi_mutex_init(&mutex, 1);
  ulapi_mutex_give(&mutex);
  
  if (NULL == (serial = ulapi_serial_new())) {
    fprintf(stderr, "Can't get a serial object\n");
    return 1;
  }

  if (ULAPI_OK != ulapi_serial_open(port, serial)) {
    fprintf(stderr, "Can't open port %s\n", port);
    return 1;
  }

  if (ULAPI_OK != ulapi_serial_baud(serial, baud)) {
    fprintf(stderr, "Can't set baud rate to %d\n", baud);
    return 1;
  }

  read_task = ulapi_task_new();
  ulapi_task_start(read_task, read_code, serial, ulapi_prio_lowest(), 0);

  write_task = ulapi_task_new();
  ulapi_task_start(write_task, write_code, serial, ulapi_prio_lowest(), 0);

  while (! feof(stdin)) {
    printf("> ");
    fflush(stdout);
    if (NULL == fgets(buffer, BUFFERLEN, stdin)) break;
    ulapi_mutex_take(&mutex);
    ulapi_serial_write(serial, buffer, strlen(buffer));
    ulapi_mutex_give(&mutex);
  }

  return 0;
}
