#include <stdio.h>
#include <stdlib.h>
#include <string.h>		/* memcpy */
#include <ulapi.h>

/*
  Time is in units of servo samples, where 128 samples is 32 ms.

  WHEREWASI - the timing looks good now that you are incrementing by
  the actual 260-261 diff, but when you change the position to even
  1 more than zero, it jumps.
*/

static unsigned int running = 0;
static unsigned int slots_available = 0;
static unsigned int sm_clock = 0;
static unsigned int sm_diff = 0;
static int position = 0;
static unsigned int sample = 0;
static unsigned int sample_period = 1;

enum {LEAD_SLOTS = 10};		/* how many slots to keep filled */

static void read_code(void *serial)
{
  enum {BUFFERLEN = 80};
  unsigned char buffer[BUFFERLEN];
  int nchars;
  int t;
  unsigned char ival;
  unsigned int sm_prev;
  
  for (;;) {
    nchars = ulapi_serial_read(serial, buffer, sizeof(buffer)-1);

    if (nchars < 0) break;
    if (nchars == 0) continue;
    if (0xF9 == buffer[0]) {
      /* status message */
      if (0x80 & buffer[1]) {
	running = 1;
	printf("running\n");
	slots_available = 0x7F & buffer[1];
	printf("%d slots available\n", slots_available);
	sm_clock = buffer[2];
	sm_clock = sm_clock << 8;
	sm_clock += buffer[3];
	sm_clock = sm_clock << 8;
	sm_clock += buffer[4];
	sm_diff = sm_clock - sm_prev;
	sm_prev = sm_clock;
	printf("%u sm_clock, %u sample, %d lead, %d diff\n", sm_clock, sample, sample - sm_clock, sm_diff);
      } else {
	running = 0;
	printf("not running\n");
	if (0x40 & buffer[1]) printf("underflow\n");
	if (0x20 & buffer[1]) printf("overflow\n");
	if (0x10 & buffer[1]) printf("internal data space error\n");
	if (0x08 & buffer[1]) printf("invalid position delta\n");
	if (0x04 & buffer[1]) printf("invalid time delta\n");
	if (0x02 & buffer[1]) printf("MD mode running\n");
	if (0x01 & buffer[1]) printf("pending G command\n");
	slots_available = buffer[2];
	printf("%d slots available\n", slots_available);
	sm_clock = buffer[3];
	sm_clock = sm_clock << 8;
	sm_clock += buffer[4];
	sm_diff = sm_clock - sm_prev;
	sm_prev = sm_clock;
	printf("%u sm_clock, %u sample, %d lead, %d diff\n", sm_clock, sample, sample - sm_clock, sm_diff);
      }
    } else {
      for (t = 0; t < nchars; t++) {
	ival = buffer[t];
	printf("%X (%u)\n", ival, ival);
      }
    }
  }

  printf("done\n");

  return;
}

static ulapi_mutex_struct mutex;

/* 256 samples = 64 msec */
enum {DEFAULT_SAMPLE_PERIOD = 256};
enum {PERIOD_NSECS = 64000000};

/* need to reverse the endian-ness */
static void sm_copy(unsigned char *dst, unsigned char *src)
{
  unsigned char *sptr = (unsigned char *) src;
  dst[0] = sptr[3];
  dst[1] = sptr[2];
  dst[2] = sptr[1];
  dst[3] = sptr[0];
  #if 0
  printf("%X %X %X %X\n", (unsigned int) dst[0], (unsigned int) dst[1], (unsigned int) dst[2], (unsigned int) dst[3]);
  #endif
}

static void write_code(void *serial)
{
  enum {COMMAND_LEN = 80};
  char command_buffer[COMMAND_LEN];
  unsigned char position_buffer[] = {250, 0, 0, 0, 0, 13};
  unsigned char time_buffer[] = {251, 0, 0, 0, 0, 13};
  unsigned int t;

  ulapi_mutex_take(&mutex);

  strcpy(command_buffer, "A=100 V=10000 MD\r");
  ulapi_serial_write(serial, command_buffer, strlen(command_buffer));

  position = 0;
  sample = 0;
  sample_period = DEFAULT_SAMPLE_PERIOD;

  for (t = 0; t < LEAD_SLOTS; t++) {
    ulapi_serial_write(serial, "Q\r", 2);
    position += 40;
    sm_copy(&position_buffer[1], &position);
    ulapi_serial_write(serial, position_buffer, sizeof(position_buffer));
    sm_copy(&time_buffer[1], &sample);
    sample += sample_period;
    ulapi_serial_write(serial, time_buffer, sizeof(time_buffer));
    ulapi_wait(PERIOD_NSECS);
  }

  strcpy(command_buffer, "G\r");
  ulapi_serial_write(serial, command_buffer, strlen(command_buffer));
  ulapi_wait(PERIOD_NSECS);

  ulapi_mutex_give(&mutex);

  for (;;) {
    ulapi_mutex_take(&mutex);
    ulapi_serial_write(serial, "Q\r", 2);
    position += 40;
    sm_copy(&position_buffer[1], &position);
    ulapi_serial_write(serial, position_buffer, sizeof(position_buffer));
    if (sm_diff > 0) sample += sm_diff;
    else sample += sample_period;
    sm_copy(&time_buffer[1], &sample);
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
  int i1;

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
    if (1 == ulapi_sscanf(buffer, "%d", &i1)) {
      position = i1;
    }
  }

  return 0;
}
