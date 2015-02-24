/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

/*
  igpsclient.c

  Standlone process that opens a socket connection as a client
  to an indoor GPS tracking system, reads the actual position
  and updates the Xinv transform.
*/

#include <stddef.h>		/* NULL, sizeof */
#include <stdio.h>		/* fprintf, stderr, FILE, fopen */
#include <stdlib.h>		/* malloc, atoi */
#include <ctype.h>		/* isgraph */
#include <string.h>		/* strlen, strcpy */
#include <signal.h>		/* SIGINT, signal */
#include <stdarg.h>		/* va_list */
#include <inifile.h>
#include <ulapi.h>		/* ulapi_time */
#include "go.h"			/* go_init, etc */
#include "trajintf.h"		/* traj_comm_struct, traj_ref_struct */

#define CONNECT_WAIT_TIME 3.0
#define DEFAULT_PORT 10001
#define DEFAULT_HOST "localhost"
enum {READ_MUTEX_KEY = 1071, QUEUE_MUTEX_KEY};

/*            |  m.x.x   m.y.x   m.z.x  | */
/* go_mat m = |  m.x.y   m.y.y   m.z.y  | */
/*            |  m.x.z   m.y.z   m.z.z  | */

static int
get_igps(ulapi_integer socket_id, int timestamped, go_pose * pose, go_real * timestamp)
{
  go_mat mat;
  int nchars, size;
  double x[17];			/* 16 + 1 for a timestamp */

  if (timestamped) {
    size = 17 * sizeof(x[0]);
  } else {
    size = 16 * sizeof(x[0]);
  }

  nchars = ulapi_socket_read(socket_id, (char *) &x[0], size);
  if (nchars != size) {
    /* if nonblocking is set, nchars will normally be 0, and
       we'll return -1 as a so-called error */
    return -1;
  }

  pose->tran.x = (go_real) x[3];
  pose->tran.y = (go_real) x[7];
  pose->tran.z = (go_real) x[11];

  mat.x.x = (go_real) x[0];
  mat.x.y = (go_real) x[4];
  mat.x.z = (go_real) x[8];

  mat.y.x = (go_real) x[1];
  mat.y.y = (go_real) x[5];
  mat.y.z = (go_real) x[9];

  mat.z.x = (go_real) x[2];
  mat.z.y = (go_real) x[6];
  mat.z.z = (go_real) x[10];

#if 0
  printf("%f %f %f\n", mat.x.x, mat.y.x, mat.z.x);
  printf("%f %f %f\n", mat.x.y, mat.y.y, mat.z.y);
  printf("%f %f %f\n", mat.x.z, mat.y.z, mat.z.z);
#endif

  if (GO_RESULT_OK != go_mat_quat_convert(&mat, &pose->rot)) {
    return -1;
  }

  if (timestamped) {
    *timestamp = (go_real) x[16];
  } else {
    *timestamp = 0.0;
  }

  return 0;
}

typedef struct {
  go_pose N;
  go_pose Xinv;
  go_real timestamp;
} posetime_queue_type;

typedef struct _posetime_queue_entry {
  posetime_queue_type val;
  struct _posetime_queue_entry * next;
} posetime_queue_entry;

typedef struct {
  posetime_queue_entry * head;
  posetime_queue_entry * tail;
  unsigned int howmany;
} posetime_queue_struct;

static int
posetime_queue_put(posetime_queue_type val, posetime_queue_struct * queue)
{
  posetime_queue_entry * entry;

  entry = malloc(sizeof(*entry));

  entry->val = val;
  entry->next = NULL;
  if (queue->head == NULL) {
    queue->head = entry;
  } else {
    queue->tail->next = entry;
  }
  queue->tail = entry;
  queue->howmany++;

  return 0;
}

static int
posetime_queue_get(posetime_queue_type * val, posetime_queue_struct * queue)
{
  posetime_queue_entry * next;

  if (NULL == queue->head) {
    return -1;
  }

  next = queue->head->next;
  *val = queue->head->val;
  free(queue->head);
  queue->head = next;
  queue->howmany--;

  return 0;
}

static int
posetime_queue_peek(posetime_queue_type * val, posetime_queue_struct * queue)
{
  if (NULL == queue->head) {
    return -1;
  }

  *val = queue->head->val;

  return 0;
}

static int
posetime_queue_howmany(posetime_queue_struct * queue)
{
  return queue->howmany;
}

static int
posetime_queue_init(posetime_queue_struct * queue)
{
  queue->head = NULL;
  queue->tail = NULL;
  queue->howmany = 0;

  return 0;
}

typedef struct {
  ulapi_real period;
  ulapi_integer socket_id;
  go_pose * Ai_ptr;
  ulapi_real * timestamp_ptr;
  void * mutex;
  ulapi_flag timestamped;
} server_read_args;

static void
server_read_code(void * args)
{
  ulapi_real period;
  ulapi_integer socket_id;
  go_pose * Ai_ptr;
  ulapi_real * timestamp_ptr;
  void * mutex;
  ulapi_flag timestamped;
  go_pose Ai;
  go_real timestamp;
  int retval;

  period = ((server_read_args *) args)->period;
  socket_id = ((server_read_args *) args)->socket_id;
  Ai_ptr = ((server_read_args *) args)->Ai_ptr;
  mutex = ((server_read_args *) args)->mutex;
  timestamp_ptr = ((server_read_args *) args)->timestamp_ptr;
  timestamped = ((server_read_args *) args)->timestamped;

  for (;;) {
    retval = get_igps(socket_id, timestamped, &Ai, &timestamp);
    if (0 == retval) {
      ulapi_mutex_take(mutex);
      *Ai_ptr = Ai;
      *timestamp_ptr = timestamp;
      ulapi_mutex_give(mutex);
    } else {
      fprintf(stderr, "can't get IGPS measurement\n");
    }

    if (period > 0.0) ulapi_sleep(period);
  }

  return;
}

typedef struct {
  ulapi_real period;
  traj_stat_struct * traj_stat_ptr;
  posetime_queue_struct * queue_ptr;
  void * mutex;
} controller_read_args;

/*
  The controller read thread reads Ni from the controller and queues it
*/
static void
controller_read_code(void * args)
{
  ulapi_real period;
  traj_stat_struct * traj_stat_ptr;
  posetime_queue_struct * queue_ptr;
  void * mutex;
  traj_stat_struct traj_stat;
  posetime_queue_type posetime;

  period = ((controller_read_args *) args)->period;
  traj_stat_ptr = ((controller_read_args *) args)->traj_stat_ptr;
  queue_ptr = ((controller_read_args *) args)->queue_ptr;
  mutex = ((controller_read_args *) args)->mutex;

  for (;;) {
    traj_stat = *traj_stat_ptr;

    if (traj_stat.head == traj_stat.tail && 
	traj_stat.type == TRAJ_STAT_TYPE &&
	traj_stat.homed) {
      posetime.N = traj_stat.ecp;
      posetime.Xinv = traj_stat.xinv;
      posetime.timestamp = ulapi_time();
      ulapi_mutex_take(mutex);
      posetime_queue_put(posetime, queue_ptr);
      ulapi_mutex_give(mutex);
    }

    if (period > 0.0) ulapi_sleep(period);
  }

  return;
}

static int
ini_load(char * inifile,
	 ulapi_real * length_units_per_m,
	 ulapi_real * angle_units_per_rad,
	 ulapi_id * traj_shm_key)
{
  FILE * fp;
  const char * key;
  const char * section;
  const char * inistring;
  double d1;
  int i1;

#undef CLOSE_AND_RETURN
#define CLOSE_AND_RETURN(ret)		       	\
    if (NULL != fp) fclose(fp); 		\
    return (ret)

  if (NULL == (fp = fopen(inifile, "r"))) {
    fprintf(stderr, "can't open %s\n", inifile);
    CLOSE_AND_RETURN(1);
  }

  section = "GOMOTION";
  key = "LENGTH_UNITS_PER_M";

  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    *length_units_per_m = 1.0;
    fprintf(stderr, "missing entry: [%s] %s, using default %f\n", section, key, *length_units_per_m);
  } else if (1 != sscanf(inistring, "%lf", &d1)) {
    fprintf(stderr, "bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN(-1);
  } else if (d1 <= 0.0) {
    fprintf(stderr, "invalid entry: [%s] %s = %s must be positive\n", section, key, inistring);
    CLOSE_AND_RETURN(-1);
  } else {
    *length_units_per_m = (go_real) d1;
  }

  section = "GOMOTION";
  key = "ANGLE_UNITS_PER_RAD";

  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    *angle_units_per_rad = 1.0;
    fprintf(stderr, "missing entry: [%s] %s, using default %f\n", section, key, *angle_units_per_rad);
  } else if (1 != sscanf(inistring, "%lf", &d1)) {
    fprintf(stderr, "bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN(-1);
  } else if (d1 <= 0.0) {
    fprintf(stderr, "invalid entry: [%s] %s = %s must be positive\n", section, key, inistring);
    CLOSE_AND_RETURN(-1);
  } else {
    *angle_units_per_rad = (go_real) d1;
  }

  section = "TRAJ";
  key = "SHM_KEY";

  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    fprintf(stderr, "missing entry: [%s] %s\n", section, key);
    CLOSE_AND_RETURN(-1);
  } else if (1 != sscanf(inistring, "%i", &i1)) {
    fprintf(stderr, "bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN(-1);
  }
  *traj_shm_key = (ulapi_id) i1;

  CLOSE_AND_RETURN(0);
}

static int debug_mask = 0;
static void
print_debug(int mask, const char * fmt, ...)
{
  va_list ap;

  if (mask == (mask & debug_mask)) {
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
  }
}

static int done = 0;
static void
quit(int sig)
{
  done = 1;
}

static char *
mybasename(char * s)
{
  char * end;

  end = s + strlen(s);

  while (--end >= s) {
    if ('/' == *end) break;
  }

  return *(end+1) == 0 ? end : end+1;
}

/*
  Usage: igpsclient 
  -i <ini file>
  -u unix | rtai

  -r <period> # for reading Ai from server
  -w <period> # for writing Xinv to controller
  -q <period> # for reading and queuing Ni from controller

  -p <TCP port>
  -h <server host address>

  -n # do nonblocking
  -s # suppress writing Xinv, just print it
  -z # resets Xinv to zero
  -a # expect an appended time stamp

  -d <debug mask>, where
  1 prints Xinv
  2 prints matched timestamps
*/

int main(int argc, char *argv[])
{
#define BN mybasename(argv[0])
  enum { BUFFERLEN = 80 };
  posetime_queue_struct queue;
  posetime_queue_type posetime;
  int option;
  char inifile_name[BUFFERLEN] = "gomotion.ini";
  int port = DEFAULT_PORT;
  char host[BUFFERLEN] = DEFAULT_HOST;
  ulapi_real length_units_per_m;
  ulapi_real angle_units_per_rad;
  ulapi_integer socket_id;
  ulapi_real read_period;
  ulapi_real write_period;
  ulapi_real queue_period;
  int reset;
  int nonblocking;
  int suppress;
  int timestamped;
  ulapi_id traj_shm_key;
  void * traj_shm;
  traj_comm_struct * traj_comm_ptr;
  traj_stat_struct traj_stat;
  traj_ref_struct * traj_ref_ptr;

  void * server_read_task;
  void * controller_read_task;
  void * read_mutex;
  void * queue_mutex;
  server_read_args * server_read_args_ptr;
  controller_read_args * controller_read_args_ptr;

  go_pose Ai_sh, Ai, Ainvi, Xinvi;
  go_real mag;
  ulapi_real timestamp_sh, timestamp;
  ulapi_real end;
  int start_it;
  int got_it;
  int heartbeat;
  int retval;

  read_period = 1.0;
  write_period = 1.0;
  queue_period = 1.0;
  reset = 0;
  nonblocking = 0;
  suppress = 0;
  timestamped = 0;
  retval = 0;

  opterr = 0;
  for (;;) {
    option = getopt(argc, argv, ":i:u:r:w:q:p:h:nszad:");
    if (option == -1)
      break;

    switch (option) {
    case 'i':
      strncpy(inifile_name, optarg, BUFFERLEN);
      inifile_name[BUFFERLEN - 1] = 0;
      break;

    case 'r':
      read_period = (ulapi_real) atof(optarg);
      break;

    case 'w':
      write_period = (ulapi_real) atof(optarg);
      break;

    case 'q':
      queue_period = (ulapi_real) atof(optarg);
      break;

    case 'p':
      port = atoi(optarg);
      break;

    case 'h':
      strncpy(host, optarg, sizeof(host));
      host[sizeof(host) - 1] = 0;
      break;

    case 'n':
      nonblocking = 1;
      break;

    case 's':
      suppress = 1;
      break;

    case 'z':
      reset = 1;
      break;

    case 'a':
      timestamped = 1;
      break;

    case 'd':
      if (1 != sscanf(optarg, "%i", &debug_mask)) {
	fprintf (stderr, "bad value for -%c: %s\n", option, optarg);
	return 1;
      }
      break;

    case ':':
      fprintf(stderr, "missing value for -%c\n", optopt);
      return 1;
      break;

    default:			/* '?' */
      fprintf (stderr, "unrecognized option -%c\n", optopt);
      return 1;
      break;
    }
  }
  if (optind < argc) {
    fprintf(stderr, "extra non-option characters: %s\n", argv[optind]);
    return 1;
  }

  if (0 != ulapi_init()) {
    fprintf(stderr, "%s: error: can't init ulapi\n", BN);
    return 1;
  }

  if (0 != go_init()) {
    fprintf(stderr, "%s: error: can't init gomotion\n", BN);
    return 1;
  }
  
  if (0 != ini_load(inifile_name,
		    &length_units_per_m,
		    &angle_units_per_rad,
		    &traj_shm_key)) {
    fprintf(stderr, "%s: error: can't load %s\n", BN, inifile_name);
    return 1;
  }

  /* macros to convert from Go units to ini file units, from-go-length,angle */
#define FGL(x) (double) ((x) * length_units_per_m)
#define FGA(x) (double) ((x) * angle_units_per_rad)

  traj_shm = NULL;
  traj_comm_ptr = NULL;

  /* this gets us out of nested loops quickly and cleanly */
#define QUIT(ret) retval = (ret); goto DONE

  /* get traj shared memory buffers */
  traj_shm = ulapi_rtm_new(traj_shm_key, sizeof(traj_comm_struct));
  if (NULL == traj_shm) {
    fprintf(stderr, "%s: error: can't get shared memory\n", BN);
    QUIT(1);
  }
  traj_comm_ptr = (traj_comm_struct *) ulapi_rtm_addr(traj_shm);

  /* check if traj is alive */
  for (start_it = 0, got_it = 0, end = ulapi_time() + CONNECT_WAIT_TIME;
       ulapi_time() < end;
       ulapi_sleep(queue_period)) {
    traj_stat = traj_comm_ptr->traj_stat;
    if (traj_stat.head == traj_stat.tail && traj_stat.type == TRAJ_STAT_TYPE) {
      if (! start_it) {
	start_it = 1;
	heartbeat = traj_stat.heartbeat;
      }
      if (heartbeat != traj_stat.heartbeat) {
	got_it = 1;
	break;
      }
    }
  }
  if (! got_it) {
    fprintf(stderr, "%s: error: can't connect to traj\n", BN);
    QUIT(1);
  }

  traj_ref_ptr = &traj_comm_ptr->traj_ref;

  if (reset) {
    traj_ref_ptr->head++;
    traj_ref_ptr->xinv = go_pose_identity();
    traj_ref_ptr->tail = traj_ref_ptr->head;
    QUIT(0);
  }

  socket_id = ulapi_socket_get_client_id(port, host);
  if (socket_id < 0) {
    fprintf(stderr, "can't connect to %s:%d\n", host, (int) port);
    go_exit();
    QUIT(1);
  }
  if (nonblocking) {
    ulapi_socket_set_nonblocking(socket_id);
  }

  timestamp_sh = -1.0;		/* flag as invalid */

  /* start the server reading task */
  read_mutex = ulapi_mutex_new(READ_MUTEX_KEY);
  server_read_task = ulapi_task_new();
  server_read_args_ptr = malloc(sizeof(server_read_args));
  server_read_args_ptr->period = read_period;
  server_read_args_ptr->socket_id = socket_id;
  server_read_args_ptr->Ai_ptr = &Ai_sh;
  server_read_args_ptr->timestamp_ptr = &timestamp_sh;
  server_read_args_ptr->mutex = read_mutex;
  server_read_args_ptr->timestamped = timestamped;
  ulapi_task_start(server_read_task, server_read_code, server_read_args_ptr, ulapi_prio_lowest(), 0);

  /* wait until we see a valid timestamp */
  do {
    ulapi_mutex_take(read_mutex);
    timestamp = timestamp_sh;
    ulapi_mutex_give(read_mutex);
    ulapi_sleep(write_period);
  } while (timestamp < 0.0);

  posetime_queue_init(&queue);

  /* start the controller writing task */
  queue_mutex = ulapi_mutex_new(QUEUE_MUTEX_KEY);
  controller_read_task = ulapi_task_new();
  controller_read_args_ptr = malloc(sizeof(controller_read_args));
  controller_read_args_ptr->period = queue_period;
  controller_read_args_ptr->traj_stat_ptr = &traj_comm_ptr->traj_stat;
  controller_read_args_ptr->queue_ptr = &queue;
  controller_read_args_ptr->mutex = queue_mutex;
  ulapi_task_start(controller_read_task, controller_read_code, controller_read_args_ptr, ulapi_prio_lowest(), 0);

  /*
    This loops takes the lastest value of Ai from the server read task,
    looks up the matching Ni from the controller read task, computes
    Xinv and writes to the controller.
  */

  signal(SIGINT, quit);
  done = 0;

  while (! done) {
    ulapi_mutex_take(read_mutex);
    Ai = Ai_sh;
    timestamp = timestamp_sh;
    ulapi_mutex_give(read_mutex);
    go_pose_inv(&Ai, &Ainvi);

    /*
      Read out all the queued Ni values, keeping the one
      whose timestamp is the first that exceeds the Ai measurement
     */

    ulapi_mutex_take(queue_mutex);
    /* read out the queue */
    for (got_it = 0;;) {
      retval = posetime_queue_get(&posetime, &queue);
      if (0 == retval) {
	got_it = 1;   /* we got one, not necessarily the best match */
	if (posetime.timestamp >= timestamp) {
	  /* we matched one, so stop */
	  break;
	} /* else didn't match yet, so keep going */
      } else {
	/* queue is empty, so stop */
	break;
      }
    }
    ulapi_mutex_give(queue_mutex);

    if (got_it) {
      print_debug(2, "matched %f to %f\n", (double) posetime.timestamp, (double) timestamp);
      print_debug(2, "for %f %f %f, %f %f %f\n",
		  FGL(posetime.N.tran.x), FGL(posetime.N.tran.y), FGL(posetime.N.tran.z),
		  FGL(Ai.tran.x), FGL(Ai.tran.y), FGL(Ai.tran.z));

      /* compute Xinv(i) = Ainv(i) N(i-1) Xinv(i-1) */
      go_pose_pose_mult(&posetime.N, &posetime.Xinv, &Xinvi);
      go_pose_pose_mult(&Ainvi, &Xinvi, &Xinvi);
      go_cart_mag(&Xinvi.tran, &mag);

      print_debug(1, "%f %f %f %f %f\n",
		  (double) (posetime.timestamp - timestamp),
		  FGL(Xinvi.tran.x), FGL(Xinvi.tran.y), FGL(Xinvi.tran.z),
		  FGL(mag));

      /* write the actual position into the reference */
      if (! suppress) {
	traj_ref_ptr->head++;
	traj_ref_ptr->xinv = Xinvi;
	traj_ref_ptr->tail = traj_ref_ptr->head;
      }
    } else {
      print_debug(2, "no match for %f\n", (double) timestamp);
    }

    ulapi_sleep(write_period);
  } /* while (! done) */

 DONE:

  if (NULL != traj_shm) {
    ulapi_rtm_delete(traj_shm);
    traj_shm = NULL;
  }
  traj_comm_ptr = NULL;

  ulapi_socket_close(socket_id);

  (void) go_exit();

  fprintf(stderr, "%s done\n", BN);

  return retval || ulapi_exit() == ULAPI_OK ? 0 : -1;
}

