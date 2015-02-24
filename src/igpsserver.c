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
  igpsserver.c

  Standlone process that serves a socket and dumps position to
  connected clients. The clients read the IPGS data and then write
  the Xinv transform into a separately running Go Motion controller.
*/

#include <stddef.h>		/* NULL, sizeof */
#include <stdio.h>		/* fprintf, stderr, FILE, fopen */
#include <stdlib.h>		/* malloc, atoi */
#include <string.h>		/* strlen, strcpy */
#include <signal.h>		/* signal */
#include <stdarg.h>		/* va_list */
#include <inifile.h>
#include <ulapi.h>		/* ulapi_time */
#include "go.h"			/* go_cart, etc */
#include "servointf.h"		/* SERVO_NUM */
#include "trajintf.h"		/* traj_comm_struct, traj_ref_struct */

#define CONNECT_WAIT_TIME 3.0

#define DEFAULT_PORT 10001

typedef struct {
  go_pose pose;
  go_real timestamp;
} posetime_queue_type;

typedef struct _posetime_queue_entry {
  posetime_queue_type val;
  struct _posetime_queue_entry * next;
} posetime_queue_entry;

typedef struct {
  posetime_queue_entry * head;
  posetime_queue_entry * tail;
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

  return 0;
}

static int
posetime_queue_init(posetime_queue_struct * queue)
{
  queue->head = NULL;
  queue->tail = NULL;

  return 0;
}

static int
posetime_queue_clear(posetime_queue_struct * queue)
{
  posetime_queue_type val;
  int retval;

  do {
    retval = posetime_queue_get(&val, queue);
  } while (retval == 0);

  return 0;
}

/*            |  m.x.x   m.y.x   m.z.x  | */
/* go_mat m = |  m.x.y   m.y.y   m.z.y  | */
/*            |  m.x.z   m.y.z   m.z.z  | */

static int
ini_load(char * inifile,
	 ulapi_real * m_per_length_units,
	 ulapi_real * rad_per_angle_units,
	 ulapi_id * traj_shm_key,
	 int * link_number,
	 go_link * link_params,
	 char * kin_name)
{
  FILE * fp;
  const char * key;
  const char * section;
  const char * inistring;
  char * servo_string;
  int i1;
  double d1, d2, d3, d4, d5, d6;
  int link;
  go_dh dh;
  go_rpy rpy;

#undef CLOSE_AND_RETURN
#define CLOSE_AND_RETURN(ret)		       	\
    if (NULL != fp) fclose(fp); 		\
    return (ret)

  if (NULL == (fp = fopen(inifile, "r"))) {
    fprintf(stderr, "can't open %s\n", inifile);
    CLOSE_AND_RETURN(-1);
  }

  section = "GOMOTION";
  key = "LENGTH_UNITS_PER_M";

  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    *m_per_length_units = 1.0;
    fprintf(stderr, "missing entry: [%s] %s, using default %f\n", section, key, *m_per_length_units);
  } else if (1 != sscanf(inistring, "%lf", &d1)) {
    fprintf(stderr, "bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN(-1);
  } else if (d1 <= 0.0) {
    fprintf(stderr, "invalid entry: [%s] %s = %s must be positive\n", section, key, inistring);
    CLOSE_AND_RETURN(-1);
  } else {
    *m_per_length_units = (go_real) (1.0 / d1);
  }

  section = "GOMOTION";
  key = "ANGLE_UNITS_PER_RAD";

  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    *rad_per_angle_units = 1.0;
    fprintf(stderr, "missing entry: [%s] %s, using default %f\n", section, key, *rad_per_angle_units);
  } else if (1 != sscanf(inistring, "%lf", &d1)) {
    fprintf(stderr, "bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN(-1);
  } else if (d1 <= 0.0) {
    fprintf(stderr, "invalid entry: [%s] %s = %s must be positive\n", section, key, inistring);
    CLOSE_AND_RETURN(-1);
  } else {
    *rad_per_angle_units = (go_real) (1.0 / d1);
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

  section = "TRAJ";
  key = "KINEMATICS";

  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    fprintf(stderr, "missing entry: [%s] %s\n", section, key);
    CLOSE_AND_RETURN(-1);
  }
  strncpy(kin_name, inistring, GO_KIN_NAME_LEN);

  servo_string = (char *) malloc(sizeof("SERVO_" + DIGITS_IN(link)));
  if (NULL == servo_string) {
    fprintf(stderr, "can't allocate space for SERVO_X section\n");
    CLOSE_AND_RETURN(-1);
  }

  for (link = 0; ; link++) {
    sprintf(servo_string, "SERVO_%d", link + 1);
    section = servo_string;

    inistring = ini_find(fp, key = "QUANTITY", section);
    if (NULL == inistring) {
      /* no "QUANTITY" in this section, or no section, so we're done */
      break;
    } else {
      if (ini_match(inistring, "ANGLE")) {
	link_params[link].quantity = GO_QUANTITY_ANGLE;
      } else if (ini_match(inistring, "LENGTH")) {
	link_params[link].quantity = GO_QUANTITY_LENGTH;
      } else {
	fprintf(stderr, "bad entry: [%s] %s = %s\n", section, key, inistring);
	CLOSE_AND_RETURN(-1);
      }
    }

    if (NULL != (inistring = ini_find(fp, key = "DH_PARAMETERS", section))) {
      if (4 == sscanf(inistring, "%lf %lf %lf %lf", &d1, &d2, &d3, &d4)) {
	dh.a = (go_real) (d1 * *m_per_length_units);
	dh.alpha = (go_real) (d2 * *rad_per_angle_units);
	dh.d = (go_real) (d3 * *m_per_length_units);
	dh.theta = (go_real) (d4 * *rad_per_angle_units);
	link_params[link].u.dh = dh;
	link_params[link].type = GO_LINK_DH;
      } else {
	fprintf(stderr, "bad entry: [%s] %s = %s\n", section, key, inistring);
	CLOSE_AND_RETURN(-1);
      }
    } else if (NULL != (inistring = ini_find(fp, key = "PP_PARAMETERS", section))) {
      if (6 == sscanf(inistring, "%lf %lf %lf %lf %lf %lf", &d1, &d2, &d3, &d4, &d5, &d6)) {
	link_params[link].u.pp.pose.tran.x = (go_real) (d1 * *m_per_length_units);
	link_params[link].u.pp.pose.tran.y = (go_real) (d2 * *m_per_length_units);
	link_params[link].u.pp.pose.tran.z = (go_real) (d3 * *m_per_length_units);
	rpy.r = (go_real) (d4 * *rad_per_angle_units);
	rpy.p = (go_real) (d5 * *rad_per_angle_units);
	rpy.y = (go_real) (d6 * *rad_per_angle_units);
	go_rpy_quat_convert(&rpy, &link_params[link].u.pp.pose.rot);
	link_params[link].type = GO_LINK_PP;
      } else {
	fprintf(stderr, "bad entry: [%s] %s = %s\n", section, key, inistring);
	CLOSE_AND_RETURN(-1);
      }
    } else if (NULL != (inistring = ini_find(fp, key = "PK_PARAMETERS", section))) {
      if (6 == sscanf(inistring, "%lf %lf %lf %lf %lf %lf", &d1, &d2, &d3, &d4, &d5, &d6)) {
	link_params[link].u.pk.base.x = (go_real) (d1 * *m_per_length_units);
	link_params[link].u.pk.base.y = (go_real) (d2 * *m_per_length_units);
	link_params[link].u.pk.base.z = (go_real) (d3 * *m_per_length_units);
	link_params[link].u.pk.platform.x = (go_real) (d4 * *m_per_length_units);
	link_params[link].u.pk.platform.y = (go_real) (d5 * *m_per_length_units);
	link_params[link].u.pk.platform.z = (go_real) (d6 * *m_per_length_units);
	link_params[link].type = GO_LINK_PK;
      } else {
	fprintf(stderr, "bad entry: [%s] %s = %s\n", section, key, inistring);
	CLOSE_AND_RETURN(-1);
      }
    } else {
      /* no "DH,PP,PK_PARAMETERS" in this section, so we're done */
      break;
    }
  } /* for (link) */
  *link_number = link;

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
static void quit(int sig)
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
  Usage: igpsserver
  -i <ini file>
  -u unix | rtai
  -t <period> # at which position is sent to client
  -p <TCP port>
  -x "<x y z r p w>" # transform to shift by
  -k # use kinematics instead of transform
  -a # append a time stamp
  -l <latency> # queue up measurements for <latency> seconds
  -n # use nominal instead of actual position

  -d <debug mask>, where
  1 prints actual position
*/

int
main(int argc, char *argv[])
{
#define BN mybasename(argv[0])
  enum { BUFFERLEN = 80 };
  int option;
  char inifile_name[BUFFERLEN] = "gomotion.ini";
  int port = DEFAULT_PORT;
  double d1, d2, d3, d4, d5, d6;
  posetime_queue_struct queue;
  posetime_queue_type posetime;
  ulapi_real period;
  ulapi_integer socket_id;
  ulapi_integer client_id;
  ulapi_real m_per_length_units;
  ulapi_real rad_per_angle_units;
  ulapi_real length_units_per_m;
  ulapi_real angle_units_per_rad;
  int link_number;
  void * kinematics;
  go_link parameters[SERVO_NUM];
  char kin_name[GO_KIN_NAME_LEN];
  int timestamped;
  int nominal;
  int use_kinematics;
  int size;
  double latency;
  int latencies;
  ulapi_id traj_shm_key;
  void * traj_shm;
  traj_comm_struct * traj_comm_ptr;
  traj_stat_struct traj_stat;
  ulapi_real end;
  int start_it;
  int got_it;
  int heartbeat;
  go_pose pose;
  go_rpy rpy;
  go_mat mat;
  go_pose xform;
  int nchars;
  double x[17];			/* 16 plus 1 for a possible timestamp */
  int retval;

  period = 1.0;
  timestamped = 0;
  nominal = 0;
  use_kinematics = 0;
  latency = 0.0;
  size = 16 * sizeof(double);
  xform = go_pose_identity();
  rpy.r = rpy.p = rpy.y = 0.0;

  opterr = 0;
  for (;;) {
    option = getopt(argc, argv, ":i:u:t:p:x:kl:and:");
    if (option == -1)
      break;

    switch (option) {
    case 'i':
      strncpy(inifile_name, optarg, BUFFERLEN);
      inifile_name[BUFFERLEN - 1] = 0;
      break;

    case 't':
      period = (ulapi_real) atof(optarg);
      if (period <= 0.0) {
	fprintf(stderr, "bad value for -%c: %s: must be positive\n", option, optarg);
	return 1;
      }
      break;

    case 'p':
      port = atoi(optarg);
      break;

    case 'x':
      if (6 != sscanf(optarg, "%lf %lf %lf %lf %lf %lf",
		      &d1, &d2, &d3, &d4, &d5, &d6)) {
	fprintf(stderr, "bad value for -%c, need <x y z r p w>\n", optopt);
	return 1;
      }
      xform.tran.x = (go_real) d1;
      xform.tran.y = (go_real) d2;
      xform.tran.z = (go_real) d3;
      rpy.r = (go_real) d4;
      rpy.p = (go_real) d5;
      rpy.y = (go_real) d6;
      /* still need to convert from user to Go units */
      break;

    case 'k':
      use_kinematics = 1;
      break;

    case 'l':
      latency = atof(optarg);
      if (latency < 0.0) latency = 0.0;
      break;

    case 'a':
      timestamped = 1;
      size = 17 * sizeof(double);
      break;

    case 'd':
      if (1 != sscanf(optarg, "%i", &debug_mask)) {
	fprintf (stderr, "bad value for -%c: %s\n", option, optarg);
	return 1;
      }
      break;

    case 'n':
      nominal = 1;
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
  
  if (0 != ini_load(inifile_name,
		    &m_per_length_units,
		    &rad_per_angle_units,
		    &traj_shm_key,
		    &link_number,
		    parameters,
		    kin_name)) {
    fprintf(stderr, "%s: error: can't load %s\n", BN, inifile_name);
    return 1;
  }

  /* macros to convert from Go units to ini file units, from-go-length,angle */
  length_units_per_m = 1.0 / m_per_length_units;
  angle_units_per_rad = 1.0 / rad_per_angle_units;
#define FGL(x) (double) ((x) * length_units_per_m)
#define FGA(x) (double) ((x) * angle_units_per_rad)

  if (link_number < 1) {
    fprintf(stderr, "no links found\n");
    return 1;
  }

  if (link_number > SERVO_NUM) {
    fprintf(stderr, "too many links, %d but only %d handled\n", link_number, SERVO_NUM);
    return 1;
  }

  if (GO_RESULT_OK != go_kin_select(kin_name)) {
    fprintf(stderr, "%s: warning: using default kinematics %s instead of %s\n", BN, go_kin_get_name(), kin_name);
  }

  if (NULL == (kinematics = malloc(go_kin_size())) ||
      GO_RESULT_OK != go_kin_init(kinematics)) {
    fprintf(stderr, "%s: error: can't init kinematics\n", BN);
    return 1;
  }

  if (GO_RESULT_OK != 
      go_kin_set_parameters(kinematics,
			    parameters,
			    link_number)) {
    fprintf(stderr, "%s: error: can't set kinematics parameters\n", BN);
    return 1;
  }

  /* convert from user to go units */
  xform.tran.x *= m_per_length_units;
  xform.tran.y *= m_per_length_units;
  xform.tran.z *= m_per_length_units;
  rpy.r *= rad_per_angle_units;
  rpy.p *= rad_per_angle_units;
  rpy.y *= rad_per_angle_units;
  go_rpy_quat_convert(&rpy, &xform.rot);
  /*
    We were given the shift S wrt world frame {0},

    0
    .T
    S

    but we'll shift the device like this,

    S    S    0
    .T =  T *  T
    6    0    6

    to get the pose of the end wrt shifted frame {S}. We'll invert
    S once here to get what we want.
  */
  go_pose_inv(&xform, &xform);

#define QUIT(ret) retval = (ret); goto DONE

  traj_shm = NULL;
  traj_comm_ptr = NULL;
  retval = 0;

  /* get traj shared memory buffers */
  traj_shm = ulapi_rtm_new(traj_shm_key, sizeof(traj_comm_struct));
  if (NULL == traj_shm) {
    fprintf(stderr, "%s: error: can't get shared memory\n", BN);
    QUIT(1);
  }
  traj_comm_ptr = (traj_comm_struct *) ulapi_rtm_addr(traj_shm);

  /* check if we got it */
  for (start_it = 0, got_it = 0, end = ulapi_time() + CONNECT_WAIT_TIME;
       ulapi_time() < end;
       ulapi_sleep(period)) {
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

  pose = nominal ? traj_stat.ecp : traj_stat.ecp_act;

  /*
    Convert latency in seconds to latencies in cycles, i.e., 
    latency [secs] / period [secs/cycle] = latencies [cycles],
    e.g., 1.5 / 0.3 = 5. Then stuff this many poses onto the
    queue to get this latency.
  */
  latencies = (int) (latency / period + 0.5);
  posetime_queue_init(&queue);
  for (; latencies > 0; latencies--) {
    /* fill the queue with a bunch of 'heres' */
    posetime.pose = pose;
    posetime.timestamp = ulapi_time();
    posetime_queue_put(posetime, &queue);
  }

  socket_id = ulapi_socket_get_server_id(port);
  if (socket_id < 0) {
    fprintf(stderr, "can't serve port %d\n", (int) port);
    go_exit();
    return 1;
  }

#if defined(SIGPIPE) && defined(SIG_IGN)
  signal(SIGPIPE, SIG_IGN);
#endif
  signal(SIGINT, quit);
  done = 0;

  while (! done) {
    fprintf(stderr, "waiting for client connection...\n");
    client_id = ulapi_socket_get_connection_id(socket_id);
    if (client_id < 0) {
      break;
    }
    fprintf(stderr, "got one on fd %d\n", client_id);

    /* clear out our queue */
    (void) posetime_queue_clear(&queue);

    for (;;) {
      traj_stat = traj_comm_ptr->traj_stat;
      if (traj_stat.head == traj_stat.tail && traj_stat.type == TRAJ_STAT_TYPE) {
	/* got a good read */
	if (use_kinematics) {
	  pose = traj_stat.ecp_act; /* get estimate */
	  if (GO_RESULT_OK != go_kin_fwd(kinematics,
					 traj_stat.joints_act,
					 &pose)) {
	    fprintf(stderr, "%s: warning: can't calculate fwd kins\n", BN);
	  }
	} else {
	  pose = nominal ? traj_stat.ecp : traj_stat.ecp_act;
	}

	/*
	  Now shift according to
	
	  S    S    0
	  .T =  T *  T
	  6    0    6

	  by premultiplying our pose by the shift.
	*/
	go_pose_pose_mult(&xform, &pose, &pose);

	/* push this and the timestamp onto the queue */
	posetime.pose = pose;
	posetime.timestamp = (double) ulapi_time(); /* may not need this */
	posetime_queue_put(posetime, &queue);

	/* dump it out if requested */
	go_quat_rpy_convert(&posetime.pose.rot, &rpy);
	print_debug(1, "%f %f %f %f %f %f %f\n",
		    posetime.timestamp,
		    FGL(posetime.pose.tran.x), FGL(posetime.pose.tran.y), FGL(posetime.pose.tran.z),
		    FGA(rpy.r), FGA(rpy.p), FGA(rpy.y));

	/* and get the one at the front for sending */
	posetime_queue_get(&posetime, &queue);
	pose = posetime.pose;

	/* and put into rotation matrix form as needed by IGPS client */
	go_quat_mat_convert(&pose.rot, &mat);

	x[3] = (double) pose.tran.x;
	x[7] = (double) pose.tran.y;
	x[11] = (double) pose.tran.z;

	x[0] = (double) mat.x.x;
	x[4] = (double) mat.x.y;
	x[8] = (double) mat.x.z;

	x[1] = (double) mat.y.x;
	x[5] = (double) mat.y.y;
	x[9] = (double) mat.y.z;

	x[2] = (double) mat.z.x;
	x[6] = (double) mat.z.y;
	x[10] = (double) mat.z.z;

	if (timestamped) {
	  x[16] = (double) posetime.timestamp;
	}

	nchars = ulapi_socket_write(client_id, (char *) x, size);
	if (nchars != size) break;
      }	/* if (got a good read) */

      ulapi_sleep(period);
    } /* for (;;) on client write */

    ulapi_socket_close(client_id);

    fprintf(stderr, "closed %d\n", client_id);
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

  return retval || ulapi_exit() == ULAPI_OK ? 0 : 1;
}

