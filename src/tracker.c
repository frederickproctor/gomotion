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
  tracker.c

  Standlone process that reads joint angles out of a Go Motion
  controller, runs them through the forward kinematics and writes the
  "true" Cartesian position into the traj_ref part of the traj comm
  struct. This simulates an external Cartesian position measurement
  system. For testing, the approach is to give this application the
  true .ini file, and give the controller a somewhat incorrect .ini
  file, and show that the tracker improves positioning.

  Testing results:

  With the cycle time set low, for high frequency, it causes the motion
  controller to go unstable. At 1 second it appears OK.
*/

#include <stddef.h>		/* NULL */
#include <stdio.h>		/* fprintf, stderr, FILE, fopen */
#include <stdlib.h>		/* malloc, sizeof, atoi */
#include <ctype.h>		/* isgraph */
#include <string.h>		/* strlen, strcpy */
#include <signal.h>		/* SIGINT, signal */
#include <inifile.h>
#include <ulapi.h>		/* ulapi_time */
#include "go.h"			/* go_init, etc */
#include "gokin.h"		/* go_kin_select, GO_KIN_NAME_LEN */
#include "servointf.h"		/* SERVO_NUM */
#include "trajintf.h"		/* traj_comm_struct, traj_ref_struct */

#define CONNECT_WAIT_TIME 3.0

static int
ini_load(char * inifile,
	 int * link_number,
	 go_link * link_params,
	 char * kin_name,
	 ulapi_id * traj_shm_key)
{
  FILE * fp;
  const char * inistring;
  char * servo_string;
  int link;
  double m_per_length_units;
  double rad_per_angle_units;
  int i1;
  double d1, d2, d3, d4, d5, d6;

  if (NULL == (fp = fopen(inifile, "r"))) return 1;

  inistring = ini_find(fp, "LENGTH_UNITS_PER_M", "GOMOTION");
  if (NULL == inistring) {
    fprintf(stderr, "[GOMOTION] LENGTH_UNITS_PER_M not found, using 1\n");
  } else if (1 != sscanf(inistring, "%lf", &d1)) {
#undef CLOSE_AND_RETURN
#define CLOSE_AND_RETURN(ret)			\
    fclose(fp); \
    return (ret)
    CLOSE_AND_RETURN(1);
    fprintf(stderr, "bad entry: [GOMOTION] LENGTH_UNITS_PER_M = %s\n", inistring);
    CLOSE_AND_RETURN(1);
  } else if (d1 <= 0.0) {
    fprintf(stderr, "invalid entry: [GOMOTION] LENGTH_UNITS_PER_M = %s must be positive\n", inistring);
    CLOSE_AND_RETURN(1);
  } else {
    m_per_length_units = 1.0 / d1;
  }

  inistring = ini_find(fp, "ANGLE_UNITS_PER_RAD", "GOMOTION");
  if (NULL == inistring) {
    fprintf(stderr, "[GOMOTION] ANGLE_UNITS_PER_RAD not found, using 1\n");
  } else if (1 != sscanf(inistring, "%lf", &d1)) {
#undef CLOSE_AND_RETURN
#define CLOSE_AND_RETURN(ret)			\
    fclose(fp); \
    return (ret)
    CLOSE_AND_RETURN(1);
    fprintf(stderr, "bad entry: [GOMOTION] ANGLE_UNITS_PER_RAD = %s\n", inistring);
    CLOSE_AND_RETURN(1);
  } else if (d1 <= 0.0) {
    fprintf(stderr, "invalid entry: [GOMOTION] ANGLE_UNITS_PER_RAD = %s must be positive\n", inistring);
    CLOSE_AND_RETURN(1);
  } else {
    rad_per_angle_units = 1.0 / d1;
  }

  inistring = ini_find(fp, "KINEMATICS", "TRAJ");
  if (NULL == inistring) {
    fprintf(stderr, "[GOMOTION] TRAJ not found\n");
    CLOSE_AND_RETURN(1);
  }
  strncpy(kin_name, inistring, GO_KIN_NAME_LEN);

  servo_string = (char *) malloc(sizeof("SERVO_" + DIGITS_IN(link)));
  if (NULL == servo_string) {
    fprintf(stderr, "can't allocate space for SERVO_X section\n");
    CLOSE_AND_RETURN(1);
  }

  for (link = 0; ; link++) {
    sprintf(servo_string, "SERVO_%d", link + 1);
    /* go for QUANTITY */
    inistring = ini_find(fp, "QUANTITY", servo_string);
    if (NULL == inistring) {
      /* no "QUANTITY" in this section, or no section, so we're done */
      break;
    } else {
      if (ini_match(inistring, "ANGLE")) {
	link_params[link].quantity = GO_QUANTITY_ANGLE;
      } else if (ini_match(inistring, "LENGTH")) {
	link_params[link].quantity = GO_QUANTITY_LENGTH;
      } else {
	fprintf(stderr, "bad entry: [%s] QUANTITY = %s\n", servo_string, inistring);
	CLOSE_AND_RETURN(1);
      }
    }

    if (NULL != (inistring = ini_find(fp, "DH_PARAMETERS", servo_string))) {
      if (4 == sscanf(inistring, "%lf %lf %lf %lf", &d1, &d2, &d3, &d4)) {
	go_dh dh;
	dh.a = (go_real) (m_per_length_units * d1);
	dh.alpha = (go_real) (rad_per_angle_units * d2);
	dh.d = (go_real) (m_per_length_units * d3);
	dh.theta = (go_real) (rad_per_angle_units * d4);
	link_params[link].u.dh = dh;
	link_params[link].type = GO_LINK_DH;
      } else {
	fprintf(stderr, "bad entry: [%s] DH = %s\n", servo_string, inistring);
	CLOSE_AND_RETURN(1);
      }
    } else if (NULL != (inistring = ini_find(fp, "PP_PARAMETERS", servo_string))) {
      if (6 == sscanf(inistring, "%lf %lf %lf %lf %lf %lf", &d1, &d2, &d3, &d4, &d5, &d6)) {
	go_rpy rpy;
	link_params[link].u.pp.pose.tran.x = (go_real) (m_per_length_units * d1);
	link_params[link].u.pp.pose.tran.y = (go_real) (m_per_length_units * d2);
	link_params[link].u.pp.pose.tran.z = (go_real) (m_per_length_units * d3);
	rpy.r = (go_real) (rad_per_angle_units * d4);
	rpy.p = (go_real) (rad_per_angle_units * d5);
	rpy.y = (go_real) (rad_per_angle_units * d6);
	go_rpy_quat_convert(&rpy, &link_params[link].u.pp.pose.rot);
	link_params[link].type = GO_LINK_PP;
      } else {
	fprintf(stderr, "bad entry: [%s] PP = %s\n", servo_string, inistring);
	CLOSE_AND_RETURN(1);
      }
    } else if (NULL != (inistring = ini_find(fp, "PK_PARAMETERS", servo_string))) {
      if (6 == sscanf(inistring, "%lf %lf %lf %lf %lf %lf", &d1, &d2, &d3, &d4, &d5, &d6)) {
	link_params[link].u.pk.base.x = (go_real) (m_per_length_units * d1);
	link_params[link].u.pk.base.y = (go_real) (m_per_length_units * d2);
	link_params[link].u.pk.base.z = (go_real) (m_per_length_units * d3);
	link_params[link].u.pk.platform.x = (go_real) (m_per_length_units * d4);
	link_params[link].u.pk.platform.y = (go_real) (m_per_length_units * d5);
	link_params[link].u.pk.platform.z = (go_real) (m_per_length_units * d6);
	link_params[link].type = GO_LINK_PK;
      } else {
	fprintf(stderr, "bad entry: [%s] PK = %s\n", servo_string, inistring);
	CLOSE_AND_RETURN(1);
      }
    } else {
      /* no "DH,PP,PK_PARAMETERS" in this section, so we're done */
      break;
    }
  } /* for (link) */
  *link_number = link;

  inistring = ini_find(fp, "SHM_KEY", "TRAJ");
  if (NULL == inistring) {
    fprintf(stderr, "[TRAJ] SHM_KEY not found\n");
    CLOSE_AND_RETURN(1);
  } else if (1 != sscanf(inistring, "%i", &i1)) {
    fprintf(stderr, "bad entry: [TRAJ] SHM_KEY = %s\n", inistring);
    CLOSE_AND_RETURN(1);
  }
  *traj_shm_key = (ulapi_id) i1;

  CLOSE_AND_RETURN(0);
}

/* basename */
char *
mybasename(char * s)
{
  char * end;

  end = s + strlen(s);

  while (--end >= s) {
    if ('/' == *end) break;
  }

  return *(end+1) == 0 ? end : end+1;
}

static int done = 0;
static void quit(int sig)
{
  done = 1;
}

/*
  Usage: tracker -i <ini file>
*/

int main(int argc, char *argv[])
{
#define BN mybasename(argv[0])
  enum { BUFFERLEN = 80 };
  int option;
  char inifile_name[BUFFERLEN] = "gomotion.ini";
  ulapi_real period;
  int reset;
  int link_number;
  void * kinematics;
  go_link parameters[SERVO_NUM];
  char kin_name[GO_KIN_NAME_LEN];
  ulapi_id traj_shm_key;
  void * traj_shm;
  traj_comm_struct * traj_comm_ptr;
  traj_ref_struct * traj_ref_ptr;
  traj_stat_struct pp_traj_stat[2], * traj_stat_ptr, * traj_stat_test;
  void * tmp;
  go_pose Xinvim1, Xinv, Ai, Ainvi, Ni;
  go_real cartmag, quatmag;
  double end;
  int start_it;
  int got_it;
  int heartbeat;
  int retval;

  period = 1.0;
  reset = 0;

  opterr = 0;
  while (1) {
    option = getopt(argc, argv, ":i:u:t:r");
    if (option == -1)
      break;

    switch (option) {
    case 'i':
      strncpy(inifile_name, optarg, BUFFERLEN);
      inifile_name[BUFFERLEN - 1] = 0;
      break;

    case 't':
      period = (ulapi_real) atof(optarg);
      break;

    case 'r':
      reset = 1;
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

  if (0 != go_init()) {
    fprintf(stderr, "%s: error: can't init gomotion\n", BN);
    return 1;
  }

  if (0 != ulapi_init()) {
    fprintf(stderr, "%s: error: can't init ulapi\n", BN);
    return 1;
  }
  
  if (0 != ini_load(inifile_name,
		    &link_number,
		    parameters,
		    kin_name,
		    &traj_shm_key)) {
    fprintf(stderr, "%s: error: can't load %s\n", BN, inifile_name);
    return 1;
  }

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

  traj_shm = NULL;
  traj_comm_ptr = NULL;
  retval = 0;
#define QUIT(ret) retval = (ret); goto DONE

  /* get traj shared memory buffers */
  traj_shm = ulapi_rtm_new(traj_shm_key, sizeof(traj_comm_struct));
  if (NULL == traj_shm) {
    fprintf(stderr, "%s: error: can't get shared memory\n", BN);
    QUIT(1);
  }
  traj_comm_ptr = (traj_comm_struct *) ulapi_rtm_addr(traj_shm);
  traj_ref_ptr = &traj_comm_ptr->traj_ref;
  /* set up traj status ping-pong buffers */
  traj_stat_ptr = &pp_traj_stat[0];
  traj_stat_test = &pp_traj_stat[1];
  /* check for traj life */
  for (start_it = 0, got_it = 0, end = ulapi_time() + CONNECT_WAIT_TIME;
       ulapi_time() < end;
       ulapi_sleep(0.1)) {
    *traj_stat_ptr = traj_comm_ptr->traj_stat;
    if (traj_stat_ptr->head == traj_stat_ptr->tail &&
	traj_stat_ptr->type == TRAJ_STAT_TYPE) {
      if (! start_it) {
	start_it = 1;
	heartbeat = traj_stat_ptr->heartbeat;
      }
      if (heartbeat != traj_stat_ptr->heartbeat) {
	got_it = 1;
	break;
      }
    }
  }
  if (! got_it) {
    fprintf(stderr, "%s: error: can't connect to traj\n", BN);
    QUIT(1);
  }

  if (reset) {
    traj_ref_ptr->head++;
    traj_ref_ptr->xinv = go_pose_identity();
    traj_ref_ptr->tail = traj_ref_ptr->head;
    QUIT(0);
  }

  signal(SIGINT, quit);
  done = 0;

  while (! done) {
    /*
      Read position and joints, run forward kinematics, write
      resulting position into traj ref
    */

    /* read in traj status, ping-pong style */
    *traj_stat_test = traj_comm_ptr->traj_stat;
    if (traj_stat_test->head == traj_stat_test->tail) {
      tmp = traj_stat_ptr;
      traj_stat_ptr = traj_stat_test;
      traj_stat_test = tmp;
    }

    /* read out controller's nominal position N(i) and last Xinv(i-1)*/
    Ni = traj_stat_ptr->ecp;
    Xinvim1 = traj_stat_ptr->xinv;
    /* compute part of Xinv(i) = Ainv(i) N(i) Xinv(i-1) */
    go_pose_pose_mult(&Ni, &Xinvim1, &Xinv);

    /* run our fwd kins using controller's actual joints to get actual
       position A(i) */
    Ai = Ni;			/* need an estimate for convergence */
    if (GO_RESULT_OK != go_kin_fwd(kinematics,
				   traj_stat_ptr->joints_act,
				   &Ai)) {
      fprintf(stderr, "%s: warning: can't calculate fwd kins\n", BN);
    } else {
      /* compute the rest of Xinv(i) = Ainv(i) N(i) Xinv(i-1) */
      go_pose_inv(&Ai, &Ainvi);
      go_pose_pose_mult(&Ainvi, &Xinv, &Xinv);
      /* write the actual position into the reference */
      traj_ref_ptr->head++;
      traj_ref_ptr->xinv = Xinv;
      traj_ref_ptr->tail = traj_ref_ptr->head;
      go_cart_mag(&Xinv.tran, &cartmag);
      go_quat_mag(&Xinv.rot, &quatmag);
      printf("%f %f\n", (double) cartmag, (double) quatmag);
    }

    ulapi_sleep(period);
  }

 DONE:

  if (NULL != traj_shm) {
    ulapi_rtm_delete(traj_shm);
    traj_shm = NULL;
  }
  traj_comm_ptr = NULL;
  if (NULL != kinematics) {
    free(kinematics);
  }
  kinematics = NULL;

  (void) go_exit();

  printf("%s done\n", BN);

  return retval;
}

