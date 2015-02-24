/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <stdlib.h>		/* malloc */
#include <string.h>		/* strncmp */
#include <ctype.h>		/* isspace */
#include <stddef.h>		/* NULL, sizeof */
#include <inifile.h>
#include <ulapi.h>		/* ulapi_time */
#include "go.h"			/* go_pose */
#include "gokin.h"
#include "trajintf.h"		/* TRAJ_KINEMATICS_PARAMETERS_NUM */

#ifdef HAVE_READLINE_READLINE_H
#include <readline/readline.h>	/* readline */
#include <readline/history.h>	/* using_history */
#endif

/* how many joints we can support */
enum {MAX_JOINT_NUM = 8};

static int
scan_em(go_real * array, char * string, int len)
{
  char * nptr = string;
  char * endptr;
  int i;

  for (i = 0; i < len; i++) {
    array[i] = (go_real) strtod(nptr, &endptr);
    if (nptr == endptr)
      break;			/* nothing was converted */
    nptr = endptr;
  }

  if (isgraph(*endptr)) {
    /* we have extra stuff after the last number */
    return -1;
  }

  return i;
}

static int
ini_load(char * inifile_name,
	 double * m_per_length_units,
	 double * rad_per_angle_units,
	 go_pose * home,
	 int * link_number,
	 go_link * link_params,
	 go_real * jhome,
	 char * kin_name)
{
  FILE * fp;
  const char * inistring;
  char * servo_string;
  int link;
  double d1, d2, d3, d4, d5, d6, d7, d8, d9;
  go_rpy rpy;

  if (NULL == (fp = fopen(inifile_name, "r"))) return 1;

  inistring = ini_find(fp, "LENGTH_UNITS_PER_M", "GOMOTION");
  if (NULL == inistring) {
    fprintf(stderr, "[GOMOTION] LENGTH_UNITS_PER_M not found, using 1\n");
  } else if (1 != sscanf(inistring, "%lf", &d1)) {
#undef CLOSE_AND_RETURN
#define CLOSE_AND_RETURN(ret)			\
    fclose(fp);					\
    return (ret)
    CLOSE_AND_RETURN(1);
    fprintf(stderr, "bad entry: [GOMOTION] LENGTH_UNITS_PER_M = %s\n", inistring);
    CLOSE_AND_RETURN(1);
  } else if (d1 <= 0.0) {
    fprintf(stderr, "invalid entry: [GOMOTION] LENGTH_UNITS_PER_M = %s must be positive\n", inistring);
    CLOSE_AND_RETURN(1);
  } else {
    *m_per_length_units = 1.0 / d1;
  }

  inistring = ini_find(fp, "ANGLE_UNITS_PER_RAD", "GOMOTION");
  if (NULL == inistring) {
    fprintf(stderr, "[GOMOTION] ANGLE_UNITS_PER_RAD not found, using 1\n");
  } else if (1 != sscanf(inistring, "%lf", &d1)) {
#undef CLOSE_AND_RETURN
#define CLOSE_AND_RETURN(ret)			\
    fclose(fp);					\
    return (ret)
    CLOSE_AND_RETURN(1);
    fprintf(stderr, "bad entry: [GOMOTION] ANGLE_UNITS_PER_RAD = %s\n", inistring);
    CLOSE_AND_RETURN(1);
  } else if (d1 <= 0.0) {
    fprintf(stderr, "invalid entry: [GOMOTION] ANGLE_UNITS_PER_RAD = %s must be positive\n", inistring);
    CLOSE_AND_RETURN(1);
  } else {
    *rad_per_angle_units = 1.0 / d1;
  }

  /* if kin_name is empty, set it from .ini file */
  if (0 == kin_name[0]) {
    inistring = ini_find(fp, "KINEMATICS", "TRAJ");
    if (NULL == inistring) {
      fprintf(stderr, "[TRAJ] KINEMATICS not found\n");
      CLOSE_AND_RETURN(1);
    }
    strncpy(kin_name, inistring, GO_KIN_NAME_LEN);
  } else {
    fprintf(stderr, "overriding ini file: [TRAJ] KINEMATICS = %s\n", kin_name);
  }

  inistring = ini_find(fp, "HOME", "TRAJ");
  if (NULL == inistring) {
    fprintf(stderr, "[TRAJ] HOME not found\n");
    CLOSE_AND_RETURN(1);
  }
  if (6 != sscanf(inistring, "%lf %lf %lf %lf %lf %lf",
		  &d1, &d2, &d3, &d4, &d5, &d6)) {
    fprintf(stderr, "invalid entry: [TRAJ] HOME\n");
    CLOSE_AND_RETURN(1);
  }
  home->tran.x = (go_real) (*m_per_length_units * d1);
  home->tran.y = (go_real) (*m_per_length_units * d2);
  home->tran.z = (go_real) (*m_per_length_units * d3);
  rpy.r = (go_real) (*rad_per_angle_units * d4);
  rpy.p = (go_real) (*rad_per_angle_units * d5);
  rpy.y = (go_real) (*rad_per_angle_units * d6);
  go_rpy_quat_convert(&rpy, &home->rot);

  servo_string = (char *) malloc(sizeof("SERVO_" + DIGITS_IN(link)));
  if (NULL == servo_string) {
    fprintf(stderr, "can't allocate space for SERVO_X section\n");
    CLOSE_AND_RETURN(1);
  }

  for (link = 0; ; link++) {
    sprintf(servo_string, "SERVO_%d", link + 1);

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

    go_body_init(&link_params[link].body);

    inistring = ini_find(fp, "MASS", servo_string);
    if (NULL != inistring) {
      if (1 == sscanf(inistring, "%lf", &d1)) {
	link_params[link].body.mass = d1;
      } else {
	fprintf(stderr, "bad entry: [%s] MASS = %s\n", servo_string, inistring);
	CLOSE_AND_RETURN(1);
      }
    }

#define TGL(x) (go_real) ((x) * (*m_per_length_units))
    inistring = ini_find(fp, "INERTIA", servo_string);
    if (NULL != inistring) {
      if (9 == sscanf(inistring, "%lf %lf %lf %lf %lf %lf %lf %lf %lf", &d1, &d2, &d3, &d4, &d5, &d6, &d7, &d8, &d9)) {
	link_params[link].body.inertia[0][0] = TGL(TGL(d1));
	link_params[link].body.inertia[0][1] = TGL(TGL(d2));
	link_params[link].body.inertia[0][2] = TGL(TGL(d3));
	link_params[link].body.inertia[1][0] = TGL(TGL(d4));
	link_params[link].body.inertia[1][1] = TGL(TGL(d5));
	link_params[link].body.inertia[1][2] = TGL(TGL(d6));
	link_params[link].body.inertia[2][0] = TGL(TGL(d7));
	link_params[link].body.inertia[2][1] = TGL(TGL(d8));
	link_params[link].body.inertia[2][2] = TGL(TGL(d9));
      } else {
	fprintf(stderr, "bad entry: [%s] INERTIA = %s\n", servo_string, inistring);
	CLOSE_AND_RETURN(1);
      }
    }
#undef TGL

    inistring = ini_find(fp, "HOME", servo_string);
    if (NULL == inistring) {
      /* no "HOME" in this section, or no section, so we're done */
      break;
    } else {
      if (1 == sscanf(inistring, "%lf", &d1)) {
	jhome[link] = link_params[link].quantity == GO_QUANTITY_ANGLE ? (go_real) (*rad_per_angle_units * d1) : (go_real) (*m_per_length_units * d1);
      } else {
	fprintf(stderr, "bad entry: [%s] HOME = %s\n", servo_string, inistring);
	CLOSE_AND_RETURN(1);
      }
    }

    if (NULL != (inistring = ini_find(fp, "DH_PARAMETERS", servo_string))) {
      if (4 == sscanf(inistring, "%lf %lf %lf %lf", &d1, &d2, &d3, &d4)) {
	go_dh dh;
	dh.a = (go_real) (*m_per_length_units * d1);
	dh.alpha = (go_real) (*rad_per_angle_units * d2);
	dh.d = (go_real) (*m_per_length_units * d3);
	dh.theta = (go_real) (*rad_per_angle_units * d4);
	link_params[link].u.dh = dh;
	link_params[link].type = GO_LINK_DH;
      } else {
	fprintf(stderr, "bad entry: [%s] DH = %s\n", servo_string, inistring);
	CLOSE_AND_RETURN(1);
      }
    } else if (NULL != (inistring = ini_find(fp, "PP_PARAMETERS", servo_string))) {
      if (6 == sscanf(inistring, "%lf %lf %lf %lf %lf %lf", &d1, &d2, &d3, &d4, &d5, &d6)) {
	go_rpy rpy;
	link_params[link].u.pp.pose.tran.x = (go_real) (*m_per_length_units * d1);
	link_params[link].u.pp.pose.tran.y = (go_real) (*m_per_length_units * d2);
	link_params[link].u.pp.pose.tran.z = (go_real) (*m_per_length_units * d3);
	rpy.r = (go_real) (*rad_per_angle_units * d4);
	rpy.p = (go_real) (*rad_per_angle_units * d5);
	rpy.y = (go_real) (*rad_per_angle_units * d6);
	go_rpy_quat_convert(&rpy, &link_params[link].u.pp.pose.rot);
	link_params[link].type = GO_LINK_PP;
      } else {
	fprintf(stderr, "bad entry: [%s] PP = %s\n", servo_string, inistring);
	CLOSE_AND_RETURN(1);
      }
    } else if (NULL != (inistring = ini_find(fp, "PK_PARAMETERS", servo_string))) {
      if (6 == sscanf(inistring, "%lf %lf %lf %lf %lf %lf", &d1, &d2, &d3, &d4, &d5, &d6)) {
	link_params[link].u.pk.base.x = (go_real) (*m_per_length_units * d1);
	link_params[link].u.pk.base.y = (go_real) (*m_per_length_units * d2);
	link_params[link].u.pk.base.z = (go_real) (*m_per_length_units * d3);
	link_params[link].u.pk.platform.x = (go_real) (*m_per_length_units * d4);
	link_params[link].u.pk.platform.y = (go_real) (*m_per_length_units * d5);
	link_params[link].u.pk.platform.z = (go_real) (*m_per_length_units * d6);
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

  CLOSE_AND_RETURN(0);
}

/*
  Syntax: gokintest -i <ini file>
*/

int main(int argc, char *argv[])
{
  enum { BUFFERLEN = 80 };
  int option;
  char inifile_name[BUFFERLEN] = "gomotion.ini";
  char prompt[BUFFERLEN];
#ifndef HAVE_READLINE_READLINE_H
  char buffer[BUFFERLEN];
  int len;
#endif
  char * line;
  char * ptr;
  double m_per_length_units;
  double length_units_per_m;
  double rad_per_angle_units;
  double angle_units_per_rad;
  double d1;
  int link_number;
  go_link link_params[MAX_JOINT_NUM];
  void * kinematics;
  char kin_name[GO_KIN_NAME_LEN] = "";
  int inverse = 0;
  int jacobian = 0;
  int matrixform = 0;
  int t;

#define mymax(a,b) ((a)>(b)?(a):(b))
  /* space to read in joints-jointvels or pos-vel */
  go_real reals[mymax(MAX_JOINT_NUM, 6)];

  /* current world position */
  go_pose current_position;
  /* home world position */
  go_pose home_position;

  /* current joint position */
  go_real current_joint[MAX_JOINT_NUM];
  /* home joint position */
  go_real home_joint[MAX_JOINT_NUM];

  go_rpy rpy;			/* we use RPY when printing orientation */
  ulapi_real start, diff;
  double delta_t = 1.0e-3;
  double delta_t_inv = 1.0 / delta_t;
  go_result retval;

#define TGL(x) ((go_real) ((x) * m_per_length_units))
#define TGA(x) ((go_real) ((x) * rad_per_angle_units))
#define TGQ(x,i) (link_params[i].quantity == GO_QUANTITY_LENGTH ? TGL(x) : TGA(x))

#define FGL(x) ((double ) ((x) * length_units_per_m))
#define FGA(x) ((double) ((x) * angle_units_per_rad))
#define FGQ(x,i) (link_params[i].quantity == GO_QUANTITY_LENGTH ? FGL(x) : FGA(x))

  opterr = 0;
  while (1) {
    option = getopt(argc, argv, ":i:k:");
    if (option == -1)
      break;

    switch (option) {
    case 'i':
      strncpy(inifile_name, optarg, BUFFERLEN);
      inifile_name[BUFFERLEN - 1] = 0;
      break;

    case 'k':
      strncpy(kin_name, optarg, BUFFERLEN);
      kin_name[BUFFERLEN - 1] = 0;
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
    fprintf(stderr, "can't init go\n");
    return 1;
  }

  if (ULAPI_OK != ulapi_init()) {
    fprintf(stderr, "can't init ulapi\n");
    return 1;
  }

  if (0 != ini_load(inifile_name,
		    &m_per_length_units,
		    &rad_per_angle_units,
		    &home_position,
		    &link_number,
		    link_params,
		    home_joint,
		    kin_name)) {
    fprintf(stderr, "can't load ini file %s\n", inifile_name);
    return 1;
  }

  if (link_number < 1) {
    fprintf(stderr, "no links found\n");
    return 1;
  }

  if (link_number > MAX_JOINT_NUM) {
    fprintf(stderr, "too many links, %d but only %d handled\n", link_number, MAX_JOINT_NUM);
    return 1;
  }

  length_units_per_m = 1.0 / m_per_length_units;
  angle_units_per_rad = 1.0 / rad_per_angle_units;

#define SET_HERE				\
  for (t = 0; t < link_number; t++) {		\
    current_joint[t] = home_joint[t];		\
  }						\
  current_position = home_position

#define PRINT_HERE							\
  printf("joints are ");						\
  for (t = 0; t < link_number; t++) {					\
    printf("%f ", (double) (link_params[t].quantity == GO_QUANTITY_ANGLE ? angle_units_per_rad * current_joint[t] : length_units_per_m * current_joint[t])); \
  }									\
  printf("\n");								\
  go_quat_rpy_convert(&current_position.rot, &rpy);			\
  printf("position is %f %f %f %f %f %f\n",				\
	 (double) (length_units_per_m * current_position.tran.x),	\
	 (double) (length_units_per_m * current_position.tran.y),	\
	 (double) (length_units_per_m * current_position.tran.z),	\
	 (double) (angle_units_per_rad * rpy.r),			\
	 (double) (angle_units_per_rad * rpy.p),			\
	 (double) (angle_units_per_rad * rpy.y))

  SET_HERE;
  PRINT_HERE;

#define PRINT_MASS							\
  printf(" ---\n");							\
  for (t = 0; t < link_number; t++) {					\
    printf("|%f %f %f|\n", FGL(FGL(link_params[t].body.inertia[0][0])), FGL(FGL(link_params[t].body.inertia[0][1])), FGL(FGL(link_params[t].body.inertia[0][2]))); \
    printf("|%f %f %f| (%f)\n", FGL(FGL(link_params[t].body.inertia[1][0])), FGL(FGL(link_params[t].body.inertia[1][1])), FGL(FGL(link_params[t].body.inertia[1][2])), (double) link_params[t].body.mass); \
    printf("|%f %f %f|\n ---\n", FGL(FGL(link_params[t].body.inertia[2][0])), FGL(FGL(link_params[t].body.inertia[2][1])), FGL(FGL(link_params[t].body.inertia[2][2]))); \
  }

#define PRINT_TYPE printf("%s\n", go_kin_get_name())

  if (GO_RESULT_OK != go_kin_select(kin_name)) {
    fprintf(stderr, "can't select kinematics %s\n", kin_name);
    return 1;
  }

  if (NULL == (kinematics = malloc(go_kin_size())) ||
      0 != go_kin_init(kinematics)) {
    fprintf(stderr, "can't initialize kinematics\n");
    return 1;
  }

  /* this tells the kinematics what the length and angle units are, so
     that the kinematics can correctly interpret the parameter numbers */
  if (GO_RESULT_OK != go_kin_set_parameters(kinematics, link_params, link_number)) {
    fprintf(stderr, "can't set kinematics parameters\n");
    return 1;
  }

  /* sync up the joints and world position, if possible */
  if (GO_KIN_BOTH == go_kin_get_type(kinematics) ||
      GO_KIN_FORWARD_ONLY == go_kin_get_type(kinematics)) {
    go_kin_fwd(kinematics, current_joint, &current_position);
  } else if (GO_KIN_INVERSE_ONLY == go_kin_get_type(kinematics)) {
    go_kin_inv(kinematics, &current_position, current_joint);
  } /* else leave the as in the ini file */

#ifdef HAVE_READLINE_READLINE_H
  using_history();
#endif

  while (! feof(stdin)) {
    if (inverse) {
      if (jacobian) {
	sprintf(prompt, "jinv> ");
      } else {
	sprintf(prompt, "inv> ");
      }
    } else {
      if (jacobian) {
	sprintf(prompt, "jfwd> ");
      } else {
	sprintf(prompt, "fwd> ");
      }
    }

#ifdef HAVE_READLINE_READLINE_H
    line = readline(prompt);
    if (NULL == line) {
      break;
    }
    if (*line) {
      add_history(line);
    }
#else
    printf(prompt);
    fflush(stdout);
    if (NULL == fgets(buffer, BUFFERLEN, stdin)) {
      break;
    }
    len = strlen(buffer);
    if (len > 0)
      buffer[len - 1] = 0;	/* take off newline */
    line = buffer;
#endif

    ptr = line;
    while (isspace(*ptr))
      ptr++;			/* skip leading white space */

    if (*ptr == 'i') {
      inverse = 1;
      continue;
    } else if (*ptr == 'f') {
      inverse = 0;
      continue;
    } else if (*ptr == 'j') {
      jacobian = 1;
      continue;
    } else if (*ptr == 'k') {
      jacobian = 0;
      continue;
    } else if (*ptr == 'm') {
      matrixform = !matrixform;
      continue;
    } else if (*ptr == 't') {
      PRINT_TYPE;
      continue;
    } else if (*ptr == 'q') {
      break;
    } else if (*ptr == 'p') {
      PRINT_HERE;
      continue;
    } else if (*ptr == 'M') {
      PRINT_MASS;
      continue;
    } else if (*ptr == 'h') {
      SET_HERE;
      PRINT_HERE;
      continue;
    } else if (*ptr == 'e') {
      while (!isspace(*ptr) && 0 != *ptr) ptr++;
      while (isspace(*ptr)) ptr++;
      if (0 == *ptr) printf("%g\n", (double) go_get_singular_epsilon());
      else if (1 == sscanf(ptr, "%lf", &d1)) {
	reals[0] = d1;
	go_set_singular_epsilon(reals[0]);
      } else {
	fprintf(stderr, "need an epsilon value\n");
      }
      continue;
    } else if (*ptr == 0) {
      /* blank line */
      continue;
    }

    if (inverse) {
      if (jacobian) {
	/* inverse Jacobian */
	go_vel vp;
	go_real jointvel[MAX_JOINT_NUM];
	go_real jointnew[MAX_JOINT_NUM];
	go_pose posnew;
	go_quat quat;
	go_rvec rvec;
	if (6 != scan_em(reals, ptr, 6)) {
	  fprintf(stderr, "need vx vy vz wx wy wz\n");
	} else {
	  vp.v.x = TGL(reals[0]);
	  vp.v.y = TGL(reals[1]);
	  vp.v.z = TGL(reals[2]);
	  vp.w.x = TGA(reals[3]);
	  vp.w.y = TGA(reals[4]);
	  vp.w.z = TGA(reals[5]);
	  start = ulapi_time();
	  retval = go_kin_jac_inv(kinematics, &current_position, &vp, current_joint, jointvel);
	  diff = ulapi_time() - start;
	  if (GO_RESULT_OK != retval) {
	    fprintf(stderr, "inv jacobian error: %s\n", go_result_to_string(retval));
	  } else {
	    for (t = 0; t < link_number; t++) {
	      printf("%f ", FGQ(jointvel[t], t));
	    }
	    printf("in %f seconds\n", (double) diff);
	    start = ulapi_time();
	    retval = go_kin_jac_fwd(kinematics, current_joint, jointvel, &current_position, &vp);
	    diff = ulapi_time() - start;
	    if (GO_RESULT_OK != retval) {
	      fprintf(stderr, "fwd kinematics error: %s\n", go_result_to_string(retval));
	    } else {
	      printf("%f %f %f %f %f %f in %f seconds\n",
		     FGL(vp.v.x),
		     FGL(vp.v.y),
		     FGL(vp.v.z),
		     FGA(vp.w.x),
		     FGA(vp.w.y),
		     FGA(vp.w.z),
		     (double) diff);
	      /* now move the joints a small increment and compare */
	      for (t = 0; t < link_number; t++) {
		jointnew[t] = current_joint[t] + jointvel[t] * delta_t;
	      }
	      posnew = current_position; /* set the estimate */
	      go_kin_fwd(kinematics, jointnew, &posnew);
	      /* to get angular increment Q from P to P', note that
		 QP = P', or Q = P'Pinv */
	      go_quat_inv(&current_position.rot, &quat);
	      go_quat_quat_mult(&posnew.rot, &quat, &quat);
	      go_quat_rvec_convert(&quat, &rvec);
	      printf("%f %f %f %f %f %f\n",
		     FGL(posnew.tran.x - current_position.tran.x) * delta_t_inv,
		     FGL(posnew.tran.y - current_position.tran.y) * delta_t_inv,
		     FGL(posnew.tran.z - current_position.tran.z) * delta_t_inv,
		     FGA(rvec.x) * delta_t_inv,
		     FGA(rvec.y) * delta_t_inv,
		     FGA(rvec.z) * delta_t_inv);
	    }
	  }
	}
      } else {
	/* inverse kinematics */
	go_rpy rpy;
	if (6 != scan_em(reals, ptr, 6)) {
	  fprintf(stderr, "need x y z r p w\n");
	} else {
	  current_position.tran.x = TGL(reals[0]);
	  current_position.tran.y = TGL(reals[1]);
	  current_position.tran.z = TGL(reals[2]);
	  rpy.r = TGA(reals[3]);
	  rpy.p = TGA(reals[4]);
	  rpy.y = TGA(reals[5]);
	  go_rpy_quat_convert(&rpy, &current_position.rot);
	  start = ulapi_time();
	  retval = go_kin_inv(kinematics, &current_position, current_joint);
	  diff = ulapi_time() - start;
	  if (GO_RESULT_OK != retval) {
	    fprintf(stderr, "inv kinematics error: %s\n", go_result_to_string(retval));
	  } else {
	    for (t = 0; t < link_number; t++) {
	      printf("%f ", FGQ(current_joint[t], t));
	    }
	    printf("in %f seconds\n", (double) diff);
	    start = ulapi_time();
	    retval = go_kin_fwd(kinematics, current_joint, &current_position);
	    diff = ulapi_time() - start;
	    if (GO_RESULT_OK != retval) {
	      fprintf(stderr, "fwd kinematics error: %s\n", go_result_to_string(retval));
	    } else {
	      go_quat_rpy_convert(&current_position.rot, &rpy);
	      printf("%f %f %f %f %f %f in %f seconds\n",
		     FGL(current_position.tran.x),
		     FGL(current_position.tran.y),
		     FGL(current_position.tran.z),
		     FGA(rpy.r),
		     FGA(rpy.p),
		     FGA(rpy.y),
		     (double) diff);
	    }
	  }
	}
      }
    } else {
      if (jacobian) {
	/* forward Jacobian */
	go_real jointvel[MAX_JOINT_NUM];
	go_vel vp;
	go_pose posnew;
	go_real jointnew[MAX_JOINT_NUM];
	go_quat quat;
	go_rvec rvec;
	if (link_number != scan_em(reals, ptr, link_number)) {
	  fprintf(stderr, "need %d joint vels\n", (int) link_number);
	} else {
	  for (t = 0; t < link_number; t++) {
	    jointvel[t] = TGQ(reals[t], t);
	  }
 	  start = ulapi_time();
	  retval = go_kin_jac_fwd(kinematics, current_joint, jointvel, &current_position, &vp);
	  diff = ulapi_time() - start;
	  if (GO_RESULT_OK != retval) {
	    fprintf(stderr, "fwd kinematics error: %s\n", go_result_to_string(retval));
	  } else {
	    printf("%f %f %f %f %f %f in %f seconds\n",
		   FGL(vp.v.x),
		   FGL(vp.v.y),
		   FGL(vp.v.z),
		   FGA(vp.w.x),
		   FGA(vp.w.y),
		   FGA(vp.w.z),
		   (double) diff);
	    start = ulapi_time();
	    retval = go_kin_jac_inv(kinematics, &current_position, &vp, current_joint, jointvel);
	    diff = ulapi_time() - start;
	    if (GO_RESULT_OK != retval) {
	      fprintf(stderr, "inv Jacobian error: %s\n", go_result_to_string(retval));
	    } else {
	      for (t = 0; t < link_number; t++) {
		printf("%f ", FGQ(jointvel[t], t));
	      }
	      printf("in %f seconds\n", (double) diff);
	      /* now move the position a small increment and compare */
	      posnew.tran.x = current_position.tran.x + vp.v.x * delta_t;
	      posnew.tran.y = current_position.tran.y + vp.v.y * delta_t;
	      posnew.tran.z = current_position.tran.z + vp.v.z * delta_t;
	      go_cart_scale_mult(&vp.w, delta_t, &vp.w);
	      rvec.x = vp.w.x, rvec.y = vp.w.y, rvec.z = vp.w.z;
	      go_rvec_quat_convert(&rvec, &quat);
	      go_quat_quat_mult(&quat, &current_position.rot, &posnew.rot);
	      for (t = 0; t < link_number; t++) jointnew[t] = current_joint[t];
	      retval = go_kin_inv(kinematics, &posnew, jointnew);
	      if (GO_RESULT_OK != retval) {
		fprintf(stderr, "inv kinematics error: %s\n", go_result_to_string(retval));
	      } else {
		for (t = 0; t < link_number; t++) {
		  printf("%f ", FGQ(jointnew[t] - current_joint[t], t) * delta_t_inv);
		}
		printf("\n");
	      }
	    }
	  }
	}
      } else {
	/* forward kinematics */
	if (link_number != scan_em(reals, ptr, link_number)) {
	  printf("need %d joint values\n", link_number);
	} else {
	  for (t = 0; t < link_number; t++) {
	    current_joint[t] = TGQ(reals[t], t);
	  }
	  start = ulapi_time();
	  retval = go_kin_fwd(kinematics, current_joint, &current_position);
	  diff = ulapi_time() - start;
	  if (GO_RESULT_OK != retval) {
	    fprintf(stderr, "fwd kinematics error: %s\n", go_result_to_string(retval));
	  } else {
	    go_quat_rpy_convert(&current_position.rot, &rpy);
	    printf("%f %f %f %f %f %f in %f seconds\n",
		   FGL(current_position.tran.x),
		   FGL(current_position.tran.y),
		   FGL(current_position.tran.z),
		   FGA(rpy.r),
		   FGA(rpy.p),
		   FGA(rpy.y),
		   (double) diff);
	    start = ulapi_time();
	    retval = go_kin_inv(kinematics, &current_position, current_joint);
	    diff = ulapi_time() - start;
	    if (GO_RESULT_OK != retval) {
	      fprintf(stderr, "inv kinematics error: %s\n", go_result_to_string(retval));
	    } else {
	      for (t = 0; t < link_number; t++) {
		printf("%f ", FGQ(current_joint[t], t));
	      }
	      printf("in %f seconds\n", (double) diff);
	    }
	  }
	}
      }
    }
#ifdef HAVE_READLINE_READLINE_H
    free(line);
#endif
  } /* while (! feof(stdin)) */

  return 0;
}
