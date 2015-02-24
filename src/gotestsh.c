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
  \file gotestsh.c

  \brief Interactive shell for testing the motion functions defined in
  gomotion.c. The implementation looks similar to the Go Motion shell
  \c gosh, but \a gotestsh doesn't connect to a running controller, it
  only links in the motion library and runs functions directly.
*/

/*
  Options:

  -t <time increment, real seconds>
  -n <interpolation multiple, integer>
  -k for constant interpolation
  -l for linear interpolation
  -c for cubic interpolation
  -q for quintic interpolation
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>		/* feof, stdin,out, fflush, fgets */
#include <stddef.h>		/* NULL */
#include <stdlib.h>		/* free, strtod,l */
#include <string.h>		/* strncmp */
#include <ctype.h>		/* isspace */
#include <unistd.h>		/* getopt, opterr */

#ifdef HAVE_READLINE_READLINE_H
#include <readline/readline.h>	/* readline */
#include <readline/history.h>	/* using_history */
#endif

#include "go.h"

/* find all of 'a' in 'b', and following chars of 'b' are space or 0 */
static int strwcmp(char * a, char * b)
{
  int i;

  i = strlen(a);
  if (0 != strncmp(a, b, i)) return 1;
  if (0 == b[i] || isspace(b[i])) return 0;

  return 1;
}

/* read numbers in a string into the double array, returning number found */
static int scan_numbers(double * array, char * string, int len)
{
  char * nptr = string;
  char * endptr;
  int i, count;

  for (i = 0; i < len; i++) {
    array[i] = strtod(nptr, &endptr);
    if (nptr == endptr) break;	/* nothing was converted */
    nptr = endptr;
  }
  count = i;
  for (i = count; i < len; i++) {
    array[i] = 0;
  }

  /* if isgraph(*endptr) we have extra numbers; ignore */

  return count;
}

void go_motion_spec_print(go_motion_spec * gms)
{
  int i;
  go_rpy rpy;

  if (gms->type == GO_MOTION_JOINT) {
    fprintf(stderr, "type:   joint\n");
    fprintf(stderr, "start:  ");
    for (i = 0; i < GO_MOTION_JOINT_NUM; i++) {
      fprintf(stderr, "%f ", (double) gms->start.u.joint[i]);
    }
    fprintf(stderr, "\n");
    fprintf(stderr, "end:  ");
    for (i = 0; i < GO_MOTION_JOINT_NUM; i++) {
      fprintf(stderr, "%f ", (double) gms->end.u.joint[i]);
    }
    fprintf(stderr, "\n");
    fprintf(stderr, "jpar:\n");
    for (i = 0; i < GO_MOTION_JOINT_NUM; i++) {
      fprintf(stderr, "%d %f %f %f", i,
	      (double) gms->par[i].vel,
	      (double) gms->par[i].acc,
	      (double) gms->par[i].jerk);
      fprintf(stderr, "\n");
    }
  } else if (gms->type == GO_MOTION_LINEAR) {
    fprintf(stderr, "type:   linear\n");
    (void) go_quat_rpy_convert(&gms->start.u.pose.rot, &rpy);
    fprintf(stderr, "start:  %f %f %f %f %f %f\n", 
	    (double) gms->start.u.pose.tran.x,
	    (double) gms->start.u.pose.tran.y,
	    (double) gms->start.u.pose.tran.z,
	    (double) GO_TO_DEG(rpy.r), 
	    (double) GO_TO_DEG(rpy.p), 
	    (double) GO_TO_DEG(rpy.y));
    (void) go_quat_rpy_convert(&gms->end.u.pose.rot, &rpy);
    fprintf(stderr, "end:  %f %f %f %f %f %f\n", 
	    (double) gms->end.u.pose.tran.x,
	    (double) gms->end.u.pose.tran.y,
	    (double) gms->end.u.pose.tran.z,
	    (double) GO_TO_DEG(rpy.r), 
	    (double) GO_TO_DEG(rpy.p), 
	    (double) GO_TO_DEG(rpy.y));
    fprintf(stderr, "tpar/rpar: %f %f %f / %f %f %f\n",
	    (double) gms->par[0].vel,
	    (double) gms->par[0].acc,
	    (double) gms->par[0].jerk,
	    (double) GO_TO_DEG(gms->par[1].vel),
	    (double) GO_TO_DEG(gms->par[1].acc),
	    (double) GO_TO_DEG(gms->par[1].jerk));
  } else {
    fprintf(stderr, "type:   circular\n");
    fprintf(stderr, "(need to print more)\n");
  }

  fprintf(stderr, "id:   %d\n", (int) gms->id);

  fprintf(stderr, "totalt:   %f\n", (double) gms->totalt);
}

#define max(a,b) ((a)>(b)?(a):(b))

int main(int argc, char *argv[])
{
#ifndef HAVE_READLINE_READLINE_H
  enum {BUFFER_SIZE = 256};
  char buffer[BUFFER_SIZE];
  int len;
#endif
  char * line;
  char * ptr;
  char option;
  enum {PRINT_POS, PRINT_VEL, PRINT_ACC, PRINT_JERK} print_type;
  enum {MAX_NUMBERS = max(6, GO_MOTION_JOINT_NUM)};
  /* we need at least 12 so far, given the 12 inputs for "movec" */
  double da[max(12, MAX_NUMBERS)];
  int count, i, ii;
  int doruns, runs;
  int interp_count;
  enum { QUEUE_SIZE = 128 };
  go_real time, deltat;
  go_real movetime;
  go_real interp_s, interp_deltat, interp_deltat_inv, interp_deltas;
  go_motion_spec gms, space[QUEUE_SIZE];
  go_motion_queue gmq;
  go_real tvel, tacc, tjerk;
  go_real rvel, racc, rjerk;
  go_real jvel[GO_MOTION_JOINT_NUM], jacc[GO_MOTION_JOINT_NUM], jjerk[GO_MOTION_JOINT_NUM];
  go_interp interp[MAX_NUMBERS];
  go_position position;
  go_real numbers[MAX_NUMBERS];
  go_real pos[MAX_NUMBERS], old_pos[MAX_NUMBERS];
  go_real vel[MAX_NUMBERS], old_vel[MAX_NUMBERS];
  go_real acc[MAX_NUMBERS], old_acc[MAX_NUMBERS];
  go_real jerk[MAX_NUMBERS];	/* no old jerks around here */
  go_interp_add_func go_interp_add = go_interp_add_quintic_pdva;
  go_interp_eval_func go_interp_eval = go_interp_eval_quintic;
  double d1, d2, d3;

  print_type = PRINT_POS;
  count = MAX_NUMBERS;
  time = 0.0;
  deltat = 0.1;
  movetime = -1.0;
  interp_count = 10;

  opterr = 0;			/* inhibit automatic getopt error printing */
  while (1) {
    option = getopt(argc, argv, ":p:t:n:");
    if (option == -1)
      break;

    switch (option) {
    case 't':
      deltat = strtod(optarg, NULL);
      break;
    case 'n':
      interp_count = strtol(optarg, NULL, 10);
      break;
    case ':':
      /* missing option argument */
      fprintf(stderr, "missing argument to -%c\n", (char) optopt);
      return 1;
    case '?':
      /* unrecognized argument */
      fprintf(stderr, "unrecognized option -%c\n", (char) optopt);
      return 1;
    }
  }
  if (argc > optind) {
    fprintf(stderr, "unrecognized argument %s\n", argv[optind]);
    return 1;
  }
  if (deltat < GO_REAL_EPSILON) {
    fprintf(stderr, "-t option must be positive\n");
    return 1;
  }
  if (interp_count < 1) {
    fprintf(stderr, "-n option must be 1 or more\n");
    return 1;
  }

  interp_deltas = 1.0 / ((double) interp_count);
  interp_deltat = deltat * interp_deltas;
  interp_deltat_inv = 1.0 / interp_deltat;

  if (0 != go_init() ||
      GO_RESULT_OK != go_motion_queue_init(&gmq, space, QUEUE_SIZE, deltat) ||
      GO_RESULT_OK != go_motion_queue_set_type(&gmq, GO_MOTION_NONE)) {
    return 1;
  }

  tvel = tacc = tjerk = 1;
  rvel = racc = rjerk = 1;
  for (i = 0; i < GO_MOTION_JOINT_NUM; i++) {
    jvel[i] = jacc[i] = jjerk[i] = 1;
  }

  go_motion_spec_init(&gms);
  go_motion_spec_set_type(&gms, GO_MOTION_LINEAR);
  go_motion_spec_set_id(&gms, 1);
  go_position_zero_pose(&gms.start);
  go_position_zero_pose(&gms.end);
  go_motion_spec_set_tpar(&gms, tvel, tacc, tjerk);
  go_motion_spec_set_rpar(&gms, rvel, racc, rjerk);

#ifdef HAVE_READLINE_READLINE_H
  using_history();
#endif

  for (i = 0; i < MAX_NUMBERS; i++) {
    old_pos[i] = 0.0;
    old_vel[i] = 0.0;
    old_acc[i] = 0.0;
  }

  while (!feof(stdin)) {
#ifdef HAVE_READLINE_READLINE_H
    line = readline(NULL);
    if (NULL == line) {
      break;
    }
    if (*line) {
      add_history(line);
    }
#else
    if (NULL == fgets(buffer, BUFFER_SIZE, stdin)) {
      break;
    }
    len = strlen(buffer);
    if (len > 0) buffer[len-1] = 0; /* take off newline */
    line = buffer;
#endif

#define TRY(s) if (! strwcmp(s, ptr))
#define TRYEM(s1,s2) if (! strwcmp(s1, ptr) || ! strwcmp(s2, ptr))
#define SKIPWHITE while (isspace(*ptr)) ptr++
#define SKIPWORD				\
    while (!isspace(*ptr) && 0 != *ptr) ptr++;	\
    while (isspace(*ptr)) ptr++

    ptr = line;
    SKIPWHITE;
    if (0 == *ptr) {
      continue;	/* blank line */
    } else if ('#' == *ptr) {
      continue;			/* comment */
    } else TRYEM("q", "quit") {
      break;			/* we're done */
    } else TRY("time") {
      SKIPWORD;
      if (1 == sscanf(ptr, "%lf", &da[0])) {
	movetime = da[0];
      } else {
	fprintf(stderr, "need a time for subsequent moves\n");
      }
    } else TRY("notime") {
      movetime = -1;
    } else TRY("movew") {
      go_rpy rpy;
      SKIPWORD;
      /* for a world move, we need exactly 6 numbers, X Y Z R P W */
      if (6 == sscanf(ptr, "%lf %lf %lf %lf %lf %lf",
		      &da[0], &da[1], &da[2], 
		      &da[3], &da[4], &da[5])) {
	gms.type = GO_MOTION_LINEAR;
	go_motion_spec_set_time(&gms, movetime);
	gms.end.u.pose.tran.x = (go_real) da[0];
	gms.end.u.pose.tran.y = (go_real) da[1];
	gms.end.u.pose.tran.z = (go_real) da[2];
	rpy.r = (go_real) GO_TO_RAD(da[3]);
	rpy.p = (go_real) GO_TO_RAD(da[4]);
	rpy.y = (go_real) GO_TO_RAD(da[5]);
	(void) go_rpy_quat_convert(&rpy, &gms.end.u.pose.rot);
	go_motion_spec_set_tpar(&gms, tvel, tacc, tjerk);
	go_motion_spec_set_rpar(&gms, rvel, racc, rjerk);

	if (GO_MOTION_WORLD != go_motion_queue_get_type(&gmq)) {
	  go_motion_queue_reset(&gmq);
	  go_motion_queue_set_type(&gmq, GO_MOTION_WORLD);
	  go_motion_queue_set_here(&gmq, &gms.end);
	  old_pos[0] = gms.end.u.pose.tran.x;
	  old_pos[1] = gms.end.u.pose.tran.y;
	  old_pos[2] = gms.end.u.pose.tran.z;
	  old_pos[3] = rpy.r;
	  old_pos[4] = rpy.p;
	  old_pos[5] = rpy.y;
	  for (i = 0; i < 6; i++) {
	    go_interp_set_here(&interp[i], old_pos[i]);
	  }
	}

	if (GO_RESULT_OK != go_motion_queue_append(&gmq, &gms)) {
	  fprintf(stderr, "can't append world move\n");
	}
      } else {
	fprintf(stderr, "movew needs X Y Z R P W\n");
      }
      count = 6;		/* so we print them all */
    } else TRY("movec") {
      go_rpy rpy;
      SKIPWORD;
      /* format is end-xyzrpw, center-xyz, normal-xyz, turns */
      /* NB -- if you change this from 13, change the decl for da[] above */
      if (13 == 
	  sscanf(ptr,
		 "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d",
		 &da[0], &da[1], &da[2], &da[3], &da[4], &da[5],
		 &da[6], &da[7], &da[8],
		 &da[9], &da[10], &da[11],
		 &i)) {
	gms.type = GO_MOTION_CIRCULAR;
	gms.end.u.pose.tran.x = (go_real) da[0];
	gms.end.u.pose.tran.y = (go_real) da[1];
	gms.end.u.pose.tran.z = (go_real) da[2];
	rpy.r = (go_real) GO_TO_RAD(da[3]);
	rpy.p = (go_real) GO_TO_RAD(da[4]);
	rpy.y = (go_real) GO_TO_RAD(da[5]);
	(void) go_rpy_quat_convert(&rpy, &gms.end.u.pose.rot);
	gms.u.cpar.center.x = (go_real) da[6];
	gms.u.cpar.center.y = (go_real) da[7];
	gms.u.cpar.center.z = (go_real) da[8];
	gms.u.cpar.normal.x = (go_real) da[9];
	gms.u.cpar.normal.y = (go_real) da[10];
	gms.u.cpar.normal.z = (go_real) da[11];
	gms.u.cpar.turns = (go_integer) i;
	go_motion_spec_set_tpar(&gms, tvel, tacc, tjerk);
	go_motion_spec_set_rpar(&gms, rvel, racc, rjerk);

	if (GO_MOTION_WORLD != go_motion_queue_get_type(&gmq)) {
	  go_motion_queue_reset(&gmq);
	  go_motion_queue_set_type(&gmq, GO_MOTION_WORLD);
	  go_motion_queue_set_here(&gmq, &gms.end);
	  old_pos[0] = gms.end.u.pose.tran.x;
	  old_pos[1] = gms.end.u.pose.tran.y;
	  old_pos[2] = gms.end.u.pose.tran.z;
	  old_pos[3] = rpy.r;
	  old_pos[4] = rpy.p;
	  old_pos[5] = rpy.y;
	  for (i = 0; i < 6; i++) {
	    go_interp_set_here(&interp[i], old_pos[i]);
	  }
	}

	if (GO_RESULT_OK != go_motion_queue_append(&gmq, &gms)) {
	  fprintf(stderr, "can't append circular move\n");
	}
      } else {
	fprintf(stderr, "movec needs end-x y z r p w center-x y z normal-x y z turns\n");
      }
      count = 6;
    } else TRYEM("movej", "moveuj") {
      go_flag type;
      TRY("movej") type = GO_MOTION_JOINT;
      else type = GO_MOTION_UJOINT;
      SKIPWORD;
      i = scan_numbers(da, ptr, MAX_NUMBERS);
      if (i <= 0) {
	fprintf(stderr, "need at least 1 joint value\n");
      } else if (i > GO_MOTION_JOINT_NUM) {
	fprintf(stderr, "need at most %d joint values\n", GO_MOTION_JOINT_NUM);
      } else {
	count = i;		/* save the count of joints */
	for (i = 0; i < count; i++) {
	  /* fill in what we got */
	  gms.end.u.joint[i] = (go_real) da[i];
	  go_motion_spec_set_jpar(&gms, i, jvel[i], jacc[i], jjerk[i]);
	}
	for (i = count; i < GO_MOTION_JOINT_NUM; i++) {
	  /* zero out the remaining joint goals */
	  gms.end.u.joint[i] = 0;
	  /* leaving the remaining jpars the same */
	  go_motion_spec_set_jpar(&gms, i, jvel[i], jacc[i], jjerk[i]);
	}

	if (type != go_motion_queue_get_type(&gmq)) {
	  go_motion_queue_reset(&gmq);
	  go_motion_queue_set_type(&gmq, type);
	  go_motion_queue_set_here(&gmq, &gms.end);
	  for (i = 0; i < GO_MOTION_JOINT_NUM; i++) {
	    old_pos[i] = gms.end.u.joint[i];
	    go_interp_set_here(&interp[i], old_pos[i]);
	  }
	}
	go_motion_spec_set_time(&gms, movetime);
	if (GO_RESULT_OK != go_motion_queue_append(&gmq, &gms)) {
	  fprintf(stderr, "can't append %s move\n", type == GO_MOTION_JOINT ? "joint" : "ujoint");
	}
      }
    } else TRYEM("r", "run") {
      SKIPWORD;
      doruns = 0;
      if (1 == sscanf(ptr, "%d", &i) && i > 0) {
	/* we asked for a specific number of runs, so force
	   them regardless of whether queue is empty */
	doruns = 1;
	runs = i;
      }
      while (1) {
	go_rpy rpy;
	(void) go_motion_queue_interp(&gmq, &position);
	if (GO_MOTION_WORLD == go_motion_queue_get_type(&gmq)) {
	  numbers[0] = position.u.pose.tran.x;
	  numbers[1] = position.u.pose.tran.y;
	  numbers[2] = position.u.pose.tran.z;
	  (void) go_quat_rpy_convert(&position.u.pose.rot, &rpy);
	  /* don't convert them to degrees yet, this will happen
	     during the printing */
	  numbers[3] = rpy.r;
	  numbers[4] = rpy.p;
	  numbers[5] = rpy.y;
	} else if (GO_MOTION_UJOINT == go_motion_queue_get_type(&gmq) ||
		   GO_MOTION_JOINT == go_motion_queue_get_type(&gmq)) {
	  for (i = 0; i < MAX_NUMBERS; i++) {
	    numbers[i] = position.u.joint[i];
	  }
	}
	/* else some other type of queue */

	/* put the numbers on the interpolators */
	/* 
	   This is a little strange for GO_MOTION_WORLD. In the real
	   servo loops, interpolation is only done on the joint values
	   that come from the inverse kinematics.  In this test shell,
	   we are converting the motion queue rotation to RPY, and
	   interpolating these. The results are interesting but not
	   relevant to testing how interpolation of rotations really
	   happens at the joint level.
	*/
	for (i = 0; i < MAX_NUMBERS;  i++) {
	  (*go_interp_add)(&interp[i], numbers[i]);
	}
	for (i = 0, interp_s = 0.0;
	     i < interp_count;
	     i++, interp_s += interp_deltas) {
	  time += interp_deltat;
	  printf("%f ", (double) time);
	  for (ii = 0; ii < count; ii++) {
	    pos[ii] = (*go_interp_eval)(&interp[ii], interp_s);
	    vel[ii] = (pos[ii] - old_pos[ii]) * interp_deltat_inv;
	    old_pos[ii] = pos[ii];
	    acc[ii] = (vel[ii] - old_vel[ii]) * interp_deltat_inv;
	    old_vel[ii] = vel[ii];
	    jerk[ii] = (acc[ii] - old_acc[ii]) * interp_deltat_inv;
	    old_acc[ii] = acc[ii];
	    /* convert RPY to degrees for output */
	    if (GO_MOTION_WORLD == go_motion_queue_get_type(&gmq) &&
		ii > 2) {
	      pos[ii] = GO_TO_DEG(pos[ii]);
	      vel[ii] = GO_TO_DEG(vel[ii]);
	      acc[ii] = GO_TO_DEG(acc[ii]);
	    }
	    printf("%f ",
		   print_type == PRINT_POS ? (double) pos[ii] : 
		   print_type == PRINT_VEL ? (double) vel[ii] : 
		   print_type == PRINT_ACC ? (double) acc[ii] : 
		   (double) jerk[ii]);
	  }
	  printf("\n");
	}
	if (doruns) {
	  runs--;
	  if (runs <= 0) break;
	} else {
	  if (go_motion_queue_is_empty(&gmq)) break;
	} 
      }
    } else TRY("reset") {
      go_motion_queue_reset(&gmq);
    } else if (!strwcmp("scale", ptr)) {
      if (3 == sscanf(ptr, "%*s %lf %lf %lf", &d1, &d2, &d3)) {
	numbers[0] = d1, numbers[1] = d2, numbers[2] = d3;
	if (GO_RESULT_OK !=
	    go_motion_queue_set_scale(&gmq, numbers[0], numbers[1], numbers[2])) {
	  fprintf(stderr, "scale: bad parameter\n");
	}
	/* else OK */
      } else {
	fprintf(stderr, "scale: need <scale factor> <v> <a>\n");
      }
    } else if (!strwcmp("stop", ptr)) {
      go_motion_queue_stop(&gmq);
    } else if (!strwcmp("tpar", ptr)) {
      if (3 == sscanf(ptr, "%*s %lf %lf %lf", &d1, &d2, &d3)) {
	numbers[0] = d1, numbers[1] = d2, numbers[2] = d3;
	tvel = (go_real) numbers[0];
	tacc = (go_real) numbers[1];
	tjerk = (go_real) numbers[2];
	if (GO_RESULT_OK !=
	    go_motion_spec_set_tpar(&gms,
				    (go_real) numbers[0],
				    (go_real) numbers[1],
				    (go_real) numbers[2])) {
	  fprintf(stderr, "tpar: bad parameters\n");
	}
	/* else OK */
      } else {
	fprintf(stderr, "tpar: need <vel> <acc> <jerk>\n");
      }
    } else if (!strwcmp("rpar", ptr)) {
      if (3 == sscanf(ptr, "%*s %lf %lf %lf", &d1, &d2, &d3)) {
	numbers[0] = GO_TO_RAD(d1);
	numbers[1] = GO_TO_RAD(d2);
	numbers[2] = GO_TO_RAD(d3);
	rvel = (go_real) numbers[0];
	racc = (go_real) numbers[1];
	rjerk = (go_real) numbers[2];
	if (GO_RESULT_OK !=
	    go_motion_spec_set_rpar(&gms,
				    (go_real) numbers[0],
				    (go_real) numbers[1],
				    (go_real) numbers[2])) {
	  fprintf(stderr, "rpar: bad parameters\n");
	}
	/* else OK */
      } else {
	fprintf(stderr, "rpar: need <omega> <alpha> <angular jerk>\n");
      }
    } else if (!strwcmp("par", ptr)) {
      if (4 == sscanf(ptr, "%*s %d %lf %lf %lf", &i, &d1, &d2, &d3)) {
	numbers[0] = d1, numbers[1] = d2, numbers[2] = d3;
	if (GO_RESULT_OK !=
	    go_motion_spec_set_jpar(&gms, (go_integer) i - 1, (go_real) numbers[0], (go_real) numbers[1], (go_real) numbers[2])) {
	  fprintf(stderr, "par: bad parameters\n");
	}
	if (i < 1) i = 1;
	jvel[i-1] = (go_real) numbers[0];
	jacc[i-1] = (go_real) numbers[1];
	jjerk[i-1] = (go_real) numbers[2];
	/* else OK */
      } else {
	fprintf(stderr, "rpar: need <joint> <vel> <acc> <jerk>\n");
      }
    } else if (!strwcmp("pos", ptr)) {
      print_type = PRINT_POS;
    } else if (!strwcmp("vel", ptr)) {
      print_type = PRINT_VEL;
    } else if (!strwcmp("acc", ptr)) {
      print_type = PRINT_ACC;
    } else if (!strwcmp("jerk", ptr)) {
      print_type = PRINT_JERK;
    } else if (!strwcmp("constant", ptr)) {
      go_interp_add = go_interp_add_constant;
      go_interp_eval = go_interp_eval_constant;
    } else if (!strwcmp("linear", ptr)) {
      go_interp_add = go_interp_add_linear;
      go_interp_eval = go_interp_eval_linear;
    } else if (!strwcmp("cubic", ptr)) {
      go_interp_add = go_interp_add_cubic_pdv;
      go_interp_eval = go_interp_eval_cubic;
    } else if (!strwcmp("quintic", ptr)) {
      go_interp_add = go_interp_add_quintic_pdva;
      go_interp_eval = go_interp_eval_quintic;
    } else {
      fprintf(stderr, "? (%s)\n", ptr);
    }

#ifdef HAVE_READLINE_READLINE_H
    free(line);
#endif
  }

  return go_exit();
}
