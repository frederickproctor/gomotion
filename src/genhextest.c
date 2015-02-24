/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#include <stdio.h>
#include <stdlib.h>		/* malloc */
#include <sys/time.h>		/* struct timeval */
#include <unistd.h>		/* gettimeofday() */
#include "go.h"			/* go_pose */
#include "gokin.h"

static double timestamp()
{
  struct timeval tp;

  if (0 != gettimeofday(&tp, NULL)) {
    return 0.0;
  }
  return ((double) tp.tv_sec) + ((double) tp.tv_usec) / 1000000.0;
}

int main(int argc, char *argv[])
{
#define BUFFERLEN 256
  char buffer[BUFFERLEN];
  void * kins;
  int inverse = 1;
  int jacobian = 0;
  go_pose pos = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}};
  go_pose vel = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}};
  go_rpy rpy, vrpy;
  go_real joints[6] = { 0.0 };
  go_real jointvels[6] = { 0.0 };
  go_kin_fwd_flags iflags = 0;
  go_kin_inv_flags fflags = 0;
  int t;
  go_result retval;
#define ITERATIONS 100000
  double start, end;

  go_kin_select("genhexkins");

  if (NULL == (kins = malloc(go_kin_size())) ||
      0 != go_kin_init(kins)) {
    printf("can't get kinematics\n");
    return 1;
  }

  /* syntax is a.out {i|f # # # # # #} */
  if (argc == 8) {
    if (argv[1][0] == 'f') {
      /* joints passed, so do interations on forward kins for timing */
      for (t = 0; t < 6; t++) {
	if (1 != sscanf(argv[t + 2], "%lf", &joints[t])) {
	  fprintf(stderr, "bad value: %s\n", argv[t + 2]);
	  return 1;
	}
      }
      inverse = 0;
    } else if (argv[1][0] == 'i') {
      /* world coords passed, so do iterations on inverse kins for timing */
      if (1 != sscanf(argv[2], "%lf", &pos.tran.x)) {
	fprintf(stderr, "bad value: %s\n", argv[2]);
	return 1;
      }
      if (1 != sscanf(argv[3], "%lf", &pos.tran.y)) {
	fprintf(stderr, "bad value: %s\n", argv[3]);
	return 1;
      }
      if (1 != sscanf(argv[4], "%lf", &pos.tran.z)) {
	fprintf(stderr, "bad value: %s\n", argv[4]);
	return 1;
      }
      if (1 != sscanf(argv[5], "%lf", &rpy.r)) {
	fprintf(stderr, "bad value: %s\n", argv[5]);
	return 1;
      }
      if (1 != sscanf(argv[6], "%lf", &rpy.p)) {
	fprintf(stderr, "bad value: %s\n", argv[6]);
	return 1;
      }
      if (1 != sscanf(argv[7], "%lf", &rpy.y)) {
	fprintf(stderr, "bad value: %s\n", argv[7]);
	return 1;
      }
      go_rpy_quat_convert(&rpy, &pos.rot);
      inverse = 1;
    } else {
      fprintf(stderr, "syntax: %s {i|f # # # # # #}\n", argv[0]);
      return 1;
    }

    /* need an initial estimate for the forward kins, so ask for it */
    if (inverse == 0) {
      do {
	printf("initial estimate for Cartesian position, xyzrpw: ");
	fflush(stdout);
	if (NULL == fgets(buffer, BUFFERLEN, stdin)) {
	  return 0;
	}
      } while (6 != sscanf(buffer, "%lf %lf %lf %lf %lf %lf",
			   &pos.tran.x, &pos.tran.y, &pos.tran.z,
			   &rpy.r, &rpy.p, &rpy.y));
      go_rpy_quat_convert(&rpy, &pos.rot);
    }

    start = timestamp();
    for (t = 0; t < ITERATIONS; t++) {
      if (inverse) {
	retval = go_kin_inv(kins, &pos, joints, &iflags, &fflags);
	if (0 != retval) {
	  printf("inv kins error %d\n", retval);
	  break;
	}
      } else {
	retval = go_kin_fwd(kins, joints, &pos, &fflags, &iflags);
	if (0 != retval) {
	  printf("fwd kins error %d\n", retval);
	  break;
	}
      }
    }
    end = timestamp();

    printf("calculation time: %f secs\n",
	   (end - start) / ((double) ITERATIONS));
    return 0;
  }

  /* end of if args for timestamping */
  /* else we're interactive */
  while (!feof(stdin)) {
    if (inverse) {
      if (jacobian) {
	printf("jinv> ");
      } else {
	printf("inv> ");
      }
    } else {
      if (jacobian) {
	printf("jfwd> ");
      } else {
	printf("fwd> ");
      }
    }
    fflush(stdout);

    if (NULL == fgets(buffer, BUFFERLEN, stdin)) {
      break;
    }

    if (buffer[0] == 'i') {
      inverse = 1;
      continue;
    } else if (buffer[0] == 'f') {
      inverse = 0;
      continue;
    } else if (buffer[0] == 'j') {
      jacobian = !jacobian;
      continue;
    } else if (buffer[0] == 'q') {
      break;
    }

    if (inverse) {
      if (jacobian) {
	if (12 !=
	    sscanf(buffer, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
		   &pos.tran.x, &pos.tran.y, &pos.tran.z,
		   &rpy.r, &rpy.p, &rpy.y,
		   &vel.tran.x, &vel.tran.y, &vel.tran.z,
		   &vrpy.r, &vrpy.p, &vrpy.y)) {
	  printf("?\n");
	} else {
	  go_rpy_quat_convert(&rpy, &pos.rot);
	  go_rpy_quat_convert(&vrpy, &vel.rot);
	  retval = go_kin_jac_inv(kins, &pos, &vel, joints, jointvels);
	  printf("%f %f %f %f %f %f\n",
		 jointvels[0],
		 jointvels[1],
		 jointvels[2], jointvels[3], jointvels[4], jointvels[5]);
	  if (0 != retval) {
	    printf("inv Jacobian error %d\n", retval);
	  } else {
	    retval = go_kin_jac_fwd(kins, joints, jointvels, &pos, &vel);
	    go_quat_rpy_convert(&vel.rot, &vrpy);
	    printf("%f %f %f %f %f %f\n",
		   vel.tran.x, vel.tran.y, vel.tran.z,
		   vrpy.r, vrpy.p, vrpy.y);
	    if (0 != retval) {
	      printf("fwd kins error %d\n", retval);
	    }
	  }
	}
      } else {
	if (6 != sscanf(buffer, "%lf %lf %lf %lf %lf %lf",
			&pos.tran.x, &pos.tran.y, &pos.tran.z,
			&rpy.r, &rpy.p, &rpy.y)) {
	  printf("?\n");
	} else {
	  go_rpy_quat_convert(&rpy, &pos.rot);
	  retval = go_kin_inv(kins, &pos, joints, &iflags, &fflags);
	  printf("%f %f %f %f %f %f\n",
		 joints[0], joints[1], joints[2],
		 joints[3], joints[4], joints[5]);
	  if (0 != retval) {
	    printf("inv kins error %d\n", retval);
	  } else {
	    retval = go_kin_fwd(kins, joints, &pos, &fflags, &iflags);
	    go_quat_rpy_convert(&pos.rot, &rpy);
	    printf("%f %f %f %f %f %f\n",
		   pos.tran.x, pos.tran.y, pos.tran.z,
		   rpy.r, rpy.p, rpy.y);
	    if (0 != retval) {
	      printf("fwd kins error %d\n", retval);
	    }
	  }
	}
      }
    } else {
      if (jacobian) {
	if (12 !=
	    sscanf(buffer, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
		   &joints[0], &joints[1], &joints[2], &joints[3], &joints[4],
		   &joints[5], &jointvels[0], &jointvels[1], &jointvels[2],
		   &jointvels[3], &jointvels[4], &jointvels[5])) {
	  printf("?\n");
	} else {
	  retval = go_kin_jac_fwd(kins, joints, jointvels, &pos, &vel);
	  go_quat_rpy_convert(&vel.rot, &vrpy);
	  printf("%f %f %f %f %f %f\n",
		 vel.tran.x, vel.tran.y, vel.tran.z,
		 vrpy.r, vrpy.p, vrpy.y);
	  if (0 != retval) {
	    printf("fwd kins error %d\n", retval);
	  } else {
	    retval = go_kin_jac_inv(kins, &pos, &vel, joints, jointvels);
	    printf("%f %f %f %f %f %f\n",
		   jointvels[0], jointvels[1], jointvels[2],
		   jointvels[3], jointvels[4], jointvels[5]);
	    if (0 != retval) {
	      printf("inv kins error %d\n", retval);
	    }
	  }
	}
      } else {
	if (6 != sscanf(buffer, "%lf %lf %lf %lf %lf %lf",
			&joints[0], &joints[1], &joints[2],
			&joints[3], &joints[4], &joints[5])) {
	  printf("?\n");
	} else {
	  retval = go_kin_fwd(kins, joints, &pos, &fflags, &iflags);
	  go_quat_rpy_convert(&pos.rot, &rpy);
	  printf("%f %f %f %f %f %f\n",
		 pos.tran.x, pos.tran.y, pos.tran.z,
		 rpy.r, rpy.p, rpy.y);
	  if (0 != retval) {
	    printf("fwd kins error %d\n", retval);
	  } else {
	    retval = go_kin_inv(kins, &pos, joints, &iflags, &fflags);
	    printf("%f %f %f %f %f %f\n",
		   joints[0], joints[1], joints[2],
		   joints[3], joints[4], joints[5]);
	    if (0 != retval) {
	      printf("inv kins error %d\n", retval);
	    }
	  }
	}
      }
    }
  }

  return 0;
}
