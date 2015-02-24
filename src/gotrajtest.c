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
  \file gotrajtest.c

  \brief Another motley bunch of test routines for verifying that
  gotraj.c works and for testing traj-related improvements.
*/

#include <stdio.h>
#include <math.h>
#include "go.h"

/*
  walkinmain moves from start pose to end pose given some max tran
  and rot vels, with each tran and rot going independently so that
  one likely reaches the end before the other. The intent is to go
  from start to end in the shortest time, without scaling for simultaneous
  arrival.

  Plot with:

  make && bin/gotrajtest  > out
  echo "plot 'out' using 1:2, '' using 1:3, '' using 1:4, '' using 1:5, '' using 1:6, '' using 1:7" | gnuplot -persist
 */
int walkinmain(int argc, char * argv[])
{
  go_pose start, end, cur, curinv, del;
  go_cart uvec;
  go_quat uquat;
  go_real tvel, rvel;
  go_real tincr, rincr, mag;
  go_real tdel, rdel;
  go_real time, delta_t;
  go_rpy rpy;

  start.tran.x = 1;
  start.tran.y = -2;
  start.tran.z = 3;
  rpy.r = GO_TO_RAD(30);
  rpy.p = GO_TO_RAD(-10);
  rpy.y = GO_TO_RAD(15);
  go_rpy_quat_convert(&rpy, &start.rot);

  end.tran.x = 2;
  end.tran.y = 3;
  end.tran.z = -1;
  rpy.r = GO_TO_RAD(0);
  rpy.p = GO_TO_RAD(-10);
  rpy.y = GO_TO_RAD(17);
  go_rpy_quat_convert(&rpy, &end.rot);

  tvel = 1;
  rvel = GO_TO_RAD(10);
  delta_t = 0.1;

  tincr = tvel * delta_t;
  rincr = rvel * delta_t;

  cur = start;

  for (time = 0; ; time += delta_t) {
    go_pose_inv(&cur, &curinv);
    go_pose_pose_mult(&curinv, &end, &del);

    if (GO_RESULT_OK != go_cart_unit(&del.tran, &uvec)) {
      del.tran.x = del.tran.y = del.tran.z = 0.0;
    } else {
      (void) go_cart_mag(&del.tran, &mag);
      if (mag > tincr) tdel = tincr;
      else tdel = mag;
      go_cart_scale_mult(&uvec, tdel, &del.tran);
    }

    if (GO_RESULT_OK != go_quat_unit(&del.rot, &uquat)) {
      del.rot.s = 1.0, del.rot.x = del.rot.y = del.rot.z = 0.0;
    } else {
      (void) go_quat_mag(&del.rot, &mag);
      if (mag > rincr) rdel = rincr;
      else rdel = mag;
      go_quat_scale_mult(&uquat, rdel, &del.rot);
    }

    (void) go_pose_pose_mult(&cur, &del, &cur);
    (void) go_quat_rpy_convert(&cur.rot, &rpy);

    printf("%f %f %f %f %f %f %f\n",
	   (double) time,
	   (double) cur.tran.x, (double) cur.tran.y, (double) cur.tran.z,
	   (double) GO_TO_DEG(rpy.r), (double) GO_TO_DEG(rpy.p), (double) GO_TO_DEG(rpy.y));

    if (go_pose_pose_compare(&cur, &end)) break;
  }

  return 0;
}

int main(int argc, char *argv[])
{
  if (go_init())
    return 1;

  return walkinmain(argc, argv);
}
