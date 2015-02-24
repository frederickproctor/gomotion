#include <stdio.h>
#include "go.h"

int main(int argc, char *argv[])
{
  enum { QUEUE_SIZE = 1 };	/* < 3 tests for good length handling */
  go_real time, deltat = 0.01;
  go_motion_spec gms, space[QUEUE_SIZE];
  go_motion_queue gmq;
  go_position position;
  go_integer i;

  if (0 != go_init() ||
      GO_RESULT_OK != go_motion_queue_init(&gmq, space, QUEUE_SIZE, deltat)) {
    return 1;
  }

  time = 0.0;

  if (argc == 1) {
    /* no arg means do world motion */
    if (GO_RESULT_OK != go_motion_queue_set_type(&gmq, GO_MOTION_WORLD)) {
      return 1;
    }

    go_motion_spec_init(&gms);
    go_motion_spec_set_type(&gms, GO_MOTION_LINEAR);
    go_motion_spec_set_id(&gms, 1);
    position.u.pose = go_pose_this(0,0,0,1,0,0,0);
    go_motion_spec_set_end_position(&gms, &position);
    go_motion_spec_set_tpar(&gms, 1,1,1);
    go_motion_spec_set_rpar(&gms, 1,1,1);

    position.u.pose.tran.x = 1.0;
    go_motion_spec_set_end_position(&gms, &position);
    if (GO_RESULT_OK != go_motion_queue_append(&gmq, &gms)) {
      fprintf(stderr, "can't append\n");
    }

    position.u.pose.tran.y = 2.0;
    go_motion_spec_set_end_position(&gms, &position);
    if (GO_RESULT_OK != go_motion_queue_append(&gmq, &gms)) {
      fprintf(stderr, "can't append\n");
    }

    position.u.pose.tran.x = 3.0;
    go_motion_spec_set_end_position(&gms, &position);
    if (GO_RESULT_OK != go_motion_queue_append(&gmq, &gms)) {
      fprintf(stderr, "can't append\n");
    }

    while (!go_motion_queue_is_empty(&gmq)) {
      go_motion_queue_interp(&gmq, &position);
      printf("%f %f %f\n", (double) time, (double) position.u.pose.tran.x, (double) position.u.pose.tran.y);
      time += deltat;
    }

    position.u.pose.tran.x = -1.0;
    position.u.pose.tran.y = -1.0;
    go_motion_spec_set_end_position(&gms, &position);
    if (GO_RESULT_OK != go_motion_queue_append(&gmq, &gms)) {
      fprintf(stderr, "can't append\n");
    }

    while (!go_motion_queue_is_empty(&gmq)) {
      go_motion_queue_interp(&gmq, &position);
      printf("%f %f %f\n", (double) time, (double) position.u.pose.tran.x, (double) position.u.pose.tran.y);
      time += deltat;
    }

  } else {
    /* any arg means do joint motion */
    /* no arg means do world motion */
    if (GO_RESULT_OK != go_motion_queue_set_type(&gmq, GO_MOTION_JOINT)) {
      return 1;
    }

    go_motion_spec_init(&gms);
    go_motion_spec_set_type(&gms, GO_MOTION_JOINT);
    go_motion_spec_set_id(&gms, 1);
    for (i = 0; i < GO_MOTION_JOINT_NUM; i++) {
      position.u.joint[i] = 0.0;
      go_motion_spec_set_end_position(&gms, &position);
      go_motion_spec_set_jpar(&gms, i, 1,1,1);
    }

    position.u.joint[0] = 1.0;
    go_motion_spec_set_end_position(&gms, &position);
    if (GO_RESULT_OK != go_motion_queue_append(&gmq, &gms)) {
      fprintf(stderr, "can't append\n");
    }

    position.u.joint[1] = 2.0;
    go_motion_spec_set_end_position(&gms, &position);
    if (GO_RESULT_OK != go_motion_queue_append(&gmq, &gms)) {
      fprintf(stderr, "can't append\n");
    }

    position.u.joint[1] = 3.0;
    go_motion_spec_set_end_position(&gms, &position);
    if (GO_RESULT_OK != go_motion_queue_append(&gmq, &gms)) {
      fprintf(stderr, "can't append\n");
    }

    while (!go_motion_queue_is_empty(&gmq)) {
      go_motion_queue_interp(&gmq, &position);
      printf("%f %f %f %f\n", (double) time, (double) position.u.joint[0], (double) position.u.joint[1], (double) position.u.joint[2]);
      time += deltat;
    }

    position.u.joint[0] = -1.0;
    position.u.joint[1] = -1.0;
    go_motion_spec_set_end_position(&gms, &position);
    if (GO_RESULT_OK != go_motion_queue_append(&gmq, &gms)) {
      fprintf(stderr, "can't append\n");
    }

    while (!go_motion_queue_is_empty(&gmq)) {
      go_motion_queue_interp(&gmq, &position);
      printf("%f %f %f %f\n", (double) time, (double) position.u.joint[0], (double) position.u.joint[1], (double) position.u.joint[2]);
      time += deltat;
    }
  }

  return go_exit();
}
