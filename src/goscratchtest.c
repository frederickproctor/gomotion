#include <stdio.h>
#include <math.h>
#include "go.h"

/*
  Given two points in one coordinate system, transform them into
  another (random) one, then figure out what that transform is by
  looking at the points. 
*/

int main(int argc, char *argv[])
{
  go_cart a1, a2, b1, b2;
  go_cart ac, bc;
  go_zyx zyx, zyxout;
  go_pose p, pout;
  go_real tha, thb, theta;
  go_integer t;

  for (t = 0; t < 10; t++) {
    a1.x = go_random();
    a1.y = go_random();
    a1.z = 0;

    a2.x = go_random();
    a2.y = go_random();
    a2.z = 0;

    p.tran.x = go_random();
    p.tran.y = go_random();
    p.tran.z = 0;

    zyx.z = GO_PI * (2.0 * go_random() - 1.0);
    zyx.y = 0;
    zyx.x = 0;
    go_zyx_quat_convert(&zyx, &p.rot);

    /* p = B_T_A, from A to B */

    go_pose_cart_mult(&p, &a1, &b1);
    go_pose_cart_mult(&p, &a2, &b2);

    tha = atan2(a2.y - a1.y, a2.x - a1.x);
    thb = atan2(b2.y - b1.y, b2.x - b1.x);
    theta = thb - tha;
    while (theta > GO_PI) theta -= GO_2_PI;
    while (theta < -GO_PI) theta += GO_2_PI;

    /* theta = B_theta_A, from A to B */

    printf("%f %f\n", zyx.z, theta);

    zyxout.z = theta;
    zyxout.y = 0;
    zyxout.x = 0;
    go_zyx_quat_convert(&zyxout, &pout.rot);
    /* pout.rot = B_R_A, from A to B */

    ac.x = (a1.x + a2.x) * 0.5;
    ac.y = (a1.y + a2.y) * 0.5;
    ac.z = 0;

    bc.x = (b1.x + b2.x) * 0.5;
    bc.y = (b1.y + b2.y) * 0.5;
    bc.z = 0;
    go_quat_cart_mult(&pout.rot, &ac, &ac);
    go_cart_cart_sub(&bc, &ac, &pout.tran);

    /* pout.tran = B_P_A, from A to B */

    printf("%f %f / %f %f\n", p.tran.x, p.tran.y, pout.tran.x, pout.tran.y);
  }

  return 0;
}
