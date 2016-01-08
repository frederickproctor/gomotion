#include <stdio.h>
#include <math.h>
#include "go.h"

int main(int argc, char *argv[])
{
  go_mat mat;
  go_quat quat = {0, 1, 0, 0};

  go_quat_mat_convert(&quat, &mat);

  go_mat_print(&mat);

  return 0;
}

int main_1(int argc, char *argv[])
{
  double d1, d2;
  go_real th4in, th5in;
  go_cart zin;
  double root22;
  double check;
  go_real th4p, th4n, th4out;
  go_real th5p, th5n, th5out;
  go_real s4, c4, s5, c5;
  go_flag th4neg;

  if (argc < 3) {
    fprintf(stderr, "need angles 4 and 5\n");
    return 1;
  }
  if (1 != sscanf(argv[1], "%lf", &d1)) {
    fprintf(stderr, "bad value for angle 4: %s\n", argv[1]);
    return 1;
  }
  if (1 != sscanf(argv[2], "%lf", &d2)) {
    fprintf(stderr, "bad value for angle 5: %s\n", argv[2]);
    return 1;
  }
  
  if (0 != go_init()) return 1;

  th4in = d1, th5in = d2;
  th4neg = th4in < 0 ? 1 : 0;
  root22 = 0.5 * sqrt(2.0);

  go_sincos(th4in, &s4, &c4);
  go_sincos(th5in, &s5, &c5);

  /* represent Z in {0} as Z in {5} at these angles, call it 'zin'; this
     would be the input to the inverse kinematics */
  zin.x = root22*(s4*c5 + root22*c4*s5) - 0.5*s5;
  zin.y = root22*(-s4*s5 + root22*c4*c5) - 0.5*c5;
  zin.z = 0.5*(1 + c4);
  /* should already be a unit vector, but we'll force it */
  if (GO_RESULT_OK != go_cart_unit(&zin, &zin)) {
    fprintf(stderr, "can't normalize the resulting Z vector\n");
    return 1;
  }
  printf("Z in {5} is %.10f, %.10f, %.10f\n", (double) zin.x, (double) zin.y, (double) zin.z);

  /* run the inverse calculations on this 'zin' and see if we get
     the original two angles */

  /* use simple formula for angle 4,
    zin.z = 1/2 + 1/2 cos th4,
    to get two possible values */
  if (GO_RESULT_OK != go_acoses(2*zin.z-1, &th4p, &th4n)) {
    printf("can't get acoses\n");
    return 1;
  }
  printf("angles for 4 are %f, %f\n", (double) th4p, (double) th4n);

  if (GO_ROT_SMALL(th4p)) {
    /* {5} aligned with {0}, full degenerate case */
    th4out = th4p;
    th5out = th5in;		/* use old value */
    printf("angle for 5 is %f\n", (double) th5out);
    printf("1 picked %f, %f\n", th4out, th5out);
    return 0;
  }

  if (GO_TRAN_SMALL(zin.x)) {
    /* plane alignment, so some degeneracy */

    /* these trigger this: */
    /*  0.8 1.2802939777 */
    /* 0.8 -1.8612986758 */
    /* -0.8 -1.2802939777 */
    /* -0.8 1.8612986758 */

    if (th4neg) th4out = th4n;
    else th4out = th4p;
    go_sincos(th4out, &s4, &c4);
    if (GO_RESULT_OK != go_atans(-root22*s4/(0.5*(c4-1)), &th5p, &th5n)) {
      printf("can't get atanses\n");
      return 1;
    }
    printf("angles for 5 are %f, %f\n", (double) th5p, (double) th5n);

    /* check positive angle 5 against z.y and see if it matches */
    th5out = th5p;
    go_sincos(th5out, &s5, &c5);
    check = root22*(-s4*s5 + root22*c4*c5)-0.5*c5;
    if (GO_TRAN_CLOSE(check, zin.y)) {
      printf("2 picked %f, %f\n", th4out, th5out);
      return 0;
    }
    /* else check negative angle 5 against z.y and see if it matches */
    th5out = th5n;
    go_sincos(th5out, &s5, &c5);
    check = root22*(-s4*s5 + root22*c4*c5)-0.5*c5;
    if (GO_TRAN_CLOSE(check, zin.y)) {
      printf("3 picked %f, %f\n", th4out, th5out);
      return 0;
    }
    /* else no match */
    printf("X picked\n");	/* no solution */
    return 1;
  }

  /* zin.x is not small, so no plane alignment */

  /* use flag to pick one of the solutions, since many will work */
  if (th4neg) th4out = th4n;
  else th4out = th4p;

  go_sincos(th4out, &s4, &c4);
  if (GO_RESULT_OK != go_linear_cos_sin_solve(root22*s4/zin.x, 0.5*(c4-1)/zin.x, &th5p, &th5n)) {
    printf("can't get linear cos sin solution\n");
    return 1;
  }
  printf("angles for 5 are %f, %f\n", (double) th5p, (double) th5n);

  /* check positive angle 5 against z.y and see if it matches */
  th5out = th5p;
  go_sincos(th5out, &s5, &c5);
  check = root22*(-s4*s5 + root22*c4*c5)-0.5*c5;
  if (GO_TRAN_CLOSE(check, zin.y)) {
    printf("4 picked %f, %f\n", th4out, th5out);

  zin.x = root22*(s4*c5 + root22*c4*s5) - 0.5*s5;
  zin.y = root22*(-s4*s5 + root22*c4*c5) - 0.5*c5;
  zin.z = 0.5*(1 + c4);

  printf("Z in {5} is %.10f, %.10f, %.10f\n", (double) zin.x, (double) zin.y, (double) zin.z);

    return 0;
  }
  /* else check negative angle 5 against z.y and see if it matches */
  th5out = th5n;
  go_sincos(th5out, &s5, &c5);
  check = root22*(-s4*s5 + root22*c4*c5)-0.5*c5;
  if (GO_TRAN_CLOSE(check, zin.y)) {
    printf("5 picked %f, %f\n", th4out, th5out);
    return 0;
  }

  /* no solution */
  printf("X picked\n");

  return 1;
}
