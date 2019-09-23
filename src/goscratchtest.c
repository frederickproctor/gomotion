#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "go.h"

int main(int argc, char *argv[])
{
  go_mat mat;
  go_rpy rpy;

  mat.x.x = 0, mat.y.x = 0, mat.z.x = 1;
  mat.x.y = 1, mat.y.y = 0, mat.z.y = 0;
  mat.x.z = 0, mat.y.z = 1, mat.z.z = 0;

  go_mat_rpy_convert(&mat, &rpy);

  printf("%f %f %f\n", GO_TO_DEG(rpy.r), GO_TO_DEG(rpy.p), GO_TO_DEG(rpy.y));

  return 0;
}
