#include <stdio.h>
#include <stdlib.h>
#include "go.h"

int main(int argc, char *argv[])
{
  go_cart *v1 = NULL;
  go_cart *v2 = NULL;
  go_cart *v1c = NULL;
  go_cart *v2c = NULL;
  enum {BUFFERSIZE = 256};
  char buffer[BUFFERSIZE];
  int num;
  go_pose pout;
  go_rpy rpy;

  for (num = 0; ;) {
    double d1, d2, d3, d4, d5, d6;
    if (NULL == fgets(buffer, sizeof(buffer), stdin)) break;
    if (6 == sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &d1, &d2, &d3, &d4, &d5, &d6)) {
      v1 = realloc(v1, (num+1) * sizeof(*v1));
      v2 = realloc(v2, (num+1) * sizeof(*v2));
      v1[num].x = d1, v1[num].y = d2, v1[num].z = d3,
	v2[num].x = d4, v2[num].y = d5, v2[num].z = d6;
      num++;
    }
    /* else ignore the line */
  }
  if (0 == num) return 0;

  v1c = malloc(num * sizeof(*v1c));
  v2c = malloc(num * sizeof(*v2c));

  if (GO_RESULT_OK !=
      go_cart_cart_pose(v1, v2, v1c, v2c, num, &pout)) {
    return 1;
  }

  go_quat_rpy_convert(&pout.rot, &rpy);

  printf("%f %f %f / %f %f %f\n",
	 (double) pout.tran.x, (double) pout.tran.y, (double) pout.tran.z,
	 (double) rpy.r, (double) rpy.p, (double) rpy.y);

  return 0;
}
