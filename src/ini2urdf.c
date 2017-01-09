/*!
  \file ini2urdf.c

  \brief Conversion utility from Go .ini file format to ROS URDF.
*/

#include <stdio.h>		/* fprintf, stderr */
#include <string.h>		/* strcmp, memset, memcpy */
#include <stddef.h>		/* sizeof */
#include <stdlib.h>		/* malloc, strtol, strtod */
#include <stdarg.h>		/* va_list */
#include <inifile.h>
#include <ulapi.h>		/* ulapi_time */
#include "go.h"	

typedef struct {
  go_pose pose;
  go_real min_limit;
  go_real max_limit;
  go_real max_vel;
} link_pose_struct;

/*
  Options:

  -i <inifile>   : use <inifile>
  -u <urdf file> : write <urdf file>
*/

int main(int argc, char *argv[])
{
  enum { BUFFERLEN = 80 };
  int option;
  char inifile_name[BUFFERLEN] = "";
  char urdf_name[BUFFERLEN] = "";
  FILE *inifp = NULL;
  FILE *urdfp = NULL;
  char *section = "";
  char *key = "";
  const char *ini_string = "";
  char name[BUFFERLEN] = "";
  char servo_name[BUFFERLEN] = "";
  go_real m_per_length_units, rad_per_angle_units;
  int servo_num = 0;
  go_link *link_params = NULL;
  link_pose_struct *link_poses = NULL;
  int saw_link_params = 0;
  int quantity;
  go_dh dh;
  go_pp pp;
  go_rpy rpy;
  double d1, d2, d3, d4, d5, d6;
  int t;

  /*
    These macros convert between Go's SI and user units.

    TGL and TGA mean "to Go length" and "to Go angle," respectively.
    Use these to convert from user units to Go units, e.g., meters =
    TGL(user), rads = TGA(user). TGL and TGA can be used anytime you
    know the quantity is length or angle.

    TGQ means "to Go quantity" and uses the 'quantity' variable
    to decide which of TGL or TGA to call.

    These refer to the variable 'm_per_length_units', 'rad_per_angle_units'
    and 'quantity', so they can only be called once these
    variables have been set. In the code below, they are set first;
    don't add any code before these.

    FGL,A,Q mean "from Go length, angle, quantity," respectively.
  */

#define TGL(x) ((x) * m_per_length_units)
#define TGA(x) ((x) * rad_per_angle_units)
#define TGQ(x)						\
  (quantity == GO_QUANTITY_LENGTH ? TGL(x) :	\
   quantity == GO_QUANTITY_ANGLE ? TGA(x) : (x))

#define FGL(x) ((x) / m_per_length_units)
#define FGA(x) ((x) / rad_per_angle_units)
#define FGQ(x)						\
  (quantity == GO_QUANTITY_LENGTH ? FGL(x) :	\
   quantity == GO_QUANTITY_ANGLE ? FGA(x) : (x))

  if (ULAPI_OK != ulapi_init()) {
    return 1;
  } 

  opterr = 0;
  while (1) {
    option = ulapi_getopt(argc, argv, ":i:u:");
    if (option == -1)
      break;

    switch (option) {
    case 'i':
      strncpy(inifile_name, ulapi_optarg, BUFFERLEN);
      inifile_name[BUFFERLEN - 1] = 0;
      break;

    case 'u':
      strncpy(urdf_name, ulapi_optarg, BUFFERLEN);
      urdf_name[BUFFERLEN - 1] = 0;
      break;

    case ':':
      fprintf(stderr, "missing value for -%c\n", ulapi_optopt);
      return 1;
      break;

    default:			/* '?' */
      fprintf (stderr, "unrecognized option -%c\n", ulapi_optopt);
      return 1;
      break;
    }
  }
  if (ulapi_optind < argc) {
    fprintf(stderr, "extra non-option characters: %s\n", argv[ulapi_optind]);
    return 1;
  }

  if (0 == *inifile_name) {
    inifp = stdin;
  } else {
    if (NULL == (inifp = fopen(inifile_name, "r"))) {
      fprintf(stderr, "can't open %s\n", inifile_name);
      return 1;
    }
  }

  if (0 == *urdf_name) {
    urdfp = stdout;
  } else {
    if (NULL == (urdfp = fopen(urdf_name, "w"))) {
      fprintf(stderr, "can't open %s\n", urdf_name);
      return 1;
    }
  }

  /* Do this first! Read units from ini file. */

  section = "GOMOTION";

  key = "NAME";
  ini_string = ini_find(inifp, key, section);
  if (NULL == ini_string) {
    fprintf(stderr, "not found: [%s] %s\n", section, key);
    return 1;
  } else {
    strncpy(name, ini_string, sizeof(name)-1);
    name[sizeof(name)-1] = 0;
  }

  key = "LENGTH_UNITS_PER_M";
  m_per_length_units = 1.0;
  ini_string = ini_find(inifp, key, section);
  if (NULL == ini_string) {
    fprintf(stderr, "not found: [%s] %s, using default %f\n", section, key, m_per_length_units);
  } else if (1 != sscanf(ini_string, "%lf", &d1)) {
    fprintf(stderr, "bad entry: [%s] %s = %s\n", section, key, ini_string);
    return 1;
  } else if (d1 <= 0.0) {
    fprintf(stderr, "invalid entry: [%s] %s = %s must be positive\n", section, key, ini_string);
    return 1;
  } else {
    m_per_length_units = (go_real) (1.0 / d1);
  }

  key = "ANGLE_UNITS_PER_RAD";
  rad_per_angle_units = 1.0;
  ini_string = ini_find(inifp, key, section);
  if (NULL == ini_string) {
    fprintf(stderr, "not found: [%s] %s, using default %f\n", section, key, rad_per_angle_units);
  } else if (1 != sscanf(ini_string, "%lf", &d1)) {
    fprintf(stderr, "bad entry: [%s] %s = %s\n", section, key, ini_string);
    return 1;
  } else if (d1 <= 0.0) {
    fprintf(stderr, "invalid entry: [%s] %s = %s must be positive\n", section, key, ini_string);
    return 1;
  } else {
    rad_per_angle_units = (go_real) (1.0 / d1);
  }

  section = servo_name;

  for (t = 0; ; t++) {
    ulapi_snprintf(servo_name, sizeof(servo_name), "SERVO_%d", t+1);
    servo_name[sizeof(servo_name)-1] = 0;

    if (! ini_has_section(inifp, section)) break;

    link_params = realloc(link_params, (t+1)*sizeof(*link_params));
    link_poses = realloc(link_poses, (t+1)*sizeof(*link_poses));

    key = "QUANTITY";
    ini_string = ini_find(inifp, key, section);
    if (NULL == ini_string) {
      fprintf(stderr, "not found: [%s] %s\n", section, key);
      return 1;
    }
    if (ini_match(ini_string, "LENGTH")) {
      quantity = GO_QUANTITY_LENGTH;
    } else if (ini_match(ini_string, "ANGLE")) {
      quantity = GO_QUANTITY_ANGLE;
    } else {
      fprintf(stderr, "bad entry: [%s] %s = %s\n", section, key, ini_string);
      return 1;
    }
    link_params[t].quantity = quantity;

    if (! saw_link_params) {
      key = "DH_PARAMETERS";
      ini_string = ini_find(inifp, key, section);
      if (NULL == ini_string) {
	/* ignore for now, we may have other params */
      } else {
	if (4 == sscanf(ini_string, "%lf %lf %lf %lf", &d1, &d2, &d3, &d4)) {
	  dh.a = TGL(d1);
	  dh.alpha = TGA(d2);
	  dh.d = TGL(d3);
	  dh.theta = TGA(d4);
	  link_params[t].u.dh = dh;
	  link_params[t].type = GO_LINK_DH;
	  go_dh_pose_convert(&link_params[t].u.dh, &link_poses[t].pose);
	} else {
	  fprintf(stderr, "bad entry: [%s] %s = %s\n", section, key, ini_string);
	  return 1;
	}
	saw_link_params = 1;
      }
    }

    if (! saw_link_params) {
      key = "PP_PARAMETERS";
      ini_string = ini_find(inifp, key, section);
      if (NULL == ini_string) {
	/* ignore for now, we may have other params */
      } else {
	if (6 == sscanf(ini_string, "%lf %lf %lf %lf %lf %lf", &d1, &d2, &d3, &d4, &d5, &d6)) {
	  pp.pose.tran.x = TGL(d1);
	  pp.pose.tran.y = TGL(d2);
	  pp.pose.tran.z = TGL(d3);
	  rpy.r = TGA(d4);
	  rpy.p = TGA(d5);
	  rpy.y = TGA(d6);
	  go_rpy_quat_convert(&rpy, &pp.pose.rot);
	  link_params[t].u.pp = pp;
	  link_params[t].type = GO_LINK_PP;
	  link_poses[t].pose = link_params[t].u.pp.pose;
	} else {
	  fprintf(stderr, "bad entry: [%s] %s = %s\n", section, key, ini_string);
	  return 1;
	}
	saw_link_params = 1;
      }
    }

    key = "MIN_LIMIT";
    d1 = 0;
    ini_string = ini_find(inifp, key, section);
    if (NULL == ini_string) {
      fprintf(stderr, "not found: [%s] %s, using default %f\n", section, key, d1);
    } else {
      if (1 == sscanf(ini_string, "%lf", &d1)) {
	link_poses[t].min_limit = TGQ(d1);
      } else {
	fprintf(stderr, "bad entry: [%s] %s = %s\n", section, key, ini_string);
	return 1;
      }
    }

    key = "MAX_LIMIT";
    d1 = 0;
    ini_string = ini_find(inifp, key, section);
    if (NULL == ini_string) {
      fprintf(stderr, "not found: [%s] %s, using default %f\n", section, key, d1);
    } else {
      if (1 == sscanf(ini_string, "%lf", &d1)) {
	link_poses[t].max_limit = TGQ(d1);
      } else {
	fprintf(stderr, "bad entry: [%s] %s = %s\n", section, key, ini_string);
	return 1;
      }
    }

    key = "MAX_VEL";
    d1 = 1;
    ini_string = ini_find(inifp, key, section);
    if (NULL == ini_string) {
      fprintf(stderr, "not found: [%s] %s, using default %f\n", section, key, d1);
    } else {
      if (1 == sscanf(ini_string, "%lf", &d1)) {
	link_poses[t].max_vel = TGQ(d1);
      } else {
	fprintf(stderr, "bad entry: [%s] %s = %s\n", section, key, ini_string);
	return 1;
      }
    }

    if (! saw_link_params) {
      fprintf(stderr, "not found: [%s] DH- or PP_PARAMETERS\n", section);
      return 1;
    }

    servo_num++;
    saw_link_params = 0;
  } /* for (servos) */

  fprintf(urdfp, "<robot name=\"%s\">\n", name);
  fprintf(urdfp, "<link name=\"link_0\">\n</link>\n");
  for (t = 0; t < servo_num; t++) {
    fprintf(urdfp, "<link name=\"link_%d\">\n</link>\n", t+1);

    fprintf(urdfp, "<joint name=\"joint_%d\" type=\"%s\">\n", t+1, link_params[t].quantity == GO_QUANTITY_LENGTH ? "prismatic" : link_params[t].quantity == GO_QUANTITY_ANGLE ? "revolute" : "floating");

    go_quat_rpy_convert(&link_poses[t].pose.rot, &rpy);
    fprintf(urdfp, "\t<origin xyz=\"%f %f %f\" rpy=\"%f %f %f\"/>\n", link_poses[t].pose.tran.x, link_poses[t].pose.tran.y, link_poses[t].pose.tran.z,
	   rpy.r, rpy.p, rpy.y);
    fprintf(urdfp, "\t<parent link=\"link_%d\"/>\n", t);
    fprintf(urdfp, "\t<child link=\"link_%d\"/>\n", t+1);
    fprintf(urdfp, "\t<axis xyz=\"0 0 1\"/>\n");
    fprintf(urdfp, "\t<limit lower=\"%f\" upper=\"%f\" effort=\"%f\" velocity=\"%f\"/>\n", link_poses[t].min_limit, link_poses[t].max_limit, 1.0, link_poses[t].max_vel);
    
    fprintf(urdfp, "</joint>\n");
  }
  fprintf(urdfp, "</robot>\n");

  return 0;
}
