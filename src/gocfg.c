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
  \file gocfg.c

  \brief Run-time configuration of a Go Motion controller via .ini files
*/

/*!
  \defgroup INICFG Configuration Using .INI Files

  The \c gorun script configures a running Go Main.
*/

#include <stdio.h>		/* fprintf, stderr */
#include <string.h>		/* strcmp, memset, memcpy */
#include <stddef.h>		/* sizeof */
#include <stdlib.h>		/* malloc, strtol, strtod */
#include <stdarg.h>		/* va_list */
#include <inifile.h>
#include <ulapi.h>		/* ulapi_time */
#include "go.h"	
#include "gorcsutil.h"
#include "servointf.h"
#include "trajintf.h"
#include "toolintf.h"
#include "taskintf.h"
#include "pid.h"

int scan_em(double *array, char *string, int len)
{
  char *nptr = string;
  char *endptr;
  int i;

  for (i = 0; i < len; i++) {
    array[i] = strtod(nptr, &endptr);
    if (nptr == endptr)
      break;			/* nothing was converted */
    nptr = endptr;
  }

  return i;
}

/*
  'dbprintf' is debug printf that shows what's going on during init
  and shutdown. Fatal errors are printed regardless.
*/
static int dbflag = 0;
static void dbprintf(int prefix, const char * fmt, ...)
{
  va_list ap;
  FILE *dst = stdout;

  if (dbflag) {
    if (prefix) {
      fprintf(dst, "gocfg: ");
    }
    va_start(ap, fmt);
    vfprintf(dst, fmt, ap);
    fflush(dst);
    va_end(ap);
  }
}

 /*
   Options:

   -i <inifile>   : use <inifile>
   -d             : print debug messages
   -t <wait time> : set the wait timeout, in seconds
   -s <section>   : only do <section>
 */

 int main(int argc, char *argv[])
 {
   enum { BUFFERLEN = 80 };
   int option;
   char inifile_name[BUFFERLEN] = "gomotion.ini";
   int TASK_SHM_KEY = 0;
   int TOOL_SHM_KEY = 0;
   int TRAJ_SHM_KEY = 0;
   int SERVO_SHM_KEY = 0;
   int SERVO_HOWMANY = 0;
   FILE *fp = NULL;
   enum { MAX_INI_ENTRIES = 100 };
   INIFILE_ENTRY ini_entries[MAX_INI_ENTRIES];
   char section[INIFILE_MAX_LINELEN];
   const char *ini_string;
   int entry, num_entries;

   servo_comm_struct *servo_comm_ptr;	/* this is an array, must use []. */
   servo_cfg_struct servo_cfg[SERVO_NUM];
   servo_set_struct pp_servo_set_struct[2][SERVO_NUM],
     *servo_set_ptr[SERVO_NUM], *servo_set_test[SERVO_NUM];
   void * servo_shm = NULL;

   traj_comm_struct *traj_comm_ptr;
   traj_cfg_struct traj_cfg;
   traj_set_struct pp_traj_set_struct[2], *traj_set_ptr, *traj_set_test;
   void * traj_shm = NULL;

   tool_comm_struct *tool_comm_ptr;
   tool_cfg_struct tool_cfg;
   tool_set_struct pp_tool_set_struct[2], *tool_set_ptr, *tool_set_test;
   void * tool_shm = NULL;

   task_comm_struct *task_comm_ptr;
   task_cfg_struct task_cfg;
   task_set_struct pp_task_set_struct[2], *task_set_ptr, *task_set_test;
   void * task_shm = NULL;

   go_real m_per_length_units, rad_per_angle_units;
   int i1;
   double d1, d2, d3, d4, d5, d6, d7, d8, d9;
   double darray[16];
   int servo_num;
   int got_it;
   double wait_time = 3.0;
   double end;
   go_cart tran;
   go_rpy rpy;
   int servo_type;
   int debug;
   int joint_quantity;
   double cycle_time;
   go_body body;
   go_pose home_pose, tool_transform;
   go_pose min_limit_pose, max_limit_pose;
   double home;
   double input_scale;
   double output_scale;
   double min_limit, max_limit;
   double max_vel = 1.0, max_acc = 1.0, max_jerk = 1.0;
   double max_tvel = 1.0, max_tacc = 1.0, max_tjerk = 1.0;
   double max_rvel = 1.0, max_racc = 1.0, max_rjerk = 1.0;
   double max_scale = 1.0, max_scale_v = 1.0, max_scale_a = 1.0;
   pid_struct pid;
   servo_cfg_parameters parameters;
   int strict = 0;
   char prog_dir[TASK_CMD_PROGRAM_LEN] = "";

   go_link link_params[SERVO_NUM];

   int saw_joint_quantity;
   int saw_active;
   int saw_servo_type;
   int saw_debug;
   int saw_cycle_time;
   int saw_home;
   int saw_tool_transform;
   int saw_input_scale;
   int saw_output_scale;
   int saw_limit;
   int saw_profile;
   int saw_scale;
   int saw_pid;
   int saw_parameters;
   int saw_link_params;
   int saw_strict;
   int saw_prog_dir;

   int joint, joint_num;
   int retval = 0;
   int t;

   /*
     These macros convert between Go's SI and user units.

     TGL and TGA mean "to Go length" and "to Go angle," respectively.
     Use these to convert from user units to Go units, e.g., meters =
     TGL(user), rads = TGA(user). TGL and TGA can be used anytime you
     know the quantity is length or angle.

     TGQ means "to Go quantity" and uses the 'joint_quantity' variable
     to decide which of TGL or TGA to call.

     These refer to the variable 'm_per_length_units', 'rad_per_angle_units'
     and 'joint_quantity', so they can only be called once these
     variables have been set. In the code below, they are set first;
     don't add any code before these.

     FGL,A,Q mean "from Go length, angle, quantity," respectively.
   */

 #define TGL(x) ((x) * m_per_length_units)
 #define TGA(x) ((x) * rad_per_angle_units)
 #define TGQ(x)					   \
   (joint_quantity == GO_QUANTITY_LENGTH ? TGL(x) : \
    joint_quantity == GO_QUANTITY_ANGLE ? TGA(x) : (x))

 #define FGL(x) ((x) / m_per_length_units)
 #define FGA(x) ((x) / rad_per_angle_units)
 #define FGQ(x)					   \
   (joint_quantity == GO_QUANTITY_LENGTH ? FGL(x) : \
    joint_quantity == GO_QUANTITY_ANGLE ? FGA(x) : (x))

 #define RETURN(x) retval = (x); goto CLOSE

   dbflag = 0;

   opterr = 0;
   while (1) {
     option = ulapi_getopt(argc, argv, ":i:u:s:c:d");
     if (option == -1)
       break;

     switch (option) {
     case 'i':
       strncpy(inifile_name, ulapi_optarg, BUFFERLEN);
       inifile_name[BUFFERLEN - 1] = 0;
       break;

     case 't':
       wait_time = strtod(ulapi_optarg, NULL);
       break;

     case 'd':
       dbflag = 1;
       break;

     case ':':
       fprintf(stderr, "gocfg: missing value for -%c\n", ulapi_optopt);
       return 1;
       break;

     default:			/* '?' */
       fprintf (stderr, "unrecognized option -%c\n", ulapi_optopt);
       return 1;
       break;
     }
   }
   if (ulapi_optind < argc) {
     fprintf(stderr, "gocfg: extra non-option characters: %s\n", argv[ulapi_optind]);
     return 1;
   }

   if (ULAPI_OK != ulapi_init()) {
     return 1;
   } 

   if (go_init()) {
     fprintf(stderr, "gocfg: go_init error\n");
     return 1;
   }

   if (NULL == (fp = fopen(inifile_name, "r"))) {
     fprintf(stderr, "gocfg: can't open %s\n", inifile_name);
     RETURN(1);
   }

   /* Do this first! Read units from ini file. */

   m_per_length_units = 1.0;
   ini_string = ini_find(fp, "LENGTH_UNITS_PER_M", "GOMOTION");
   if (NULL == ini_string) {
     fprintf(stderr, "gocfg: [GOMOTION] LENGTH_UNITS_PER_M not found, using 1\n");
   } else if (1 != sscanf(ini_string, "%lf", &d1)) {
     fprintf(stderr, "gocfg: bad entry: [GOMOTION] LENGTH_UNITS_PER_M = %s\n", ini_string);
     RETURN(1);
   } else if (d1 <= 0.0) {
     fprintf(stderr, "gocfg: invalid entry: [GOMOTION] LENGTH_UNITS_PER_M = %s must be positive\n", ini_string);
     RETURN(1);
   } else {
     m_per_length_units = (go_real) (1.0 / d1);
   }
   rad_per_angle_units = 1.0;
   ini_string = ini_find(fp, "ANGLE_UNITS_PER_RAD", "GOMOTION");
   if (NULL == ini_string) {
     fprintf(stderr, "gocfg: [GOMOTION] ANGLE_UNITS_PER_RAD not found, using 1\n");
   } else if (1 != sscanf(ini_string, "%lf", &d1)) {
     fprintf(stderr, "gocfg: bad entry: [GOMOTION] ANGLE_UNITS_PER_RAD = %s\n", ini_string);
     RETURN(1);
   } else if (d1 <= 0.0) {
     fprintf(stderr, "gocfg: invalid entry: [GOMOTION] ANGLE_UNITS_PER_RAD = %s must be positive\n", ini_string);
     RETURN(1);
   } else {
     rad_per_angle_units = (go_real) (1.0 / d1);
   }

   /* now the TGL,A macros will work */

   /* read comm params from ini file */

   ini_string = ini_find(fp, "SHM_KEY", "TASK");
   if (NULL == ini_string) {
     /* optional */
     TASK_SHM_KEY = 0;
   } else if (1 != sscanf(ini_string, "%i", &TASK_SHM_KEY)) {
     fprintf(stderr, "gocfg: bad entry: [TASK] SHM_KEY = %s\n", ini_string);
     RETURN(1);
   }

   ini_string = ini_find(fp, "SHM_KEY", "TOOL");
   if (NULL == ini_string) {
     /* optional */
     TOOL_SHM_KEY = 0;
   } else if (1 != sscanf(ini_string, "%i", &TOOL_SHM_KEY)) {
     fprintf(stderr, "gocfg: bad entry: [TOOL] SHM_KEY = %s\n", ini_string);
     RETURN(1);
   }

   ini_string = ini_find(fp, "SHM_KEY", "TRAJ");
   if (NULL == ini_string) {
     fprintf(stderr, "gocfg: [TRAJ] SHM_KEY not found\n");
     RETURN(1);
   } else if (1 != sscanf(ini_string, "%i", &TRAJ_SHM_KEY)) {
     fprintf(stderr, "gocfg: bad entry: [TRAJ] SHM_KEY = %s\n", ini_string);
     RETURN(1);
   }

   ini_string = ini_find(fp, "HOWMANY", "SERVO");
   if (NULL == ini_string) {
     /* can't find HOWMANY, so read SERVO_# sections to the last */
     for (servo_num = 0; servo_num < SERVO_NUM; servo_num++) {
       sprintf(section, "SERVO_%d", servo_num + 1);
       if (NULL == ini_find(fp, "CYCLE_TIME", section)) break;
     }
     SERVO_HOWMANY = servo_num;
     fprintf(stderr, "gocfg: [SERVO] HOWMANY not found, using %d by counting\n", SERVO_HOWMANY);
   } else if (1 != sscanf(ini_string, "%i", &SERVO_HOWMANY)) {
     fprintf(stderr, "gocfg: bad entry: [SERVO] HOWMANY = %s\n", ini_string);
     RETURN(1);
   }

   ini_string = ini_find(fp, "SHM_KEY", "SERVO");
   if (NULL == ini_string) {
     fprintf(stderr, "gocfg: [SERVO] SHM_KEY not found\n");
     RETURN(1);
   } else if (1 != sscanf(ini_string, "%i", &SERVO_SHM_KEY)) {
     fprintf(stderr, "gocfg: bad entry: [SERVO] SHM_KEY = %s\n", ini_string);
     RETURN(1);
   }

   /* get servo shared memory buffers */
   servo_shm = ulapi_rtm_new(SERVO_SHM_KEY, SERVO_NUM * sizeof(servo_comm_struct));
   if (NULL == servo_shm) {
     fprintf(stderr, "gocfg: can't get servo comm shm\n");
     RETURN(1);
   }
   servo_comm_ptr = ulapi_rtm_addr(servo_shm);
   for (servo_num = 0; servo_num < SERVO_HOWMANY; servo_num++) {
     /* set up ping-pong buffers */
     servo_set_ptr[servo_num] = &pp_servo_set_struct[0][servo_num];
     servo_set_test[servo_num] = &pp_servo_set_struct[1][servo_num];
     /* compose a new NOP command to test for aliveness */
     servo_cfg[servo_num].type = SERVO_CFG_NOP_TYPE;
     servo_cfg[servo_num].serial_number = 1;
     servo_cfg[servo_num].tail = servo_cfg[servo_num].head = 1;
     /* loop and check that it got it */
     for (got_it = 0, end = ulapi_time() + wait_time;
	  ulapi_time() < end;
	  ulapi_sleep(0.001)) {
       /* rewrite it */
       servo_comm_ptr[servo_num].servo_cfg = servo_cfg[servo_num];
       /* reread it */
       *servo_set_ptr[servo_num] = servo_comm_ptr[servo_num].servo_set;
       /* and check it */
       if (servo_set_ptr[servo_num]->head == servo_set_ptr[servo_num]->tail &&
	   servo_set_ptr[servo_num]->type == SERVO_SET_TYPE &&
	   servo_set_ptr[servo_num]->echo_serial_number == servo_cfg[servo_num].serial_number) {
	 got_it = 1;
	 break;
       }
     }
     if (! got_it) {
       fprintf(stderr, "gocfg: timed out connecting to servo %d\n", servo_num);
       RETURN(1);
     }
   }

   /* get traj shared memory buffers */
   traj_shm = ulapi_rtm_new(TRAJ_SHM_KEY, sizeof(traj_comm_struct));
   if (NULL == traj_shm) {
     fprintf(stderr, "gocfg: can't get traj comm shm\n");
     RETURN(1);
   }
   traj_comm_ptr = ulapi_rtm_addr(traj_shm);
   /* set up ping-pong buffers */
   traj_set_ptr = &pp_traj_set_struct[0];
   traj_set_test = &pp_traj_set_struct[1];
   /* compose a new NOP command to test for aliveness */
   traj_cfg.type = TRAJ_CFG_NOP_TYPE;
   traj_cfg.serial_number = 1;
   traj_cfg.tail = traj_cfg.head = 1;
   /* loop and check that it got it */
   for (got_it = 0, end = ulapi_time() + wait_time;
	ulapi_time() < end;
	ulapi_sleep(0.001)) {
     /* rewrite it */
     traj_comm_ptr->traj_cfg = traj_cfg;
     /* reread it */
     *traj_set_ptr = traj_comm_ptr->traj_set;
     /* and check it */
     if (traj_set_ptr->head == traj_set_ptr->tail &&
	 traj_set_ptr->type == TRAJ_SET_TYPE &&
	 traj_set_ptr->echo_serial_number == traj_cfg.serial_number) {
       got_it = 1;
       break;
     }
   }
   if (! got_it) {
     fprintf(stderr, "gocfg: timed out connecting to traj\n");
     RETURN(1);
   }
   traj_cfg.serial_number = traj_set_ptr->echo_serial_number + 1;

   /* get tool shared memory buffers if called for */
   if (TOOL_SHM_KEY != 0) {
     tool_shm = ulapi_rtm_new(TOOL_SHM_KEY, sizeof(tool_comm_struct));
     if (NULL == tool_shm) {
       fprintf(stderr, "gocfg: can't get tool comm shm\n");
       RETURN(1);
     }
     tool_comm_ptr = ulapi_rtm_addr(tool_shm);
     /* set up ping-pong buffers */
     tool_set_ptr = &pp_tool_set_struct[0];
     tool_set_test = &pp_tool_set_struct[1];
     /* compose a new NOP command to test for aliveness */
     tool_cfg.type = TOOL_CFG_NOP_TYPE;
     tool_cfg.serial_number = 1;
     tool_cfg.tail = tool_cfg.head = 1;
     /* loop and check that it got it */
     for (got_it = 0, end = ulapi_time() + wait_time;
	  ulapi_time() < end;
	  ulapi_sleep(0.001)) {
       /* rewrite it */
       tool_comm_ptr->tool_cfg = tool_cfg;
       /* reread it */
       *tool_set_ptr = tool_comm_ptr->tool_set;
       /* and check it */
       if (tool_set_ptr->head == tool_set_ptr->tail &&
	   tool_set_ptr->type == TOOL_SET_TYPE &&
	   tool_set_ptr->echo_serial_number == tool_cfg.serial_number) {
	 got_it = 1;
	 break;
       }
     }
     if (! got_it) {
       fprintf(stderr, "gocfg: timed out connecting to tool\n");
       RETURN(1);
     }
     tool_cfg.serial_number = tool_set_ptr->echo_serial_number + 1;
   }

   /* get task shared memory buffers if called for */
   if (TASK_SHM_KEY != 0) {
     task_shm = ulapi_rtm_new(TASK_SHM_KEY, sizeof(task_comm_struct));
     if (NULL == task_shm) {
       fprintf(stderr, "gocfg: can't get task comm shm\n");
       RETURN(1);
     }
     task_comm_ptr = ulapi_rtm_addr(task_shm);
     /* set up ping-pong buffers */
     task_set_ptr = &pp_task_set_struct[0];
     task_set_test = &pp_task_set_struct[1];
     /* compose a new NOP command to test for aliveness */
     task_cfg.type = TASK_CFG_NOP_TYPE;
     task_cfg.serial_number = 1;
     task_cfg.tail = task_cfg.head = 1;
     /* loop and check that it got it */
     for (got_it = 0, end = ulapi_time() + wait_time;
	  ulapi_time() < end;
	  ulapi_sleep(0.001)) {
       /* rewrite it */
       task_comm_ptr->task_cfg = task_cfg;
       /* reread it */
       *task_set_ptr = task_comm_ptr->task_set;
       /* and check it */
       if (task_set_ptr->head == task_set_ptr->tail &&
	   task_set_ptr->type == TASK_SET_TYPE &&
	   task_set_ptr->echo_serial_number == task_cfg.serial_number) {
	 got_it = 1;
	 break;
       }
     }
     if (! got_it) {
       fprintf(stderr, "gocfg: timed out connecting to task\n");
       RETURN(1);
     }
     task_cfg.serial_number = task_set_ptr->echo_serial_number + 1;
   }

   debug = 0x0;
   home = 0.0;
   cycle_time = 1.0;
   input_scale = 1.0;
   output_scale = 1.0;
   pid_init(&pid);
   pid_set_gains(&pid, 1, 0, 0,	/* p,i,d */
		 0, 0,		/* vff,aff */
		 -1, 1,		/* min,maxOutput */
		 0, 0,		/* neg,posBias */
		 0);		/* deadband */

   saw_active = 0;
   saw_servo_type = 0;
   saw_joint_quantity = 0;
   saw_debug = 0;
   saw_cycle_time = 0;
   saw_home = 0;
   saw_input_scale = 0;
   saw_output_scale = 0;
   saw_limit = 0;
   saw_profile = 0;
   saw_scale = 0;
   saw_pid = 0;
   saw_parameters = 0;
   saw_link_params = 0;
   saw_strict = 0;
   saw_prog_dir = 0;

   joint_num = 0;

   /* SERVO */

   for (servo_num = 0; servo_num < SERVO_HOWMANY; servo_num++) {
     go_body_init(&body);
     sprintf(section, "SERVO_%d", servo_num + 1);
     num_entries = ini_section(fp, section, ini_entries, MAX_INI_ENTRIES);
     if (num_entries == MAX_INI_ENTRIES) {
       fprintf(stderr,
	       "warning, read max %d entries in [%s], may have missed some\n",
	       MAX_INI_ENTRIES, section);
     }
     if (num_entries <= 0) {
       fprintf(stderr, "gocfg: %s not found, assuming inactive\n", section);
       continue;
     }
     saw_active = 1;

     joint_num++;

     for (entry = 0; entry < num_entries; entry++) {
       /* Do this first! Read what type of joint it is. */
       if (! strcmp(ini_entries[entry].tag, "QUANTITY")) {
	 if (ini_match(ini_entries[entry].rest, "ANGLE")) {
	   joint_quantity = GO_QUANTITY_ANGLE;
	   saw_joint_quantity = 1;
	 } else if (ini_match(ini_entries[entry].rest, "LENGTH")) {
	   joint_quantity = GO_QUANTITY_LENGTH;
	   saw_joint_quantity = 1;
	 } else
 #define REPORT_BAD					\
	   fprintf(stderr, "gocfg: bad entry: [%s] %s = %s\n",	\
		   section,				\
		   ini_entries[entry].tag,		\
		   ini_entries[entry].rest)
	   REPORT_BAD;
	 /* now the TGQ macro will work */
       } else if (! strcmp(ini_entries[entry].tag, "NAME")) {
	 /* Go Motion ignores names, but they may be used by others */
       } else if (! strcmp(ini_entries[entry].tag, "DEBUG")) {
	 if (1 == sscanf(ini_entries[entry].rest, "%i", &i1)) {
	   debug = i1;
	   saw_debug = 1;
	 } else
	   REPORT_BAD;
       } else if (! strcmp(ini_entries[entry].tag, "TYPE")) {
	 if (ini_match(ini_entries[entry].rest, "PID")) {
	   servo_type = GO_SERVO_TYPE_PID;
	   saw_servo_type = 1;
	 } else if (ini_match(ini_entries[entry].rest, "PASS")) {
	   servo_type = GO_SERVO_TYPE_PASS;
	   saw_servo_type = 1;
	 } else
	   REPORT_BAD;
       } else if (! strcmp(ini_entries[entry].tag, "DH_PARAMETERS")) {
	 /* there are always 4 of these */
	 if (4 == sscanf(ini_entries[entry].rest, "%lf %lf %lf %lf", &d1, &d2, &d3, &d4)) {
	   go_dh dh;
	   dh.a = TGL(d1);
	   dh.alpha = TGA(d2);
	   dh.d = TGL(d3);
	   dh.theta = TGA(d4);
	   link_params[joint_num - 1].u.dh = dh;
	   link_params[joint_num - 1].type = GO_LINK_DH;
	   saw_link_params = 1;
	 } else {
	   REPORT_BAD;
	 }
       } else if (! strcmp(ini_entries[entry].tag, "PP_PARAMETERS")) {
	 /* there are always 6 of these, XYZ RPW */
	 if (6 == sscanf(ini_entries[entry].rest, "%lf %lf %lf %lf %lf %lf", &d1, &d2, &d3, &d4, &d5, &d6)) {
	   go_rpy rpy;
	   go_pp pp;
	   pp.pose.tran.x = TGL(d1);
	   pp.pose.tran.y = TGL(d2);
	   pp.pose.tran.z = TGL(d3);
	   rpy.r = TGA(d4);
	   rpy.p = TGA(d5);
	   rpy.y = TGA(d6);
	   go_rpy_quat_convert(&rpy, &pp.pose.rot);
	   link_params[joint_num - 1].u.pp = pp;
	   link_params[joint_num - 1].type = GO_LINK_PP;
	   saw_link_params = 1;
	 } else {
	   REPORT_BAD;
	 }
       } else if (! strcmp(ini_entries[entry].tag, "URDF_PARAMETERS")) {
	 /* there are always 9 of these, XYZ RPW IJK */
	 if (9 == sscanf(ini_entries[entry].rest, "%lf %lf %lf %lf %lf %lf %lf %lf %lf", &d1, &d2, &d3, &d4, &d5, &d6, &d7, &d8, &d9)) {
	   go_rpy rpy;
	   go_urdf urdf;
	   urdf.pose.tran.x = TGL(d1);
	   urdf.pose.tran.y = TGL(d2);
	   urdf.pose.tran.z = TGL(d3);
	   rpy.r = TGA(d4);
	   rpy.p = TGA(d5);
	   rpy.y = TGA(d6);
	   go_rpy_quat_convert(&rpy, &urdf.pose.rot);
	   urdf.axis.x = TGL(d7);
	   urdf.axis.y = TGL(d8);
	   urdf.axis.z = TGL(d9);
	   if (GO_RESULT_OK != go_cart_unit(&urdf.axis, &urdf.axis)) {
	     REPORT_BAD;
	   }
	   link_params[joint_num - 1].u.urdf = urdf;
	   link_params[joint_num - 1].type = GO_LINK_URDF;
	   saw_link_params = 1;
	 } else {
	   REPORT_BAD;
	 }
       } else if (! strcmp(ini_entries[entry].tag, "PK_PARAMETERS")) {
	 /* there are always 6 of these, XYZ XYZ */
	 if (6 == sscanf(ini_entries[entry].rest, "%lf %lf %lf %lf %lf %lf", &d1, &d2, &d3, &d4, &d5, &d6)) {
	   go_pk pk;
	   pk.base.x = TGL(d1);
	   pk.base.y = TGL(d2);
	   pk.base.z = TGL(d3);
	   pk.platform.x = TGL(d4);
	   pk.platform.y = TGL(d5);
	   pk.platform.z = TGL(d6);
	   link_params[joint_num - 1].u.pk = pk;
	   link_params[joint_num - 1].type = GO_LINK_PK;
	   saw_link_params = 1;
	 } else {
	   REPORT_BAD;
	 }
       } else if (! strcmp(ini_entries[entry].tag, "MASS")) {
	 if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	   body.mass = d1;
	 } else
	   REPORT_BAD;
       } else if (! strcmp(ini_entries[entry].tag, "INERTIA")) {
	 /* there are always 9 of these */
	 if (9 == sscanf(ini_entries[entry].rest, "%lf %lf %lf %lf %lf %lf %lf %lf %lf", &d1, &d2, &d3, &d4, &d5, &d6, &d7, &d8, &d9)) {
	   /* need to run TGL twice, since length units are squared */
	   body.inertia[0][0] = TGL(TGL(d1));
	   body.inertia[0][1] = TGL(TGL(d2));
	   body.inertia[0][2] = TGL(TGL(d3));
	   body.inertia[1][0] = TGL(TGL(d4));
	   body.inertia[1][1] = TGL(TGL(d5));
	   body.inertia[1][2] = TGL(TGL(d6));
	   body.inertia[2][0] = TGL(TGL(d7));
	   body.inertia[2][1] = TGL(TGL(d8));
	   body.inertia[2][2] = TGL(TGL(d9));
	 } else {
	   REPORT_BAD;
	 }
       } else if (! strcmp(ini_entries[entry].tag, "CYCLE_TIME")) {
	 if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	   cycle_time = d1;
	   saw_cycle_time = 1;
	 } else
	   REPORT_BAD;
       } else if (! strcmp(ini_entries[entry].tag, "HOME")) {
	 if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	   home = TGQ(d1);
	   saw_home = 1;
	 } else
	   REPORT_BAD;
       } else if (! strcmp(ini_entries[entry].tag, "INPUT_SCALE")) {
	 if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	   input_scale = TGQ(d1);
	   saw_input_scale = 1;
	 } else
	   REPORT_BAD;
       } else if (! strcmp(ini_entries[entry].tag, "OUTPUT_SCALE")) {
	 if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	   /* we use FGQ here as the inverse of TGQ, since we want
	      to convert a user unit that's in the denominator,
	      for example, counts/cm */
	   output_scale = FGQ(d1);
	   saw_output_scale = 1;
	 } else
	   REPORT_BAD;
       } else if (! strcmp(ini_entries[entry].tag, "PARAMETERS")) {
	 i1 = scan_em(darray, ini_entries[entry].rest, sizeof(darray) / sizeof(darray[0]));
	 if (0 < i1) {
	   for (t = 0; t < i1 && t < sizeof(parameters.parameters) / sizeof(parameters.parameters[0]); t++) {
	     parameters.parameters[t] = darray[t];
	   }
	   parameters.number = i1;
	   saw_parameters = 1;
	 } else
	   REPORT_BAD;
       } else if (! strcmp(ini_entries[entry].tag, "P")) {
	 if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	   pid.p = TGQ(d1);
	   saw_pid = 1;
	 } else
	   REPORT_BAD;
       } else if (! strcmp(ini_entries[entry].tag, "I")) {
	 if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	   pid.i = TGQ(d1);
	   saw_pid = 1;
	 } else
	   REPORT_BAD;
       } else if (! strcmp(ini_entries[entry].tag, "D")) {
	 if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	   pid.d = TGQ(d1);
	   saw_pid = 1;
	 } else
	   REPORT_BAD;
       } else if (! strcmp(ini_entries[entry].tag, "PFF")) {
	 if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	   pid.pff = TGQ(d1);
	   saw_pid = 1;
	 } else
	   REPORT_BAD;
       } else if (! strcmp(ini_entries[entry].tag, "VFF")) {
	 if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	   pid.vff = TGQ(d1);
	   saw_pid = 1;
	 } else
	   REPORT_BAD;
       } else if (! strcmp(ini_entries[entry].tag, "AFF")) {
	 if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	   pid.aff = TGQ(d1);
	   saw_pid = 1;
	 } else
	   REPORT_BAD;
       } else if (! strcmp(ini_entries[entry].tag, "MIN_OUTPUT")) {
	 if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	   pid.min_output = TGQ(d1);
	   saw_pid = 1;
	 } else
	   REPORT_BAD;
       } else if (! strcmp(ini_entries[entry].tag, "MAX_OUTPUT")) {
	 if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	   pid.max_output = TGQ(d1);
	   saw_pid = 1;
	 } else
	   REPORT_BAD;
       } else if (! strcmp(ini_entries[entry].tag, "NEG_BIAS")) {
	 if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	   pid.neg_bias = TGQ(d1);
	   saw_pid = 1;
	 } else
	   REPORT_BAD;
       } else if (! strcmp(ini_entries[entry].tag, "POS_BIAS")) {
	 if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	   pid.pos_bias = TGQ(d1);
	   saw_pid = 1;
	 } else
	   REPORT_BAD;
       } else if (! strcmp(ini_entries[entry].tag, "DEADBAND")) {
	 if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	   pid.deadband = TGQ(d1);
	   saw_pid = 1;
	 } else
	   REPORT_BAD;
       } else if (! strcmp(ini_entries[entry].tag, "MIN_LIMIT")) {
	 if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	   min_limit = TGQ(d1);
	   saw_limit = 1;
	 } else
	   REPORT_BAD;
       } else if (! strcmp(ini_entries[entry].tag, "MAX_LIMIT")) {
	 if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	   max_limit = TGQ(d1);
	   saw_limit = 1;
	 } else
	   REPORT_BAD;
       } else if (! strcmp(ini_entries[entry].tag, "MAX_VEL")) {
	 if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	   max_vel = TGQ(d1);
	   saw_profile = 1;
	 } else
	   REPORT_BAD;
       } else if (! strcmp(ini_entries[entry].tag, "MAX_ACC")) {
	 if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	   max_acc = TGQ(d1);
	   saw_profile = 1;
	 } else
	   REPORT_BAD;
       } else if (! strcmp(ini_entries[entry].tag, "MAX_JERK")) {
	 if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	   max_jerk = TGQ(d1);
	   saw_profile = 1;
	 } else
	   REPORT_BAD;
       } else if (! strcmp(ini_entries[entry].tag, "MIN_UP_COUNT") ||
		  ! strcmp(ini_entries[entry].tag, "MIN_DOWN_COUNT") ||
		  ! strcmp(ini_entries[entry].tag, "HOME_VEL") ||
		  ! strcmp(ini_entries[entry].tag, "COUNT_ON_UP")) {
	 /* ignore these, other config apps will handle them */
       } else {
	 fprintf(stderr, "gocfg: warning: unrecognized entry [%s] %s = %s\n",
		 section, ini_entries[entry].tag, ini_entries[entry].rest);
       }
     } /* for (entry) */

     if (saw_active) {
       saw_active = 0;
       servo_cfg[servo_num].type = SERVO_CFG_ACTIVE_TYPE;
       servo_cfg[servo_num].u.active.active = 1;
       /* we're not using ping-pong buffers here, since they aren't
	  really necessary. Consider taking them out above. */
 #define SEND_AND_CHECK							 \
       servo_cfg[servo_num].serial_number++;				 \
       servo_cfg[servo_num].tail = ++servo_cfg[servo_num].head;		 \
       dbprintf(1, "sending %s to servo %d...",				 \
		servo_cfg_symbol(servo_cfg[servo_num].type),		 \
		servo_num+1);						 \
       servo_comm_ptr[servo_num].servo_cfg = servo_cfg[servo_num];	 \
       for (got_it = 0, end = ulapi_time() + wait_time;			 \
	    ulapi_time() < end;						 \
	    ulapi_sleep(0.001)) {					 \
	 *servo_set_ptr[servo_num] = servo_comm_ptr[servo_num].servo_set; \
	 if (servo_set_ptr[servo_num]->head ==				 \
	     servo_set_ptr[servo_num]->tail &&				 \
	     servo_set_ptr[servo_num]->command_type ==			 \
	     servo_cfg[servo_num].type &&				 \
	     servo_set_ptr[servo_num]->echo_serial_number ==		 \
	     servo_cfg[servo_num].serial_number &&			 \
	     servo_set_ptr[servo_num]->status != GO_RCS_STATUS_EXEC) {	 \
	   got_it = 1;							 \
	   break;							 \
	 }								 \
       }									 \
       if (got_it) {							 \
	 if (servo_set_ptr[servo_num]->status == GO_RCS_STATUS_DONE) {	 \
	   dbprintf(0, "done\n");					\
	 } else {							 \
	   dbprintf(0, "error\n");					\
	   RETURN(1);							 \
	 }								 \
       } else {								 \
	 dbprintf(0, "timed out\n");					\
	 RETURN(1);							 \
       }
       SEND_AND_CHECK;
     } /* if (saw_active) */

     if (saw_joint_quantity) {
       saw_joint_quantity = 0;
       link_params[servo_num].quantity = joint_quantity;
     } else {
       fprintf(stderr, "gocfg: required item [%s] QUANTITY not found\n", section);
       RETURN(1);
     }

     go_body_copy(&body, &link_params[servo_num].body);

     if (saw_link_params) {
       saw_link_params = 0;
     } else {
       fprintf(stderr, "gocfg: required item [%s] DH,PP,PK_PARAMETERS not found\n", section);
       RETURN(1);
     }

     /* send the link params */
     servo_cfg[servo_num].type = SERVO_CFG_LINK_TYPE;
     servo_cfg[servo_num].u.link.link = link_params[servo_num];
     SEND_AND_CHECK;

     /* configure servo type in any case, using PID as default */
     servo_cfg[servo_num].type = SERVO_CFG_SERVO_TYPE_TYPE;
     if (saw_servo_type) {
       saw_servo_type = 0;
       servo_cfg[servo_num].u.servo_type.servo_type = servo_type;
     } else {
       servo_cfg[servo_num].u.servo_type.servo_type = GO_SERVO_TYPE_PID;
     }
     SEND_AND_CHECK;

     if (saw_debug) {
       saw_debug = 0;
       servo_cfg[servo_num].type = SERVO_CFG_DEBUG_TYPE;
       servo_cfg[servo_num].u.debug.debug = debug;
       SEND_AND_CHECK;
     }

     if (saw_cycle_time) {
       saw_cycle_time = 0;
       servo_cfg[servo_num].type = SERVO_CFG_CYCLE_TIME_TYPE;
       servo_cfg[servo_num].u.cycle_time.cycle_time = cycle_time;
       SEND_AND_CHECK;
     } else {
       fprintf(stderr, "gocfg: required item [%s] CYCLE_TIME not found\n", section);
       RETURN(1);
     }

     if (saw_home) {
       saw_home = 0;
       servo_cfg[servo_num].type = SERVO_CFG_HOME_TYPE;
       servo_cfg[servo_num].u.home.home = home;
       SEND_AND_CHECK;
     }

     if (saw_input_scale) {
       saw_input_scale = 0;
       servo_cfg[servo_num].type = SERVO_CFG_INPUT_SCALE_TYPE;
       servo_cfg[servo_num].u.scale.scale = input_scale;
       SEND_AND_CHECK;
     } else {
       fprintf(stderr, "gocfg: required item [%s] INPUT_SCALE not found\n", section);
       RETURN(1);
     }

     if (saw_output_scale) {
       saw_output_scale = 0;
       servo_cfg[servo_num].type = SERVO_CFG_OUTPUT_SCALE_TYPE;
       servo_cfg[servo_num].u.scale.scale = output_scale;
       SEND_AND_CHECK;
     } else {
       fprintf(stderr, "gocfg: required item [%s] OUTPUT_SCALE not found\n", section);
       RETURN(1);
     }

     if (saw_limit) {
       saw_limit = 0;
       servo_cfg[servo_num].type = SERVO_CFG_LIMIT_TYPE;
       servo_cfg[servo_num].u.limit.min_limit = min_limit;
       servo_cfg[servo_num].u.limit.max_limit = max_limit;
       SEND_AND_CHECK;
     }

     if (saw_profile) {
       saw_profile = 0;
       servo_cfg[servo_num].type = SERVO_CFG_PROFILE_TYPE;
       servo_cfg[servo_num].u.profile.max_vel = max_vel;
       servo_cfg[servo_num].u.profile.max_acc = max_acc;
       servo_cfg[servo_num].u.profile.max_jerk = max_jerk;
       SEND_AND_CHECK;
     }

     if (saw_pid) {
       saw_pid = 0;
       /* convert some of the gains from user units, but not all */
       servo_cfg[servo_num].type = SERVO_CFG_PID_TYPE;
       pid_copy_gains(&servo_cfg[servo_num].u.pid, &pid);
       SEND_AND_CHECK;
     }

     if (saw_parameters) {
       saw_parameters = 0;
       servo_cfg[servo_num].type = SERVO_CFG_PARAMETERS_TYPE;
       servo_cfg[servo_num].u.parameters = parameters;
       SEND_AND_CHECK;
     }
   }				/* for (servo_num) */

   /* TRAJ */

   saw_debug = 0;
   saw_cycle_time = 0;
   saw_home = 0;
   saw_tool_transform = 0;
   saw_limit = 0;
   saw_profile = 0;

   strcpy(section, "TRAJ");

   num_entries = ini_section(fp, section, ini_entries, MAX_INI_ENTRIES);
   if (num_entries == MAX_INI_ENTRIES) {
     fprintf(stderr,
	     "warning, read max %d entries in [%s], may have missed some\n",
	     MAX_INI_ENTRIES, section);
   }
   for (entry = 0; entry < num_entries; entry++) {
     if (! strcmp(ini_entries[entry].tag, "SHM_KEY")) {
       /* ignore this here, it's handled by the run script */
     } else if (! strcmp(ini_entries[entry].tag, "KINEMATICS")) {
       /* ignore this here, it's handled by the run script */
     } else if (! strcmp(ini_entries[entry].tag, "DEBUG")) {
       if (1 == sscanf(ini_entries[entry].rest, "%i", &i1)) {
	 debug = i1;
	 saw_debug = 1;
       } else
	 REPORT_BAD;
     } else if (! strcmp(ini_entries[entry].tag, "CYCLE_TIME")) {
       if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	 cycle_time = d1;
	 saw_cycle_time = 1;
       } else
	 REPORT_BAD;
     } else if (! strcmp(ini_entries[entry].tag, "TOOL_TRANSFORM")) {
       if (6 == sscanf(ini_entries[entry].rest,
		       "%lf %lf %lf %lf %lf %lf", &d1, &d2, &d3, &d4, &d5, &d6)) {
	 tran.x = d1, tran.y = d2, tran.z = d3, rpy.r = d4, rpy.p = d5, rpy.y = d6;
	 tool_transform.tran.x = TGL(tran.x), tool_transform.tran.y = TGL(tran.y), tool_transform.tran.z = TGL(tran.z);
	 rpy.r = TGA(rpy.r), rpy.p = TGA(rpy.p), rpy.y = TGA(rpy.y);
	 go_rpy_quat_convert(&rpy, &tool_transform.rot);
	 saw_tool_transform = 1;
       } else
	 REPORT_BAD;
     } else if (! strcmp(ini_entries[entry].tag, "HOME")) {
       if (6 == sscanf(ini_entries[entry].rest,
		       "%lf %lf %lf %lf %lf %lf", &d1, &d2, &d3, &d4, &d5, &d6)) {
	 tran.x = d1, tran.y = d2, tran.z = d3, rpy.r = d4, rpy.p = d5, rpy.y = d6;
	 home_pose.tran.x = TGL(tran.x), home_pose.tran.y = TGL(tran.y), home_pose.tran.z = TGL(tran.z);
	 rpy.r = TGA(rpy.r), rpy.p = TGA(rpy.p), rpy.y = TGA(rpy.y);
	 go_rpy_quat_convert(&rpy, &home_pose.rot);
	 saw_home = 1;
       } else
	 REPORT_BAD;
     } else if (! strcmp(ini_entries[entry].tag, "MIN_LIMIT")) {
       if (6 == sscanf(ini_entries[entry].rest,
		       "%lf %lf %lf %lf %lf %lf", &d1, &d2, &d3, &d4, &d5, &d6)) {
	 tran.x = d1, tran.y = d2, tran.z = d3, rpy.r = d4, rpy.p = d5, rpy.y = d6;
	 min_limit_pose.tran.x = TGL(tran.x), min_limit_pose.tran.y = TGL(tran.y), min_limit_pose.tran.z = TGL(tran.z);
	 rpy.r = TGA(rpy.r), rpy.p = TGA(rpy.p), rpy.y = TGA(rpy.y);
	 go_rpy_quat_convert(&rpy, &min_limit_pose.rot);
	 saw_limit = 1;
       } else
	 REPORT_BAD;
     } else if (! strcmp(ini_entries[entry].tag, "MAX_LIMIT")) {
       if (6 == sscanf(ini_entries[entry].rest,
		       "%lf %lf %lf %lf %lf %lf", &d1, &d2, &d3, &d4, &d5, &d6)) {
	 tran.x = d1, tran.y = d2, tran.z = d3, rpy.r = d4, rpy.p = d5, rpy.y = d6;
	 max_limit_pose.tran.x = TGL(tran.x), max_limit_pose.tran.y = TGL(tran.y), max_limit_pose.tran.z = TGL(tran.z);
	 rpy.r = TGA(rpy.r), rpy.p = TGA(rpy.p), rpy.y = TGA(rpy.y);
	 go_rpy_quat_convert(&rpy, &max_limit_pose.rot);
	 saw_limit = 1;
       } else
	 REPORT_BAD;
     } else if (! strcmp(ini_entries[entry].tag, "MAX_TVEL")) {
       if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	 max_tvel = TGL(d1);
	 saw_profile = 1;
       } else
	 REPORT_BAD;
     } else if (! strcmp(ini_entries[entry].tag, "MAX_TACC")) {
       if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	 max_tacc = TGL(d1);
	 saw_profile = 1;
       } else
	 REPORT_BAD;
     } else if (! strcmp(ini_entries[entry].tag, "MAX_TJERK")) {
       if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	 max_tjerk = TGL(d1);
	 saw_profile = 1;
       } else
	 REPORT_BAD;
     } else if (! strcmp(ini_entries[entry].tag, "MAX_RVEL")) {
       if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	 max_rvel = TGA(d1);
	 saw_profile = 1;
       } else
	 REPORT_BAD;
     } else if (! strcmp(ini_entries[entry].tag, "MAX_RACC")) {
       if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	 max_racc = TGA(d1);
	 saw_profile = 1;
       } else
	 REPORT_BAD;
     } else if (! strcmp(ini_entries[entry].tag, "MAX_RJERK")) {
       if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	 max_rjerk = TGA(d1);
	 saw_profile = 1;
       } else
	 REPORT_BAD;
     } else if (! strcmp(ini_entries[entry].tag, "MAX_SCALE")) {
       if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1) && d1 > 0.0) {
	 max_scale = d1;
	 saw_scale = 1;
       } else
	 REPORT_BAD;
     } else if (! strcmp(ini_entries[entry].tag, "MAX_SCALE_V")) {
       if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1) && d1 > 0.0) {
	 max_scale_v = d1;
	 saw_scale = 1;
       } else
	 REPORT_BAD;
     } else if (! strcmp(ini_entries[entry].tag, "MAX_SCALE_A")) {
       if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1) && d1 > 0.0) {
	 max_scale_a = d1;
	 saw_scale = 1;
       } else
	 REPORT_BAD;
     } else
       fprintf(stderr, "gocfg: warning: unrecognized entry: [%s] %s = %s\n",
	       section, ini_entries[entry].tag, ini_entries[entry].rest);
   } /* for (entry) */

   if (saw_debug) {
     saw_debug = 0;
     traj_cfg.type = TRAJ_CFG_DEBUG_TYPE;
     traj_cfg.u.debug.debug = debug;
     /* we're not using ping-pong buffers here, since they aren't
	really necessary. Consider taking them out above. */
 #undef SEND_AND_CHECK
 #define SEND_AND_CHECK							\
     traj_cfg.serial_number++;						\
     traj_cfg.tail = ++traj_cfg.head;					\
     dbprintf(1, "sending %s to traj...",				\
	     traj_cfg_symbol(traj_cfg.type)); 				\
    traj_comm_ptr->traj_cfg = traj_cfg;					\
    for (got_it = 0, end = ulapi_time() + wait_time;			\
	 ulapi_time() < end;						\
	 ulapi_sleep(0.001)) {						\
      *traj_set_ptr = traj_comm_ptr->traj_set;				\
      if (traj_set_ptr->head == traj_set_ptr->tail &&			\
	  traj_set_ptr->command_type == traj_cfg.type &&		\
	  traj_set_ptr->echo_serial_number ==				\
	  traj_cfg.serial_number &&					\
	  traj_set_ptr->status != GO_RCS_STATUS_EXEC) {			\
	got_it = 1;							\
	break;								\
      }									\
    }									\
    if (got_it) {							\
      if (traj_set_ptr->status == GO_RCS_STATUS_DONE) {			\
	dbprintf(0, "done\n");						\
      } else {								\
	dbprintf(0, "error\n");						\
	RETURN(1);							\
      }									\
    } else {								\
      dbprintf(0, "timed out\n");					\
      RETURN(1);                                                	\
    }
    SEND_AND_CHECK;
  }
  /* if (saw_debug) */

  if (saw_cycle_time) {
    saw_cycle_time = 0;
    traj_cfg.type = TRAJ_CFG_CYCLE_TIME_TYPE;
    traj_cfg.u.cycle_time.cycle_time = cycle_time;
    SEND_AND_CHECK;
  }

  /* do the tool transform before the limits and home, since
     the limits and home are with respect to any established
     tool transform, and the one in the .ini file is taken
     to be the default */
  if (saw_tool_transform) {
    saw_tool_transform = 0;
    traj_cfg.type = TRAJ_CFG_TOOL_TRANSFORM_TYPE;
    traj_cfg.u.tool_transform.tool_transform = tool_transform;
    SEND_AND_CHECK;
  }

  if (saw_home) {
    saw_home = 0;
    traj_cfg.type = TRAJ_CFG_HOME_TYPE;
    traj_cfg.u.home.home = home_pose;
    SEND_AND_CHECK;
  }

  if (saw_limit) {
    saw_limit = 0;
    traj_cfg.type = TRAJ_CFG_LIMIT_TYPE;
    traj_cfg.u.limit.min_limit = min_limit_pose;
    traj_cfg.u.limit.max_limit = max_limit_pose;
    SEND_AND_CHECK;
  }

  if (saw_profile) {
    saw_limit = 0;
    traj_cfg.type = TRAJ_CFG_PROFILE_TYPE;
    traj_cfg.u.profile.max_tvel = max_tvel;
    traj_cfg.u.profile.max_tacc = max_tacc;
    traj_cfg.u.profile.max_tjerk = max_tjerk;
    traj_cfg.u.profile.max_rvel = max_rvel;
    traj_cfg.u.profile.max_racc = max_racc;
    traj_cfg.u.profile.max_rjerk = max_rjerk;
    SEND_AND_CHECK;
  }

  if (saw_scale) {
    saw_scale = 0;
    traj_cfg.type = TRAJ_CFG_MAX_SCALE_TYPE;
    traj_cfg.u.scale.scale = max_scale;
    traj_cfg.u.scale.scale_v = max_scale_v;
    traj_cfg.u.scale.scale_a = max_scale_a;
    SEND_AND_CHECK;
  }

  /* link parameters are in the link_params[] array */
  traj_cfg.type = TRAJ_CFG_KINEMATICS_TYPE;
  for (joint = 0; joint < joint_num; joint++) {
    traj_cfg.u.kinematics.parameters[joint] = link_params[joint];
  }
  traj_cfg.u.kinematics.num = joint_num;
  SEND_AND_CHECK;

  /* TOOL */
  if (TOOL_SHM_KEY != 0) {
    saw_debug = 0;
    saw_cycle_time = 0;

    strcpy(section, "TOOL");

    num_entries = ini_section(fp, section, ini_entries, MAX_INI_ENTRIES);
    if (num_entries == MAX_INI_ENTRIES) {
      fprintf(stderr,
	      "warning, read max %d entries in [%s], may have missed some\n",
	      MAX_INI_ENTRIES, section);
    }
    for (entry = 0; entry < num_entries; entry++) {
      if (! strcmp(ini_entries[entry].tag, "SHM_KEY")) {
	/* ignore this here, it's handled by the run script */
      } else if (! strcmp(ini_entries[entry].tag, "DEBUG")) {
	if (1 == sscanf(ini_entries[entry].rest, "%i", &i1)) {
	  debug = i1;
	  saw_debug = 1;
	} else
	  REPORT_BAD;
      } else if (! strcmp(ini_entries[entry].tag, "CYCLE_TIME")) {
	if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	  cycle_time = d1;
	  saw_cycle_time = 1;
	} else
	  REPORT_BAD;
      } else if (! strcmp(ini_entries[entry].tag, "TOOLMAIN")) {
	/* ignore this, it's used by 'gorun' directly */
      } else
	fprintf(stderr, "gocfg: warning: unrecognized entry: [%s] %s = %s\n",
		section, ini_entries[entry].tag, ini_entries[entry].rest);
    } /* for (entry) */

    if (saw_debug) {
      saw_debug = 0;
      tool_cfg.type = TOOL_CFG_DEBUG_TYPE;
      tool_cfg.u.debug.debug = debug;
#undef SEND_AND_CHECK
#define SEND_AND_CHECK						\
      tool_cfg.serial_number++;					\
      tool_cfg.tail = ++tool_cfg.head;				\
      dbprintf(1, "sending %s to tool...",			\
	       tool_cfg_symbol(tool_cfg.type));			\
      tool_comm_ptr->tool_cfg = tool_cfg;			\
      for (got_it = 0, end = ulapi_time() + wait_time;		\
	   ulapi_time() < end;					\
	   ulapi_sleep(0.001)) {				\
	*tool_set_ptr = tool_comm_ptr->tool_set;		\
	if (tool_set_ptr->head == tool_set_ptr->tail &&		\
	    tool_set_ptr->command_type == tool_cfg.type &&	\
	    tool_set_ptr->echo_serial_number ==			\
	    tool_cfg.serial_number &&				\
	    tool_set_ptr->status != GO_RCS_STATUS_EXEC) {	\
	  got_it = 1;						\
	  break;						\
	}							\
      }								\
      if (got_it) {						\
	if (tool_set_ptr->status == GO_RCS_STATUS_DONE) {	\
	  dbprintf(0, "done\n");				\
	} else {						\
	  dbprintf(0, "error\n");				\
	  RETURN(1);						\
	}							\
      } else {							\
	dbprintf(0, "timed out\n");				\
	RETURN(1);						\
      }
      SEND_AND_CHECK;
    }
    /* if (saw_debug) */

    if (saw_cycle_time) {
      saw_cycle_time = 0;
      tool_cfg.type = TOOL_CFG_CYCLE_TIME_TYPE;
      tool_cfg.u.cycle_time.cycle_time = cycle_time;
      SEND_AND_CHECK;
    }
  }

  /* TASK */
  if (TASK_SHM_KEY != 0) {
    saw_debug = 0;
    saw_cycle_time = 0;

    strcpy(section, "TASK");

    num_entries = ini_section(fp, section, ini_entries, MAX_INI_ENTRIES);
    if (num_entries == MAX_INI_ENTRIES) {
      fprintf(stderr,
	      "warning, read max %d entries in [%s], may have missed some\n",
	      MAX_INI_ENTRIES, section);
    }
    for (entry = 0; entry < num_entries; entry++) {
      if (! strcmp(ini_entries[entry].tag, "SHM_KEY")) {
	/* ignore this here, it's handled by the run script */
      } else if (! strcmp(ini_entries[entry].tag, "DEBUG")) {
	if (1 == sscanf(ini_entries[entry].rest, "%i", &i1)) {
	  debug = i1;
	  saw_debug = 1;
	} else
	  REPORT_BAD;
      } else if (! strcmp(ini_entries[entry].tag, "CYCLE_TIME")) {
	if (1 == sscanf(ini_entries[entry].rest, "%lf", &d1)) {
	  cycle_time = d1;
	  saw_cycle_time = 1;
	} else
	  REPORT_BAD;
      } else if (! strcmp(ini_entries[entry].tag, "STRICT")) {
	if (1 == sscanf(ini_entries[entry].rest, "%i", &i1)) {
	  strict = i1;
	  saw_strict = 1;
	} else
	  REPORT_BAD;
      } else if (! strcmp(ini_entries[entry].tag, "TCP_PORT") ||
		 ! strcmp(ini_entries[entry].tag, "PARAMETER_FILE_NAME") ||
		 ! strcmp(ini_entries[entry].tag, "TOOL_FILE_NAME") ||
		 ! strcmp(ini_entries[entry].tag, "MTTF") ||
		 ! strcmp(ini_entries[entry].tag, "MTTR")) {
	/* we don't do anything with these -- they are handled by
	   the applications that direct read the ini file */
      } else if (! strcmp(ini_entries[entry].tag, "PROG_DIR")) {
	if (1 == sscanf(ini_entries[entry].rest, "%s", prog_dir)) {
	  saw_prog_dir = 1;
	} else
	  REPORT_BAD;
      } else
	fprintf(stderr, "gocfg: warning: unrecognized entry: [%s] %s = %s\n",
		section, ini_entries[entry].tag, ini_entries[entry].rest);
    } /* for (entry) */

    if (saw_debug) {
      saw_debug = 0;
      task_cfg.type = TASK_CFG_DEBUG_TYPE;
      task_cfg.u.debug.debug = debug;
#undef SEND_AND_CHECK
#define SEND_AND_CHECK						\
      task_cfg.serial_number++;					\
      task_cfg.tail = ++task_cfg.head;				\
      dbprintf(1, "sending %s to task...",			\
	       task_cfg_symbol(task_cfg.type));			\
      task_comm_ptr->task_cfg = task_cfg;			\
      for (got_it = 0, end = ulapi_time() + wait_time;		\
	   ulapi_time() < end;					\
	   ulapi_sleep(0.001)) {				\
	*task_set_ptr = task_comm_ptr->task_set;		\
	if (task_set_ptr->head == task_set_ptr->tail &&		\
	    task_set_ptr->command_type == task_cfg.type &&	\
	    task_set_ptr->echo_serial_number ==			\
	    task_cfg.serial_number &&				\
	    task_set_ptr->status != GO_RCS_STATUS_EXEC) {	\
	  got_it = 1;						\
	  break;						\
	}							\
      }								\
      if (got_it) {						\
	if (task_set_ptr->status == GO_RCS_STATUS_DONE) {	\
	  dbprintf(0, "done\n");				\
	} else {						\
	  dbprintf(0, "error\n");				\
	  RETURN(1);						\
	}							\
      } else {							\
	dbprintf(0, "timed out\n");				\
	RETURN(1);						\
      }
      SEND_AND_CHECK;
    }
    /* if (saw_debug) */

    if (saw_cycle_time) {
      saw_cycle_time = 0;
      task_cfg.type = TASK_CFG_CYCLE_TIME_TYPE;
      task_cfg.u.cycle_time.cycle_time = cycle_time;
      SEND_AND_CHECK;
    }

    if (saw_strict) {
      saw_strict = 0;
      task_cfg.type = TASK_CFG_STRICT_TYPE;
      task_cfg.u.strict.strict = strict;
      SEND_AND_CHECK;
    }

    if (saw_prog_dir) {
      saw_prog_dir = 0;
      task_cfg.type = TASK_CFG_PROG_DIR_TYPE;
      strcpy(task_cfg.u.prog_dir.prog_dir, prog_dir);
      SEND_AND_CHECK;
    }
  }

CLOSE:
  if (NULL != fp) {
	  fclose(fp);
	  fp = NULL;
  }
  if (NULL != task_shm) {
    ulapi_rtm_delete(task_shm);
    task_shm = NULL;
  }
  if (NULL != tool_shm) {
    ulapi_rtm_delete(tool_shm);
    tool_shm = NULL;
  }
  traj_comm_ptr = NULL;
  if (NULL != traj_shm) {
    ulapi_rtm_delete(traj_shm);
    traj_shm = NULL;
  }
  traj_comm_ptr = NULL;
  if (NULL != servo_shm) {
    ulapi_rtm_delete(servo_shm);
    servo_shm = NULL;
  }
  servo_comm_ptr = NULL;

  return ulapi_exit();
}
