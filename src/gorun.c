#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>		/* atexit */
#include <signal.h>
#include <ulapi.h>
#include <inifile.h>
#include "go.h"
#include "servointf.h"		/* SERVO_NUM */
#include "taskintf.h"		/* DEFAULT_TASK_TCP_PORT */

/*
  Usage: gorun [-i <inifile>] [-u unix | rtai] [-s] [-w] [-d]

  <inifile> is optional, defaults to gomotion.ini

  -s means run the go shell, gosh, instead of the GUI pendant
  -w means wait for gomain, instead of running gosh or the pendant
  -d means pass a debug flag if applicable
*/

static enum {USE_UNIX, USE_RTAI} which_ulapi = USE_UNIX;

#define DEFAULT_INI_FILE "gomotion.ini"

static int ini_load(char *inifile_name,
		    char ext_init_string[INIFILE_MAX_LINELEN],
		    int *rtapi_hal_nsecs_per_period,
		    int *go_stepper_type,
		    int *go_stepper_shm_key,
		    int *servo_howmany,
		    int *servo_shm_key,
		    int *servo_sem_key,
		    int *traj_shm_key,
		    char kinematics[INIFILE_MAX_LINELEN],
		    int *go_log_shm_key,
		    int *go_io_shm_key,
		    int *tool_shm_key,
		    int *task_shm_key,
		    int *task_tcp_port)
{
  FILE *fp;
  const char *section;
  const char *key;
  const char *inistring;

  if (NULL == (fp = fopen(inifile_name, "r"))) {
    fprintf(stderr, "gorun: can't open %s\n", inifile_name);
    return 1;
  }

#define CLOSE_AND_RETURN \
  fclose(fp);		 \
  return 1

  section = "GOMOTION";

  key = "EXT_INIT_STRING";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    /* optional, make it "0" */
    strncpy(ext_init_string, "0", INIFILE_MAX_LINELEN);
    ext_init_string[INIFILE_MAX_LINELEN-1] = 0;
  } else {
    strncpy(ext_init_string, inistring, INIFILE_MAX_LINELEN);
    ext_init_string[INIFILE_MAX_LINELEN-1] = 0;
  }

  section = "RTAPI_HAL";

  key = "NSECS_PER_PERIOD";
  inistring = ini_find(fp, key, section);
  if (NULL != inistring) {
    if (1 != sscanf(inistring, "%i", rtapi_hal_nsecs_per_period)) {
      fprintf(stderr, "gorun: bad entry: [%s] %s = %s\n", section, key, inistring);
      CLOSE_AND_RETURN;
    }
  } else {
    /* not present, so set to default */
    *rtapi_hal_nsecs_per_period = 0;
  }

  section = "GO_STEPPER";

  key = "TYPE";
  inistring = ini_find(fp, key, section);
  if (NULL != inistring) {
    if (1 != sscanf(inistring, "%i", go_stepper_type)) {
      fprintf(stderr, "gorun: bad entry: [%s] %s = %s\n", section, key, inistring);
      CLOSE_AND_RETURN;
    }
  } else {
    /* not present, so set to default */
    *go_stepper_type = 0;
  }

  key = "SHM_KEY";
  inistring = ini_find(fp, key, section);
  if (NULL != inistring) {
    if (1 != sscanf(inistring, "%i", go_stepper_shm_key)) {
      fprintf(stderr, "gorun: bad entry: [%s] %s = %s\n", section, key, inistring);
      CLOSE_AND_RETURN;
    }
  } else {
    /* not present, so set to default */
    *go_stepper_shm_key = 0;
  }

  section = "SERVO";

  key = "HOWMANY";
  inistring = ini_find(fp, key, section);
  if (NULL != inistring) {
    if (1 != sscanf(inistring, "%i", servo_howmany)) {
      fprintf(stderr, "gorun: bad entry: [%s] %s = %s\n", section, key, inistring);
      CLOSE_AND_RETURN;
    }
  } else {
    /* not present, so set to max */
    *servo_howmany = SERVO_NUM;
  }

  key = "SHM_KEY";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    fprintf(stderr, "gorun: missing entry: [%s] %s\n", section, key);
    CLOSE_AND_RETURN;
  } else if (1 != sscanf(inistring, "%i", servo_shm_key)) {
    fprintf(stderr, "gorun: bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN;
  }

  key = "SEM_KEY";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    fprintf(stderr, "gorun: missing entry: [%s] %s\n", section, key);
    CLOSE_AND_RETURN;
  } else if (1 != sscanf(inistring, "%i", servo_sem_key)) {
    fprintf(stderr, "gorun: bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN;
  }

  section = "TRAJ";

  key = "SHM_KEY";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    fprintf(stderr, "gorun: missing entry: [%s] %s\n", section, key);
    CLOSE_AND_RETURN;
  } else if (1 != sscanf(inistring, "%i", traj_shm_key)) {
    fprintf(stderr, "gorun: bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN;
  }

  key = "KINEMATICS";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    /* optional, make it "trivkins" */
    strncpy(kinematics, "trivkins", INIFILE_MAX_LINELEN);
    kinematics[INIFILE_MAX_LINELEN-1] = 0;
  } else {
    strncpy(kinematics, inistring, INIFILE_MAX_LINELEN);
    kinematics[INIFILE_MAX_LINELEN-1] = 0;
  }

  section = "GO_LOG";

  key = "SHM_KEY";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    fprintf(stderr, "gorun: missing entry: [%s] %s\n", section, key);
    CLOSE_AND_RETURN;
  } else if (1 != sscanf(inistring, "%i", go_log_shm_key)) {
    fprintf(stderr, "gorun: bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN;
  }

  section = "GO_IO";

  key = "SHM_KEY";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    fprintf(stderr, "gorun: missing entry: [%s] %s\n", section, key);
    CLOSE_AND_RETURN;
  } else if (1 != sscanf(inistring, "%i", go_io_shm_key)) {
    fprintf(stderr, "gorun: bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN;
  }

  section = "TOOL";

  key = "SHM_KEY";
  inistring = ini_find(fp, key, section);
  if (NULL != inistring) {
    if (1 != sscanf(inistring, "%i", tool_shm_key)) {
      fprintf(stderr, "gorun: bad entry: [%s] %s = %s\n", section, key, inistring);
      CLOSE_AND_RETURN;
    }
  } /* else not present, so leave it alone */

  section = "TASK";

  key = "SHM_KEY";
  inistring = ini_find(fp, key, section);
  if (NULL != inistring) {
    if (1 != sscanf(inistring, "%i", task_shm_key)) {
      fprintf(stderr, "gorun: bad entry: [%s] %s = %s\n", section, key, inistring);
      CLOSE_AND_RETURN;
    }
  } /* else not present */

  key = "TCP_PORT";
  inistring = ini_find(fp, key, section);
  if (NULL != inistring) {
    if (1 != sscanf(inistring, "%i", task_tcp_port)) {
      fprintf(stderr, "gorun: bad entry: [%s] %s = %s\n", section, key, inistring);
      CLOSE_AND_RETURN;
    }
  } /* else not present */

  fclose(fp);
  return 0;
}

static void print_help(void)
{
  printf("-i <file> : use initialization file <file>, default %s\n", DEFAULT_INI_FILE);
  printf("-s        : run the go shell, gosh, instead of the GUI pendant\n");
  printf("-w        : wait for gomain, instead of running gosh or the pendant\n");
  printf("-d        : turn debug on\n");
  printf("-?        : print this help message\n");
}

static void *gostepper_proc = NULL;
static void *gomain_proc = NULL;
static void *toolmain_proc = NULL;
static void *taskmain_proc = NULL;
static void *tasksvr_proc = NULL;
static void *gui_proc = NULL;

static void cleanup(void)
{
  int result, sysresult;

#define CLEANIT(PROC)				\
  if (NULL != PROC) {				\
    ulapi_process_stop(PROC);			\
    ulapi_process_delete(PROC);			\
    PROC = NULL;				\
  }

  CLEANIT(gui_proc);
  CLEANIT(tasksvr_proc);
  CLEANIT(taskmain_proc);
  CLEANIT(toolmain_proc);
  CLEANIT(gomain_proc);
  CLEANIT(gostepper_proc);

  if (USE_RTAI == which_ulapi) {
    result = ulapi_system("sudo rmmod toolmain_mod", &sysresult);
    result = ulapi_system("sudo rmmod gomain_mod", &sysresult);
    result = ulapi_system("sudo rmmod gostepper_mod", &sysresult);
  }
}

static void sigquit(int sig)
{
  exit(0);
}

int main(int argc, char *argv[])
{
#define BUFFERLEN 256
  int option;
  int shell_arg = 0;
  int wait_arg = 0;
  int debug_arg = 0;
  const char *ularg = "";
  char inifile_name[INIFILE_MAX_LINELEN] = DEFAULT_INI_FILE;
  char ext_init_string[INIFILE_MAX_LINELEN];
  int rtapi_hal_nsecs_per_period = 0;
  int go_stepper_type = 0;
  int go_stepper_shm_key = 0;
  int servo_howmany;
  int servo_shm_key;
  int servo_sem_key;
  int traj_shm_key;
  char kinematics[INIFILE_MAX_LINELEN];
  int go_log_shm_key;
  int go_io_shm_key;
  int tool_shm_key = 0;
  int task_shm_key = 0;
  int task_tcp_port = DEFAULT_TASK_TCP_PORT;
  char dirname[BUFFERLEN];
  char path[BUFFERLEN];
  ulapi_integer retval;
  ulapi_integer result;
  int sysresult;

  opterr = 0;
  while (1) {
    option = ulapi_getopt(argc, argv, ":i:u:swd?");
    if (option == -1)
      break;

    switch (option) {
    case 'i':
      strncpy(inifile_name, ulapi_optarg, sizeof(inifile_name));
      inifile_name[sizeof(inifile_name) - 1] = 0;
      break;

    case 'u':
      if (! strcmp(ulapi_optarg, "unix")) {
	which_ulapi = USE_UNIX;
	ularg = "-u unix";
      } else if (! strcmp(ulapi_optarg, "rtai")) {
	which_ulapi = USE_RTAI;
	ularg = "-u rtai";
      } else {
	fprintf(stderr, "gorun: invalid target for -%c: %s\n", option, ulapi_optarg);
	return 1;
      }
      break;

    case 's':
      shell_arg = 1;
      break;

    case 'w':
      wait_arg = 1;
      break;

    case 'd':
      debug_arg = 0xFFFFFFFF;
      break;

    case '?':
      print_help();
      return 0;
      break;

    case ':':
      fprintf(stderr, "gorun: missing value for -%c\n", ulapi_optopt);
      return 1;
      break;

    default:			/* '?' */
      fprintf (stderr, "gorun: unrecognized option -%c\n", ulapi_optopt);
      return 1;
      break;
    }
  }
  if (ulapi_optind < argc) {
    fprintf(stderr, "gorun: extra non-option characters: %s\n", argv[ulapi_optind]);
    return 1;
  }

  if (ULAPI_OK != ulapi_init()) {
    fprintf(stderr, "gorun: can't init ulapi\n");
    return 1;
  } 

  ulapi_dirname(argv[0], dirname);

  if (0 != ini_load(inifile_name,
		    ext_init_string,
		    &rtapi_hal_nsecs_per_period,
		    &go_stepper_type,
		    &go_stepper_shm_key,
		    &servo_howmany,
		    &servo_shm_key,
		    &servo_sem_key,
		    &traj_shm_key,
		    kinematics,
		    &go_log_shm_key,
		    &go_io_shm_key,
		    &tool_shm_key,
		    &task_shm_key,
		    &task_tcp_port)) {
    return 1;
  }

  atexit(cleanup);
  signal(SIGINT, sigquit);

  if (USE_RTAI == which_ulapi) {
    result = ulapi_snprintf(path, sizeof(path)-1,
			    "%s%s%s %s",
			    dirname, ulapi_pathsep, "insrtl",
			    debug_arg ? "" : "2>/dev/null");
    if (result >= sizeof(path)) {
      fprintf(stderr, "gorun: insrtl command too long\n");
      return 1;
    }

    result = ulapi_system(path, &sysresult);
    if (GO_RESULT_OK != result || 0 != sysresult) {
      fprintf(stderr, "gorun: can't execute insrtl command\n");
      return 1;
    }

    result = ulapi_snprintf(path, sizeof(path)-1,
			    "%s%s%s -i %s %s",
			    dirname, ulapi_pathsep, "insrtapi",
			    inifile_name,
			    debug_arg ? "" : "2>/dev/null");
    if (result >= sizeof(path)) {
      fprintf(stderr, "gorun: insrtapi command too long\n");
      return 1;
    }
    result = ulapi_system(path, &sysresult);
    if (GO_RESULT_OK != result || 0 != sysresult) {
      fprintf(stderr, "gorun: can't execute insrtapi command\n");
      return 1;
    }
  }

  if (GO_RESULT_OK != go_init()) {
    fprintf(stderr, "gorun: can't init go motion\n");
    return 1;
  }

  if (0 != go_stepper_shm_key) {
    if (USE_RTAI == which_ulapi) {
      result = ulapi_snprintf(path, sizeof(path)-1,
			      "sudo insmod -f %s%s%s%s%s%s%s DEBUG=%d GO_STEPPER_TYPE=%d GO_STEPPER_SHM_KEY=%d",
			      dirname, ulapi_pathsep, "..", ulapi_pathsep, "rtlib", ulapi_pathsep, "gostepper_mod.ko",
			      debug_arg ? 1 : 0,
			      (int) go_stepper_type,
			      (int) go_stepper_shm_key);
      if (result >= sizeof(path)) {
	fprintf(stderr, "gorun: install go stepper command too long\n");
	return 1;
      }
      result = ulapi_system(path, &sysresult);
      /* ignore sysresult, which can be nonzero if the module exists */
      if (GO_RESULT_OK != result) {
	fprintf(stderr, "gorun: can't execute install go stepper command\n");
	return 1;
      }
    } else {
      result = ulapi_snprintf(path, sizeof(path)-1,
			      "%s%s%s DEBUG=%d GO_STEPPER_TYPE=%d GO_STEPPER_SHM_KEY=%d",
			      dirname, ulapi_pathsep, "gostepper",
			      debug_arg ? 1 : 0,
			      (int) go_stepper_type,
			      (int) go_stepper_shm_key);
      if (result >= sizeof(path)) {
	fprintf(stderr, "gorun: go stepper command too long\n");
	return 1;
      }
      gostepper_proc = ulapi_process_new();
      if (ULAPI_OK != ulapi_process_start(gostepper_proc, path)) {
	ulapi_process_delete(gostepper_proc);
	gostepper_proc = NULL;
	fprintf(stderr, "gorun: can't run go stepper process\n");
	return 1;
      }
    }
  }

  if (USE_RTAI == which_ulapi) {
    result = ulapi_snprintf(path, sizeof(path)-1,
			    "sudo insmod -f %s%s%s%s%s%s%s DEBUG=%d EXT_INIT_STRING=%s SERVO_HOWMANY=%d SERVO_SHM_KEY=%d SERVO_SEM_KEY=%d TRAJ_SHM_KEY=%d KINEMATICS=%s GO_LOG_SHM_KEY=%d GO_IO_SHM_KEY=%d", 
			    dirname, ulapi_pathsep, "..", ulapi_pathsep, "rtlib", ulapi_pathsep, "gomain_mod.ko",
			    debug_arg ? 1 : 0,
			    ext_init_string,
			    (int) servo_howmany,
			    (int) servo_shm_key,
			    (int) servo_sem_key, 
			    (int) traj_shm_key,
			    kinematics,
			    (int) go_log_shm_key,
			    (int) go_io_shm_key);
    if (result >= sizeof(path)) {
      fprintf(stderr, "gorun: install go main command too long\n");
      return 1;
    }
    result = ulapi_system(path, &sysresult);
    /* ignore sysresult, which can be nonzero if the module exists */
    if (GO_RESULT_OK != result) {
      fprintf(stderr, "gorun: can't execute install go main command\n");
      return 1;
    }
  } else {
    result = ulapi_snprintf(path, sizeof(path)-1,
			    "%s%s%s DEBUG=%d EXT_INIT_STRING=%s SERVO_HOWMANY=%d SERVO_SHM_KEY=%d SERVO_SEM_KEY=%d TRAJ_SHM_KEY=%d KINEMATICS=%s GO_LOG_SHM_KEY=%d GO_IO_SHM_KEY=%d", 
			    dirname, ulapi_pathsep, "gomain",
			    debug_arg ? 1 : 0,
			    ext_init_string,
			    (int) servo_howmany,
			    (int) servo_shm_key,
			    (int) servo_sem_key, 
			    (int) traj_shm_key,
			    kinematics,
			    (int) go_log_shm_key,
			    (int) go_io_shm_key);
    if (result >= sizeof(path)) {
      fprintf(stderr, "gorun: gomain command too long\n");
      return 1;
    }
    gomain_proc = ulapi_process_new();
    if (ULAPI_OK != ulapi_process_start(gomain_proc, path)) {
      ulapi_process_delete(gomain_proc);
      gomain_proc = NULL;
      fprintf(stderr, "gorun: can't run gomain process\n");
      return 1;
    }
  }

  if (0 != tool_shm_key) {
    if (USE_RTAI == which_ulapi) {
      result = ulapi_snprintf(path, sizeof(path)-1,
			      "sudo insmod -f %s%s%s%s%s%s%s DEBUG=%d TOOL_SHM_KEY=%d", 
			      dirname, ulapi_pathsep, "..", ulapi_pathsep, "rtlib", ulapi_pathsep, "toolmain_mod.ko",
			      debug_arg ? 1 : 0,
			      (int) tool_shm_key);
      if (result >= sizeof(path)) {
	fprintf(stderr, "gorun: install tool main command too long\n");
	return 1;
      }
      result = ulapi_system(path, &sysresult);
      if (GO_RESULT_OK != result || 0 != sysresult) {
	fprintf(stderr, "gorun: can't execute install tool main command\n");
	return 1;
      }
    } else {
      result = ulapi_snprintf(path, sizeof(path)-1,
			      "%s%s%s DEBUG=%d TOOL_SHM_KEY=%d", 
			      dirname, ulapi_pathsep, "toolmain",
			      debug_arg ? 1 : 0,
			      (int) tool_shm_key);
      if (result >= sizeof(path)) {
	fprintf(stderr, "gorun: toolmain command too long\n");
	return 1;
      }
      toolmain_proc = ulapi_process_new();
      if (ULAPI_OK != ulapi_process_start(toolmain_proc, path)) {
	ulapi_process_delete(toolmain_proc);
	toolmain_proc = NULL;
	fprintf(stderr, "gorun: can't run toolmain process\n");
	return 1;
      }
    }
  }

  if (0 != task_shm_key) {
    result = ulapi_snprintf(path, sizeof(path)-1,
			    "%s%s%s -i %s %s %s",
			    dirname, ulapi_pathsep, "taskmain",
			    inifile_name,
			    debug_arg ? "-d" : "",
			    ularg);
    if (result >= sizeof(path)) {
      fprintf(stderr, "gorun: taskmain command too long\n");
      return 1;
    }
    taskmain_proc = ulapi_process_new();
    if (ULAPI_OK != ulapi_process_start(taskmain_proc, path)) {
      ulapi_process_delete(taskmain_proc);
      taskmain_proc = NULL;
      fprintf(stderr, "gorun: can't run taskmain process\n");
      return 1;
    }

    /* only run tasksvr if task is running */
    if (0 != task_tcp_port) {
      result = ulapi_snprintf(path, sizeof(path)-1,
			      "%s%s%s -p %d -i %s %s %s",
			      dirname, ulapi_pathsep, "tasksvr",
			      (int) task_tcp_port,
			      inifile_name,
			      debug_arg ? "-d" : "",
			      ularg);
      if (result >= sizeof(path)) {
	fprintf(stderr, "gorun: tasksvr command too long\n");
	return 1;
      }
      tasksvr_proc = ulapi_process_new();
      if (ULAPI_OK != ulapi_process_start(tasksvr_proc, path)) {
	ulapi_process_delete(tasksvr_proc);
	tasksvr_proc = NULL;
	fprintf(stderr, "gorun: can't run tasksvr process\n");
	return 1;
      }
    }
  }

  /* wait a bit before running gocfg and gosteppercfg */
  ulapi_sleep(1);

  result = ulapi_snprintf(path, sizeof(path)-1,
			  "%s%s%s -i %s %s %s",
			  dirname, ulapi_pathsep, "gocfg",
			  inifile_name,
			  debug_arg ? "-d" : "",
			  ularg);
  if (result >= sizeof(path)) {
    fprintf(stderr, "gorun: gocfg command too long\n");
    return 1;
  }
  result = ulapi_system(path, &sysresult);
  if (GO_RESULT_OK != result || 0 != sysresult) {
    fprintf(stderr, "gorun: can't execute gocfg command\n");
    return 1;
  }

  /*  */
  if (0 != go_stepper_shm_key) {
    result = ulapi_snprintf(path, sizeof(path)-1,
			    "%s%s%s -i %s %s %s",
			    dirname, ulapi_pathsep, "gosteppercfg",
			    inifile_name,
			    debug_arg ? "-d" : "",
			    ularg);
    if (result >= sizeof(path)) {
      fprintf(stderr, "gorun: gosteppercfg command too long\n");
      return 1;
    }
    result = ulapi_system(path, &sysresult);
    if (GO_RESULT_OK != result || 0 != sysresult) {
      fprintf(stderr, "gorun: can't execute gosteppercfg command\n");
      return 1;
    }
  }

  if (wait_arg) {
    if (NULL == gomain_proc) {
      /* must be a real-time process, so just wait for a signal */
      ulapi_app_wait();
    } else {
      retval = ulapi_process_wait(gomain_proc, &result);
      if (ULAPI_OK == retval) {
	printf("gorun: main process returned with result %d\n", (int) result);
      } else {
	printf("gorun: error waiting for the main process\n");
      }
    }
  } else {
    if (shell_arg) {
      result = ulapi_snprintf(path, sizeof(path)-1,
			      "%s%s%s -i %s %s",
			      dirname, ulapi_pathsep, "gosh",
			      inifile_name,
			      ularg);
    } else {
      result = ulapi_snprintf(path, sizeof(path)-1,
			      "%s%s%s %s%s%s -- -i %s %s",
			      dirname, ulapi_pathsep, "gotk",
			      dirname, ulapi_pathsep, "pendant.tcl",
			      inifile_name,
			      ularg);
    }
    if (result >= sizeof(path)) {
      fprintf(stderr, "gorun: gotk pendant command too long\n");
      return 1;
    }

    gui_proc = ulapi_process_new();
    if (ULAPI_OK != ulapi_process_start(gui_proc, path)) {
      ulapi_process_delete(gui_proc);
      gui_proc = NULL;
      fprintf(stderr, "gorun: can't run GUI process\n");
      return 1;
    }
    retval = ulapi_process_wait(gui_proc, &result);
    if (ULAPI_OK == retval) {
      printf("gorun: GUI process returned with result %d\n", (int) result);
    } else {
      printf("gorun: error waiting for the GUI process\n");
    }
  }

  return 0;
}
