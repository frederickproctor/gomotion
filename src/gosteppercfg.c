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
  \file gosteppercfg.c

  \brief Interactive shell to stepper motor external interface
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>		/* f,printf, stderr */
#include <string.h>		/* strncpy, strcmp */
#include <stdarg.h>		/* va_list */
#include <inifile.h>
#include <ulapi.h>		/* ulapi_time */
#include "gotypes.h"
#include "gostepper.h"

#define CONNECT_WAIT_TIME 3.0

/*
  'dbprintf' is debug printf that shows what's going on during init
  and shutdown. Fatal errors are printed regardless.
*/
static int dbflag = 0;
static void dbprintf(const char * fmt, ...)
{
  va_list ap;

  if (dbflag) {
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    fflush(stderr);
    va_end(ap);
  }
}

static int ini_load(char * inifile_name,
		    int * shm_key,
		    int * lo_port, int * hi_port,
		    int * min_up_count, int * min_down_count,
		    int * count_on_up)
{
  FILE * fp;
  const char * inistring;
  char section[] = "SERVO_999"; /* see sprintf below for check on size */
  int servo_num;

  if (NULL == (fp = fopen(inifile_name, "r"))) {
    fprintf(stderr, "gosteppercfg: can't open %s\n", inifile_name);
    return 1;
  }

  inistring = ini_find(fp, "SHM_KEY", "GO_STEPPER");
  if (NULL == inistring) {
    fprintf(stderr, "gosteppercfg: [GO_STEPPER] SHM_KEY not found in %s\n", inifile_name);
#undef CLOSE_AND_RETURN
#define CLOSE_AND_RETURN \
    fclose(fp); \
    return 1
    CLOSE_AND_RETURN;
  } else if (1 != sscanf(inistring, "%i", shm_key)) {
    fprintf(stderr, "gosteppercfg: bad [GO_STEPPER] SHM_KEY in %s: %s\n", inifile_name, inistring);
    CLOSE_AND_RETURN;
  }

  inistring = ini_find(fp, "LO_PORT", "GO_STEPPER");
  if (NULL == inistring) {
    fprintf(stderr, "gosteppercfg: [GO_STEPPER] LO_PORT not found in %s\n", inifile_name);
#undef CLOSE_AND_RETURN
#define CLOSE_AND_RETURN \
    fclose(fp); \
    return 1
    CLOSE_AND_RETURN;
  } else if (1 != sscanf(inistring, "%i", lo_port)) {
    fprintf(stderr, "gosteppercfg: bad [GO_STEPPER] LO_PORT in %s: %s\n", inifile_name, inistring);
    CLOSE_AND_RETURN;
  }

  inistring = ini_find(fp, "HI_PORT", "GO_STEPPER");
  if (NULL == inistring) {
    fprintf(stderr, "gosteppercfg: [GO_STEPPER] HI_PORT not found in %s\n", inifile_name);
    CLOSE_AND_RETURN;
  } else if (1 != sscanf(inistring, "%i", hi_port)) {
    fprintf(stderr, "gosteppercfg: bad [GO_STEPPER] HI_PORT in %s: %s\n", inifile_name, inistring);
    CLOSE_AND_RETURN;
  }

  for (servo_num = 0; servo_num < GO_STEPPER_NUM; servo_num++) {
    if (servo_num >= 999) {	/* see servo_num[] decl above */
      fprintf(stderr, "gosteppercfg: ignoring entries above %d in %s\n", servo_num, inifile_name);
      break;
    }
    sprintf(section, "SERVO_%d", servo_num + 1);
    inistring = ini_find(fp, "MIN_UP_COUNT", section);
    if (NULL == inistring) {
      fprintf(stderr, "gosteppercfg: [%s] MIN_UP_COUNT not found in %s\n", section, inifile_name);
      CLOSE_AND_RETURN;
    } else if (1 != sscanf(inistring, "%i", &min_up_count[servo_num])) {
      fprintf(stderr, "gosteppercfg: bad %s MIN_UP_COUNT in %s: %s\n", section, inifile_name, inistring);
      CLOSE_AND_RETURN;
    }

    inistring = ini_find(fp, "MIN_DOWN_COUNT", section);
    if (NULL == inistring) {
      fprintf(stderr, "gosteppercfg: [%s] MIN_DOWN_COUNT not found in %s\n", section, inifile_name);
      CLOSE_AND_RETURN;
    } else if (1 != sscanf(inistring, "%i", &min_down_count[servo_num])) {
      fprintf(stderr, "gosteppercfg: bad [%s] MIN_DOWN_COUNT in %s: %s\n", section, inifile_name, inistring);
      CLOSE_AND_RETURN;
    }

    inistring = ini_find(fp, "COUNT_ON_UP", section);
    if (NULL == inistring) {
      fprintf(stderr, "gosteppercfg: [%s] COUNT_ON_UP not found in %s\n", section, inifile_name);
      CLOSE_AND_RETURN;
    } else if (1 != sscanf(inistring, "%i", &count_on_up[servo_num])) {
      fprintf(stderr, "gosteppercfg: bad [%s] COUNT_ON_UP in %s: %s\n", section, inifile_name, inistring);
      CLOSE_AND_RETURN;
    }
  }

  fclose(fp);
  return 0;
}

/*
  Options:

  -i <inifile>   : use <inifile>
  -u unix | rtai : select a ULAPI implementation
  -p             : go interactive, with a prompt
  -d             : print debug info
*/

int main(int argc, char *argv[])
{
  enum {BUFFERLEN = 80};
  int option;
  char inifile_name[BUFFERLEN] = "gomotion.ini";
  int interactive;
  char buffer[BUFFERLEN];
  int gss_shm_key;
  void * gss_shm;
  go_stepper_struct dummy;	/* so it's never null, avoiding checks */
  go_stepper_struct * gss_ptr = &dummy;
  go_integer heartbeat;
  int got_it;
  double end;
  int lo_port, hi_port;
  int min_up_count[GO_STEPPER_NUM];
  int min_down_count[GO_STEPPER_NUM];
  int count_on_up[GO_STEPPER_NUM];
  int joint;
  int i1, i2;

  interactive = 0;

  opterr = 0;
  while (1) {
    option = getopt(argc, argv, ":i:u:pd");
    if (option == -1)
      break;

    switch (option) {
    case 'i':
      strncpy(inifile_name, optarg, BUFFERLEN);
      inifile_name[BUFFERLEN - 1] = 0;
      break;

    case 'p':
      interactive = 1;
      break;

    case 'd':
      dbflag = 1;
      break;

    case ':':
      fprintf(stderr, "gosteppercfg: missing value for -%c\n", optopt);
      return 1;
      break;

    default:			/* '?' */
      fprintf (stderr, "gosteppercfg: unrecognized option -%c\n", optopt);
      return 1;
      break;
    }
  }
  if (optind < argc) {
    fprintf(stderr, "gosteppercfg: extra non-option characters: %s\n", argv[optind]);
    return 1;
  }

  if (0 != ini_load(inifile_name,
		    &gss_shm_key,
		    &lo_port, &hi_port,
		    min_up_count, min_down_count, count_on_up)) {
    return 1;
  }

  if (ULAPI_OK != ulapi_init()) {
    return 1;
  } 

  gss_shm = ulapi_rtm_new(gss_shm_key, sizeof(go_stepper_struct));
  if (NULL == gss_shm) {
    fprintf(stderr, "gosteppercfg: can't get stepper controller shared memory\n");
    return 1;
  }
  gss_ptr = ulapi_rtm_addr(gss_shm);
  heartbeat = gss_ptr->heartbeat;

  /* wait for stepper controller to run, by looking at heartbeat */
  for (got_it = 0, end = ulapi_time() + CONNECT_WAIT_TIME;
       ulapi_time() < end;
       ulapi_sleep(0.001)) {
    if (gss_ptr->heartbeat != heartbeat) {
      got_it = 1;
      break;
    }
  }
  if (! got_it) {
    fprintf(stderr, "gosteppercfg: timed out connecting to stepper controller\n");
    return 1;
  }

  gss_ptr->lo_port = (rtapi_integer) lo_port;
  gss_ptr->hi_port = (rtapi_integer) hi_port;
  dbprintf("gosteppercfg: setting go_stepper_struct lo,hi port to %X %X\n", lo_port, hi_port);
  for (joint = 0; joint < GO_STEPPER_NUM; joint++) {
    gss_ptr->min_up_count[joint] = (rtapi_integer) min_up_count[joint];
    gss_ptr->min_down_count[joint] = (rtapi_integer) min_down_count[joint];
    gss_ptr->count_on_up[joint] = (rtapi_integer) count_on_up[joint];
    dbprintf("gosteppercfg: setting go_stepper_struct counts %d to %d %d %d\n", joint + 1, min_up_count[joint], min_down_count[joint], count_on_up[joint]);
  }

  /* if we're interactive, we want to read and write, so why not
     use the same external interface functions */
  if (interactive) {
    while (! feof(stdin)) {
      if (NULL == fgets(buffer, BUFFERLEN, stdin)) {
	break;
      }
      if (2 == sscanf(buffer, "%i %i", &i1, &i2)) {
	if (i1 < 1) i1 = 1;
	else if (i1 > GO_STEPPER_NUM) i1 = GO_STEPPER_NUM;
	i1--;
	gss_ptr->freq[i1] = (rtapi_integer) i2;
      } else {
	printf("%f\n", (double) gss_ptr->count[i1]);
      }
    }
  }

  if (NULL != gss_shm) {
    ulapi_rtm_delete(gss_shm);
  }

  return (int) ulapi_exit();
}
