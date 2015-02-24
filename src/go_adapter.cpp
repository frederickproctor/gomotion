/*
 * Copyright (c) 2008, AMT – The Association For Manufacturing
 * Technology (“AMT”).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the AMT nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * DISCLAIMER OF WARRANTY. ALL MTCONNECT MATERIALS AND SPECIFICATIONS
 * PROVIDED BY AMT, MTCONNECT OR ANY PARTICIPANT TO YOU OR ANY PARTY ARE
 * PROVIDED "AS IS" AND WITHOUT ANY WARRANTY OF ANY KIND. AMT, MTCONNECT,
 * AND EACH OF THEIR RESPECTIVE MEMBERS, OFFICERS, DIRECTORS, AFFILIATES,
 * SPONSORS, AND AGENTS (COLLECTIVELY, THE "AMT PARTIES") AND
 * PARTICIPANTS MAKE NO REPRESENTATION OR WARRANTY OF ANY KIND WHATSOEVER
 * RELATING TO THESE MATERIALS, INCLUDING, WITHOUT LIMITATION, ANY
 * EXPRESS OR IMPLIED WARRANTY OF NONINFRINGEMENT, MERCHANTABILITY, OR
 * FITNESS FOR A PARTICULAR PURPOSE.
 * 
 * LIMITATION OF LIABILITY. IN NO EVENT SHALL AMT, MTCONNECT, ANY OTHER
 * AMT PARTY, OR ANY PARTICIPANT BE LIABLE FOR THE COST OF PROCURING
 * SUBSTITUTE GOODS OR SERVICES, LOST PROFITS, LOSS OF USE, LOSS OF DATA
 * OR ANY INCIDENTAL, CONSEQUENTIAL, INDIRECT, SPECIAL OR PUNITIVE
 * DAMAGES OR OTHER DIRECT DAMAGES, WHETHER UNDER CONTRACT, TORT,
 * WARRANTY OR OTHERWISE, ARISING IN ANY WAY OUT OF THIS AGREEMENT, USE
 * OR INABILITY TO USE MTCONNECT MATERIALS, WHETHER OR NOT SUCH PARTY HAD
 * ADVANCE NOTICE OF THE POSSIBILITY OF SUCH DAMAGES.
 */

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>		// atoi
#include <signal.h>
#include <ulapi.h>
#include <inifile.h>
#include "go.h"
#include "trajintf.h"
#include "taskintf.h"
#include "toolintf.h"
#include "variates.h"

#include <internal.hpp>
#include <server.hpp>
#include <string_buffer.hpp>
#include "go_adapter.hpp"

#define ROUND(x) ((x) < 0 ? ((int) ((x) - 0.5)) : ((int) ((x) + 0.5)))


/*
  'dbprintf' is debug printf that shows what's going on during init
  and shutdown. Fatal errors are printed regardless.
*/
static int dbflag = 0;
static void dbprintf(int prefix, const char * fmt, ...)
{
  va_list ap;
  FILE *dbdst = stdout;

  if (dbflag) {
    if (prefix) {
      fprintf(dbdst, "go_adapter: ");
    }
    va_start(ap, fmt);
    vfprintf(dbdst, fmt, ap);
    fflush(dbdst);
    va_end(ap);
  }
}

GoAdapter::GoAdapter(int aPort, int aScanDelay) : Adapter(aPort, aScanDelay), mAvailability("availability"), mExecution("execution"), mCondition("condition"), mHeartbeat("heartbeat"), mSrpm("Srpm"), mFover("Fover"), mPosition("position"), mProgram("program"), mPartCount("PartCount"), mPower("power"), mMode("mode")
{
  int t;
  char name[] = "Ferr123456789"; // enough for a billion joints

  addDatum(mAvailability);
  addDatum(mExecution);
  addDatum(mCondition);
  addDatum(mHeartbeat);
  addDatum(mSrpm);
  addDatum(mFover);
  addDatum(mPosition);
  addDatum(mProgram);
  addDatum(mPartCount);
  addDatum(mPower);
  addDatum(mMode);

  for (t = 0; t < SERVO_NUM; t++) {
    ulapi_snprintf(name, sizeof(name) - 1, "Ferr%d", t+1);
    mFerr[t] = new Sample(name);
    addDatum(*(mFerr[t]));
  }
}

GoAdapter::~GoAdapter()
{
}

void GoAdapter::initialize(int aArgc, const char *aArgv[])
{
  MTConnectService::initialize(aArgc, aArgv);
  if (aArgc > 1) {
    mPort = atoi(aArgv[1]);
  }
}

void GoAdapter::start()
{
  startServer();
}

void GoAdapter::stop()
{
  stopServer();
}

static task_comm_struct *task_comm_ptr;
static task_stat_struct pp_task_stat[2], *task_stat_ptr, *task_stat_test;
static void *task_shm = NULL;

static traj_comm_struct *traj_comm_ptr;
static traj_stat_struct pp_traj_stat[2], *traj_stat_ptr, *traj_stat_test;
static traj_set_struct pp_traj_set[2], *traj_set_ptr, *traj_set_test;
static void *traj_shm = NULL;

static tool_comm_struct *tool_comm_ptr;
static tool_stat_struct pp_tool_stat[2], *tool_stat_ptr, *tool_stat_test;
static void *tool_shm = NULL;

static int update_go_status(void)
{
  task_stat_struct *task_tmp;
  traj_stat_struct *traj_stat_tmp;
  traj_set_struct *traj_set_tmp;
  tool_stat_struct *tool_tmp;

  *task_stat_test = task_comm_ptr->task_stat;
  if (task_stat_test->head == task_stat_test->tail) {
    task_tmp = task_stat_ptr;
    task_stat_ptr = task_stat_test;
    task_stat_test = task_tmp;
  }

  *traj_stat_test = traj_comm_ptr->traj_stat;
  if (traj_stat_test->head == traj_stat_test->tail) {
    traj_stat_tmp = traj_stat_ptr;
    traj_stat_ptr = traj_stat_test;
    traj_stat_test = traj_stat_tmp;
  }

  *traj_set_test = traj_comm_ptr->traj_set;
  if (traj_set_test->head == traj_set_test->tail) {
    traj_set_tmp = traj_set_ptr;
    traj_set_ptr = traj_set_test;
    traj_set_test = traj_set_tmp;
  }

  *tool_stat_test = tool_comm_ptr->tool_stat;
  if (tool_stat_test->head == tool_stat_test->tail) {
    tool_tmp = tool_stat_ptr;
    tool_stat_ptr = tool_stat_test;
    tool_stat_test = tool_tmp;
  }
}

void GoAdapter::gatherDeviceData()
{
  static double last_go_timestamp = 0;
  static int wasExecuting = 0;
  static int partCount = 0;
  char buffer[DIGITS_IN(int)];

  update_go_status();

  mAvailability.available();
  mPower.setValue("ON");

  /*
    Set 'Execution' to one of
    eUNAVAILABLE,
    eREADY,
    eINTERRUPTED,
    eSTOPPED,
    eACTIVE,
    eFEED_HOLD
  */

  switch (task_stat_ptr->state_model) {
  case TASK_STATE_ABORTING:
  case TASK_STATE_ABORTED:
  case TASK_STATE_CLEARING:
    mExecution.setValue(Execution::eINTERRUPTED);
    break;

  case TASK_STATE_HOLDING:
  case TASK_STATE_HELD:
  case TASK_STATE_UNHOLDING:
  case TASK_STATE_SUSPENDING:
  case TASK_STATE_SUSPENDED:
  case TASK_STATE_UNSUSPENDING:
    mExecution.setValue(Execution::eFEED_HOLD);
    break;

  case TASK_STATE_COMPLETE:
  case TASK_STATE_STOPPING:
  case TASK_STATE_STOPPED:
  case TASK_STATE_RESETTING:
    mExecution.setValue(Execution::eSTOPPED);
    break;

  case TASK_STATE_STARTING:
  case TASK_STATE_EXECUTE:
  case TASK_STATE_COMPLETING:
    mExecution.setValue(Execution::eACTIVE);
    wasExecuting = 1;
    break;

  case TASK_STATE_IDLE:
    mExecution.setValue(Execution::eREADY);
    if (wasExecuting) {
      wasExecuting = 0;
      partCount++;
      ulapi_snprintf(buffer, sizeof(buffer), "%d", partCount);
      mPartCount.setValue(buffer);
      dbprintf(1, "setting part count to %s\n", buffer);
    }
    break;

  default:
    mPower.setValue("OFF");
    mExecution.setValue(Execution::eUNAVAILABLE);
    break;
  }

  /*
    Set 'Condition' to one of

    eUNAVAILABLE,
    eNORMAL,
    eWARNING,
    eFAULT
  */

  // number of elements in an array
#define ARRAY_ELS(a) (sizeof(a) / sizeof(*(a)))

#define  USE_TRIGGER 0

#if USE_TRIGGER

  // report task errors from oldest to newest, where the oldest is at the
  // 'error_index' and we need to wrap around and do this in two chunks
  for (int t = task_stat_ptr->error_index; t < ARRAY_ELS(task_stat_ptr->error); t++) {
#define CONDIT								\
    if (task_stat_ptr->error[t].timestamp > last_go_timestamp) {	\
      last_go_timestamp = task_stat_ptr->error[t].timestamp;		\
      if (task_stat_ptr->error[t].code == TASK_ERROR_NONE) {		\
	mCondition.add(Condition::eNORMAL);				\
	dbprintf(1, "setting condition to NORMAL\n");			\
      } else {								\
	mCondition.add(Condition::eFAULT, task_error_symbol(task_stat_ptr->error[t].code), "", "", "");	\
	dbprintf(1, "setting condition to FAULT\n");			\
      }									\
    }
    CONDIT;
  }
  // here's the second chunk
  for (int t = 0; t < task_stat_ptr->error_index; t++) {
    CONDIT;
  }

#else

  int index;
  if (task_stat_ptr->error_index == 0) {
    index = ARRAY_ELS(task_stat_ptr->error) - 1;
  } else {
    index = task_stat_ptr->error_index - 1;
  }

  if (task_stat_ptr->error[index].code == TASK_ERROR_NONE) {
    mCondition.add(Condition::eNORMAL);
    dbprintf(1, "setting condition to NORMAL\n");
  } else {
    mCondition.add(Condition::eFAULT, task_error_symbol(task_stat_ptr->error[index].code), "", "", "");
    dbprintf(1, "setting condition to FAULT\n");
  }

#endif

#undef DO_DYNAMIC

#ifdef DO_DYNAMIC

  mHeartbeat.setValue(task_stat_ptr->heartbeat);

  mSrpm.setValue(tool_stat_ptr->value);

  mFover.setValue(traj_set_ptr->scale);

  for (int t = 0; t < SERVO_NUM; t++) {
    mFerr[t]->setValue(traj_stat_ptr->joints_ferror[t]);
  }

  mPosition.setValue(traj_stat_ptr->ecp_act.tran.x,
		     traj_stat_ptr->ecp_act.tran.y,
		     traj_stat_ptr->ecp_act.tran.z);

#endif	// DO_DYNAMIC

  mProgram.setValue(task_stat_ptr->program);

  /*
    Set 'ControllerMode' to one of

    eUNAVAILABLE,
    eAUTOMATIC,
    eMANUAL,
    eMANUAL_DATA_INPUT,
    eSEMI_AUTOMATIC
  */

  switch (task_stat_ptr->state_model) {
  case TASK_STATE_ABORTING:
  case TASK_STATE_ABORTED:
  case TASK_STATE_CLEARING:
    mMode.setValue(ControllerMode::eUNAVAILABLE);
    break;

  case TASK_STATE_HOLDING:
  case TASK_STATE_HELD:
  case TASK_STATE_UNHOLDING:
  case TASK_STATE_SUSPENDING:
  case TASK_STATE_SUSPENDED:
  case TASK_STATE_UNSUSPENDING:
  case TASK_STATE_STARTING:
  case TASK_STATE_EXECUTE:
  case TASK_STATE_COMPLETING:
    mMode.setValue(ControllerMode::eAUTOMATIC);
    break;

  case TASK_STATE_COMPLETE:
  case TASK_STATE_STOPPING:
  case TASK_STATE_STOPPED:
  case TASK_STATE_RESETTING:
  case TASK_STATE_IDLE:
    mMode.setValue(ControllerMode::eMANUAL);
    break;

  default:
    mPower.setValue("OFF");
    mMode.setValue(ControllerMode::eUNAVAILABLE);
    break;
  }

  return;
}

void print_help(void)
{
  printf("-i <ini file> , provide an initialization file\n");
  printf("-u unix | rtai , specify which Unix subsystem Go Motion is running on\n");
  printf("-c <connect time> , how long to wait in seconds to connect to Go Motion\n");
  printf("-p <port number>, TCP port number to use\n");
  printf("-t <period in seconds>, polling time period\n");
  printf("-d , turn debug on\n");
  printf("-? , print this help message\n");

  printf("-- , arguments for the MTConnect agent, e.g., debug, install, remove, run\n");
}

/*
  Usage: go_adaptor 
  -i <ini file> , provide an initialization file
  -u unix | rtai , specify which Unix subsystem Go Motion is running on
  -c <connect time> , how long to wait in seconds to connect to Go Motion
  -p <port number>, TCP port number to use
  -t <period in seconds>, polling time period
  -d , turn debug on
  -? , print this help message

  -- , arguments for the MTConnect agent, e.g., debug, install, remove, run
 */

int main(int argc, char *argv[])
{    
#define DEFAULT_PORT 7878
#define DEFAULT_PERIOD 0.2
  int port = DEFAULT_PORT;
  double period = DEFAULT_PERIOD;
  int option;
  const char *ini_string;
  enum { BUFFERLEN = 80 };
  char inifile_name[BUFFERLEN] = "gomotion.ini";
  ulapi_integer sel;
  FILE *fp;
  int task_shm_key, traj_shm_key, tool_shm_key;
  int start_it;
  int got_it;
  int heartbeat;
  double connect_wait_time = 10.0;
  double end;
  GoAdapter *adapter;

  sel = UL_USE_DEFAULT;
  opterr = 0;

  while (1) {
    option = getopt(argc, argv, ":i:u:c:p:t:d?");
    if (option == -1)
      break;

    switch (option) {
    case 'i':
      strncpy(inifile_name, optarg, BUFFERLEN);
      inifile_name[BUFFERLEN - 1] = 0;
      break;

    case 'u':
      if (! strcmp(optarg, "unix")) {
	sel = UL_USE_UNIX;
      } else if (! strcmp(optarg, "rtai")) {
	sel = UL_USE_RTAI;
      } else {
	fprintf(stderr, "go_adapter: bad value for -%c: %s\n", option, optarg);
	return 1;
      }
      break;

    case 'c':
      connect_wait_time = strtod(optarg, NULL);
      break;

    case 'p':
      port = atoi(optarg);
      break;

    case 't':
      period = atof(optarg);
      break;

    case 'd':
      dbflag = 1;
      break;

    case '?':
      print_help();
      return 0;
      break;

    case ':':
      fprintf(stderr, "go_adapter: missing value for -%c\n", optopt);
      return 1;
      break;

    default:			/* '?' */
      fprintf (stderr, "go_adapter: unrecognized option -%c\n", optopt);
      return 1;
      break;
    }
  }

  // mark the remaining arguments at 'optind' and beyond to be passed
  // to MTConnectService::main()
  argv[optind-1] = argv[0];	// copy the real first arg over
  argc -= optind-1;		// update the count
  argv += optind-1;		// and first argument pointer

   if (ULAPI_OK != ulapi_init(sel)) {
     return 1;
   } 

   if (go_init()) {
     fprintf(stderr, "go_adapter: go_init error\n");
     return 1;
   }

   if (NULL == (fp = fopen(inifile_name, "r"))) {
     fprintf(stderr, "go_adapter: can't open %s\n", inifile_name);
     return 1;
   }

  ini_string = ini_find(fp, "SHM_KEY", "TASK");
  if (NULL == ini_string) {
    fprintf(stderr, "go_adapter: [TASK] SHM_KEY not found\n");
    return 1;
  } else if (1 != sscanf(ini_string, "%i", &task_shm_key)) {
    fprintf(stderr, "go_adapter: bad entry: [TASK] SHM_KEY = %s\n", ini_string);
    return 1;
  }

  ini_string = ini_find(fp, "SHM_KEY", "TRAJ");
  if (NULL == ini_string) {
    fprintf(stderr, "go_adapter: [TRAJ] SHM_KEY not found\n");
    return 1;
  } else if (1 != sscanf(ini_string, "%i", &traj_shm_key)) {
    fprintf(stderr, "go_adapter: bad entry: [TRAJ] SHM_KEY = %s\n", ini_string);
    return 1;
  }

  ini_string = ini_find(fp, "SHM_KEY", "TOOL");
  if (NULL == ini_string) {
    fprintf(stderr, "go_adapter: [TOOL] SHM_KEY not found\n");
    return 1;
  } else if (1 != sscanf(ini_string, "%i", &tool_shm_key)) {
    fprintf(stderr, "go_adapter: bad entry: [TOOL] SHM_KEY = %s\n", ini_string);
    return 1;
  }

  task_shm = ulapi_rtm_new(task_shm_key, sizeof(task_comm_struct));
  if (NULL == task_shm) {
    fprintf(stderr, "go_adapter: can't get task comm shm\n");
    return 1;
  }
  task_comm_ptr = (task_comm_struct *) ulapi_rtm_addr(task_shm);
  task_stat_ptr = &pp_task_stat[0];
  task_stat_test = &pp_task_stat[1];
  for (start_it = 0, got_it = 0, end = ulapi_time() + connect_wait_time;
       ulapi_time() < end;
       ulapi_sleep(0.1)) {
    *task_stat_ptr = task_comm_ptr->task_stat;
    if (task_stat_ptr->head == task_stat_ptr->tail &&
	task_stat_ptr->type == TASK_STAT_TYPE) {
      if (! start_it) {
	start_it = 1;
	heartbeat = task_stat_ptr->heartbeat;
      }
      if (heartbeat != task_stat_ptr->heartbeat) {
	got_it = 1;
	break;
      }
    }
  }
  if (! got_it) {
    fprintf(stderr, "go_adapter: timed out connecting to task status\n");
    return 1;
  }

  traj_shm = ulapi_rtm_new(traj_shm_key, sizeof(traj_comm_struct));
  if (NULL == traj_shm) {
    fprintf(stderr, "go_adapter: can't get traj comm shm\n");
    return 1;
  }
  traj_comm_ptr = (traj_comm_struct *) ulapi_rtm_addr(traj_shm);

  traj_stat_ptr = &pp_traj_stat[0];
  traj_stat_test = &pp_traj_stat[1];
  for (start_it = 0, got_it = 0, end = ulapi_time() + connect_wait_time;
       ulapi_time() < end;
       ulapi_sleep(0.1)) {
    *traj_stat_ptr = traj_comm_ptr->traj_stat;
    if (traj_stat_ptr->head == traj_stat_ptr->tail &&
	traj_stat_ptr->type == TRAJ_STAT_TYPE) {
      if (! start_it) {
	start_it = 1;
	heartbeat = traj_stat_ptr->heartbeat;
      }
      if (heartbeat != traj_stat_ptr->heartbeat) {
	got_it = 1;
	break;
      }
    }
  }
  if (! got_it) {
    fprintf(stderr, "go_adapter: timed out connecting to traj status\n");
    return 1;
  }

  traj_set_ptr = &pp_traj_set[0];
  traj_set_test = &pp_traj_set[1];
  for (start_it = 0, got_it = 0, end = ulapi_time() + connect_wait_time;
       ulapi_time() < end;
       ulapi_sleep(0.1)) {
    *traj_set_ptr = traj_comm_ptr->traj_set;
    if (traj_set_ptr->head == traj_set_ptr->tail &&
	traj_set_ptr->type == TRAJ_SET_TYPE) {
      got_it = 1;
      break;
    }
  }
  if (! got_it) {
    fprintf(stderr, "go_adapter: timed out connecting to traj settings\n");
    return 1;
  }

  tool_shm = ulapi_rtm_new(tool_shm_key, sizeof(tool_comm_struct));
  if (NULL == tool_shm) {
    fprintf(stderr, "go_adapter: can't get tool comm shm\n");
    return 1;
  }
  tool_comm_ptr = (tool_comm_struct *) ulapi_rtm_addr(tool_shm);
  tool_stat_ptr = &pp_tool_stat[0];
  tool_stat_test = &pp_tool_stat[1];
  for (start_it = 0, got_it = 0, end = ulapi_time() + connect_wait_time;
       ulapi_time() < end;
       ulapi_sleep(0.1)) {
    *tool_stat_ptr = tool_comm_ptr->tool_stat;
    if (tool_stat_ptr->head == tool_stat_ptr->tail &&
	tool_stat_ptr->type == TOOL_STAT_TYPE) {
      if (! start_it) {
	start_it = 1;
	heartbeat = tool_stat_ptr->heartbeat;
      }
      if (heartbeat != tool_stat_ptr->heartbeat) {
	got_it = 1;
	break;
      }
    }
  }
  if (! got_it) {
    fprintf(stderr, "go_adapter: timed out connecting to tool status\n");
    return 1;
  }

  adapter = new GoAdapter(port, ROUND(period * 1000));
  adapter->setName("Go MTConnect Adapter");

  if (dbflag) {
    dbprintf(1, "running MTConnectService::main(%s", argv[0]);
    for (int t = 1; t < argc; t++) {
      dbprintf(0, " %s", argv[t]);
    }
    dbprintf(0, ")\n");
  }

  signal(SIGINT, SIG_DFL);

  return adapter->main(argc, (const char **) argv);
}

