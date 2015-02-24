/*
  ASCII protocol: every message has a null or newline at the end

  Commands from client:
  [ ! <id> init ]
  [ ! <id> stop ]
  [ ! <id> run <program> ]
  
  Status request from client:
  [ ? ]

  Response to client:
  [ <id> <command> done | exec | error ]
*/

#include <stdio.h>		/* stdin, stderr */
#include <stddef.h>		/* NULL, sizeof */
#include <stdlib.h>		/* malloc, free, atoi */
#include <string.h>		/* strncpy */
#include <stdarg.h>		/* va_list */
#include <ctype.h>
#include <signal.h>
#include <setjmp.h>
#include "ulapi.h"
#include "inifile.h"
#include "go.h"
#include "gorcs.h"
#include "taskintf.h"

#define FILENAME_LEN 256
#define DEFAULT_INI_FILE "gomotion.ini"
#define CONNECT_WAIT_TIME 3.0

#define SAFECPY(dst,src) strncpy(dst,src,sizeof(dst)); (dst)[sizeof(dst)-1] = 0

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
      fprintf(dbdst, "tasksvr: ");
    }
    va_start(ap, fmt);
    vfprintf(dbdst, fmt, ap);
    fflush(dbdst);
    va_end(ap);
  }
}

static jmp_buf env;
static void quit(int sig)
{
  longjmp(env, 1);
}

typedef struct {
  void *task;
  ulapi_integer client_id;
  void *task_write_mutex;
  task_comm_struct *task_comm_ptr;
} client_args;

int get_task_status(task_stat_struct *dst, task_stat_struct *src, double timeout)
{
  int start_it;
  int got_it;
  int heartbeat;
  double end;

  for (start_it = 0, got_it = 0, end = ulapi_time() + timeout;
       ulapi_time() < end;
       ulapi_sleep(0.1)) {
    *dst = *src;
    if (dst->head == dst->tail &&
	dst->type == TASK_STAT_TYPE) {
      if (! start_it) {
	start_it = 1;
	heartbeat = dst->heartbeat;
      }
      if (heartbeat != dst->heartbeat) {
	got_it = 1;
	break;
      }
    }
  }

  return got_it ? 0 : 1;
}

static char is_in(char c, char *str, size_t size)
{
  while (size-- > 0) if (c == *str++) return 1;
  return 0;
}

static void client_code(void *args)
{
  void *task;
  ulapi_integer client_id;
  void *task_write_mutex;
  task_comm_struct *task_comm_ptr;
  task_cmd_struct task_cmd;
  task_stat_struct pp_task_stat[2], *task_stat_ptr, *task_stat_test;
  void *tmp;
  ulapi_integer nchars;
  enum {BUFFERLEN = 256};
  char inbuf[BUFFERLEN];
  char outbuf[BUFFERLEN];
  char *ptr;
  int serial_number;

  char DELIMITER[] = {0, '\n'};
  enum {BUILDMAX = 8192};
  int buildlen = BUFFERLEN;
  char *build = NULL;
  char *build_ptr;
  char *build_end;
  char *buffer_ptr;
  char *buffer_end;
  ptrdiff_t offset;
  char c;

  task = ((client_args *) args)->task;
  client_id = ((client_args *) args)->client_id;
  task_write_mutex = ((client_args *) args)->task_write_mutex;
  task_comm_ptr = ((client_args *) args)->task_comm_ptr;
  free(args);

  task_stat_ptr = &pp_task_stat[0];
  task_stat_test = &pp_task_stat[1];

  /* get the first good task status read */
  if (0 != get_task_status(task_stat_ptr, &task_comm_ptr->task_stat, CONNECT_WAIT_TIME)) {
    fprintf(stderr, "tasksvr: client thread can't read task status\n");
    ulapi_socket_close(client_id);
    ulapi_task_exit(0);
  }

  task_cmd.head = task_cmd.tail = 0;
  task_cmd.serial_number = task_stat_ptr->echo_serial_number + 1;
  task_cmd.type = TASK_CMD_NOP_TYPE;

  build = (char *) realloc(build, buildlen * sizeof(*build));
  build_ptr = build;
  build_end = build + buildlen;

  for (;;) {
    nchars = ulapi_socket_read(client_id, inbuf, sizeof(inbuf) - 1);
    if (-1 == nchars) {
      break;
    }
    if (0 == nchars) {
      break;
    }

    buffer_ptr = inbuf;
    buffer_end = buffer_ptr + nchars;
    while (buffer_ptr != buffer_end) {
      if (build_ptr == build_end) {
	if (buildlen > BUILDMAX) {
	  fprintf(stderr, "tasksvr: message overrun in reader\n");
	  build_ptr = build;
	  break;
	}
	offset = build_ptr - build;
	buildlen *= 2;
	build = (char *) realloc(build, buildlen * sizeof(*build));
	build_ptr = build + offset;
	build_end = build + buildlen;
      }
      *build_ptr++ = c = *buffer_ptr++;
      if (is_in(c,DELIMITER, sizeof(DELIMITER))) {
	offset = build_ptr - build;
	build_ptr = build;
	build[offset] = 0;

	/* trim off trailing white space */
	ptr = &build[offset - 1];
	while (isspace(*ptr) && ptr >= build) *ptr-- = 0;
	/* trim off leading white space */
	ptr = build;
	while (isspace(*ptr)) ptr++;

	dbprintf(1, "client message: ``%s''\n", build);

	if ('?' == *ptr) {
	  /* read in the latest task status */
	  *task_stat_test = task_comm_ptr->task_stat;
	  if (task_stat_test->head == task_stat_test->tail) {
	    tmp = task_stat_ptr;
	    task_stat_ptr = task_stat_test;
	    task_stat_test = tmp;
	  }
	  /* write out the status to the client */
	  ulapi_snprintf(outbuf, sizeof(outbuf), "%d %s\n",
			 (int) task_stat_ptr->echo_serial_number,
			 task_stat_ptr->status == GO_RCS_STATUS_DONE ? "done" :
			 task_stat_ptr->status == GO_RCS_STATUS_EXEC ? "exec" : "error");
	  ulapi_socket_write(client_id, outbuf, strlen(outbuf) + 1);
	} else if (! strncmp(ptr, "! ", 2)) {
	  ptr += 2;			/* skip the "!" and required space */
	  while (isspace(*ptr)) ptr++; /* skip white space */
	  if (1 == sscanf(ptr, "%i", &serial_number)) {
	    task_cmd.serial_number = serial_number;
	    task_cmd.head = ++task_cmd.tail;
	    while (! isspace(*ptr) && 0 != *ptr) ptr++; /* skip serial number */
	    while (isspace(*ptr)) ptr++;		    /* skip white space */
	    if (! strcmp(ptr, "init") ||
		! strcmp(ptr, "reset")) {
	      task_cmd.type = TASK_CMD_RESET_TYPE;
	      task_comm_ptr->task_cmd = task_cmd;
	    } else if (! strcmp(ptr, "stop")) {
	      task_cmd.type = TASK_CMD_STOP_TYPE;
	      task_comm_ptr->task_cmd = task_cmd;
	    } else if (! strncmp(ptr, "run ", 4)) {
	      ptr += 4;		/* skip the "run" and required space */
	      while (isspace(*ptr)) ptr++; /* and skip any more space */
	      task_cmd.type = TASK_CMD_START_TYPE;
	      strncpy(task_cmd.u.start.program, ptr, sizeof(task_cmd.u.start.program));
	      task_cmd.u.start.program[sizeof(task_cmd.u.start.program) - 1] = 0;
	      task_comm_ptr->task_cmd = task_cmd;
	    } else {
	      /* unrecognized command */
	      dbprintf(1, "unrecognized command: %s\n", ptr);
	    }
	  } else {
	    /* no serial number */
	    dbprintf(1, "no serial number: %s\n", ptr);
	  }
	} else {
	  /* unrecognized command */
	  dbprintf(1, "unrecognized message: %s\n", ptr);
	}
      }	/* if (is_in(c, DELIMITER)) */
    } /* while (buffer_ptr != buffer_end) */
  } /* for (;;) */

  dbprintf(1, "client thread done, closed %d\n", (int) client_id);

  ulapi_socket_close(client_id);

  ulapi_task_exit(0);
}

static int ini_load(char *inifile_name, ulapi_id *task_shm_key, ulapi_id *task_tcp_port)
{
  FILE *fp;
  const char *section;
  const char *key;
  const char *inistring;
  int i1;

  if (NULL == (fp = fopen(inifile_name, "r"))) {
    fprintf(stderr, "tasksvr: can't open %s\n", inifile_name);
    return 1;
  }

#define CLOSE_AND_RETURN \
  fclose(fp);		 \
  return 1

  section = "TASK";

  key = "SHM_KEY";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    fprintf(stderr, "tasksvr: missing entry: [%s] %s\n", section, key);
    i1 = DEFAULT_TASK_SHM_KEY;
  } else if (1 != sscanf(inistring, "%i", &i1)) {
    fprintf(stderr, "tasksvr: bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN;
  }
  *task_shm_key = i1;

  key = "TCP_PORT";
  inistring = ini_find(fp, key, section);
  if (NULL == inistring) {
    fprintf(stderr, "tasksvr: missing entry: [%s] %s\n", section, key);
    i1 = DEFAULT_TASK_TCP_PORT;
  } else if (1 != sscanf(inistring, "%i", &i1)) {
    fprintf(stderr, "tasksvr: bad entry: [%s] %s = %s\n", section, key, inistring);
    CLOSE_AND_RETURN;
  }
  *task_tcp_port = i1;

  fclose(fp);
  return 0;
}

static void print_help(void)
{
  printf("-i <file> : use initialization file <file>, default %s\n", DEFAULT_INI_FILE);
  printf("-p <port> : serve socket <port>, default %d\n", DEFAULT_TASK_TCP_PORT);
  printf("-d        : turn debug on\n");
  printf("-?        : print this help message\n");
}

int main(int argc, char *argv[])
{
  int option;
  char ini_file[FILENAME_LEN];
  ulapi_id task_shm_key;
  ulapi_integer task_tcp_port = DEFAULT_TASK_TCP_PORT;
  ulapi_integer opt_port = 0;
  ulapi_integer server_id;
  ulapi_integer client_id;
  void *task_shm = NULL;
  task_comm_struct *task_comm_ptr;
  task_stat_struct task_stat;
  void *task;
  void *task_write_mutex;
  client_args *client_args_ptr;

  if (ulapi_init()) {
    fprintf(stderr, "tasksvr: ulapi_init error\n");
    return 1;
  }

  if (go_init()) {
    fprintf(stderr, "tasksvr: go_init error\n");
    return 1;
  }

  SAFECPY(ini_file, DEFAULT_INI_FILE);

  opterr = 0;
  for (;;) {
    option = getopt(argc, argv, ":i:u:p:d?");
    if (option == -1)
      break;

    switch (option) {
    case 'i':
      SAFECPY(ini_file, optarg);
      break;

    case 'p':
      opt_port = atoi(optarg);
      break;

    case 'd':
      dbflag = 1;
      break;

    case '?':
      print_help();
      return 0;
      break;

    case ':':
      fprintf(stderr, "tasksvr: missing value for -%c\n", optopt);
      return 1;
      break;

    default:
      fprintf(stderr, "tasksvr: unrecognized option -%c\n", optopt);
      return 1;
      break;
    }
  }
  if (optind < argc) {
    fprintf(stderr, "tasksvr: extra non-option characters: %s\n", argv[optind]);
    return 1;
  }

  if (0 != ini_load(ini_file, &task_shm_key, &task_tcp_port)) {
    fprintf(stderr, "tasksvr: can't read ini file %s\n", ini_file);
    return 1;
  }

  /* override ini file with options if provided */
  if (opt_port > 0) {
    task_tcp_port = opt_port;
  }

  server_id = ulapi_socket_get_server_id(task_tcp_port);
  if (server_id < 0) {
    fprintf(stderr, "tasksvr: can't serve port %d\n", (int) task_tcp_port);
    return 1;
  }

  task_write_mutex = ulapi_mutex_new(0);

  task_shm = ulapi_rtm_new(task_shm_key, sizeof(task_comm_struct));
  if (NULL == task_shm) {
    fprintf(stderr, "tasksvr: can't get task comm shm\n");
    return 1;
  }
  task_comm_ptr = ulapi_rtm_addr(task_shm);

  /* check for running task */
  if (0 != get_task_status(&task_stat, &task_comm_ptr->task_stat, CONNECT_WAIT_TIME)) {
    fprintf(stderr, "tasksvr: timed out connecting to task status\n");
    return 1;
  }

  if (0 == setjmp(env)) {
    signal(SIGINT, quit);

    for (;;) {
      dbprintf(1, "waiting for client connection on port %d...\n", (int) task_tcp_port);
      client_id = ulapi_socket_get_connection_id(server_id);
      if (client_id < 0) {
	break;
      }
      dbprintf(1, "got one on fd %d\n", client_id);

      task = ulapi_task_new();
      client_args_ptr = malloc(sizeof(client_args));
      client_args_ptr->task = task;
      client_args_ptr->client_id = client_id;
      client_args_ptr->task_write_mutex = task_write_mutex;
      client_args_ptr->task_comm_ptr = task_comm_ptr;
      ulapi_task_start(task, client_code, client_args_ptr, ulapi_prio_lowest(), 0);
    }
  }

  dbprintf(1, "tasksvr done\n");

  return 0;
}

