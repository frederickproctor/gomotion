/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#ifndef GOLOG_H
#define GOLOG_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "gotypes.h"		/* go_result */
#include "gomath.h"		/* go_pose */

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

/* FIXME-- consider putting this in ini file, make dynamic */
#define GO_LOG_MAX 10000

enum {
  GO_LOG_NONE = 0,
  GO_LOG_FERROR,		/* single servo following error */
  GO_LOG_INPUT,			/* single servo input */
  GO_LOG_ACT_POS,		/* single Cartesian actual position value */
  GO_LOG_CMD_POS,		/* single Cartesian commanded position value */
  GO_LOG_SETPOINT,		/* single servo setpoint */
  GO_LOG_SPEED,			/* single speed input */
  GO_LOG_XINV,			/* Xinv pose */
  GO_LOG_MAGXINV		/* magnitude of Xinv tran v. XY */
};

#define go_log_symbol(x) \
(x) == GO_LOG_NONE ? "None" : \
(x) == GO_LOG_FERROR ? "Ferror" : \
(x) == GO_LOG_INPUT ? "Input" : \
(x) == GO_LOG_ACT_POS ? "ActPos" : \
(x) == GO_LOG_CMD_POS ? "CmdPos" : \
(x) == GO_LOG_SETPOINT ? "Setpoint" : \
(x) == GO_LOG_SPEED ? "Speed" : \
(x) == GO_LOG_XINV ? "Xinv" : \
(x) == GO_LOG_MAGXINV ? "MagXinv" : "?"

typedef struct {
  go_real ferror;
} go_log_ferror;

typedef struct {
  go_real input;
} go_log_input;

typedef struct {
  go_real setpoint;
} go_log_setpoint;

typedef struct {
  go_real speed;
} go_log_speed;

typedef struct {
  go_pose pos;
} go_log_act_pos;

typedef struct {
  go_pose pos;
} go_log_cmd_pos;

typedef struct {
  go_pose xinv;
} go_log_xinv;

typedef struct {
  go_real x;
  go_real y;
  go_real mag;
} go_log_magxinv;

typedef struct
{
  go_real time;
  union {
    go_log_ferror ferror;
    go_log_input input;
    go_log_act_pos act_pos;
    go_log_cmd_pos cmd_pos;
    go_log_setpoint setpoint;
    go_log_speed speed;
    go_log_xinv xinv;
    go_log_magxinv magxinv;
  } u;
} go_log_entry;

/* full log, with header and union of types */
typedef struct
{
  go_integer type;		/* one of GO_LOG_FERROR, ... */
  go_integer which;		/* identifier for which joint or related */
  go_integer size;
  go_integer start;
  go_integer end;
  go_integer howmany;
  go_log_entry log[GO_LOG_MAX];
} go_log_struct;

extern go_result go_log_init(go_log_struct * log, go_integer type, go_integer which, go_integer size);

extern go_result go_log_add(go_log_struct * log, const go_log_entry * entry);

extern go_result go_log_get(go_log_struct * log, go_log_entry * entry);

extern go_integer go_log_type(const go_log_struct * log);

extern go_integer go_log_which(const go_log_struct * log);

extern go_integer go_log_howmany(const go_log_struct * log);

/* if you have a global log, you can use these declarations */

extern go_log_struct * global_go_log_ptr;

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif
