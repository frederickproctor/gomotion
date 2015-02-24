/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#ifndef INTERPLIST_H
#define INTERPLIST_H

#include "taskintf.h"
#include "trajintf.h"
#include "toolintf.h"

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

typedef struct {
  go_integer type;
  union {
    task_cmd_struct task_cmd;
    traj_cmd_struct traj_cmd;
    tool_cmd_struct tool_cmd;
  } u;
} interplist_type;

typedef struct _interplist_entry {
  interplist_type val;
  struct _interplist_entry *next;
} interplist_entry;

typedef struct {
  interplist_entry *head;
  interplist_entry *tail;
  unsigned int howmany;
} interplist_struct;

extern int interplist_put(interplist_struct *list, interplist_type val);

extern int interplist_get(interplist_struct *list, interplist_type *val);

extern int interplist_peek(interplist_struct *list, interplist_type *val);

extern unsigned int interplist_howmany(interplist_struct *list);

extern int interplist_init(interplist_struct *list);

extern int interplist_clear(interplist_struct *list);

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif	/* INTERPLIST_H */
