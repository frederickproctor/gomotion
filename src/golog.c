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
  \file golog.c

  \brief Logging data structures and functions.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stddef.h>		/* NULL */
#include "gotypes.h"		/* go_result */
#include "golog.h"		/* these decls */

go_result go_log_init(go_log_struct * log, go_integer type, go_integer which, go_integer size)
{
  if (NULL == log) return GO_RESULT_ERROR;

  log->type = type;
  if (size <= 0) return GO_RESULT_ERROR;
  log->which = which;
  log->size = (size < 1 ? 1 : size > GO_LOG_MAX ? GO_LOG_MAX : size);
  log->start = 0;
  log->end = 0;
  log->howmany = 0;

  return GO_RESULT_OK;
}

go_result go_log_add(go_log_struct * log, const go_log_entry * entry)
{
  if (NULL == log || NULL == entry) return GO_RESULT_ERROR;

  log->log[log->end] = *entry;

  log->end++;
  if (log->end >= log->size) {
    log->end = 0;
  }

  log->howmany++;
  if (log->howmany > log->size) {
    log->howmany = log->size;
    log->start++;
    if (log->start >= log->size) {
      log->start = 0;
    }
  }

  return GO_RESULT_OK;
}

go_result go_log_get(go_log_struct * log, go_log_entry * entry)
{
  if (NULL == log || NULL == entry) return GO_RESULT_ERROR;

  if (log->howmany == 0) {
    return GO_RESULT_ERROR;
  }

  *entry = log->log[log->start];
  log->start++;
  if (log->start >= log->size) {
    log->start = 0;
  }

  log->howmany--;

  return GO_RESULT_OK;
}

go_integer go_log_type(const go_log_struct * log)
{
  if (NULL == log) return 0;

  return log->type;
}

go_integer go_log_which(const go_log_struct * log)
{
  if (NULL == log) return 0;

  return log->which;
}

go_integer go_log_howmany(const go_log_struct * log)
{
  if (NULL == log) return 0;

  return log->howmany;
}
