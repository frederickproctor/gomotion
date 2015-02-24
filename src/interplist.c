/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#include <stdlib.h>		/* malloc, free */
#include "interplist.h"

int interplist_put(interplist_struct *list, interplist_type val)
{
  interplist_entry *entry;

  entry = malloc(sizeof(*entry));

  entry->val = val;
  entry->next = NULL;
  if (list->head == NULL) {
    list->head = entry;
  } else {
    list->tail->next = entry;
  }
  list->tail = entry;
  list->howmany++;

  return 0;
}

int interplist_get(interplist_struct *list, interplist_type *val)
{
  interplist_entry *next;

  if (NULL == list->head) {
    return -1;
  }

  next = list->head->next;
  *val = list->head->val;
  free(list->head);
  list->head = next;
  list->howmany--;

  return 0;
}

int interplist_peek(interplist_struct *list, interplist_type *val)
{
  if (NULL == list->head) {
    return -1;
  }

  *val = list->head->val;

  return 0;
}

unsigned int interplist_howmany(interplist_struct *list)
{
  return list->howmany;
}

int interplist_init(interplist_struct *list)
{
  list->head = NULL;
  list->tail = NULL;

  return 0;
}

int interplist_clear(interplist_struct *list)
{
  interplist_type val;
  int retval;

  do {
    retval = interplist_get(list, &val);
  } while (retval == 0);

  return 0;
}
