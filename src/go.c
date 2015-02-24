/*
  go.c

  Top-level initialization
*/

#include "go.h"

/* here's where we match the header to the library */
int go_lib_version = GO_HEADER_VERSION;

/* initialization function, referenced in _go_init() macro */

int _go_init(void)
{
  /* nothing need be done for now; add code here if you've changed
     Go and something needs to be done when the application starts */
  return 0;
}

int go_exit(void)
{
  /* nothing need be done for now; add code here if you've changed
     Go and something needs to be done when the application ends */
  return 0;
}

