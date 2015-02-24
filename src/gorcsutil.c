/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#include <stdio.h>		/* sprintf */
#include "gotypes.h"		/* go_real */
#include "gorcs.h"		/* NEW_COMMAND, ... */
#include "gorcsutil.h"		/* these decls */

char *rcs_state_to_string(go_integer s)
{
  enum { BUFLEN = 80 };
  static char buf[BUFLEN];	/* warning-- not reentrant */

  if (s == GO_RCS_STATE_UNINITIALIZED)
    sprintf(buf, "Uninitialized");
  else if (s == GO_RCS_STATE_NEW_COMMAND)
    sprintf(buf, "NewCommand");
  else
    sprintf(buf, "S%d", (int) (s - 10)); /* S0 starts at 10 */

  return buf;
}

char *rcs_admin_state_to_string(go_integer s)
{
  enum { BUFLEN = 80 };
  static char buf[BUFLEN];	/* warning-- not reentrant */

  if (s == GO_RCS_ADMIN_STATE_UNINITIALIZED)
    sprintf(buf, "Uninitialized");
  else if (s == GO_RCS_ADMIN_STATE_INITIALIZED)
    sprintf(buf, "Initialized");
  else if (s == GO_RCS_ADMIN_STATE_SHUT_DOWN)
    sprintf(buf, "ShutDown");
  else
    sprintf(buf, "%d", (int) s);

  return buf;
}

char *rcs_status_to_string(go_integer s)
{
  enum { BUFLEN = 80 };
  static char buf[BUFLEN];	/* warning-- not reentrant */

  if (s == GO_RCS_STATUS_UNINITIALIZED)
    sprintf(buf, "Uninitialized");
  else if (s == GO_RCS_STATUS_DONE)
    sprintf(buf, "Done");
  else if (s == GO_RCS_STATUS_EXEC)
    sprintf(buf, "Exec");
  else if (s == GO_RCS_STATUS_ERROR)
    sprintf(buf, "Error");
  else
    sprintf(buf, "%d", (int) s);

  return buf;
}
