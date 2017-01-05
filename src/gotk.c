/*!
  \file gotk.c

  \brief C code for common Tk main interfaces to Go Motion.
*/

#include <stdio.h>		/* fprintf, stderr, FILE */
#include <stddef.h>		/* sizeof, NULL */
#include <stdlib.h>		/* exit, atexit */
#include <signal.h>		/* signal, SIGINT */
#include <tk.h>			/* Tk_XXX */
#include "gotcltk.h"		/* gotk_xxx */

static int
Tk_AppInit(Tcl_Interp * interp)
{
  if (Tcl_Init(interp) == TCL_ERROR) {
    return TCL_ERROR;
  }
  if (Tk_Init(interp) == TCL_ERROR) {
    return TCL_ERROR;
  }

  gotk_create_commands(interp);

  return TCL_OK;
}

static void sigquit(int sig)
{
  exit(0);
}

/*
  Built as either 'gotk' or 'gotcl', with or without windowing support,
  respectively.

  We will be called with a script as something like

  gotk pendant.tcl {args to Tcl} -- {args for us}

  Interactively, call with a script '-' to represent stdin, e.g.,

  gotcl - -- -i gomotion.ini

  Args for us are:
  -i <inifile> , specify a initialization file
  -u unix | rtai , specify either unix or real-time unix
  -d , turn debug on
  -k , kill controllers on shutdown
  -x , ignore failures to connect, useful for testing
  -p , peek mode -- don't do any inits, stops, shutdowns
*/

int main(int argc, char *argv[])
{
  if (0 != gotk_main(argc, argv)) {
    gotk_exit();
    return 1;
  }

  signal(SIGINT, sigquit);
  atexit(gotk_exit);
  Tk_Main(argc, argv, Tk_AppInit);

  return 0;
}
