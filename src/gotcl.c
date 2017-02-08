/*!
  \file gotcl.c

  \brief C code for common Tcl main interfaces to Go Motion.
*/

#include <stdio.h>		/* fprintf, stderr, FILE */
#include <stddef.h>		/* sizeof, NULL */
#include <stdlib.h>		/* exit, atexit */
#include <signal.h>		/* signal, SIGINT */
#include <tcl.h>		/* Tcl_XXX */
#include "gotcltk.h"		/* gotk_xxx */

int
Tcl_AppInit(Tcl_Interp * interp)
{
  if (Tcl_Init(interp) == TCL_ERROR) {
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
  Tcl_Main(argc, argv, Tcl_AppInit);

  return 0;
}
