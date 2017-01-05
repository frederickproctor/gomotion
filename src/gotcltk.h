#ifndef GOTCLTK_H
#define GOTCLTK_H

#include <tcl.h>

extern void gotk_create_commands(Tcl_Interp * interp);
extern int gotk_main(int argc, char *argv[]);
extern void gotk_exit(void);

#endif	/* GOTCLTK_H */
