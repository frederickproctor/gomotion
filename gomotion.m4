AC_DEFUN([ACX_SYSTEM_TYPE],
    [AC_MSG_CHECKING([for system type])]
    [AC_REQUIRE([AC_CANONICAL_HOST])]
    case $host in
	[*cygwin*)]
	[AC_DEFINE(HAVE_CYGWIN, 1, [Define non-zero if you have Cygwin.])]
	[AC_MSG_RESULT([$host, setting HAVE_CYGWIN = 1 in config.h])]
	;;
	[*linux*)]
	[AC_DEFINE(HAVE_LINUX, 1, [Define non-zero if you have Linux.])]
	[AC_MSG_RESULT([$host, setting HAVE_LINUX = 1 in config.h])]
	;;
	[*)]
	[AC_MSG_RESULT([$host, nothing added to config.h])]
    esac
)

AC_DEFUN([ACX_KERNEL_SOURCE],
    [AC_MSG_CHECKING([for kernel source])]
    [AC_ARG_WITH(kernel-source,
	    [  --with-kernel-source=<path to kernel source>  Specify path to Linux kernel source],
	    dirs=$withval,dirs="/usr/src/linux")]
    for dir in $dirs ; do
	if test -f $dir/include/linux/kernel.h ; then kernel_source_dir=$dir ; break; fi
    done
    if test x$kernel_source_dir = x ; then
	[AC_MSG_RESULT([no])]
	[AC_MSG_WARN([not found in $dirs, try --with-kernel-source=<path to kernel source>])]
    else
	KERNEL_SOURCE_DIR=$kernel_source_dir
# put HAVE_KERNEL_SOURCE in config.h
	[AC_DEFINE(HAVE_KERNEL_SOURCE,
		1, [Define non-zero if you have Linux kernel source.])]
# put KERNEL_SOURCE_DIR in Makefile
	[AC_SUBST(KERNEL_SOURCE_DIR)]
	[AC_MSG_RESULT([$KERNEL_SOURCE_DIR])]
    fi
)

AC_DEFUN([ACX_HAVE_IOPORTS],
	[AC_MSG_CHECKING([for inb/outb])]
	[AC_TRY_LINK([#include <sys/io.h>],
		[int in=inb(0x80);],
		have_ioports=yes)]
	if test x$have_ioports = xyes ; then
	[AC_MSG_RESULT([yes])]
dnl put HAVE_IOPORTS in config.h
	[AC_DEFINE(HAVE_IOPORTS,
		1, [Define non-zero if you have inb/outb.])]
	else
	[AC_MSG_RESULT([no])]
	fi
dnl enable check for HAVE_IOPORTS in Makefile.am
	[AM_CONDITIONAL(HAVE_IOPORTS, test x$have_ioports = xyes)]
)

AC_DEFUN([ACX_HAVE_IOPL],
	[AC_MSG_CHECKING([for iopl])]
	[AC_TRY_LINK([#include <sys/io.h>],
		[int val=iopl(0);],
		have_iopl=yes)]
	if test x$have_iopl = xyes ; then
	[AC_MSG_RESULT([yes])]
dnl put HAVE_IOPL in config.h
	[AC_DEFINE(HAVE_IOPL,
		1, [Define non-zero if you have iopl.])]
	else
	[AC_MSG_RESULT([no])]
	fi
)

AC_DEFUN([ACX_TCL_INCLUDE],
    [AC_MSG_CHECKING([for Tcl headers])]
    [AC_ARG_WITH(tclinclude,
	[  --with-tclinclude=<path to Tcl headers>  Specify path to Tcl headers],
	dirs=$withval,
	dirs="/usr/include/tcl*")]
    for dir in $dirs ; do
	if test -f $dir/tcl.h ; then tclinclude_dir=$dir ; break; fi
    done
    if test x$tclinclude_dir = x ; then
	[AC_MSG_RESULT([not found in $dirs, try --with-tclinclude=<path to Tcl headers>])]
    else
	TCL_INCLUDE_DIR=$tclinclude_dir
dnl put HAVE_TCL_INCLUDE in config.h
	[AC_DEFINE(HAVE_TCL_INCLUDE,
		1, [Define non-zero if you have Tcl headers.])]
dnl put TCL_INCLUDE_DIR in Makefile
	[AC_SUBST(TCL_INCLUDE_DIR)]
	[AC_MSG_RESULT([$TCL_INCLUDE_DIR])]
    fi
    [AM_CONDITIONAL(HAVE_TCL_INCLUDE, test x$tclinclude_dir != x)]
)

AC_DEFUN([ACX_TCL_LIB],
    [AC_MSG_CHECKING([for Tcl library])]
    [AC_ARG_WITH(tcllib,
	[  --with-tcllib=<path to Tcl library>  Specify path to Tcl library],
	dirs=$withval,
	dirs="/usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu")]
    [AC_ARG_WITH(tcllink,
	    [  --with-tcllink=<link name of Tcl library>  Specify link name of Tcl library],
	    names=$withval,names="tcl8.6 tcl8.5")]
    for dir in $dirs ; do
	for name in $names ; do
		if test -f $dir/lib"$name".a ; then tcllink="-L$dir -l$name"; break; fi
		if test -f $dir/lib"$name".so ; then tcllink="-L$dir -l$name"; break; fi
	done
    done
    if test x"$tcllink" = x ; then
	[AC_MSG_RESULT([none of lib"$names".a,so found in $dirs, try --with-tcllib=<path to Tcl library, --with-tcllink=<link name of Tcl library>])]
    else
	TCL_LINK="$tcllink"
dnl put TCL_LINK in Makefile
	[AC_SUBST(TCL_LINK)]
	[AC_MSG_RESULT([$TCL_LINK])]
    fi
    [AM_CONDITIONAL(HAVE_TCL_LIB, test x"$tcllink" != x)]
)

AC_DEFUN([ACX_TK_INCLUDE],
    [AC_MSG_CHECKING([for Tk headers])]
    [AC_ARG_WITH(tkinclude,
	[  --with-tkinclude=<path to Tk headers>  Specify path to Tk headers],
	dirs=$withval,
	dirs="/usr/include/tcl* /usr/include/tk*")]
    for dir in $dirs ; do
	if test -f $dir/tk.h ; then tkinclude_dir=$dir ; break; fi
    done
    if test x$tkinclude_dir = x ; then
	[AC_MSG_RESULT([not found in $dirs, try --with-tkinclude=<path to Tk headers>])]
    else
	TK_INCLUDE_DIR=$tkinclude_dir
dnl put HAVE_TK_INCLUDE in config.h
	[AC_DEFINE(HAVE_TK_INCLUDE,
		1, [Define non-zero if you have Tk headers.])]
dnl put TK_INCLUDE_DIR in Makefile
	[AC_SUBST(TK_INCLUDE_DIR)]
	[AC_MSG_RESULT([$TK_INCLUDE_DIR])]
    fi
    [AM_CONDITIONAL(HAVE_TK_INCLUDE, test x$tkinclude_dir != x)]
)

AC_DEFUN([ACX_TK_LIB],
    [AC_MSG_CHECKING([for Tk library])]
    [AC_ARG_WITH(tklib,
	    [  --with-tklib=<path to Tk library>  Specify path to Tk library],
	    dirs=$withval,dirs="/usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu")]
    [AC_ARG_WITH(tklink,
	    [  --with-tklink=<link name of Tk library>  Specify link name of Tk library],
	    names=$withval,names="tk8.6 tk8.5")]
    for dir in $dirs ; do
	for name in $names ; do
		if test -f $dir/lib"$name".a ; then tklink="-L$dir -l$name"; break; fi
		if test -f $dir/lib"$name".so ; then tklink="-L$dir -l$name"; break; fi
	done
    done
    if test x"$tklink" = x ; then
	[AC_MSG_RESULT([none of lib"$names".a,so found in $dirs, try --with-tklib=<path to Tk library, --with-tklink=<link name of Tk library>])]
    else
	TK_LINK="$tklink"
dnl put TK_LINK in Makefile
	[AC_SUBST(TK_LINK)]
	[AC_MSG_RESULT([$TK_LINK])]
    fi
    [AM_CONDITIONAL(HAVE_TK_LIB, test x"$tklink" != x)]
)

AC_DEFUN([ACX_S626],
    [AC_MSG_CHECKING([for Sensoray 626 driver])]
    [AC_ARG_WITH(s626,
	    [  --with-s626=<path to s626>  Specify path to s626],
	    dirs=$withval,dirs="`pwd`/drivers/s626-1.0 `pwd`/drivers/s626-0.3")]
    for dir in $dirs ; do
	if test -f $dir/libs626.a ; then s626_dir=$dir ; break; fi
    done
    if test x$s626_dir = x ; then
	[AC_MSG_RESULT([not found in $dirs, try --with-s626=<path to s626>])]
    else
	S626_DIR=$s626_dir
dnl put HAVE_S626 in config.h
	[AC_DEFINE(HAVE_S626,
		1, [Define non-zero if you have S626.])]
dnl put S626_DIR in Makefile
	[AC_SUBST(S626_DIR)]
	[AC_MSG_RESULT([$S626_DIR])]
    fi
    [AM_CONDITIONAL(HAVE_S626, test x$s626_dir != x)]
)

AC_DEFUN([ACX_HAVE_EXTINTF],
	[AC_MSG_CHECKING([for user-provided external interface])]
	[AC_ARG_WITH(extintf,
		[--with-extintf=<path to object code> Specify link path to external interface object code],
		EXTINTF="$withval",EXTINTF="")]
	if test x"$EXTINTF" = x ; then
	[AC_MSG_RESULT([none provided, use --with-extintf=<link path to object code> if desired])]
	else
	[AC_MSG_RESULT([$EXTINTF])]
	fi
	[AC_SUBST(EXTINTF)]
	[AM_CONDITIONAL(HAVE_EXTINTF, test x"$EXTINTF" != x)]
)

AC_DEFUN([ACX_HAVE_SDL],
	[AC_CHECK_PROG([have_sdl_config],[sdl-config],[yes],[no])]
	if test x"$have_sdl_config" = xyes ; then
		SDL_CFLAGS=`sdl-config --cflags`
		SDL_LIBS=`sdl-config --libs`
dnl put HAVE_SDL in config.h
		[AC_DEFINE(HAVE_SDL, 1, [Define non-zero if you have SDL])]
	fi
dnl put SDL_CFLAGS and SDL_LIBS in Makefile
	[AC_SUBST(SDL_CFLAGS)]
	[AC_SUBST(SDL_LIBS)]
	[AM_CONDITIONAL(HAVE_SDL, test x"$have_sdl_config" = xyes)]
	)

dnl Check for Gnu Scientific Library
dnl It is important to check for libm and libgslcblas before libgsl,
dnl otherwise the tests will fail.
dnl If the library is found then the tests will define the macros
dnl HAVE_LIBGSL, HAVE_LIBGSLCBLAS, HAVE_LIBM and add the options -lgsl
dnl -lgslcblas -lm to the variable LIBS.
AC_DEFUN([ACX_HAVE_LIBGSL],
	[AC_CHECK_LIB([m],[cos])]
	[AC_CHECK_LIB([gslcblas],[cblas_dgemm])]
	[AC_CHECK_LIB([gsl],[gsl_blas_dgemm])]
dnl exploit variable generated inside configure script
 	[AM_CONDITIONAL(HAVE_LIBGSL, test x$ac_cv_lib_gsl_gsl_blas_dgemm = xyes)]
)

AC_DEFUN([ACX_PIC],
	PIC_CFLAGS="-fPIC"
	[AC_SUBST(PIC_CFLAGS)]
)	

AC_DEFUN([ACX_PRE_GOMOTION],
	[ACX_ULAPI]
	[ACX_SYSTEM_TYPE]
	[ACX_TCL_INCLUDE]
	[ACX_TCL_LIB]
	[ACX_TK_INCLUDE]
	[ACX_TK_LIB]
	[ACX_KERNEL_SOURCE]
	[ACX_HAVE_IOPORTS]
	[ACX_HAVE_IOPL]
	[ACX_S626]
	[ACX_HAVE_EXTINTF]
	[ACX_HAVE_SDL]
	[ACX_HAVE_LIBGSL]
	[ACX_PIC]
)

AC_DEFUN([ACX_GOMOTION],
	[ACX_PRE_GOMOTION]
	[AC_MSG_CHECKING([for Go Motion])]
	[AC_ARG_WITH(gomotion,
		[ --with-gomotion=<path to Go Motion>  Specify path to Go Motion directory],
		dirs=$withval,dirs="/usr/local /usr/local/gomotion /usr/local/src/gomotion $HOME/gomotion")]
	for dir in $dirs ; do
		if test -f $dir/include/go.h ; then GOMOTION_DIR=$dir ; break ; fi
	done
	if test x$GOMOTION_DIR = x ; then
	[AC_MSG_WARN([not found, specify using --with-gomotion=<path to Go Motion>])]
	else
	[AC_MSG_RESULT([$GOMOTION_DIR])]
	GOMOTION_CFLAGS="-I$GOMOTION_DIR/include"
	GOMOTION_LIBS="-L$GOMOTION_DIR/lib -lgokin -lgo"
dnl put HAVE_GOMOTION in config.h
	[AC_DEFINE(HAVE_GOMOTION,
		1, [Define non-zero if you have Go Motion.])]
dnl put GOMOTION_DIR in Makefile
	[AC_SUBST(GOMOTION_DIR)]
	[AC_SUBST(GOMOTION_CFLAGS)]
	[AC_SUBST(GOMOTION_LIBS)]
	fi
dnl put GOMOTION_DIR into variable file for use by shell scripts
	echo GOMOTION_DIR=$GOMOTION_DIR > gomotion_dir
dnl enable HAVE_GOMOTION test in Makefile
	[AM_CONDITIONAL(HAVE_GOMOTION, test x$GOMOTION_DIR != x)]
)
