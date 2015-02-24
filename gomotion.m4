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

AC_DEFUN([ACX_TK_INCLUDE],
    [AC_MSG_CHECKING([for Tk headers])]
    [AC_ARG_WITH(tkinclude,
	[  --with-tkinclude=<path to Tk headers>  Specify path to Tk headers],
	dirs=$withval,
	dirs="/usr/include /usr/local/include /usr/include/tk8.5 /usr/include/tk8.4 /usr/include/tk8.3 /usr/include/tcl8.5 /usr/include/tcl8.4 /usr/include/tcl8.3")]
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

AC_DEFUN([ACX_TCL_LIB],
    [AC_MSG_CHECKING([for Tcl library])]
    [AC_ARG_WITH(tcllib,
	[  --with-tcllib=<path to Tcl library>  Specify path to Tcl library],
	dirs=$withval,
	dirs="/usr/lib /usr/local/lib")]
    [AC_ARG_WITH(tcllink,
	    [  --with-tcllink=<link name of Tcl library>  Specify link name of Tcl library],
	    names=$withval,names="tcl tcl8.5 tcl85 tcl8.4 tcl84 tcl8.3 tcl83")]
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

AC_DEFUN([ACX_TK_LIB],
    [AC_MSG_CHECKING([for Tk library])]
    [AC_ARG_WITH(tklib,
	    [  --with-tklib=<path to Tk library>  Specify path to Tk library],
	    dirs=$withval,dirs="/usr/lib /usr/local/lib")]
    [AC_ARG_WITH(tklink,
	    [  --with-tklink=<link name of Tk library>  Specify link name of Tk library],
	    names=$withval,names="tk tk8.5 tk85 tk8.4 tk84 tk8.3 tk83")]
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

# Simple DirectMedia Layer
# Configure paths for SDL
# Sam Lantinga 9/21/99
# stolen from Manish Singh
# stolen back from Frank Belew
# stolen from Manish Singh
# Shamelessly stolen from Owen Taylor

dnl AM_PATH_SDL([MINIMUM-VERSION, [ACTION-IF-FOUND [, ACTION-IF-NOT-FOUND]]])
dnl Test for SDL, and define SDL_CFLAGS and SDL_LIBS
dnl
AC_DEFUN([AM_PATH_SDL],
[dnl 
dnl Get the cflags and libraries from the sdl-config script
dnl
AC_ARG_WITH(sdl-prefix,[  --with-sdl-prefix=PFX   Prefix where SDL is installed (optional)],
            sdl_prefix="$withval", sdl_prefix="")
AC_ARG_WITH(sdl-exec-prefix,[  --with-sdl-exec-prefix=PFX Exec prefix where SDL is installed (optional)],
            sdl_exec_prefix="$withval", sdl_exec_prefix="")
AC_ARG_ENABLE(sdltest, [  --disable-sdltest       Do not try to compile and run a test SDL program],
		    , enable_sdltest=yes)

  if test x$sdl_exec_prefix != x ; then
    sdl_args="$sdl_args --exec-prefix=$sdl_exec_prefix"
    if test x${SDL_CONFIG+set} != xset ; then
       SDL_CONFIG=$sdl_exec_prefix/bin/sdl-config
    fi
  fi
  if test x$sdl_prefix != x ; then
    sdl_args="$sdl_args --prefix=$sdl_prefix"
    if test x${SDL_CONFIG+set} != xset ; then
       SDL_CONFIG=$sdl_prefix/bin/sdl-config
    fi
  fi

  if test "x$prefix" != xNONE; then
    PATH="$prefix/bin:$prefix/usr/bin:$PATH"
  fi
  AC_PATH_PROG(SDL_CONFIG, sdl-config, no, [$PATH])
  min_sdl_version=ifelse([$1], ,0.11.0,$1)
  AC_MSG_CHECKING(for SDL - version >= $min_sdl_version)
  no_sdl=""
  if test "$SDL_CONFIG" = "no" ; then
    no_sdl=yes
  else
    SDL_CFLAGS=`$SDL_CONFIG $sdlconf_args --cflags`
    SDL_LIBS=`$SDL_CONFIG $sdlconf_args --libs`

    sdl_major_version=`$SDL_CONFIG $sdl_args --version | \
           sed 's/\([[0-9]]*\).\([[0-9]]*\).\([[0-9]]*\)/\1/'`
    sdl_minor_version=`$SDL_CONFIG $sdl_args --version | \
           sed 's/\([[0-9]]*\).\([[0-9]]*\).\([[0-9]]*\)/\2/'`
    sdl_micro_version=`$SDL_CONFIG $sdl_config_args --version | \
           sed 's/\([[0-9]]*\).\([[0-9]]*\).\([[0-9]]*\)/\3/'`
    if test "x$enable_sdltest" = "xyes" ; then
      ac_save_CFLAGS="$CFLAGS"
      ac_save_CXXFLAGS="$CXXFLAGS"
      ac_save_LIBS="$LIBS"
      CFLAGS="$CFLAGS $SDL_CFLAGS"
      CXXFLAGS="$CXXFLAGS $SDL_CFLAGS"
      LIBS="$LIBS $SDL_LIBS"
dnl
dnl Now check if the installed SDL is sufficiently new. (Also sanity
dnl checks the results of sdl-config to some extent
dnl
      rm -f conf.sdltest
      AC_TRY_RUN([
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "SDL.h"

char*
my_strdup (char *str)
{
  char *new_str;
  
  if (str)
    {
      new_str = (char *)malloc ((strlen (str) + 1) * sizeof(char));
      strcpy (new_str, str);
    }
  else
    new_str = NULL;
  
  return new_str;
}

int main (int argc, char *argv[])
{
  int major, minor, micro;
  char *tmp_version;

  /* This hangs on some systems (?)
  system ("touch conf.sdltest");
  */
  { FILE *fp = fopen("conf.sdltest", "a"); if ( fp ) fclose(fp); }

  /* HP/UX 9 (%@#!) writes to sscanf strings */
  tmp_version = my_strdup("$min_sdl_version");
  if (sscanf(tmp_version, "%d.%d.%d", &major, &minor, &micro) != 3) {
     printf("%s, bad version string\n", "$min_sdl_version");
     exit(1);
   }

   if (($sdl_major_version > major) ||
      (($sdl_major_version == major) && ($sdl_minor_version > minor)) ||
      (($sdl_major_version == major) && ($sdl_minor_version == minor) && ($sdl_micro_version >= micro)))
    {
      return 0;
    }
  else
    {
      printf("\n*** 'sdl-config --version' returned %d.%d.%d, but the minimum version\n", $sdl_major_version, $sdl_minor_version, $sdl_micro_version);
      printf("*** of SDL required is %d.%d.%d. If sdl-config is correct, then it is\n", major, minor, micro);
      printf("*** best to upgrade to the required version.\n");
      printf("*** If sdl-config was wrong, set the environment variable SDL_CONFIG\n");
      printf("*** to point to the correct copy of sdl-config, and remove the file\n");
      printf("*** config.cache before re-running configure\n");
      return 1;
    }
}

],, no_sdl=yes,[echo $ac_n "cross compiling; assumed OK... $ac_c"])
       CFLAGS="$ac_save_CFLAGS"
       CXXFLAGS="$ac_save_CXXFLAGS"
       LIBS="$ac_save_LIBS"
     fi
  fi
  if test "x$no_sdl" = x ; then
     AC_MSG_RESULT(yes)
     ifelse([$2], , :, [$2])     
  else
     AC_MSG_RESULT(no)
     if test "$SDL_CONFIG" = "no" ; then
       echo "*** The sdl-config script installed by SDL could not be found"
       echo "*** If SDL was installed in PREFIX, make sure PREFIX/bin is in"
       echo "*** your path, or set the SDL_CONFIG environment variable to the"
       echo "*** full path to sdl-config."
     else
       if test -f conf.sdltest ; then
        :
       else
          echo "*** Could not run SDL test program, checking why..."
          CFLAGS="$CFLAGS $SDL_CFLAGS"
          CXXFLAGS="$CXXFLAGS $SDL_CFLAGS"
          LIBS="$LIBS $SDL_LIBS"
          AC_TRY_LINK([
#include <stdio.h>
#include "SDL.h"

int main(int argc, char *argv[])
{ return 0; }
#undef  main
#define main K_and_R_C_main
],      [ return 0; ],
        [ echo "*** The test program compiled, but did not run. This usually means"
          echo "*** that the run-time linker is not finding SDL or finding the wrong"
          echo "*** version of SDL. If it is not finding SDL, you'll need to set your"
          echo "*** LD_LIBRARY_PATH environment variable, or edit /etc/ld.so.conf to point"
          echo "*** to the installed location  Also, make sure you have run ldconfig if that"
          echo "*** is required on your system"
	  echo "***"
          echo "*** If you have an old version installed, it is best to remove it, although"
          echo "*** you may also be able to get things to work by modifying LD_LIBRARY_PATH"],
        [ echo "*** The test program failed to compile or link. See the file config.log for the"
          echo "*** exact error that occured. This usually means SDL was incorrectly installed"
          echo "*** or that you have moved SDL since it was installed. In the latter case, you"
          echo "*** may want to edit the sdl-config script: $SDL_CONFIG" ])
          CFLAGS="$ac_save_CFLAGS"
          CXXFLAGS="$ac_save_CXXFLAGS"
          LIBS="$ac_save_LIBS"
       fi
     fi
     SDL_CFLAGS=""
     SDL_LIBS=""
     ifelse([$3], , :, [$3])
  fi
  AC_SUBST(SDL_CFLAGS)
  AC_SUBST(SDL_LIBS)
  rm -f conf.sdltest
])

AC_DEFUN([ACX_HAVE_SDL],
	[AM_PATH_SDL([1.2.8], have_sdl=yes)]
	if test x$have_sdl = xyes ; then
dnl put HAVE_SDL in config.h
	[AC_DEFINE(HAVE_SDL, 1, [Define non-zero if you have SDL.])]
	fi
	[AM_CONDITIONAL(HAVE_SDL, test x$have_sdl = xyes)]
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

AC_DEFUN([ACX_PRE_GOMOTION],
	[ACX_ULAPI]
	[ACX_SYSTEM_TYPE]
	[ACX_TK_INCLUDE]
	[ACX_TCL_LIB]
	[ACX_TK_LIB]
	[ACX_KERNEL_SOURCE]
	[ACX_HAVE_IOPORTS]
	[ACX_HAVE_IOPL]
	[ACX_S626]
	[ACX_HAVE_EXTINTF]
	[ACX_HAVE_SDL]
	[ACX_HAVE_LIBGSL]
)

AC_DEFUN([ACX_GOMOTION],
	[ACX_PRE_GOMOTION]
	[AC_MSG_CHECKING([for Go Motion])]
	[AC_ARG_WITH(gomotion,
		[ --with-gomotion=<path to Go Motion>  Specify path to Go Motion directory],
		dirs=$withval,dirs="/usr/local/gomotion /usr/local/src/gomotion $HOME/gomotion")]
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
