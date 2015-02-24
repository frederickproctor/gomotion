AC_INIT(gomotion, 1.27)
AC_CONFIG_AUX_DIR([config])
AM_INIT_AUTOMAKE(gomotion, 1.27)
AC_CONFIG_SRCDIR([src/go.c])
AC_CONFIG_HEADERS([config/config.h])

# Checks for programs.
AC_PROG_CC
AM_PROG_CC_C_O
AC_PROG_RANLIB
AC_PROG_CXX

# Checks for header files.
AC_HEADER_STDC
AC_CHECK_HEADERS([float.h stddef.h stdlib.h string.h unistd.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_C_CONST

# Checks for library functions.
AC_HAVE_LIBRARY(m)
AC_FUNC_STRTOD
AC_CHECK_FUNCS([pow sqrt])
AC_CHECK_FUNCS(sincos)

AC_CHECK_HEADERS([readline/readline.h])

AC_SEARCH_LIBS(socket, [socket nsl])
AC_SEARCH_LIBS(gethostname, [socket nsl])

AC_SEARCH_LIBS(tputs, [termcap])
AC_SEARCH_LIBS(readline, [readline])

# Checks for X development files. It adds the C compiler flags that X
# needs to output variable X_CFLAGS, and the X linker flags to
# X_LIBS. Define X_DISPLAY_ MISSING if X is not available.  This macro
# also checks for special libraries that some systems need in order to
# compile X programs. It adds any that the system needs to output
# variable X_EXTRA_LIBS.  And it checks for special X11R6 libraries that
# need to be linked with before -lX11, and adds any found to the output
# variable X_PRE_LIBS.
AC_PATH_XTRA

# All these are part of the preface to ACX_GOMOTION in gomotion.am,
# which also includes the test for the gomotion directory that we
# don't need here. Here, we just call the preface.
ACX_PRE_GOMOTION

######################################################################
# DOXYGEN SUPPORT
######################################################################

DX_HTML_FEATURE(ON)
DX_CHM_FEATURE(OFF)
DX_CHI_FEATURE(OFF)
DX_MAN_FEATURE(OFF)
DX_RTF_FEATURE(OFF)
DX_XML_FEATURE(OFF)
DX_PDF_FEATURE(OFF)
DX_PS_FEATURE(OFF)

DX_INIT_DOXYGEN([gomotion],[Doxyfile],[doxygen-doc])

######################################################################
# MTCONNECT SUPPORT
######################################################################

ACX_MTCONNECT

AC_CONFIG_FILES([Makefile lib/Makefile bin/Makefile src/Makefile etc/Makefile])

AC_OUTPUT

