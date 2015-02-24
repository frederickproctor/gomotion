AC_DEFUN([ACX_MTCONNECT],
	[AC_MSG_CHECKING([for MTConnect])]
	[AC_ARG_WITH(mtconnect,
		[ --with-mtconnect=<path to mtconnect>  Specify path to MTConnect directory],
		dirs=$withval,dirs="/usr/local/mtconnect /usr/local/src/mtconnect $HOME/mtconnect")]
	for dir in $dirs ; do
		if test -f $dir/include/adapter.hpp ; then MTCONNECT_DIR=$dir ; break ; fi
	done
	if test x$MTCONNECT_DIR = x ; then
	[AC_MSG_WARN([not found, specify using --with-mtconnect=<path to MTConnect>])]
	else
	[AC_MSG_RESULT([$MTCONNECT_DIR])]
	MTCONNECT_CXXFLAGS="-I$MTCONNECT_DIR/include"
	MTCONNECT_LIBS="-L$MTCONNECT_DIR/lib -lmtconnect"
dnl put HAVE_MTCONNECT in config.h
	[AC_DEFINE(HAVE_MTCONNECT,
		1, [Define non-zero if you have mtconnect.])]
dnl put MTCONNECT_DIR in Makefile
	[AC_SUBST(MTCONNECT_DIR)]
	[AC_SUBST(MTCONNECT_CXXFLAGS)]
	[AC_SUBST(MTCONNECT_LIBS)]
	fi
dnl enable HAVE_MTCONNECT test in Makefile
	[AM_CONDITIONAL(HAVE_MTCONNECT, test x$MTCONNECT_DIR != x)]
)
