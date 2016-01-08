SUBDIRS = lib bin src etc

EXTRA_DIST = FAQ Makefile.am Makefile.local acinclude.m4 configure.ac autoconf.sh build config/config.guess config/config.sub rtlib/README rtlib/Makefile rtlib/SETUP doc testing drivers scripts ulapi.m4 gomotion.m4 win32
nobase_data_DATA = ulapi.m4 gomotion.m4

# uncomment these lines if you want to include the Doxygen documentation
# in the distribution and in the install
# EXTRA_DIST += @DX_CONFIG@ @DX_DOCDIR@ @DX_DOCDIR@/html
# nobase_data_DATA += @DX_DOCDIR@/html/*

include doxygen.am

if HAVE_RTAI

all-local:
	make -f Makefile.local modules

clean-local:
	make -f Makefile.local modules_clean

endif

MOSTLYCLEANFILES = @DX_CLEANFILES@
