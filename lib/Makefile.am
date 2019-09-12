AM_CPPFLAGS = -I../src -I@ULAPI_DIR@/include

lib_LIBRARIES = libgo.a libgokin.a

libgo_a_SOURCES = ../src/go.c ../src/gotypes.c ../src/gomath.c ../src/goutil.c ../src/gotraj.c ../src/gomotion.c ../src/gointerp.c ../src/golog.c ../src/goprint.c ../src/variates.c ../src/variates.h

libgokin_a_SOURCES = \
../src/kinselect.c \
../src/trivkins.c ../src/trivkins.h \
../src/genhexkins.c ../src/genhexkins.h \
../src/genserkins.c ../src/genserkins.h \
../src/pumakins.c ../src/pumakins.h \
../src/fanuckins.c ../src/fanuckins.h \
../src/fanuc_lrmate200id_kins.c ../src/fanuc_lrmate200id_kins.h \
../src/scarakins.c ../src/scarakins.h \
../src/tripointkins.c ../src/tripointkins.h \
../src/spheristkins.c ../src/spheristkins.h \
../src/three21kins.c ../src/three21kins.h \
../src/roboch.c ../src/roboch.h

