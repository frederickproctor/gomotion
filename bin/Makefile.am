AM_CPPFLAGS = -I. -I@ULAPI_DIR@/include

if HAVE_TCL_INCLUDE
AM_CPPFLAGS += -I@TCL_INCLUDE_DIR@
endif

if HAVE_TK_INCLUDE
AM_CPPFLAGS += -I@TK_INCLUDE_DIR@
endif

if HAVE_SDL
AM_CPPFLAGS += @SDL_CFLAGS@
endif

EXTRA_DIST = gorun.sh checkgo killgo pendant.tcl gogui.tcl move.tcl insrtl rmrtl ipc-clear updown mtconnect_client spinup modbus_write.py

bin_PROGRAMS = goscratchtest gomathtest gotrajtest gomotiontest gointerptest gokintest gotestsh gostepper gomain gosteppercfg gocfg gosh gotestmmavg tracker igpsclient igpsserver taskmain tasksvr toolmain variates rs274ngc cartfit

if HAVE_TCL_LIB
bin_PROGRAMS += gotcl
endif
if HAVE_TK_LIB
bin_PROGRAMS += gotk
endif

bin_PROGRAMS += gorun
gorun_SOURCES = ../src/gorun.c
gorun_LDADD = ../lib/libgo.a @ULAPI_LIBS@ 
gorun_DEPENDENCIES = ../lib/libgo.a

bin_SCRIPTS = gorun.sh checkgo killgo pendant.tcl move.tcl insrtl rmrtl ipc-clear updown mtconnect_client spinup

goscratchtest_SOURCES = ../src/goscratchtest.c 
goscratchtest_LDADD = ../lib/libgokin.a ../lib/libgo.a
goscratchtest_DEPENDENCIES = ../lib/libgokin.a ../lib/libgo.a

gomathtest_SOURCES = ../src/gomathtest.c 
gomathtest_LDADD = ../lib/libgo.a
gomathtest_DEPENDENCIES = ../lib/libgo.a

gotrajtest_SOURCES = ../src/gotrajtest.c 
gotrajtest_LDADD = ../lib/libgo.a
gotrajtest_DEPENDENCIES = ../lib/libgo.a

gomotiontest_SOURCES = ../src/gomotiontest.c 
gomotiontest_LDADD = ../lib/libgo.a
gomotiontest_DEPENDENCIES = ../lib/libgo.a

gointerptest_SOURCES = ../src/gointerptest.c 
gointerptest_LDADD = ../lib/libgo.a
gointerptest_DEPENDENCIES = ../lib/libgo.a

gokintest_SOURCES = ../src/gokintest.c
gokintest_LDADD = ../lib/libgokin.a ../lib/libgo.a @ULAPI_LIBS@ 
gokintest_DEPENDENCIES = ../lib/libgokin.a ../lib/libgo.a

gotestsh_SOURCES = ../src/gotestsh.c
gotestsh_LDADD = ../lib/libgo.a
gotestsh_DEPENDENCIES = ../lib/libgo.a

cartfit_SOURCES = ../src/cartfit.c 
cartfit_LDADD = ../lib/libgo.a
cartfit_DEPENDENCIES = ../lib/libgo.a

# gomain is Unix only, with gomain_mod its rtlib/ counterpart
gomain_SOURCES = \
../src/extintf.h \
../src/pid.c ../src/pid.h \
../src/servoloop.c ../src/servointf.h \
../src/trajloop.c ../src/trajintf.h \
../src/gomain.c
gomain_LDADD = ../lib/libgokin.a ../lib/libgo.a
if HAVE_S626
gomain_SOURCES += ../src/ext_s626.c ../src/dcmotor.c ../src/dcmotor.h
gomain_LDADD += -L@S626_DIR@ -ls626
else
gomain_SOURCES += ../src/ext_sim.c ../src/dcmotor.c ../src/dcmotor.h
endif
gomain_LDADD += @ULAPI_LIBS@ 
gomain_DEPENDENCIES = ../lib/libgokin.a ../lib/libgo.a

# gostepper is Unix only, with gostepper_mod its rtlib/ counterpart
gostepper_SOURCES = ../src/gostepper.c ../src/gostepper.h
gostepper_LDADD = ../lib/libgo.a @ULAPI_LIBS@ 
gostepper_DEPENDENCIES = ../lib/libgo.a

# gosteppercfg can use both Unix and RTAI ULAPI
gosteppercfg_SOURCES = ../src/gosteppercfg.c 
gosteppercfg_LDADD = ../lib/libgo.a @ULAPI_LIBS@ 
gosteppercfg_DEPENDENCIES = ../lib/libgo.a

# gosh can use both Unix and RTAI ULAPI
gosh_SOURCES = ../src/gosh.c ../src/gorcsutil.c ../src/gorcsutil.h ../src/servointf.h ../src/trajintf.h ../src/toolintf.h ../src/taskintf.c ../src/taskintf.h
gosh_LDADD = ../lib/libgo.a @ULAPI_LIBS@ 
gosh_DEPENDENCIES = ../lib/libgo.a

# gocfg can use both Unix and RTAI ULAPI
gocfg_SOURCES = ../src/gocfg.c ../src/pid.c ../src/gorcsutil.c ../src/gorcsutil.h ../src/servointf.h ../src/trajintf.h ../src/toolintf.h ../src/taskintf.c ../src/taskintf.h
gocfg_LDADD = ../lib/libgo.a @ULAPI_LIBS@ 
gocfg_DEPENDENCIES = ../lib/libgo.a

tracker_SOURCES = ../src/tracker.c
tracker_LDADD = ../lib/libgokin.a ../lib/libgo.a @ULAPI_LIBS@ 
tracker_DEPENDENCIES = ../lib/libgokin.a ../lib/libgo.a

igpsclient_SOURCES = ../src/igpsclient.c
igpsclient_LDADD = ../lib/libgokin.a ../lib/libgo.a @ULAPI_LIBS@ 
igpsclient_DEPENDENCIES = ../lib/libgokin.a ../lib/libgo.a

igpsserver_SOURCES = ../src/igpsserver.c
igpsserver_LDADD = ../lib/libgokin.a ../lib/libgo.a @ULAPI_LIBS@ 
igpsserver_DEPENDENCIES = ../lib/libgokin.a ../lib/libgo.a

# taskmain can use both Unix and RTAI ULAPI
taskmain_SOURCES = ../src/taskintf.c ../src/taskintf.h ../src/taskmain.c ../src/gorcsutil.c ../src/gorcsutil.h ../src/rs274ngc_pre.cc ../src/rs274ngc.h ../src/canon_go.cc ../src/interplist.c ../src/interplist.h
taskmain_LDADD = ../lib/libgo.a @ULAPI_LIBS@ 
taskmain_DEPENDENCIES = ../lib/libgo.a
taskmain_CFLAGS = -DAA -DBB -DCC
taskmain_CXXFLAGS = -DAA -DBB -DCC

tasksvr_SOURCES = ../src/tasksvr.c
tasksvr_LDADD = ../lib/libgo.a @ULAPI_LIBS@ 
tasksvr_DEPENDENCIES = ../lib/libgo.a

toolmain_SOURCES = \
../src/extintf.h ../src/toolintf.h \
../src/toolmain.c
toolmain_SOURCES += ../src/ext_sim.c ../src/dcmotor.c ../src/dcmotor.h
toolmain_LDADD = ../lib/libgo.a
toolmain_LDADD += @ULAPI_LIBS@ 
toolmain_DEPENDENCIES = ../lib/libgo.a

variates_SOURCES = ../src/variates.c ../src/variates.h
variates_LDADD = ../lib/libgo.a
variates_DEPENDENCIES = ../lib/libgo.a
variates_CFLAGS = -DMAIN

rs274ngc_SOURCES = ../src/rs274ngc_pre.cc ../src/rs274ngc.h ../src/rs274ngc_return.h ../src/canon_pre.cc ../src/canon.h ../src/rs274ngc_driver.cc
rs274ngc_CFLAGS = -DAA -DBB -DCC

if HAVE_TCL_INCLUDE
if HAVE_TCL_LIB

# gotcl is the Tcl-enabled script interface that doesn't require X Windows
# or a DISPLAY environment variable
gotcl_SOURCES = ../src/gotcl.c ../src/gotcltk.c ../src/gorcsutil.c ../src/gorcsutil.h ../src/servointf.h ../src/trajintf.h ../src/toolintf.h ../src/taskintf.c ../src/taskintf.h
gotcl_LDADD = ../lib/libgo.a @TCL_LINK@ @ULAPI_LIBS@ 
if HAVE_SDL
gotcl_LDADD += @SDL_LIBS@
endif
gotcl_DEPENDENCIES = ../lib/libgo.a

if HAVE_TK_INCLUDE
if HAVE_TK_LIB

# gotk is the Tk-enabled script interface that requires X Windows
# and a DISPLAY environment variable
gotk_SOURCES = ../src/gotk.c ../src/gotcltk.c ../src/gorcsutil.c ../src/gorcsutil.h ../src/servointf.h ../src/trajintf.h ../src/toolintf.h ../src/taskintf.c ../src/taskintf.h
gotk_CFLAGS = @X_CFLAGS@
gotk_LDADD = ../lib/libgo.a @TCL_LINK@ @TK_LINK@ @X_PRE_LIBS@ @X_LIBS@ @X_EXTRA_LIBS@ @ULAPI_LIBS@ 
if HAVE_SDL
gotk_LDADD += @SDL_LIBS@
endif
gotk_DEPENDENCIES = ../lib/libgo.a

endif
endif

endif
endif

gotestmmavg_SOURCES = ../src/gotestmmavg.c
gotestmmavg_LDADD = ../lib/libgo.a @ULAPI_LIBS@  

# Extras if we have IO ports
if HAVE_IOPORTS
bin_PROGRAMS += inb outb
inb_SOURCES = ../src/inb.c
inb_CFLAGS = -O2
outb_SOURCES = ../src/outb.c
outb_CFLAGS = -O2
endif

# stuff for Sensoray S626

if HAVE_S626

# INCLUDES += -I@S626_DIR@
AM_CPPFLAGS += -I@S626_DIR@

bin_PROGRAMS += gomain_s626

gomain_s626_SOURCES = \
../src/extintf.h \
../src/pid.c ../src/pid.h \
../src/servoloop.c ../src/servointf.h \
../src/trajloop.c ../src/trajintf.h \
../src/ext_s626.c ../src/dcmotor.c ../src/dcmotor.h \
../src/gomain.c
gomain_s626_LDADD = ../lib/libgokin.a ../lib/libgo.a
gomain_s626_LDADD += -L@S626_DIR@ -ls626
gomain_s626_LDADD += @ULAPI_LIBS@ 
gomain_s626_DEPENDENCIES = ../lib/libgokin.a ../lib/libgo.a

endif

# stuff for Galil

bin_PROGRAMS += gomain_galil emu_galil

gomain_galil_SOURCES = \
../src/extintf.h \
../src/pid.c ../src/pid.h \
../src/servoloop.c ../src/servointf.h \
../src/trajloop.c ../src/trajintf.h \
../src/gomain.c
gomain_galil_SOURCES += ../src/ext_galil.c
gomain_galil_LDADD = ../lib/libgokin.a ../lib/libgo.a @ULAPI_LIBS@ 
gomain_galil_DEPENDENCIES = ../lib/libgokin.a ../lib/libgo.a

emu_galil_SOURCES = ../src/emu_galil.c
emu_galil_LDADD = ../lib/libgo.a @ULAPI_LIBS@ 
emu_galil_DEPENDENCIES = ../lib/libgo.a

# stuff for Smart Motors

bin_PROGRAMS += gomain_smartmotor emu_smartmotor test_smartmotor

gomain_smartmotor_SOURCES = \
../src/extintf.h \
../src/pid.c ../src/pid.h \
../src/servoloop.c ../src/servointf.h \
../src/trajloop.c ../src/trajintf.h \
../src/gomain.c
gomain_smartmotor_SOURCES += ../src/ext_smartmotor.c
gomain_smartmotor_LDADD = ../lib/libgokin.a ../lib/libgo.a @ULAPI_LIBS@ 
gomain_smartmotor_DEPENDENCIES = ../lib/libgokin.a ../lib/libgo.a

emu_smartmotor_SOURCES = ../src/emu_smartmotor.c
emu_smartmotor_LDADD = ../lib/libgo.a @ULAPI_LIBS@ 
emu_smartmotor_DEPENDENCIES = ../lib/libgo.a

test_smartmotor_SOURCES = ../src/test_smartmotor.c
test_smartmotor_LDADD = ../lib/libgo.a @ULAPI_LIBS@ 
test_smartmotor_DEPENDENCIES = ../lib/libgo.a

if HAVE_EXTINTF
bin_PROGRAMS += gomain_extintf
gomain_extintf_SOURCES = \
../src/extintf.h \
../src/pid.c ../src/pid.h \
../src/servoloop.c ../src/servointf.h \
../src/trajloop.c ../src/trajintf.h \
../src/gomain.c
gomain_extintf_LDADD = @EXTINTF@
gomain_extintf_LDADD += ../lib/libgokin.a ../lib/libgo.a
gomain_extintf_LDADD += @ULAPI_LIBS@ 
gomain_extintf_DEPENDENCIES = ../lib/libgokin.a ../lib/libgo.a
endif

if HAVE_LIBGSL
bin_PROGRAMS += sixdof
sixdof_SOURCES = ../src/sixdof.c
sixdof_LDADD = ../lib/libgo.a
sixdof_DEPENDENCIES = ../lib/libgo.a
endif

if HAVE_MTCONNECT
# INCLUDES += -I@MTCONNECT_DIR@/include
AM_CPPFLAGS += -I@MTCONNECT_DIR@/include
bin_PROGRAMS += go_adapter
go_adapter_SOURCES = ../src/go_adapter.cpp ../src/go_adapter.hpp ../src/trajintf.h ../src/toolintf.h ../src/taskintf.c ../src/taskintf.h
go_adapter_LDADD = ../lib/libgo.a @ULAPI_LIBS@ @MTCONNECT_LIBS@
go_adapter_DEPENDENCIES = ../lib/libgo.a
endif

bin_PROGRAMS += toolmain_modbus
toolmain_modbus_SOURCES = \
../src/extintf.h ../src/toolintf.h \
../src/toolmain_modbus.c
toolmain_modbus_SOURCES += ../src/ext_sim.c ../src/dcmotor.c ../src/dcmotor.h
toolmain_modbus_LDADD = ../lib/libgo.a
toolmain_modbus_LDADD += @ULAPI_LIBS@ 
toolmain_modbus_DEPENDENCIES = ../lib/libgo.a

bin_PROGRAMS += smartmotor_md
smartmotor_md_SOURCES = \
../src/smartmotor_md.c
smartmotor_md_LDADD = @ULAPI_LIBS@ 
