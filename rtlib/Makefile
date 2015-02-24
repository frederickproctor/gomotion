# Get Linux kernel version major and minor, e.g., 2.6
UTS_RELEASE = $(shell uname -r | cut -f 1,2 -d .)

ifeq ($(UTS_RELEASE), 2.4)

ifdef MAKING_MODULES

EXTRA_CFLAGS += -I$(ULAPI_DIR)/include -I/usr/include -I/usr/realtime/include -ffast-math -mhard-float -D__IN_RTAI__ -DEXPORT_SYMTAB

obj-m += gomain_mod.o gostepper_mod.o toolmain_mod.o

gomain_mod-objs := \
go.o gotypes.o gomath.o goutil.o gotraj.o gomotion.o gointerp.o golog.o \
servoloop.o trajloop.o gomain.o \
dcmotor.o pid.o \
fanuckins.o spheristkins.o genhexkins.o genserkins.o pumakins.o scarakins.o trivkins.o tripointkins.o three21kins.o kinselect.o \
ext_stepper.o

gostepper_mod-objs := gostepper.o

toolmain_mod-objs := toolmain.o \
go.o gotypes.o gomath.o goutil.o ext_stub.o

### custom depends for 2.4

obj-m += gomain_profi_mod.o

gomain_profi_mod-objs := \
go.o gotypes.o gomath.o goutil.o gotraj.o gomotion.o gointerp.o golog.o \
servoloop.o trajloop.o gomain.o \
dcmotor.o pid.o \
fanuckins.o spheristkins.o genhexkins.o genserkins.o pumakins.o scarakins.o trivkins.o tripointkins.o three21kins.o kinselect.o \
ext_profi.o robocrane.o ProfibusIOInterface.o

### end custom depends for 2.4

include $(TOPDIR)/Rules.make

gomain_mod.o:  $(gomain_mod-objs)
	$(LD) $(LD_RFLAG) -r -o $@ $(gomain_mod-objs)

gostepper_mod.o:  $(gostepper_mod-objs)
	$(LD) $(LD_RFLAG) -r -o $@ $(gostepper_mod-objs)

toolmain_mod.o:  $(toolmain_mod-objs)
	$(LD) $(LD_RFLAG) -r -o $@ $(toolmain_mod-objs)

### custom builds for 2.4

CIF_DIR = /usr/local/cif_rtai

ProfibusIOInterface.o : ProfibusIOInterface.c
	gcc -D__KERNEL__ -DMODULE -Wall -O2 -I/lib/modules/`uname -r`/build/include -I/usr/realtime/include -I$(CIF_DIR)/inc -I$(CIF_DIR)/usr-inc $< -c -o $@

gomain_profi_mod.o:  $(gomain_profi_mod-objs)
	$(LD) $(LD_RFLAG) -r -o $@ $(gomain_profi_mod-objs)

### end custom builds for 2.4

endif

clean :
	\rm -f *.o

else
# not 2.4, try 2.6

ifeq ($(UTS_RELEASE), 2.6)

# Linux 2.6 kernel module compilation

# -ffast-math -mhard-float build in references to symbols like
# __fixunsdfsi
# __adddf3
# __fixdfsi
# __ledf2
# __divdf3
# __muldf3
# __gedf2
# __subdf3

EXTRA_CFLAGS += -I$(ULAPI_DIR)/include -I/usr/include -I/usr/realtime/include -ffast-math -mhard-float -D__IN_RTAI__ -DEXPORT_SYMTAB

obj-m +=  gomain_mod.o gostepper_mod.o toolmain_mod.o

gomain_mod-objs := \
go.o gotypes.o gomath.o goutil.o gotraj.o gomotion.o gointerp.o golog.o \
servoloop.o trajloop.o gomain.o \
dcmotor.o pid.o \
fanuckins.o spheristkins.o genhexkins.o genserkins.o pumakins.o scarakins.o trivkins.o tripointkins.o three21kins.o kinselect.o \
ext_stepper.o

gostepper_mod-objs := gostepper.o

toolmain_mod-objs := toolmain.o \
go.o gotypes.o gomath.o goutil.o ext_stub.o

clean :
	- \rm -f *.o *.ko

endif
# else not 2.4 or 2.6

endif
