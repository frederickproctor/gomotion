#!/bin/bash

#
# Usage: gorun.sh [-i <inifile>] [-u unix | rtai ] [-m <name>] [-s] [-w] [-d] [-k]
# 
# <inifile> is optional, defaults to gomotion.ini
# 
# 'rtai' or 'unix' specify whether to run real time with rtai
# or simulated with unix.
#
# <name> is optional, and will set the name of the main program
# and kernel module. Normally these are 'gomain' and 'gomain_mod'.
# If <name> is, for example, 'roboch', these would be
# 'gomain_roboch' and 'gomain_roboch_mod'. 
#
# -s means run the go shell, gosh, instead of the GUI pendant
# -w means wait for gomain, instead of running gosh or the pendant
# -d means pass a debug flag if available
# -k means kill all other Go Motion applications that may be running

USAGE="usage : [-i <inifile>] [-u unix | rtai] [-m <name of main>] [-s] [-d] [-k]"

UTS_RELEASE=`uname -r | cut -f 1,2 -d .`

thisdir=`dirname $0`
inifile=
do_wait=
use_gosh=
debugarg=
debugval=0
killgo=

# set the default platform based on the existence of the 'rtaidir' file
if [ -f $thisdir/../rtaidir ] ; then
    do_rtai=yes
    u_arg=rtai
else
    do_rtai=no
    u_arg=unix
fi

while getopts 'i:u:m:swdk?' c
do
    case $c in
	i)
	    inifile=$OPTARG
	    ;;
	u)
	    if [ x"$OPTARG" = xrtai ] ; then
		if [ -f $thisdir/../rtaidir ] ; then
		    do_rtai=yes
		    u_arg=rtai
		else
		    echo gorun.sh: no rtai available
		    exit 1
		fi
	    elif [ x"$OPTARG" = xunix ] ; then
		do_rtai=no
		u_arg=unix
	    fi
	    ;;
	m)
	    name=$OPTARG
	    ;;
	w)
	    do_wait=1
	    ;;
	s)
	    use_gosh=1
	    ;;
	d)
	    debugarg=-d
	    debugval=1
	    ;;
	k)
	    killgo=yes
	    ;;
	\?)
	    echo gorun.sh: $USAGE
	    exit 1
	    ;;
    esac
done

if [ x"$inifile" = x ] ; then
    inifile=gomotion.ini
    echo gorun.sh: using $inifile by default
fi

if [ ! -f "$inifile" ] ; then
    echo gorun.sh: can\'t find $inifile
    exit 1
fi

if [ x"$name" = x ] ; then
    main=gomain
    mainmod=gomain_mod
else
    main=gomain_$name
    mainmod=gomain_$name_mod
fi

#
# Get any command-line args to pass from ini file
# 
# To set up manually, do this, for example:
#
# export thisdir=bin
# export main=gomain
# export inifile=etc/genhex.ini
# 
# then paste the following lines into the shell
# 

if [ -f $thisdir/../ulapi_dir ] ; then
    . $thisdir/../ulapi_dir
else
    ULAPI_DIR=/usr/local/ulapi
fi

SERVO_HOWMANY=`$ULAPI_DIR/bin/inifind HOWMANY SERVO $inifile`
if [ x"$SERVO_HOWMANY" = x ] ; then
    num=1
    while true ; do
	if [ x`$ULAPI_DIR/bin/inifind CYCLE_TIME SERVO_$num $inifile` = x ] ; then break ; fi
	SERVO_HOWMANY=$num
	num=$(($num + 1))
    done
    echo gorun.sh: [SERVO] HOWMANY not found, using $SERVO_HOWMANY by counting
fi

RTAPI_HAL_NSECS_PER_PERIOD=`$ULAPI_DIR/bin/inifind NSECS_PER_PERIOD RTAPI_HAL $inifile`
if [ x"$RTAPI_HAL_NSECS_PER_PERIOD" = x ] ; then RTAPI_HAL_NSECS_PER_PERIOD=0 ; fi
GO_STEPPER_TYPE=`$ULAPI_DIR/bin/inifind TYPE GO_STEPPER $inifile`
GO_STEPPER_SHM_KEY=`$ULAPI_DIR/bin/inifind SHM_KEY GO_STEPPER $inifile`
if [ x"$GO_STEPPER_SHM_KEY" = x ] ; then GO_STEPPER_SHM_KEY=0 ; fi
TRAJ_SHM_KEY=`$ULAPI_DIR/bin/inifind SHM_KEY TRAJ $inifile`
if [ x"$TRAJ_SHM_KEY" = x ] ; then TRAJ_SHM_KEY=0 ; fi
SERVO_SHM_KEY=`$ULAPI_DIR/bin/inifind SHM_KEY SERVO $inifile`
if [ x"$SERVO_SHM_KEY" = x ] ; then SERVO_SHM_KEY=0 ; fi
SERVO_SEM_KEY=`$ULAPI_DIR/bin/inifind SEM_KEY SERVO $inifile`
if [ x"$SERVO_SEM_KEY" = x ] ; then SERVO_SEM_KEY=0 ; fi
EXT_INIT_STRING=`$ULAPI_DIR/bin/inifind EXT_INIT_STRING GOMOTION $inifile`
if [ x"$EXT_INIT_STRING" = x ] ; then EXT_INIT_STRING=0 ; fi
KINEMATICS=`$ULAPI_DIR/bin/inifind KINEMATICS TRAJ $inifile`
if [ x"$KINEMATICS" = x ] ; then KINEMATICS=0 ; fi
GO_LOG_SHM_KEY=`$ULAPI_DIR/bin/inifind SHM_KEY GO_LOG $inifile`
if [ x"$GO_LOG_SHM_KEY" = x ] ; then GO_LOG_SHM_KEY=0 ; fi
GO_IO_SHM_KEY=`$ULAPI_DIR/bin/inifind SHM_KEY GO_IO $inifile`
if [ x"$GO_IO_SHM_KEY" = x ] ; then GO_IO_SHM_KEY=0 ; fi
TOOL_SHM_KEY=`$ULAPI_DIR/bin/inifind SHM_KEY TOOL $inifile`
if [ x"$TOOL_SHM_KEY" = x ] ; then TOOL_SHM_KEY=0 ; fi
TASK_SHM_KEY=`$ULAPI_DIR/bin/inifind SHM_KEY TASK $inifile`
if [ x"$TASK_SHM_KEY" = x ] ; then TASK_SHM_KEY=0 ; fi
TASK_SERVER_PORT=`$ULAPI_DIR/bin/inifind TCP_PORT TASK $inifile`

pid1=0
pid2=0
pid3=0
pid4=0
pid5=0

cleanup () {
    if [ x$do_rtai = xyes ] ; then
	sudo rmmod $mainmod 2> /dev/null
	sudo rmmod gostepper_mod 2> /dev/null
	sudo rmmod rtapi_hal_mod 2> /dev/null
    fi

    if [ ! $pid5 = 0 ] ; then kill -INT $pid5 ; fi
    if [ ! $pid4 = 0 ] ; then kill -INT $pid4 ; fi
    if [ ! $pid3 = 0 ] ; then kill -INT $pid3 ; fi
    if [ ! $pid2 = 0 ] ; then kill -INT $pid2 ; fi
    if [ ! $pid1 = 0 ] ; then kill -INT $pid1 ; fi

    if [ ! x$debugarg = x ] ; then echo gorun cleaning up with code $1 ; fi

    exit $1
}

#
# Test for RTAI, and run that version of the code if found,
# otherwise run the Unix simulated version
#

if [ x$do_rtai = xyes ] ; then

# we're RTAI
    $thisdir/insrtl 2> /dev/null
    if [ "$UTS_RELEASE" = "2.4" ] ; then rtapi_hal_mod=$ULAPI_DIR/lib/rtapi_hal_mod.o ; else rtapi_hal_mod=$ULAPI_DIR/lib/rtapi_hal_mod.ko ; fi
    sudo insmod -f $rtapi_hal_mod RTAPI_HAL_NSECS_PER_PERIOD=$RTAPI_HAL_NSECS_PER_PERIOD DEBUG=$debugval
# run the stepper controller, if indicated
    if [ ! x"$GO_STEPPER_TYPE" = x ] ; then
	for mod in $thisdir/../rtlib/{gostepper_mod.ko,gostepper_mod.o} ; do
	    if test -f $mod ; then
		sudo insmod -f $mod GO_STEPPER_TYPE=$GO_STEPPER_TYPE GO_STEPPER_SHM_KEY=$GO_STEPPER_SHM_KEY
		break
	    fi
	done
    fi
# run the main controller
    for mod in $thisdir/../rtlib/{$mainmod.ko,$mainmod.o} ; do
	if test -f $mod ; then
	    sudo insmod -f $mod DEBUG=$debugval TRAJ_SHM_KEY=$TRAJ_SHM_KEY SERVO_HOWMANY=$SERVO_HOWMANY SERVO_SHM_KEY=$SERVO_SHM_KEY SERVO_SEM_KEY=$SERVO_SEM_KEY EXT_INIT_STRING="$EXT_INIT_STRING" KINEMATICS=$KINEMATICS GO_LOG_SHM_KEY=$GO_LOG_SHM_KEY GO_IO_SHM_KEY=$GO_IO_SHM_KEY
	    break
	fi
    done
# run the tool controller, if indicated
    if [ ! x"$TOOL_SHM_KEY" = x ] ; then
	for mod in $thisdir/../rtlib/{toolmod.ko,toolmod.o} ; do
	    if test -f $mod ; then
		sudo insmod -f $mod DEBUG=$debugval TOOL_SHM_KEY=$TOOL_SHM_KEY
		break
	    fi
	done
    fi

else

# we're Unix

    if [ x"$killgo" = xyes ] ; then
	killall gostepper 2> /dev/null
	killall $main 2> /dev/null
    fi
# run the stepper controller, if indicated
    if [ ! x"$GO_STEPPER_TYPE" = x ] ; then
	$thisdir/gostepper GO_STEPPER_TYPE=$GO_STEPPER_TYPE GO_STEPPER_SHM_KEY=$GO_STEPPER_SHM_KEY &
	pid1=$!
    fi
# run the main controller, something like bin/gomain DEBUG=1 TRAJ_SHM_KEY=201 SERVO_HOWMANY=6 SERVO_SHM_KEY=101 SERVO_SEM_KEY=101 EXT_INIT_STRING=I KINEMATICS=genhexkins GO_LOG_SHM_KEY=1001 GO_IO_SHM_KEY=1002
    $thisdir/$main DEBUG=$debugval TRAJ_SHM_KEY=$TRAJ_SHM_KEY SERVO_HOWMANY=$SERVO_HOWMANY SERVO_SHM_KEY=$SERVO_SHM_KEY SERVO_SEM_KEY=$SERVO_SEM_KEY EXT_INIT_STRING="$EXT_INIT_STRING" KINEMATICS=$KINEMATICS GO_LOG_SHM_KEY=$GO_LOG_SHM_KEY GO_IO_SHM_KEY=$GO_IO_SHM_KEY &
    pid2=$!
# run the tool controller, if indicated
    if [ ! "$TOOL_SHM_KEY" = "0" ] ; then
	$thisdir/toolmain DEBUG=$debugval TOOL_SHM_KEY=$TOOL_SHM_KEY &
	pid3=$!
    fi

# end of rtai else unix
fi

# run the task controller, if indicated
if [ ! "$TASK_SHM_KEY" = "0" ] ; then
    $thisdir/taskmain -i $inifile $debugarg &
    pid4=$!
fi

# run the task server, if indicated
if [ ! x"$TASK_SERVER_PORT" = x ] ; then
    $thisdir/tasksvr -p $TASK_SERVER_PORT -i $inifile $debugarg &
    pid5=$!
fi

# configure the stepper controller, if indicated
if [ ! x"$GO_STEPPER_TYPE" = x ] ; then
    $thisdir/gosteppercfg -i $inifile -u $u_arg $debugarg
    if [ $? -ne 0 ] ; then cleanup 1 ; fi
fi

# configure the main controller; ignore debug arg since it clutters
$thisdir/gocfg -i $inifile -u $u_arg 
if [ $? -ne 0 ] ; then cleanup 1 ; fi

# no need to configure the tool controller
# and no likewise for the task controller

# We can stop the 'gomain' process by either passing the -k option to
# pendant.tcl, in which case it sends a shutdown message when it quits,
# or via the 'cleanup' function that kills both 'gomain' and
# 'gostepper'. Since pendant.tcl's -k option doesn't kill gostepper,
# we'll use cleanup instead. On Windows systems, where 'gostepper' is
# not run, we use the -k option since killing processes in scripts is
# hard to achieve.

trap "cleanup 2" SIGINT

# run the pendant in the foreground; when it quits, everything else should
if [ ! x"$do_wait" = x ] ; then
    wait $pid2
elif [ ! x"$use_gosh" = x ] ; then
    $thisdir/gosh -i $inifile -u $u_arg
else
    $thisdir/pendant.tcl -- -i $inifile -u $u_arg $debugarg
fi

cleanup 0
