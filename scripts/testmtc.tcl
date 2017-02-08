#!/bin/sh
# the next line restarts using gotcl \
exec /usr/local/bin/gotcl "$0" -- "$@"

gotk_set_timeout forever

proc homeall {} {
    set max [gotk_servo_num]
    for {set j 1} {$j <= $max} {set j [expr $j + 1]} {
	if {[gotk_joint_active $j]} {
	    if {! [gotk_joint_homed $j]} {
		gotk_traj_home $j
	    }
	}
    }
}

proc runit {p} {
    gotk_task reset
    gotk_task start "$p -i [gotk_inifile]"
}

if {! [string equal [gotk_traj_admin_state] ADMIN_INITIALIZED]} {
    gotk_traj_init
}
gotk_traj_hold

homeall
gotk_task abort
gotk_task clear

while {true} {
    runit "../scripts/raster.tcl"
    if {[gotk_task status] == "Error"} {
	break
    }
}

exit 0

