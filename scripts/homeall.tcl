#!/bin/sh
# the next line restarts using gotk \
exec /usr/local/bin/gotcl "$0" -- "$@"

set inifile [gotk_inifile]
set homestr [gotk_ini HOME TRAJ $inifile]

set homex [lindex $homestr 0]
set homey [lindex $homestr 1]
set homez [lindex $homestr 2]
set homer [lindex $homestr 3]
set homep [lindex $homestr 4]
set homew [lindex $homestr 5]

gotk_set_timeout forever
gotk_traj_set_move_time 0

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

if {! [string equal [gotk_traj_admin_state] ADMIN_INITIALIZED]} {
    gotk_traj_init
}

homeall

gotk_traj_move_world $homex $homey $homez $homer $homep $homew

gotk_traj_hold

exit 0

