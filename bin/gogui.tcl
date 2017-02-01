#!/bin/sh
# syntax: pendant.tcl {-- -i <inifile> -u unix | rtai}
# the next line restarts using gotk \
exec `dirname $0`/gotk "$0" "$@"

# get the inifile name
set inifile [gotk_inifile]

if {[string equal x$inifile x]} {
    wm title . "Go Motion"
} else {
    set name [gotk_ini NAME GOMOTION $inifile]
    if {[string equal x$name x]} {
	wm title . "Go Motion"
    } else {
	wm title . $name
    }
}

# set timeout to none when interactive in a GUI
gotk_set_timeout none

# load fresh data
gotk_update

set top [frame .top -borderwidth 2 -relief ridge]
pack $top -side top

# use ACT and CMD to indicate actual vs. commanded display values
set val_select "ACT"

# --- Joint display ---

# which joint is selected
set joint_select 1

set joint_number [gotk_joint_number]
set joint_rows [expr ($joint_number + 1) / 2]

# create all the joint position variables
for {set i 1} {$i <= $joint_number} {incr i 1} {
    set joint_pos_$i 0
}

set jf [frame $top.joint_frame -borderwidth 2 -relief ridge]
set jfn [frame $jf.name]
set jfl [frame $jf.left]
set jfr [frame $jf.right]
set jfa [frame $jf.actions]

label $jfn.val -text "Joint Positions"
grid $jfn.val

for {set row 1} {$row <= $joint_rows} {incr row 1} {
    set lx $row
    set rx [expr $row + $joint_rows]

    label $jfl.val_$lx -width 10 -anchor e -textvariable joint_pos_$lx
    label $jfl.num_$lx -text $lx -borderwidth 2 -relief ridge
    radiobutton $jfl.rad_$lx -variable joint_select -value $lx
    grid $jfl.val_$lx -row $row -column 0
    grid $jfl.num_$lx -row $row -column 1
    grid $jfl.rad_$lx -row $row -column 2

    if {$rx <= $joint_number} {
	label $jfr.val_$rx -width 10 -anchor e -textvariable joint_pos_$rx
	label $jfr.num_$rx -text $rx -borderwidth 2 -relief ridge
	radiobutton $jfr.rad_$rx -variable joint_select -value $rx
	grid $jfr.val_$rx -row $row -column 3
	grid $jfr.num_$rx -row $row -column 4
	grid $jfr.rad_$rx -row $row -column 5
    }
}

proc do_home {} {
    global joint_select
    gotk_traj_home $joint_select
}

button $jfa.jog_minus -text "Jog -"
button $jfa.home -text "Home" -command {do_home}
button $jfa.jog_plus -text "Jog +"

proc do_jog {dir} {
    global joint_select

    set jog_speed [expr 0.1 * $dir * [gotk_max_vel $joint_select]]

    gotk_traj_jog_joint $joint_select $jog_speed
}

proc do_stop {} {
    gotk_traj_stop
}

bind $jfa.jog_minus <ButtonPress-1> {do_jog -1}
bind $jfa.jog_minus <ButtonRelease-1> {do_stop}

bind $jfa.jog_plus <ButtonPress-1> {do_jog +1}
bind $jfa.jog_plus <ButtonRelease-1> {do_stop}

grid $jfa.jog_minus -row 0 -column 0
grid $jfa.home -row 0 -column 1
grid $jfa.jog_plus -row 0 -column 2

grid $jfn -row 0 -columnspan 5 -sticky n
grid $jfl -row 1 -column 0 -sticky n
grid $jfr -row 1 -column 1 -sticky n
grid $jfa -row 2 -columnspan 5 -sticky n
grid $jf

# --- Cartesian display ---

# which coordinate is selected
set cart_select 1

set x_pos 0
set y_pos 0
set z_pos 0
set r_pos 0
set p_pos 0
set w_pos 0

set cf [frame $top.cart_frame -borderwidth 2 -relief ridge]
set cfn [frame $cf.name]
set cfl [frame $cf.left]
set cfr [frame $cf.right]

label $cfn.val -text "Cartesian Position"
grid $cfn.val

label $cfl.val_x -width 10 -anchor e -textvariable x_pos
label $cfl.ltr_x -text "X" -borderwidth 2 -relief ridge
radiobutton $cfl.rad_x -variable cart_select -value 1
grid $cfl.val_x -row 0 -column 0
grid $cfl.ltr_x -row 0 -column 1
grid $cfl.rad_x -row 0 -column 2

label $cfl.val_y -width 10 -anchor e -textvariable y_pos
label $cfl.ltr_y -text "Y" -borderwidth 2 -relief ridge
radiobutton $cfl.rad_y -variable cart_select -value 2
grid $cfl.val_y -row 1 -column 0
grid $cfl.ltr_y -row 1 -column 1
grid $cfl.rad_y -row 1 -column 2

label $cfl.val_z -width 10 -anchor e -textvariable z_pos
label $cfl.ltr_z -text "Z" -borderwidth 2 -relief ridge
radiobutton $cfl.rad_z -variable cart_select -value 3
grid $cfl.val_z -row 2 -column 0
grid $cfl.ltr_z -row 2 -column 1
grid $cfl.rad_z -row 2 -column 2

label $cfr.val_r -width 10 -anchor e -textvariable r_pos
label $cfr.ltr_r -text "R" -borderwidth 2 -relief ridge
radiobutton $cfr.rad_r -variable cart_select -value 4
grid $cfr.val_r -row 0 -column 0
grid $cfr.ltr_r -row 0 -column 1
grid $cfr.rad_r -row 0 -column 2

label $cfr.val_p -width 10 -anchor e -textvariable p_pos
label $cfr.ltr_p -text "P" -borderwidth 2 -relief ridge
radiobutton $cfr.rad_p -variable cart_select -value 5
grid $cfr.val_p -row 1 -column 0
grid $cfr.ltr_p -row 1 -column 1
grid $cfr.rad_p -row 1 -column 2

label $cfr.val_w -width 10 -anchor e -textvariable w_pos
label $cfr.ltr_w -text "W" -borderwidth 2 -relief ridge
radiobutton $cfr.rad_w -variable cart_select -value 6
grid $cfr.val_w -row 2 -column 0
grid $cfr.ltr_w -row 2 -column 1
grid $cfr.rad_w -row 2 -column 2

grid $cfn -row 0 -columnspan 5
grid $cfl -row 1 -column 0 -sticky n
grid $cfr -row 1 -column 1 -sticky n
grid $cf

# --- Actions ---

proc update_status {} {
    global val_select
    global joint_number

    gotk_update

    for {set i 1} {$i <= $joint_number} {incr i 1} {
	global joint_pos_$i
	if {$val_select == "ACT"} {
	    set joint_pos_$i [format %f [gotk_joint_pos $i]]
	} else {
	    set joint_pos_$i [format %f [gotk_joint_cmd_pos $i]]
	}
    }

    global x_pos
    global y_pos
    global z_pos
    global r_pos
    global p_pos
    global w_pos

    set x_pos [format %f [gotk_world_pos "X"]]
    set y_pos [format %f [gotk_world_pos "Y"]]
    set z_pos [format %f [gotk_world_pos "Z"]]
    set r_pos [format %f [gotk_world_pos "R"]]
    set p_pos [format %f [gotk_world_pos "P"]]
    set w_pos [format %f [gotk_world_pos "W"]]
    
    after 200 update_status
}

update_status

