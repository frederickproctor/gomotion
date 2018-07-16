#!/bin/sh
# syntax: pendant.tcl {-- -i <inifile> -u unix | rtai}
# the next line restarts using gotk \
exec `dirname $0`/gotk "$0" "$@"

# Can debug with this:
# gdb `dirname $0`/gotk
# and run in gdb with "r bin/pendant.tcl -- -i <inifile>"

# $::argv0 is the program name
# [lindex $::argv 0] is '-i'
# [lindex $::argv 1] is 'gomotion.ini'

# set this directory name
set thisdir [file dirname $argv0]

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

# read the application defaults
set rcfiles [list $env(HOME)/.gotkrc ../etc/gotkrc]
foreach f $rcfiles {
    if { [file exists $f] } {
	option readfile $f
	break
    }
}

##############################################################################
# balloon.tcl - procedures used by balloon help
#
# Copyright (C) 1996-1997 Stewart Allen
# 
# This is part of vtcl source code
# Adapted for general purpose by 
# Daniel Roche <daniel.roche@bigfoot.com>
# thanks to D. Richard Hipp for the multi-headed display fix
# version 1.2 ( Sep 21 2000 ) 
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
#
# Modification history:
#  5-Jul-2001  FMP added -justify left to the balloon help label;
#              enable_balloons
##############################################################################

set do_balloons 0

proc enable_balloons {e} {
    global do_balloons
    set do_balloons $e
}

bind Bulle <Enter> {
    global do_balloons
    if {$do_balloons} {
	set Bulle(set) 0
	set Bulle(first) 1
	set Bulle(id) [after 500 {balloon %W $Bulle(%W) %X %Y}]
    }
}

bind Bulle <Button> {
    global do_balloons
    if {$do_balloons} {
	set Bulle(first) 0
	kill_balloon
    }
}

bind Bulle <Leave> {
    global do_balloons
    if {$do_balloons} {
	set Bulle(first) 0
	kill_balloon
    }
}

bind Bulle <Motion> {
    global do_balloons
    if {$do_balloons} {
	if {$Bulle(set) == 0} {
	    after cancel $Bulle(id)
	    set Bulle(id) [after 500 {balloon %W $Bulle(%W) %X %Y}]
	}
    }
}

proc set_balloon {target message} {
    global Bulle
    set Bulle($target) $message
    bindtags $target "[bindtags $target] Bulle"
}

proc kill_balloon {} {
    global Bulle
    after cancel $Bulle(id)
    if {[winfo exists .balloon] == 1} {
        destroy .balloon
    }
    set Bulle(set) 0
}

proc balloon {target message {cx 0} {cy 0} } {
    global Bulle
    if {$Bulle(first) == 1 } {
        set Bulle(first) 2
	if { $cx == 0 && $cy == 0 } {
	    set x [expr [winfo rootx $target] + ([winfo width $target]/2)]
	    set y [expr [winfo rooty $target] + [winfo height $target] + 4]
	} else {
	    set x [expr $cx + 4]
	    set y [expr $cy + 4]
	}
        toplevel .balloon -bg black -screen [winfo screen $target]
        wm overrideredirect .balloon 1
        label .balloon.l \
            -text $message -justify left -relief flat \
            -bg #ffffaa -fg black -padx 2 -pady 0 -anchor w
        pack .balloon.l -side left -padx 1 -pady 1
        wm geometry .balloon +${x}+${y}
        set Bulle(set) 1
    }
}

##############################################################################
# end of balloon.tcl
##############################################################################

# read out Go's comm buffers so that we can start with fresh data
gotk_update

# get the coordinate system out of gotk, one of joint, world or tool
set csys_select [gotk_get_csys_select]

# get the selected joint out of gotk, indexed starting at 1
set joint_select [gotk_get_joint_select]

# get the selected Cartesian axis out of gotk, X Y Z ...
set cart_select [gotk_get_cart_select]

# get the owner of operator input, non-zero means us
set input_control [gotk_get_input_control]

# build an array that associates numbers to Cartesian letters
set cart_letter(1) X
set cart_letter(2) Y
set cart_letter(3) Z
set cart_letter(4) R
set cart_letter(5) P
set cart_letter(6) W
set cart_letter(7) A
set cart_letter(8) B
# and one that associates Cartesian letters to numbers
set cart_number(X) 1
set cart_number(Y) 2
set cart_number(Z) 3
set cart_number(R) 4
set cart_number(P) 5
set cart_number(W) 6
set cart_number(A) 7
set cart_number(B) 8

# 
# 'whichselect' is the integer associated with the position radio button.
# It is manually associated with joint_select or cart_select in
# 'do_whichselect' below.
#
if {$csys_select == "joint"} {
    set whichselect $joint_select
} else {
    set whichselect $cart_number($cart_select)
}

# set values for max vel, acc and jerk
foreach item {1 2 3 4 5 6 7 8} {
    set maxvel($item) [gotk_max_vel $item]
    set maxacc($item) [gotk_max_acc $item]
    set maxjerk($item) [gotk_max_jerk $item]
}

set maxtvel [gotk_max_tvel]
set maxtacc [gotk_max_tacc]
set maxtjerk [gotk_max_tjerk]
set maxrvel [gotk_max_rvel]
set maxracc [gotk_max_racc]
set maxrjerk [gotk_max_rjerk]

set jogspeed $maxvel($joint_select)

set maxscale [gotk_max_scale]
set maxscalev [gotk_max_scale_v]
set maxscalea [gotk_max_scale_a]

# adds a simple separator
set sep_count 1
proc add_sep {p w} {
    global sep_count

    set f [frame $p.sep$sep_count]
    label $f.label -text $w
    pack $f -side top
    pack $f.label -side left

    incr sep_count
}

set d [frame .display -borderwidth 2 -relief ridge]
pack $d -side top
foreach row {1 2 3 4} {
    frame $d.row$row
    set number$row [label $d.row$row.posl -textvariable pos$row -width 10 -anchor e -font {Helvetica 12 bold}]
    label $d.row$row.namel -textvariable name$row -width 2 -relief ridge -justify center
    radiobutton $d.row$row.rbl -variable whichselect -value $row -command do_whichselect
    set number[expr $row + 4] [label $d.row$row.posr -textvariable pos[expr $row + 4] -width 10 -anchor e -font {Helvetica 12 bold}]
    label $d.row$row.namer -textvariable name[expr $row + 4] -width 2 -relief ridge -justify center
    radiobutton $d.row$row.rbr -variable whichselect -value [expr $row + 4] -command do_whichselect
    pack $d.row$row -side top
    pack $d.row$row.posl $d.row$row.namel $d.row$row.rbl $d.row$row.posr $d.row$row.namer $d.row$row.rbr -side left
}

set bd [frame .body]
pack $bd

add_sep $bd ""

set cs [frame $bd.cs]
pack $cs -side top
set jointbutton [radiobutton $cs.joint -variable csys_select -value joint -text Joint -command name_joint]
set worldbutton [radiobutton $cs.world -variable csys_select -value world -text World -command name_world]
set toolbutton [radiobutton $cs.tool -variable csys_select -value tool -text Tool -command name_tool]
pack $cs.joint $cs.world $cs.tool -side left

# actref is 0 for commanded position, 1 for actual position
set actref 1

set as [frame $bd.as]
pack $as -side top
set actbutton [radiobutton $as.act -variable actref -value 1 -text Actual -command name_act]
set cmdbutton [radiobutton $as.cmd -variable actref -value 0 -text Commanded -command name_cmd]
pack $as.act $as.cmd -side left

set spl [frame $bd.speed]
pack $spl -side top
label $spl.label -text Speed:
if {$csys_select == "joint"} {
    scale $spl.scale -orient horizontal -length 180 -from 0 -to $maxvel($whichselect) -resolution [expr 0.01 * $maxvel($whichselect)] -variable jogspeed -showvalue false
} else {
    scale $spl.scale -orient horizontal -length 180 -from 0 -to $maxtvel -resolution [expr 0.01 * $maxtvel] -variable jogspeed -showvalue false
}
label $spl.speed -textvariable jogspeed -width 5
pack $spl.label $spl.scale $spl.speed -side left

# note that our scale values are 100X the real scale values,
# since they're percents
set scalevalue 100
set scl [frame $bd.scale]
pack $scl -side top
label $scl.label -text Scale:
scale $scl.scale -orient horizontal -length 180 -from 0 -to [expr $maxscale * 100] -variable scalevalue -showvalue false -command do_cfgscale
label $scl.speed -textvariable scalevalue -width 5
pack $scl.label $scl.scale $scl.speed -side left

set jb [frame $bd.jogbuttons]
set jogplusbutton [button $jb.jogplus -text Jog+]
set movebutton [button $jb.move -text Move... -width 6 -command {popup_move}]
set stopbutton [button $jb.stop -text Stop -width 6 -command {do_stop}]
set jogminusbutton [button $jb.jogminus -text Jog-]
pack $jb -side top
pack $jb.jogminus $jb.move $jb.stop $jb.jogplus -side left

set hb [frame $bd.homebuttons]
set homebutton [button $hb.home -text Home -width 6 -command {do_home $whichselect}]
set herebutton [button $hb.here -text Here... -width 6 -command {popup_here}]
pack $hb -side top
pack $hb.home $hb.here -side left

add_sep $bd -----

set ub [frame $bd.utilbuttons]
button $ub.setup -text Setup... -command "popup_setup .setupwindow"
button $ub.log -text Log... -command "popup_log .logwindow out.log"
button $ub.io -text IO... -command "popup_io .iowindow"
button $ub.script -text Run... -command "popup_script"
pack $ub
pack $ub.setup $ub.log $ub.io $ub.script -side left

add_sep $bd -----

set qb [frame $bd.quitbuttons]
set resetbutton [button $qb.reset -text Reset -command {do_init}]
set quitbutton [button $qb.quit -text Quit -command {exit}]
pack $qb
pack $qb.reset $qb.quit -side left

# 
# do_input_control handles the enabling and disabling of the
# jog, move and home buttons according to the state of the
# input control variable. With an argument, it sets the value
# of the input control, 0 giving up control and 1 taking it.
# With no argument, leaves the input control alone and just
# changes the active state of the buttons.
#
proc do_input_control {{w -1}} {
    global jogplusbutton jogminusbutton homebutton movebutton

    if {$w != -1} {
	gotk_set_input_control $w
    }

    if {[gotk_get_input_control]} {
	if {[gotk_get_csys_select] == "joint"} {
	    $jogplusbutton config -state active
	    $jogminusbutton config -state active
	    $homebutton config -state active
	    $movebutton config -state disabled
	    bind $jogplusbutton <ButtonPress-1> {do_jog +1}
	    bind $jogplusbutton <ButtonRelease-1> gotk_traj_stop
	    bind $jogminusbutton <ButtonPress-1> {do_jog -1}
	    bind $jogminusbutton <ButtonRelease-1> gotk_traj_stop
	} else {
	    if {[gotk_world_homed]} {
		$jogplusbutton config -state active
		$jogminusbutton config -state active
		$homebutton config -state disabled
		$movebutton config -state active
		bind $jogplusbutton <ButtonPress-1> {do_jog +1}
		bind $jogplusbutton <ButtonRelease-1> gotk_traj_stop
		bind $jogminusbutton <ButtonPress-1> {do_jog -1}
		bind $jogminusbutton <ButtonRelease-1> gotk_traj_stop
	    } else {
		$jogplusbutton config -state disabled
		$jogminusbutton config -state disabled
		$homebutton config -state disabled
		$movebutton config -state disabled
		bind $jogplusbutton <ButtonPress-1> {}
		bind $jogplusbutton <ButtonRelease-1> {}
		bind $jogminusbutton <ButtonPress-1> {}
		bind $jogminusbutton <ButtonRelease-1> {}
	    }
	}
    } else {
	$jogplusbutton config -state disabled
	$jogminusbutton config -state disabled
	$homebutton config -state disabled
	$movebutton config -state disabled
	bind $jogplusbutton <ButtonPress-1> {}
	bind $jogplusbutton <ButtonRelease-1> {}
	bind $jogminusbutton <ButtonPress-1> {}
	bind $jogminusbutton <ButtonRelease-1> {}
    }
}

set bb [frame $bd.bottombuttons]
checkbutton $bb.balloons -text ? -indicatoron 0 -variable var_do_balloons -command {enable_balloons $var_do_balloons}
$bb.balloons configure -selectcolor [$bb.balloons cget -background]
pack $bb -side left
pack $bb.balloons -side left
if {[gotk_have_joystick]} {
    checkbutton $bb.input_control -text Grab -indicatoron 0 -variable input_control -command {do_input_control 1}
    $bb.input_control configure -selectcolor [$bb.input_control cget -background]
    label $bb.joystick -text "Joysticks Available"
    pack $bb.input_control $bb.joystick -side left
}

proc popup_message {message} {
    set w .message
    if {[winfo exists $w]} {
        destroy $w

    }
    toplevel $w
    wm title $w Message
    message $w.msg -text $message
    frame $w.buttons
    button $w.buttons.dismiss -default active -text Dismiss -command "destroy $w"
    pack $w.msg -side top
    pack $w.buttons -side bottom
    pack $w.buttons.dismiss -side left
    bind $w <Return> "destroy $w"
}

proc update_log {w} {
    global whichselect
    global logtype logsize loglogging loghowmany

    set logtype [gotk_log_type]
    set logsize [gotk_log_size]
    if {[gotk_log_logging $whichselect] == 0} {
	set loglogging Off
    } else {
	set loglogging On
    }
    set loghowmany [gotk_log_howmany]

    if {[winfo exists $w]} {
	after 200 "update_log $w"
    }
}

proc save_log {logfile} {
    set newfile [tk_getSaveFile -initialfile new.log]

    if {[catch {file copy -force $logfile $newfile} errstring] != 0} {
	popup_message $errstring
    }
}

proc popup_log {w logfile} {
    global csys_select whichselect
    global logtype logsize loglogging loghowmany

    if {[winfo exists $w]} {
	wm deiconify $w
	raise $w
	focus $w
	return
    }

    toplevel $w
    wm title $w "Go Motion Logging"
    bind $w <Return> "destroy $w"

    frame $w.logbuttons
    menubutton $w.logbuttons.loginit -text Log... -menu $w.logbuttons.loginit.menu -relief raised
    set m [menu $w.logbuttons.loginit.menu]
    if {$csys_select == "joint"} {
	$m add command -label Ferror -command "gotk_log_init Ferror 10000 $whichselect"
	$m add command -label Input -command "gotk_log_init Input 10000 $whichselect"
	$m add command -label Setpoint -command "gotk_log_init Setpoint 10000 $whichselect"
	$m add command -label Speed -command "gotk_log_init Speed 10000 $whichselect"
	button $w.logbuttons.logstart -text Start -command "gotk_log_start $whichselect"
	button $w.logbuttons.logstop -text Stop -command "gotk_log_stop $whichselect"
    } else {
	$m add command -label ActPos -command "gotk_log_init ActPos 10000"
	$m add command -label CmdPos -command "gotk_log_init CmdPos 10000"
	$m add command -label Xinv -command "gotk_log_init Xinv 10000"
	$m add command -label MagXinv -command "gotk_log_init MagXinv 10000"
	button $w.logbuttons.logstart -text Start -command gotk_log_start
	button $w.logbuttons.logstop -text Stop -command gotk_log_stop
    }
    button $w.logbuttons.logdump -text Dump -command "gotk_log_dump $logfile"
    button $w.logbuttons.logplot -text Plot -command "plot_log $logfile"
    button $w.logbuttons.logsave -text Save... -command "save_log $logfile"
    pack $w.logbuttons -side top
    pack $w.logbuttons.loginit $w.logbuttons.logstart $w.logbuttons.logstop $w.logbuttons.logdump $w.logbuttons.logplot $w.logbuttons.logsave -side left

    frame $w.logstatus
    label $w.logstatus.logtype -textvariable logtype
    label $w.logstatus.loglogging -textvariable loglogging
    label $w.logstatus.logsize -textvariable logsize
    label $w.logstatus.loghowmany -textvariable loghowmany
    pack $w.logstatus -side top
    pack $w.logstatus.logtype $w.logstatus.loglogging $w.logstatus.logsize $w.logstatus.loghowmany -side left

    set qb [frame $w.quitbuttons]
    button $qb.quit -text Done -command "destroy $w"
    pack $qb
    pack $qb.quit

    update_log $w
}

proc update_setup {w} {
    global setupselect
    global pgain igain dgain vff aff minoutput maxoutput posbias negbias deadband

    $w.p.ent config -textvariable pgain($setupselect)
    $w.i.ent config -textvariable igain($setupselect)
    $w.d.ent config -textvariable dgain($setupselect)
    $w.vff.ent config -textvariable vff($setupselect)
    $w.aff.ent config -textvariable aff($setupselect)
    $w.minoutput.ent config -textvariable minoutput($setupselect)
    $w.maxoutput.ent config -textvariable maxoutput($setupselect)
    $w.posbias.ent config -textvariable posbias($setupselect)
    $w.negbias.ent config -textvariable negbias($setupselect)
    $w.deadband.ent config -textvariable deadband($setupselect)
}

proc write_setup {} {
    global setupselect
    global pgain igain dgain vff aff minoutput maxoutput posbias negbias deadband

    gotk_servo_set_pid $setupselect $pgain($setupselect) $igain($setupselect) $dgain($setupselect) $vff($setupselect) $aff($setupselect) $minoutput($setupselect) $maxoutput($setupselect) $posbias($setupselect) $negbias($setupselect) $deadband($setupselect)
}

proc popup_setup {w} {
    global whichselect setupselect
    global pgain igain dgain vff aff minoutput maxoutput posbias negbias deadband

    if {[winfo exists $w]} {
	wm deiconify $w
	raise $w
	focus $w
	return
    }

    toplevel $w
    wm title $w "Go Motion Setup"

    set setupselect $whichselect

    foreach item {1 2 3 4 5 6 7 8} {
	set str [gotk_servo_get_pid $item]
	set pgain($item) [lindex $str 0]
	set igain($item) [lindex $str 1]
	set dgain($item) [lindex $str 2]
	set vff($item) [lindex $str 3]
	set aff($item) [lindex $str 4]
	set minoutput($item) [lindex $str 5]
	set maxoutput($item) [lindex $str 6]
	set posbias($item) [lindex $str 7]
	set negbias($item) [lindex $str 8]
	set deadband($item) [lindex $str 9]
    }

    foreach row {1 2 3 4} {
	frame $w.row$row
	label $w.row$row.namel -text $row -width 2 -relief ridge -justify center
	radiobutton $w.row$row.rbl -variable setupselect -value $row -command "update_setup $w"
	label $w.row$row.namer -text [expr $row + 4] -width 2 -relief ridge -justify center
	radiobutton $w.row$row.rbr -variable setupselect -value [expr $row + 4] -command "update_setup $w"
	pack $w.row$row -side top
	pack $w.row$row.namel $w.row$row.rbl $w.row$row.namer $w.row$row.rbr -side left
    }

    frame $w.p
    label $w.p.lab -text "P Gain:" -anchor w
    entry $w.p.ent -width 10 -textvariable pgain($setupselect)
    pack $w.p -side top -fill x
    pack $w.p.lab -side left
    pack $w.p.ent -side right

    frame $w.i
    label $w.i.lab -text "I Gain:" -anchor w
    entry $w.i.ent -width 10 -textvariable igain($setupselect)
    pack $w.i -side top -fill x
    pack $w.i.lab -side left
    pack $w.i.ent -side right

    frame $w.d
    label $w.d.lab -text "D Gain:" -anchor w
    entry $w.d.ent -width 10 -textvariable dgain($setupselect)
    pack $w.d -side top -fill x
    pack $w.d.lab -side left
    pack $w.d.ent -side right

    frame $w.vff
    label $w.vff.lab -text Vff: -anchor w
    entry $w.vff.ent -width 10 -textvariable vff($setupselect)
    pack $w.vff -side top -fill x
    pack $w.vff.lab -side left
    pack $w.vff.ent -side right

    frame $w.aff
    label $w.aff.lab -text Aff: -anchor w
    entry $w.aff.ent -width 10 -textvariable aff($setupselect)
    pack $w.aff -side top -fill x
    pack $w.aff.lab -side left
    pack $w.aff.ent -side right

    frame $w.minoutput
    label $w.minoutput.lab -text "Min Output:" -anchor w
    entry $w.minoutput.ent -width 10 -textvariable minoutput($setupselect)
    pack $w.minoutput -side top -fill x
    pack $w.minoutput.lab -side left
    pack $w.minoutput.ent -side right

    frame $w.maxoutput
    label $w.maxoutput.lab -text "Max Output:" -anchor w
    entry $w.maxoutput.ent -width 10 -textvariable maxoutput($setupselect)
    pack $w.maxoutput -side top -fill x
    pack $w.maxoutput.lab -side left
    pack $w.maxoutput.ent -side right

    frame $w.posbias
    label $w.posbias.lab -text "Pos Bias:" -anchor w
    entry $w.posbias.ent -width 10 -textvariable posbias($setupselect)
    pack $w.posbias -side top -fill x
    pack $w.posbias.lab -side left
    pack $w.posbias.ent -side right

    frame $w.negbias
    label $w.negbias.lab -text "Neg Bias:" -anchor w
    entry $w.negbias.ent -width 10 -textvariable negbias($setupselect)
    pack $w.negbias -side top -fill x
    pack $w.negbias.lab -side left
    pack $w.negbias.ent -side right

    frame $w.deadband
    label $w.deadband.lab -text Deadband: -anchor w
    entry $w.deadband.ent -width 10 -textvariable deadband($setupselect)
    pack $w.deadband -side top -fill x
    pack $w.deadband.lab -side left
    pack $w.deadband.ent -side right

    set wb [frame $w.writebuttons]
    button $wb.write -text Write -command write_setup
    pack $wb
    pack $wb.write

    set qb [frame $w.quitbuttons]
    button $qb.quit -text Done -command "destroy $w"
    pack $qb
    pack $qb.quit
}

proc update_io {w} {
    global ain_data

    set num_ain [gotk_io_num_ain]
    for {set i 1} {$i <= $num_ain} {incr i 1} {
	set ain_data($i) [format %.3f [gotk_io_ain $i]]
    }

    if {[winfo exists $w]} {
	after 100 "update_io $w"
    }
}

proc popup_io {w} {
    global ain_data

    if {[winfo exists $w]} {
	wm deiconify $w
	raise $w
	focus $w
	return
    }

    toplevel $w
    wm title $w "Go Motion IO"

    set num_ain [gotk_io_num_ain]
    for {set i 1} {$i <= $num_ain} {incr i 1} {
	set f [frame $w.ain$i]
	label $f.lab -text $i -anchor w -width 3
	label $f.val -textvariable ain_data($i) -anchor w -width 10
	pack $f -side top
	pack $f.lab $f.val -side left
    }

    set qb [frame $w.donebuttons]
    button $qb.done -text Done -command "destroy $w"
    pack $qb
    pack $qb.done

    update_io $w
}

set scriptdir [gotk_ini PROG_DIR TASK $inifile]

# 
# Script handling has some differences between Windows and Unix. In
# Windows, a Tcl script can't be run directly as an executable file,
# since Windows doesn't understand the header line. Tcl files must be
# run as arguments to gotk in an 'exec' here. Since that will work in
# Unix, that method is used for both.
# 
# The task controller can handle NC code files directly, and spawns
# other programs if not. If it's running, the program is sent there.
# 
# If the task controller is not running, then a last-ditch 'exec' is done
# here.
# 

proc popup_script {} {
    global thisdir inifile scriptdir

    if {[gotk_have_task]} {
	set types {
	    {"All files" *}
	    {"Tcl files" {.tcl .TCL}}
	    {"NC files" {.nc .NC .ngc .NGC}}
	}
    } else {
        set types {
	    {"Tcl files" {.tcl .TCL}}
	}
    }

    set f [tk_getOpenFile -filetypes $types -initialdir $thisdir/../$scriptdir]

    if {$f != ""} {
	set ext [file extension $f]
	if {$ext == ".tcl" || $ext == ".TCL"} {
	    set ulapi [gotk_ulapi]
	    if {$ulapi == ""} {
		exec $thisdir/gotk $f -- -i $inifile &
	    } else {
		exec $thisdir/gotk $f -- -i $inifile -u $ulapi &
	    }
	} else {
	    if {[gotk_have_task]} {
		gotk_task start $f
	    } else {
		exec $f
	    }
	}
    }
}

#
# 'whichselect' is a number tied to the radio buttons that show
# joint- or Cartesian position. The associated 'joint_select'
# and 'cart_select' variables here and in gotk are updated in
# 'do_whichselect'.
#
proc do_whichselect {} {
    global csys_select joint_select cart_select whichselect
    global cart_letter
    global jogspeed maxtvel maxrvel maxvel 
    global spl

    # update gotk's notion of the selected joint
    gotk_set_joint_select $whichselect

    if {$csys_select == "joint"} {
	# update the speed
	set maxspeed $maxvel($whichselect)
    } else {
	# update gotk's notion of the selected Cartesian axis
	set cl $cart_letter($whichselect)
	gotk_set_cart_select $cl
	if {$cl == "X" || $cl == "Y" || $cl == "Z"} {
	    set maxspeed $maxtvel
	} else {
	    set maxspeed $maxrvel
	}
    }

    set scaletop [lindex [$spl.scale config -to] 4]
    set ratio [expr $jogspeed / $scaletop]
    set res [expr 0.01 * $maxspeed]
    set newspeed [expr $ratio * $maxspeed]

    $spl.scale config -to $maxspeed -resolution $res
    set jogspeed $newspeed
}

proc do_jog {dir} {
    global csys_select cart_select joint_select
    global jogspeed

    if {$csys_select == "world"} {
	gotk_traj_jog_world $cart_select [expr $dir * $jogspeed]
    } else {
	if {$csys_select == "tool"} {
	    gotk_traj_jog_tool $cart_select [expr $dir * $jogspeed]
	} else {
	    gotk_traj_jog_joint $joint_select [expr $dir * $jogspeed]
	}
    }
}

proc write_move {} {
    global csys_select
    global move1 move2 move3 move4 move5 move6 movetime

    gotk_traj_set_move_time $movetime

    if {$csys_select == "world"} {
	gotk_traj_move_world $move1 $move2 $move3 $move4 $move5 $move6
    } else {
	if {$csys_select == "tool"} {
	    gotk_traj_move_tool $move1 $move2 $move3 $move4 $move5 $move6
	}
    }
}

proc popup_move {} {
    global csys_select cart_letter

    set w .movewindow

    if {[winfo exists $w]} {
	wm deiconify $w
	raise $w
	focus $w
	return
    }

    toplevel $w
    wm title $w "Go Motion Move"

    foreach item {1 2 3 4 5 6 7 8} {
	global move$item pos$item
	# set the initial entry values to be the actual position
	if {$csys_select == "world"} {
	    set move$item [set pos$item]
	} else {
	    set move$item 0
	}
	frame $w.$item
	set n $cart_letter($item) ; append n :
	label $w.$item.lab -text $n -anchor w
	entry $w.$item.ent -width 10 -textvariable move$item
	pack $w.$item -side top -fill x
	pack $w.$item.lab -side left
	pack $w.$item.ent -side right
    }

    # now do time entry
    global movetime
    set movetime 0
    frame $w.time
    label $w.time.lab -text Time: -anchor w
    entry $w.time.ent -width 10 -textvariable movetime
    pack $w.time -side top -fill x
    pack $w.time.lab -side left
    pack $w.time.ent -side right

    set wb [frame $w.writebuttons]
    button $wb.write -text Write -command write_move
    pack $wb
    pack $wb.write

    set qb [frame $w.quitbuttons]
    button $qb.quit -text Done -command "destroy $w"
    pack $qb
    pack $qb.quit
}

proc write_here {} {
    global here1 here2 here3 here4 here5 here6

    gotk_traj_here $here1 $here2 $here3 $here4 $here5 $here6
}

proc popup_here {} {
    global cart_letter

    set w .herewindow

    if {[winfo exists $w]} {
	wm deiconify $w
	raise $w
	focus $w
	return
    }

    toplevel $w
    wm title $w "Set Current Position"

    foreach item {1 2 3 4 5 6 7 8} {
	global here$item pos$item
	# set the initial entry values to be the actual position
	set here$item [set pos$item]
	frame $w.$item
	set n $cart_letter($item) ; append n :
	label $w.$item.lab -text $n -anchor w
	entry $w.$item.ent -width 10 -textvariable here$item
	pack $w.$item -side top -fill x
	pack $w.$item.lab -side left
	pack $w.$item.ent -side right
    }

    set wb [frame $w.writebuttons]
    button $wb.write -text Write -command write_here
    pack $wb
    pack $wb.write

    set qb [frame $w.quitbuttons]
    button $qb.quit -text Done -command "destroy $w"
    pack $qb
    pack $qb.quit
}

proc do_cfgscale {s} {
    global maxscalev maxscalea

    gotk_traj_cfg_scale [expr $s * 0.01] $maxscalev $maxscalea
}

proc do_home {w} {
    gotk_traj_home $w
}

proc do_stop {} {
    if {[gotk_have_task]} {
	gotk_task stop
    } else {
	gotk_traj_stop
    }
}

proc do_init {} {
    gotk_traj_init
}

if {$::tcl_platform(platform) == "windows"} then {
    set plotter {|pgnuplot}
} else {
    set plotter {|gnuplot}
}

proc plot_log {filename} {
    global plotter

    set tclfile out.tcl
    # 'which' will be in range 1..N
    set which [gotk_log_which]

    file delete -force $tclfile

    if {[catch {set gps [open $plotter r+]}]} {
	puts stdout "can't open $plotter"
	return
    }

    puts $gps "set term tk"
    puts $gps "set output '$tclfile'"
    if {[gotk_log_type] == "ActPos" ||
	[gotk_log_type] == "CmdPos" ||
	[gotk_log_type] == "Xinv"} {
	puts $gps "plot '$filename' using 1:2 title 'X', '' using 1:3 title 'Y', '' using 1:4 title 'Z'"
    } else {
	if {[gotk_log_type] == "MagXinv"} {
	    puts $gps "splot '$filename' using 2:3:4 title 'MaxXinv'"
	} else {
	    puts $gps "plot '$filename' using 1:2 title 'joint $which'"
	}
    }
    close $gps

    # wait until $tclfile is written
    set gotit 0
    for {set i 0} {$i < 10} {incr i} {
	if {! [catch {source $tclfile}]} {
	    set gotit 1
	    break
	}
	after 100
    }
    if {! $gotit} {
	puts stdout "can't read plot file $tclfile"
	return
    }

    # it was read ok, now put it in a window
    toplevel .plotwindow
    bind .plotwindow <Return> {destroy .plotwindow}
    wm title .plotwindow $filename
    canvas .plotwindow.c
    pack .plotwindow.c
    gnuplot .plotwindow.c
}

proc name_joint {} {
    global jogplusbutton jogminusbutton
    global jointbutton worldbutton toolbutton

    gotk_set_csys_select joint

    foreach item {1 2 3 4 5 6 7 8} {
	global name$item
	set name$item $item
    }

    # change the button enable states
    do_input_control

    $jointbutton config -underline 0
    $worldbutton config -underline -1
    $toolbutton config -underline -1

    do_whichselect
}

proc name_world {} {
    global cart_letter
    global jogplusbutton jogminusbutton
    global jointbutton worldbutton toolbutton

    gotk_set_csys_select world

    foreach item {1 2 3 4 5 6 7 8} {
	global name$item
	set name$item $cart_letter($item)
    }

    # change the button enable states
    do_input_control

    $jointbutton config -underline -1
    $worldbutton config -underline 0
    $toolbutton config -underline -1

    do_whichselect
}

proc name_tool {} {
    global cart_letter
    global jogplusbutton jogminusbutton
    global jointbutton worldbutton toolbutton

    gotk_set_csys_select tool

    foreach item {1 2 3 4 5 6 7 8} {
	global name$item
	set name$item $cart_letter($item)
    }

    # change the button enable states
    do_input_control

    $jointbutton config -underline -1
    $worldbutton config -underline -1
    $toolbutton config -underline 0

    do_whichselect
}

proc name_act {} {
    global actbutton cmdbutton

    $actbutton config -underline 0
    $cmdbutton config -underline -1
}

proc name_cmd {} {
    global actbutton cmdbutton

    $actbutton config -underline -1
    $cmdbutton config -underline 0
}

proc color_numbers {} {
    global csys_select

    if {$csys_select == "world" || $csys_select == "tool"} {
	set homed [gotk_world_homed]
	foreach item {1 2 3 4 5 6 7 8} {
	    global number$item
	    if {$homed} {
		[set number$item] config -foreground black
	    } else {
		[set number$item] config -foreground red
	    }
	}
    } else {
	foreach item {1 2 3 4 5 6 7 8} {
	    global number$item
	    if {[gotk_joint_active $item]} {
		if {[gotk_joint_homed $item]} {
		    [set number$item] config -foreground black
		} else {
		    [set number$item] config -foreground red
		}
	    } else {
		[set number$item] config -foreground gray
	    }
	}    
    }
}

set old_csys_select $csys_select
set old_input_control $input_control

set normalbackground [$quitbutton cget -background]

proc color_reset {} {
    global resetbutton normalbackground
    if {[gotk_traj_admin_state] == "Uninitialized"} {
	[set resetbutton] config -background red
    } else {
	[set resetbutton] config -background $normalbackground
    }
}

proc update_status {} {
    global csys_select old_csys_select
    global joint_select cart_select actref
    global whichselect cart_letter
    global input_control old_input_control

    gotk_update

    set csys_select [gotk_get_csys_select]
    set joint_select [gotk_get_joint_select]
    set cart_select [gotk_get_cart_select]
    set input_control [gotk_get_input_control]

    set csys_changed 0
    if {$old_csys_select != $csys_select} {set csys_changed 1}
    set old_csys_select $csys_select

    if {$old_input_control != $input_control} {do_input_control}
    set old_input_control $input_control

    # set the radio button in case gotk changed it
    set whichselect $joint_select

    if {$csys_select == "joint"} {
	if {$csys_changed} name_joint
	foreach item {1 2 3 4 5 6 7 8} {
	    global pos$item
	    if {$actref == 1} {
		set pos$item [format %f [gotk_joint_pos $item]]
	    } else {
		set pos$item [format %f [gotk_joint_cmd_pos $item]]
	    }
	}
    } else {
	if {$csys_select == "world"} {
	    if {$csys_changed} name_world
	    foreach item {1 2 3 4 5 6 7 8} {
		global pos$item
		if {$actref == 1} {
		    set pos$item [format %f [gotk_world_pos $cart_letter($item)]]
		} else {
		    set pos$item [format %f [gotk_world_cmd_pos $cart_letter($item)]]
		}
	    }
	} else {
	    if {$csys_changed} name_tool
	    foreach item {1 2 3 4 5 6 7 8} {
		global pos$item
		if {$actref == 1} {
		    # FIXME-- need to add gotk_tool_{cmd}pos
		    set pos$item [format %f [gotk_world_pos $cart_letter($item)]]
		} else {
		    set pos$item [format %f [gotk_world_cmd_pos $cart_letter($item)]]
		}
	    }
	}
    }

    color_numbers
    color_reset
    
    after 200 update_status
}

# send an init if necessary and wait until done
if {! [string equal [gotk_traj_admin_state] "Initialized"]} {
    gotk_set_timeout forever
    gotk_traj_init
    gotk_traj_hold
}

# set up no timeout, since we're interactive and the user will
# decide when to start and stop commands
gotk_set_timeout none

bind . <KeyPress-q> {exit}

name_joint
name_act

# insert balloon help
set_balloon $jointbutton "Display and move individual joints"
set_balloon $worldbutton "Display and move Cartesian world axes"
set_balloon $toolbutton "Display and move Cartesian tool axes"
set_balloon $actbutton "Display actual position values"
set_balloon $cmdbutton "Display target position values"
set_balloon $homebutton "Home the selected joint"
set_balloon $herebutton "Set the current Cartesian position"

update_status

