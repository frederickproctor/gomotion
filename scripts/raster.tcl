#!/bin/sh
# run as "scripts/raster.tcl -- -i etc/genhexh.ini -u rtai"
# the next line restarts using gotcl \
exec /usr/local/proj/gomotion/bin/gotcl "$0" -- "$@"

puts "inifile is [gotk_inifile]"

# set a convenient alias
interp alias {} move {} gotk_traj_move_world

# we need to set timeout to forever, so we'll queue every move
gotk_set_timeout forever

# make moves go their fastest
gotk_traj_set_move_time 0

set x [gotk_world_pos X]
set y [gotk_world_pos Y]
set z [gotk_world_pos Z]
set r [gotk_world_pos R]
set p [gotk_world_pos P]
set w [gotk_world_pos W]

set startx $x
set minx [expr $x - 2]
set maxx [expr $x + 2]

set starty $y
set miny [expr $y - 2]
set maxy [expr $y + 2]

while {true} {
    for {set y $miny} {$y <= $maxy} {set y [expr $y + 1]} {
	gotk_tool_on 3500
	move $minx $y $z $r $p $w
	gotk_tool_on 2500
	move $maxx $y $z $r $p $w
    }
    move $startx $starty $z $r $p $w
    gotk_tool_off
}

exit 0

