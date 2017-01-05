#!/bin/sh
# run as "scripts/godseye.tcl -i etc/genhexh.ini -u rtai"
# the next line restarts using gotcl \
exec /usr/local/bin/gotcl "$0" -- "$@"

# set a convenient alias
interp alias {} move {} gotk_traj_move_world

# we need to set timeout to forever, so we'll queue every move
gotk_set_timeout forever

# make moves go their fastest
gotk_traj_set_move_time 0

move 40 25 30 0 0 0

move 33 18 30 0 0 0
move 47 18 30 0 0 0
move 47 32 30 0 0 0
move 33 32 30 0 0 0
move 33 18 30 0 0 0

move 34 19 30 0 0 0
move 46 19 30 0 0 0
move 46 31 30 0 0 0
move 34 31 30 0 0 0
move 34 19 30 0 0 0

move 35 20 30 0 0 0
move 45 20 30 0 0 0
move 45 30 30 0 0 0
move 35 30 30 0 0 0
move 35 20 30 0 0 0

move 36 21 30 0 0 0
move 44 21 30 0 0 0
move 44 29 30 0 0 0
move 36 29 30 0 0 0
move 36 21 30 0 0 0

move 37 22 30 0 0 0
move 43 22 30 0 0 0
move 43 28 30 0 0 0
move 37 28 30 0 0 0
move 37 22 30 0 0 0

move 40 25 30 0 0 0

exit 0

