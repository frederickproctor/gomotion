#!/bin/sh
# syntax: move.tcl {-- -i <inifile> -u unix | rtai}
# e.g., bin/move.tcl -- -i etc/genhex.ini -u unix
# the next line restarts using gotk \
exec `dirname $0`/gotk "$0" "$@"

proc prompt {str} {
    return [tk_dialog .dialog Prompt "$str" "" 0 OK]
}

wm title . Move

gotk_traj_home 1
gotk_traj_home 2
gotk_traj_home 3
gotk_traj_home 4
gotk_traj_home 5
gotk_traj_home 6

gotk_traj_move_joint 300 300 300 300 300 300
gotk_traj_move_world 1 2 -120 0 0 0
gotk_traj_move_joint 300 300 300 300 300 300
gotk_traj_move_world 1 2 -120 0 0 0
gotk_traj_move_joint 300 300 300 300 300 300
gotk_traj_move_world 1 2 -120 0 0 0
gotk_traj_move_joint 300 300 300 300 300 300
gotk_traj_move_world 1 2 -120 0 0 0
gotk_traj_move_joint 300 300 300 300 300 300
gotk_traj_move_world 1 2 -120 0 0 0
gotk_traj_move_joint 300 300 300 300 300 300
gotk_traj_move_world 1 2 -120 0 0 0
gotk_traj_move_joint 300 300 300 300 300 300
gotk_traj_move_world 1 2 -120 0 0 0

exit
