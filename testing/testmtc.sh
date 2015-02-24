#!/bin/sh

cd `dirname $0`

cleanup () {
    killall -KILL go_adapter 2> /dev/null
    killall -KILL taskloop 2> /dev/null
    killall -KILL gotk 2> /dev/null
}

cleanup

trap cleanup INT

../bin/gorun -i ../etc/genhex1.ini &
sleep 3

../bin/taskloop -i ../etc/genhex1.ini &
sleep 1

../bin/go_adapter -i ../etc/genhex1.ini -p 7878 &

../bin/gorun -i ../etc/genhex2.ini &
sleep 3

../bin/taskloop -i ../etc/genhex2.ini &
sleep 1

../bin/go_adapter -i ../etc/genhex2.ini -p 7879 &

# do the 'wait' if you don't want to run 'gosh' as the last process
# wait $!

../bin/gosh -i ../etc/genhex1.ini

# now in some other shell, cd into the testing/ directory, and run
# some script, e.g., ../scripts/task.tcl -- -i ../etc/genhex1.ini

cleanup

exit 0
