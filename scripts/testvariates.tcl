#!/bin/sh
# the next line restarts using gotcl \
exec /usr/local/proj/gomotion/bin/gotcl "$0" -- "$@"

if {false} {
    set r [weibull init 3 1]
    for {set t 0} {$t < 1000} {set t [expr $t + 1]} {
	puts [set val [weibull get $r]]
    }
}

if {false} {
    set r [uniform init 1 3]
    for {set t 0} {$t < 1000} {set t [expr $t + 1]} {
	puts [set val [uniform get $r]]
    }
}

if {false} {
    set r [pearson_v init 1.5 1]
    for {set t 0} {$t < 1000} {set t [expr $t + 1]} {
	puts [set val [pearson_v get $r]]
    }
}

if {true} {
    set r [gamma init 1.5 1]
    for {set t 0} {$t < 1000} {set t [expr $t + 1]} {
	puts [set val [gamma get $r]]
    }
}

exit 0

