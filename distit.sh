#!/bin/sh

CVSDIR=`cat CVS/Root`

cd /tmp && rm -rf gomotion && cvs -d $CVSDIR co gomotion && cd gomotion && ./autoconf.sh && make dist && tar xzvf *.tar.gz && cd gomotion-* && ./configure && make && make dist || exit 1

exit 0

