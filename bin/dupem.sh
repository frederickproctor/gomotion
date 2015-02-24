#!/bin/sh

thisdir=`dirname $0`
cd $thisdir

./godup.pl 1 < ../etc/gomotion.ini > ../etc/gomotion1.ini

./godup.pl 2 < ../etc/gomotion.ini > ../etc/gomotion2.ini

./godup.pl 3 < ../etc/gomotion.ini > ../etc/gomotion3.ini

./godup.pl 4 < ../etc/gomotion.ini > ../etc/gomotion4.ini

./godup.pl 5 < ../etc/gomotion.ini > ../etc/gomotion5.ini

./godup.pl 6 < ../etc/gomotion.ini > ../etc/gomotion6.ini

./godup.pl 7 < ../etc/gomotion.ini > ../etc/gomotion7.ini

./godup.pl 8 < ../etc/gomotion.ini > ../etc/gomotion8.ini

./godup.pl 9 < ../etc/gomotion.ini > ../etc/gomotion9.ini

exit 0


