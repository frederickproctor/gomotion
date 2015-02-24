#!/usr/bin/perl

# Reads a Go Motion .ini file from stdin, takes the first argument as a
# number to add to the keys, and writes the revised file to stdout.
# For example, 
# ./godup.pl 1 < ../etc/gomotion.ini > ../etc/gomotion1.ini

$incval = 1;
if ($#ARGV >= 0) {
    $incval = $ARGV[0]
}

sub incit {
    if ($_[0] =~ /0x\w+/) {
	return sprintf("%#x", hex($_[0]) + $_[1]);
    } else {
	return $_[0] + $_[1];
    }
}

while (<STDIN>) {
# matches <anything>_KEY = <pattern 1>
    if (/_KEY\s*=\s*(.*)/) {
# increments pattern 1
	$a = $1; # save $1 since it's written at every pattern compare
	$b = incit($a,$incval);
# substitutes pattern 1 with its increment
	s/$a/$b/;
    } elsif (/_PORT\s*=\s*(.*)/) {
# ditto for <anything>_PORT
	$a = $1;
	$b = incit($a,$incval);
	s/$a/$b/;
    } elsif (/\s*NAME\s*=\s*\S*\s*(\S*)/) {
# ditto for NAME
	$a = $1;
	$b = incit($a,$incval);
	s/$a/$b/;
    }
    print $_;
}

exit 0;

