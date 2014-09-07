#! /usr/bin/perl

use strict;
use warnings;

use IO::Stty;

my $tty;
my $mode;
my $dc_value;


if (@ARGV == 4 && $ARGV[0] eq '-t') {
  $tty = $ARGV[1];
  $dc_value = $ARGV[2];
  $mode = $ARGV[3];
} elsif (@ARGV == 2) {
  $tty = '/dev/ttyACM0';
  $dc_value = $ARGV[0];
  $mode = $ARGV[1];
} else {
  print STDERR "Usage: $0 [-t /dev/ttyACMx] dc_value mode\n";
  exit 1;
}

die "DC value must be between 0 and 63\n"
    unless $dc_value >= 0 && $dc_value <= 63;
die "Mode must be 0 (rectangular framebuffer) or 1 (polar/triangle)\n"
    unless $mode == 0 || $mode == 1;;

open TTY, "+<", $tty
    or die "Failed to open programmer on $tty: $!\n";

select TTY;
$| = 1;
select STDOUT;
$| = 1;
binmode TTY;
IO::Stty::stty(\*TTY, 'raw');


sub mk_packet {
  my $packet = '';
  while (@_) {
    $packet .= chr(shift);
  }
  while (length($packet) < 32) {
    $packet .= chr(0);
  }
  return $packet;
}


sub send_packet {
  my ($packet) = @_;

  print TTY $packet;
}


print "Sending config command to POV...";

send_packet(mk_packet(255, 1, $dc_value, $mode));
print " done!\n";
