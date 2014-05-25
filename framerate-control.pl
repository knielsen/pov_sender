#! /usr/bin/perl

use strict;
use warnings;

use Time::HiRes;
use IO::Stty;

my $FRAMERATE= 25;

$| = 1;
binmode STDOUT;

IO::Stty::stty(\*STDOUT, 'raw');

my $len = int( (int((65*65*3+1)/2) + 30) / 31 ) * 32;
my $inter_frame_delay= 1.0/$FRAMERATE;

my $init_time = [Time::HiRes::gettimeofday()];
my $tot_frames = 0;

for (;;) {
  my $old_time= [Time::HiRes::gettimeofday()];
  my $sofar= 0;
  my $buf;
  while ($sofar < $len) {
    $buf= '';
    my $res= sysread(STDIN, $buf, $len - $sofar);
    if (!defined($res)) {
      die "Error reading: $!\n";
    } elsif (!$res) {
      if (seek(STDIN, 0, 0)) {
        # File ended, loop back to the start.
        next;
      }
      print "EOF on input, exiting\n";
      exit 0;
    }
    print $buf;
    $sofar+= $res;
  }
  ++$tot_frames;

  my $cur_time= [Time::HiRes::gettimeofday()];
  my $elapsed= Time::HiRes::tv_interval($old_time, $cur_time);
  if ($elapsed < $inter_frame_delay) {
    Time::HiRes::sleep($inter_frame_delay - $elapsed);
  }
  my $tot_elapsed = Time::HiRes::tv_interval($init_time, [Time::HiRes::gettimeofday()]);
  if ($tot_frames < 10 ||
      ($tot_frames < 100 && !($tot_frames % 10)) ||
      !($tot_frames % 100)) {
    printf STDERR "Frames: $tot_frames  rate: %7.3f per second.\n", $tot_frames/$tot_elapsed;
  }
}
