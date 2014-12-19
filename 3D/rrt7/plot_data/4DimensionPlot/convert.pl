#!/usr/bin/perl
use strict;
use warnings;

## usage: convert.pl "input" "output"

open(FILE,$ARGV[0]);
my @memory=<FILE>;
close(FILE);

my (%x,%y,%z,%s);
foreach my $line (@memory)
{
  if($line=~m/^-*\d/)
  {
    chomp($line);
    my @tmp=split(/\s+/,$line);
    $x{$tmp[0]}=1;
    $y{$tmp[1]}=1;
    $z{$tmp[2]}=1;
    $s{$tmp[0]."_".$tmp[1]."_".$tmp[2]}=$tmp[3];
  }
}

open(NEWFILE,">$ARGV[1]");
foreach my $z (sort{$a<=>$b}(keys(%z)))
{
  foreach my $y (sort{$a<=>$b}(keys(%y)))
  {
    foreach my $x (sort{$a<=>$b}(keys(%x)))
    {
      print NEWFILE $x,"\t",$y,"\t",$z,"\t",$s{$x."_".$y."_".$z},"\n";
    }
    print NEWFILE "\n";
  }
  print NEWFILE "\n";
}

print STDERR "Gnuplot command: ","splot for [j=0:",scalar(keys(%z))*scalar(keys(%y))-1,":1] \'",$ARGV[1],"\' every :::(j*",scalar(keys(%y)),"+0)::(j*",scalar(keys(%y)),"+",scalar(keys(%y))-1,") with pm3d not","\n";
