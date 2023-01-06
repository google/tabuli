#!/usr/bin/env perl
use strict;
use warnings;

use List::Util qw(all min max);
use MIDI;

all {defined} my ($input_midi, $output_template) = @ARGV, or do {
	print STDERR "Usage: $0 <input midi> <output template>\n\nExample: $0 input.midi notes/note-%03d.midi\n";
	exit 1;
};

my $opus = MIDI::Opus->new({from_file => $input_midi});

my @notes = map {$$_[4]} grep {$$_[0] eq 'note'} map {@{MIDI::Score::events_r_to_score_r($_->events_r)}} $opus->tracks;
my $lower = min @notes;
my $upper = max @notes;

for my $note ($lower .. $upper) {
	my $filtered_opus = $opus->copy;

	for my $track ($filtered_opus->tracks) {
		my $score = MIDI::Score::events_r_to_score_r($track->events_r);
		my @filtered_score = grep {$$_[0] ne 'note' || $$_[4] == $note} @$score;
		$track->events_r(MIDI::Score::score_r_to_events_r(\@filtered_score));
	}

	$filtered_opus->write_to_file(sprintf($output_template, 1 + $note - $lower));
}
