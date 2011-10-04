#!/usr/bin/perl
#
#
use strict;
use warnings;
use Getopt::Long;

my ( $inputfile, $format, $help ) = '';

my $gpx_header =
  "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\" ?>
<gpx xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" version=\"1.1\" xmlns:gpxtpx=\"http://www.garmin.com/xmlschemas/TrackPointExtension/v1\" xmlns:gpxdata=\"http://www.cluetrust.com/XML/GPXDATA/1/0\" xmlns=\"http://www.topografix.com/GPX/1/1\" xmlns:rmc=\"urn:net:trekbuddy:1.0:nmea:rmc\" creator=\"QLandkarteGT 1.2.3 http://www.qlandkarte.org/\" xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd http://www.garmin.com/xmlschemas/GpxExtensions/v3 http://www.garmin.com/xmlschemas/GpxExtensionsv3.xsd http://www.garmin.com/xmlschemas/TrackPointExtension/v1 http://www.garmin.com/xmlschemas/TrackPointExtensionv1.xsd\" xmlns:gpxx=\"http://www.garmin.com/xmlschemas/GpxExtensions/v3\">";

GetOptions(
    'input_file|i=s' => \$inputfile,
    'format|f'       => \$format,
    'help|h'         => \$help
);

sub usage {
    print "\nUSAGE: splitgpx.pl -i INPUTFILE\n";
    print
"OPTIONS: -f converts heart rate data to a format usable for pytrainer\n";
    print "         -h prints this help\n\n";
    exit;
}

usage if $help;
usage unless ($inputfile);

my $gpx = slurp_inputfile($inputfile);

if ($format) {    # make gpx format readable for pytrainer
    $gpx =~ s/<[\/]*gpxtpx:TrackPointExtension>\n//igs;
    $gpx =~ s/gpxtpx:hr/gpxdata:hr/igs;
}

$gpx =~ s/<wpt.+?<\/gpx>//igs;    #remove waypoints

my @tracks = split /<trk>/, $gpx;
shift @tracks;

for my $track (@tracks) {
    my $trackname;
    
    if ($track =~ /<name>(.+?)<\/name>/){
    $trackname = $1;
    }
    else{
        die "Cannot determine track number\nEnding..\n";
    }
    
    my $filename  = $trackname . ".gpx";

    open OUT, ">$filename";
    print OUT $gpx_header;
    print OUT "<metadata>\n  <name>"
      . $trackname
      . "</name>\n</metadata>\n<trk>\n";
    print OUT "$track";
    print OUT "</gpx>\n";
    close OUT;
}

sub slurp_inputfile {
    my ($input) = @_;
    local $/ = undef;
    open FILE, $input or die "Couldn't open file: $!";
    my $file = <FILE>;
    close FILE;
    return $file;
}
