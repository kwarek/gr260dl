# Installation: #
  1. Extract content
  1. go to directory in terminal and run 'make install'

Find your GPS device:
--> device should be named something like '/dev/ttyUSBxx'; check in the /dev folder for the file 'ttyUSBxx' that has been created after the device has been plugged in
and run all following commands accordingly.

# Usage: #
  1. run 'gr260dl' in terminal within the directory of the program
  1. run 'sudo gr260dl -i /dev/ttyUSB0 -v -g out.gpx' in terminal
  1. run 'perl splitgpx.pl -i out.gpx' to split the output into the single tracks
  1. Use the obtained single track `*`.gpx files (located in program folder) as you wish!