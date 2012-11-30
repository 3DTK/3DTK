****************************************************************

  txt2las:

  Converts LIDAR data from a standard ASCII format into the more
  efficient binary LAS/LAZ/BIN representations.

  Reads also directy from *.gz, *.zip, *.rar, and *.7z files if
  the corresponding gzip.exe, unzip.exe, unrar.exe, and 7z.exe
  are in the same folder.

  Allows adding a VLR to the header with projection information.
 
  For updates check the website or join the LAStools mailing list.

  http://lastools.org/
  http://groups.google.com/group/lastools/
  http://twitter.com/lastools/
  http://facebook.com/lastools/
  http://linkedin.com/groups?gid=4408378

  Martin @lastools

****************************************************************

example usage:

>> txt2las -i lidar.txt.gz -o lidar.las -parse ssxyz

converts a gzipped ASCII file and uses the 3rd, 4th, and 5th entry
of each line as the x, y, and z coordinate of each point

>> txt2las -i lidar.zip -o lidar.laz -parse ssxyz -utm 14T

same as above for a zipped ASCII file but produces compressed LAZ
and adds projection info for utm zone with wgs84 

>> txt2las -i lidar.txt -o lidar.laz -parse xyzai -scale_scan_angle 57.3 -scale_intensity 65535

also reads the 4th entry as the scan angle and multiplies it by 57.3
(radian to angle) and the 5th entry as the intensity and multiplies
it by 65535 (converts range [0.0 .. 1.0] to range [0 .. 65535]. then
produces a compressed LAZ file.

>> txt2las -skip 3 -i lidar.txt.gz -o lidar.las -parse txyzsa -sp83 OH_N

converts a gzipped ASCII file and uses the 1st entry of each line
as the gps time, the 3rd, 4th, and 5th entry as the x, y, and z
coordinate of each point, and the 6th entry as the scan angle. it
skips the first three lines of the ASCII data file and adds projection
info for state plane ohio north with nad83. 

>> txt2las -i lidar.txt.gz -o lidar.laz -parse xyzRGB -set_scale 0.001 0.001 0.001 -set_offset 500000 4000000 0

converts a gzipped ASCII file and uses the 1st 2nd, and 3rd entry
of each line as the x, y, and z coordinate of each point, and the
4th, 5th, and 6th entry as the RGB color. the created compressed
LAZ file will have a precision of 0.001 0.001 0.001 and an offset
of 500000 4000000 0

for more info:

C:\lastools\bin>txt2las -h
Supported LAS Outputs
  -o lidar.las
  -o lidar.laz
  -o xyzta.txt -oparse xyzta (on-the-fly to ASCII)
  -olas -olaz -otxt (specify format)
  -stdout (pipe to stdout)
  -nil    (pipe to NULL)
LAStools (by martin.isenburg@gmail.com) version 110521
Supported ASCII Inputs:
  -i lidar.txt
  -i lidar.txt.gz
  -i lidar.zip
  -i lidar.rar
  -i lidar.7z
  -stdin (pipe from stdin)
usage:
txt2las -parse tsxyz -i lidar.txt.gz
txt2las -parse xyzairn -i lidar.zip -utm 17T -olaz -quiet
unzip -p lidar.zip | txt2las -parse xyz -stdin -o lidar.las -longlat -elevation_survey_feet
txt2las -i lidar.zip -parse txyzar -scale_scan_angle 57.3 -o lidar.laz
txt2las -skip 5 -parse xyz -i lidar.rar -set_file_creation 28 2011 -o lidar.las
txt2las -parse xyzsst -verbose -set_scale 0.001 0.001 0.001 -i lidar.txt
txt2las -parse xsysz -set_scale 0.1 0.1 0.01 -i lidar.txt.gz -sp83 OH_N -feet
las2las -parse tsxyzRGB -i lidar.txt -set_version 1.2 -scale_intensity 65535 -o lidar.las
txt2las -h
---------------------------------------------
The '-parse tsxyz' flag specifies how to interpret
each line of the ASCII file. For example, 'tsxyzssa'
means that the first number is the gpstime, the next
number should be skipped, the next three numbers are
the x, y, and z coordinate, the next two should be
skipped, and the next number is the scan angle.
The other supported entries are i - intensity,
n - number of returns of given pulse, r - number
of return, c - classification, u - user data, and
p - point source ID, e - edge of flight line flag, and
d - direction of scan flag, R - red channel of RGB
color, G - green channel, B - blue channel
---------------------------------------------
Other parameters are
'-set_scale 0.05 0.05 0.001'
'-set_offset 500000 2000000 0'
'-set_file_creation 67 2011'
'-set_system_identifier "Riegl 500,000 Hz"'
'-set_generating_software "LAStools"'
'-utm 14T'
'-sp83 CA_I -feet -elevation_survey_feet'
'-longlat -elevation_feet'

---------------

if you find bugs let me (martin.isenburg@gmail.com) know.
