****************************************************************

  las2las14:

  upconverts LiDAR data from LAS/LAZ/ASCII to the LAS 1.4 format
 
  for updates check the website or join the LAStools mailing list

  http://lastools.org/
  http://groups.google.com/group/lastools/
  http://groups.google.com/group/lasroom/
  http://twitter.com/lastools/
  http://facebook.com/lastools/
  http://linkedin.com/groups?gid=4408378

  Martin @lastools

****************************************************************

example usage:

las2las14 -i ..\data\TO_core_last_zoom.laz -o to_14.las
las2las14 -i ..\data\TO_core_last_zoom.laz -o to_14_6.las -six
las2las14 -i ..\data\TO_core_last_zoom.laz -o to_14_7.las -seven
las2las14 -i ..\data\TO_core_last_zoom.laz -o to_14_8.las -eight
las2las14 -i ..\data\TO_core_last_zoom.laz -o to_14_9.las -nine
las2las14 -i ..\data\TO_core_last_zoom.laz -o to_14_A.las -ten

the flags '-six' to '-ten' indicate that the old point types 0 to 5 of the input should
be upconverted to the new point types 6 to 10. then the LAS 1.4 file will no longer
be compatible with earlier versions of LAS (i.e. readable by older, forward compatible
LAS readers).

examples:

C:\lastools\bin>las2las14  -i ..\data\TO_core_last_zoom.laz -o to_14.las
writing 213093 points of type 1 to LAS 1.4 took 0.515 secs.

C:\lastools\bin>lasinfo to_14.las
WARNING: unknown version 1.4 (should be 1.0 or 1.1 or 1.2 or 1.3)
reporting all LAS header entries:
  file signature:            'LASF'
  file source ID:            0
  reserved (global_encoding):0
  project ID GUID data 1-4:  0 0 0 ''
  version major.minor:       1.4
  system identifier:         'LAStools (c) by Martin Isenburg'
  generating software:       'LAS 1.4 example (v111207)'
  file creation day/year:    0/0
  header size                375
  offset to point data       377
  number var. length records 0
  point data format          1
  point data record length   28
  number of point records    213093
  number of points by return 128621 84472 0 0 0
  scale factor x y z         0.01 0.01 0.01
  offset x y z               0 0 0
  min x y z                  630250.00 4834500.00 46.83
  max x y z                  630500.00 4834750.00 170.65
  start_of_waveform_data_packet_record 0
the header contains 140 user-defined bytes
the header is followed by 2 user-defined bytes
reporting minimum and maximum for all LAS point record entries ...
  x 63025000 63050000
  y 483450000 483475000
  z 4683 17065
  intensity 10 50200
  edge_of_flight_line 0 0
  scan_direction_flag 0 0
  number_of_returns_of_given_pulse 1 2
  return_number 1 2
  classification 1 1
  scan_angle_rank 0 0
  user_data 2 4
  point_source_ID 0 0
  gps_time 413162.560400 414095.322000
number of last returns: 213093
covered area in square units/kilounits: 62552/0.06
point density: all returns 3.41 last only 3.41 (per square units)
overview over number of returns of given pulse: 128621 84472 0 0 0 0 0
histogram of classification of points:
   213093 Unclassified (1)


C:\lastools\bin>las2las14  -i ..\data\TO_core_last_zoom.laz -o to_14_6.las -six
writing 213093 points of type 6 to LAS 1.4 took 0.312 secs.

C:\lastools\bin>lasinfo to_14_6.las
WARNING: unknown version 1.4 (should be 1.0 or 1.1 or 1.2 or 1.3)
ERROR: unknown point type 6 with point size 30
ERROR: cannot open lasreaderlas with file name 'to_14_6.las'
ERROR: cannot open lasreader


C:\lastools\bin>las2las14  -i ..\data\TO_core_last_zoom.laz -o to_14_6.las -seven
writing 213093 points of type 7 to LAS 1.4 took 0.484 secs.

C:\lastools\bin>lasinfo to_14_7.las
WARNING: unknown version 1.4 (should be 1.0 or 1.1 or 1.2 or 1.3)
ERROR: unknown point type 7 with point size 36
ERROR: cannot open lasreaderlas with file name 'to_14_7.las'
ERROR: cannot open lasreader


C:\lastools\bin>las2las14  -i ..\data\TO_core_last_zoom.laz -o to_14_8.las -eight
writing 213093 points of type 8 to LAS 1.4 took 0.437 secs.

C:\lastools\bin>lasinfo -i to_14_8.las
WARNING: unknown version 1.4 (should be 1.0 or 1.1 or 1.2 or 1.3)
ERROR: unknown point type 8 with point size 38
ERROR: cannot open lasreaderlas with file name 'to_14_8.las'
ERROR: cannot open lasreader


C:\lastools\bin>dir to_14*
Directory of C:\lastools\bin
12/07/2011  01:11 PM         5,966,981 to_14.las
12/07/2011  01:13 PM         7,671,725 to_14_6.las
12/07/2011  12:11 PM         7,671,725 to_14_7.las
12/07/2011  01:13 PM         8,097,911 to_14_8.las

---------------

if you find bugs let me (martin.isenburg@gmail.com) know.
