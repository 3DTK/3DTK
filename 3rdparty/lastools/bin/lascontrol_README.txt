****************************************************************

  lascontrol:

  This tool computes the height of the LIDAR at certain x and y
  control points locations and reports the height difference in
  respect to the control points' elevation.

  The tool reads LIDAR in LAS/LAZ/ASCII format, triangulates the
  relevant points into a TIN. For classified data sets containing
  a mix of ground and vegetation/building points it is imperative
  to specified the points against which the reference measurements
  are to be taken (i.e. usually '-keep_class 2').

  If the horizontal units are in feet this should be specified in
  the command-line to make sure that a wide enough patch of LiDAR
  points is used around each control point (i.e. '-feet'). If the
  LIDAR file contains proper projection information this can be
  automatically detected by the tool.
  
  The output defaults to stdout if no output file is specified.

  Please license from martin.isenburg@gmail.com to use lascontrol
  commercially.

  For updates check the website or join the LAStools mailing list.

  http://lastools.org/
  http://groups.google.com/group/lastools/
  http://twitter.com/lastools/
  http://facebook.com/lastools/
  http://linkedin.com/groups?gid=4408378

  Martin @lastools

****************************************************************

example usage:

>> lascontrol -i *.las -cp cp.csv -keep_class 2 8

assumes the x/y/z coordinates of the control points are stored as
the 1nd/2nd/3rd entry on each line of the 'cp.csv' file and only
points with ground and mass-point classification (class 2 or 8)
are used to construct the reference TIN. all LAS files that match
'*.las' get merged on the fly into one. output goes to stdout.

>> lascontrol -i lidar.las -cp cp.csv -keep_class 2 -parse sxyz

assumes the x/y/z coordinates of the control points are stored as
the 2nd/3rd/4th entry on each line of the 'cp.csv' file and only
points with ground classification (class = 2) are used to construct
the reference TIN. output goes to stdout.

>> lascontrol -i lidar.laz -cp cp.csv -keep_class 2 -parse xsysz -cp_out cp_out.csv

assumes the x/y/z coordinates of the control points are stored as
the 1st/3rd/5th entry on each line of the 'cp.csv' file and only
points with ground classification (class = 2) are used to construct
the reference TIN. output goes to 'cp_out.csv'.

>> lascontrol -i *.laz -cp cp.txt -skip 3 -keep_class 2

skips the first three lines of the file 'cp.txt' and assumes the
x/y/z coordinates of the control points are stored as 1st/2nd/3rd
entry on each line. all LAS files that match '*.laz' get merged
on the fly into one. output goes to stdout.

>> lascontrol -i flight1*.txt -iparse sxyz -iskip 2 -cp cp.txt -parse sssxyz -skip 1  -cp_out cp_out.csv

skips the first lines of the file 'cp.txt' and assumes the x/y/z
coordinates of the control points are stored as the 4th/5th/6th
entry on each line. the LIDAR points get merged from all ASCII
files that match the wildcard 'flight1*.txt'. the first 2 lines
of each ASCII file are skipped and the x/y/z coordinates of the
LIDAR points are taked from the nd/3rd/4th entry on each line and
the output goes to 'cp_out.csv'.

for more info:

C:\lastools\bin>lascontrol -h
Please license from 'martin.isenburg@gmail.com' to use LAStools commercially.
Filter points based on their coordinates.
  -clip_tile 631000 4834000 1000 (ll_x, ll_y, size)
  -clip_circle 630250.00 4834750.00 100 (x, y, radius)
  -clip 630000 4834000 631000 4836000 (min_x, min_y, max_x, max_y)
  -clip_x_below 630000.50 (min_x)
  -clip_y_below 4834500.25 (min_y)
  -clip_x_above 630500.50 (max_x)
  -clip_y_above 4836000.75 (max_y)
  -clip_z 11.125 130.725 (min_z, max_z)
  -clip_z_below 11.125 (min_z)
  -clip_z_above 130.725 (max_z)
Filter points based on their return number.
  -first_only
  -last_only
  -keep_return 1 2 3
  -drop_return 3 4
  -keep_single -drop_single
  -keep_double -drop_double
  -keep_triple -drop_triple
  -keep_quadruple -drop_quadruple
  -keep_quintuple -drop_quintuple
Filter points based on the scanline flags.
  -drop_scan_direction 0
  -scan_direction_change_only
  -edge_of_flight_line_only
Filter points based on their intensity.
  -keep_intensity 20 380
  -drop_intensity_below 20
  -drop_intensity_above 380
  -drop_intensity_between 4000 5000
Filter points based on their classification.
  -keep_class 1 3 7
  -drop_class 4 2
Filter points based on their point source ID.
  -keep_point_source 3
  -keep_point_source_between 2 6
  -drop_point_source_below 6
  -drop_point_source_above 15
  -drop_point_source_between 17 21
Filter points based on their scan angle.
  -keep_scan_angle -15 15
  -drop_scan_angle_below -15
  -drop_scan_angle_above 15
  -drop_scan_angle_between -25 -23
Filter points based on their gps time.
  -keep_gps_time 11.125 130.725
  -drop_gps_time_below 11.125
  -drop_gps_time_above 130.725
  -drop_gps_time_between 22.0 48.0
Filter points with simple thinning.
  -keep_every_nth 2
  -keep_random_fraction 0.1
  -thin_with_grid 1.0
Transform coordinates.
  -translate_x -2.5
  -scale_z 0.3048
  -rotate_xy 15.0 620000 4100000 (angle + origin)
  -translate_xyz 0.5 0.5 0
  -translate_then_scale_y -0.5 1.001
  -clamp_z_min 70.5
  -clamp_z 70.5 72.5
Transform raw xyz integers.
  -translate_raw_z 20
  -translate_raw_xyz 1 1 0
  -clamp_raw_z 500 800
Transform intensity.
  -scale_intensity 2.5
  -translate_intensity 50
  -translate_then_scale_intensity 0.5 3.1
Transform scan_angle.
  -scale_scan_angle 1.944445
  -translate_scan_angle -5
  -translate_then_scale_scan_angle -0.5 2.1
Repair points with return number or pulse count of zero.
  -repair_zero_returns
Change classification by replacing one with another.
  -change_classification_from_to 2 4
Transform gps_time.
  -translate_gps_time 40.50
Transform RGB colors.
  -scale_rgb_down (by 256)
  -scale_rgb_up (by 256)
Supported LAS Inputs
  -i lidar.las
  -i lidar.laz
  -i lidar1.las lidar2.las lidar3.las -merged
  -i *.las
  -i flight0??.laz flight1??.laz -single
  -i lidar.txt -iparse xyzti -iskip 2 (on-the-fly from ASCII)
  -i lidar.txt -iparse xyzi -itranslate_intensity 1024
  -lof file_list.txt
  -stdin (pipe from stdin)
  -rescale 0.1 0.1 0.1
  -reoffset 600000 4000000 0
LAStools (by martin.isenburg@gmail.com) version 110822 (unlicensed)
usage:
lascontrol -i lidar.las -cp cp.txt -keep_class 2
lascontrol -i lidar.las -cp cp.csv -keep_class 2 -parse sxyz
lascontrol -i lidar.laz -cp cp.csv -keep_class 2 -parse xsysz -cp_out cp_out.csv
lascontrol -i *.las -cp cp.txt -skip 3 -keep_class 2
lascontrol -i lidar.txt -iparse cssxyz -cp cp.txt -parse ssxyz -skip 3 -keep_class 2
lascontrol -i flight1*.txt -iparse sxyz -iskip 2 -cp cp.txt -parse sssxyz -skip 1
lascontrol -h

---------------

if you find bugs let me (martin.isenburg@gmail.com) know.
