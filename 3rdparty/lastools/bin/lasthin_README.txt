****************************************************************

  lasthin:

  A simple LIDAR thinning algorithm for LAS/LAZ/ASCII. It places
  a uniform grid over the points and within each grid cell keeps
  only the point with the lowest (or '-highest' or '-random') Z
  coordinate. When keeping '-random' points you can in addition
  specify a '-seed 232' for the random generator.

  Instead of removing the thinned out points from the output file
  you can also mark them as '-withheld'. The you can use the 
  standard '-drop_withheld' or '-keep_withheld' filters to get
  either the thinned points or their complement.

  In order to process very large but sparse grids such as, for
  example, a single but very long diagonal flight line, it is
  beneficial to use the '-sparse' option to avoid exceeding the
  main memory (and start thrashing).

  Optionally you can only consider last returns ('-last_only')
  or a particular classification ('-keep_class 2') while doing
  the thinning. There are many other filters to choose from.
  Simply run 'lasthin -h' to lists them all.

  The grid spacing default is 1 unit and it can be changed, for
  example to 5 units, with '-step 5'.
 
  Please license from martin.isenburg@gmail.com to use lasthin
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

>> lasthin -i *.las

thins all LAS files with the grid spacing default of 1 unit
and keeps the lowest point per grid cell

>> lasthin -i *.laz -olaz

same but with LAZ files

>> lasthin -i *.txt -iparse xyzt -otxt -oparse xyzt

same but with ASCII files

>> lasthin -i in.las -o out.las

does LAS thinning with the grid spacing default of 1 unit
and keeps the lowest point per grid cell

>> lasthin -i in.las -o out.las -highest -step 2.0

does point thinning with a grid spacing of 2.0 units and
keeps the highest point per grid cell

>> lasthin -i in.laz -step 0.5 -o out.laz -random

does point thinning with a grid spacing of 0.5 units and
keeps a random point per grid cell

>> lasthin -i in.laz -step 0.5 -o out.laz -sparse

does point thinning with a grid spacing of 0.5 units using
a sparse grid representation

>> lasthin -i in.laz -o out.laz -first_only

does LIDAR thinning with a grid spacing of 1 unit but keeps
the highest points while considering only first returns

>> lasthin -i in.las -o out.las -keep_class 2 -keep_class 3

looks only at the points classfied as 2 or 3 from in.las and
thins them with a grid spacing of 1 unit 

>> lasthin -i file_list.txt -lof -o combined.laz

looks at all the points from all the LAS or LAZ files listed
in the text file 'file_list.txt', thins them with a grid
spacing of 1 unit and outputs them compressed to 'combined.laz'

for more info:

C:\lastools\bin>lasthin -h
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
  -single_returns_only
  -double_returns_only
  -triple_returns_only
  -quadruple_returns_only
  -quintuple_returns_only
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
  -i lidar.txt -iparse xyzti (on-the-fly from ASCII)
  -i lidar.txt -iparse xyzi -itranslate_intensity 1024
  -lof file_list.txt
  -stdin (pipe from stdin)
  -rescale 0.1 0.1 0.1
  -reoffset 600000 4000000 0
Supported LAS Outputs
  -o lidar.las
  -o lidar.laz
  -o xyzta.txt -oparse xyzta (on-the-fly to ASCII)
  -olas -olaz -otxt (specify format)
  -stdout (pipe to stdout)
  -nil    (pipe to NULL)
usage:
lasthin -i *.las
lasthin -i *.laz -olaz
lasthin -i in.las -o out.las
lasthin -i in.las -step 0.5 -o out.las -highest
lasthin -i in.las -last_only -step 0.5 -sparse -o out.las
lasthin -i in.las -keep_class 2 3 4 -step 2 -o out.las
lasthin -h

---------------

if you find bugs let me (martin.isenburg@gmail.com) know.
