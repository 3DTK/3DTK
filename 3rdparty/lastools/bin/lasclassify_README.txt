****************************************************************

  lasclassify:

  This tool classifies buildings and high vegetation (i.e. trees)
  in LAS/LAZ files. This tool requires that the bare-earth points
  have already been identified (e.g. with lasground) and that the
  elevation of each point above the ground was already computed
  with lasheight (which stores this height in the 'user data' field
  of each point).

  The tool essentially tries to find neighboring points that are
  at least 2 meter (or 6 feet) above the ground and form '-planar 0.1'
  (= roofs) or '-ragged 0.4' (= trees) regions. You can change the
  above the ground threshold with '-ground_offset 5'.

  If your data is very noisy the tool have trouble finding planar
  regions. Try playing with the '-planar 0.1' default. Often the
  flight lines are not properly aligned which will also destroy
  planarity. Here you may be better served to process your flight
  lines separately from another.

  It is also important to tell the tool whether the horizontal
  and vertical units are meters (which is assumed by default)
  or '-feet' or '-elevation_feet'. Should the LAS file contain
  projection information then there is no need to specify this
  explicitly. If the input coordinates are in an earth-centered
  or a longlat representation, the file needs converted to, for
  example, a UTM projection first.

  The experienced user can fine-tune the algorithm by specifing
  a threshold until which points are considered planar with
  '-planar 0.2'. This would roughly correspond to a standard
  deviation of up to 0.2 meters that neighboring points can have
  from the planar region they share. The default is 0.1 meters.

  Please license from martin.isenburg@gmail.com to use lasclassify
  commercially. Please note that the unlicensed version will set
  intensity, gps_time, user data, and point source ID to zero,
  slightly change the LAS point order, and randomly add a tiny
  bit of white noise to the points coordinates.

  For updates check the website or join the LAStools mailing list.

  http://lastools.org/
  http://groups.google.com/group/lastools/
  http://twitter.com/lastools/
  http://facebook.com/lastools/
  http://linkedin.com/groups?gid=4408378

  Martin @lastools

****************************************************************

example usage:

>> lasground -i lidar.las -o lidar_with_bare_earth.las -city
>> lasheight -i lidar_with_bare_earth.las -o lidar_with_heights.las
>> lasclassify -i lidar_with_heights.las -o lidar_classified.las

finds the ground points with lasground, computes the height of each
point with lasheight, and classifies buildings and high vegetation
with the default settings.

>> lasground -i lidar.las -o lidar_with_bare_earth.las -city -feet -elevation_feet
>> lasheight -i lidar_with_bare_earth.las -o lidar_with_heights.las
>> lasclassify -i lidar_with_heights.las -o lidar_classified.las -feet -elevation_feet

the same as above for LIDAR where both horizontal and vertical units
are in feet instead of in meters (meters are assumed by default unless
there is projection information in the LAS file saying otherwise).

>> lasclassify -i *.las

classifies all LAS files with the default settings (the LAS files need
to already have ground points classified and point heigths computed).

>> lasclassify -i *.laz

classifies all LAZ files with the default settings (the LAZ files need
to already have ground points classified and point heigths computed).

>> lasclassify -i *.laz -planar 0.2

experimental. same as above but more points will be joined into roofs.

for more info:

C:\lastools\bin>lasclassify -h

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
Change point source ID by replacing one with another.
  -change_point_source_from_to 1023 1024
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
  -i terrasolid.bin
  -i lidar.txt -iparse xyzti -iskip 2 (on-the-fly from ASCII)
  -i lidar.txt -iparse xyzi -itranslate_intensity 1024
  -lof file_list.txt
  -stdin (pipe from stdin)
  -rescale 0.1 0.1 0.1
  -reoffset 600000 4000000 0
Supported LAS Outputs
  -o lidar.las
  -o lidar.laz
  -o xyzta.txt -oparse xyzta (on-the-fly to ASCII)
  -o terrasolid.bin
  -olas -olaz -otxt -obin (specify format)
  -stdout (pipe to stdout)
  -nil    (pipe to NULL)
LAStools (by martin.isenburg@gmail.com) version 110914 (unlicensed)
usage:
lasclassify -i in.las -o out.laz
lasclassify -i in.laz -o out.las -foot -elevation_foot
lasclassify -i *.las -verbose
lasclassify -i *.laz -verbose -foot -elevation_foot
lasclassify -h

---------------

if you find bugs let me (martin.isenburg@gmail.com) know.
