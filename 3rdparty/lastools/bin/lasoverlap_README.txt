****************************************************************

  lasoverlap:

  This tool reads LIDAR points from LAS/LAZ/ASCII/BIN/SHP and
  checks the flight line overlap and / or the vertical and
  horizontal alignment.

  The most important parameter '-step n' specifies the n x n
  area that of LiDAR points that are gridded on one raster
  (or pixel) that are then used for doing the overlap and/or
  difference calculations. The output is either in BIL, ASC,
  IMG, TIF, PNG, JPG, XYZ,or DTM format. When computing the
  difference in the flight line overlaps, by default the tool
  will check '-elevation_lowest', but '-intensity_highest', or
  '-density' are possible too.

  Optionally, a KML file is generated that allows the resulting
  raster  to be immediately displayed inside a geospatial context
  provided by Google Earth (for TIF/PNG/JPG images). In case the
  LAS/LAZ file contains projection information (i.e. geo keys
  as variable length records) this metadata is used to correctly
  geo-reference the KML file. It is also possible to provide the
  proper geo-referencing information in the command-line.

  Use '-subsample n' with 1 < n < 5 to anti-alias the gridding of
  LiDAR points by their x and y coordinate into disjunct rasters.
  The option '-subsample 3' adds each LiDAR point 9 times to the
  raster at locations (x/y), (x+0.33*step/y), (x+0.66*step/y),
  (x/y+0.33*step) (x+0.33*step/y+0.33*step) (x+0.66*step/y+0.33*step),
  (x/y+0.66*step) (x+0.33*step/y+0.66*step) (x+0.66*step/y+0.66*step)
  and thereby "washes out" hard boundaries.

  Please license from martin.isenburg@gmail.com to use lasoverlap
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

>> lasoverlap -i tile.las -step 2

creates an overlap raster as well as a difference raster with a
step of 2 units. For this, it is necessary that the LiDAR points
in the LAS file have their point source ID populated with the
flight line ID. The overlap raster uses the default color ramp
which maps overlap counts from 0 to 5 to a different colors. The
difference raster uses the default color ramp that maps blue to
-2.5, white to 0, and red to 2.5. The default output is PNG.

>> lasoverlap -i tile.las -step 2 -values -oasc

same as above but output the actual values into an ASC raster
(instead of the color-coded values into a PNG).

>> lasoverlap -i LDR*.las -files_are_flightlines -step 3 -max_diff 2

merges all the files "LDR*.las" while assigning the points of each
file a unique point source ID (aka flight line number), and then
creates an overlap raster as well as a difference raster with a step
of 3 units. The overlap raster uses the default range of 5 overlaps
lines for the color ramp. The difference raster uses '-max_diff 2'
and maps range (-2 ... 0 ... 2) to colors (blue ... white ... red).

>> lasoverlap -i LDR*.las -files_are_flightlines -step 2 -max_diff 4 -no_over

same as above but does not output an overlap raster ('-no_over') and
operates with different '-step 2' and '-max_diff 4' settings.

>> lasoverlap -i LDR*.las -files_are_flightlines -step 5 -no_diff

same as above but does not output a difference raster ('-no_diff') but
only a coarse (-step 5) overlap raster.

>> lasoverlap -i tiles*.laz -step 2 -no_over -utm 15N

operates tile by tile (e.g. creates one difference raster per file)
and generates a Google Earth KML file for each tile using UTM 15N. It
is necessary that the LiDAR points in the LAZ file have their point
source ID populated.

>> lasoverlap -i tiles*.laz -step 2 -no_over -utm 15N -intensity -highest

same as above but computes intensity differences. For intensities the
difference raster maps range -255 ... 0 ... 255 (e.g. '-max_diff 255').

>> lasoverlap -i tiles*.laz -step 2 -no_over -utm 15N -intensity -highest

same as above but computes density differences. For densities the
difference raster maps range -100 ... 0 ... 100 (e.g. '-max_diff 100').

for more info:

C:\lastools\bin>lasoverlap -h
Please license from 'martin.isenburg@gmail.com' to use LAStools commercially.
Filter points based on their coordinates.
  -clip_tile 631000 4834000 1000 (ll_x ll_y size)
  -clip_circle 630250.00 4834750.00 100 (x y radius)
  -clip_box 620000 4830000 100 621000 4831000 200 (min_x min_y min_z max_x max_y max_z)
  -clip 630000 4834000 631000 4836000 (min_x min_y max_x max_y)
  -clip_x_below 630000.50 (min_x)
  -clip_y_below 4834500.25 (min_y)
  -clip_x_above 630500.50 (max_x)
  -clip_y_above 4836000.75 (max_y)
  -clip_z 11.125 130.725 (min_z, max_z)
  -clip_z_below 11.125 (min_z)
  -clip_z_above 130.725 (max_z)
Filter points based on their return number.
  -first_only -keep_first -drop_first
  -last_only -keep_last -drop_last
  -keep_middle -drop_middle
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
  -drop_synthetic -keep_synthetic
  -drop_keypoint -keep_keypoint
  -drop_withheld -keep_withheld
Filter points based on their user data.
  -keep_user_data 1
  -drop_user_data 255
  -keep_user_data_between 10 20
  -drop_user_data_below 1
  -drop_user_data_above 100
  -drop_user_data_between 10 40
Filter points based on their point source ID.
  -keep_point_source 3
  -keep_point_source_between 2 6
  -drop_point_source 27
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
Filter points based on their wavepacket.
  -keep_wavepacket 1 2
  -drop_wavepacket 0
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
Change the return number or return count of points.
  -repair_zero_returns
  -change_return_number_from_to 2 1
  -change_number_of_returns_from_to 0 2
Modify the classification.
  -set_classification_to 2
  -change_classification_from_to 2 4
Modify the point source ID.
  -set_point_source_to 500
  -change_point_source_from_to 1023 1024
Transform gps_time.
  -translate_gps_time 40.50
  -adjusted_to_week
  -week_to_adjusted 1671
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
  -i esri.shp
  -i nasa.qi
  -i lidar.txt -iparse xyzti -iskip 2 (on-the-fly from ASCII)
  -i lidar.txt -iparse xyzi -itranslate_intensity 1024
  -lof file_list.txt
  -stdin (pipe from stdin)
  -rescale 0.1 0.1 0.1
  -reoffset 600000 4000000 0
Supported Raster Outputs
  -o dem.asc
  -o dsm.bil
  -o dtm.dtm
  -o map.xyz
  -o dem.img
  -o hillshade.png
  -o dem.tif
  -o false_color.jpg
  -oasc -obil -oimg -opng -odtm -otif -ojpg -oxyz -nil
LAStools (by martin.isenburg@gmail.com) version 120214 (unlicensed)
usage:
lasoverlap -i tile.laz -step 2 -max_diff 4.0
lasoverlap -i tile.las -step 3 -values
lasoverlap -i *.laz -step 4 -max_diff 1.5
lasoverlap -i tiles*.las -step 2
lasoverlap -i tile1.las tile2.las tile3.las tile4.las -merged -step 2
lasoverlap -i swath1.las swath2.las swath3.las -files_are_flightlines -step 2
lasoverlap -i tiles*.las -merged -density
lasoverlap -i tiles*.las -files_are_flightlines -intensity -highest
lasoverlap -h

---------------

if you find bugs let me (martin.isenburg@gmail.com) know
