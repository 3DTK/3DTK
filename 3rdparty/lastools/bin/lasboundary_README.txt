****************************************************************

  lasboundary:

  reads LIDAR from LAS/LAZ/ASCII format and computes a boundary
  polygon for the points. By default this is a *concave hull*
  of the points which is - by default - always a single polygon
  where "islands of points" are connected by edges that are
  traversed in each direction once.

  Optionally a *disjoint hull* is computed with the '-disjoint'
  flag. This can lead to multiple hulls in case of islands. Note
  that tiny islands of the size of one or two LIDAR points that
  are too small to form a triangle and are "lost".

  The tool can also compute *interior* holes in the data via
  the '-holes' flag. It not only finds holes but also islands in
  the holes.

  The controlling value is '-concavity 100' that can be specified
  in the command line. The default is 50, meaning that voids with
  distances of moew than 50 units are considered the exterior (or
  part of an interior hole).

  lasboundary can directly output in KML format for easy viewing
  in GE. In case there is no projection information in the LAS
  file it can be specified in the command line with '-utm 15T' or
  '-sp83 OH_N' or similar.

  Finally, the tool can also compute a standard *convex hull* with
  the '-convex' flag.

  The algorithm has recently been redesigned to make very efficient
  use of main memory. It now scales to much much larger LAS/LAZ/ASCII
  inputs than it was previously possible. For comparison, you can
  still run the older version of the algorithm that was limited to
  30 million points with the '-use_old' flag.
                 
  Please license from martin.isenburg@gmail.com to use lasboundary
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

>> lasboundary -i *.las -oshp

computes the boundaries of all LAS file '*.las' individually and stores 
the result to ESRI's Shapefiles '*.shp'.

>> lasboundary -i *.las -merged -o merged.shp

computes the boundaries of the merged points from all LAS file '*.las'
and stores the result to the ESRI Shapefile 'merged.shp'.

>> lasboundary -i lidar1.las lidar2.las -merged -o lidar_boundary.shp

computes the boundary of the LAS file created from merging 'lidar1.las'
and 'lidar2.las' and stores the result to 'lidar_boundary.shp'.

>> lasboundary -i lidar1.las lidar2.las -otxt

the same but without merging and storing the results to ASCII files.

>> lasboundary -i lidar1.las lidar2.las -oshp -concavity 100

the same but with creating less detailed concavities. the default
value for concavities is 50 (meaning edges along the convex hull
that are shorter than 50 units get "pushed" inwards)

>> lasboundary -i lidar.las -o lidar_boundary.kml -utm 10T -disjoint

computes a disjoint hull instead of a concave hull and uses a utm
projeciton 10T to store the boundary in geo-referenced KML format

>> lasboundary -i lidar.las -o lidar_holes.kml -disjoint -holes

same as before but assumes geo-referencing is in the KML file. it
also computes holes in the interior of the boundary.

C:\lastools\bin>lasboundary -h
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
  -drop_abs_scan_angle_above 15
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
  -clamp_z_below 70.5
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
  -set_classification 2
  -change_classification_from_to 2 4
Modify the user data.
  -set_user_data 0
  -change_user_data_from_to 23 26
Modify the point source ID.
  -set_point_source 500
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
Supported Line Outputs
  -o lines.shp
  -o boundary.wkt
  -o contours.kml
  -o output.txt
  -odir C:\footprints (specify output directory)
  -odix _isolines (specify file name appendix)
  -ocut 2 (cut the last two characters from name)
  -oshp -owkt -okml -otxt
  -stdout (pipe to stdout)
Optional Settings
  -only_2d
  -kml_absolute
  -kml_elevation_offset 17.5
LAStools (by martin.isenburg@gmail.com) version 120403 (unlicensed)
usage:
lasboundary -i *.las -merged -o merged.shp
lasboundary -i *.laz -owkt -concavity 100 (default is 50)
lasboundary -i flight???.las -oshp -disjoint
lasboundary -i USACE_Merrick_lots_of_VLRs.las -o USACE.kml
lasboundary -i Serpent.las -o boundary.kml -disjoint -holes
lasboundary -i *.laz -merged -o merged.kml -disjoint -utm 17S
lasboundary -i lidar.las -o boundary.kml -lonlat -concavity 0.00002
lasboundary -i *.txt -iparse ssxyz -otxt -first_only
lasboundary -i lidar.las -keep_class 6 -o boundary.shp
lasboundary -h

other possible transformations for KML generation:
-sp83 IA_N
-sp27 IA_S

---------------

if you find bugs let me (martin.isenburg@gmail.com) know.
