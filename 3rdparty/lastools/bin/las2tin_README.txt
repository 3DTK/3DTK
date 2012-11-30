****************************************************************

  las2tin:

  reads LIDAR data in LAS/LAZ/ASCII format and creates a TIN. It
  is possible to triangulate only certain points like only first
  returns (-first_only) or only last returns (-last_only). One
  can also only triangulate only points that have a particular
  classification. The option (-keep_class 2 3) will triangulate
  only the points of classification 2 or 3.

  The output of large TIN triangles along the convex hull of the
  point cloud can be suppressed with the '-concavity 10' option.
  This will recursively remove all triangles with an edge length
  of 10 units that are adjacent to the boundary. The default is
  a concavity of 50. Use '-concavity 0' to disable this.
 
  Closed breaklines can be supplied for hydro-enforcment of
  lakes, for example ('-lakes lakes.shp', '-lakes hydro.txt')
  but they must form proper closed polygons and have elevations.

  Hard breaklines can be integrated for improving the TIN before
  it is sampled with ('-creeks roads.shp', '-creeks creeks.txt')
  and while they can be open they must also have elevations.

  Please license from martin.isenburg@gmail.com to use las2tin
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

>> las2tin -i *.las

triangulates all files individually and stores the resulting
TINs in ESRI's Shapefile format in files with the same name but
different endings.

>> las2tin -i *.las -concavity 25

same as above but dropping large triangles along the boundary ...

>> las2tin -i lidar.las -o tin.shp

triangulates all points and stores the resulting TIN in ESRI's Shapefile
format.

>> las2tin -i lidar.las -o tin.obj

triangulates all points and stores the resulting TIN in OBJ format.

>> las2tin -i lidar.laz -o triangles.txt -last_only

triangulates all last return points from lidar.laz and stores them
as a list of triangles in ASCII format in triangles.txt.

>> las2tin -i lidar.laz -o tin.shp -keep_class 2 3

triangulates all points classfied as 2 or 3 from lidar.laz and stores
the resulting TIN in ESRI's Shapefile format to file tin.shp.

>> las2tin -i lidar1.las lidar2.laz lidar3.laz lidar4.las -merged -o tin.shp

triangulates the combined points from the four LAS and LAZ files and
stores the resulting TIN in ESRI's Shapefile format to file tin.shp.

for more info:

C:\lastools\bin>las2tin -h
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
Change the return number or return count of points.
  -repair_zero_returns
  -change_return_number_from_to 2 1
  -change_number_of_returns_from_to 0 2
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
LAStools (by martin.isenburg@gmail.com) version 111006 (unlicensed)
usage:
las2tin -i *.las
las2tin -i *.las -concavity 25
las2tin -i lidar.las -o tin.shp
las2tin -i lidar.laz -first_only -o mesh.obj
las2tin -i lidar.laz -last_only -o triangles.txt
las2tin -i lidar.las -last_only -keep_class 2 6 8 -o tin.shp
las2tin -i lidar.laz -keep_class 8 -oobj
las2tin -i lidar.las -keep_class 8 -otxt
las2tin -h

---------------

if you find bugs let me (martin.isenburg@gmail.com) know.
