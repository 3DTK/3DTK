****************************************************************

  las2iso:

  reads LIDAR in LAS/LAZ/ASCII format and extracts isocontours by
  constructing and interpolating a temporary TIN. It is possible
  to triangulate only certain points such as only first returns
  (-first_only) or only last returns (-last_only). One can also
  only triangulate points that have a particular classification.
  For example, the option '-keep_class 2 3' will triangulate only
  the points of classification 2 or 3.

  The resulting isocontours are stored either in ESRI's Shapefile
  format (-o contours.shp) or as a text file (-o contours.txt) 
  or in Google Earth's KML format for immediate visualization
  (-o contours.kml). For the latter georeferencing information
  is needed. If it is not provided in the LAS header then it can
  be specified in the command line (-utm 14T or -sp83 OH_S).

  Optionally the contours can also be simplified, cleaned, and
  smoothed before output.
 
  Isolines along the convex hull of the point cloud where triangles
  are very large are suppressed with the '-concavity 10' option.
  This will recursively remove all triangles with an edge length
  of 10 units that are adjacent to the boundary. The default is
  a concavity of 50. Use '-concavity 0' to disable this.
 
  Closed breaklines can be supplied for hydro-enforcment of
  lakes, for example ('-lakes lakes.shp', '-lakes hydro.txt')
  but they must form proper closed polygons and have elevations.

  Hard breaklines can be integrated for improving the TIN before
  it is sampled with ('-creeks roads.shp', '-creeks creeks.txt')
  and while they can be open they must also have elevations.

  Please license from martin.isenburg@gmail.com to use las2iso
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

>> las2iso -i *.las -oshp -iso_every 2

extracts 2 unit contours from all files that match *.las and stores
the result in ESRI's Shapefile format.

>> las2iso -i *.txt -iparse ssxyz -oshp -iso_number 20

extracts 20 evenly spaced contours from all ASCII files *.txt that
are parsed with "ssxyz" and stores the result in Shapefile format.

>> las2iso -i lidar.las -o contours.shp -iso_every 5 -simplify 1 -clean 10
 
extracts 5 unit contours, simplifies away all segments that are
shorter than 1 unit, cleans out all contours whose total length
is shorter than 10 units, and then stores the result in ESRI's
Shapefile format.

>> las2iso -i lidar.las -o contours.kml -iso_number 20 -utm 10T

extracts 20 evenly spaced contours and stores the result in KML
format using UTM 10T as projection information.

>> las2iso -i flight1*.laz flight2*.laz -oshp -iso_every 2

extracts 2 unit contours from all LAZ files that match flight1*.laz
or flight2*.laz and stores the result in ESRI's Shapefile format.

>> las2iso -i lidar.txt -iparse xyz -o contours.kml -iso_number 50 -sp83 IA_N

extracts 50 evenly spaced contours and stores the result in KML
format using state plane83 Iowa North as projection information.

>> las2iso -i lidar.las -o contours.shp -iso_every 10 -simplify 0.5

extracts evenly spaced contours every 10 units, simplifies away all
segments that are shorter than 0.5 units, and stores the result in
ESRI's Shapefile format.

>> las2iso -i lidar1.las lidar2.las -merged -o contours.wkt -iso_range 400 800 5 

merges LAS files lidar1.las lidar2.las, extracts 5 units contours
from 400 to 800, and stores the result in Well Known Text format.

>> las2iso -i lidar.laz -o contours.shp -last_only -clean 100

extracts 10 contours using only the last returns from the LAZ file, 
cleans out all contours whose total length is shorter than 100 units,
and stores the result in ESRI's Shapefile format.

>> las2iso -i data*.las -merged -o merged.shp -iso_every 5 -keep_class 2 3

merges all LAS files data*.las into one and extracts contours every
5 units using only points that are classified as 2 or 3 and stores
the result in ESRI's Shapefile format.

for more info:

C:\lastools\bin>las2iso -h
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
Supported Line Outputs
  -o lines.shp
  -o boundary.wkt
  -o contours.kml
  -o output.txt
  -oshp -owkt -okml -otxt
  -stdout (pipe to stdout)
Optional Settings
  -only_2d
  -kml_absolute
  -kml_elevation_offset 17.5
usage:
las2iso -i *.las -oshp
las2iso -i flight1*.las flight2*.las -oshp -simplify 1 -clean 10
las2iso -i *.las -okml -iso_range 400 600 20 -utm 14S
las2iso -i *.txt -iparse ssxyz -owkt -iso_every 2 -simplify 0.5 -kill 100
las2iso -i lidar.las -first_only -o contours.wkt -iso_number 20
las2iso -i lidar.las -o contours.shp -last_only -iso_range 400 600 20
las2iso -i lidar.las -otxt -stdout -keep_class 2 3 9 > lines.txt
las2iso -h

other possible transformations for KML generation:
-utm 11T
-sp83 IA_N
-sp27 IA_S
-longlat
-latlong

---------------

if you find bugs let me (martin.isenburg@gmail.com) know.
