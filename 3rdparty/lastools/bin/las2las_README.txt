****************************************************************

  las2las:

  reads and writes LIDAR data in LAS/LAZ/ASCII format to filter,
  transform, project, thin, or otherwise modify its contents.
  Examples are clipping of points to those that lie within a
  bounding box '-clip 10 10 20 20' or points that are between
  a certain height '-clip_z 10 100', or dropping points that
  are a certain return '-drop_return 2', that have a scan angle
  above some threshold '-drop_scan_angle_above 5', or below some
  intensity '-drop_intensity_below 15'. Sometimes points are far
  outside the bounding box (corrupted files) and it is handy to
  remove them with '-clip_to_bounding_box'.

  It is also possible to add missing projection information to
  the LAS/LAZ file or to reproject (using the same ellipsoid)
  for example from latitude/longitude to UTM or the stateplane
  of Ohio_North.

  Another typical use is extract only first (-first_only) or only
  last (-last_only) returns. Extracting the first return is the
  same as dropping all others (e.g. -drop_return 2 3 4 5).

  Or one can extract a subsequence of 1000 points (-subseq 540 1000)
  which will start at point 540.

  Finally one can also only keep or drop certain classifications.
  The option -keep_class 2 3 will keep only those points that are
  of classification 2 or 3 and the option -drop_class 2 3 will drop
  only those points. For all options run 'las2las -h'.
 
  For updates check the website or join the LAStools mailing list.

  http://lastools.org/
  http://groups.google.com/group/lastools/
  http://twitter.com/lastools/
  http://facebook.com/lastools/
  http://linkedin.com/groups?gid=4408378

  Martin @lastools

****************************************************************

example usage:

>> las2las -i s1885565.laz -o out.las -sp83 OH_S -feet -elevation_feet

Adding the projection information to the file 's1885565.laz' (*). This
will not modify the points but merely change the projection VLR in the
header to contain these four geokeys:

  GeoKeyDirectoryTag version 1.1.0 number of keys 4
  - key 1024 value_offset 1 - GTModelTypeGeoKey: ModelTypeProjected
  - key 3072 value_offset 32123 - ProjectedCSTypeGeoKey: PCS_NAD83_Ohio_South
  - key 3076 value_offset 9002 - ProjLinearUnitsGeoKey: Linear_Foot
  - key 4099 value_offset 9002 - VerticalUnitsGeoKey: Linear_Foot

(*) http://www.cs.unc.edu/~isenburg/lastools/download/test/s1885565.laz

>> las2las -i s1885565.laz -o out.las -sp83 OH_S -feet -elevation_feet -target_utm auto

Reprojects the points from the Ohio_South NAD83 state plane with all units
in feet to NAD83 UTM coordinates with all units in meter and sets these four
geokeys as the projection information:

  GeoKeyDirectoryTag version 1.1.0 number of keys 4
  - key 1024 value_offset 1 - GTModelTypeGeoKey: ModelTypeProjected
  - key 3072 value_offset 26917 - ProjectedCSTypeGeoKey: PCS_NAD83_UTM_zone_17N
  - key 3076 value_offset 9001 - ProjLinearUnitsGeoKey: Linear_Meter
  - key 4099 value_offset 9001 - VerticalUnitsGeoKey: Linear_Meter

>> las2las -i s1885565.laz -o out.las -sp83 OH_S -feet -elevation_feet -target_longlat

Reprojects the points from the Ohio_South NAD83 state plane with all units
in feet to geographic coordinates with x being longitude and y latitude and
sets these three geokeys as the projection information:

  GeoKeyDirectoryTag version 1.1.0 number of keys 3
  - key 1024 value_offset 2 - GTModelTypeGeoKey: ModelTypeGeographic
  - key 2048 value_offset 4269 - GeographicTypeGeoKey: GCS_NAD83
  - key 4099 value_offset 9001 - VerticalUnitsGeoKey: Linear_Meter

>> las2las -i s1885565.laz -o out.las -sp83 OH_S -feet -elevation_feet -target_sp83 OH_N -target_survey_feet -target_elevation_survey_feet 
>> las2las -i TO_core_last_zoom.laz -o out.laz -utm 17T
>> las2las -i TO_core_last_zoom.laz -o out.laz -utm 17T -target_latlong

other variations of adding / changing projection information.

>> las2las -i *.las -last_only

processes all LAS files that match *.las and stores only the last returns
to a corresponding LAS file called *_1.las (an added '_1' in the name).

>> las2las -i *.las -olaz -clip_tile 630000 4830000 10000

clips a 10000 by 10000 unit tile with a lower left coordinate of x=630000
and y=4830000 out of all LAS files that match *.las and stores each as a
compressed LAZ file *_1.laz (an added '_1' in the name).

>> las2las -i *.txt -iparse xyztiarn -keep_scan_angle -15 15

processes all ASCII files that match *.txt, parses them with "xyztiarn",
keeps all points whose scan angle is between -15 and 15, and stores them
to a corresponding LAS file called *_1.las (an added '_1' in the name).

>> las2las -i in.las -o out.las -clip 630250 4834500 630500 4834750

clips points of in.las whose double-precision coordinates fall outside
the box (630250,4834500) to (630500,4834750) and stores surviving points
to out.las.

>> las2las -lof file_list.txt -merged -o out.laz -clip_circle 630000 4850000 100

clips points of all files listed in the list of files file_list.txt whose
double-precision coordinates fall into the circle centered at 630000 4850000
with radius 100 and stores surviving points compressed to out.laz.

>> las2las -i in.las -o out.las -clip_z 10 100

clips points of in.las whose double-precision elevations falls outside
the range 10 to 100 and stores surviving points to out.las.

>> las2las -i in.las -o out.laz -drop_return 1

drops all points of in.las that are designated first returns by
the value in their return_number field and stores surviving points
compressed to out.laz.

>> las2las -i in.laz -o out.las -clip_scan_angle_above 15

clips all points of compressed in.laz whose scan angle is above 15 or
below -15 and stores surviving points compressed to out.laz.

>> las2las -i in.las -o out.las -clip_intensity_below 1000 -remove_extra_header

clips all points of in.las whose intensity is below 1000 and
stores surviving points to out.las. in addition all variable headers
and any additional user headers are stripped from the file.

>> las2las -i in.laz -o out.laz -last_only

extracts all last return points from compressed in.laz and stores them
compressed to out.laz.

>> las2las -i in.las -o out.las -scale_rgb_up

multiplies all rgb values in the file with 256. this is used to scale
the rgb values from standard unsigned char range (0 ... 255) to the
unsigned short range (0 ... 65535) used in the LAS format.

>> las2las -i in.laz -o out.laz -scale_rgb_down

does the opposite with compressed input and output files

>> las2las -i in.las -o out.las -subseq 1000 2000

extracts a subsequence of points by skipping the first 1000 points and
then collecting points until 2000 points were read.

>> las2las -i in.las -o out.las -keep_class 2 -keep_class 3

extracts all points classfied as 2 or 3 from in.las and stores
them to out.las.

>> las2las -i in.las -o out.las -clip_raw 63025000 483450000 63050000 483475000

similar to '-clip' but uses the integer values point.x and point.y
that the points are stored with for clipping (and not the double
precision floating point coordinates they represent). clips all the
points of in.las that have point.x<63025000 or point.y<483450000 or
point.x>63050000 or point.y>483475000 and stores surviving points to
out.las (use lasinfo.exe to see the range of point.x and point.y).

>> las2las -i in.las -o out.las -clip_raw_z 1000 4000

similar to '-clip_z' but uses the integer values point.z that the
points are stored with for clipping (and not the double-precision
floating point coordinates they represent). clips all the points
of in.las that have point.z<1000 or point.z>4000 and stores all
surviving points to out.las (use lasinfo.exe to see the range of
point.z).

other commandline arguments are

-subseq 20 100        : extract a subsequence of 100 points starting from point 20
-point_type 0         : force point type to be 0
-point_size 26        : force point size to be 26
-remove_all_vlrs      : remove all VLRs
-remove_vlr 2         : remove VLR number 2
-remove_extra         : remove extra bytes before and after the header 
-set_version 1.2      : set LAS version number to 1.2
-wgs84                : use the WGS-84 ellipsoid
-wgs72                : use the WGS-72 ellipsoid
-nad83                : use the NAD83 ellipsoid
-nad27                : use the NAD27 ellipsoid
-utm 12T              : input is UTM zone 12T 
-sp83 CO_S            : input is state plane NAD83 Colorado South
-sp27 SC_N            : input is state plane NAD27 South Carolina North 
-longlat              : input is geometric coordinates in longitude/latitude 
-latlong              : input is geometric coordinates in latitude/longitude
-survey_feet          : input uses survey feet
-feet                 : input uses feet
-meter                : input uses meter
-elevation_surveyfeet : input uses survey feet for elevation
-elevation_feet       : input uses feet for elevation
-elevation_meter      : input uses meter for elevation
-target_utm 12T              : output is UTM zone 12T 
-target_sp83 CO_S            : output is state plane NAD83 Colorado South
-target_sp27 SC_N            : output is state plane NAD27 South Carolina North 
-target_longlat              : output is geometric coordinates in longitude/latitude 
-target_latlong              : output is geometric coordinates in latitude/longitude
-target_survey_feet          : output uses survey feet
-target_feet                 : output uses feet
-target_meter                : output uses meter
-target_elevation_surveyfeet : output uses survey feet for elevation
-target_elevation_feet       : output uses feet for elevation
-target_elevation_meter      : output uses meter for elevation

-tm 609601.22 0.0 meter 33.75 -79 0.99996                 : specifies a transverse mercator projection
-tm 1804461.942257 0.0 feet 0.8203047 -2.1089395 0.99996
-lcc 609601.22 0.0 meter 33.75 -79 34.33333 36.16666      : specifies a lambertian conic confomal projection
-lcc 1640416.666667 0.0 surveyfeet 47.000000 -120.833333 47.50 48.733333
-ellipsoid 23                                             : use WGS-84 (specify '-ellipsoid -1' for a list)

for more info:

C:\lastools\bin>las2las -h
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
Supported LAS Outputs
  -o lidar.las
  -o lidar.laz
  -o xyzta.txt -oparse xyzta (on-the-fly to ASCII)
  -o terrasolid.bin
  -o nasa.qi
  -olas -olaz -otxt -obin -oqfit (specify format)
  -stdout (pipe to stdout)
  -nil    (pipe to NULL)
LAStools (by martin.isenburg@gmail.com) version 120205
usage:
las2las -i *.las -utm 13N
las2las -i *.laz -first_only -olaz
las2las -i *.las -drop_return 4 5 -olaz
las2las -latlong -target_utm 12T -i in.las -o out.las
las2las -point_type 0 -lof file_list.txt -merged -o out.las
las2las -remove_vlr 2 -scale_rgb_up -i in.las -o out.las
las2las -i in.las -clip 630000 4834500 630500 4835000 -clip_z 10 100 -o out.las
las2las -i in.txt -iparse xyzit -clip_circle 630200 4834750 100 -oparse xyzit -o out.txt
las2las -i in.las -keep_scan_angle -15 15 -o out.las
las2las -i in.las -rescale 0.01 0.01 0.01 -reoffset 0 300000 0 -o out.las
las2las -i in.las -set_version 1.2 -keep_gpstime 46.5 47.5 -o out.las
las2las -i in.las -drop_intensity_below 10 -olaz -stdout > out.laz
las2las -i in.las -last_only -drop_gpstime_below 46.75 -otxt -oparse xyzt -stdout > out.txt
las2las -i in.las -remove_all_vlrs -keep_class 2 3 4 -olas -stdout > out.las
las2las -h

---------------

if you find bugs let me (martin.isenburg@gmail.com) know.
