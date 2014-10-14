****************************************************************

  lasgrid:

  This tool reads LIDAR from LAS/LAZ/ASCII and grids them onto
  a raster. The most important parameter '-step n' specifies the
  n x n area that of LiDAR points that are gridded on one raster
  (or pixel). The output is either in BIL, ASC, IMG, TIF, PNG,
  JPG, XYZ,or DTM format. The tool can raster the '-elevation'
  or the '-intensity' of each point and stores the '-lowest' or
  the '-highest', the '-average', or the standard deviation
  '-stddev'.

  Other gridding options are '-scan_angle_abs_lowest', '-density',
  '-density_16bit', '-user_data_highest', '-point_source_lowest',
  and more. See below for a complete list.

  This tool can read BILLIONS of points very efficiently. By
  default it uses only 500MB of main memory. You can increase
  this with the '-mem 2000' option to up to 2 GB. The tool
  pages larger rasters out to disk. If you have a second hard
  drive it is beneficial to use this instead. You can specify
  the temporary file location with '-temp_files E:\temp\temp'.

  For BIL, ASC, IMG, DTM, and XYZ output one typically stores
  the actual (elevation, intensity, ...) values whereas for TIF,
  PNG, and JPG one usually chooses to express the variation with
  '-gray' or with '-false' colors for simple visualizion. Here
  the variation can be limited with '-set_min_max 10 100' to a
  particular range or it can be set to '-compute_min_max'.

  Optionally, a KML file is generated that allows the resulting
  raster  to be immediately displayed inside a geospatial context
  provided by Google Earth (for TIF/PNG/JPG images). In case the
  LAS/LAZ file contains projection information (i.e. geo keys
  as variable length records) this metadata is used to correctly
  geo-reference the KML file. It is also possible to provide the
  proper geo-referencing information in the command-line.

  By default the generated raster spans the extend of all LiDAR
  points. It is possible to specify this to be identical to the
  bounding box with '-use_bb' or the bounding box of the tile
  with '-use_tile_bb' (the latter only if the LAS/LAZ file was
  generated using lastile). The extend can also be defined by
  setting '-ll min_x min_y' plus '-ncols 512' and '-nrows 512'.

  Use '-subsample n' with n > 1 to anti-alias "hard" gridding of
  LiDAR points by their x and y coordinate into disjunct rasters.
  The option '-subsample 3' adds each LiDAR point 9 times to the
  raster at locations (x/y), (x+0.33*step/y), (x+0.66*step/y),
  (x/y+0.33*step) (x+0.33*step/y+0.33*step) (x+0.66*step/y+0.33*step),
  (x/y+0.66*step) (x+0.33*step/y+0.66*step) (x+0.66*step/y+0.66*step)
  and thereby "washes out" hard boundaries. Obviously, this will
  lead to wrongful increase in the '-density' counters, but the
  '-averages', '-highest', '-lowest', and '-stddev' will have less
  aliasing

  Please license from martin.isenburg@gmail.com to use lasgrid
  commercially.

  For updates check the website or join the LAStools mailing list.

  http://lastools.org/
  http://groups.google.com/group/lastools/
  http://twitter.com/lastools/
  http://facebook.com/lastools/
  http://linkedin.com/groups?gid=4408378

  Martin @lastools
 
****************************************************************

example command lines with this LAZ file:
http://www.cs.unc.edu/~isenburg/lastools/download/test/s1885565.laz

lasgrid -v -sp83 OH_S -feet -i s1885565.laz -step 10 -gray -o elev_low.png
lasgrid -v -sp83 OH_S -feet -i s1885565.laz -step 10 -gray -o elev_high.png -highest
lasgrid -v -sp83 OH_S -feet -i s1885565.laz -step 10 -gray -o elev_std.png -stddev
lasgrid -v -sp83 OH_S -feet -i s1885565.laz -step 10 -gray -keep_class 2 -o elev_grnd_low.png
lasgrid -v -sp83 OH_S -feet -i s1885565.laz -step 10 -gray -keep_class 2 -o elev_grnd_low_fill.png -fill 5
lasgrid -v -sp83 OH_S -feet -i s1885565.laz -step 10 -gray -keep_class 2 -o elev_grnd_std_fill.png -stddev -fill 5

lasgrid -v -sp83 OH_S -feet -i s1885565.laz -step 10 -false -o elev_f_low.png
lasgrid -v -sp83 OH_S -feet -i s1885565.laz -step 10 -false -o elev_f_high.png -highest
lasgrid -v -sp83 OH_S -feet -i s1885565.laz -step 10 -false -o elev_f_std.png -stddev
lasgrid -v -sp83 OH_S -feet -i s1885565.laz -step 10 -false -keep_class 2 -o elev_f_grnd_low.png
lasgrid -v -sp83 OH_S -feet -i s1885565.laz -step 10 -false -keep_class 2 -o elev_f_grnd_low_fill.png -fill 5
lasgrid -v -sp83 OH_S -feet -i s1885565.laz -step 10 -false -keep_class 2 -o elev_f_grnd_std_fill.png -std -fill 5

lasgrid -v -sp83 OH_S -feet -i s1885565.laz -step 10 -gray -intensity -o int_low.png
lasgrid -v -sp83 OH_S -feet -i s1885565.laz -step 10 -gray -intensity -o int_high.png -highest
lasgrid -v -sp83 OH_S -feet -i s1885565.laz -step 10 -gray -intensity -o int_avg.png -average
lasgrid -v -sp83 OH_S -feet -i s1885565.laz -step 10 -gray -intensity -o int_std.png -stddev

lasgrid -v -sp83 OH_S -feet -i s1885565.laz -step 10 -false -intensity -o int_f_low.png
lasgrid -v -sp83 OH_S -feet -i s1885565.laz -step 10 -false -intensity -o int_f_high.png -highest
lasgrid -v -sp83 OH_S -feet -i s1885565.laz -step 10 -false -intensity -o int_f_avg.png -average
lasgrid -v -sp83 OH_S -feet -i s1885565.laz -step 10 -false -intensity -o int_f_std.png -stddev

example command lines with data from http://liblas.org/samples

lasgrid -v -o result.png -false -i line_27007_dd.las -lonlat -step 0.00002 -stddev
lasgrid -v -o result.png -false -i IowaDNR-CloudPeakSoft-1.0-UTM15N.las 
lasgrid -v -o result.png -false -i LAS12_Sample_withIntensity_Quick_Terrain_Modeler.las -step 2 -stddev
lasgrid -v -o result.png -false -i LAS12_Sample_withRGB_Quick_Terrain_Modeler.las -high
lasgrid -v -o result.png -false -i Lincoln.las -utm 14T -step 5
lasgrid -v -o result.png -false -i S1C1_strip021.las -set_min_max 1630 1690 -step 2 -high
lasgrid -v -o result.png -false -i "Serpent Mound Model LAS Data.las" -intensity -set_min_max 0 400
lasgrid -v -o result.png -false -i USACE_Merrick_lots_of_VLRs.las -step 10 -intensity

example usage:

>> lasgrid -i *.las -opng -step 5 -false -sp83 OH_N

rasters for each *.las files the lowest elevation of all points
that fall into cells of size 5 by 5, stores the resulting grid
in PNG format using false coloring, and creates a KML file that
maps the PNG to state plane NAD83 of Northern Ohio.

>> lasgrid -i *.txt -iparse xyz -oasc -step 2 -highest

rasters for each *.txt files the highest elevation of all points
that fall into cells of size 2 by 2 and stores the resulting grids
in ASC format.

>> lasgrid -i lidar1.las lidar2.las lidar3.las -merged -o dem.bil -step 4 -highest -intensity

merges the points of lidar1.las lidar2.las lidar3.las and rasters
the highest intensity of all points that fall into cells of size 4
by 4 and stores the resulting grid in BIL format.

>> lasgrid -v -i lidar.las -o dem.png -step 5 -false -stddev -utm 14T

rasters the standard deviations of the elevation of all points that
fall into cells of size 5 by 5 and stores the resulting grid in PNG
format using false coloring and creates a KML file that maps the file
to UTM zone 14

>> lasgrid -v -i lidar.las -o dem.jpg -last_only -false -highest -step 2

rasters the highest elevation from all points that fall into cells of
size 2 by 2 units and are classfied as last returns and stores the
resulting grid in JPG format using false elevation coloring

>> lasgrid -v -i lidar.las -o dem.tif -keep_class 2 -keep_class 3 -gray

rasters the lowest elevation from all points that fall into cells of
size 1 by 1 unit and are classfied as 2 or 3 and stores the resulting
grid in TIF format using gray-scale elevation coloring

>> lasgrid -v -i lidar.las -o dem.asc -step 2 -average

rasters the average elevations from all points that fall into cells of
size 2 by 2 units and stores the resulting grid in ASC format.

>> lasgrid -v -lof lidar_files.txt -merged -o merged.bil -step 10

rasters the lowest elevation from all points of all files listed in
lidar_files.txt that fall into cells of size 10 by 10 units and stores
the resulting grid in BIL format with 32 bits floats.

>> lasgrid -v -lof lidar_files.txt -obil -step 10

rasters the lowest elevation for each file listed in lidar_files.txt
individually that fall into cells of size 10 by 10 units and stores
each resulting grid in BIL format with 32 bits floats.

the following commands generate some interesting georeferenced grids that
you can look at in Google Earth by double clicking the generated KML file

>> lasgrid -i ..\data\test.las -false -o test.png
>> lasgrid -i ..\data\TO_core_last_zoom.las -gray -o toronto.png -utm 17T
>> lasgrid -i ..\data\SerpentMound.las -false -o SerpentMound.png

other commandline arguments are

-mem                   : amount of main memory to use in MB (500 - 2000) [default: 500]
-temp_files            : base file name for temp files (example: E:\tmp)
-step 2                : raster with stepsize 2 [default: 1]
-fill 5                : fills voids in the grid with a square search radius of 5 
-nrows 512             : raster at most 512 rows
-ncols 512             : raster at most 512 columns
-ll 300000 600000      : start rastering at these lower left x and y coordinates
-nodata 9999           : use 9999 as the nodata value in the BIL / ASC format
-elevation             : use elevation values
-intensity             : use intensity values
-highest -high -max    : for each grid cell keep highest value
-lowest -low -min      : for each grid cell keep lowest value
-average -avg -mean    : for each grid cell compute average
-stddev -std           : for each grid cell compute standard deviation
-density               : count point densities with an 8 bit counter
-density_16bit         : count point densities with a 16 bit counter
-density_32bit         : count point densities with a 32 bit counter
-scan_angle_lowest     : for each grid cell keep lowest scan angle value 
-scan_angle_highest    : for each grid cell keep highest scan angle value 
-scan_angle_abs_lowest : for each grid cell keep lowest absolute scan angle value 
-scan_angle_abs_highest: for each grid cell keep highest absolute scan angle value 
-user_data_lowest      : for each grid cell keep lowest user data value 
-user_data_highest     : for each grid cell keep highest user data value 
-point_source_lowest   : for each grid cell keep lowest point source value 
-point_source_highest  : for each grid cell keep highest point source value 
-gray                  : gray-scale based on min/max range (used with PNG/TIF/JPG)
-false                 : false-color based on min/max range (used with PNG/TIF/JPG)
-set_min_max           : sets min & max range for -gray and -false
-compute_min_max       : computes the range for -gray and -false
-nbits 16              : use 16 bits to represent the elevation (mainly used with BIL format)
-utm 12T               : use UTM zone 12T to spatially georeference the raster
-sp83 CO_S             : use the NAD83 Colorado South state plane for georeferencing
-sp27 SC_N             : use the NAD27 South Carolina North state plane
-longlat               : geometric coordinates in longitude/latitude order 
-latlong               : geometric coordinates in latitude/longitude order 
-wgs84                 : use the WGS-84 ellipsoid
-wgs72                 : use the WGS-72 ellipsoid
-nad83                 : use the NAD83 ellipsoid
-nad27                 : use the NAD27 ellipsoid
-survey_feet           : use survey feet
-feet                  : use feet
-meter                 : use meter
-elevation_surveyfeet  : use survey feet for elevation
-elevation_feet        : use feet for elevation
-elevation_meter       : use meter for elevation
-tiling_ns crater 500  : create a tiling of DEMs named crater with tiles of size 500 
-tm 609601.22 0.0 meter 33.75 -79 0.99996
-transverse_mercator 1804461.942257 0.0 feet 0.8203047 -2.1089395 0.99996
-lcc 609601.22 0.0 meter 33.75 -79 34.33333 36.16666
-lambert_conic_conformal 1640416.666667 0.0 surveyfeet 47.000000 -120.833333 47.50 48.733333
-ellipsoid 23          : use the WGS-84 ellipsoid (do -ellipsoid -1 for a list)

for more info:

C:\lastools\bin>lasgrid -h
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
Supported Raster Outputs
  -o dem.asc
  -o dem.bil
  -o dem.img
  -o hillshade.png
  -o dem.tif
  -o false_color.jpg
  -oasc -obil -oimg -opng -otif -ojpg -nil
LAStools (by martin.isenburg@gmail.com) version 110902 (unlicensed)
Supported raster operations
  -elevation_lowest (default)
  -elevation_highest
  -elevation_average
  -elevation_stddev
  -intensity_lowest
  -intensity_highest
  -intensity_average
  -intensity_stddev
  -density
  -density_16bit
  -density_32bit
  -scan_angle_lowest
  -scan_angle_highest
  -scan_angle_abs_lowest
  -scan_angle_abs_highest
  -user_data_lowest
  -user_data_highest
  -point_source_lowest
  -point_source_highest
usage:
lasgrid -i in.las -o out.asc -mem 1000
lasgrid -i in.las -o out.img -elevation_highest -mem 2000 -temp_files E:\tmp
lasgrid -i in.las -o out.png -elevation_stddev -false -step 5
lasgrid -i in.las -o out.asc -intensity_lowest -step 2 -temp_files E:\tmp
lasgrid -i in.las -o out.png -scan_angle_abs_lowest -gray -step 2
lasgrid -i in.las -o out.tif -density -false -step 2
lasgrid -i in.las -o out.bil -density_16bit -step 5
lasgrid -h

---------------

if you find bugs let me (martin.isenburg@gmail.com) know
