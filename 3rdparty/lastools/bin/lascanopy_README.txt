****************************************************************

  lascanopy:

  This tool reads LiDAR from LAS/LAZ/BIN/SHP/QFIT/ASCII, computes
  popular forestry metrics, and grids them onto a raster. A very
  important parameter is '-step n' that specifies the n x n area
  of LiDAR points that are gridded on one raster (or pixel). The
  default of step is 20. The output can be either in BIL, ASC, IMG,
  TIF, XYZ, FLT, or DTM format.

  The tool can concurrently compute a number height percentiles
  ('-p 5 10 25 50 75 90'), the '-min', the '-max', the '-avg', and
  the '-std' of all heights above the cutoff that is usually at
  breast height. The default is 1.37 and it can be changed with
  the option '-height_cutoff 0.75'.

  By default the generated raster spans the extend of the header
  bounding box. You can use the bounding box of the tile with
  '-use_tile_bb' (which only makes sense if the LAS/LAZ file was
  generated using lastile) or the original bounding box in case
  of a buffered tile with '-use_orig_bb' (which only makes sense
  if the input has an on-the-fly buffer aka '-buffered 50'). The
  extend can also be defined by setting '-ll min_x min_y' plus
  '-ncols 512' and '-nrows 512'.

  It is very important (at the moment) that the input files are
  height normalized, meaning that the z coordinate of each point
  corresponds to the height above ground and not the elevation of
  the point. With 'lasheight -i in.laz -replace_z -i out.laz' you
  can height-normalize a ground classified LiDAR file.

  Let me know which other metrics you would like to see ...

  Please license from martin.isenburg@gmail.com to use lascanopy
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

>> lascanopy -i *.las -min -max -avg

for each *.las files and for all height above 1.37 it computes
the minimum, maximum, and average value from all points that fall
into cells of size 20 by 20 and stores the resulting grid in ASC
format using the endings '_min.asc', '_max.asc', '_avg.asc'.

>> lascanopy -i lidar*.laz -merged -p 20 40 60 80 -step 10 -o dem.bil

merges the points of all files that match the wildcard lidar*.laz
on-the-fly into one file and computes for all heights above 1.37
the 20th, 40th, 60th, and 80th percentile for 10 by 10 grid cells
and stores the resulting rasters in BIL format using the endings
'_p20.bil', '_p40.bil', '_p60.bil', and '_p80.bil'.

other commandline arguments are

-mem                   : amount of main memory to use in MB (500 - 2000) [default: 500]
-temp_files            : base file name for temp files (example: E:\tmp)
-step 2                : raster with stepsize 2 [default: 1]
-nrows 512             : raster at most 512 rows
-ncols 512             : raster at most 512 columns
-ll 300000 600000      : start rastering at these lower left x and y coordinates
-nodata 9999           : use 9999 as the nodata value in the BIL / ASC format
-max                   : for each grid cell keep highest value
-min                   : for each grid cell keep lowest value
-avg                   : for each grid cell compute average
-std                   : for each grid cell compute standard deviation
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

C:\lastools\bin>lascanopy -h
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
  -set_return_number 1
  -change_return_number_from_to 2 1
  -set_number_of_returns 2
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
  -i *.las - merged
  -i flight0??.laz flight1??.laz
  -i terrasolid.bin
  -i esri.shp
  -i nasa.qi
  -i lidar.txt -iparse xyzti -iskip 2 (on-the-fly from ASCII)
  -i lidar.txt -iparse xyzi -itranslate_intensity 1024
  -lof file_list.txt
  -stdin (pipe from stdin)
  -rescale 0.01 0.01 0.001
  -rescale_xy 0.01 0.01
  -rescale_z 0.01
  -reoffset 600000 4000000 0
Supported Raster Outputs
  -o dtm.asc
  -o dsm.bil
  -o canopy.flt
  -o dtm.dtm
  -o density.xyz
  -o intensity.img
  -o hillshade.png
  -o slope.tif
  -o false_color.jpg
  -oasc -obil -oflt -oimg -opng -odtm -otif -ojpg -oxyz -nil
  -odir C:\data\hillshade (specify output directory)
  -odix _small (specify file name appendix)
  -ocut 2 (cut the last two characters from name)
LAStools (by martin.isenburg@gmail.com) version 120914 (unlicensed)
usage:
lascanopy -i *.las -max -avg
lascanopy -i *.laz -p 1 5 10 25 50 75 90 95 99
lascanopy -i *.laz -merged -max -avg -p 20 40 60 80 -o merged.dtm
lascanopy -h

---------------

if you find bugs let me (martin.isenburg@gmail.com) know
