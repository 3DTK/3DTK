****************************************************************

  blast2dem:

  This tool can reads billion of LIDAR points from the LAS/LAZ
  format, triangulates them a seamless TIN, and rasters the TIN
  onto a DEM that can optionally be tiled. The output is either
  in BIL, ASC, IMG, XYZ, DTM, TIF, PNG or JPG format.

  For BIL, ASC, IMG, DTM, and XYZ output one typically stores
  the actual '-elevation' or '-intensity' values whereas the
  TIF, PNG, and JPG formats are usually used for a '-hillshade',
  or '-gray' coloring. The particular range of values to be
  color mapped can be clamped using '-set_min_max 10 100'.

  This is part of the BLAST extension pack of LAStools that is
  built on streaming TINs via spatial finalization & streaming
  Delaunay. Please license from martin.isenburg@gmail.com before
  you use blast2dem commercially.

  By default the generated raster is sized based on the extend
  of the bounding box. If the LAS/LAZ file was generated using
  lastile, its extend can be set to that of the tile using the
  '-use_tile_bb' option. Any "border buffer" that the tile may
  have had is then not rastered. This allows to avoid boundary
  artifacts and yet create matching tiles in parallel. The exact
  raster extend can also be defined by setting '-ll min_x min_y'
  together with '-ncols 512' and '-nrows 512'.    

  For updates check the website or join the LAStools mailing list.

  http://lastools.org/
  http://groups.google.com/group/lastools/
  http://twitter.com/lastools/
  http://facebook.com/lastools/
  http://linkedin.com/groups?gid=4408378

  Martin @lastools
 
****************************************************************

example with LAZ file from the .\lastools\data folder:

>> blast2dem -i ..\data\TO_core_last_zoom.laz -o dem.asc -v
>> blast2dem -i ..\data\TO_core_last_zoom.laz -o dem.bil -v -step 0.5
>> blast2dem -i ..\data\TO_core_last_zoom.laz -o dem.bil -v -nbits 16

>> blast2dem -i ..\data\TO_core_last_zoom.laz -o dem.png -utm 17T -v -hillshade
>> blast2dem -i ..\data\TO_core_last_zoom.laz -o dem.png -utm 17T -v -hillshade -light 0 0 1
>> blast2dem -i ..\data\TO_core_last_zoom.laz -o dem.png -utm 17T -v -gray

>> blast2dem -i ..\data\TO_core_last_zoom.laz -o dem.png -v -hillshade -nrows 200 -ncols 100
>> blast2dem -i ..\data\TO_core_last_zoom.laz -o dem.png -v -hillshade -nrows 200 -ncols 100 -ll 630300 4834550

example usage:

>> blast2dem -i *.las -merged -keep_class 2 8 -o merged.asc

combines the ground points of all LAS files that match *.las, rasters
them seamlessly with step size 1, and stores the combined DEM in ASC
format.

>> blast2dem -i *.las -oasc

rasters the elevations of all LAS files *.las with step size 1 and
stores each resulting DEM in ASC format.

>> blast2dem -i *.laz -opng -utm 17T -step 2.5 -hillshade

rasters the hillside-shaded elevations of all LAZ files *.laz with
step size 2.5 and stores the resulting DEM in PNG format and with
it a KML file that geo-references each PNG in GE with UTM zone 17T.

>> blast2dem -i huge_lidar.las -o dem.asc -step 2 -keep_class 2

creates a temporary TIN from all points in the LAS file 'lidar.las'
that are classified as ground, rasters the elevation values of the
resulting TIN onto a grid with step size 2, and stores the resulting
DEM in ASC format.

>> blast2dem -i lidar.las lidar2.las lidar3.las -hillshade -opng -step 0.5

creates a hillside-shaded DEM with step size 0.5 units for each of
the 3 LAS files 'lidar1.las', 'lidar2.las', and 'lidar3.las' and stores
the result in PNG format.

>> blast2dem -i lidar.las -o dem.tif -first_only -gray -step 2

creates a gray-scale elevation colored DEM of the elevations of
lidar.las with step size 2 in TIF format.

>> blast2dem -i lidar.las -o dem.png -first_only -false -step 2 -utm 14T

same as above but with false elevation coloring and output of a KML
file that georeferences the PNG file in Google Earth

other commandline arguments are

-kill 50              : do not raster triangles with edges longer than 50 units
-step 2               : raster with stepsize 2 (the default is 1)
-nrows 512            : raster at most 512 rows
-ncols 512            : raster at most 512 columns
-ll 300000 600000     : start rastering at these lower left x and y coordinates
-nodata -9999         : use -9999 as the nodata value in the BIL/ASC format
-intensity            : raster intensities instead of elevations
-hillshade            : color the image with hillside shading
-gray                 : gray-scale based on elevation/intensity (used with PNG/TIF/JPG)
-scale 2.0            : multiply all elevation values by 2.0 before rastering
-float_precision 0.1  : sets output float precision (used with ASC/BIL/TIF)
-nbits 32             : use 32 bits to represent the elevation (mainly used with BIL format)
-light 1 1 3          : change the direction of the light vector for hillside shading
-utm 12T              : use UTM zone 12T to spatially georeference the raster
-sp83 CO_S            : use the NAD83 Colorado South state plane for georeferencing
-sp27 SC_N            : use the NAD27 South Carolina North state plane
-longlat              : geometric coordinates in longitude/latitude order 
-latlong              : geometric coordinates in latitude/longitude order 
-wgs84                : use the WGS-84 ellipsoid
-wgs72                : use the WGS-72 ellipsoid
-nad83                : use the NAD83 ellipsoid
-nad27                : use the NAD27 ellipsoid
-survey_feet          : use survey feet
-feet                 : use feet
-meter                : use meter
-elevation_surveyfeet : use survey feet for elevation
-elevation_feet       : use feet for elevation
-elevation_meter      : use meter for elevation
-tiling_ns crater 500 : create a tiling of DEMs named crater with tiles of size 500 
-tm 609601.22 0.0 meter 33.75 -79 0.99996
-tm 1804461.942257 0.0 feet 0.8203047 -2.1089395 0.99996
-lcc 609601.22 0.0 meter 33.75 -79 34.33333 36.16666
-lcc 1640416.666667 0.0 surveyfeet 47.000000 -120.833333 47.50 48.733333
-ellipsoid 23         : use the WGS-84 ellipsoid (do -ellipsoid -1 for a list of ellipsoids)

for more info:

C:\lastools\bin>blast2dem -h
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
  -single_only
  -double_only
  -triple_only
  -quadruple_only
  -quintuple_only
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
Supported Raster Outputs
  -o dem.asc
  -o dem.bil
  -o hillshade.png
  -o dem.tif
  -o false_color.jpg
  -oasc -obil -opng -otif -ojpg
usage:
blast2dem -i huge_lidar.las -o dem.asc -step 2
blast2dem -i huge_lidar.laz -o dem.bil -keep_class 2
blast2dem -i *.las -oasc -step 5
blast2dem -i huge_lidar.las -o lidar.png -hillshade
blast2dem -i *.laz -opng -hillshade -light 1 1 1 -last_only
blast2dem -i huge_lidar.laz -ll 640000 4320000 -ncols 10000 -nrows 10000 -o dem.asc
blast2dem -i huge_lidar.las -keep_class 2 8 3 -o dem.tif
blast2dem -h

---------------

if you find bugs let me (martin.isenburg@gmail.com) know
