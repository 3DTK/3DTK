****************************************************************

  lasinfo:

  Reports the contents of the header and a short summary of the
  points. Warns when there is a difference between the header
  information and the point content. When run with option '-cd'
  or '-compute_density', lasinfo will compute the point density.

  All differences can be repaired with the '-repair' option. The
  option '-repair_bb' will only repair (or tighten) the bounding
  box while '-repair_counters' will only repair wrong reported
  number of points. 

  By default the output of lasinfo goes to stderr. The output can
  be suppressed with '-quiet' or changed to '-stdout'. To write
  into a file use '-o output.txt' or '-otxt' which, for example,
  will write to "lidar_info.txt" for a file named "lidar.las".

  The tool can also be used to modify various other entries in
  the header as described below. This needs to be done with care
  as such changes can potentially corrupt the file.

  In case you just want to report or modify the header entries
  you can skip the parsing of the points with '-no_check'. Some
  LAS files have excessive amounts of VLRs. You can supress them
  being output with '-no_vlrs'. By default lasinfo reports the 
  min and the max of every point attribute after parsing all the
  points. You can supress this with '-no_min_max'. By default the
  tool also counts the points falling outside the header bounding
  box. This can be disabled with '-no_check_outside'. If instead
  you want to report all the individual points that fall outside
  use '-report_outside'. Another interesting option is to report
  the GPS time min and max as '-gps_week' in case it is stored as
  adjusted Standard GPS time. 

  For updates check the website or join the LAStools mailing list.

  http://lastools.org/
  http://groups.google.com/group/lastools/
  http://twitter.com/lastools/
  http://facebook.com/lastools/
  http://linkedin.com/groups?gid=4408378

  Martin @lastools

****************************************************************

example usage:

>> lasinfo lidar.las

reports all information to stderr.

>> lasinfo -i lidar1.las lidar2.las

reports all information for a merged LAS file that contains the
merged contents of lidar1.las and lidar2.las to stderr.

>> lasinfo -i lidar1.las lidar2.las -single

reports the information of the two files individually to stderr.

>> lasinfo *.las -stdout

reports information for all files merged to stdout.

>> lasinfo *.las -single -stdout

reports information for all files individually to stdout.

>> lasinfo *.las -single -otxt

reports information for all files individually to *_info.txt

>> lasinfo -i lidar.las -o lidar_info.txt 

reports the information to a text file called lidar_info.txt.

>> lasinfo -i lidar.las -no_header

supresses the reporting of the header information (short: '-nh').

>> lasinfo -i lidar.las -no_vlrs

supresses the reporting of variable length records (short: '-nv').

>> lasinfo -i lidar.las -no_min_max

supresses the reporting of minimum/maximum value for each attribute
of the point records (short: '-nmm').

>> lasinfo -i lidar.las -no_check_outside

supresses checking whether points fall outside of the bounding box that
is reported in the header (short: '-nco').

>> lasinfo -i lidar.las -report_outside

reports the index and the coordinates of each point that falls outside
of the bounding box that is reported in the header (short: '-ro').

>> lasinfo -i lidar.las -nh -nv -progress 1000000

supresses reporting of the header information and the vlrs and reports
progress each time 1,000,000 points are parsed.

>> lasinfo -i lidar.las -no_check

only reports header information (short: '-nc'). does not parse the points.

>> lasinfo -i lidar.las -compute_density

computes and reports a good estimate of the point density (short: '-cd').

>> lasinfo -i lidar.las -repair_bb

corrects missing or wrong bounding box info in the header.

>> lasinfo -i lidar.las -repair_counters

corrects missing or wrong point number info in the header.

>> lasinfo -i lidar.las -set_file_source_id 27

sets the file source ID in the LAS header to 27.

>> lasinfo -i lidar.las -auto_date

sets the file creation day/year in the header to the creation date of the file.

>> lasinfo -i lidar.las -set_file_creation 8 2007

sets the file creation day/year in the header to 8/2007

>> lasinfo -i lidar.las -set_system_identifier "hello world!"

copies the first 31 characters of the string into the system_identifier field of the header 

>> lasinfo -i lidar.las -set_generating_software "this is a test (-:"

copies the first 31 characters of the string into the generating_software field of the header 

>> lasinfo -i lidar.las -set_bounding_box 4100000 1800000 150 4110000 1810000 400

sets the bounding box in the header to min_x=4100000, max_x=4110000, min_y=1800000, ...

>> lasinfo -i lidar.las -set_offset 3000000 1000000 0

CAREFUL! sets the offset in the LAS header to 3000000 1000000 0 without changing
the point coordinates. hence this will, in general, translate the point cloud.

>> lasinfo -i lidar.las -set_scale 0.001 0.001 0.001

CAREFUL! sets the scale in the LAS header to 0.001 0.001 0.001 without changing
the point coordinates. hence this will, in general, scale the point cloud.

>> lasinfo -i lidar.las -set_global_encoding 1

CAREFUL! sets the global encoding field of the LAS header to 1 without checking
whether this will corrupt the file.

>> lasinfo -i lidar.las -set_version 1.1

CAREFUL! sets the version field of the LAS header to 1.1 without checking whether
this will corrupt the file.

>> lasinfo -i lidar.las -set_header_size 235

CAREFUL! sets the header size field of the LAS header to 235 without checking
whether this will corrupt the file.

>> lasinfo -i lidar.las -set_offset_to_point_data 460

CAREFUL! sets the offset to point data field of the LAS header to 460 without
checking whether this will corrupt the file.

>> lasinfo -i lidar.las -set_point_data_format 1

CAREFUL! sets the point data format field of the LAS header to point type 1
without checking whether this will corrupt the file.

>> lasinfo -i lidar.las -set_point_data_record_length 32

CAREFUL! sets the point data record length field of the LAS header to size 32
without checking whether this will corrupt the file.

>> lasinfo -i lidar.las -set_start_of_waveform_data_packet_record 0

CAREFUL! sets the start of waveform data packet record field of the LAS header
to 0 without checking whether this will corrupt the file.

>> lasinfo -help
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
LAStools (by martin.isenburg@gmail.com) version 120206
usage:
lasinfo -i lidar.las
lasinfo -i lidar.las -compute_density -o lidar_info.txt
lasinfo -i *.las
lasinfo -i *.las -single -otxt
lasinfo -no_header -no_vlrs -i lidar.laz
lasinfo -nv -nc -stdout lidar.las
lasinfo -nv -nc -stdout *.laz -single | grep version
lasinfo -i *.las -repair
lasinfo -i *.laz -repair_bb -set_file_creation 8 2007
lasinfo -i *.las -repair_counters -set_version 1.2
lasinfo -i *.laz -set_system_identifier "hello world!" -set_generating_software "this is a test (-:"

----

if you find bugs let me (martin.isenburg@gmail.com) know.
