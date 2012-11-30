/*
===============================================================================

  FILE:  lasinfo.cpp

  CONTENTS:

    This tool reads a LIDAR file in LAS or LAZ format and prints out the info
    from the standard header. It also prints out info from any variable length
    headers and gives detailed info on any geokeys that may be present (this
    can be disabled with the '-no_vrls' flag). The real bounding box of the
    points is computed and compared with the bounding box specified in the header
    (this can be disables with the '-no_check' flag). It is also possible to
    change or repair some aspects of the header 

  PROGRAMMERS:

    martin.isenburg@gmail.com  -  http://rapidlasso.com

  COPYRIGHT:

    (c) 2007-2012, martin isenburg, rapidlasso - tools to catch reality

    This is free software; you can redistribute and/or modify it under the
    terms of the GNU Lesser General Licence as published by the Free Software
    Foundation except for (R). See the LICENSE.txt file for more information.

    This software is distributed WITHOUT ANY WARRANTY and without even the
    implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

  CHANGE HISTORY:

     9 July 2012 -- fixed crash that occured when input had a corrupt VLRs
     7 January 2012 -- set bounding box / file source id / point type & size / ...
     6 January 2012 -- make area/density optional
     5 September 2011 -- also compute total area covered and point densities
    26 January 2011 -- added LAStransform because it allows quick previews
    21 January 2011 -- added LASreadOpener and reading of multiple LAS files 
     4 January 2011 -- added the LASfilter to clip or eliminate points 
    10 July 2009 -- '-auto_date' sets the day/year from the file creation date
    12 March 2009 -- updated to ask for input if started without arguments 
     9 March 2009 -- added output for size of user-defined header data
    17 September 2008 -- updated to deal with LAS format version 1.2
    13 July 2007 -- added the option to "repair" the header and change items
    11 June 2007 -- fixed number of return counts after Vinton found another bug
     6 June 2007 -- added lidardouble2string() after Vinton Valentine's bug report
    25 March 2007 -- sitting at the Pacific Coffee after doing the escalators

===============================================================================
*/

#include "lasreader.hpp"
#include "lasutility.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#ifdef _WIN32
#include <windows.h>
#endif

static const char * LASpointClassification [] = {
  "Created, never classified",
  "Unclassified",
  "Ground",
  "Low Vegetation",
  "Medium Vegetation",
  "High Vegetation",
  "Building",
  "Low Point (noise)",
  "Model Key-point (mass point)",
  "Water",
  "Reserved for ASPRS Definition",
  "Reserved for ASPRS Definition",
  "Overlap Points",
  "Reserved for ASPRS Definition",
  "Reserved for ASPRS Definition",
  "Reserved for ASPRS Definition",
  "Reserved for ASPRS Definition",
  "Reserved for ASPRS Definition",
  "Reserved for ASPRS Definition",
  "Reserved for ASPRS Definition",
  "Reserved for ASPRS Definition",
  "Reserved for ASPRS Definition",
  "Reserved for ASPRS Definition",
  "Reserved for ASPRS Definition",
  "Reserved for ASPRS Definition",
  "Reserved for ASPRS Definition",
  "Reserved for ASPRS Definition",
  "Reserved for ASPRS Definition",
  "Reserved for ASPRS Definition",
  "Reserved for ASPRS Definition",
  "Reserved for ASPRS Definition",
  "Reserved for ASPRS Definition"
};

void usage(bool error=false, bool wait=false)
{
  fprintf(stderr,"usage:\n");
  fprintf(stderr,"lasinfo -i lidar.las\n");
  fprintf(stderr,"lasinfo -i lidar.las -compute_density -o lidar_info.txt\n");
  fprintf(stderr,"lasinfo -i *.las\n");
  fprintf(stderr,"lasinfo -i *.las -single -otxt\n");
  fprintf(stderr,"lasinfo -no_header -no_vlrs -i lidar.laz\n");
  fprintf(stderr,"lasinfo -nv -nc -stdout lidar.las\n");
  fprintf(stderr,"lasinfo -nv -nc -stdout *.laz -single | grep version\n");
  fprintf(stderr,"lasinfo -i *.las -repair\n");
  fprintf(stderr,"lasinfo -i *.laz -repair_bb -set_file_creation 8 2007\n");
  fprintf(stderr,"lasinfo -i *.las -repair_counters -set_version 1.2\n");
  fprintf(stderr,"lasinfo -i *.laz -set_system_identifier \"hello world!\" -set_generating_software \"this is a test (-:\"\n");
  if (wait)
  {
    fprintf(stderr,"<press ENTER>\n");
    getc(stdin);
  }
  exit(error);
}

static void byebye(bool error=false, bool wait=false)
{
  if (wait)
  {
    fprintf(stderr,"<press ENTER>\n");
    getc(stdin);
  }
  exit(error);
}

static inline void VecUpdateMinMax3dv(double min[3], double max[3], const double v[3])
{
  if (v[0]<min[0]) min[0]=v[0]; else if (v[0]>max[0]) max[0]=v[0];
  if (v[1]<min[1]) min[1]=v[1]; else if (v[1]>max[1]) max[1]=v[1];
  if (v[2]<min[2]) min[2]=v[2]; else if (v[2]>max[2]) max[2]=v[2];
}

static inline void VecCopy3dv(double v[3], const double a[3])
{
  v[0] = a[0];
  v[1] = a[1];
  v[2] = a[2];
}

static int lidardouble2string(char* string, double value)
{
  int len;
  len = sprintf(string, "%.15f", value) - 1;
  while (string[len] == '0') len--;
  if (string[len] != '.') len++;
  string[len] = '\0';
  return len;
}

static int lidardouble2string(char* string, double value, double precision)
{
  if (precision == 0.1)
    sprintf(string, "%.1f", value);
  else if (precision == 0.01)
    sprintf(string, "%.2f", value);
  else if (precision == 0.001)
    sprintf(string, "%.3f", value);
  else if (precision == 0.0001)
    sprintf(string, "%.4f", value);
  else if (precision == 0.00001)
    sprintf(string, "%.5f", value);
  else if (precision == 0.000001)
    sprintf(string, "%.6f", value);
  else if (precision == 0.0000001)
    sprintf(string, "%.7f", value);
  else if (precision == 0.00000001)
    sprintf(string, "%.8f", value);
  else
    return lidardouble2string(string, value);
  return strlen(string)-1;
}

#ifdef COMPILE_WITH_GUI
extern int lasinfo_gui(int argc, char *argv[], LASreadOpener* lasreadopener);
#endif

int main(int argc, char *argv[])
{
  int i;
#ifdef COMPILE_WITH_GUI
  bool gui = false;
#endif
  bool verbose = false;
  bool otxt = false;
  bool no_header = false;
  bool no_variable_header = false;
  bool no_min_max = false;
  bool check_points = true;
  bool compute_density = false;
  bool gps_week = false;
  bool check_outside = true;
  bool report_outside = false;
  bool repair_bb = false;
  bool repair_counters = false;
  bool change_header = false;
  I32 set_file_source_id = -1;
  I32 set_global_encoding = -1;
  I8 set_version_major = -1;
  I8 set_version_minor = -1;
	I8* set_system_identifier = 0;
	I8* set_generating_software = 0;
	I32 set_creation_day = -1;
	I32 set_creation_year = -1;
  I32 set_point_data_format = -1;
  I32 set_point_data_record_length = -1;
  U16 set_header_size = 0;
  U32 set_offset_to_point_data = 0;
  F64* set_bounding_box = 0;
  F64* set_offset = 0;
  F64* set_scale = 0;
  I64 set_start_of_waveform_data_packet_record = -1;
  bool auto_date_creation = false;
  char* file_name_out = 0;
  FILE* file_out = stderr;
  U32 horizontal_units = 0; 
  U32 progress = 0;

  LAShistogram lashistogram;
  LASreadOpener lasreadopener;

  if (argc == 1)
  {
#ifdef COMPILE_WITH_GUI
    return lasinfo_gui(argc, argv, 0);
#else
    fprintf(stderr,"lasinfo.exe is better run in the command line\n");
    char file_name[256];
    fprintf(stderr,"enter input file: "); fgets(file_name, 256, stdin);
    file_name[strlen(file_name)-1] = '\0';
    lasreadopener.set_file_name(file_name);
#endif
  }
  else
  {
    for (i = 1; i < argc; i++)
    {
      if (argv[i][0] == '–') argv[i][0] = '-';
    }
    if (!lashistogram.parse(argc, argv)) byebye(true);
    if (!lasreadopener.parse(argc, argv)) byebye(true);
  }

  for (i = 1; i < argc; i++)
  {
    if (argv[i][0] == '\0')
    {
      continue;
    }
    else if (strcmp(argv[i],"-h") == 0 || strcmp(argv[i],"-help") == 0)
    {
      fprintf(stderr, "LAStools (by martin.isenburg@gmail.com) version %d\n", LAS_TOOLS_VERSION);
      usage();
    }
    else if (strcmp(argv[i],"-v") == 0 || strcmp(argv[i],"-verbose") == 0)
    {
      verbose = true;
    }
    else if (strcmp(argv[i],"-version") == 0)
    {
      fprintf(stderr, "LAStools (by martin.isenburg@gmail.com) version %d\n", LAS_TOOLS_VERSION);
      byebye();
    }
    else if (strcmp(argv[i],"-gui") == 0)
    {
#ifdef COMPILE_WITH_GUI
      gui = true;
#else
      fprintf(stderr, "WARNING: not compiled with GUI support. ignoring '-gui' ...\n");
#endif
    }
    else if (strcmp(argv[i],"-quiet") == 0)
    {
      file_out = 0;
    }
    else if (strcmp(argv[i],"-o") == 0)
    {
      if ((i+1) >= argc)
      {
        fprintf(stderr,"ERROR: '%s' needs 1 argument: file_name\n", argv[i]);
        byebye(true);
      }
      i++;
      file_name_out = argv[i];
    }
    else if (strcmp(argv[i],"-stdout") == 0)
    {
      file_out = stdout;
    }
    else if (strcmp(argv[i],"-otxt") == 0)
    {
      otxt = true;
    }
    else if (strcmp(argv[i],"-nh") == 0 || strcmp(argv[i],"-no_header") == 0)
    {
      no_header = true;
    }
    else if (strcmp(argv[i],"-nv") == 0 || strcmp(argv[i],"-no_vlrs") == 0)
    {
      no_variable_header = true;
    }
    else if (strcmp(argv[i],"-nmm") == 0 || strcmp(argv[i],"-no_min_max") == 0)
    {
      no_min_max = true;
    }
    else if (strcmp(argv[i],"-nc") == 0 || strcmp(argv[i],"-no_check") == 0)
    {
      check_points = false;
    }
    else if (strcmp(argv[i],"-cd") == 0 || strcmp(argv[i],"-compute_density") == 0)
    {
      compute_density = true;
    }
    else if (strcmp(argv[i],"-gw") == 0 || strcmp(argv[i],"-gps_week") == 0)
    {
      gps_week = true;
    }
    else if (strcmp(argv[i],"-nco") == 0 || strcmp(argv[i],"-no_check_outside") == 0)
    {
      check_outside = false;
    }
    else if (strcmp(argv[i],"-ro") == 0 || strcmp(argv[i],"-report_outside") == 0)
    {
      report_outside = true;
      check_outside = true;
    }
    else if (strcmp(argv[i],"-repair") == 0)
    {
      lasreadopener.set_merged(FALSE);
      repair_bb = true;
      repair_counters = true;
    }
    else if (strcmp(argv[i],"-repair_bb") == 0)
    {
      lasreadopener.set_merged(FALSE);
      repair_bb = true;
    }
    else if (strcmp(argv[i],"-repair_counters") == 0)
    {
      lasreadopener.set_merged(FALSE);
      repair_counters = true;
    }
    else if (strcmp(argv[i],"-auto_date") == 0 || strcmp(argv[i],"-auto_creation_date") == 0 || strcmp(argv[i],"-auto_creation") == 0)
    {
      lasreadopener.set_merged(FALSE);
      auto_date_creation = true;
    }
    else if (strcmp(argv[i],"-set_file_source_id") == 0)
    {
      lasreadopener.set_merged(FALSE);
      if ((i+1) >= argc)
      {
        fprintf(stderr,"ERROR: '%s' needs 1 argument: index\n", argv[i]);
        byebye(true);
      }
			i++;
			set_file_source_id = atoi(argv[i]);
      change_header = true;
		}
    else if (strcmp(argv[i],"-set_system_identifier") == 0)
    {
      lasreadopener.set_merged(FALSE);
      if ((i+1) >= argc)
      {
        fprintf(stderr,"ERROR: '%s' needs 1 argument: name\n", argv[i]);
        byebye(true);
      }
			i++;
			set_system_identifier = new I8[32];
      memset(set_system_identifier, 0, 32);
      strncpy(set_system_identifier, argv[i], 32);
      change_header = true;
		}
    else if (strcmp(argv[i],"-set_generating_software") == 0)
    {
      lasreadopener.set_merged(FALSE);
      if ((i+1) >= argc)
      {
        fprintf(stderr,"ERROR: '%s' needs 1 argument: name\n", argv[i]);
        byebye(true);
      }
			i++;
			set_generating_software = new I8[32];
      memset(set_generating_software, 0, 32);
      strncpy(set_generating_software, argv[i], 32);
      change_header = true;
		}
    else if (strcmp(argv[i],"-set_bb") == 0 || strcmp(argv[i],"-set_bounding_box") == 0)
    {
      lasreadopener.set_merged(FALSE);
      if ((i+6) >= argc)
      {
        fprintf(stderr,"ERROR: '%s' needs 6 arguments: min_x min_y min_z max_x max_y max_z\n", argv[i]);
        byebye(true);
      }
			set_bounding_box = new F64[6];
			i++;
      set_bounding_box[1] = atof(argv[i]);
			i++;
      set_bounding_box[3] = atof(argv[i]);
			i++;
      set_bounding_box[5] = atof(argv[i]);
			i++;
      set_bounding_box[0] = atof(argv[i]);
			i++;
      set_bounding_box[2] = atof(argv[i]);
			i++;
      set_bounding_box[4] = atof(argv[i]);
      change_header = true;
		}
    else if (strcmp(argv[i],"-set_offset") == 0)
    {
      lasreadopener.set_merged(FALSE);
      if ((i+3) >= argc)
      {
        fprintf(stderr,"ERROR: '%s' needs 3 arguments: x y z\n", argv[i]);
        byebye(true);
      }
			set_offset = new F64[3];
			i++;
      set_offset[0] = atof(argv[i]);
			i++;
      set_offset[1] = atof(argv[i]);
			i++;
      set_offset[2] = atof(argv[i]);
      change_header = true;
		}
    else if (strcmp(argv[i],"-set_scale") == 0)
    {
      lasreadopener.set_merged(FALSE);
      if ((i+3) >= argc)
      {
        fprintf(stderr,"ERROR: '%s' needs 3 arguments: x y z\n", argv[i]);
        byebye(true);
      }
			set_scale = new F64[3];
			i++;
      set_scale[0] = atof(argv[i]);
			i++;
      set_scale[1] = atof(argv[i]);
			i++;
      set_scale[2] = atof(argv[i]);
      change_header = true;
		}
    else if (strcmp(argv[i],"-set_global_encoding") == 0)
    {
      lasreadopener.set_merged(FALSE);
      if ((i+1) >= argc)
      {
        fprintf(stderr,"ERROR: '%s' needs 1 argument: number\n", argv[i]);
        byebye(true);
      }
			i++;
      set_global_encoding = atoi(argv[i]);
      change_header = true;
		}
    else if (strcmp(argv[i],"-set_version") == 0)
    {
      lasreadopener.set_merged(FALSE);
      if ((i+1) >= argc)
      {
        fprintf(stderr,"ERROR: '%s' needs 1 argument: major.minor\n", argv[i]);
        byebye(true);
      }
      i++;
      int major;
      int minor;
      if (sscanf(argv[i],"%d.%d",&major,&minor) != 2)
      {
        fprintf(stderr,"ERROR: cannot understand argument '%s' of '%s'\n", argv[i], argv[i-1]);
        usage();
      }
      set_version_major = (I8)major;
      set_version_minor = (I8)minor;
      change_header = true;
    }
    else if (strcmp(argv[i],"-set_creation_date") == 0 || strcmp(argv[i],"-set_file_creation") == 0)
    {
      lasreadopener.set_merged(FALSE);
      if ((i+2) >= argc)
      {
        fprintf(stderr,"ERROR: '%s' needs 2 arguments: day year\n", argv[i]);
        byebye(true);
      }
			i++;
      set_creation_day = (U16)atoi(argv[i]);
			i++;
      set_creation_year = (U16)atoi(argv[i]);
      change_header = true;
		}
    else if (strcmp(argv[i],"-set_header_size") == 0 )
    {
      lasreadopener.set_merged(FALSE);
      if ((i+1) >= argc)
      {
        fprintf(stderr,"ERROR: '%s' needs 1 argument: size\n", argv[i]);
        byebye(true);
      }
			i++;
      set_header_size = atoi(argv[i]);
      change_header = true;
    }
    else if (strcmp(argv[i],"-set_offset_to_point_data") == 0)
    {
      lasreadopener.set_merged(FALSE);
      if ((i+1) >= argc)
      {
        fprintf(stderr,"ERROR: '%s' needs 1 argument: offset\n", argv[i]);
        byebye(true);
      }
			i++;
      set_offset_to_point_data = atoi(argv[i]);
      change_header = true;
    }
    else if (strcmp(argv[i],"-set_point_data_format") == 0)
    {
      lasreadopener.set_merged(FALSE);
      if ((i+1) >= argc)
      {
        fprintf(stderr,"ERROR: '%s' needs 1 argument: type\n", argv[i]);
        byebye(true);
      }
			i++;
      set_point_data_format = atoi(argv[i]);
      change_header = true;
    }
    else if (strcmp(argv[i],"-set_point_data_record_length") == 0)
    {
      lasreadopener.set_merged(FALSE);
      if ((i+1) >= argc)
      {
        fprintf(stderr,"ERROR: '%s' needs 1 argument: size\n", argv[i]);
        byebye(true);
      }
			i++;
      set_point_data_record_length = atoi(argv[i]);
      change_header = true;
    }
    else if (strcmp(argv[i],"-set_start_of_waveform_data_packet_record") == 0)
    {
      lasreadopener.set_merged(FALSE);
      if ((i+1) >= argc)
      {
        fprintf(stderr,"ERROR: '%s' needs 1 argument: start\n", argv[i]);
        byebye(true);
      }
			i++;
      set_start_of_waveform_data_packet_record = atoi(argv[i]);
      change_header = true;
    }
    else if (strcmp(argv[i],"-progress") == 0)
    {
      if ((i+1) >= argc)
      {
        fprintf(stderr,"ERROR: '%s' needs 1 argument: every\n", argv[i]);
        byebye(true);
      }
			i++;
      progress = atoi(argv[i]);
    }
    else if ((argv[i][0] != '-') && (lasreadopener.get_file_name_number() == 0))
    {
      lasreadopener.add_file_name(argv[i]);
      argv[i][0] = '\0';
    }
    else
    {
      fprintf(stderr, "ERROR: cannot understand argument '%s'\n", argv[i]);
      byebye(true);
    }
  }

#ifdef COMPILE_WITH_GUI
  if (gui)
  {
    return lasinfo_gui(argc, argv, &lasreadopener);
  }
#endif

  if (!lasreadopener.active())
  {
    fprintf (stderr, "ERROR: no input specified\n");
    byebye(true, argc==1);
  }

  // possibly loop over multiple input files

  while (lasreadopener.active())
  {
    // open lasreader

    LASreader* lasreader = lasreadopener.open();
    if (lasreader == 0)
    {
      fprintf(stderr, "ERROR: cannot open lasreader\n");
      byebye(true, argc==1);
    }

    LASheader* lasheader = &lasreader->header;

#ifdef _WIN32
    if (verbose) fprintf(stderr, "%s '%s' with %I64d points\n", (repair_bb || repair_counters ? "repairing" : "reading"), (lasreadopener.get_file_name() ? lasreadopener.get_file_name() : "stdin"), lasreader->npoints);
#else
    if (verbose) fprintf(stderr, "%s '%s' with %lld points\n", (repair_bb || repair_counters ? "repairing" : "reading"), (lasreadopener.get_file_name() ? lasreadopener.get_file_name() : "stdin"), lasreader->npoints);
#endif

    if (auto_date_creation && lasreadopener.get_file_name())
    {
#ifdef _WIN32
      WIN32_FILE_ATTRIBUTE_DATA attr;
	    SYSTEMTIME creation;
      GetFileAttributesEx(lasreadopener.get_file_name(), GetFileExInfoStandard, &attr);
	    FileTimeToSystemTime(&attr.ftCreationTime, &creation);
      int startday[13] = {-1, 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
      set_creation_day = startday[creation.wMonth] + creation.wDay;
      set_creation_year = creation.wYear;
      // leap year handling
      if ((((creation.wYear)%4) == 0) && (creation.wMonth > 2)) set_creation_day++;
      change_header = true;
#endif
    }

    if (file_name_out)
    {
      file_out = fopen(file_name_out, "w");
      if (file_out == 0)
      {
        fprintf (stderr, "WARNING: could not open output text file '%s'\n", file_name_out);
        file_out = stderr;
      }
    }
    else if (otxt && lasreadopener.get_file_name())
    {
      int len = strlen(lasreadopener.get_file_name());
      char* file_name_out = (char*)malloc(len+10);
      strcpy(file_name_out, lasreadopener.get_file_name());
      file_name_out[len-4] = '_';
      file_name_out[len-3] = 'i';
      file_name_out[len-2] = 'n';
      file_name_out[len-1] = 'f';
      file_name_out[len] = 'o';
      file_name_out[len+1] = '.';
      file_name_out[len+2] = 't';
      file_name_out[len+3] = 'x';
      file_name_out[len+4] = 't';
      file_name_out[len+5] = '\0';
      file_out = fopen(file_name_out, "w");
      if (file_out == 0)
      {
        fprintf (stderr, "WARNING: could not open output text file '%s'\n", file_name_out);
        file_out = stderr;
      }
      free(file_name_out);
    }

    U32 number_of_point_records = lasheader->number_of_point_records;
    U32 number_of_points_by_return0 = lasheader->number_of_points_by_return[0];

    // print header info

    char printstring[4096];

    if (file_out && !no_header)
    {
#ifdef _WIN32
      if (lasreader->npoints > number_of_point_records) fprintf(file_out, "extended LAS file has %I64d points, more than 32 bits can count.\012", lasreader->npoints);
#else
      if (lasreader->npoints > number_of_point_records) fprintf(file_out, "extended LAS file has %lld points, more than 32 bits can count.\012", lasreader->npoints);
#endif
      fprintf(file_out, "reporting all LAS header entries:\012");
      fprintf(file_out, "  file signature:             '%.4s'\012", lasheader->file_signature);
      fprintf(file_out, "  file source ID:             %d\012", lasheader->file_source_id);
      fprintf(file_out, "  global_encoding:            %d\012", lasheader->global_encoding);
      fprintf(file_out, "  project ID GUID data 1-4:   %u %d %d '%.8s'\012", lasheader->project_ID_GUID_data_1, lasheader->project_ID_GUID_data_2, lasheader->project_ID_GUID_data_3, lasheader->project_ID_GUID_data_4);
      fprintf(file_out, "  version major.minor:        %d.%d\012", lasheader->version_major, lasheader->version_minor);
      fprintf(file_out, "  system identifier:          '%.32s'\012", lasheader->system_identifier);
      fprintf(file_out, "  generating software:        '%.32s'\012", lasheader->generating_software);
      fprintf(file_out, "  file creation day/year:     %d/%d\012", lasheader->file_creation_day, lasheader->file_creation_year);
      fprintf(file_out, "  header size:                %d\012", lasheader->header_size);
      fprintf(file_out, "  offset to point data:       %u\012", lasheader->offset_to_point_data);
      fprintf(file_out, "  number var. length records: %u\012", lasheader->number_of_variable_length_records);
      fprintf(file_out, "  point data format:          %d\012", lasheader->point_data_format);
      fprintf(file_out, "  point data record length:   %d\012", lasheader->point_data_record_length);
      fprintf(file_out, "  number of point records:    %u\012", lasheader->number_of_point_records);
      fprintf(file_out, "  number of points by return: %u %u %u %u %u\012", lasheader->number_of_points_by_return[0], lasheader->number_of_points_by_return[1], lasheader->number_of_points_by_return[2], lasheader->number_of_points_by_return[3], lasheader->number_of_points_by_return[4]);
      fprintf(file_out, "  scale factor x y z:         %g %g %g\012", lasheader->x_scale_factor, lasheader->y_scale_factor, lasheader->z_scale_factor);
      fprintf(file_out, "  offset x y z:               "); lidardouble2string(printstring, lasheader->x_offset); fprintf(file_out, "%s ", printstring);  lidardouble2string(printstring, lasheader->y_offset); fprintf(file_out, "%s ", printstring);  lidardouble2string(printstring, lasheader->z_offset); fprintf(file_out, "%s\012", printstring);
      fprintf(file_out, "  min x y z:                  "); lidardouble2string(printstring, lasheader->min_x, lasheader->x_scale_factor); fprintf(file_out, "%s ", printstring); lidardouble2string(printstring, lasheader->min_y, lasheader->y_scale_factor); fprintf(file_out, "%s ", printstring); lidardouble2string(printstring, lasheader->min_z, lasheader->z_scale_factor); fprintf(file_out, "%s\012", printstring);
      fprintf(file_out, "  max x y z:                  "); lidardouble2string(printstring, lasheader->max_x, lasheader->x_scale_factor); fprintf(file_out, "%s ", printstring); lidardouble2string(printstring, lasheader->max_y, lasheader->y_scale_factor); fprintf(file_out, "%s ", printstring); lidardouble2string(printstring, lasheader->max_z, lasheader->z_scale_factor); fprintf(file_out, "%s\012", printstring);
      if ((lasheader->version_major == 1) && (lasheader->version_minor >= 3))
      {
#ifdef _WIN32
        fprintf(file_out, "  start of waveform data packet record: %I64d\012", lasheader->start_of_waveform_data_packet_record);
#else
        fprintf(file_out, "  start of waveform data packet record: %lld\012", lasheader->start_of_waveform_data_packet_record);
#endif
      }
      if ((lasheader->version_major == 1) && (lasheader->version_minor >= 4))
      {
#ifdef _WIN32
        fprintf(file_out, "  start of first extended variable length record: %I64d\012", lasheader->start_of_first_extended_variable_length_record);
#else
        fprintf(file_out, "  start of first extended variable length record: %lld\012", lasheader->start_of_first_extended_variable_length_record);
#endif
        fprintf(file_out, "  number of extended_variable length records: %d\012", lasheader->number_of_extended_variable_length_records);
#ifdef _WIN32
        fprintf(file_out, "  extended number of point records: %I64d\012", lasheader->extended_number_of_point_records);
        fprintf(file_out, "  extended number of points by return: %I64d %I64d %I64d %I64d %I64d %I64d %I64d %I64d %I64d %I64d %I64d %I64d %I64d %I64d %I64d\012", lasheader->extended_number_of_points_by_return[0], lasheader->extended_number_of_points_by_return[1], lasheader->extended_number_of_points_by_return[2], lasheader->extended_number_of_points_by_return[3], lasheader->extended_number_of_points_by_return[4], lasheader->extended_number_of_points_by_return[5], lasheader->extended_number_of_points_by_return[6], lasheader->extended_number_of_points_by_return[7], lasheader->extended_number_of_points_by_return[8], lasheader->extended_number_of_points_by_return[9], lasheader->extended_number_of_points_by_return[10], lasheader->extended_number_of_points_by_return[11], lasheader->extended_number_of_points_by_return[12], lasheader->extended_number_of_points_by_return[13], lasheader->extended_number_of_points_by_return[14]);
#else
        fprintf(file_out, "  extended number of point records: %lld\012", lasheader->extended_number_of_point_records);
        fprintf(file_out, "  extended number of points by return: %lld %lld %lld %lld %lld %lld %lld %lld %lld %lld %lld %lld %lld %lld %lld\012", lasheader->extended_number_of_points_by_return[0], lasheader->extended_number_of_points_by_return[1], lasheader->extended_number_of_points_by_return[2], lasheader->extended_number_of_points_by_return[3], lasheader->extended_number_of_points_by_return[4], lasheader->extended_number_of_points_by_return[5], lasheader->extended_number_of_points_by_return[6], lasheader->extended_number_of_points_by_return[7], lasheader->extended_number_of_points_by_return[8], lasheader->extended_number_of_points_by_return[9], lasheader->extended_number_of_points_by_return[10], lasheader->extended_number_of_points_by_return[11], lasheader->extended_number_of_points_by_return[12], lasheader->extended_number_of_points_by_return[13], lasheader->extended_number_of_points_by_return[14]);
#endif
      }
      if (lasheader->user_data_in_header_size) fprintf(file_out, "the header contains %u user-defined bytes\012", lasheader->user_data_in_header_size);
    }

    // maybe print variable header

    if (file_out && !no_variable_header)
    {
      for (int i = 0; i < (int)lasheader->number_of_variable_length_records; i++)
      {
        fprintf(file_out, "variable length header record %d of %d:\012", i+1, (int)lasheader->number_of_variable_length_records);
        fprintf(file_out, "  reserved             %d\012", lasreader->header.vlrs[i].reserved);
        fprintf(file_out, "  user ID              '%s'\012", lasreader->header.vlrs[i].user_id);
        fprintf(file_out, "  record ID            %d\012", lasreader->header.vlrs[i].record_id);
        fprintf(file_out, "  length after header  %d\012", lasreader->header.vlrs[i].record_length_after_header);
        fprintf(file_out, "  description          '%s'\012", lasreader->header.vlrs[i].description);

        // special handling for known variable header tags

        if ((strcmp(lasheader->vlrs[i].user_id, "LASF_Projection") == 0) && (lasheader->vlrs[i].data != 0))
        {
          if (lasheader->vlrs[i].record_id == 34735) // GeoKeyDirectoryTag
          {
            fprintf(file_out, "    GeoKeyDirectoryTag version %d.%d.%d number of keys %d\012", lasheader->vlr_geo_keys->key_directory_version, lasheader->vlr_geo_keys->key_revision, lasheader->vlr_geo_keys->minor_revision, lasheader->vlr_geo_keys->number_of_keys);
            for (int j = 0; j < lasheader->vlr_geo_keys->number_of_keys; j++)
            {
              if (file_out)
              {
                fprintf(file_out, "      key %d tiff_tag_location %d count %d value_offset %d - ", lasheader->vlr_geo_key_entries[j].key_id, lasheader->vlr_geo_key_entries[j].tiff_tag_location, lasheader->vlr_geo_key_entries[j].count, lasheader->vlr_geo_key_entries[j].value_offset);
                switch (lasreader->header.vlr_geo_key_entries[j].key_id)
                {
                case 1024: // GTModelTypeGeoKey 
                  switch (lasreader->header.vlr_geo_key_entries[j].value_offset)
                  {
                  case 1: // ModelTypeProjected   
                    fprintf(file_out, "GTModelTypeGeoKey: ModelTypeProjected\012");
                    break;
                  case 2:
                    fprintf(file_out, "GTModelTypeGeoKey: ModelTypeGeographic\012");
                    break;
                  case 3:
                    fprintf(file_out, "GTModelTypeGeoKey: ModelTypeGeocentric\012");
                    break;
                  default:
                    fprintf(file_out, "GTModelTypeGeoKey: look-up for %d not implemented\012", lasreader->header.vlr_geo_key_entries[j].value_offset);
                 }
                  break;
                case 1025: // GTRasterTypeGeoKey 
                  switch (lasreader->header.vlr_geo_key_entries[j].value_offset)
                  {
                  case 1: // RasterPixelIsArea   
                    fprintf(file_out, "GTRasterTypeGeoKey: RasterPixelIsArea\012");
                    break;
                  case 2: // RasterPixelIsPoint
                    fprintf(file_out, "GTRasterTypeGeoKey: RasterPixelIsPoint\012");
                    break;
                 default:
                    fprintf(file_out, "GTRasterTypeGeoKey: look-up for %d not implemented\012", lasreader->header.vlr_geo_key_entries[j].value_offset);
                  }
                  break;
                case 1026: // GTCitationGeoKey
                  if (lasreader->header.vlr_geo_ascii_params)
                  {
                    char dummy[256];
                    strncpy(dummy, &(lasreader->header.vlr_geo_ascii_params[lasreader->header.vlr_geo_key_entries[j].value_offset]), lasreader->header.vlr_geo_key_entries[j].count);
                    dummy[lasreader->header.vlr_geo_key_entries[j].count-1] = '\0';
                    fprintf(file_out, "GTCitationGeoKey: %s\012",dummy);
                  }
                  break;
                case 2048: // GeographicTypeGeoKey 
                  switch (lasreader->header.vlr_geo_key_entries[j].value_offset)
                  {
                  case 32767: // user-defined
                    fprintf(file_out, "GeographicTypeGeoKey: user-defined\012");
                    break;
                  case 4001: // GCSE_Airy1830
                    fprintf(file_out, "GeographicTypeGeoKey: GCSE_Airy1830\012");
                    break;
                  case 4002: // GCSE_AiryModified1849 
                    fprintf(file_out, "GeographicTypeGeoKey: GCSE_AiryModified1849\012");
                    break;
                  case 4003: // GCSE_AustralianNationalSpheroid
                    fprintf(file_out, "GeographicTypeGeoKey: GCSE_AustralianNationalSpheroid\012");
                    break;
                  case 4004: // GCSE_Bessel1841
                    fprintf(file_out, "GeographicTypeGeoKey: GCSE_Bessel1841\012");
                    break;
                  case 4005: // GCSE_Bessel1841Modified
                    fprintf(file_out, "GeographicTypeGeoKey: GCSE_Bessel1841Modified\012");
                    break;
                  case 4006: // GCSE_BesselNamibia
                    fprintf(file_out, "GeographicTypeGeoKey: GCSE_BesselNamibia\012");
                    break;
                  case 4008: // GCSE_Clarke1866
                    fprintf(file_out, "GeographicTypeGeoKey: GCSE_Clarke1866\012");
                    break;
                  case 4009: // GCSE_Clarke1866Michigan
                    fprintf(file_out, "GeographicTypeGeoKey: GCSE_Clarke1866Michigan\012");
                    break;
                  case 4010: // GCSE_Clarke1880_Benoit
                    fprintf(file_out, "GeographicTypeGeoKey: GCSE_Clarke1880_Benoit\012");
                    break;
                  case 4011: // GCSE_Clarke1880_IGN
                    fprintf(file_out, "GeographicTypeGeoKey: GCSE_Clarke1880_IGN\012");
                    break;
                  case 4012: // GCSE_Clarke1880_RGS
                    fprintf(file_out, "GeographicTypeGeoKey: GCSE_Clarke1880_RGS\012");
                    break;
                  case 4013: // GCSE_Clarke1880_Arc
                    fprintf(file_out, "GeographicTypeGeoKey: GCSE_Clarke1880_Arc\012");
                    break;
                  case 4014: // GCSE_Clarke1880_SGA1922
                    fprintf(file_out, "GeographicTypeGeoKey: GCSE_Clarke1880_SGA1922\012");
                    break;
                  case 4015: // GCSE_Everest1830_1937Adjustment
                    fprintf(file_out, "GeographicTypeGeoKey: GCSE_Everest1830_1937Adjustment\012");
                    break;
                  case 4016: // GCSE_Everest1830_1967Definition
                    fprintf(file_out, "GeographicTypeGeoKey: GCSE_Everest1830_1967Definition\012");
                    break;
                  case 4017: // GCSE_Everest1830_1975Definition
                    fprintf(file_out, "GeographicTypeGeoKey: GCSE_Everest1830_1975Definition\012");
                    break;
                  case 4018: // GCSE_Everest1830Modified
                    fprintf(file_out, "GeographicTypeGeoKey: GCSE_Everest1830Modified\012");
                    break;
                  case 4019: // GCSE_GRS1980
                    fprintf(file_out, "GeographicTypeGeoKey: GCSE_GRS1980\012");
                    break;
                  case 4020: // GCSE_Helmert1906
                    fprintf(file_out, "GeographicTypeGeoKey: GCSE_Helmert1906\012");
                    break;
                  case 4022: // GCSE_International1924
                    fprintf(file_out, "GeographicTypeGeoKey: GCSE_International1924\012");
                    break;
                  case 4023: // GCSE_International1967
                    fprintf(file_out, "GeographicTypeGeoKey: GCSE_International1967\012");
                    break;
                  case 4024: // GCSE_Krassowsky1940
                    fprintf(file_out, "GeographicTypeGeoKey: GCSE_Krassowsky1940\012");
                    break;
                  case 4030: // GCSE_WGS84
                    fprintf(file_out, "GeographicTypeGeoKey: GCSE_WGS84\012");
                    break;
                  case 4034: // GCSE_Clarke1880
                    fprintf(file_out, "GeographicTypeGeoKey: GCSE_Clarke1880\012");
                    break;
                  case 4267: // GCS_NAD27
                    fprintf(file_out, "GeographicTypeGeoKey: GCS_NAD27\012");
                    break;
                  case 4269: // GCS_NAD83
                    fprintf(file_out, "GeographicTypeGeoKey: GCS_NAD83\012");
                    break;
                  case 4322: // GCS_WGS_72
                    fprintf(file_out, "GeographicTypeGeoKey: GCS_WGS_72\012");
                    break;
                  case 4326: // GCS_WGS_84
                    fprintf(file_out, "GeographicTypeGeoKey: GCS_WGS_84\012");
                    break;
                  case 4289: // GCS_Amersfoort
                    fprintf(file_out, "GeographicTypeGeoKey: GCS_Amersfoort\012");
                    break;
                  default:
                    fprintf(file_out, "GeographicTypeGeoKey: look-up for %d not implemented\012", lasreader->header.vlr_geo_key_entries[j].value_offset);
                  }
                  break;
                case 2049: // GeogCitationGeoKey
                  if (lasreader->header.vlr_geo_ascii_params)
                  {
                    char dummy[256];
                    strncpy(dummy, &(lasreader->header.vlr_geo_ascii_params[lasreader->header.vlr_geo_key_entries[j].value_offset]), lasreader->header.vlr_geo_key_entries[j].count);
                    dummy[lasreader->header.vlr_geo_key_entries[j].count-1] = '\0';
                    fprintf(file_out, "GeogCitationGeoKey: %s\012",dummy);
                  }
                  break;
                case 2050: // GeogGeodeticDatumGeoKey 
                  switch (lasreader->header.vlr_geo_key_entries[j].value_offset)
                  {
                  case 32767: // user-defined
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: user-defined\012");
                    break;
                  case 6202: // Datum_Australian_Geodetic_Datum_1966
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: Datum_Australian_Geodetic_Datum_1966\012");
                    break;
                  case 6203: // Datum_Australian_Geodetic_Datum_1984
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: Datum_Australian_Geodetic_Datum_1984\012");
                    break;
                  case 6267: // Datum_North_American_Datum_1927
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: Datum_North_American_Datum_1927\012");
                    break;
                  case 6269: // Datum_North_American_Datum_1983
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: Datum_North_American_Datum_1983\012");
                    break;
                  case 6322: // Datum_WGS72
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: Datum_WGS72\012");
                    break;
                  case 6326: // Datum_WGS84
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: Datum_WGS84\012");
                    break;
                  case 6001: // DatumE_Airy1830
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: DatumE_Airy1830\012");
                    break;
                  case 6002: // DatumE_AiryModified1849
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: DatumE_AiryModified1849\012");
                    break;
                  case 6003: // DatumE_AustralianNationalSpheroid
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: DatumE_AustralianNationalSpheroid\012");
                    break;
                  case 6004: // DatumE_Bessel1841
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: DatumE_Bessel1841\012");
                    break;
                  case 6005: // DatumE_BesselModified
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: DatumE_BesselModified\012");
                    break;
                  case 6006: // DatumE_BesselNamibia
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: DatumE_BesselNamibia\012");
                    break;
                  case 6008: // DatumE_Clarke1866
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: DatumE_Clarke1866\012");
                    break;
                  case 6009: // DatumE_Clarke1866Michigan
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: DatumE_Clarke1866Michigan\012");
                    break;
                  case 6010: // DatumE_Clarke1880_Benoit
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: DatumE_Clarke1880_Benoit\012");
                    break;
                  case 6011: // DatumE_Clarke1880_IGN
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: DatumE_Clarke1880_IGN\012");
                    break;
                  case 6012: // DatumE_Clarke1880_RGS
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: DatumE_Clarke1880_RGS\012");
                    break;
                  case 6013: // DatumE_Clarke1880_Arc
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: DatumE_Clarke1880_Arc\012");
                    break;
                  case 6014: // DatumE_Clarke1880_SGA1922
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: DatumE_Clarke1880_SGA1922\012");
                    break;
                  case 6015: // DatumE_Everest1830_1937Adjustment
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: DatumE_Everest1830_1937Adjustment\012");
                    break;
                  case 6016: // DatumE_Everest1830_1967Definition
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: DatumE_Everest1830_1967Definition\012");
                    break;
                  case 6017: // DatumE_Everest1830_1975Definition
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: DatumE_Everest1830_1975Definition\012");
                    break;
                  case 6018: // DatumE_Everest1830Modified
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: DatumE_Everest1830Modified\012");
                    break;
                  case 6019: // DatumE_GRS1980
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: DatumE_GRS1980\012");
                    break;
                  case 6020: // DatumE_Helmert1906
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: DatumE_Helmert1906\012");
                    break;
                  case 6022: // DatumE_International1924
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: DatumE_International1924\012");
                    break;
                  case 6023: // DatumE_International1967
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: DatumE_International1967\012");
                    break;
                  case 6024: // DatumE_Krassowsky1940
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: DatumE_Krassowsky1940\012");
                    break;
                  case 6030: // DatumE_WGS84
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: DatumE_WGS84\012");
                    break;
                  case 6034: // DatumE_Clarke1880
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: DatumE_Clarke1880\012");
                    break;
                  case 6289: // Datum_Amersfoort
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: Datum_Amersfoort\012");
                    break;
                  default:
                    fprintf(file_out, "GeogGeodeticDatumGeoKey: look-up for %d not implemented\012", lasreader->header.vlr_geo_key_entries[j].value_offset);
                  }
                  break;
                case 2051: // GeogPrimeMeridianGeoKey
                  switch (lasreader->header.vlr_geo_key_entries[j].value_offset)
                  {
                  case 8901: // PM_Greenwich
                    fprintf(file_out, "GeogPrimeMeridianGeoKey: PM_Greenwich\012");
                    break;
                  case 8902: // PM_Lisbon
                    fprintf(file_out, "GeogPrimeMeridianGeoKey: PM_Lisbon\012");
                    break;
                  default:
                    fprintf(file_out, "GeogPrimeMeridianGeoKey: look-up for %d not implemented\012", lasreader->header.vlr_geo_key_entries[j].value_offset);
                  }
                  break;
                case 2052: // GeogLinearUnitsGeoKey 
                  horizontal_units = lasreader->header.vlr_geo_key_entries[j].value_offset;
                  switch (lasreader->header.vlr_geo_key_entries[j].value_offset)
                  {
                  case 9001: // Linear_Meter
                    fprintf(file_out, "GeogLinearUnitsGeoKey: Linear_Meter\012");
                    break;
                  case 9002: // Linear_Foot
                    fprintf(file_out, "GeogLinearUnitsGeoKey: Linear_Foot\012");
                    break;
                  case 9003: // Linear_Foot_US_Survey
                    fprintf(file_out, "GeogLinearUnitsGeoKey: Linear_Foot_US_Survey\012");
                    break;
                  case 9004: // Linear_Foot_Modified_American
                    fprintf(file_out, "GeogLinearUnitsGeoKey: Linear_Foot_Modified_American\012");
                    break;
                  case 9005: // Linear_Foot_Clarke
                    fprintf(file_out, "GeogLinearUnitsGeoKey: Linear_Foot_Clarke\012");
                    break;
                  case 9006: // Linear_Foot_Indian
                    fprintf(file_out, "GeogLinearUnitsGeoKey: Linear_Foot_Indian\012");
                    break;
                  case 9007: // Linear_Link
                    fprintf(file_out, "GeogLinearUnitsGeoKey: Linear_Link\012");
                    break;
                  case 9008: // Linear_Link_Benoit
                    fprintf(file_out, "GeogLinearUnitsGeoKey: Linear_Link_Benoit\012");
                    break;
                  case 9009: // Linear_Link_Sears
                    fprintf(file_out, "GeogLinearUnitsGeoKey: Linear_Link_Sears\012");
                    break;
                  case 9010: // Linear_Chain_Benoit
                    fprintf(file_out, "GeogLinearUnitsGeoKey: Linear_Chain_Benoit\012");
                    break;
                  case 9011: // Linear_Chain_Sears
                    fprintf(file_out, "GeogLinearUnitsGeoKey: Linear_Chain_Sears\012");
                    break;
                  case 9012: // Linear_Yard_Sears
                    fprintf(file_out, "GeogLinearUnitsGeoKey: Linear_Yard_Sears\012");
                    break;
                  case 9013: // Linear_Yard_Indian
                    fprintf(file_out, "GeogLinearUnitsGeoKey: Linear_Yard_Indian\012");
                    break;
                  case 9014: // Linear_Fathom
                    fprintf(file_out, "GeogLinearUnitsGeoKey: Linear_Fathom\012");
                    break;
                  case 9015: // Linear_Mile_International_Nautical
                    fprintf(file_out, "GeogLinearUnitsGeoKey: Linear_Mile_International_Nautical\012");
                    break;
                  default:
                    fprintf(file_out, "GeogLinearUnitsGeoKey: look-up for %d not implemented\012", lasreader->header.vlr_geo_key_entries[j].value_offset);
                  }
                  break;
                case 2053: // GeogLinearUnitSizeGeoKey  
                  if (lasreader->header.vlr_geo_double_params)
                  {
                    fprintf(file_out, "GeogLinearUnitSizeGeoKey: %.10g\012",lasreader->header.vlr_geo_double_params[lasreader->header.vlr_geo_key_entries[j].value_offset]);
                  }
                  break;
                case 2054: // GeogAngularUnitsGeoKey
                  switch (lasreader->header.vlr_geo_key_entries[j].value_offset)
                  {
                  case 9101: // Angular_Radian
                    fprintf(file_out, "GeogAngularUnitsGeoKey: Angular_Radian\012");
                    break;
                  case 9102: // Angular_Degree
                    fprintf(file_out, "GeogAngularUnitsGeoKey: Angular_Degree\012");
                    break;
                  case 9103: // Angular_Arc_Minute
                    fprintf(file_out, "GeogAngularUnitsGeoKey: Angular_Arc_Minute\012");
                    break;
                  case 9104: // Angular_Arc_Second
                    fprintf(file_out, "GeogAngularUnitsGeoKey: Angular_Arc_Second\012");
                    break;
                  case 9105: // Angular_Grad
                    fprintf(file_out, "GeogAngularUnitsGeoKey: Angular_Grad\012");
                    break;
                  case 9106: // Angular_Gon
                    fprintf(file_out, "GeogAngularUnitsGeoKey: Angular_Gon\012");
                    break;
                  case 9107: // Angular_DMS
                    fprintf(file_out, "GeogAngularUnitsGeoKey: Angular_DMS\012");
                    break;
                  case 9108: // Angular_DMS_Hemisphere
                    fprintf(file_out, "GeogAngularUnitsGeoKey: Angular_DMS_Hemisphere\012");
                    break;
                  default:
                    fprintf(file_out, "GeogAngularUnitsGeoKey: look-up for %d not implemented\012", lasreader->header.vlr_geo_key_entries[j].value_offset);
                  }
                  break;
                case 2055: // GeogAngularUnitSizeGeoKey 
                  if (lasreader->header.vlr_geo_double_params)
                  {
                    fprintf(file_out, "GeogAngularUnitSizeGeoKey: %.10g\012",lasreader->header.vlr_geo_double_params[lasreader->header.vlr_geo_key_entries[j].value_offset]);
                  }
                  break;
                case 2056: // GeogEllipsoidGeoKey
                  switch (lasreader->header.vlr_geo_key_entries[j].value_offset)
                  {
                  case 7001: // Ellipse_Airy_1830
                    fprintf(file_out, "GeogEllipsoidGeoKey: Ellipse_Airy_1830\012");
                    break;
                  case 7002: // Ellipse_Airy_Modified_1849
                    fprintf(file_out, "GeogEllipsoidGeoKey: Ellipse_Airy_Modified_1849\012");
                    break;
                  case 7003: // Ellipse_Australian_National_Spheroid
                    fprintf(file_out, "GeogEllipsoidGeoKey: Ellipse_Australian_National_Spheroid\012");
                    break;
                  case 7004: // Ellipse_Bessel_1841
                    fprintf(file_out, "GeogEllipsoidGeoKey: Ellipse_Bessel_1841\012");
                    break;
                  case 7005: // Ellipse_Bessel_Modified
                    fprintf(file_out, "GeogEllipsoidGeoKey: Ellipse_Bessel_Modified\012");
                    break;
                  case 7006: // Ellipse_Bessel_Namibia
                    fprintf(file_out, "GeogEllipsoidGeoKey: Ellipse_Bessel_Namibia\012");
                    break;
                  case 7008: // Ellipse_Clarke_1866
                    fprintf(file_out, "GeogEllipsoidGeoKey: Ellipse_Clarke_1866\012");
                    break;
                  case 7009: // Ellipse_Clarke_1866_Michigan
                    fprintf(file_out, "GeogEllipsoidGeoKey: Ellipse_Clarke_1866_Michigan\012");
                    break;
                  case 7010: // Ellipse_Clarke1880_Benoit
                    fprintf(file_out, "GeogEllipsoidGeoKey: Ellipse_Clarke1880_Benoit\012");
                    break;
                  case 7011: // Ellipse_Clarke1880_IGN
                    fprintf(file_out, "GeogEllipsoidGeoKey: Ellipse_Clarke1880_IGN\012");
                    break;
                  case 7012: // Ellipse_Clarke1880_RGS
                    fprintf(file_out, "GeogEllipsoidGeoKey: Ellipse_Clarke1880_RGS\012");
                    break;
                  case 7013: // Ellipse_Clarke1880_Arc
                    fprintf(file_out, "GeogEllipsoidGeoKey: Ellipse_Clarke1880_Arc\012");
                    break;
                  case 7014: // Ellipse_Clarke1880_SGA1922
                    fprintf(file_out, "GeogEllipsoidGeoKey: Ellipse_Clarke1880_SGA1922\012");
                    break;
                  case 7015: // Ellipse_Everest1830_1937Adjustment
                    fprintf(file_out, "GeogEllipsoidGeoKey: Ellipse_Everest1830_1937Adjustment\012");
                    break;
                  case 7016: // Ellipse_Everest1830_1967Definition
                    fprintf(file_out, "GeogEllipsoidGeoKey: Ellipse_Everest1830_1967Definition\012");
                    break;
                  case 7017: // Ellipse_Everest1830_1975Definition
                    fprintf(file_out, "GeogEllipsoidGeoKey: Ellipse_Everest1830_1975Definition\012");
                    break;
                  case 7018: // Ellipse_Everest1830Modified
                    fprintf(file_out, "GeogEllipsoidGeoKey: Ellipse_Everest1830Modified\012");
                    break;
                  case 7019: // Ellipse_GRS_1980
                    fprintf(file_out, "GeogEllipsoidGeoKey: Ellipse_GRS_1980\012");
                    break;
                  case 7020: // Ellipse_Helmert1906
                    fprintf(file_out, "GeogEllipsoidGeoKey: Ellipse_Helmert1906\012");
                    break;
                  case 7022: // Ellipse_International1924
                    fprintf(file_out, "GeogEllipsoidGeoKey: Ellipse_International1924\012");
                    break;
                  case 7023: // Ellipse_International1967
                    fprintf(file_out, "GeogEllipsoidGeoKey: Ellipse_International1967\012");
                    break;
                  case 7024: // Ellipse_Krassowsky1940
                    fprintf(file_out, "GeogEllipsoidGeoKey: Ellipse_Krassowsky1940\012");
                    break;
                  case 7030: // Ellipse_WGS_84
                    fprintf(file_out, "GeogEllipsoidGeoKey: Ellipse_WGS_84\012");
                    break;
                  case 7034: // Ellipse_Clarke_1880
                    fprintf(file_out, "GeogEllipsoidGeoKey: Ellipse_Clarke_1880\012");
                    break;
                  default:
                    fprintf(file_out, "GeogEllipsoidGeoKey: look-up for %d not implemented\012", lasreader->header.vlr_geo_key_entries[j].value_offset);
                  }
                  break;
                case 2057: // GeogSemiMajorAxisGeoKey 
                  if (lasreader->header.vlr_geo_double_params)
                  {
                    fprintf(file_out, "GeogSemiMajorAxisGeoKey: %.10g\012",lasreader->header.vlr_geo_double_params[lasreader->header.vlr_geo_key_entries[j].value_offset]);
                  }
                  break;
                case 2058: // GeogSemiMinorAxisGeoKey 
                  if (lasreader->header.vlr_geo_double_params)
                  {
                    fprintf(file_out, "GeogSemiMinorAxisGeoKey: %.10g\012",lasreader->header.vlr_geo_double_params[lasreader->header.vlr_geo_key_entries[j].value_offset]);
                  }
                  break;
                case 2059: // GeogInvFlatteningGeoKey
                  if (lasreader->header.vlr_geo_double_params)
                  {
                    fprintf(file_out, "GeogInvFlatteningGeoKey: %.10g\012",lasreader->header.vlr_geo_double_params[lasreader->header.vlr_geo_key_entries[j].value_offset]);
                  }
                  break;
                case 2060: // GeogAzimuthUnitsGeoKey
                  switch (lasreader->header.vlr_geo_key_entries[j].value_offset)
                  {
                  case 9101: // Angular_Radian
                    fprintf(file_out, "GeogAzimuthUnitsGeoKey: Angular_Radian\012");
                    break;
                  case 9102: // Angular_Degree
                    fprintf(file_out, "GeogAzimuthUnitsGeoKey: Angular_Degree\012");
                    break;
                  case 9103: // Angular_Arc_Minute
                    fprintf(file_out, "GeogAzimuthUnitsGeoKey: Angular_Arc_Minute\012");
                    break;
                  case 9104: // Angular_Arc_Second
                    fprintf(file_out, "GeogAzimuthUnitsGeoKey: Angular_Arc_Second\012");
                    break;
                  case 9105: // Angular_Grad
                    fprintf(file_out, "GeogAzimuthUnitsGeoKey: Angular_Grad\012");
                    break;
                  case 9106: // Angular_Gon
                    fprintf(file_out, "GeogAzimuthUnitsGeoKey: Angular_Gon\012");
                    break;
                  case 9107: // Angular_DMS
                    fprintf(file_out, "GeogAzimuthUnitsGeoKey: Angular_DMS\012");
                    break;
                  case 9108: // Angular_DMS_Hemisphere
                    fprintf(file_out, "GeogAzimuthUnitsGeoKey: Angular_DMS_Hemisphere\012");
                    break;
                  default:
                    fprintf(file_out, "GeogAzimuthUnitsGeoKey: look-up for %d not implemented\012", lasreader->header.vlr_geo_key_entries[j].value_offset);
                  }
                  break;
                case 2061: // GeogPrimeMeridianLongGeoKey  
                  if (lasreader->header.vlr_geo_double_params)
                  {
                    fprintf(file_out, "GeogPrimeMeridianLongGeoKey: %.10g\012",lasreader->header.vlr_geo_double_params[lasreader->header.vlr_geo_key_entries[j].value_offset]);
                  }
                  break;
                case 3072: // ProjectedCSTypeGeoKey 
                  switch (lasreader->header.vlr_geo_key_entries[j].value_offset)
                  {
                  case 32767: // user-defined
                    fprintf(file_out, "ProjectedCSTypeGeoKey: user-defined\012");
                    break;
                  case 20137: // PCS_Adindan_UTM_zone_37N
                  case 20138: // PCS_Adindan_UTM_zone_38N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Adindan_UTM_zone_%dN\012", lasreader->header.vlr_geo_key_entries[j].value_offset-20100);
                    break;
                  case 20248: // PCS_AGD66_AMG_zone_48
                  case 20249:
                  case 20250:
                  case 20251:
                  case 20252:
                  case 20253:
                  case 20254:
                  case 20255:
                  case 20256:
                  case 20257:
                  case 20258: // PCS_AGD66_AMG_zone_58
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_AGD66_AMG_zone_%d\012", lasreader->header.vlr_geo_key_entries[j].value_offset-20200);
                    break;
                  case 20348: // PCS_AGD84_AMG_zone_48
                  case 20349:
                  case 20350:
                  case 20351:
                  case 20352:
                  case 20353:
                  case 20354:
                  case 20355:
                  case 20356:
                  case 20357:
                  case 20358: // PCS_AGD84_AMG_zone_58
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_AGD84_AMG_zone_%d\012", lasreader->header.vlr_geo_key_entries[j].value_offset-20300);
                    break;
                  case 20437: // PCS_Ain_el_Abd_UTM_zone_37N
                  case 20438: // PCS_Ain_el_Abd_UTM_zone_38N
                  case 20439: // PCS_Ain_el_Abd_UTM_zone_39N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Ain_el_Abd_UTM_zone_%dN\012", lasreader->header.vlr_geo_key_entries[j].value_offset-20400);
                    break;                
                  case 20538: // PCS_Afgooye_UTM_zone_38N
                  case 20539: // PCS_Afgooye_UTM_zone_39N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Afgooye_UTM_zone_%dN\012", lasreader->header.vlr_geo_key_entries[j].value_offset-20500);
                    break;
                  case 20822: // PCS_Aratu_UTM_zone_22S
                  case 20823: // PCS_Aratu_UTM_zone_23S
                  case 20824: // PCS_Aratu_UTM_zone_24S
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Aratu_UTM_zone_%dS\012", lasreader->header.vlr_geo_key_entries[j].value_offset-20800);
                    break;
                  case 21148: // PCS_Batavia_UTM_zone_48S
                  case 21149: // PCS_Batavia_UTM_zone_49S
                  case 21150: // PCS_Batavia_UTM_zone_50S
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Batavia_UTM_zone_%dS\012", lasreader->header.vlr_geo_key_entries[j].value_offset-21100);
                    break;
                  case 21817: // PCS_Bogota_UTM_zone_17N
                  case 21818: // PCS_Bogota_UTM_zone_18N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Bogota_UTM_zone_%dN\012", lasreader->header.vlr_geo_key_entries[j].value_offset-21800);
                    break;
                  case 22032: // PCS_Camacupa_UTM_32S
                  case 22033: // PCS_Camacupa_UTM_33S
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Camacupa_UTM_%dS\012", lasreader->header.vlr_geo_key_entries[j].value_offset-22000);
                    break; 
                  case 22332: // PCS_Carthage_UTM_zone_32N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Carthage_UTM_zone_32N\012");
                    break; 
                  case 22523: // PCS_Corrego_Alegre_UTM_23S
                  case 22524: // PCS_Corrego_Alegre_UTM_24S
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Corrego_Alegre_UTM_%dS\012", lasreader->header.vlr_geo_key_entries[j].value_offset-22500);
                    break;
                  case 22832: // PCS_Douala_UTM_zone_32N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Douala_UTM_zone_32N\012");
                    break;
                  case 23028: // PCS_ED50_UTM_zone_28N
                  case 23029: // PCS_ED50_UTM_zone_29N
                  case 23030: 
                  case 23031: 
                  case 23032: 
                  case 23033: 
                  case 23034: 
                  case 23035: 
                  case 23036: 
                  case 23037: 
                  case 23038: // PCS_ED50_UTM_zone_38N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_ED50_UTM_zone_%dN\012", lasreader->header.vlr_geo_key_entries[j].value_offset-23000);
                    break;
                  case 23239: // PCS_Fahud_UTM_zone_39N
                  case 23240: // PCS_Fahud_UTM_zone_40N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Fahud_UTM_zone_%dN\012", lasreader->header.vlr_geo_key_entries[j].value_offset-23200);
                    break;
                  case 23433: // PCS_Garoua_UTM_zone_33N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Garoua_UTM_zone_33N\012");
                    break;
                  case 23846: // PCS_ID74_UTM_zone_46N
                  case 23847: // PCS_ID74_UTM_zone_47N
                  case 23848:
                  case 23849:
                  case 23850:
                  case 23851:
                  case 23852:
                  case 23853: // PCS_ID74_UTM_zone_53N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_ID74_UTM_zone_%dN\012", lasreader->header.vlr_geo_key_entries[j].value_offset-23800);
                    break;
                  case 23886: // PCS_ID74_UTM_zone_46S
                  case 23887: // PCS_ID74_UTM_zone_47S
                  case 23888:
                  case 23889:
                  case 23890:
                  case 23891:
                  case 23892:
                  case 23893:
                  case 23894: // PCS_ID74_UTM_zone_54S
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_ID74_UTM_zone_%dS\012", lasreader->header.vlr_geo_key_entries[j].value_offset-23840);
                    break;
                  case 23947: // PCS_Indian_1954_UTM_47N
                  case 23948: // PCS_Indian_1954_UTM_48N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Indian_1954_UTM_%dN\012", lasreader->header.vlr_geo_key_entries[j].value_offset-23900);
                    break;
                  case 24047: // PCS_Indian_1975_UTM_47N
                  case 24048: // PCS_Indian_1975_UTM_48N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Indian_1975_UTM_%dN\012", lasreader->header.vlr_geo_key_entries[j].value_offset-24000);
                    break;
                  case 24547: // PCS_Kertau_UTM_zone_47N
                  case 24548: // PCS_Kertau_UTM_zone_48N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Kertau_UTM_zone_%dN\012", lasreader->header.vlr_geo_key_entries[j].value_offset-24500);
                    break;
                  case 24720: // PCS_La_Canoa_UTM_zone_20N
                  case 24721: // PCS_La_Canoa_UTM_zone_21N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_La_Canoa_UTM_zone_%dN\012", lasreader->header.vlr_geo_key_entries[j].value_offset-24700);
                    break;
                  case 24818: // PCS_PSAD56_UTM_zone_18N
                  case 24819: // PCS_PSAD56_UTM_zone_19N
                  case 24820: // PCS_PSAD56_UTM_zone_20N
                  case 24821: // PCS_PSAD56_UTM_zone_21N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_PSAD56_UTM_zone_%dN\012", lasreader->header.vlr_geo_key_entries[j].value_offset-24800);
                    break;
                  case 24877: // PCS_PSAD56_UTM_zone_17S
                  case 24878: // PCS_PSAD56_UTM_zone_18S
                  case 24879: // PCS_PSAD56_UTM_zone_19S
                  case 24880: // PCS_PSAD56_UTM_zone_20S
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_PSAD56_UTM_zone_%dS\012", lasreader->header.vlr_geo_key_entries[j].value_offset-24860);
                    break;
                  case 25231: // PCS_Lome_UTM_zone_31N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Lome_UTM_zone_31N\012");
                    break;
                  case 25828: // PCS_ETRS89_UTM_zone_28N
                  case 25829: // PCS_ETRS89_UTM_zone_29N
                  case 25830: // PCS_ETRS89_UTM_zone_30N
                  case 25831: // PCS_ETRS89_UTM_zone_31N
                  case 25832: // PCS_ETRS89_UTM_zone_32N
                  case 25833: // PCS_ETRS89_UTM_zone_33N
                  case 25834: // PCS_ETRS89_UTM_zone_34N
                  case 25835: // PCS_ETRS89_UTM_zone_35N
                  case 25836: // PCS_ETRS89_UTM_zone_36N
                  case 25837: // PCS_ETRS89_UTM_zone_37N
                  case 25838: // PCS_ETRS89_UTM_zone_38N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_ETRS89_UTM_zone_%dN\012", lasreader->header.vlr_geo_key_entries[j].value_offset-25800);
                    break;
                  case 25932: // PCS_Malongo_1987_UTM_32S
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Malongo_1987_UTM_32S\012");
                    break;
                  case 26237: // PCS_Massawa_UTM_zone_37N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Massawa_UTM_zone_37N\012");
                    break;
                  case 26331: // PCS_Minna_UTM_zone_31N
                  case 26332: // PCS_Minna_UTM_zone_32N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Minna_UTM_zone_%dN\012", lasreader->header.vlr_geo_key_entries[j].value_offset-26300);
                    break;
                  case 26432: // PCS_Mhast_UTM_zone_32S
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Mhast_UTM_zone_32S\012");
                    break;
                  case 26632: // PCS_M_poraloko_UTM_32N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_M_poraloko_UTM_32N\012");
                    break;
                  case 26692: // PCS_Minna_UTM_zone_32S
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Minna_UTM_zone_32S\012");
                    break;
                  case 26703: // PCS_NAD27_UTM_zone_3N
                  case 26704: // PCS_NAD27_UTM_zone_4N
                  case 26705: // PCS_NAD27_UTM_zone_5N
                  case 26706: // PCS_NAD27_UTM_zone_6N
                  case 26707: // PCS_NAD27_UTM_zone_7N
                  case 26708: // PCS_NAD27_UTM_zone_7N
                  case 26709: // PCS_NAD27_UTM_zone_9N
                  case 26710: // PCS_NAD27_UTM_zone_10N
                  case 26711: // PCS_NAD27_UTM_zone_11N
                  case 26712: // PCS_NAD27_UTM_zone_12N
                  case 26713: // PCS_NAD27_UTM_zone_13N
                  case 26714: // PCS_NAD27_UTM_zone_14N
                  case 26715: // PCS_NAD27_UTM_zone_15N
                  case 26716: // PCS_NAD27_UTM_zone_16N
                  case 26717: // PCS_NAD27_UTM_zone_17N
                  case 26718: // PCS_NAD27_UTM_zone_18N
                  case 26719: // PCS_NAD27_UTM_zone_19N
                  case 26720: // PCS_NAD27_UTM_zone_20N
                  case 26721: // PCS_NAD27_UTM_zone_21N
                  case 26722: // PCS_NAD27_UTM_zone_22N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_UTM_zone_%dN\012", lasreader->header.vlr_geo_key_entries[j].value_offset-26700);
                    break;
                  case 26729: // PCS_NAD27_Alabama_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Alabama_East\012");
                    break;
                  case 26730: // PCS_NAD27_Alabama_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Alabama_West\012");
                    break;
                  case 26731: // PCS_NAD27_Alaska_zone_1
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Alaska_zone_1\012");
                    break;
                  case 26732: // PCS_NAD27_Alaska_zone_2
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Alaska_zone_2\012");
                    break;
                  case 26733: // PCS_NAD27_Alaska_zone_3
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Alaska_zone_3\012");
                    break;
                  case 26734: // PCS_NAD27_Alaska_zone_4
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Alaska_zone_4\012");
                    break;
                  case 26735: // PCS_NAD27_Alaska_zone_5
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Alaska_zone_5\012");
                    break;
                  case 26736: // PCS_NAD27_Alaska_zone_6
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Alaska_zone_6\012");
                    break;
                  case 26737: // PCS_NAD27_Alaska_zone_7
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Alaska_zone_7\012");
                    break;
                  case 26738: // PCS_NAD27_Alaska_zone_8
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Alaska_zone_8\012");
                    break;
                  case 26739: // PCS_NAD27_Alaska_zone_9
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Alaska_zone_9\012");
                    break;
                  case 26740: // PCS_NAD27_Alaska_zone_10
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Alaska_zone_10\012");
                    break;
                  case 26741: // PCS_NAD27_California_I
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_California_I\012");
                    break;
                  case 26742: // PCS_NAD27_California_II
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_California_II\012");
                    break;
                  case 26743: // PCS_NAD27_California_III
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_California_III\012");
                    break;
                  case 26744: // PCS_NAD27_California_IV
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_California_IV\012");
                    break;
                  case 26745: // PCS_NAD27_California_V
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_California_V\012");
                    break;
                  case 26746: // PCS_NAD27_California_VI
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_California_VI\012");
                    break;
                  case 26747: // PCS_NAD27_California_VII
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_California_VII\012");
                    break;
                  case 26748: // PCS_NAD27_Arizona_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Arizona_East\012");
                    break;
                  case 26749: // PCS_NAD27_Arizona_Central
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Arizona_Central\012");
                    break;
                  case 26750: // PCS_NAD27_Arizona_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Arizona_West\012");
                    break;
                  case 26751: // PCS_NAD27_Arkansas_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Arkansas_North\012");
                    break;
                  case 26752: // PCS_NAD27_Arkansas_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Arkansas_South\012");
                    break;
                  case 26753: // PCS_NAD27_Colorado_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Colorado_North\012");
                    break;
                  case 26754: // PCS_NAD27_Colorado_Central
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Colorado_Central\012");
                    break;
                  case 26755: // PCS_NAD27_Colorado_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Colorado_South\012");
                    break;
                  case 26756: // PCS_NAD27_Connecticut
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Connecticut\012");
                    break;
                  case 26757: // PCS_NAD27_Delaware
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Delaware\012");
                    break;
                  case 26758: // PCS_NAD27_Florida_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Florida_East\012");
                    break;
                  case 26759: // PCS_NAD27_Florida_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Florida_West\012");
                    break;
                  case 26760: // PCS_NAD27_Florida_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Florida_North\012");
                    break;
                  case 26761: // PCS_NAD27_Hawaii_zone_1
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Hawaii_zone_1\012");
                    break;
                  case 26762: // PCS_NAD27_Hawaii_zone_2
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Hawaii_zone_2\012");
                    break;
                  case 26763: // PCS_NAD27_Hawaii_zone_3
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Hawaii_zone_3\012");
                    break;
                  case 26764: // PCS_NAD27_Hawaii_zone_4
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Hawaii_zone_4\012");
                    break;
                  case 26765: // PCS_NAD27_Hawaii_zone_5
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Hawaii_zone_5\012");
                    break;
                  case 26766: // PCS_NAD27_Georgia_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Georgia_East\012");
                    break;
                  case 26767: // PCS_NAD27_Georgia_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Georgia_West\012");
                    break;
                  case 26768: // PCS_NAD27_Idaho_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Idaho_East\012");
                    break;
                  case 26769: // PCS_NAD27_Idaho_Central
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Idaho_Central\012");
                    break;
                  case 26770: // PCS_NAD27_Idaho_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Idaho_West\012");
                    break;
                  case 26771: // PCS_NAD27_Illinois_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Illinois_East\012");
                    break;
                  case 26772: // PCS_NAD27_Illinois_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Illinois_West\012");
                    break;
                  case 26773: // PCS_NAD27_Indiana_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Indiana_East\012");
                    break;
                  case 26774: // PCS_NAD27_Indiana_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Indiana_West\012");
                    break;
                  case 26775: // PCS_NAD27_Iowa_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Iowa_North\012");
                    break;
                  case 26776: // PCS_NAD27_Iowa_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Iowa_South\012");
                    break;
                  case 26777: // PCS_NAD27_Kansas_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Kansas_North\012");
                    break;
                  case 26778: // PCS_NAD27_Kansas_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Kansas_South\012");
                    break;
                  case 26779: // PCS_NAD27_Kentucky_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Kentucky_North\012");
                    break;
                  case 26780: // PCS_NAD27_Kentucky_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Kentucky_South\012");
                    break;
                  case 26781: // PCS_NAD27_Louisiana_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Louisiana_North\012");
                    break;
                  case 26782: // PCS_NAD27_Louisiana_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Louisiana_South\012");
                    break;
                  case 26783: // PCS_NAD27_Maine_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Maine_East\012");
                    break;
                  case 26784: // PCS_NAD27_Maine_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Maine_West\012");
                    break;
                  case 26785: // PCS_NAD27_Maryland
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Maryland\012");
                    break;
                  case 26786: // PCS_NAD27_Massachusetts
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Massachusetts\012");
                    break;
                  case 26787: // PCS_NAD27_Massachusetts_Is
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Massachusetts_Is\012");
                    break;
                  case 26788: // PCS_NAD27_Michigan_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Michigan_North\012");
                    break;
                  case 26789: // PCS_NAD27_Michigan_Central
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Michigan_Central\012");
                    break;
                  case 26790: // PCS_NAD27_Michigan_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Michigan_South\012");
                    break;
                  case 26791: // PCS_NAD27_Minnesota_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Minnesota_North\012");
                    break;
                  case 26792: // PCS_NAD27_Minnesota_Cent
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Minnesota_Cent\012");
                    break;
                  case 26793: // PCS_NAD27_Minnesota_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Minnesota_South\012");
                    break;
                  case 26794: // PCS_NAD27_Mississippi_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Mississippi_East\012");
                    break;
                  case 26795: // PCS_NAD27_Mississippi_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Mississippi_West\012");
                    break;
                  case 26796: // PCS_NAD27_Missouri_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Missouri_East\012");
                    break;
                  case 26797: // PCS_NAD27_Missouri_Central
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Missouri_Central\012");
                    break;
                  case 26798: // PCS_NAD27_Missouri_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Missouri_West\012");
                    break;
                  case 26801: // PCS_NAD_Michigan_Michigan_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD_Michigan_Michigan_East\012");
                    break;
                  case 26802: // PCS_NAD_Michigan_Michigan_Old_Central
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD_Michigan_Michigan_Old_Central\012");
                    break;
                  case 26803: // PCS_NAD_Michigan_Michigan_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD_Michigan_Michigan_West\012");
                    break;
                  case 26903: // PCS_NAD83_UTM_zone_3N
                  case 26904: // PCS_NAD83_UTM_zone_4N
                  case 26905: // PCS_NAD83_UTM_zone_5N
                  case 26906: // PCS_NAD83_UTM_zone_6N
                  case 26907: // PCS_NAD83_UTM_zone_7N
                  case 26908: // PCS_NAD83_UTM_zone_7N
                  case 26909: // PCS_NAD83_UTM_zone_9N
                  case 26910: // PCS_NAD83_UTM_zone_10N
                  case 26911: // PCS_NAD83_UTM_zone_11N
                  case 26912: // PCS_NAD83_UTM_zone_12N
                  case 26913: // PCS_NAD83_UTM_zone_13N
                  case 26914: // PCS_NAD83_UTM_zone_14N
                  case 26915: // PCS_NAD83_UTM_zone_15N
                  case 26916: // PCS_NAD83_UTM_zone_16N
                  case 26917: // PCS_NAD83_UTM_zone_17N
                  case 26918: // PCS_NAD83_UTM_zone_18N
                  case 26919: // PCS_NAD83_UTM_zone_19N
                  case 26920: // PCS_NAD83_UTM_zone_20N
                  case 26921: // PCS_NAD83_UTM_zone_21N
                  case 26922: // PCS_NAD83_UTM_zone_22N
                  case 26923: // PCS_NAD83_UTM_zone_23N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_UTM_zone_%dN\012", lasreader->header.vlr_geo_key_entries[j].value_offset-26900);
                    break;
                  case 26929: // PCS_NAD83_Alabama_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Alabama_East\012");
                    break;
                  case 26930: // PCS_NAD83_Alabama_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Alabama_West\012");
                    break;
                  case 26931: // PCS_NAD83_Alaska_zone_1
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Alaska_zone_1\012");
                    break;
                  case 26932: // PCS_NAD83_Alaska_zone_2
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Alaska_zone_2\012");
                    break;
                  case 26933: // PCS_NAD83_Alaska_zone_3
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Alaska_zone_3\012");
                    break;
                  case 26934: // PCS_NAD83_Alaska_zone_4
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Alaska_zone_4\012");
                    break;
                  case 26935: // PCS_NAD83_Alaska_zone_5
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Alaska_zone_5\012");
                    break;
                  case 26936: // PCS_NAD83_Alaska_zone_6
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Alaska_zone_6\012");
                    break;
                  case 26937: // PCS_NAD83_Alaska_zone_7
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Alaska_zone_7\012");
                    break;
                  case 26938: // PCS_NAD83_Alaska_zone_8
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Alaska_zone_8\012");
                    break;
                  case 26939: // PCS_NAD83_Alaska_zone_9
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Alaska_zone_9\012");
                    break;
                  case 26940: // PCS_NAD83_Alaska_zone_10
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Alaska_zone_10\012");
                    break;
                  case 26941: // PCS_NAD83_California_I
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_California_I\012");
                    break;
                  case 26942: // PCS_NAD83_California_II
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_California_II\012");
                    break;
                  case 26943: // PCS_NAD83_California_III
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_California_III\012");
                    break;
                  case 26944: // PCS_NAD83_California_IV
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_California_IV\012");
                    break;
                  case 26945: // PCS_NAD83_California_V
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_California_V\012");
                    break;
                  case 26946: // PCS_NAD83_California_VI
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_California_VI\012");
                    break;
                  case 26947: // PCS_NAD83_California_VII
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_California_VII\012");
                    break;
                  case 26948: // PCS_NAD83_Arizona_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Arizona_East\012");
                    break;
                  case 26949: // PCS_NAD83_Arizona_Central
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Arizona_Central\012");
                    break;
                  case 26950: // PCS_NAD83_Arizona_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Arizona_West\012");
                    break;
                  case 26951: // PCS_NAD83_Arkansas_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Arkansas_North\012");
                    break;
                  case 26952: // PCS_NAD83_Arkansas_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Arkansas_South\012");
                    break;
                  case 26953: // PCS_NAD83_Colorado_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Colorado_North\012");
                    break;
                  case 26954: // PCS_NAD83_Colorado_Central
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Colorado_Central\012");
                    break;
                  case 26955: // PCS_NAD83_Colorado_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Colorado_South\012");
                    break;
                  case 26956: // PCS_NAD83_Connecticut
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Connecticut\012");
                    break;
                  case 26957: // PCS_NAD83_Delaware
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Delaware\012");
                    break;
                  case 26958: // PCS_NAD83_Florida_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Florida_East\012");
                    break;
                  case 26959: // PCS_NAD83_Florida_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Florida_West\012");
                    break;
                  case 26960: // PCS_NAD83_Florida_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Florida_North\012");
                    break;
                  case 26961: // PCS_NAD83_Hawaii_zone_1
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Hawaii_zone_1\012");
                    break;
                  case 26962: // PCS_NAD83_Hawaii_zone_2
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Hawaii_zone_2\012");
                    break;
                  case 26963: // PCS_NAD83_Hawaii_zone_3
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Hawaii_zone_3\012");
                    break;
                  case 26964: // PCS_NAD83_Hawaii_zone_4
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Hawaii_zone_4\012");
                    break;
                  case 26965: // PCS_NAD83_Hawaii_zone_5
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Hawaii_zone_5\012");
                    break;
                  case 26966: // PCS_NAD83_Georgia_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Georgia_East\012");
                    break;
                  case 26967: // PCS_NAD83_Georgia_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Georgia_West\012");
                    break;
                  case 26968: // PCS_NAD83_Idaho_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Idaho_East\012");
                    break;
                  case 26969: // PCS_NAD83_Idaho_Central
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Idaho_Central\012");
                    break;
                  case 26970: // PCS_NAD83_Idaho_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Idaho_West\012");
                    break;
                  case 26971: // PCS_NAD83_Illinois_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Illinois_East\012");
                    break;
                  case 26972: // PCS_NAD83_Illinois_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Illinois_West\012");
                    break;
                  case 26973: // PCS_NAD83_Indiana_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Indiana_East\012");
                    break;
                  case 26974: // PCS_NAD83_Indiana_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Indiana_West\012");
                    break;
                  case 26975: // PCS_NAD83_Iowa_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Iowa_North\012");
                    break;
                  case 26976: // PCS_NAD83_Iowa_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Iowa_South\012");
                    break;
                  case 26977: // PCS_NAD83_Kansas_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Kansas_North\012");
                    break;
                  case 26978: // PCS_NAD83_Kansas_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Kansas_South\012");
                    break;
                  case 26979: // PCS_NAD83_Kentucky_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Kentucky_North\012");
                    break;
                  case 26980: // PCS_NAD83_Kentucky_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Kentucky_South\012");
                    break;
                  case 26981: // PCS_NAD83_Louisiana_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Louisiana_North\012");
                    break;
                  case 26982: // PCS_NAD83_Louisiana_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Louisiana_South\012");
                    break;
                  case 26983: // PCS_NAD83_Maine_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Maine_East\012");
                    break;
                  case 26984: // PCS_NAD83_Maine_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Maine_West\012");
                    break;
                  case 26985: // PCS_NAD83_Maryland
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Maryland\012");
                    break;
                  case 26986: // PCS_NAD83_Massachusetts
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Massachusetts\012");
                    break;
                  case 26987: // PCS_NAD83_Massachusetts_Is
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Massachusetts_Is\012");
                    break;
                  case 26988: // PCS_NAD83_Michigan_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Michigan_North\012");
                    break;
                  case 26989: // PCS_NAD83_Michigan_Central
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Michigan_Central\012");
                    break;
                  case 26990: // PCS_NAD83_Michigan_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Michigan_South\012");
                    break;
                  case 26991: // PCS_NAD83_Minnesota_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Minnesota_North\012");
                    break;
                  case 26992: // PCS_NAD83_Minnesota_Cent
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Minnesota_Cent\012");
                    break;
                  case 26993: // PCS_NAD83_Minnesota_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Minnesota_South\012");
                    break;
                  case 26994: // PCS_NAD83_Mississippi_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Mississippi_East\012");
                    break;
                  case 26995: // PCS_NAD83_Mississippi_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Mississippi_West\012");
                    break;
                  case 26996: // PCS_NAD83_Missouri_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Missouri_East\012");
                    break;
                  case 26997: // PCS_NAD83_Missouri_Central
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Missouri_Central\012");
                    break;
                  case 26998: // PCS_NAD83_Missouri_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Missouri_West\012");
                    break;
                  case 28348: // PCS_GDA94_MGA_zone_48
                  case 28349:
                  case 28350:
                  case 28351:
                  case 28352:
                  case 28353:
                  case 28354: // PCS_GDA94_MGA_zone_54
                  case 28355: // PCS_GDA94_MGA_zone_55
                  case 28356: // PCS_GDA94_MGA_zone_56
                  case 28357: // PCS_GDA94_MGA_zone_57
                  case 28358: // PCS_GDA94_MGA_zone_58
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_GDA94_MGA_zone_%d\012", lasreader->header.vlr_geo_key_entries[j].value_offset-28300);
                    break;
                  case 29118: // PCS_SAD69_UTM_zone_18N
                  case 29119: // PCS_SAD69_UTM_zone_19N
                  case 29120: // PCS_SAD69_UTM_zone_20N
                  case 29121: // PCS_SAD69_UTM_zone_21N
                  case 29122: // PCS_SAD69_UTM_zone_22N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_SAD69_UTM_zone_%dN\012", lasreader->header.vlr_geo_key_entries[j].value_offset-29100);
                    break;
                  case 29177: // PCS_SAD69_UTM_zone_17S
                  case 29178: // PCS_SAD69_UTM_zone_18S
                  case 29179: // PCS_SAD69_UTM_zone_19S
                  case 29180: // PCS_SAD69_UTM_zone_20S
                  case 29181: // PCS_SAD69_UTM_zone_21S
                  case 29182: // PCS_SAD69_UTM_zone_22S
                  case 29183: // PCS_SAD69_UTM_zone_23S
                  case 29184: // PCS_SAD69_UTM_zone_24S
                  case 29185: // PCS_SAD69_UTM_zone_25S
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_SAD69_UTM_zone_%dS\012", lasreader->header.vlr_geo_key_entries[j].value_offset-29160);
                    break;
                  case 29220: // PCS_Sapper_Hill_UTM_20S
                  case 29221: // PCS_Sapper_Hill_UTM_21S
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Sapper_Hill_UTM_%dS\012", lasreader->header.vlr_geo_key_entries[j].value_offset-29200);
                    break;
                  case 29333: // PCS_Schwarzeck_UTM_33S
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Schwarzeck_UTM_33S\012");
                    break;
                  case 29635: // PCS_Sudan_UTM_zone_35N
                  case 29636: // PCS_Sudan_UTM_zone_35N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Sudan_UTM_zone_%dN\012", lasreader->header.vlr_geo_key_entries[j].value_offset-29600);
                    break;
                  case 29738: // PCS_Tananarive_UTM_38S
                  case 29739: // PCS_Tananarive_UTM_39S
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Tananarive_UTM_%dS\012", lasreader->header.vlr_geo_key_entries[j].value_offset-29700);
                    break;
                  case 29849: // PCS_Timbalai_1948_UTM_49N
                  case 29850: // PCS_Timbalai_1948_UTM_50N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Timbalai_1948_UTM_%dN\012", lasreader->header.vlr_geo_key_entries[j].value_offset-29800);
                    break;
                  case 30339: // PCS_TC_1948_UTM_zone_39N
                  case 30340: // PCS_TC_1948_UTM_zone_40N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_TC_1948_UTM_zone_%dN\012", lasreader->header.vlr_geo_key_entries[j].value_offset-30300);
                    break;
                  case 30729: // PCS_Nord_Sahara_UTM_29N
                  case 30730: // PCS_Nord_Sahara_UTM_30N
                  case 30731: // PCS_Nord_Sahara_UTM_31N
                  case 30732: // PCS_Nord_Sahara_UTM_32N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Nord_Sahara_UTM_%dN\012", lasreader->header.vlr_geo_key_entries[j].value_offset-30700);
                    break;
                  case 31028: // PCS_Yoff_UTM_zone_28N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Yoff_UTM_zone_28N\012");
                    break;
                  case 31121: // PCS_Zanderij_UTM_zone_21N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_Zanderij_UTM_zone_21N\012");
                    break;
                  case 32001: // PCS_NAD27_Montana_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Montana_North\012");
                    break;
                  case 32002: // PCS_NAD27_Montana_Central
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Montana_Central\012");
                    break;
                  case 32003: // PCS_NAD27_Montana_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Montana_South\012");
                    break;
                  case 32005: // PCS_NAD27_Nebraska_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Nebraska_North\012");
                    break;
                  case 32006: // PCS_NAD27_Nebraska_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Nebraska_South\012");
                    break;
                  case 32007: // PCS_NAD27_Nevada_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Nevada_East\012");
                    break;
                  case 32008: // PCS_NAD27_Nevada_Central
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Nevada_Central\012");
                    break;
                  case 32009: // PCS_NAD27_Nevada_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Nevada_West\012");
                    break;
                  case 32010: // PCS_NAD27_New_Hampshire
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_New_Hampshire\012");
                    break;
                  case 32011: // PCS_NAD27_New_Jersey
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_New_Jersey\012");
                    break;
                  case 32012: // PCS_NAD27_New_Mexico_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_New_Mexico_East\012");
                    break;
                  case 32013: // PCS_NAD27_New_Mexico_Cent
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_New_Mexico_Cent\012");
                    break;
                  case 32014: // PCS_NAD27_New_Mexico_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_New_Mexico_West\012");
                    break;
                  case 32015: // PCS_NAD27_New_York_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_New_York_East\012");
                    break;
                  case 32016: // PCS_NAD27_New_York_Central
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_New_York_Central\012");
                    break;
                  case 32017: // PCS_NAD27_New_York_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_New_York_West\012");
                    break;
                  case 32018: // PCS_NAD27_New_York_Long_Is
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_New_York_Long_Is\012");
                    break;
                  case 32019: // PCS_NAD27_North_Carolina
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_North_Carolina\012");
                    break;
                  case 32020: // PCS_NAD27_North_Dakota_N
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_North_Dakota_N\012");
                    break;
                  case 32021: // PCS_NAD27_North_Dakota_S
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_North_Dakota_S\012");
                    break;
                  case 32022: // PCS_NAD27_Ohio_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Ohio_North\012");
                    break;
                  case 32023: // PCS_NAD27_Ohio_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Ohio_South\012");
                    break;
                  case 32024: // PCS_NAD27_Oklahoma_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Oklahoma_North\012");
                    break;
                  case 32025: // PCS_NAD27_Oklahoma_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Oklahoma_South\012");
                    break;
                  case 32026: // PCS_NAD27_Oregon_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Oregon_North\012");
                    break;
                  case 32027: // PCS_NAD27_Oregon_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Oregon_South\012");
                    break;
                  case 32028: // PCS_NAD27_Pennsylvania_N
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Pennsylvania_N\012");
                    break;
                  case 32029: // PCS_NAD27_Pennsylvania_S
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Pennsylvania_S\012");
                    break;
                  case 32030: // PCS_NAD27_Rhode_Island
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Rhode_Island\012");
                    break;
                  case 32031: // PCS_NAD27_South_Carolina_N
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_South_Carolina_N\012");
                    break;
                  case 32033: // PCS_NAD27_South_Carolina_S
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_South_Carolina_S\012");
                    break;
                  case 32034: // PCS_NAD27_South_Dakota_N
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_South_Dakota_N\012");
                    break;
                  case 32035: // PCS_NAD27_South_Dakota_S
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_South_Dakota_S\012");
                    break;
                  case 32036: // PCS_NAD27_Tennessee
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Tennessee\012");
                    break;
                  case 32037: // PCS_NAD27_Texas_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Texas_North\012");
                    break;
                  case 32038: // PCS_NAD27_Texas_North_Cen
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Texas_North_Cen\012");
                    break;
                  case 32039: // PCS_NAD27_Texas_Central
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Texas_Central\012");
                    break;
                  case 32040: // PCS_NAD27_Texas_South_Cen
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Texas_South_Cen\012");
                    break;
                  case 32041: // PCS_NAD27_Texas_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Texas_South\012");
                    break;
                  case 32042: // PCS_NAD27_Utah_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Utah_North\012");
                    break;
                  case 32043: // PCS_NAD27_Utah_Central
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Utah_Central\012");
                    break;
                  case 32044: // PCS_NAD27_Utah_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Utah_South\012");
                    break;
                  case 32045: // PCS_NAD27_Vermont
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Vermont\012");
                    break;
                  case 32046: // PCS_NAD27_Virginia_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Virginia_North\012");
                    break;
                  case 32047: // PCS_NAD27_Virginia_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Virginia_South\012");
                    break;
                  case 32048: // PCS_NAD27_Washington_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Washington_North\012");
                    break;
                  case 32049: // PCS_NAD27_Washington_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Washington_South\012");
                    break;
                  case 32050: // PCS_NAD27_West_Virginia_N
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_West_Virginia_N\012");
                    break;
                  case 32051: // PCS_NAD27_West_Virginia_S
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_West_Virginia_S\012");
                    break;
                  case 32052: // PCS_NAD27_Wisconsin_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Wisconsin_North\012");
                    break;
                  case 32053: // PCS_NAD27_Wisconsin_Cen
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Wisconsin_Cen\012");
                    break;
                  case 32054: // PCS_NAD27_Wisconsin_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Wisconsin_South\012");
                    break;
                  case 32055: // PCS_NAD27_Wyoming_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Wyoming_East\012");
                    break;
                  case 32056: // PCS_NAD27_Wyoming_E_Cen
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Wyoming_E_Cen\012");
                    break;
                  case 32057: // PCS_NAD27_Wyoming_W_Cen
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Wyoming_W_Cen\012");
                    break;
                  case 32058: // PCS_NAD27_Wyoming_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Wyoming_West\012");
                    break;
                  case 32059: // PCS_NAD27_Puerto_Rico
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_Puerto_Rico\012");
                    break;
                  case 32060: // PCS_NAD27_St_Croix
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD27_St_Croix\012");
                    break;
                  case 32100: // PCS_NAD83_Montana
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Montana\012");
                    break;
                  case 32104: // PCS_NAD83_Nebraska
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Nebraska\012");
                    break;
                  case 32107: // PCS_NAD83_Nevada_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Nevada_East\012");
                    break;
                  case 32108: // PCS_NAD83_Nevada_Central
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Nevada_Central\012");
                    break;
                  case 32109: // PCS_NAD83_Nevada_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Nevada_West\012");
                    break;
                  case 32110: // PCS_NAD83_New_Hampshire
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_New_Hampshire\012");
                    break;
                  case 32111: // PCS_NAD83_New_Jersey
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_New_Jersey\012");
                    break;
                  case 32112: // PCS_NAD83_New_Mexico_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_New_Mexico_East\012");
                    break;
                  case 32113: // PCS_NAD83_New_Mexico_Cent
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_New_Mexico_Cent\012");
                    break;
                  case 32114: // PCS_NAD83_New_Mexico_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_New_Mexico_West\012");
                    break;
                  case 32115: // PCS_NAD83_New_York_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_New_York_East\012");
                    break;
                  case 32116: // PCS_NAD83_New_York_Central
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_New_York_Central\012");
                    break;
                  case 32117: // PCS_NAD83_New_York_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_New_York_West\012");
                    break;
                  case 32118: // PCS_NAD83_New_York_Long_Is
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_New_York_Long_Is\012");
                    break;
                  case 32119: // PCS_NAD83_North_Carolina
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_North_Carolina\012");
                    break;
                  case 32120: // PCS_NAD83_North_Dakota_N
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_North_Dakota_N\012");
                    break;
                  case 32121: // PCS_NAD83_North_Dakota_S
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_North_Dakota_S\012");
                    break;
                  case 32122: // PCS_NAD83_Ohio_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Ohio_North\012");
                    break;
                  case 32123: // PCS_NAD83_Ohio_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Ohio_South\012");
                    break;
                  case 32124: // PCS_NAD83_Oklahoma_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Oklahoma_North\012");
                    break;
                  case 32125: // PCS_NAD83_Oklahoma_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Oklahoma_South\012");
                    break;
                  case 32126: // PCS_NAD83_Oregon_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Oregon_North\012");
                    break;
                  case 32127: // PCS_NAD83_Oregon_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Oregon_South\012");
                    break;
                  case 32128: // PCS_NAD83_Pennsylvania_N
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Pennsylvania_N\012");
                    break;
                  case 32129: // PCS_NAD83_Pennsylvania_S
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Pennsylvania_S\012");
                    break;
                  case 32130: // PCS_NAD83_Rhode_Island
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Rhode_Island\012");
                    break;
                  case 32133: // PCS_NAD83_South_Carolina
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_South_Carolina\012");
                    break;
                  case 32134: // PCS_NAD83_South_Dakota_N
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_South_Dakota_N\012");
                    break;
                  case 32135: // PCS_NAD83_South_Dakota_S
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_South_Dakota_S\012");
                    break;
                  case 32136: // PCS_NAD83_Tennessee
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Tennessee\012");
                    break;
                  case 32137: // PCS_NAD83_Texas_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Texas_North\012");
                    break;
                  case 32138: // PCS_NAD83_Texas_North_Cen
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Texas_North_Cen\012");
                    break;
                  case 32139: // PCS_NAD83_Texas_Central
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Texas_Central\012");
                    break;
                  case 32140: // PCS_NAD83_Texas_South_Cen
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Texas_South_Cen\012");
                    break;
                  case 32141: // PCS_NAD83_Texas_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Texas_South\012");
                    break;
                  case 32142: // PCS_NAD83_Utah_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Utah_North\012");
                    break;
                  case 32143: // PCS_NAD83_Utah_Central
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Utah_Central\012");
                    break;
                  case 32144: // PCS_NAD83_Utah_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Utah_South\012");
                    break;
                  case 32145: // PCS_NAD83_Vermont
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Vermont\012");
                    break;
                  case 32146: // PCS_NAD83_Virginia_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Virginia_North\012");
                    break;
                  case 32147: // PCS_NAD83_Virginia_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Virginia_South\012");
                    break;
                  case 32148: // PCS_NAD83_Washington_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Washington_North\012");
                    break;
                  case 32149: // PCS_NAD83_Washington_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Washington_South\012");
                    break;
                  case 32150: // PCS_NAD83_West_Virginia_N
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_West_Virginia_N\012");
                    break;
                  case 32151: // PCS_NAD83_West_Virginia_S
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_West_Virginia_S\012");
                    break;
                  case 32152: // PCS_NAD83_Wisconsin_North
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Wisconsin_North\012");
                    break;
                  case 32153: // PCS_NAD83_Wisconsin_Cen
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Wisconsin_Cen\012");
                    break;
                  case 32154: // PCS_NAD83_Wisconsin_South
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Wisconsin_South\012");
                    break;
                  case 32155: // PCS_NAD83_Wyoming_East
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Wyoming_East\012");
                    break;
                  case 32156: // PCS_NAD83_Wyoming_E_Cen
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Wyoming_E_Cen\012");
                    break;
                  case 32157: // PCS_NAD83_Wyoming_W_Cen
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Wyoming_W_Cen\012");
                    break;
                  case 32158: // PCS_NAD83_Wyoming_West
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Wyoming_West\012");
                    break;
                  case 32161: // PCS_NAD83_Puerto_Rico_Virgin_Is
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_NAD83_Puerto_Rico_Virgin_Is\012");
                    break;
                  case 32201: // PCS_WGS72_UTM_zone_1N 
                  case 32202: // PCS_WGS72_UTM_zone_2N 
                  case 32203: // PCS_WGS72_UTM_zone_3N 
                  case 32204: // PCS_WGS72_UTM_zone_4N 
                  case 32205: // PCS_WGS72_UTM_zone_5N 
                  case 32206: // PCS_WGS72_UTM_zone_6N 
                  case 32207: // PCS_WGS72_UTM_zone_7N 
                  case 32208:
                  case 32209:
                  case 32210:
                  case 32211:
                  case 32212:
                  case 32213:
                  case 32214:
                  case 32215:
                  case 32216:
                  case 32217:
                  case 32218:
                  case 32219:
                  case 32220:
                  case 32221:
                  case 32222:
                  case 32223:
                  case 32224:
                  case 32225:
                  case 32226:
                  case 32227:
                  case 32228:
                  case 32229:
                  case 32230:
                  case 32231:
                  case 32232:
                  case 32233:
                  case 32234:
                  case 32235:
                  case 32236:
                  case 32237:
                  case 32238:
                  case 32239:
                  case 32240:
                  case 32241:
                  case 32242:
                  case 32243:
                  case 32244:
                  case 32245:
                  case 32246:
                  case 32247:
                  case 32248:
                  case 32249:
                  case 32250:
                  case 32251:
                  case 32252:
                  case 32253:
                  case 32254:
                  case 32255:
                  case 32256:
                  case 32257:
                  case 32258:
                  case 32259: // PCS_WGS72_UTM_zone_59N 
                  case 32260: // PCS_WGS72_UTM_zone_60N 
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_WGS72_UTM_zone_%dN\012", lasreader->header.vlr_geo_key_entries[j].value_offset-32200);
                    break;
                  case 32301: // PCS_WGS72_UTM_zone_1S
                  case 32302: // PCS_WGS72_UTM_zone_2S
                  case 32303: // PCS_WGS72_UTM_zone_3S
                  case 32304: // PCS_WGS72_UTM_zone_4S
                  case 32305: // PCS_WGS72_UTM_zone_5S
                  case 32306: // PCS_WGS72_UTM_zone_6S
                  case 32307: // PCS_WGS72_UTM_zone_7S
                  case 32308:
                  case 32309:
                  case 32310:
                  case 32311:
                  case 32312:
                  case 32313:
                  case 32314:
                  case 32315:
                  case 32316:
                  case 32317:
                  case 32318:
                  case 32319:
                  case 32320:
                  case 32321:
                  case 32322:
                  case 32323:
                  case 32324:
                  case 32325:
                  case 32326:
                  case 32327:
                  case 32328:
                  case 32329:
                  case 32330:
                  case 32331:
                  case 32332:
                  case 32333:
                  case 32334:
                  case 32335:
                  case 32336:
                  case 32337:
                  case 32338:
                  case 32339:
                  case 32340:
                  case 32341:
                  case 32342:
                  case 32343:
                  case 32344:
                  case 32345:
                  case 32346:
                  case 32347:
                  case 32348:
                  case 32349:
                  case 32350:
                  case 32351:
                  case 32352:
                  case 32353:
                  case 32354:
                  case 32355:
                  case 32356:
                  case 32357:
                  case 32358:
                  case 32359: // PCS_WGS72_UTM_zone_59S
                  case 32360: // PCS_WGS72_UTM_zone_60S
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_WGS72_UTM_zone_%dS\012", lasreader->header.vlr_geo_key_entries[j].value_offset-32300);
                    break;
                  case 32401: // PCS_WGS72BE_UTM_zone_1N
                  case 32402: // PCS_WGS72BE_UTM_zone_2N
                  case 32403: // PCS_WGS72BE_UTM_zone_3N
                  case 32404: // PCS_WGS72BE_UTM_zone_4N
                  case 32405: // PCS_WGS72BE_UTM_zone_5N
                  case 32406: // PCS_WGS72BE_UTM_zone_6N
                  case 32407: // PCS_WGS72BE_UTM_zone_7N
                  case 32408:
                  case 32409:
                  case 32410:
                  case 32411:
                  case 32412:
                  case 32413:
                  case 32414:
                  case 32415:
                  case 32416:
                  case 32417:
                  case 32418:
                  case 32419:
                  case 32420:
                  case 32421:
                  case 32422:
                  case 32423:
                  case 32424:
                  case 32425:
                  case 32426:
                  case 32427:
                  case 32428:
                  case 32429:
                  case 32430:
                  case 32431:
                  case 32432:
                  case 32433:
                  case 32434:
                  case 32435:
                  case 32436:
                  case 32437:
                  case 32438:
                  case 32439:
                  case 32440:
                  case 32441:
                  case 32442:
                  case 32443:
                  case 32444:
                  case 32445:
                  case 32446:
                  case 32447:
                  case 32448:
                  case 32449:
                  case 32450:
                  case 32451:
                  case 32452:
                  case 32453:
                  case 32454:
                  case 32455:
                  case 32456:
                  case 32457:
                  case 32458:
                  case 32459: // PCS_WGS72BE_UTM_zone_59N
                  case 32460: // PCS_WGS72BE_UTM_zone_60N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_WGS72BE_UTM_zone_%dN\012", lasreader->header.vlr_geo_key_entries[j].value_offset-32400);
                    break;
                  case 32501: // PCS_WGS72BE_UTM_zone_1S
                  case 32502: // PCS_WGS72BE_UTM_zone_2S
                  case 32503: // PCS_WGS72BE_UTM_zone_3S
                  case 32504: // PCS_WGS72BE_UTM_zone_4S
                  case 32505: // PCS_WGS72BE_UTM_zone_5S
                  case 32506: // PCS_WGS72BE_UTM_zone_6S
                  case 32507: // PCS_WGS72BE_UTM_zone_7S
                  case 32508:
                  case 32509:
                  case 32510:
                  case 32511:
                  case 32512:
                  case 32513:
                  case 32514:
                  case 32515:
                  case 32516:
                  case 32517:
                  case 32518:
                  case 32519:
                  case 32520:
                  case 32521:
                  case 32522:
                  case 32523:
                  case 32524:
                  case 32525:
                  case 32526:
                  case 32527:
                  case 32528:
                  case 32529:
                  case 32530:
                  case 32531:
                  case 32532:
                  case 32533:
                  case 32534:
                  case 32535:
                  case 32536:
                  case 32537:
                  case 32538:
                  case 32539:
                  case 32540:
                  case 32541:
                  case 32542:
                  case 32543:
                  case 32544:
                  case 32545:
                  case 32546:
                  case 32547:
                  case 32548:
                  case 32549:
                  case 32550:
                  case 32551:
                  case 32552:
                  case 32553:
                  case 32554:
                  case 32555:
                  case 32556:
                  case 32557:
                  case 32558:
                  case 32559: // PCS_WGS72BE_UTM_zone_59S
                  case 32560: // PCS_WGS72BE_UTM_zone_60S
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_WGS72BE_UTM_zone_%dS\012", lasreader->header.vlr_geo_key_entries[j].value_offset-32500);
                    break;
                  case 32601: // PCS_WGS84_UTM_zone_1N
                  case 32602: // PCS_WGS84_UTM_zone_2N
                  case 32603: // PCS_WGS84_UTM_zone_3N
                  case 32604: // PCS_WGS84_UTM_zone_4N
                  case 32605: // PCS_WGS84_UTM_zone_5N
                  case 32606: // PCS_WGS84_UTM_zone_6N
                  case 32607: // PCS_WGS84_UTM_zone_7N
                  case 32608:
                  case 32609:
                  case 32610:
                  case 32611:
                  case 32612:
                  case 32613:
                  case 32614:
                  case 32615:
                  case 32616:
                  case 32617:
                  case 32618:
                  case 32619:
                  case 32620:
                  case 32621:
                  case 32622:
                  case 32623:
                  case 32624:
                  case 32625:
                  case 32626:
                  case 32627:
                  case 32628:
                  case 32629:
                  case 32630:
                  case 32631:
                  case 32632:
                  case 32633:
                  case 32634:
                  case 32635:
                  case 32636:
                  case 32637:
                  case 32638:
                  case 32639:
                  case 32640:
                  case 32641:
                  case 32642:
                  case 32643:
                  case 32644:
                  case 32645:
                  case 32646:
                  case 32647:
                  case 32648:
                  case 32649:
                  case 32650:
                  case 32651:
                  case 32652:
                  case 32653:
                  case 32654:
                  case 32655:
                  case 32656:
                  case 32657:
                  case 32658:
                  case 32659: // PCS_WGS84_UTM_zone_59N
                  case 32660: // PCS_WGS84_UTM_zone_60N
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_WGS84_UTM_zone_%dN\012", lasreader->header.vlr_geo_key_entries[j].value_offset-32600);
                    break;
                  case 32701: // PCS_WGS84_UTM_zone_1S
                  case 32702: // PCS_WGS84_UTM_zone_2S
                  case 32703: // PCS_WGS84_UTM_zone_3S
                  case 32704: // PCS_WGS84_UTM_zone_4S
                  case 32705: // PCS_WGS84_UTM_zone_5S
                  case 32706: // PCS_WGS84_UTM_zone_6S
                  case 32707: // PCS_WGS84_UTM_zone_7S
                  case 32708:
                  case 32709:
                  case 32710:
                  case 32711:
                  case 32712:
                  case 32713:
                  case 32714:
                  case 32715:
                  case 32716:
                  case 32717:
                  case 32718:
                  case 32719:
                  case 32720:
                  case 32721:
                  case 32722:
                  case 32723:
                  case 32724:
                  case 32725:
                  case 32726:
                  case 32727:
                  case 32728:
                  case 32729:
                  case 32730:
                  case 32731:
                  case 32732:
                  case 32733:
                  case 32734:
                  case 32735:
                  case 32736:
                  case 32737:
                  case 32738:
                  case 32739:
                  case 32740:
                  case 32741:
                  case 32742:
                  case 32743:
                  case 32744:
                  case 32745:
                  case 32746:
                  case 32747:
                  case 32748:
                  case 32749:
                  case 32750:
                  case 32751:
                  case 32752:
                  case 32753:
                  case 32754:
                  case 32755:
                  case 32756:
                  case 32757:
                  case 32758:
                  case 32759: // PCS_WGS84_UTM_zone_59S
                  case 32760: // PCS_WGS84_UTM_zone_60S
                    horizontal_units = 9001;
                    fprintf(file_out, "ProjectedCSTypeGeoKey: PCS_WGS84_UTM_zone_%dS\012", lasreader->header.vlr_geo_key_entries[j].value_offset-32700);
                    break;
                  default:
                    fprintf(file_out, "ProjectedCSTypeGeoKey: look-up for %d not implemented\012", lasreader->header.vlr_geo_key_entries[j].value_offset);
                  }
                  break;
                case 3073: // PCSCitationGeoKey
                  if (lasreader->header.vlr_geo_ascii_params)
                  {
                    char dummy[256];
                    strncpy(dummy, &(lasreader->header.vlr_geo_ascii_params[lasreader->header.vlr_geo_key_entries[j].value_offset]), lasreader->header.vlr_geo_key_entries[j].count);
                    dummy[lasreader->header.vlr_geo_key_entries[j].count-1] = '\0';
                    fprintf(file_out, "PCSCitationGeoKey: %s\012",dummy);
                  }
                  break;
                case 3074: // ProjectionGeoKey
                  switch (lasreader->header.vlr_geo_key_entries[j].value_offset)
                  {
                  case 32767: // user-defined
                    fprintf(file_out, "ProjectionGeoKey: user-defined\012");
                    break;
                  case 10101: // Proj_Alabama_CS27_East
                    fprintf(file_out, "ProjectionGeoKey: Proj_Alabama_CS27_East\012");
                    break;
                  case 10102: // Proj_Alabama_CS27_West
                    fprintf(file_out, "ProjectionGeoKey: Proj_Alabama_CS27_West\012");
                    break;
                  case 10131: // Proj_Alabama_CS83_East				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Alabama_CS83_East\012");
                    break;
                  case 10132: // Proj_Alabama_CS83_West				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Alabama_CS83_West\012");
                    break;
                  case 10201: // Proj_Arizona_Coordinate_System_east			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Arizona_Coordinate_System_east\012");
                    break;
                  case 10202: // Proj_Arizona_Coordinate_System_Central		
                    fprintf(file_out, "ProjectionGeoKey: Proj_Arizona_Coordinate_System_Central\012");
                    break;
                  case 10203: // Proj_Arizona_Coordinate_System_west			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Arizona_Coordinate_System_west\012");
                    break;
                  case 10231: // Proj_Arizona_CS83_east				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Arizona_CS83_east\012");
                    break;
                  case 10232: // Proj_Arizona_CS83_Central				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Arizona_CS83_Central\012");
                    break;
                  case 10233: // Proj_Arizona_CS83_west				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Arizona_CS83_west\012");
                    break;
                  case 10301: // Proj_Arkansas_CS27_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Arkansas_CS27_North\012");
                    break;
                  case 10302: // Proj_Arkansas_CS27_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Arkansas_CS27_South\012");
                    break;
                  case 10331: // Proj_Arkansas_CS83_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Arkansas_CS83_North\012");
                    break;
                  case 10332: // Proj_Arkansas_CS83_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Arkansas_CS83_South\012");
                    break;
                  case 10401: // Proj_California_CS27_I				
                    fprintf(file_out, "ProjectionGeoKey: Proj_California_CS27_I\012");
                    break;
                  case 10402: // Proj_California_CS27_II				
                    fprintf(file_out, "ProjectionGeoKey: Proj_California_CS27_II\012");
                    break;
                  case 10403: // Proj_California_CS27_III				
                    fprintf(file_out, "ProjectionGeoKey: Proj_California_CS27_III\012");
                    break;
                  case 10404: // Proj_California_CS27_IV				
                    fprintf(file_out, "ProjectionGeoKey: Proj_California_CS27_IV\012");
                    break;
                  case 10405: // Proj_California_CS27_V				
                    fprintf(file_out, "ProjectionGeoKey: Proj_California_CS27_V\012");
                    break;
                  case 10406: // Proj_California_CS27_VI				
                    fprintf(file_out, "ProjectionGeoKey: Proj_California_CS27_VI\012");
                    break;
                  case 10407: // Proj_California_CS27_VII				
                    fprintf(file_out, "ProjectionGeoKey: Proj_California_CS27_VII\012");
                    break;
                  case 10431: // Proj_California_CS83_1				
                    fprintf(file_out, "ProjectionGeoKey: Proj_California_CS83_1\012");
                    break;
                  case 10432: // Proj_California_CS83_2				
                    fprintf(file_out, "ProjectionGeoKey: Proj_California_CS83_2\012");
                    break;
                  case 10433: // Proj_California_CS83_3				
                    fprintf(file_out, "ProjectionGeoKey: Proj_California_CS83_3\012");
                    break;
                  case 10434: // Proj_California_CS83_4				
                    fprintf(file_out, "ProjectionGeoKey: Proj_California_CS83_4\012");
                    break;
                  case 10435: // Proj_California_CS83_5				
                    fprintf(file_out, "ProjectionGeoKey: Proj_California_CS83_5\012");
                    break;
                  case 10436: // Proj_California_CS83_6				
                    fprintf(file_out, "ProjectionGeoKey: Proj_California_CS83_6\012");
                    break;
                  case 10501: // Proj_Colorado_CS27_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Colorado_CS27_North\012");
                    break;
                  case 10502: // Proj_Colorado_CS27_Central				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Colorado_CS27_Central\012");
                    break;
                  case 10503: // Proj_Colorado_CS27_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Colorado_CS27_South\012");
                    break;
                  case 10531: // Proj_Colorado_CS83_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Colorado_CS83_North\012");
                    break;
                  case 10532: // Proj_Colorado_CS83_Central				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Colorado_CS83_Central\012");
                    break;
                  case 10533: // Proj_Colorado_CS83_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Colorado_CS83_South\012");
                    break;
                  case 10600: // Proj_Connecticut_CS27				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Connecticut_CS27\012");
                    break;
                  case 10630: // Proj_Connecticut_CS83				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Connecticut_CS83\012");
                    break;
                  case 10700: // Proj_Delaware_CS27					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Delaware_CS27\012");
                    break;
                  case 10730: // Proj_Delaware_CS83					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Delaware_CS83\012");
                    break;
                  case 10901: // Proj_Florida_CS27_East				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Florida_CS27_East\012");
                    break;
                  case 10902: // Proj_Florida_CS27_West				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Florida_CS27_West\012");
                    break;
                  case 10903: // Proj_Florida_CS27_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Florida_CS27_North\012");
                    break;
                  case 10931: // Proj_Florida_CS83_East				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Florida_CS83_East\012");
                    break;
                  case 10932: // Proj_Florida_CS83_West				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Florida_CS83_West\012");
                    break;
                  case 10933: // Proj_Florida_CS83_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Florida_CS83_North\012");
                    break;
                  case 11001: // Proj_Georgia_CS27_East				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Georgia_CS27_East\012");
                    break;
                  case 11002: // Proj_Georgia_CS27_West				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Georgia_CS27_West\012");
                    break;
                  case 11031: // Proj_Georgia_CS83_East				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Georgia_CS83_East\012");
                    break;
                  case 11032: // Proj_Georgia_CS83_West				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Georgia_CS83_West\012");
                    break;
                  case 11101: // Proj_Idaho_CS27_East				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Idaho_CS27_East\012");
                    break;
                  case 11102: // Proj_Idaho_CS27_Central				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Idaho_CS27_Central\012");
                    break;
                  case 11103: // Proj_Idaho_CS27_West				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Idaho_CS27_West\012");
                    break;
                  case 11131: // Proj_Idaho_CS83_East				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Idaho_CS83_East\012");
                    break;
                  case 11132: // Proj_Idaho_CS83_Central				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Idaho_CS83_Central\012");
                    break;
                  case 11133: // Proj_Idaho_CS83_West				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Idaho_CS83_West\012");
                    break;
                  case 11201: // Proj_Illinois_CS27_East				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Illinois_CS27_East\012");
                    break;
                  case 11202: // Proj_Illinois_CS27_West				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Illinois_CS27_West\012");
                    break;
                  case 11231: // Proj_Illinois_CS83_East				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Illinois_CS83_East\012");
                    break;
                  case 11232: // Proj_Illinois_CS83_West				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Illinois_CS83_West\012");
                    break;
                  case 11301: // Proj_Indiana_CS27_East				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Indiana_CS27_East\012");
                    break;
                  case 11302: // Proj_Indiana_CS27_West				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Indiana_CS27_West\012");
                    break;
                  case 11331: // Proj_Indiana_CS83_East				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Indiana_CS83_East\012");
                    break;
                  case 11332: // Proj_Indiana_CS83_West				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Indiana_CS83_West\012");
                    break;
                  case 11401: // Proj_Iowa_CS27_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Iowa_CS27_North\012");
                    break;
                  case 11402: // Proj_Iowa_CS27_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Iowa_CS27_South\012");
                    break;
                  case 11431: // Proj_Iowa_CS83_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Iowa_CS83_North\012");
                    break;
                  case 11432: // Proj_Iowa_CS83_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Iowa_CS83_South\012");
                    break;
                  case 11501: // Proj_Kansas_CS27_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Kansas_CS27_North\012");
                    break;
                  case 11502: // Proj_Kansas_CS27_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Kansas_CS27_South\012");
                    break;
                  case 11531: // Proj_Kansas_CS83_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Kansas_CS83_North\012");
                    break;
                  case 11532: // Proj_Kansas_CS83_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Kansas_CS83_South\012");
                    break;
                  case 11601: // Proj_Kentucky_CS27_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Kentucky_CS27_North\012");
                    break;
                  case 11602: // Proj_Kentucky_CS27_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Kentucky_CS27_South\012");
                    break;
                  case 11631: // Proj_Kentucky_CS83_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Kentucky_CS83_North\012");
                    break;
                  case 11632: // Proj_Kentucky_CS83_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Kentucky_CS83_South\012");
                    break;
                  case 11701: // Proj_Louisiana_CS27_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Louisiana_CS27_North\012");
                    break;
                  case 11702: // Proj_Louisiana_CS27_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Louisiana_CS27_South\012");
                    break;
                  case 11731: // Proj_Louisiana_CS83_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Louisiana_CS83_North\012");
                    break;
                  case 11732: // Proj_Louisiana_CS83_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Louisiana_CS83_South\012");
                    break;
                  case 11801: // Proj_Maine_CS27_East				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Maine_CS27_East\012");
                    break;
                  case 11802: // Proj_Maine_CS27_West				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Maine_CS27_West\012");
                    break;
                  case 11831: // Proj_Maine_CS83_East				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Maine_CS83_East\012");
                    break;
                  case 11832: // Proj_Maine_CS83_West				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Maine_CS83_West\012");
                    break;
                  case 11900: // Proj_Maryland_CS27					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Maryland_CS27\012");
                    break;
                  case 11930: // Proj_Maryland_CS83					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Maryland_CS83\012");
                    break;
                  case 12001: // Proj_Massachusetts_CS27_Mainland			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Massachusetts_CS27_Mainland\012");
                    break;
                  case 12002: // Proj_Massachusetts_CS27_Island			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Massachusetts_CS27_Island\012");
                    break;
                  case 12031: // Proj_Massachusetts_CS83_Mainland			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Massachusetts_CS83_Mainland\012");
                    break;
                  case 12032: // Proj_Massachusetts_CS83_Island			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Massachusetts_CS83_Island\012");
                    break;
                  case 12101: // Proj_Michigan_State_Plane_East			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Michigan_State_Plane_East\012");
                    break;
                  case 12102: // Proj_Michigan_State_Plane_Old_Central		
                    fprintf(file_out, "ProjectionGeoKey: Proj_Michigan_State_Plane_Old_Central\012");
                    break;
                  case 12103: // Proj_Michigan_State_Plane_West			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Michigan_State_Plane_West\012");
                    break;
                  case 12111: // Proj_Michigan_CS27_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Michigan_CS27_North\012");
                    break;
                  case 12112: // Proj_Michigan_CS27_Central				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Michigan_CS27_Central\012");
                    break;
                  case 12113: // Proj_Michigan_CS27_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Michigan_CS27_South\012");
                    break;
                  case 12141: // Proj_Michigan_CS83_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Michigan_CS83_North\012");
                    break;
                  case 12142: // Proj_Michigan_CS83_Central				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Michigan_CS83_Central\012");
                    break;
                  case 12143: // Proj_Michigan_CS83_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Michigan_CS83_South\012");
                    break;
                  case 12201: // Proj_Minnesota_CS27_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Minnesota_CS27_North\012");
                    break;
                  case 12202: // Proj_Minnesota_CS27_Central				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Minnesota_CS27_Central\012");
                    break;
                  case 12203: // Proj_Minnesota_CS27_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Minnesota_CS27_South\012");
                    break;
                  case 12231: // Proj_Minnesota_CS83_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Minnesota_CS83_North\012");
                    break;
                  case 12232: // Proj_Minnesota_CS83_Central				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Minnesota_CS83_Central\012");
                    break;
                  case 12233: // Proj_Minnesota_CS83_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Minnesota_CS83_South\012");
                    break;
                  case 12301: // Proj_Mississippi_CS27_East				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Mississippi_CS27_East\012");
                    break;
                  case 12302: // Proj_Mississippi_CS27_West				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Mississippi_CS27_West\012");
                    break;
                  case 12331: // Proj_Mississippi_CS83_East				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Mississippi_CS83_East\012");
                    break;
                  case 12332: // Proj_Mississippi_CS83_West				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Mississippi_CS83_West\012");
                    break;
                  case 12401: // Proj_Missouri_CS27_East				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Missouri_CS27_East\012");
                    break;
                  case 12402: // Proj_Missouri_CS27_Central				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Missouri_CS27_Central\012");
                    break;
                  case 12403: // Proj_Missouri_CS27_West				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Missouri_CS27_West\012");
                    break;
                  case 12431: // Proj_Missouri_CS83_East				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Missouri_CS83_East\012");
                    break;
                  case 12432: // Proj_Missouri_CS83_Central				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Missouri_CS83_Central\012");
                    break;
                  case 12433: // Proj_Missouri_CS83_West				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Missouri_CS83_West\012");
                    break;
                  case 12501: // Proj_Montana_CS27_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Montana_CS27_North\012");
                    break;
                  case 12502: // Proj_Montana_CS27_Central				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Montana_CS27_Central\012");
                    break;
                  case 12503: // Proj_Montana_CS27_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Montana_CS27_South\012");
                    break;
                  case 12530: // Proj_Montana_CS83					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Montana_CS83\012");
                    break;
                  case 12601: // Proj_Nebraska_CS27_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Nebraska_CS27_North\012");
                    break;
                  case 12602: // Proj_Nebraska_CS27_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Nebraska_CS27_South\012");
                    break;
                  case 12630: // Proj_Nebraska_CS83					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Nebraska_CS83\012");
                    break;
                  case 12701: // Proj_Nevada_CS27_East				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Nevada_CS27_East\012");
                    break;
                  case 12702: // Proj_Nevada_CS27_Central				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Nevada_CS27_Central\012");
                    break;
                  case 12703: // Proj_Nevada_CS27_West				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Nevada_CS27_West\012");
                    break;
                  case 12731: // Proj_Nevada_CS83_East				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Nevada_CS83_East\012");
                    break;
                  case 12732: // Proj_Nevada_CS83_Central				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Nevada_CS83_Central\012");
                    break;
                  case 12733: // Proj_Nevada_CS83_West				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Nevada_CS83_West\012");
                    break;
                  case 12800: // Proj_New_Hampshire_CS27				
                    fprintf(file_out, "ProjectionGeoKey: Proj_New_Hampshire_CS27\012");
                    break;
                  case 12830: // Proj_New_Hampshire_CS83				
                    fprintf(file_out, "ProjectionGeoKey: Proj_New_Hampshire_CS83\012");
                    break;
                  case 12900: // Proj_New_Jersey_CS27				
                    fprintf(file_out, "ProjectionGeoKey: Proj_New_Jersey_CS27\012");
                    break;
                  case 12930: // Proj_New_Jersey_CS83				
                    fprintf(file_out, "ProjectionGeoKey: Proj_New_Jersey_CS83\012");
                    break;
                  case 13001: // Proj_New_Mexico_CS27_East				
                    fprintf(file_out, "ProjectionGeoKey: Proj_New_Mexico_CS27_East\012");
                    break;
                  case 13002: // Proj_New_Mexico_CS27_Central			
                    fprintf(file_out, "ProjectionGeoKey: Proj_New_Mexico_CS27_Central\012");
                    break;
                  case 13003: // Proj_New_Mexico_CS27_West				
                    fprintf(file_out, "ProjectionGeoKey: Proj_New_Mexico_CS27_West\012");
                    break;
                  case 13031: // Proj_New_Mexico_CS83_East				
                    fprintf(file_out, "ProjectionGeoKey: Proj_New_Mexico_CS83_East\012");
                    break;
                  case 13032: // Proj_New_Mexico_CS83_Central			
                    fprintf(file_out, "ProjectionGeoKey: Proj_New_Mexico_CS83_Central\012");
                    break;
                  case 13033: // Proj_New_Mexico_CS83_West				
                    fprintf(file_out, "ProjectionGeoKey: Proj_New_Mexico_CS83_West\012");
                    break;
                  case 13101: // Proj_New_York_CS27_East				
                    fprintf(file_out, "ProjectionGeoKey: Proj_New_York_CS27_East\012");
                    break;
                  case 13102: // Proj_New_York_CS27_Central				
                    fprintf(file_out, "ProjectionGeoKey: Proj_New_York_CS27_Central\012");
                    break;
                  case 13103: // Proj_New_York_CS27_West				
                    fprintf(file_out, "ProjectionGeoKey: Proj_New_York_CS27_West\012");
                    break;
                  case 13104: // Proj_New_York_CS27_Long_Island			
                    fprintf(file_out, "ProjectionGeoKey: Proj_New_York_CS27_Long_Island\012");
                    break;
                  case 13131: // Proj_New_York_CS83_East				
                    fprintf(file_out, "ProjectionGeoKey: Proj_New_York_CS83_East\012");
                    break;
                  case 13132: // Proj_New_York_CS83_Central				
                    fprintf(file_out, "ProjectionGeoKey: Proj_New_York_CS83_Central\012");
                    break;
                  case 13133: // Proj_New_York_CS83_West				
                    fprintf(file_out, "ProjectionGeoKey: Proj_New_York_CS83_West\012");
                    break;
                  case 13134: // Proj_New_York_CS83_Long_Island			
                    fprintf(file_out, "ProjectionGeoKey: Proj_New_York_CS83_Long_Island\012");
                    break;
                  case 13200: // Proj_North_Carolina_CS27				
                    fprintf(file_out, "ProjectionGeoKey: Proj_North_Carolina_CS27\012");
                    break;
                  case 13230: // Proj_North_Carolina_CS83				
                    fprintf(file_out, "ProjectionGeoKey: Proj_North_Carolina_CS83\012");
                    break;
                  case 13301: // Proj_North_Dakota_CS27_North			
                    fprintf(file_out, "ProjectionGeoKey: Proj_North_Dakota_CS27_North\012");
                    break;
                  case 13302: // Proj_North_Dakota_CS27_South			
                    fprintf(file_out, "ProjectionGeoKey: Proj_North_Dakota_CS27_South\012");
                    break;
                  case 13331: // Proj_North_Dakota_CS83_North			
                    fprintf(file_out, "ProjectionGeoKey: Proj_North_Dakota_CS83_North\012");
                    break;
                  case 13332: // Proj_North_Dakota_CS83_South			
                    fprintf(file_out, "ProjectionGeoKey: Proj_North_Dakota_CS83_South\012");
                    break;
                  case 13401: // Proj_Ohio_CS27_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Ohio_CS27_North\012");
                    break;
                  case 13402: // Proj_Ohio_CS27_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Ohio_CS27_South\012");
                    break;
                  case 13431: // Proj_Ohio_CS83_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Ohio_CS83_North\012");
                    break;
                  case 13432: // Proj_Ohio_CS83_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Ohio_CS83_South\012");
                    break;
                  case 13501: // Proj_Oklahoma_CS27_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Oklahoma_CS27_North\012");
                    break;
                  case 13502: // Proj_Oklahoma_CS27_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Oklahoma_CS27_South\012");
                    break;
                  case 13531: // Proj_Oklahoma_CS83_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Oklahoma_CS83_North\012");
                    break;
                  case 13532: // Proj_Oklahoma_CS83_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Oklahoma_CS83_South\012");
                    break;
                  case 13601: // Proj_Oregon_CS27_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Oregon_CS27_North\012");
                    break;
                  case 13602: // Proj_Oregon_CS27_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Oregon_CS27_South\012");
                    break;
                  case 13631: // Proj_Oregon_CS83_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Oregon_CS83_North\012");
                    break;
                  case 13632: // Proj_Oregon_CS83_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Oregon_CS83_South\012");
                    break;
                  case 13701: // Proj_Pennsylvania_CS27_North			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Pennsylvania_CS27_North\012");
                    break;
                  case 13702: // Proj_Pennsylvania_CS27_South			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Pennsylvania_CS27_South\012");
                    break;
                  case 13731: // Proj_Pennsylvania_CS83_North			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Pennsylvania_CS83_North\012");
                    break;
                  case 13732: // Proj_Pennsylvania_CS83_South			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Pennsylvania_CS83_South\012");
                    break;
                  case 13800: // Proj_Rhode_Island_CS27				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Rhode_Island_CS27\012");
                    break;
                  case 13830: // Proj_Rhode_Island_CS83				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Rhode_Island_CS83\012");
                    break;
                  case 13901: // Proj_South_Carolina_CS27_North			
                    fprintf(file_out, "ProjectionGeoKey: Proj_South_Carolina_CS27_North\012");
                    break;
                  case 13902: // Proj_South_Carolina_CS27_South			
                    fprintf(file_out, "ProjectionGeoKey: Proj_South_Carolina_CS27_South\012");
                    break;
                  case 13930: // Proj_South_Carolina_CS83				
                    fprintf(file_out, "ProjectionGeoKey: Proj_South_Carolina_CS83\012");
                    break;
                  case 14001: // Proj_South_Dakota_CS27_North			
                    fprintf(file_out, "ProjectionGeoKey: Proj_South_Dakota_CS27_North\012");
                    break;
                  case 14002: // Proj_South_Dakota_CS27_South			
                    fprintf(file_out, "ProjectionGeoKey: Proj_South_Dakota_CS27_South\012");
                    break;
                  case 14031: // Proj_South_Dakota_CS83_North			
                    fprintf(file_out, "ProjectionGeoKey: Proj_South_Dakota_CS83_North\012");
                    break;
                  case 14032: // Proj_South_Dakota_CS83_South			
                    fprintf(file_out, "ProjectionGeoKey: Proj_South_Dakota_CS83_South\012");
                    break;
                  case 14100: // Proj_Tennessee_CS27					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Tennessee_CS27\012");
                    break;
                  case 14130: // Proj_Tennessee_CS83					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Tennessee_CS83\012");
                    break;
                  case 14201: // Proj_Texas_CS27_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Texas_CS27_North\012");
                    break;
                  case 14202: // Proj_Texas_CS27_North_Central			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Texas_CS27_North_Central\012");
                    break;
                  case 14203: // Proj_Texas_CS27_Central				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Texas_CS27_Central\012");
                    break;
                  case 14204: // Proj_Texas_CS27_South_Central			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Texas_CS27_South_Central\012");
                    break;
                  case 14205: // Proj_Texas_CS27_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Texas_CS27_South\012");
                    break;
                  case 14231: // Proj_Texas_CS83_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Texas_CS83_North\012");
                    break;
                  case 14232: // Proj_Texas_CS83_North_Central			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Texas_CS83_North_Central\012");
                    break;
                  case 14233: // Proj_Texas_CS83_Central				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Texas_CS83_Central\012");
                    break;
                  case 14234: // Proj_Texas_CS83_South_Central			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Texas_CS83_South_Central\012");
                    break;
                  case 14235: // Proj_Texas_CS83_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Texas_CS83_South\012");
                    break;
                  case 14301: // Proj_Utah_CS27_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Utah_CS27_North\012");
                    break;
                  case 14302: // Proj_Utah_CS27_Central				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Utah_CS27_Central\012");
                    break;
                  case 14303: // Proj_Utah_CS27_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Utah_CS27_South\012");
                    break;
                  case 14331: // Proj_Utah_CS83_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Utah_CS83_North\012");
                    break;
                  case 14332: // Proj_Utah_CS83_Central				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Utah_CS83_Central\012");
                    break;
                  case 14333: // Proj_Utah_CS83_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Utah_CS83_South\012");
                    break;
                  case 14400: // Proj_Vermont_CS27					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Vermont_CS27\012");
                    break;
                  case 14430: // Proj_Vermont_CS83					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Vermont_CS83\012");
                    break;
                  case 14501: // Proj_Virginia_CS27_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Virginia_CS27_North\012");
                    break;
                  case 14502: // Proj_Virginia_CS27_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Virginia_CS27_South\012");
                    break;
                  case 14531: // Proj_Virginia_CS83_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Virginia_CS83_North\012");
                    break;
                  case 14532: // Proj_Virginia_CS83_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Virginia_CS83_South\012");
                    break;
                  case 14601: // Proj_Washington_CS27_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Washington_CS27_North\012");
                    break;
                  case 14602: // Proj_Washington_CS27_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Washington_CS27_South\012");
                    break;
                  case 14631: // Proj_Washington_CS83_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Washington_CS83_North\012");
                    break;
                  case 14632: // Proj_Washington_CS83_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Washington_CS83_South\012");
                    break;
                  case 14701: // Proj_West_Virginia_CS27_North			
                    fprintf(file_out, "ProjectionGeoKey: Proj_West_Virginia_CS27_North\012");
                    break;
                  case 14702: // Proj_West_Virginia_CS27_South			
                    fprintf(file_out, "ProjectionGeoKey: Proj_West_Virginia_CS27_South\012");
                    break;
                  case 14731: // Proj_West_Virginia_CS83_North			
                    fprintf(file_out, "ProjectionGeoKey: Proj_West_Virginia_CS83_North\012");
                    break;
                  case 14732: // Proj_West_Virginia_CS83_South			
                    fprintf(file_out, "ProjectionGeoKey: Proj_West_Virginia_CS83_South\012");
                    break;
                  case 14801: // Proj_Wisconsin_CS27_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Wisconsin_CS27_North\012");
                    break;
                  case 14802: // Proj_Wisconsin_CS27_Central				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Wisconsin_CS27_Central\012");
                    break;
                  case 14803: // Proj_Wisconsin_CS27_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Wisconsin_CS27_South\012");
                    break;
                  case 14831: // Proj_Wisconsin_CS83_North				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Wisconsin_CS83_North\012");
                    break;
                  case 14832: // Proj_Wisconsin_CS83_Central				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Wisconsin_CS83_Central\012");
                    break;
                  case 14833: // Proj_Wisconsin_CS83_South				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Wisconsin_CS83_South\012");
                    break;
                  case 14901: // Proj_Wyoming_CS27_East				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Wyoming_CS27_East\012");
                    break;
                  case 14902: // Proj_Wyoming_CS27_East_Central			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Wyoming_CS27_East_Central\012");
                    break;
                  case 14903: // Proj_Wyoming_CS27_West_Central			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Wyoming_CS27_West_Central\012");
                    break;
                  case 14904: // Proj_Wyoming_CS27_West				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Wyoming_CS27_West\012");
                    break;
                  case 14931: // Proj_Wyoming_CS83_East				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Wyoming_CS83_East\012");
                    break;
                  case 14932: // Proj_Wyoming_CS83_East_Central			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Wyoming_CS83_East_Central\012");
                    break;
                  case 14933: // Proj_Wyoming_CS83_West_Central			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Wyoming_CS83_West_Central\012");
                    break;
                  case 14934: // Proj_Wyoming_CS83_West				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Wyoming_CS83_West\012");
                    break;
                  case 15001: // Proj_Alaska_CS27_1					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Alaska_CS27_1\012");
                    break;
                  case 15002: // Proj_Alaska_CS27_2					
                    fprintf(file_out, "ProjectionGeoKey: ProjectionGeoKey\012");
                    break;
                  case 15003: // Proj_Alaska_CS27_3					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Alaska_CS27_3\012");
                    break;
                  case 15004: // Proj_Alaska_CS27_4					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Alaska_CS27_4\012");
                    break;
                  case 15005: // Proj_Alaska_CS27_5					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Alaska_CS27_5\012");
                    break;
                  case 15006: // Proj_Alaska_CS27_6					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Alaska_CS27_6\012");
                    break;
                  case 15007: // Proj_Alaska_CS27_7					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Alaska_CS27_7\012");
                    break;
                  case 15008: // Proj_Alaska_CS27_8					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Alaska_CS27_8\012");
                    break;
                  case 15009: // Proj_Alaska_CS27_9					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Alaska_CS27_9\012");
                    break;
                  case 15010: // Proj_Alaska_CS27_10					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Alaska_CS27_10\012");
                    break;
                  case 15031: // Proj_Alaska_CS83_1					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Alaska_CS83_1\012");
                    break;
                  case 15032: // Proj_Alaska_CS83_2					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Alaska_CS83_2\012");
                    break;
                  case 15033: // Proj_Alaska_CS83_3					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Alaska_CS83_3\012");
                    break;
                  case 15034: // Proj_Alaska_CS83_4					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Alaska_CS83_4\012");
                    break;
                  case 15035: // Proj_Alaska_CS83_5					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Alaska_CS83_5\012");
                    break;
                  case 15036: // Proj_Alaska_CS83_6					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Alaska_CS83_6\012");
                    break;
                  case 15037: // Proj_Alaska_CS83_7					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Alaska_CS83_7\012");
                    break;
                  case 15038: // Proj_Alaska_CS83_8					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Alaska_CS83_8\012");
                    break;
                  case 15039: // Proj_Alaska_CS83_9					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Alaska_CS83_9\012");
                    break;
                  case 15040: // Proj_Alaska_CS83_10					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Alaska_CS83_10\012");
                    break;
                  case 15101: // Proj_Hawaii_CS27_1					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Hawaii_CS27_1\012");
                    break;
                  case 15102: // Proj_Hawaii_CS27_2					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Hawaii_CS27_2\012");
                    break;
                  case 15103: // Proj_Hawaii_CS27_3					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Hawaii_CS27_3\012");
                    break;
                  case 15104: // Proj_Hawaii_CS27_4					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Hawaii_CS27_4\012");
                    break;
                  case 15105: // Proj_Hawaii_CS27_5					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Hawaii_CS27_5\012");
                    break;
                  case 15131: // Proj_Hawaii_CS83_1					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Hawaii_CS83_1\012");
                    break;
                  case 15132: // Proj_Hawaii_CS83_2					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Hawaii_CS83_2\012");
                    break;
                  case 15133: // Proj_Hawaii_CS83_3					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Hawaii_CS83_3\012");
                    break;
                  case 15134: // Proj_Hawaii_CS83_4					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Hawaii_CS83_4\012");
                    break;
                  case 15135: // Proj_Hawaii_CS83_5					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Hawaii_CS83_5\012");
                    break;
                  case 15201: // Proj_Puerto_Rico_CS27				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Puerto_Rico_CS27\012");
                    break;
                  case 15202: // Proj_St_Croix					
                    fprintf(file_out, "ProjectionGeoKey: Proj_St_Croix\012");
                    break;
                  case 15230: // Proj_Puerto_Rico_Virgin_Is				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Puerto_Rico_Virgin_Is\012");
                    break;
                  case 15914: // Proj_BLM_14N_feet					
                    fprintf(file_out, "ProjectionGeoKey: Proj_BLM_14N_feet\012");
                    break;
                  case 15915: // Proj_BLM_15N_feet					
                    fprintf(file_out, "ProjectionGeoKey: Proj_BLM_15N_feet\012");
                    break;
                  case 15916: // Proj_BLM_16N_feet					
                    fprintf(file_out, "ProjectionGeoKey: Proj_BLM_16N_feet\012");
                    break;
                  case 15917: // Proj_BLM_17N_feet					
                    fprintf(file_out, "ProjectionGeoKey: Proj_BLM_17N_feet\012");
                    break;
                  case 17348: // Proj_Map_Grid_of_Australia_48			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Map_Grid_of_Australia_48\012");
                    break;
                  case 17349: // Proj_Map_Grid_of_Australia_49			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Map_Grid_of_Australia_49\012");
                    break;
                  case 17350: // Proj_Map_Grid_of_Australia_50			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Map_Grid_of_Australia_50\012");
                    break;
                  case 17351: // Proj_Map_Grid_of_Australia_51			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Map_Grid_of_Australia_51\012");
                    break;
                  case 17352: // Proj_Map_Grid_of_Australia_52			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Map_Grid_of_Australia_52\012");
                    break;
                  case 17353: // Proj_Map_Grid_of_Australia_53			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Map_Grid_of_Australia_53\012");
                    break;
                  case 17354: // Proj_Map_Grid_of_Australia_54			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Map_Grid_of_Australia_54\012");
                    break;
                  case 17355: // Proj_Map_Grid_of_Australia_55			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Map_Grid_of_Australia_55\012");
                    break;
                  case 17356: // Proj_Map_Grid_of_Australia_56			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Map_Grid_of_Australia_56\012");
                    break;
                  case 17357: // Proj_Map_Grid_of_Australia_57			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Map_Grid_of_Australia_57\012");
                    break;
                  case 17358: // Proj_Map_Grid_of_Australia_58			
                    fprintf(file_out, "ProjectionGeoKey: Proj_Map_Grid_of_Australia_58\012");
                    break;
                  case 17448: // Proj_Australian_Map_Grid_48				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Australian_Map_Grid_48\012");
                    break;
                  case 17449: // Proj_Australian_Map_Grid_49				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Australian_Map_Grid_49\012");
                    break;
                  case 17450: // Proj_Australian_Map_Grid_50				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Australian_Map_Grid_50\012");
                    break;
                  case 17451: // Proj_Australian_Map_Grid_51				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Australian_Map_Grid_51\012");
                    break;
                  case 17452: // Proj_Australian_Map_Grid_52				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Australian_Map_Grid_52\012");
                    break;
                  case 17453: // Proj_Australian_Map_Grid_53				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Australian_Map_Grid_53\012");
                    break;
                  case 17454: // Proj_Australian_Map_Grid_54				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Australian_Map_Grid_54\012");
                    break;
                  case 17455: // Proj_Australian_Map_Grid_55				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Australian_Map_Grid_55\012");
                    break;
                  case 17456: // Proj_Australian_Map_Grid_56				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Australian_Map_Grid_56\012");
                    break;
                  case 17457: // Proj_Australian_Map_Grid_57				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Australian_Map_Grid_57\012");
                    break;
                  case 17458: // Proj_Australian_Map_Grid_58				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Australian_Map_Grid_58\012");
                    break;
                  case 18031: // Proj_Argentina_1					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Argentina_1\012");
                    break;
                  case 18032: // Proj_Argentina_2					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Argentina_2\012");
                    break;
                  case 18033: // Proj_Argentina_3					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Argentina_3\012");
                    break;
                  case 18034: // Proj_Argentina_4					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Argentina_4\012");
                    break;
                  case 18035: // Proj_Argentina_5					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Argentina_5\012");
                    break;
                  case 18036: // Proj_Argentina_6					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Argentina_6\012");
                    break;
                  case 18037: // Proj_Argentina_7					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Argentina_7\012");
                    break;
                  case 18051: // Proj_Colombia_3W					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Colombia_3W\012");
                    break;
                  case 18052: // Proj_Colombia_Bogota				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Colombia_Bogota\012");
                    break;
                  case 18053: // Proj_Colombia_3E					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Colombia_3E\012");
                    break;
                  case 18054: // Proj_Colombia_6E					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Colombia_6E\012");
                    break;
                  case 18072: // Proj_Egypt_Red_Belt					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Egypt_Red_Belt\012");
                    break;
                  case 18073: // Proj_Egypt_Purple_Belt				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Egypt_Purple_Belt\012");
                    break;
                  case 18074: // Proj_Extended_Purple_Belt				
                    fprintf(file_out, "ProjectionGeoKey: Proj_Extended_Purple_Belt\012");
                    break;
                  case 18141: // Proj_New_Zealand_North_Island_Nat_Grid		
                    fprintf(file_out, "ProjectionGeoKey: Proj_New_Zealand_North_Island_Nat_Grid\012");
                    break;
                  case 18142: // Proj_New_Zealand_South_Island_Nat_Grid		
                    fprintf(file_out, "ProjectionGeoKey: Proj_New_Zealand_South_Island_Nat_Grid\012");
                    break;
                  case 19900: // Proj_Bahrain_Grid					
                    fprintf(file_out, "ProjectionGeoKey: Proj_Bahrain_Grid\012");
                    break;
                  case 19905: // Proj_Netherlands_E_Indies_Equatorial		
                    fprintf(file_out, "ProjectionGeoKey: Proj_Netherlands_E_Indies_Equatorial\012");
                    break;
                  case 19912: // Proj_RSO_Borneo
                    fprintf(file_out, "ProjectionGeoKey: Proj_RSO_Borneo\012");
                    break;
                  default:
                    fprintf(file_out, "ProjectionGeoKey: look-up for %d not implemented\012", lasreader->header.vlr_geo_key_entries[j].value_offset);
                  }
                  break;
                case 3075: // ProjCoordTransGeoKey
                  switch (lasreader->header.vlr_geo_key_entries[j].value_offset)
                  {
                  case 1: // CT_TransverseMercator
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_TransverseMercator\012");
                    break;
                  case 2: // CT_TransvMercator_Modified_Alaska
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_TransvMercator_Modified_Alaska\012");
                    break;
                  case 3: // CT_ObliqueMercator
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_ObliqueMercator\012");
                    break;
                  case 4: // CT_ObliqueMercator_Laborde
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_ObliqueMercator_Laborde\012");
                    break;
                  case 5: // CT_ObliqueMercator_Rosenmund
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_ObliqueMercator_Rosenmund\012");
                    break;
                  case 6: // CT_ObliqueMercator_Spherical
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_ObliqueMercator_Spherical\012");
                    break;
                  case 7: // CT_Mercator
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_Mercator\012");
                    break;
                  case 8: // CT_LambertConfConic_2SP
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_LambertConfConic_2SP\012");
                    break;
                  case 9: // CT_LambertConfConic_Helmert
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_LambertConfConic_Helmert\012");
                    break;
                  case 10: // CT_LambertAzimEqualArea
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_LambertAzimEqualArea\012");
                    break;
                  case 11: // CT_AlbersEqualArea
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_AlbersEqualArea\012");
                    break;
                  case 12: // CT_AzimuthalEquidistant
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_AzimuthalEquidistant\012");
                    break;
                  case 13: // CT_EquidistantConic
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_EquidistantConic\012");
                    break;
                  case 14: // CT_Stereographic
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_Stereographic\012");
                    break;
                  case 15: // CT_PolarStereographic
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_PolarStereographic\012");
                    break;
                  case 16: // CT_ObliqueStereographic
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_ObliqueStereographic\012");
                    break;
                  case 17: // CT_Equirectangular
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_Equirectangular\012");
                    break;
                  case 18: // CT_CassiniSoldner
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_CassiniSoldner\012");
                    break;
                  case 19: // CT_Gnomonic
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_Gnomonic\012");
                    break;
                  case 20: // CT_MillerCylindrical
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_MillerCylindrical\012");
                    break;
                  case 21: // CT_Orthographic
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_Orthographic\012");
                    break;
                  case 22: // CT_Polyconic
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_Polyconic\012");
                    break;
                  case 23: // CT_Robinson
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_Robinson\012");
                    break;
                  case 24: // CT_Sinusoidal
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_Sinusoidal\012");
                    break;
                  case 25: // CT_VanDerGrinten
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_VanDerGrinten\012");
                    break;
                  case 26: // CT_NewZealandMapGrid
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_NewZealandMapGrid\012");
                    break;
                  case 27: // CT_TransvMercator_SouthOriented
                    fprintf(file_out, "ProjCoordTransGeoKey: CT_TransvMercator_SouthOriented\012");
                    break;
                  default:
                    fprintf(file_out, "ProjCoordTransGeoKey: look-up for %d not implemented\012", lasreader->header.vlr_geo_key_entries[j].value_offset);
                  }
                  break;
                case 3076: // ProjLinearUnitsGeoKey
                  horizontal_units = lasreader->header.vlr_geo_key_entries[j].value_offset;
                  switch (lasreader->header.vlr_geo_key_entries[j].value_offset)
                  {
                  case 9001: // Linear_Meter
                    fprintf(file_out, "ProjLinearUnitsGeoKey: Linear_Meter\012");
                    break;
                  case 9002: // Linear_Foot
                    fprintf(file_out, "ProjLinearUnitsGeoKey: Linear_Foot\012");
                    break;
                  case 9003: // Linear_Foot_US_Survey
                    fprintf(file_out, "ProjLinearUnitsGeoKey: Linear_Foot_US_Survey\012");
                    break;
                  case 9004: // Linear_Foot_Modified_American
                    fprintf(file_out, "ProjLinearUnitsGeoKey: Linear_Foot_Modified_American\012");
                    break;
                  case 9005: // Linear_Foot_Clarke
                    fprintf(file_out, "ProjLinearUnitsGeoKey: Linear_Foot_Clarke\012");
                    break;
                  case 9006: // Linear_Foot_Indian
                    fprintf(file_out, "ProjLinearUnitsGeoKey: Linear_Foot_Indian\012");
                    break;
                  case 9007: // Linear_Link
                    fprintf(file_out, "ProjLinearUnitsGeoKey: Linear_Link\012");
                    break;
                  case 9008: // Linear_Link_Benoit
                    fprintf(file_out, "ProjLinearUnitsGeoKey: Linear_Link_Benoit\012");
                    break;
                  case 9009: // Linear_Link_Sears
                    fprintf(file_out, "ProjLinearUnitsGeoKey: Linear_Link_Sears\012");
                    break;
                  case 9010: // Linear_Chain_Benoit
                    fprintf(file_out, "ProjLinearUnitsGeoKey: Linear_Chain_Benoit\012");
                    break;
                  case 9011: // Linear_Chain_Sears
                    fprintf(file_out, "ProjLinearUnitsGeoKey: Linear_Chain_Sears\012");
                    break;
                  case 9012: // Linear_Yard_Sears
                    fprintf(file_out, "ProjLinearUnitsGeoKey: Linear_Yard_Sears\012");
                    break;
                  case 9013: // Linear_Yard_Indian
                    fprintf(file_out, "ProjLinearUnitsGeoKey: Linear_Yard_Indian\012");
                    break;
                  case 9014: // Linear_Fathom
                    fprintf(file_out, "ProjLinearUnitsGeoKey: Linear_Fathom\012");
                    break;
                  case 9015: // Linear_Mile_International_Nautical
                    fprintf(file_out, "ProjLinearUnitsGeoKey: Linear_Mile_International_Nautical\012");
                    break;
                  default:
                    fprintf(file_out, "ProjLinearUnitsGeoKey: look-up for %d not implemented\012", lasreader->header.vlr_geo_key_entries[j].value_offset);
                  }
                  break;
                case 3077: // ProjLinearUnitSizeGeoKey
                  if (lasreader->header.vlr_geo_double_params)
                  {
                    fprintf(file_out, "ProjLinearUnitSizeGeoKey: %.10g\012",lasreader->header.vlr_geo_double_params[lasreader->header.vlr_geo_key_entries[j].value_offset]);
                  }
                  break;
                case 3078: // ProjStdParallel1GeoKey
                  if (lasreader->header.vlr_geo_double_params)
                  {
                    fprintf(file_out, "ProjStdParallel1GeoKey: %.10g\012",lasreader->header.vlr_geo_double_params[lasreader->header.vlr_geo_key_entries[j].value_offset]);
                  }
                  break;
                case 3079: // ProjStdParallel2GeoKey
                  if (lasreader->header.vlr_geo_double_params)
                  {
                    fprintf(file_out, "ProjStdParallel2GeoKey: %.10g\012",lasreader->header.vlr_geo_double_params[lasreader->header.vlr_geo_key_entries[j].value_offset]);
                  }
                  break;        
                case 3080: // ProjNatOriginLongGeoKey
                  if (lasreader->header.vlr_geo_double_params)
                  {
                    fprintf(file_out, "ProjNatOriginLongGeoKey: %.10g\012",lasreader->header.vlr_geo_double_params[lasreader->header.vlr_geo_key_entries[j].value_offset]);
                  }
                  break;
                case 3081: // ProjNatOriginLatGeoKey
                  if (lasreader->header.vlr_geo_double_params)
                  {
                    fprintf(file_out, "ProjNatOriginLatGeoKey: %.10g\012",lasreader->header.vlr_geo_double_params[lasreader->header.vlr_geo_key_entries[j].value_offset]);
                  }
                  break;
                case 3082: // ProjFalseEastingGeoKey
                  if (lasreader->header.vlr_geo_double_params)
                  {
                    fprintf(file_out, "ProjFalseEastingGeoKey: %.10g\012",lasreader->header.vlr_geo_double_params[lasreader->header.vlr_geo_key_entries[j].value_offset]);
                  }
                  break;
                case 3083: // ProjFalseNorthingGeoKey
                  if (lasreader->header.vlr_geo_double_params)
                  {
                    fprintf(file_out, "ProjFalseNorthingGeoKey: %.10g\012",lasreader->header.vlr_geo_double_params[lasreader->header.vlr_geo_key_entries[j].value_offset]);
                  }
                  break;
                case 3084: // ProjFalseOriginLongGeoKey
                  if (lasreader->header.vlr_geo_double_params)
                  {
                    fprintf(file_out, "ProjFalseOriginLongGeoKey: %.10g\012",lasreader->header.vlr_geo_double_params[lasreader->header.vlr_geo_key_entries[j].value_offset]);
                  }
                  break;
                case 3085: // ProjFalseOriginLatGeoKey
                  if (lasreader->header.vlr_geo_double_params)
                  {
                    fprintf(file_out, "ProjFalseOriginLatGeoKey: %.10g\012",lasreader->header.vlr_geo_double_params[lasreader->header.vlr_geo_key_entries[j].value_offset]);
                  }
                  break;
                case 3086: // ProjFalseOriginEastingGeoKey
                  if (lasreader->header.vlr_geo_double_params)
                  {
                    fprintf(file_out, "ProjFalseOriginEastingGeoKey: %.10g\012",lasreader->header.vlr_geo_double_params[lasreader->header.vlr_geo_key_entries[j].value_offset]);
                  }
                  break;
                case 3087: // ProjFalseOriginNorthingGeoKey
                  if (lasreader->header.vlr_geo_double_params)
                  {
                    fprintf(file_out, "ProjFalseOriginNorthingGeoKey: %.10g\012",lasreader->header.vlr_geo_double_params[lasreader->header.vlr_geo_key_entries[j].value_offset]);
                  }
                  break;
                case 3088: // ProjCenterLongGeoKey
                  if (lasreader->header.vlr_geo_double_params)
                  {
                    fprintf(file_out, "ProjCenterLongGeoKey: %.10g\012",lasreader->header.vlr_geo_double_params[lasreader->header.vlr_geo_key_entries[j].value_offset]);
                  }
                  break;
                case 3089: // ProjCenterLatGeoKey
                  if (lasreader->header.vlr_geo_double_params)
                  {
                    fprintf(file_out, "ProjCenterLatGeoKey: %.10g\012",lasreader->header.vlr_geo_double_params[lasreader->header.vlr_geo_key_entries[j].value_offset]);
                  }
                  break;
                case 3090: // ProjCenterEastingGeoKey
                  if (lasreader->header.vlr_geo_double_params)
                  {
                    fprintf(file_out, "ProjCenterEastingGeoKey: %.10g\012",lasreader->header.vlr_geo_double_params[lasreader->header.vlr_geo_key_entries[j].value_offset]);
                  }
                  break;
                case 3091: // ProjCenterNorthingGeoKey
                  if (lasreader->header.vlr_geo_double_params)
                  {
                    fprintf(file_out, "ProjCenterNorthingGeoKey: %.10g\012",lasreader->header.vlr_geo_double_params[lasreader->header.vlr_geo_key_entries[j].value_offset]);
                  }
                  break;
                case 3092: // ProjScaleAtNatOriginGeoKey
                  if (lasreader->header.vlr_geo_double_params)
                  {
                    fprintf(file_out, "ProjScaleAtNatOriginGeoKey: %.10g\012",lasreader->header.vlr_geo_double_params[lasreader->header.vlr_geo_key_entries[j].value_offset]);
                  }
                  break;
                case 3093: // ProjScaleAtCenterGeoKey
                  if (lasreader->header.vlr_geo_double_params)
                  {
                    fprintf(file_out, "ProjScaleAtCenterGeoKey: %.10g\012",lasreader->header.vlr_geo_double_params[lasreader->header.vlr_geo_key_entries[j].value_offset]);
                  }
                  break;
                case 3094: // ProjAzimuthAngleGeoKey
                  if (lasreader->header.vlr_geo_double_params)
                  {
                    fprintf(file_out, "ProjAzimuthAngleGeoKey: %.10g\012",lasreader->header.vlr_geo_double_params[lasreader->header.vlr_geo_key_entries[j].value_offset]);
                  }
                  break;
                case 3095: // ProjStraightVertPoleLongGeoKey
                  if (lasreader->header.vlr_geo_double_params)
                  {
                    fprintf(file_out, "ProjStraightVertPoleLongGeoKey: %.10g\012",lasreader->header.vlr_geo_double_params[lasreader->header.vlr_geo_key_entries[j].value_offset]);
                  }
                  break;
                case 4096: // VerticalCSTypeGeoKey 
                  switch (lasreader->header.vlr_geo_key_entries[j].value_offset)
                  {
                  case 5001: // VertCS_Airy_1830_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_Airy_1830_ellipsoid\012");
                    break;
                  case 5002: // VertCS_Airy_Modified_1849_ellipsoid 
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_Airy_Modified_1849_ellipsoid\012");
                    break;
                  case 5003: // VertCS_ANS_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_ANS_ellipsoid\012");
                    break;
                  case 5004: // VertCS_Bessel_1841_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_Bessel_1841_ellipsoid\012");
                    break;
                  case 5005: // VertCS_Bessel_Modified_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_Bessel_Modified_ellipsoid\012");
                    break;
                  case 5006: // VertCS_Bessel_Namibia_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_Bessel_Namibia_ellipsoid\012");
                    break;
                  case 5007: // VertCS_Clarke_1858_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_Clarke_1858_ellipsoid\012");
                    break;
                  case 5008: // VertCS_Clarke_1866_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_Clarke_1866_ellipsoid\012");
                    break;
                  case 5010: // VertCS_Clarke_1880_Benoit_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_Clarke_1880_Benoit_ellipsoid\012");
                    break;
                  case 5011: // VertCS_Clarke_1880_IGN_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_Clarke_1880_IGN_ellipsoid\012");
                    break;
                  case 5012: // VertCS_Clarke_1880_RGS_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_Clarke_1880_RGS_ellipsoid\012");
                    break;
                  case 5013: // VertCS_Clarke_1880_Arc_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_Clarke_1880_Arc_ellipsoid\012");
                    break;
                  case 5014: // VertCS_Clarke_1880_SGA_1922_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_Clarke_1880_SGA_1922_ellipsoid\012");
                    break;
                  case 5015: // VertCS_Everest_1830_1937_Adjustment_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_Everest_1830_1937_Adjustment_ellipsoid\012");
                    break;
                  case 5016: // VertCS_Everest_1830_1967_Definition_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_Everest_1830_1967_Definition_ellipsoid\012");
                    break;
                  case 5017: // VertCS_Everest_1830_1975_Definition_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_Everest_1830_1975_Definition_ellipsoid\012");
                    break;
                  case 5018: // VertCS_Everest_1830_Modified_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_Everest_1830_Modified_ellipsoid\012");
                    break;
                  case 5019: // VertCS_GRS_1980_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_GRS_1980_ellipsoid\012");
                    break;
                  case 5020: // VertCS_Helmert_1906_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_Helmert_1906_ellipsoid\012");
                    break;
                  case 5021: // VertCS_INS_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_INS_ellipsoid\012");
                    break;
                  case 5022: // VertCS_International_1924_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_International_1924_ellipsoid\012");
                    break;
                  case 5023: // VertCS_International_1967_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_International_1967_ellipsoid\012");
                    break;
                  case 5024: // VertCS_Krassowsky_1940_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_Krassowsky_1940_ellipsoid\012");
                    break;
                  case 5025: // VertCS_NWL_9D_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_NWL_9D_ellipsoid\012");
                    break;
                  case 5026: // VertCS_NWL_10D_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_NWL_10D_ellipsoid\012");
                    break;
                  case 5027: // VertCS_Plessis_1817_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_Plessis_1817_ellipsoid\012");
                    break;
                  case 5028: // VertCS_Struve_1860_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_Struve_1860_ellipsoid\012");
                    break;
                  case 5029: // VertCS_War_Office_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_War_Office_ellipsoid\012");
                    break;
                  case 5030: // VertCS_WGS_84_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_WGS_84_ellipsoid\012");
                    break;
                  case 5031: // VertCS_GEM_10C_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_GEM_10C_ellipsoid\012");
                    break;
                  case 5032: // VertCS_OSU86F_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_OSU86F_ellipsoid\012");
                    break;
                  case 5033: // VertCS_OSU91A_ellipsoid
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_OSU91A_ellipsoid\012");
                    break;
                  case 5101: // VertCS_Newlyn
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_Newlyn\012");
                    break;
                  case 5102: // VertCS_North_American_Vertical_Datum_1929
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_North_American_Vertical_Datum_1929\012");
                    break;
                  case 5103: // VertCS_North_American_Vertical_Datum_1988
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_North_American_Vertical_Datum_1988\012");
                    break;
                  case 5104: // VertCS_Yellow_Sea_1956
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_Yellow_Sea_1956\012");
                    break;
                  case 5105: // VertCS_Baltic_Sea
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_Baltic_Sea\012");
                    break;
                  case 5106: // VertCS_Caspian_Sea
                    fprintf(file_out, "VerticalCSTypeGeoKey: VertCS_Caspian_Sea\012");
                    break;
                  default:
                    fprintf(file_out, "VerticalCSTypeGeoKey: look-up for %d not implemented\012", lasreader->header.vlr_geo_key_entries[j].value_offset);
                  }
                  break;
                case 4097: // VerticalCitationGeoKey
                  if (lasreader->header.vlr_geo_ascii_params)
                  {
                    char dummy[256];
                    strncpy(dummy, &(lasreader->header.vlr_geo_ascii_params[lasreader->header.vlr_geo_key_entries[j].value_offset]), lasreader->header.vlr_geo_key_entries[j].count);
                    dummy[lasreader->header.vlr_geo_key_entries[j].count-1] = '\0';
                    fprintf(file_out, "VerticalCitationGeoKey: %s\012",dummy);
                  }
                  break;
                case 4098: // VerticalDatumGeoKey 
                  fprintf(file_out, "VerticalDatumGeoKey: Vertical Datum Codes %d\012",lasreader->header.vlr_geo_key_entries[j].value_offset);
                  break;
                case 4099: // VerticalUnitsGeoKey 
                  switch (lasreader->header.vlr_geo_key_entries[j].value_offset)
                  {
                  case 9001: // Linear_Meter
                    fprintf(file_out, "VerticalUnitsGeoKey: Linear_Meter\012");
                    break;
                  case 9002: // Linear_Foot
                    fprintf(file_out, "VerticalUnitsGeoKey: Linear_Foot\012");
                    break;
                  case 9003: // Linear_Foot_US_Survey
                    fprintf(file_out, "VerticalUnitsGeoKey: Linear_Foot_US_Survey\012");
                    break;
                  case 9004: // Linear_Foot_Modified_American
                    fprintf(file_out, "VerticalUnitsGeoKey: Linear_Foot_Modified_American\012");
                    break;
                  case 9005: // Linear_Foot_Clarke
                    fprintf(file_out, "VerticalUnitsGeoKey: Linear_Foot_Clarke\012");
                    break;
                  case 9006: // Linear_Foot_Indian
                    fprintf(file_out, "VerticalUnitsGeoKey: Linear_Foot_Indian\012");
                    break;
                  case 9007: // Linear_Link
                    fprintf(file_out, "VerticalUnitsGeoKey: Linear_Link\012");
                    break;
                  case 9008: // Linear_Link_Benoit
                    fprintf(file_out, "VerticalUnitsGeoKey: Linear_Link_Benoit\012");
                    break;
                  case 9009: // Linear_Link_Sears
                    fprintf(file_out, "VerticalUnitsGeoKey: Linear_Link_Sears\012");
                    break;
                  case 9010: // Linear_Chain_Benoit
                    fprintf(file_out, "VerticalUnitsGeoKey: Linear_Chain_Benoit\012");
                    break;
                  case 9011: // Linear_Chain_Sears
                    fprintf(file_out, "VerticalUnitsGeoKey: Linear_Chain_Sears\012");
                    break;
                  case 9012: // Linear_Yard_Sears
                    fprintf(file_out, "VerticalUnitsGeoKey: Linear_Yard_Sears\012");
                    break;
                  case 9013: // Linear_Yard_Indian
                    fprintf(file_out, "VerticalUnitsGeoKey: Linear_Yard_Indian\012");
                    break;
                  case 9014: // Linear_Fathom
                    fprintf(file_out, "VerticalUnitsGeoKey: Linear_Fathom\012");
                    break;
                  case 9015: // Linear_Mile_International_Nautical
                    fprintf(file_out, "VerticalUnitsGeoKey: Linear_Mile_International_Nautical\012");
                    break;
                  default:
                    fprintf(file_out, "VerticalUnitsGeoKey: look-up for %d not implemented\012", lasreader->header.vlr_geo_key_entries[j].value_offset);
                  }
                  break;
                default:
                  fprintf(file_out, "key ID %d not implemented\012", lasreader->header.vlr_geo_key_entries[j].key_id);
                }
              }
            }
          }
          else if (lasheader->vlrs[i].record_id == 34736) // GeoDoubleParamsTag
          {
            fprintf(file_out, "    GeoDoubleParamsTag (number of doubles %d)\012", lasreader->header.vlrs[i].record_length_after_header/8);
            fprintf(file_out, "      ");
            for (int j = 0; j < lasreader->header.vlrs[i].record_length_after_header/8; j++)
            {
              fprintf(file_out, "%lg ", lasheader->vlr_geo_double_params[j]);
            }
            fprintf(file_out, "\012");
          }
          else if (lasheader->vlrs[i].record_id == 34737) // GeoAsciiParamsTag
          {
            fprintf(file_out, "    GeoAsciiParamsTag (number of characters %d)\012", lasreader->header.vlrs[i].record_length_after_header);
            fprintf(file_out, "      ");
            for (int j = 0; j < lasreader->header.vlrs[i].record_length_after_header; j++)
            {
              fprintf(file_out, "%c", lasheader->vlr_geo_ascii_params[j]);
            }
            fprintf(file_out, "\012");
          }
          else if (lasheader->vlrs[i].record_id == 2111) // OGC MATH TRANSFORM WKT
          {
            fprintf(file_out, "    OGC MATH TRANSFORM WKT:\012");
            fprintf(file_out, "    %s\012", lasreader->header.vlrs[i].data);
          }
          else if (lasheader->vlrs[i].record_id == 2112) // OGC COORDINATE SYSTEM WKT
          {
            fprintf(file_out, "    OGC COORDINATE SYSTEM WKT:\012");
            fprintf(file_out, "    %s\012", lasreader->header.vlrs[i].data);
          }
        }
        else if ((strcmp(lasheader->vlrs[i].user_id, "LASF_Spec") == 0) && (lasheader->vlrs[i].data != 0))
        {
          if (lasheader->vlrs[i].record_id == 0) // ClassificationLookup
          {
            LASvlr_classification* vlr_classification = (LASvlr_classification*)lasheader->vlrs[i].data;
            for (int j = 0; j < 256; j++)
            {
              fprintf(file_out, "    %d %s", vlr_classification[j].class_number, vlr_classification[j].description);
            }
          }
          else if (lasheader->vlrs[i].record_id == 2) // Histogram
          {
          }
          else if (lasheader->vlrs[i].record_id == 3) // TextAreaDescription
          {
            fprintf(file_out, "    ");
            for (int j = 0; j < lasheader->vlrs[i].record_length_after_header; j++)
            {
              fprintf(file_out, "%c", lasheader->vlrs[i].data[j]);
            }
            fprintf(file_out, "\012");
          }
          else if (lasheader->vlrs[i].record_id == 4) // ExtraBytes
          {
            const char* name_table[10] = { "unsigned char", "char", "unsigned short", "short", "unsigned long", "long", "unsigned long long", "long long", "float", "double" };
            fprintf(file_out, "    Extra Byte Descriptions\012");
            for (int j = 0; j < lasheader->vlrs[i].record_length_after_header; j += 192)
            {
              if (lasheader->vlrs[i].data[j+2])
              {
                int type = ((I32)(lasheader->vlrs[i].data[j+2])-1)%10;
                int dim = ((I32)(lasheader->vlrs[i].data[j+2])-1)/10+1;
                if (file_out)
                {
                  fprintf(file_out, "      data type: %d (%s), name \"%s\", description: \"%s\"", (I32)(lasheader->vlrs[i].data[j+2]), name_table[type], (char*)(lasheader->vlrs[i].data + j + 4), (char*)(lasheader->vlrs[i].data + j + 160));
                  if (lasheader->vlrs[i].data[j+3] & 0x02) // if min is set
                  {
                    fprintf(file_out, ", min:");
                    for (int k = 0; k < dim; k++)
                    {
                      if (type < 8)
                      {
#ifdef _WIN32
                        fprintf(file_out, " %I64d", ((I64*)(lasheader->vlrs[i].data + j + 64))[k]);
#else
                        fprintf(file_out, ", %lld", ((I64*)(lasheader->vlrs[i].data + j + 64))[k]);
#endif
                      }
                      else
                      {
                        fprintf(file_out, " %g", ((F64*)(lasheader->vlrs[i].data + j + 64))[k]);
                      }
                    }
                  }
                  if (lasheader->vlrs[i].data[j+3] & 0x04) // if max is set
                  {
                    fprintf(file_out, ", max:");
                    for (int k = 0; k < dim; k++)
                    {
                      if (type < 8)
                      {
#ifdef _WIN32
                        fprintf(file_out, " %I64d", ((I64*)(lasheader->vlrs[i].data + j + 88))[k]);
#else
                        fprintf(file_out, ", %lld", ((I64*)(lasheader->vlrs[i].data + j + 88))[k]);
#endif
                      }
                      else
                      {
                        fprintf(file_out, " %g", ((F64*)(lasheader->vlrs[i].data + j + 88))[k]);
                      }
                    }
                  }
                  if (lasheader->vlrs[i].data[j+3] & 0x08) fprintf(file_out, ", scale: %g", ((F64*)(lasheader->vlrs[i].data + j + 112))[0]);
                  if (lasheader->vlrs[i].data[j+3] & 0x08) // if scale is set
                  {
                    fprintf(file_out, ", scale:");
                    for (int k = 0; k < dim; k++)
                    {
                      fprintf(file_out, " %g", ((F64*)(lasheader->vlrs[i].data + j + 112))[k]);
                    }
                  }
                  if (lasheader->vlrs[i].data[j+3] & 0x10) // if offset is set
                  {
                    fprintf(file_out, ", offset:");
                    for (int k = 0; k < dim; k++)
                    {
                      fprintf(file_out, " %g", ((F64*)(lasheader->vlrs[i].data + j + 136))[k]);
                    }
                  }
                  fprintf(file_out, "\012");
                }
              }
              else
              {
                fprintf(file_out, "      data type: 0 (untyped bytes), size: %d\012", lasheader->vlrs[i].data[j+3]);
              }
            }
          }
          else if ((lasheader->vlrs[i].record_id >= 100) && (lasheader->vlrs[i].record_id < 355)) // WavePacketDescriptor
          {
            LASvlr_wave_packet_descr* vlr_wave_packet_descr = (LASvlr_wave_packet_descr*)lasheader->vlrs[i].data;
            fprintf(file_out, "  index %d bits/sample %d compression %d samples %u temporal %u gain %lg, offset %lg\012", lasheader->vlrs[i].record_id-99, vlr_wave_packet_descr->getBitsPerSample(), vlr_wave_packet_descr->getCompressionType(), vlr_wave_packet_descr->getNumberOfSamples(), vlr_wave_packet_descr->getTemporalSpacing(), vlr_wave_packet_descr->getDigitizerGain(), vlr_wave_packet_descr->getDigitizerOffset());
          }
        }
      }
    }

    if (file_out && !no_variable_header)
    {
      for (int i = 0; i < (int)lasheader->number_of_extended_variable_length_records; i++)
      {
        fprintf(file_out, "extended variable length header record %d of %d:\012", i+1, (int)lasheader->number_of_extended_variable_length_records);
        fprintf(file_out, "  reserved             %d\012", lasreader->header.evlrs[i].reserved);
        fprintf(file_out, "  user ID              '%s'\012", lasreader->header.evlrs[i].user_id);
        fprintf(file_out, "  record ID            %d\012", lasreader->header.evlrs[i].record_id);
#ifdef _WIN32
        fprintf(file_out, "  length after header  %I64d\012", lasreader->header.evlrs[i].record_length_after_header);
#else
        fprintf(file_out, "  length after header  %lld\012", lasreader->header.evlrs[i].record_length_after_header);
#endif
        fprintf(file_out, "  description          '%s'\012", lasreader->header.evlrs[i].description);
        if (strcmp(lasheader->vlrs[i].user_id, "LASF_Projection") == 0)
        {
          if (lasheader->evlrs[i].record_id == 2111) // OGC MATH TRANSFORM WKT
          {
            fprintf(file_out, "    OGC MATH TRANSFORM WKT:\012");
            fprintf(file_out, "    %s\012", lasreader->header.evlrs[i].data);
          }
          else if (lasheader->evlrs[i].record_id == 2112) // OGC COORDINATE SYSTEM WKT
          {
            fprintf(file_out, "    OGC COORDINATE SYSTEM WKT:\012");
            fprintf(file_out, "    %s\012", lasreader->header.evlrs[i].data);
          }
        }
      }
    }

    if (file_out && !no_header)
    {
      if (lasheader->user_data_after_header_size) fprintf(file_out, "the header is followed by %u user-defined bytes\012", lasheader->user_data_after_header_size);

      if (lasheader->laszip)
      {
        fprintf(file_out, "LASzip compression (version %d.%dr%d c%d", lasheader->laszip->version_major, lasheader->laszip->version_minor, lasheader->laszip->version_revision, lasheader->laszip->compressor);
        if (lasheader->laszip->compressor == LASZIP_COMPRESSOR_CHUNKED) fprintf(file_out, " %d):", lasheader->laszip->chunk_size);
        else fprintf(file_out, "):");
        for (i = 0; i < (int)lasheader->laszip->num_items; i++) fprintf(file_out, " %s %d", lasheader->laszip->items[i].get_name(), lasheader->laszip->items[i].version);
        fprintf(file_out, "\012");
      }
      if (lasheader->vlr_lastiling)
      {
        fprintf(file_out, "LAStiling (idx %d, lvl %d, sub %d, bbox %g %g %g %g%s%s)\n", 
          lasheader->vlr_lastiling->level_index, 
          lasheader->vlr_lastiling->level,
          lasheader->vlr_lastiling->implicit_levels,
          lasheader->vlr_lastiling->min_x,
          lasheader->vlr_lastiling->min_y,
          lasheader->vlr_lastiling->max_x,
          lasheader->vlr_lastiling->max_y,
          (lasheader->vlr_lastiling->buffer ? ", buffer" : ""),
          (lasheader->vlr_lastiling->reversible ? ", reversible" : ""));
      }
      if (lasheader->vlr_lasoriginal)
      {
        fprintf(file_out, "LASoriginal (npoints %u, bbox %g %g %g %g %g %g)\n", 
          (U32)lasheader->vlr_lasoriginal->number_of_point_records, 
          lasheader->vlr_lasoriginal->min_x,
          lasheader->vlr_lasoriginal->min_y,
          lasheader->vlr_lasoriginal->min_z,
          lasheader->vlr_lasoriginal->max_x,
          lasheader->vlr_lasoriginal->max_y,
          lasheader->vlr_lasoriginal->max_z);
      }
    }

    // loop over the points
    F64 enlarged_min_x = lasreader->header.min_x - 0.25 * lasreader->header.x_scale_factor;
    F64 enlarged_max_x = lasreader->header.max_x + 0.25 * lasreader->header.x_scale_factor;
    F64 enlarged_min_y = lasreader->header.min_y - 0.25 * lasreader->header.y_scale_factor;
    F64 enlarged_max_y = lasreader->header.max_y + 0.25 * lasreader->header.y_scale_factor;
    F64 enlarged_min_z = lasreader->header.min_z - 0.25 * lasreader->header.z_scale_factor;
    F64 enlarged_max_z = lasreader->header.max_z + 0.25 * lasreader->header.z_scale_factor;
    LASsummary lassummary;

    if (check_points)
    {
      I64 num_last_returns = 0;
      I64 outside_bounding_box = 0;
      LASoccupancyGrid* lasoccupancygrid = 0;

      if (compute_density)
      {
        lasoccupancygrid = new LASoccupancyGrid(horizontal_units > 9001 ? 6.0f : 2.0f);
      }

      if (file_out && !no_min_max) fprintf(file_out, "reporting minimum and maximum for all LAS point record entries ...\012");

      while (lasreader->read_point())
      {
        if (check_outside)
        {
          if (!lasreader->point.inside_bounding_box(enlarged_min_x, enlarged_min_y, enlarged_min_z, enlarged_max_x, enlarged_max_y, enlarged_max_z))
          {
            outside_bounding_box++;
            if (file_out && report_outside)
            {
              fprintf(file_out, "%u t %g x %g y %g z %g (%d of %d) i %d c %d s %d %u p %d d %d e %d\012", (U32)(lasreader->p_count-1), lasreader->point.gps_time, lasreader->point.get_x(), lasreader->point.get_y(), lasreader->point.get_z(), lasreader->point.return_number, lasreader->point.number_of_returns_of_given_pulse, lasreader->point.intensity, lasreader->point.classification, lasreader->point.scan_angle_rank, lasreader->point.user_data, lasreader->point.point_source_ID, lasreader->point.scan_direction_flag, lasreader->point.edge_of_flight_line);
            }
          }
        }

        lassummary.add(&lasreader->point);

        if (lasreader->point.return_number >= lasreader->point.number_of_returns_of_given_pulse)
        {
          if (lasoccupancygrid)
          {
            lasoccupancygrid->add(&lasreader->point);
          }
          num_last_returns++;
        }

        if (lashistogram.active())
        {
          lashistogram.add(&lasreader->point);
        }

#ifdef _WIN32
        if (file_out && progress && (lasreader->p_count % progress) == 0) fprintf(file_out, " ... processed %I64d points ...\012", lasreader->p_count);
#else
        if (file_out && progress && (lasreader->p_count % progress) == 0) fprintf(file_out, " ... processed %lld points ...\012", lasreader->p_count);
#endif
      }
      if (file_out && !no_min_max)
      {
        fprintf(file_out, "  X %10d %10d\012",lassummary.min.x, lassummary.max.x);
        fprintf(file_out, "  Y %10d %10d\012",lassummary.min.y, lassummary.max.y);
        fprintf(file_out, "  Z %10d %10d\012",lassummary.min.z, lassummary.max.z);
        fprintf(file_out, "  intensity %d %d\012",lassummary.min.intensity, lassummary.max.intensity);
        fprintf(file_out, "  edge_of_flight_line %d %d\012",lassummary.min.edge_of_flight_line, lassummary.max.edge_of_flight_line);
        fprintf(file_out, "  scan_direction_flag %d %d\012",lassummary.min.scan_direction_flag, lassummary.max.scan_direction_flag);
        fprintf(file_out, "  number_of_returns_of_given_pulse %d %d\012",lassummary.min.number_of_returns_of_given_pulse, lassummary.max.number_of_returns_of_given_pulse);
        fprintf(file_out, "  return_number                    %d %d\012",lassummary.min.return_number, lassummary.max.return_number);
        fprintf(file_out, "  classification  %5d %5d\012",lassummary.min.classification, lassummary.max.classification);
        fprintf(file_out, "  scan_angle_rank %5d %5d\012",lassummary.min.scan_angle_rank, lassummary.max.scan_angle_rank);
        fprintf(file_out, "  user_data       %5d %5d\012",lassummary.min.user_data, lassummary.max.user_data);
        fprintf(file_out, "  point_source_ID %5d %5d\012",lassummary.min.point_source_ID, lassummary.max.point_source_ID);
        if (lasreader->point.have_gps_time)
        {
          fprintf(file_out, "  gps_time %f %f\012",lassummary.min.gps_time, lassummary.max.gps_time);
          if ((lasreader->header.global_encoding & 1) == 0)
          {
            if (lassummary.min.gps_time < 0.0 || lassummary.max.gps_time > 604800.0)
            {
              fprintf(file_out, "WARNING: range violates GPS week time specified by global encoding bit 0\012");
            }
          }
          else if (gps_week)
          {
            I32 week_min = (I32)(lassummary.min.gps_time/604800.0 + 1653.4391534391534391534391534392);
            I32 week_max = (I32)(lassummary.max.gps_time/604800.0 + 1653.4391534391534391534391534392);
            I32 secs_min = week_min*604800 - 1000000000;
            I32 secs_max = week_max*604800 - 1000000000;
            fprintf(file_out, "  gps_week %d %d\012", week_min, week_max);
            fprintf(file_out, "  gps_secs_of_week %f %f\012", (lassummary.min.gps_time-secs_min), (lassummary.max.gps_time-secs_max));
          }
        }
        if (lasreader->point.have_rgb)
        {
          fprintf(file_out, "  Color R %d %d\012", lassummary.min.rgb[0], lassummary.max.rgb[0]);
          fprintf(file_out, "        G %d %d\012", lassummary.min.rgb[1], lassummary.max.rgb[1]);
          fprintf(file_out, "        B %d %d\012", lassummary.min.rgb[2], lassummary.max.rgb[2]);
        }
        if (lasreader->point.have_nir)
        {
          fprintf(file_out, "      NIR %d %d\012", lassummary.min.rgb[3], lassummary.max.rgb[3]);
        }
        if (lasreader->point.have_wavepacket)
        {
          fprintf(file_out, "  Wavepacket Index    %d %d\012", lassummary.min.wavepacket.getIndex(), lassummary.max.wavepacket.getIndex());
#ifdef _WIN32
          fprintf(file_out, "             Offset   %I64d %I64d\012", lassummary.min.wavepacket.getOffset(), lassummary.max.wavepacket.getOffset());
#else
          fprintf(file_out, "             Offset   %lld %lld\012", lassummary.min.wavepacket.getOffset(), lassummary.max.wavepacket.getOffset());
#endif
          fprintf(file_out, "             Size     %d %d\012", lassummary.min.wavepacket.getSize(), lassummary.max.wavepacket.getSize());
          fprintf(file_out, "             Location %g %g\012", lassummary.min.wavepacket.getLocation(), lassummary.max.wavepacket.getLocation());
          fprintf(file_out, "             Xt       %g %g\012", lassummary.min.wavepacket.getXt(), lassummary.max.wavepacket.getXt());
          fprintf(file_out, "             Yt       %g %g\012", lassummary.min.wavepacket.getYt(), lassummary.max.wavepacket.getYt());
          fprintf(file_out, "             Zt       %g %g\012", lassummary.min.wavepacket.getZt(), lassummary.max.wavepacket.getZt());
        }
        if (lasreader->point.extended_point_type)
        {
          fprintf(file_out, "  extended_number_of_returns_of_given_pulse %d %d\012",lassummary.min.extended_number_of_returns_of_given_pulse, lassummary.max.extended_number_of_returns_of_given_pulse);
          fprintf(file_out, "  extended_return_number                    %d %d\012",lassummary.min.extended_return_number, lassummary.max.extended_return_number);
          fprintf(file_out, "  extended_classification  %6d %6d\012",lassummary.min.extended_classification, lassummary.max.extended_classification);
          fprintf(file_out, "  extended_scan_angle      %6.3f %6.3f\012",0.006*lassummary.min.extended_scan_angle, 0.006*lassummary.max.extended_scan_angle);
        }
        if (((number_of_point_records == 0) && (lasheader->number_of_point_records > 0)) || ((number_of_points_by_return0 == 0) && (lasheader->number_of_points_by_return[0] > 0)))
        {
          fprintf(file_out, "re-reporting LAS header entries populated during read pass:\012");
          if ((number_of_point_records == 0) && (lasheader->number_of_point_records > 0)) fprintf(file_out, "  number of point records    %u\012", lasheader->number_of_point_records);
          if ((number_of_points_by_return0 == 0) && (lasheader->number_of_points_by_return[0] > 0)) fprintf(file_out, "  number of points by return %u %u %u %u %u\012", lasheader->number_of_points_by_return[0], lasheader->number_of_points_by_return[1], lasheader->number_of_points_by_return[2], lasheader->number_of_points_by_return[3], lasheader->number_of_points_by_return[4]);
          fprintf(file_out, "  min x y z                  "); lidardouble2string(printstring, lasheader->min_x, lasheader->x_scale_factor); fprintf(file_out, "%s ", printstring); lidardouble2string(printstring, lasheader->min_y, lasheader->y_scale_factor); fprintf(file_out, "%s ", printstring); lidardouble2string(printstring, lasheader->min_z, lasheader->z_scale_factor); fprintf(file_out, "%s\012", printstring);
          fprintf(file_out, "  max x y z                  "); lidardouble2string(printstring, lasheader->max_x, lasheader->x_scale_factor); fprintf(file_out, "%s ", printstring); lidardouble2string(printstring, lasheader->max_y, lasheader->y_scale_factor); fprintf(file_out, "%s ", printstring); lidardouble2string(printstring, lasheader->max_z, lasheader->z_scale_factor); fprintf(file_out, "%s\012", printstring);
        }
      }
      if (file_out && outside_bounding_box)
      {
#ifdef _WIN32
        fprintf(file_out, "WARNING: %I64d points outside of header bounding box\012", outside_bounding_box);
#else
        fprintf(file_out, "WARNING: %lld points outside of header bounding box\012", outside_bounding_box);
#endif
      }
      if (lashistogram.active())
      {
        lashistogram.report(file_out);
      }
      if (file_out && lasoccupancygrid)
      {
#ifdef _WIN32
        fprintf(file_out, "number of last returns: %I64d\012", num_last_returns);
#else
        fprintf(file_out, "number of last returns: %lld\012", num_last_returns);
#endif
        if (num_last_returns)
        {
          if (horizontal_units == 9001)
          {
            fprintf(file_out, "covered area in square meters/kilometers: %d/%.2f\012", 4*lasoccupancygrid->get_num_occupied(), 0.000004*lasoccupancygrid->get_num_occupied());
            fprintf(file_out, "point density: all returns %.2f last only %.2f (per square meter)\012", ((F64)lasreader->p_count/(4.0*lasoccupancygrid->get_num_occupied())), ((F64)num_last_returns/(4.0*lasoccupancygrid->get_num_occupied())));
            fprintf(file_out, "      spacing: all returns %.2f last only %.2f (in meters)\012", sqrt(4.0*lasoccupancygrid->get_num_occupied()/(F64)lasreader->p_count), sqrt(4.0*lasoccupancygrid->get_num_occupied()/(F64)num_last_returns));
          }
          else if (horizontal_units == 9002)
          {
            fprintf(file_out, "covered area in square feet/miles: %d/%.2f\012", 36*lasoccupancygrid->get_num_occupied(),1.2913223e-6*lasoccupancygrid->get_num_occupied());
            fprintf(file_out, "point density: all returns %.2f last only %.2f (per square foot)\012", ((F64)lasreader->p_count/(36.0*lasoccupancygrid->get_num_occupied())), ((F64)num_last_returns/(36.0*lasoccupancygrid->get_num_occupied())));
            fprintf(file_out, "      spacing: all returns %.2f last only %.2f (in feet)\012", sqrt(36.0*lasoccupancygrid->get_num_occupied()/(F64)lasreader->p_count), sqrt(36.0*lasoccupancygrid->get_num_occupied()/(F64)num_last_returns));
          }
          else if (horizontal_units == 9003)
          {
            fprintf(file_out, "covered area in square survey feet: %d\012", 36*lasoccupancygrid->get_num_occupied());
            fprintf(file_out, "point density: all returns %.2f last only %.2f (per square survey foot)\012", ((F64)lasreader->p_count/(36.0*lasoccupancygrid->get_num_occupied())), ((F64)num_last_returns/(36.0*lasoccupancygrid->get_num_occupied())));
            fprintf(file_out, "      spacing: all returns %.2f last only %.2f (in survey feet)\012", sqrt(36.0*lasoccupancygrid->get_num_occupied()/(F64)lasreader->p_count), sqrt(36.0*lasoccupancygrid->get_num_occupied()/(F64)num_last_returns));
          }
          else
          {
            fprintf(file_out, "covered area in square units/kilounits: %d/%.2f\012", 4*lasoccupancygrid->get_num_occupied(), 0.000004*lasoccupancygrid->get_num_occupied());
            fprintf(file_out, "point density: all returns %.2f last only %.2f (per square units)\012", ((F64)lasreader->p_count/(4.0*lasoccupancygrid->get_num_occupied())), ((F64)num_last_returns/(4.0*lasoccupancygrid->get_num_occupied())));
            fprintf(file_out, "      spacing: all returns %.2f last only %.2f (in units)\012", sqrt(4.0*lasoccupancygrid->get_num_occupied()/(F64)lasreader->p_count), sqrt(4.0*lasoccupancygrid->get_num_occupied()/(F64)num_last_returns));
          }
        }
        delete lasoccupancygrid;
      }
    }

    lasreader->close();

    FILE* file = 0;

    if (repair_bb || repair_counters || change_header)
    {
      if (lasreadopener.get_file_name())
      {
        file = fopen(lasreadopener.get_file_name(), "rb+");
        if (file == 0)
        {
          fprintf (stderr, "ERROR: could reopen file '%s' for changing header\n", lasreadopener.get_file_name());
          repair_bb = repair_counters = change_header = false;
        }
      }
      else
      {
        fprintf(stderr, "ERROR: cannot change header of piped input\n");
        repair_bb = repair_counters = change_header = false;
      }
    }

    if (change_header)
    {
      if (set_file_source_id != -1)
      {
        U16 file_source_id = U16_CLAMP(set_file_source_id);
        fseek(file, 4, SEEK_SET);
        fwrite(&file_source_id, sizeof(U16), 1, file);
      }
      if (set_global_encoding != -1)
      {
        U16 global_encoding = U16_CLAMP(set_global_encoding);
        fseek(file, 6, SEEK_SET);
        fwrite(&global_encoding, sizeof(U16), 1, file);
      }
      if (set_version_major != -1)
      {
        fseek(file, 24, SEEK_SET);
        fwrite(&set_version_major, sizeof(I8), 1, file);
      }
      if (set_version_minor != -1)
      {
        fseek(file, 25, SEEK_SET);
        fwrite(&set_version_minor, sizeof(I8), 1, file);
      }
      if (set_system_identifier)
      {
        fseek(file, 26, SEEK_SET);
        fwrite(set_system_identifier, sizeof(I8), 32, file);
      }
      if (set_generating_software)
      {
        fseek(file, 58, SEEK_SET);
        fwrite(set_generating_software, sizeof(I8), 32, file);
      }
      if (set_creation_day != -1)
      {
        U16 creation_day = U16_CLAMP(set_creation_day);
        fseek(file, 90, SEEK_SET);
        fwrite(&creation_day, sizeof(U16), 1, file);
      }
      if (set_creation_year != -1)
      {
        U16 creation_year = U16_CLAMP(set_creation_year);
        fseek(file, 92, SEEK_SET);
        fwrite(&creation_year, sizeof(U16), 1, file);
      }
      if (set_header_size)
      {
        fseek(file, 94, SEEK_SET);
        fwrite(&set_header_size, sizeof(U16), 1, file);
      }
      if (set_offset_to_point_data)
      {
        fseek(file, 96, SEEK_SET);
        fwrite(&set_offset_to_point_data, sizeof(U32), 1, file);
      }
      if (set_point_data_format != -1)
      {
        U8 point_data_format = U8_CLAMP(set_point_data_format);
        fseek(file, 104, SEEK_SET);
        fwrite(&point_data_format, sizeof(U8), 1, file);
      }
      if (set_point_data_record_length != -1)
      {
        U16 point_data_record_length = U16_CLAMP(set_point_data_record_length);
        fseek(file, 105, SEEK_SET);
        fwrite(&point_data_record_length, sizeof(U16), 1, file);
      }
      if (set_scale)
      {
        fseek(file, 131, SEEK_SET);
        fwrite(set_scale, 3*sizeof(F64), 1, file);
      }
      if (set_offset)
      {
        fseek(file, 155, SEEK_SET);
        fwrite(set_offset, 3*sizeof(F64), 1, file);
      }
      if (set_bounding_box)
      {
        fseek(file, 179, SEEK_SET);
        fwrite(set_bounding_box, 6*sizeof(F64), 1, file);
      }
      if (set_start_of_waveform_data_packet_record != -1)
      {
        fseek(file, 227, SEEK_SET);
        fwrite(&set_start_of_waveform_data_packet_record, sizeof(I64), 1, file);
      }
    }

    if (check_points)
    {
      if (lassummary.number_of_point_records != lasheader->number_of_point_records)
      {
        if (repair_counters)
        {
          if (lassummary.number_of_point_records > U32_MAX)
          {
#ifdef _WIN32
            fprintf(file_out, "WARNING: real number of points (%I64d) exceeds 4,294,967,295. cannot repair. too big.\n", lassummary.number_of_point_records);
#else
            fprintf(file_out, "WARNING: real number of points (%lld) exceeds 4,294,967,295. cannot repair. too big.\n", lassummary.number_of_point_records);
#endif
          }
          else
          {
            U32 number_of_point_records = (U32)lassummary.number_of_point_records;
            fseek(file, 107, SEEK_SET);
            fwrite(&number_of_point_records, sizeof(U32), 1, file);
            if (file_out)
            {
              fprintf(file_out, "real number of points (%u) is different from header number of points (%u). it was repaired. \n", number_of_point_records, lasheader->number_of_point_records);
            }
          }
        }
        else
        {
          if (!no_min_max && file_out)
          {
            if (lassummary.number_of_point_records > U32_MAX)
            {
#ifdef _WIN32
              fprintf(file_out, "WARNING: real number of points (%I64d) exceeds 4,294,967,295.\n", lassummary.number_of_point_records);
#else
              fprintf(file_out, "WARNING: real number of points (%lld) exceeds 4,294,967,295.\n", lassummary.number_of_point_records);
#endif
            }
            else
            {
#ifdef _WIN32
              fprintf(file_out, "real number of points (%I64d) is different from header number of points (%u).\n", lassummary.number_of_point_records, lasheader->number_of_point_records);
#else
              fprintf(file_out, "real number of points (%lld) is different from header number of points (%u).\n", lassummary.number_of_point_records, lasheader->number_of_point_records);
#endif
            }
          }
        }
      }
      else
      {
        if (repair_counters)
        {
          if (file_out)
          {
            fprintf(file_out, "number of points in header is correct.\n");
          }
        }
      }

      bool wrong_entry = false;
      bool was_set = false;
      for (i = 1; i < 6; i++) if (lasheader->number_of_points_by_return[i-1] != lassummary.number_of_points_by_return[i]) wrong_entry = true;
      for (i = 1; i < 6; i++) if (lasheader->number_of_points_by_return[i-1]) was_set = true;
      if (wrong_entry)
      {
        if (repair_counters)
        {
          U32 number_of_points_by_return[5];
          for (i = 1; i < 6; i++) number_of_points_by_return[i-1] = (U32)lassummary.number_of_points_by_return[i];
          fseek(file, 111, SEEK_SET);
          fwrite(&(number_of_points_by_return[0]), sizeof(U32), 5, file);
          if (file_out)
          {
            fprintf(file_out, "number of points by return %s", was_set ? "is different than reported in header:" : "was not set in header:"); 
#ifdef _WIN32
            for (i = 1; i < 6; i++) fprintf(file_out, " %I64d", lassummary.number_of_points_by_return[i]); 
#else
            for (i = 1; i < 6; i++) fprintf(file_out, " %lld", lassummary.number_of_points_by_return[i]); 
#endif
            fprintf(file_out, ". repairing ...\n");
          }
        }
        else
        {
          if (!no_min_max && file_out)
          {
            fprintf(file_out, "number of points by return %s", was_set ? "is different than reported in header:" : "was not set in header:"); 
#ifdef _WIN32
            for (i = 1; i < 6; i++) fprintf(file_out, " %I64d", lassummary.number_of_points_by_return[i]); 
#else
            for (i = 1; i < 6; i++) fprintf(file_out, " %lld", lassummary.number_of_points_by_return[i]); 
#endif
            fprintf(file_out, "\n");
          }
        }
      }
      else
      {
        if (repair_counters)
        {
          if (file_out)
          {
            fprintf(file_out, "number of points by return in header is correct.\n");
          }
        }
      }

      if (file_out && !no_min_max)
      {
#ifdef _WIN32
        if (lassummary.number_of_points_by_return[0]) fprintf(file_out, "WARNING: there are %I64d points with return number 0\n", lassummary.number_of_points_by_return[0]); 
        if (lassummary.number_of_points_by_return[6]) fprintf(file_out, "WARNING: there are %I64d points with return number 6\n", lassummary.number_of_points_by_return[6]); 
        if (lassummary.number_of_points_by_return[7]) fprintf(file_out, "WARNING: there are %I64d points with return number 7\n", lassummary.number_of_points_by_return[7]); 
#else
        if (lassummary.number_of_points_by_return[0]) fprintf(file_out, "WARNING: there are %lld points with return number 0\n", lassummary.number_of_points_by_return[0]); 
        if (lassummary.number_of_points_by_return[6]) fprintf(file_out, "WARNING: there are %lld points with return number 6\n", lassummary.number_of_points_by_return[6]); 
        if (lassummary.number_of_points_by_return[7]) fprintf(file_out, "WARNING: there are %lld points with return number 7\n", lassummary.number_of_points_by_return[7]); 
#endif

        wrong_entry = false;
        for (i = 1; i < 8; i++) if (lassummary.number_of_returns_of_given_pulse[i]) wrong_entry = true;
        if (wrong_entry)
        {
          fprintf(file_out, "overview over number of returns of given pulse:"); 
#ifdef _WIN32
          for (i = 1; i < 8; i++) fprintf(file_out, " %I64d", lassummary.number_of_returns_of_given_pulse[i]);
#else
          for (i = 1; i < 8; i++) fprintf(file_out, " %lld", lassummary.number_of_returns_of_given_pulse[i]);
#endif
          fprintf(file_out, "\n"); 
        }

#ifdef _WIN32
        if (lassummary.number_of_returns_of_given_pulse[0]) fprintf(file_out, "WARNING: there are %I64d points with a number of returns of given pulse of 0\n", lassummary.number_of_returns_of_given_pulse[0]); 
#else
        if (lassummary.number_of_returns_of_given_pulse[0]) fprintf(file_out, "WARNING: there are %lld points with a number of returns of given pulse of 0\n", lassummary.number_of_returns_of_given_pulse[0]); 
#endif

        wrong_entry = false;
        for (i = 0; i < 32; i++) if (lassummary.classification[i]) wrong_entry = true;
        if (lassummary.classification_synthetic || lassummary.classification_keypoint ||  lassummary.classification_withheld) wrong_entry = true;

        if (wrong_entry)
        {
          fprintf(file_out, "histogram of classification of points:\n"); 
#ifdef _WIN32
          for (i = 0; i < 32; i++) if (lassummary.classification[i]) fprintf(file_out, " %15I64d  %s (%u)\n", lassummary.classification[i], LASpointClassification[i], i);
          if (lassummary.classification_synthetic) fprintf(file_out, " +-> flagged as synthetic: %I64d\n", lassummary.classification_synthetic);
          if (lassummary.classification_keypoint) fprintf(file_out,  " +-> flagged as keypoints: %I64d\n", lassummary.classification_keypoint);
          if (lassummary.classification_withheld) fprintf(file_out,  " +-> flagged as withheld:  %I64d\n", lassummary.classification_withheld);
#else
          for (i = 0; i < 32; i++) if (lassummary.classification[i]) fprintf(file_out, " %15lld %s (%u)\n", lassummary.classification[i], LASpointClassification[i], i);
          if (lassummary.classification_synthetic) fprintf(file_out, " +-> flagged as synthetic: %lld\n", lassummary.classification_synthetic);
          if (lassummary.classification_keypoint) fprintf(file_out,  " +-> flagged as keypoints: %lld\n", lassummary.classification_keypoint);
          if (lassummary.classification_withheld) fprintf(file_out,  " +-> flagged as withheld:  %lld\n", lassummary.classification_withheld);
#endif
        }
      }

      double value;
      if (repair_bb)
      {
        wrong_entry = false;
        if (lasheader->get_x(lassummary.max.x) != lasheader->max_x) wrong_entry = true;
        if (lasheader->get_x(lassummary.min.x) != lasheader->min_x) wrong_entry = true;
        if (lasheader->get_y(lassummary.max.y) != lasheader->max_y) wrong_entry = true;
        if (lasheader->get_y(lassummary.min.y) != lasheader->min_y) wrong_entry = true;
        if (lasheader->get_z(lassummary.max.z) != lasheader->max_z) wrong_entry = true;
        if (lasheader->get_z(lassummary.min.z) != lasheader->min_z) wrong_entry = true;
        if (wrong_entry)
        {
          fseek(file, 179, SEEK_SET);
          value = lasheader->get_x(lassummary.max.x); fwrite(&value, sizeof(double), 1, file);
          value = lasheader->get_x(lassummary.min.x); fwrite(&value, sizeof(double), 1, file);
          value = lasheader->get_y(lassummary.max.y); fwrite(&value, sizeof(double), 1, file);
          value = lasheader->get_y(lassummary.min.y); fwrite(&value, sizeof(double), 1, file);
          value = lasheader->get_z(lassummary.max.z); fwrite(&value, sizeof(double), 1, file);
          value = lasheader->get_z(lassummary.min.z); fwrite(&value, sizeof(double), 1, file);
          if (file_out) fprintf(file_out, "bounding box was repaired.\n");
        }
        else
        {
          if (file_out) fprintf(file_out, "bounding box is correct.\n");
        }
      }
      else
      {
        value = lasheader->get_x(lassummary.max.x);
        if (value > enlarged_max_x)
        {
          if (file_out) fprintf(file_out, "real max x larger than header max x by %lf\n", value - lasheader->max_x);
        }
        value = lasheader->get_x(lassummary.min.x);
        if (value < enlarged_min_x)
        {
          if (file_out) fprintf(file_out, "real min x smaller than header min x by %lf\n", lasheader->min_x - value);
        }
        value = lasheader->get_y(lassummary.max.y);
        if (value > enlarged_max_y)
        {
          if (file_out) fprintf(file_out, "real max y larger than header max y by %lf\n", value - lasheader->max_y);
        }
        value = lasheader->get_y(lassummary.min.y);
        if (value < enlarged_min_y)
        {
          if (file_out) fprintf(file_out, "real min y smaller than header min y by %lf\n", lasheader->min_y - value);
        }
        value = lasheader->get_z(lassummary.max.z);
        if (value > enlarged_max_z)
        {
          if (file_out) fprintf(file_out, "real max z larger than header max z by %lf\n", value - lasheader->max_z);
        }
        value = lasheader->get_z(lassummary.min.z);
        if (value < enlarged_min_z)
        {
          if (file_out) fprintf(file_out, "real min z smaller than header min z by %lf\n", lasheader->min_z - value);
        }
      }
    }

    if (file_out && (file_out != stdout) && (file_out != stderr)) fclose(file_out);

    delete lasreader;
    if (file) fclose(file);
  }

  if (set_system_identifier) delete [] set_system_identifier;
  if (set_generating_software) delete [] set_generating_software;
  if (set_bounding_box) delete [] set_bounding_box;
  if (set_offset) delete [] set_offset;
  if (set_scale) delete [] set_scale;

  byebye(false, argc==1);

  return 0;
}
