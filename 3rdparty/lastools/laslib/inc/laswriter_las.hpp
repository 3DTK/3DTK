/*
===============================================================================

  FILE:  laswriter_las.hpp
  
  CONTENTS:
  
    Writes LIDAR points to the LAS format (Version 1.x , April 29, 2008).

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
  
    5 November 2011 -- changed default IO buffer size with setvbuf() to 65536
    8 May 2011 -- added an option for variable chunking via chunk()
    23 April 2011 -- added additional open parameters to support chunking
    21 January 2011 -- adapted from laswriter to create abstract reader class
    3 December 2010 -- updated to (somewhat) support LAS format 1.3
    7 September 2008 -- updated to support LAS format 1.2 
    21 February 2007 -- created after eating Sarah's veggies with peanutsauce
  
===============================================================================
*/
#ifndef LAS_WRITER_LAS_HPP
#define LAS_WRITER_LAS_HPP

#include "laswriter.hpp"

#include <stdio.h>

#ifdef LZ_WIN32_VC6
#include <fstream.h>
#else
#include <istream>
#include <fstream>
using namespace std;
#endif

class ByteStreamOut;
class LASwritePoint;

class LASwriterLAS : public LASwriter
{
public:

  BOOL refile(FILE* file);

  BOOL open(const LASheader* header, U32 compressor=LASZIP_COMPRESSOR_NONE, I32 requested_version=0, I32 chunk_size=50000);
  BOOL open(const char* file_name, const LASheader* header, U32 compressor=LASZIP_COMPRESSOR_NONE, I32 requested_version=0, I32 chunk_size=50000, U32 io_buffer_size=65536);
  BOOL open(FILE* file, const LASheader* header, U32 compressor=LASZIP_COMPRESSOR_NONE, I32 requested_version=0, I32 chunk_size=50000);
  BOOL open(ostream& ostream, const LASheader* header, U32 compressor=LASZIP_COMPRESSOR_NONE, I32 requested_version=0, I32 chunk_size=50000);

  BOOL write_point(const LASpoint* point);
  BOOL chunk();

  BOOL update_header(const LASheader* header, BOOL use_inventory=TRUE, BOOL update_extra_bytes=FALSE);
  I64 close(BOOL update_npoints=true);

  LASwriterLAS();
  ~LASwriterLAS();

private:
  BOOL open(ByteStreamOut* stream, const LASheader* header, U32 compressor, I32 requested_version, I32 chunk_size);
  ByteStreamOut* stream;
  LASwritePoint* writer;
  FILE* file;
  I64 header_start_position;
  BOOL writing_las_1_4;
};

#endif
