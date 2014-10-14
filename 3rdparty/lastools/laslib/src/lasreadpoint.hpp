/*
===============================================================================

  FILE:  lasreadpoint.hpp
  
  CONTENTS:
  
    Common interface for the classes that read points raw or compressed.

  PROGRAMMERS:

    martin.isenburg@gmail.com  -  http://rapidlasso.com

  COPYRIGHT:

    (c) 2007-2012, martin isenburg, rapidlasso - fast tools to catch reality

    This is free software; you can redistribute and/or modify it under the
    terms of the GNU Lesser General Licence as published by the Free Software
    Foundation. See the COPYING file for more information.

    This software is distributed WITHOUT ANY WARRANTY and without even the
    implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  
  CHANGE HISTORY:
  
    6 October 2011 -- large file support & reading with missing chunk table
    9 May 2011 -- the chunked compressor now allows variable chunk sizes
    25 April 2011 -- added chunked laszip for random access decompression
    10 January 2011 -- licensing change for LGPL release and liblas integration
    7 December 2010 -- adapted from LASpointReader for better code modularity
    3 December 2010 -- updated to (somewhat) support LAS format 1.3
    7 September 2008 -- updated to support LAS format 1.2 
    22 February 2007 -- created about an hour before henna's birthday
  
===============================================================================
*/
#ifndef LAS_READ_POINT_H
#define LAS_READ_POINT_H

#include "mydefs.hpp"
#include "laszip.hpp"
#include "bytestreamin.hpp"

class LASreadItem;
class EntropyDecoder;

class LASreadPoint
{
public:
  LASreadPoint();
  ~LASreadPoint();

  // should only be called *once*
  BOOL setup(const U32 num_items, const LASitem* items, const LASzip* laszip=0);

  BOOL init(ByteStreamIn* instream);
  BOOL seek(const U32 current, const U32 target);
  BOOL read(U8* const * point);
  BOOL done();

private:
  ByteStreamIn* instream;
  U32 num_readers;
  LASreadItem** readers;
  LASreadItem** readers_raw;
  LASreadItem** readers_compressed;
  EntropyDecoder* dec;
  // used for chunking
  U32 chunk_size;
  U32 chunk_count;
  U32 current_chunk;
  U32 number_chunks;
  U32 tabled_chunks;
  I64* chunk_starts;
  U32* chunk_totals;
  BOOL read_chunk_table();
  U32 search_chunk_table(const U32 index, const U32 lower, const U32 upper);
  // used for seeking
  I64 point_start;
  U32 point_size;
  U8** seek_point;
};

#endif
