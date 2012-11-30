/*
===============================================================================

  FILE:  lasfilter.hpp
  
  CONTENTS:
  
    Filters LIDAR points based on certain criteria being true (or not).

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
  
    25 December 2010 -- created after swinging in Mara's hammock for hours
  
===============================================================================
*/
#ifndef LAS_FILTER_HPP
#define LAS_FILTER_HPP

#include "lasdefinitions.hpp"

class LAScriterion
{
public:
  virtual const char * name() const = 0;
  virtual int get_command(char* string) const = 0;
  virtual BOOL filter(const LASpoint* point) = 0;
  virtual void reset(){};
  virtual ~LAScriterion(){};
};

class LASfilter
{
public:

  void usage() const;
  void clean();
  BOOL parse(int argc, char* argv[]);
  I32 unparse(char* string) const;
  inline BOOL active() const { return (num_criteria != 0); };

  void addClipCircle(F64 x, F64 y, F64 radius);
  void addClipBox(F64 min_x, F64 min_y, F64 min_z, F64 max_x, F64 max_y, F64 max_z);
  void addScanDirectionChangeOnly();

  BOOL filter(const LASpoint* point);
  void reset();

  LASfilter();
  ~LASfilter();

private:

  void add_criterion(LAScriterion* criterion);
  U32 num_criteria;
  U32 alloc_criteria;
  LAScriterion** criteria;
  int* counters;
};

#endif
