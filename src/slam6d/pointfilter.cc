/*
 * pointfilter implementation
 *
 * Copyright (C) by the 3DTK contributors
 *
 * Released under the GPL version 3.
 *
 */

#include "slam6d/pointfilter.h"

using std::string;
using std::map;
#include <sstream>
using std::stringstream;
#include <stdexcept>
using std::runtime_error;

#include <iostream>
using std::cout;
using std::endl;

#include <cmath>


map<string, Checker* (*)(const string&)>*
PointFilter::factory = new map<string, Checker* (*)(const string&)>;


PointFilter::PointFilter() :
  m_changed(true), m_checker(0)
{ }

PointFilter::PointFilter(const std::string& params) :
  m_changed(true), m_checker(0)
{
  size_t start = 0, end = string::npos;
  while((end = params.find(' ', start)) != string::npos) {
    // extract the word (start-end+1) without the space (-1)
    string key(params.substr(start, start-end));
    end++;
    // get the second word position
    start = params.find(' ', end);
    // insert
    m_params[key] = params.substr(end, (start-end));
    // advance to the character after space
    if(start != string::npos)
      start++;
    else
      break;
  }
}

PointFilter::~PointFilter()
{
  if(m_checker)
    delete m_checker;
}

PointFilter& PointFilter::setRange(double maxDist, double minDist)
{
  m_changed = true;
  stringstream s_max; s_max << maxDist;
  stringstream s_min; s_min << minDist;
  m_params["rangemax"] = s_max.str();
  m_params["rangemin"] = s_min.str();
  return *this;
}

PointFilter& PointFilter::setHeight(double top, double bottom)
{
  m_changed = true;
  stringstream s_top; s_top << top;
  stringstream s_bottom; s_bottom << bottom;
  m_params["heighttop"] = s_top.str();
  m_params["heightbottom"] = s_bottom.str();
  return *this;
}

PointFilter& PointFilter::setCustom(const std::string& customFilterStr)
{
  m_changed = true;
  std::string cFiltStr(customFilterStr);
  m_params["customFilter"] = cFiltStr;
  return *this;
}

PointFilter& PointFilter::setRangeMutator(double range)
{
  m_changed = true;
  stringstream s_range; s_range << range;
  m_params["rangemutation"] = s_range.str();
  return *this;
}

std::string PointFilter::getParams()
{
  stringstream s;
  for(map<string, string>::iterator it = m_params.begin();
      it != m_params.end();
      ++it) {
    s << (*it).first << " " << (*it).second << " ";
  }
  return s.str();
}

void PointFilter::createCheckers()
{
  // delete the outdated ones
  if(m_checker) {
    delete m_checker;
    m_checker = 0;
  }
  
  // create new ones
  Checker** current = &m_checker;
  for(map<string, string>::iterator it = m_params.begin();
      it != m_params.end();
      ++it) {
    *current = (*factory)[it->first](it->second);
    // if a Checker has been successfully created advance to
    // its pointer in the chain
    if(*current) {
      current = &((*current)->m_next);
    }
  }
}

Checker::Checker() :
  m_next(0)
{
}

Checker::~Checker()
{
  if (m_next)
    delete m_next;
}

// create factory instaces for key string to factory function mapping
namespace {
  CheckerFactory<CheckerRangeMax> max("rangemax");
  CheckerFactory<CheckerRangeMin> min("rangemin");
  CheckerFactory<CheckerCustom> customFilter("customFilter");
  CheckerFactory<CheckerHeightTop> top("heighttop");
  CheckerFactory<CheckerHeightBottom> bottom("heightbottom");
  CheckerFactory<RangeMutator> range_mutation("rangemutation");
}

CheckerRangeMax::CheckerRangeMax(const std::string& value) {
  stringstream s(value);
  s >> m_max;
  // default value: no check
  if(m_max <= 0.0) throw runtime_error("No range filter needed.");
  m_max *= m_max;
}

bool CheckerRangeMax::test(double* point) {
  if(point[0]*point[0] + point[1]*point[1] + point[2]*point[2] < m_max)
    return true;
  return false;
}

CheckerRangeMin::CheckerRangeMin(const std::string& value) {
  stringstream s(value);
  s >> m_min;
  // default value: no check
  if(m_min <= 0.0) throw runtime_error("No range filter needed.");
  m_min *= m_min;
}

bool CheckerRangeMin::test(double* point) {
  if(point[0]*point[0] + point[1]*point[1] + point[2]*point[2] > m_min)
    return true;
  return false;
}

CheckerHeightTop::CheckerHeightTop(const std::string& value) {
  stringstream s(value);
  s >> m_top;
}

bool CheckerHeightTop::test(double* point) {
  if (point[1] < m_top)
    return true;
  return false;
}

CheckerHeightBottom::CheckerHeightBottom(const std::string& value) {
  stringstream s(value);
  s >> m_bottom;
}

bool CheckerHeightBottom::test(double* point) {
  if (point[1] > m_bottom)
    return true;
  return false;
}

CheckerCustom::CheckerCustom(const std::string& value) {
    try{
		// every custom filter description is defined as 
		// {filterMode};{nrOfParams}[;param1][;param2][...]
      std::string str(value);
      size_t pos = str.find_first_of(";");
      stringstream ss(str.substr(0, pos));
      ss >> filterMode;
      
      str = str.substr(pos + 1);
      pos = str.find_first_of(";");
	  stringstream ss2(str.substr(0, pos));
	  ss2 >> nrOfParam;

	  if (nrOfParam > 0){
		  custParamsSet = true;
		  custFiltParams = new double[nrOfParam];
		  // initialize to 0
		  for (size_t i = 0; i < nrOfParam; i++)
		  {
			  custFiltParams[i] = 0.0;
		  }
	  }
	  // parse parameters for filter
	  for (size_t i = 0; i < nrOfParam; i++)
	  {
		  str = str.substr(pos + 1);
		  pos = str.find_first_of(";");
		  if (pos == std::string::npos){
			  if (i != nrOfParam - 1){
			  // less than indicated parameters have been provided, error!
			  throw runtime_error("Error parsing arguments for CustomFilter.");
			  }
			  else {
				  // last param is not ended with ';'
				  stringstream ss3(str);
				  ss3 >> custFiltParams[i];
				  break;
			  }
		  }
		  stringstream ss3(str.substr(0, pos));
		  ss3 >> custFiltParams[i];
	  }

    }
    catch (...){
        throw runtime_error("Error parsing arguments for CustomFilter.");
    }
}

CheckerCustom::~CheckerCustom(){
	if (custParamsSet)
      delete[] custFiltParams;
}

bool CheckerCustom::test(double* point) {
	bool filterTest = false;

	try{
		switch (filterMode)
		{
		case 0:
			// Custom Filter 0: symetrical, axis-parallel cuboid
			// all values inside the cuboid will be filtered (filterTest = false)
			// parameters: xFilterRange yFilterRange zFilterRange
			if (abs(point[0]) > custFiltParams[0] || abs(point[1]) > custFiltParams[1] || abs(point[2]) > custFiltParams[2])
				filterTest = true;
			break;
		case 1:
			// Custom Filter 1: asymetrical axis-parallel cuboid
			// all values inside the cuboid will be filtered
			// parameters: xFilterRangeLow xFilterRangeHigh yFilterRangeLow yFilterRangeHigh zFilterRangeLow zFilterRangeHigh
			if (point[0] < custFiltParams[0] || point[0] > custFiltParams[1]
				|| point[1] < custFiltParams[2] || point[1] > custFiltParams[3]
				|| point[2] < custFiltParams[4] || point[2] > custFiltParams[5])
				filterTest = true;
			break;
		case 2:
			// As Custom Filter 1: asymetrical axis-parallel cuboid, with additional max range limitation
			// parameters: xFilterRangeLow xFilterRangeHigh yFilterRangeLow yFilterRangeHigh zFilterRangeLow zFilterRangeHigh maxRange 
			if (point[0] < custFiltParams[0] || point[0] > custFiltParams[1]
				|| point[1] < custFiltParams[2] || point[1] > custFiltParams[3]
				|| point[2] < custFiltParams[4] || point[2] > custFiltParams[5]){
				if ((point[0] * point[0] + point[1] * point[1] + point[2] * point[2]) < (custFiltParams[6] * custFiltParams[6])){
					filterTest = true;
				}
				break;
			}
		default:
			filterTest = true;
			break;
		}
	}
	catch (...){
		// Error occured - deactivate filter
		filterTest = true;
	}

	return filterTest;
}


RangeMutator::RangeMutator(const std::string& value) {
  stringstream s(value);
  s >> m_range;
}

bool RangeMutator::test(double* point) {
  double orig_range = sqrt(point[0]*point[0]
                           + point[1]*point[1]
                           + point[2]*point[2]);
  double scale_mutation = m_range / orig_range;
  point[0] *= scale_mutation;
  point[1] *= scale_mutation;
  point[2] *= scale_mutation;

  return true;
}
