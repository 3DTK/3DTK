/**
 * @file FeatureSet.h
 * @brief A vector of Features
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
 */
#ifndef __FEATURE_SET_H__
#define __FEATURE_SET_H__

#include <string>
#include <vector>
#include <fstream>
#include "Feature.h"

class FeatureSet
{
 public:
  FeatureSet () {}
  virtual ~FeatureSet () {}

  void serialize(const char* filename);
  FeatureSet(const char* filename);
  std::string scanid;
  std::vector<Feature> features;	
};

#endif /* __FEATURE_SET_H__ */
