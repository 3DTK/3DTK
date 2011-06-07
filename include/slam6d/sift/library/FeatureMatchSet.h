/**
 * @file FeatureMatchSet.h
 * @brief A vector of FeatureMatch
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
 */
#ifndef __FEATURE_MATCH_SET_H__
#define __FEATURE_MATCH_SET_H__

#include <string>
#include <vector>
#include "FeatureMatch.h"

#include "PanoramaMap.h"
#include "ScanTransform.h"

class FeatureMatchSet
{
 public:
  FeatureMatchSet();
  virtual ~FeatureMatchSet ();
  
  void serialize(std::ofstream &out);
  FeatureMatchSet(std::ifstream &in);
  
  bool operator<(FeatureMatchSet &c) {return false;}
  
  std::string firstscan;
  std::string secondscan;
  std::vector<FeatureMatch> matches;	
};

#endif /* __FEATURE_MATCH_SET_H__ */

