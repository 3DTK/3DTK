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

