/**
 * @file FeatureMatch.h
 * @brief A match of FeaturBase from two scans
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
 */
#ifndef __FEATURE_MATCH_H__
#define __FEATURE_MATCH_H__

#include "FeatureBase.h"

class FeatureMatch
{
 public:
  FeatureMatch (FeatureBase first, FeatureBase second, double dist1, double dist2)
    {
      this->first = first;
      this->second = second;
      this->dist1 = dist1;
      this->dist2 = dist2;
    }
  
  void serialize(std::ofstream &out);
  FeatureMatch(std::ifstream &in);
  
  virtual ~FeatureMatch () {}
  FeatureBase first;
  FeatureBase second;
  double dist1;
  double dist2;
};

#endif /* __FEATURE_MATCH_H__ */

