/**
 * @file FeatureMatchSetGroup.h
 * @brief A list of FeatureMatchSets
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
 */
#ifndef __FEATURE_MATCH_SET_GROUP_H__
#define __FEATURE_MATCH_SET_GROUP_H__

#include <list>
#include "FeatureMatchSet.h"

class FeatureMatchSetGroup
{
 public:
  FeatureMatchSetGroup();
  virtual ~FeatureMatchSetGroup ();
  
  void serialize(const char * filename);
  FeatureMatchSetGroup(const char * filename);
  
  std::list<FeatureMatchSet> matchsets;
};

#endif /* __FEATURE_MATCH_SET_GROUP_H__ */

