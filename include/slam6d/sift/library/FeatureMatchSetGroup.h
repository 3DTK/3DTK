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

