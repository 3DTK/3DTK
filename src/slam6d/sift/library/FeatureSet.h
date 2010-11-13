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
		FeatureSet (std::string scanid, std::vector<Feature> features);
		virtual ~FeatureSet () {}
		void serialize(const char* filename);
		FeatureSet(const char* filename);
		std::string scanid;
		std::vector<Feature> features;	
};

#endif /* __FEATURE_SET_H__ */

