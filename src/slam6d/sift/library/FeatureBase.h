/*
 * DataMap.h
 *
 *  Created on: April, 15, 2010
 *      Author: darko makreshanski
 */

#ifndef FEATURE_BASE_H_
#define FEATURE_BASE_H_

#include <vector>
#include <fstream>

class FeatureBase {
public:
	FeatureBase(){}
	FeatureBase(double nx, double ny, double sc, double ori) {
		x = nx;
		y = ny;
		scale = sc; 
		orientation = ori; 
	}
	void serialize(std::ofstream& ostr);
	FeatureBase(std::ifstream& istr);
	virtual ~FeatureBase() {}
	double x;
	double y;
	double scale;
	double orientation;
};

#endif /* CONTROL_POINT_H_ */
