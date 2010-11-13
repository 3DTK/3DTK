/*
 * PanoramaMap.h
 *
 *  Created on: Feb 20, 2010
 *      Author: darko
 */

#ifndef PANORAMAMAP_H_
#define PANORAMAMAP_H_

#include "SuperPixel.h"
#include "PolarPointCloud.h"

#include <string>

class PanoramaMap {
public:
//	PanoramaMap(SuperPixel **data, int width, int height) {this->data = data; this->width = width; this->height = height;}
	PanoramaMap(PolarPointCloud* cloud, int width, int height);
    PanoramaMap(const char* filename); //deserialize
	PanoramaMap();

    void serialize(const char* filename);

	void toJpeg(std::string filename);

	virtual ~PanoramaMap();


public:

	double mina; //horizontal angle
	double maxa; //horizontal angle
	
	double minb; //vertical angle
	double maxb; //vertical angle

	std::string scanid;
	SuperPixel **data;
	int width;
	int height;
};

#endif /* PANORAMAMAP_H_ */
