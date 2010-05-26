/*
 * PolarPointCloud.h
 *
 *  Created on: Feb 20, 2010
 *      Author: darko
 */

#ifndef POLARPOINTCLOUD_H_
#define POLARPOINTCLOUD_H_

#include "PolarPoint.h"
#include "PointC.h"
#include <vector>
#include <string>

//using namespace std;


class PolarPointCloud {
public:
	PolarPointCloud(std::string scanid, std::vector<PolarPoint> *data, long length, double mina, double maxa, double minb, double maxb);
	PolarPointCloud(PointC* data, long length);
	virtual ~PolarPointCloud();

	void serialize(const char* filename);
	PolarPointCloud(const char* filename, int readevery = 1);

	const std::vector<PolarPoint> *getData();
    long getLength() const;
//    void setData(std::vector<PolarPoint> &data, long length);

public:
	
	double mina; //horizontal angle
	double maxa; //horizontal angle
	
	double minb; //vertical angle
	double maxb; //vertical angle

	std::string scanid;
	std::vector<PolarPoint> *data;
	long length;
};

#endif /* POLARPOINTCLOUD_H_ */
