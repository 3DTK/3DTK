/*
 * PointCloud.h
 *
 *  Created on: Mar 14, 2010
 *      Author: darko
 */

#ifndef POINTCLOUD_H_
#define POINTCLOUD_H_

#include "PolarPointCloud.h"
#include "PointC.h"


class PointCloud {
public:
	PointCloud(PointC* data, long length);
	PointCloud(PolarPointCloud *ppc);
	virtual ~PointCloud();

	PointC *getData() const;
    long getLength() const;
    void setData(PointC *data, long length);

private:
	PointC* data;
	long length;
};

#endif /* POINTCLOUD_H_ */
