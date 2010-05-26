/*
 * PointCloud.cpp
 *
 *  Created on: Mar 14, 2010
 *      Author: darko
 */

#include "PointCloud.h"
#include "PointC.h"
#include <vector>

using namespace std;

PointCloud::PointCloud(PointC* data, long length) {
	this->data = data;
	this->length = length;
}


PointCloud::PointCloud(PolarPointCloud *ppc)
{
	const vector<PolarPoint> *ppdata = ppc->getData();
	long length = ppc->getLength();

	PointC* cdata = new PointC[length];

	for (int i = 0 ; i < length ; i++) {
		cdata[i] = PointC(POLAR, (*ppdata)[i].a, (*ppdata)[i].b, (*ppdata)[i].d, (*ppdata)[i].r);
	}

	this->data = cdata;
	this->length = length;

}

PointCloud::~PointCloud() {
	// TODO Auto-generated destructor stub
	delete [] this->data;
}

void PointCloud::setData(PointC *data, long  length)
{
}



PointC *PointCloud::getData() const
{
	return this->data;
}



long PointCloud::getLength() const
{
	return this->length;
}




