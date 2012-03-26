#ifndef __GRID_CLUSTER_H__
#define __GRID_CLUSTER_H__

#include "slam6d/scan.h"
#include "veloslam/gridcell.h"

#define CUDE_TYPE_ABOVE_LIDAR        0x00000001
#define CUDE_TYPE_IN_OBSTACLE_RANGE  0x00000002
#define CUDE_TYPE_CONTAIN_SICK       0x00000400

#define CLUSTER_TYPE_OBJECT          0x00000001
#define CLUSTER_TYPE_MOVING_OBJECT   0x00000002

class clusterFeature
{
public:
	float min_x,min_y,min_z;
	float max_x,max_y,max_z;
	float avg_x, avg_y, avg_z;
	float min_y_x,   max_y_x;
	float size_x,size_y,size_z;
	float speed_x, speed_y, speed;

	float theta,radius;
	int size;

	float length;
	float width;
	float boxDirection;
	double boxVex[4][2];

	int frameNO;
	int trackNO;

	int pointNumber;
	unsigned int clusterType;
};

typedef vector<cellFeature*> cluster;
typedef vector<cluster> clusterArray;
typedef vector<clusterFeature> clusterFeatureArray;


#endif  //__GRID_CLUSTER_H__
