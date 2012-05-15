#ifndef __GRID_CLUSTER_H__
#define __GRID_CLUSTER_H__

#include "slam6d/scan.h"
#include "veloslam/gridcell.h"

#define CUDE_TYPE_ABOVE_LIDAR        0x00000001
#define CUDE_TYPE_IN_OBSTACLE_RANGE  0x00000002
#define CUDE_TYPE_CONTAIN_SICK       0x00000400

#define CLUSTER_FEATURE_TYPE_OBJECT     0x00000001

#define CLUSTER_TYPE_MOVING_OBJECT   0x00000002
#define CLUSTER_TYPE_MOVING_OBJECT_PEDESTRIAN   0x00000004
#define CLUSTER_TYPE_MOVING_OBJECT_BICYCLE   0x00000008
#define CLUSTER_TYPE_MOVING_OBJECT_CAR   0x00000010
#define CLUSTER_TYPE_MOVING_OBJECT_ BUS  0x00000020

#define CLUSTER_TYPE_STATIC_OBJECT   0x00010000
#define CLUSTER_TYPE_STATIC_OBJECT_BUILDING     0x00020000
#define CLUSTER_TYPE_STATIC_OBJECT_BILLBOARD   0x00040000
#define CLUSTER_TYPE_STATIC_OBJECT_SCULPTURE   0x00080000

#define CLUSTER_TYPE_STATIC_OBJECT_PLANT    0x00100000
#define CLUSTER_TYPE_STATIC_OBJECT_TREE      0x00200000
#define CLUSTER_TYPE_STATIC_OBJECT_SHRUB   0x00400000

class clusterFeature
{
public:
	float min_x,min_y,min_z;
	float max_x,max_y,max_z;
	float avg_x, avg_y, avg_z;
	float min_y_x,   max_y_x;
	float size_x,size_y,size_z;
	float speed_x, speed_y, speed;

	float theta, radius;
	int size;

	float length;
	float width;
	float boxDirection;
	double boxVex[4][2];

	int frameNO;  //in which scan
    int selfID;   //which number in clusters
	int trackNO;  //which track log it

	int pointNumber;
	unsigned int clusterType;
};

typedef std::vector<cellFeature*> cluster;
typedef std::vector<cluster> clusterArray;
typedef std::vector<clusterFeature> clusterFeatureArray;

#endif  //__GRID_CLUSTER_H__
