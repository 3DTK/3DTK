#ifndef __GRID_CLUSTER_H__
#define __GRID_CLUSTER_H__

#include "slam6d/scan.h"
#include "veloslam/gridcell.h"

/** ±ÈÀ×´ï¸ß*/
#define CUDE_TYPE_ABOVE_LIDAR       0x00000001
#define CUDE_TYPE_IN_OBSTACLE_RANGE       0x00000002
#define CUDE_TYPE_CONTAIN_SICK        0x00000400

/** ±ÈÀ×´ï¸ß*/
#define CLUSTER_TYPE_OBJECT       0x00000001
#define CLUSTER_TYPE_MOVING_OBJECT       0x00000002

/** 集群的特征 这个是饼型网格集群的特征*/// 动目标的特征 
class clusterFeature
{
public:
	/** x,y,z的最值*/
	float min_x,min_y,min_z;
	float max_x,max_y,max_z;
	float avg_x, avg_y, avg_z;
	float min_y_x,   max_y_x;           //最大最小Y 对应的x ，朱神添 添加
	float size_x,size_y,size_z;
	float speed_x, speed_y, speed;   // 速度特征

	float theta,radius;
	int size;

	int frameNO;
	int trackNO;
		/*point的数量*/
	int pointNumber;
	unsigned int clusterType;
};

//Ò»¸ö¼¯ÈºÀïÓÐºÃ¼¸¸öÍø¸ñ£¬Ã¿¸öÍø¸ñÌØÕ÷ÏàËÆ
typedef vector<cellFeature*> cluster;
typedef vector<cluster> clusterArray;
typedef vector<clusterFeature> clusterFeatureArray;


#endif  //__GRID_CLUSTER_H__
