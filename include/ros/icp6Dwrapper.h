#ifndef __ICP6DWRAPPER_H__
#define __ICP6DWRAPPER_H__

#include "ros/tdtkroswrapper.h"

#include <slam6d/icp6D.h>
#include <slam6d/icp6Dsvd.h>
#include <slam6d/scan.h>
#include <slam6d/globals.icc>

#include <vector>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class Icp6DWrapper : public TdtkRosWrapper {
public:
	Icp6DWrapper();
	~Icp6DWrapper();

	void spin();

protected:
	void callbackPointCloud(const sensor_msgs::PointCloud2ConstPtr &msg);

	string sensor_topic;
	string sensor_frame;
	string base_frame;
	string odom_frame;
	string map_frame;

	// ICP parameters
	double mdm;
	double epsICP;
	int rnd;
	int iters;
	double red;
	int ppvoxel;
	double minDist;
	double maxDist;
	unsigned int nns_method;

	ros::NodeHandle nh;
	ros::Subscriber subPCL2;
//	ros::Publisher  pubMapOdom;

	tf::TransformListener *l;
	tf::TransformBroadcaster b;
	tf::StampedTransform map_odom;

	icp6Dminimizer *my_icp6Dminimizer;
	icp6D *icp;
	std::vector<Scan*> scans;
};


#endif
