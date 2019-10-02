#ifndef __TDTKROSWRAPPER_H__
#define __TDTKROSWRAPPER_H__

#include "slam6d/globals.icc"

#include <vector>

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>

class TdtkRosWrapper {
public:
	/**
	 * Converts right handed tf::Transform to a left handed 3DTK transformation matrix
	 */
	void ros2frames(const tf::StampedTransform &transform, double* transmat);

	/**
	 * Converts right handed tf::Transform to a left handed 3DTK pose
	 */
	void ros2pose(const tf::StampedTransform &transform, double* rPos, double *rPosTheta);

	/** Converts a left handed 3DTK transformation matrix to tf::StampedTransform */
	void frames2ros(const double* transmat, tf::StampedTransform &transform);

	/** Converts a left handed 3DTK pose to tf::StampedTransform */
	void pose2ros(const double* rPos, const double* rPosTheta, tf::StampedTransform &transform);

	/**
	 *	convert a right handed sensor_msgs::PointCloud2 to a left handed vector<double*> pointlcoud
	 *  for use in 3DTK functions
	 */
	void pointcloud2scan(const sensor_msgs::PointCloud2ConstPtr &cloud, std::vector<double*> &pts);

	/**
	 * create a matrix representation of a Transform for debug output
	 */
	std::string transformToString(const tf::Transform &t);


};

#endif
