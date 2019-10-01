
#include "ros/tdtkroswrapper.h"
#include "slam6d/globals.icc"

#include <vector>
#include <sstream>

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <pcl_conversions/pcl_conversions.h>

void TdtkRosWrapper::ros2frames(const tf::StampedTransform &transform, double* transmat){
	transmat[0] = transform.getBasis().getRow(1).getY();
	transmat[1] = -transform.getBasis().getRow(2).getY();
	transmat[2] = -transform.getBasis().getRow(0).getY();
	transmat[3] = 0.0;

	transmat[4] = -transform.getBasis().getRow(1).getZ();
	transmat[5] = transform.getBasis().getRow(2).getZ();
	transmat[6] = transform.getBasis().getRow(0).getZ();
	transmat[7] = 0.0;

	transmat[8] = -transform.getBasis().getRow(1).getX();
	transmat[9] = transform.getBasis().getRow(2).getX();
	transmat[10] = transform.getBasis().getRow(0).getX();
	transmat[11] = 0.0;

	// translation
	transmat[12] = -transform.getOrigin().getY()*100;
	transmat[13] =  transform.getOrigin().getZ()*100;
	transmat[14] =  transform.getOrigin().getX()*100;
	transmat[15] = 1;

};

void TdtkRosWrapper::ros2pose(const tf::StampedTransform &transform, double* rPos, double *rPosTheta) {
	double transmat[16];
	ros2frames(transform,transmat);
	Matrix4ToEuler(transmat,rPosTheta,rPos);
};

void TdtkRosWrapper::frames2ros(const double* transmat, tf::StampedTransform &transform){
	transform.getBasis().setValue( transmat[10], -transmat[2], transmat[6],
			-transmat[8], transmat[0], -transmat[4],
			transmat[9], -transmat[1], transmat[5] );

	// translation
	transform.getOrigin().setY(-transmat[12]/100.0);
	transform.getOrigin().setZ(transmat[13]/100.0);
	transform.getOrigin().setX(transmat[14]/100.0);
};

void TdtkRosWrapper::pose2ros(const double* rPos, const double* rPosTheta, tf::StampedTransform &transform){
	double transmat[16];
	EulerToMatrix4(rPos,rPosTheta,transmat);
	frames2ros(transmat,transform);
};

void TdtkRosWrapper::pointcloud2scan(const sensor_msgs::PointCloud2ConstPtr &cloud, std::vector<double*> &pts) {
	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*cloud,pcl_pc2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

	//Maybe accessing by iterators is faster then converting to pcl first

	for(const auto &pt : temp_cloud->points) {
		double *d = new double[3];
		d[0] = -100.0*pt.y;
		d[1] = 100.0*pt.z;
		d[2] = 100.0*pt.x;
		pts.push_back(d);
	}
}

std::string TdtkRosWrapper::transformToString(const tf::Transform &t) {
	std::stringstream ss;
	tf::Matrix3x3 mat = t.getBasis();
	ss << mat.getRow(0).getX() << " ";
	ss << mat.getRow(0).getY() << " ";
	ss << mat.getRow(0).getZ() << " ";
	ss << t.getOrigin().getX() << "\n";
	ss << mat.getRow(1).getX() << " ";
	ss << mat.getRow(1).getY() << " ";
	ss << mat.getRow(1).getZ() << " ";
	ss << t.getOrigin().getY() << "\n";
	ss << mat.getRow(2).getX() << " ";
	ss << mat.getRow(2).getY() << " ";
	ss << mat.getRow(2).getZ() << " ";
	ss << t.getOrigin().getZ() << "\n";
	ss << "0 0 0 1\n";
	return ss.str();
}
