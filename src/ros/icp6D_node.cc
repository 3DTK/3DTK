#include "ros/icp6Dwrapper.h"

int main(int argc, char** argv) {

	ros::init(argc,argv, "icp6Dwrapper");

	Icp6DWrapper wrapper;
	wrapper.spin();

}
