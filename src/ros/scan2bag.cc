#include "slam6d/globals.icc"
#include "slam6d/io_utils.h"
#include "slam6d/scan.h"
#include "ros/tdtkroswrapper.h"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>

#include <boost/program_options.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef _MSC_VER
#if !defined _OPENMP && defined OPENMP
#define _OPENMP
#endif
#endif

#ifdef _OPENMP
#include <omp.h>
#endif

using namespace boost;

int main(int argc, char** argv) {

	// parse args
	int start		=  0;
	int end			= -1;
	IOType type		=  UOS;
	bool scanserver	=  false;
	double minDist	= -1;
	double maxDist	= -1;
	double scale	= 0.01;	// TODO: consider scale Parameter instead of hardcoded scale

	double voxelsize= -1.0;
	double octree	=  0;

	std::string bagfile  =  "log.bag";
	std::string topic	=  "/pointcloud";
	std::string dir		=  "./";
	bool useStamps	=  false;

	bool noOdom     =  false;
	bool noTf       =  false;

	double  frequency  =  -1.0;

    program_options::options_description desc("Allowed options");
    desc.add_options()
            ("help,h", "produce help message")
            ("out-bag,b", program_options::value<std::string>(&bagfile)->default_value("log.bag"), "output ros bag file")
			("start,s", program_options::value<int>(&start)->default_value(0), "start at scan NR")
			("end,e", program_options::value<int>(&end)->default_value(-1), "end at scan NR")
			/*("format,f",program_options::value(&type)->default_value(UOS, "uos"),
			 "The input files are read with this shared library.\n"
			 "Available values: uos, uos_map, uos_rgb, uos_frames, uos_map_frames, "
			 "old, rts, rts_map, ifp, riegl_txt, riegl_rgb, riegl_bin, zahn, ply, "
			 "wrl, xyz, zuf, iais, front, x3d, rxp, ais.")*/ // TODO
			("scale", program_options::value<double>(&scale)->default_value(0.01), "Scale of exported point cloud")
			("min,M", program_options::value<double>(&minDist)->default_value(0.0), "Neglect all points closer than this to the origin")
			("max,m", program_options::value<double>(&maxDist)->default_value(std::numeric_limits<double>::max()), "Neglect all points further than this from the origin")
			("reduce,r", program_options::value(&voxelsize)->default_value(0),
			 "Turn on octree based point reduction with voxels  of size arg^3.")
			("octree,O", program_options::value(&octree)->implicit_value(1),
			 "Enable randomized octree based point reduction with arg points per voxel. "
			 "Requires --reduce (-r).") // TODO where is this enforced?
			("topic", program_options::value<std::string>(&topic)->default_value("/pointcloud"), "topic in bagfile withcreated PointCloud2 messages")
			("timestamp",program_options::bool_switch(&useStamps),"use timestamps if they are available in pose files")
			("noOdom",program_options::bool_switch(&noOdom),"do not write odometry messages")
			("noTf",program_options::bool_switch(&noTf),"do not write tf messages")
			("freq",program_options::value<double>(&frequency)->default_value(-1.0), "frequency of scanning device")
			;



    program_options::options_description hidden("Hidden options");
    hidden.add_options()
        ("input-dir", program_options::value<std::string>(&dir), "input dir");

    // all options
    program_options::options_description all;
    all.add(desc).add(hidden);

    // options visible with --help
    program_options::options_description cmdline_options;
    cmdline_options.add(desc);

    // positional argument
    program_options::positional_options_description pd;
    pd.add("input-dir", 1);

    // process options
    program_options::variables_map vm;
    program_options::store(program_options::command_line_parser(argc, argv).
              options(all).positional(pd).run(), vm);
    program_options::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return EXIT_SUCCESS;
    }

    program_options::notify(vm);

	// Load scans
//	Scan::setProcessingCommand(argc, argv);
	Scan::openDirectory(scanserver, dir, type, start, end);

	if(Scan::allScans.size() == 0) {
		std::cerr << "No scans found. Did you use the correct format?" << std::endl;
		exit(-1);
	}

	rosbag::Bag bag(bagfile,rosbag::bagmode::Write);

	ros::Time::init();

	TdtkRosWrapper tdtk2ros;

	ros::Time base_stamp = ros::Time::now();
	int count = 0;

	for(auto scan : Scan::allScans) {
		ros::Time stamp;
		if(frequency > 0) {
			stamp = base_stamp + ros::Duration(count / frequency);
			count++;
		} else {
			stamp = ros::Time::now();	// Maybe create timestamp with certain frequency?
		}

		scan->setRangeFilter(maxDist, minDist);
		scan->setReductionParameter(voxelsize,octree);

		// save initial pose for later use
		const double *rPos = scan->get_rPos();
		const double *rPosTheta = scan->get_rPosTheta();

		/*
			Transform all point with inverse transmat, to get a point cloud in
			sensor coordinate system. Maybe there is a better way to get the raw
			points? scan->get("xyz") returns transformed Points (using original pose)
		*/
		double transmat_inv[16];
		M4inv(scan->get_transMatOrg(),transmat_inv);
		scan->transformAll(transmat_inv);

		DataXYZ xyz(scan->get("xyz reduced"));

		// copy data to pointcloud2 via pcl::pointcloud
		pcl::PointCloud<pcl::PointXYZ> cloud;
		cloud.width  = xyz.size();
		cloud.height = 1;
		cloud.points.resize(cloud.width*cloud.height);

		for(size_t i=0; i < xyz.size(); i++) {
			cloud.points[i].x =  xyz[i][2]/100.0;
			cloud.points[i].y = -xyz[i][0]/100.0;
			cloud.points[i].z =  xyz[i][1]/100.0;
		}
		sensor_msgs::PointCloud2 cloud_out;
		pcl::toROSMsg(cloud,cloud_out);
		cloud_out.header.stamp = stamp;
		cloud_out.header.frame_id = "base_link";
		cloud_out.header.seq = atoi(scan->getIdentifier());//cloud_seq++;

		// TODO: how to handle other datatypes, like reflection, color, temperature, etc?

		// for the moment publish Odometry in /odom topic, TODO: use tf instead
		tf::StampedTransform transform;
		tdtk2ros.pose2ros(rPos,rPosTheta,transform);
		nav_msgs::Odometry odom;
		odom.header.stamp = stamp;
		odom.header.seq   = cloud_out.header.seq;
		odom.header.frame_id = "odom"; //"odom";
		odom.child_frame_id  = "base_link";	// "laser" ?
		tf::poseTFToMsg(transform,odom.pose.pose);

		/* additionally store tf-stack in bag. for simplicity assume sensor is
		   located in base_link */
		tf::tfMessage tfmsg;
		geometry_msgs::TransformStamped msg;
		tf::transformStampedTFToMsg(transform,msg);
		msg.header.stamp = stamp;
		msg.header.frame_id = "odom";//"odom";
		msg.child_frame_id  = "base_link";
		std::vector<geometry_msgs::TransformStamped> vec;
		vec.push_back(msg);
		tfmsg.transforms = vec;


		if(!noOdom)
			bag.write("/odom",stamp,odom);
		bag.write(topic,stamp,cloud_out);
		if(!noTf)
			bag.write("/tf",stamp,tfmsg);
	}

	return 0;

}
