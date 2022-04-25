#include <iostream>
#include <fstream>
#include <cassert>
#include <string>

#include <boost/program_options.hpp>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>

#include <sys/types.h>
#include <sys/stat.h>

// checks if the given path exists
int existsDir(const char* path)
{
    struct stat info;
    if (stat( path, &info ) != 0) return 0;
    else if ( info.st_mode & S_IFDIR ) return 1;
    else return 0;
}

using namespace std;
namespace po = boost::program_options;


const char* PATH; // Current directory
const int POSE_WINDOW_SIZE = 1; // Size of pose sliding window average
std::vector<geometry_msgs::PoseStamped> current_poses;

uint seq;
bool firstLidarCallback = false;

// Transform between IMU frame and Laserscan frame
tf::StampedTransform transform_lidar2pose;

int parse_options (int argc, char** argv,
  string& outdir,
  string& lidar_topic,
  string& pose_topic,
  bool& lookup_tf,
  string& lidar_frame,
  string& source_frame,
  int& pose_type
)
{
  po::options_description generic("Generic options");
  po::options_description input("Program options");
  po::options_description hidden("Hidden options");
  generic.add_options()
    ("help,h", "Display a very helpful message");
  input.add_options()
    ("lidar_topic,L", po::value<string>(&lidar_topic)->default_value("/lidar"),
     "Provide the name of the topic where LiDAR data gets published.\n"
     "Currently, only sensor_msgs::PointCloud2 format is supported.")
    ("pose_topic,P", po::value<string>(&pose_topic)->default_value("/pose"),
     "Provide the name of the topic where pose data gets published.")
    ("pose_type", po::value<int>(&pose_type)->default_value(1),
     "Chose the ROS data type of pose input:\n"
     " 1 - geometry_msgs::PoseStamped\n"
     " 2 - sensor_msgs::Imu (this will only use rotation)")
    ("lookup_tf,t", po::bool_switch(&lookup_tf)->default_value(false),
     "Wether to look up the transformation between the LiDAR and pose frames.\n"
     "If used, consider the next two parameters.")
    ("lidar_frame,l", po::value<string>(&lidar_frame)->default_value("/lidar_frame"),
     "Provide the name of frame that the LiDAR data uses.")
    ("pose_frame,p", po::value<string>(&source_frame)->default_value("/map"),
     "Provide the name of frame that the pose data uses. Usually called \"/map\".");
  hidden.add_options()
    ("output-dir", po::value<string>(&outdir), "output-dir");
  // All options together
    po::options_description alloptions;
    alloptions.add(generic).add(input).add(hidden);

    // Only commandline, visible with --help
    po::options_description cmdoptions;
    cmdoptions.add(generic).add(input);

    // positional argument for input directory
    po::positional_options_description pos;
    pos.add("output-dir", 1); // max 1 pos arg

    // Map and store option inputs to variables
    po::variables_map vars;
    po::store( po::command_line_parser(argc, argv).
                options(alloptions).
                positional(pos).
                run(),
                vars);

    // help display msg
    if ( vars.count("help") )
    {
        cout << cmdoptions;
        cout << endl << "Example usage:" << endl
           << "\t bin/bag_listener dat/your/out/dir --lidar_topic=/livox/lidar --pose_topic=/imu/pose --lookup_tf --lidar_frame=/livox_frame --pose_frame=/map" << endl;
        exit(0);
    }
    po::notify(vars);

    // Add trailing directory slash if there is none.
    // Works differently when compiling under Windows
#ifndef _MSC_VER
    if (outdir[ outdir.length()-1 ] != '/') outdir = outdir + "/";
#else
    if (outdir[ outdir.length()-1]  != '\\') outdir = outdir + "\\";
#endif

    // return with success exit code
    return 0;
}

/**
 * Converts a right-hand-side matrix into a 3DTK matrix
 * @param *inMatrix pointer to matrix (double[16])
 * @param *outMatrix pointer to matrix (double[16])
 * @param scale used for unit conversion, default 100.0 for Riegl
 */
inline void to3DTKMat(const double *inMatrix,
				  double *outMatrix, float scale = 100.0)
{
    outMatrix[0] = inMatrix[5];
    outMatrix[1] = -inMatrix[9];
    outMatrix[2] = -inMatrix[1];
    outMatrix[3] = -inMatrix[13];
    outMatrix[4] = -inMatrix[6];
    outMatrix[5] = inMatrix[10];
    outMatrix[6] = inMatrix[2];
    outMatrix[7] = inMatrix[14];
    outMatrix[8] = -inMatrix[4];
    outMatrix[9] = inMatrix[8];
    outMatrix[10] = inMatrix[0];
    outMatrix[11] = inMatrix[12];
    outMatrix[12] = -scale*inMatrix[7];
    outMatrix[13] = scale*inMatrix[11];
    outMatrix[14] = scale*inMatrix[3];
    outMatrix[15] = inMatrix[15];
}

static inline void Matrix4ToEuler(const double *alignxf,
                                  double *rPosTheta,
                                  double *rPos = 0)
{
  double _trX, _trY;

  // Calculate Y-axis angle
  if(alignxf[0] > 0.0) {
    rPosTheta[1] = asin(alignxf[8]);
  } else {
    rPosTheta[1] = M_PI - asin(alignxf[8]);
  }

  double  C    =  cos( rPosTheta[1] );
  if ( fabs( C ) > 0.005 )  {                 // Gimbal lock?
    _trX      =  alignxf[10] / C;             // No, so get X-axis angle
    _trY      =  -alignxf[9] / C;
    rPosTheta[0]  = atan2( _trY, _trX );
    _trX      =  alignxf[0] / C;              // Get Z-axis angle
    _trY      = -alignxf[4] / C;
    rPosTheta[2]  = atan2( _trY, _trX );
  } else {                                    // Gimbal lock has occurred
    rPosTheta[0] = 0.0;                       // Set X-axis angle to zero
    _trX      =  alignxf[5];  //1                // And calculate Z-axis angle
    _trY      =  alignxf[1];  //2
    rPosTheta[2]  = atan2( _trY, _trX );
  }

  rPosTheta[0] = rPosTheta[0];
  rPosTheta[1] = rPosTheta[1];
  rPosTheta[2] = rPosTheta[2];

  if (rPos != 0) {
    rPos[0] = alignxf[12];
    rPos[1] = alignxf[13];
    rPos[2] = alignxf[14];
  }
}

void slidingWindow(geometry_msgs::PoseStamped& pose)
{
// If we have reached the current window size we remove the first
  // element and pushback the new element
  if(current_poses.size() == POSE_WINDOW_SIZE)
  {
    current_poses.erase(current_poses.begin());
    current_poses.push_back(pose);
  }
  // Otherwise we simply pushback
  else
  {
    current_poses.push_back(pose);
  }

  // Vector should be window size or less
  assert(current_poses.size() <= POSE_WINDOW_SIZE);
}

void poseMsgCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // Copy msg data
  geometry_msgs::PoseStamped pose = geometry_msgs::PoseStamped();
  pose.header = msg->header;
  pose.pose = msg->pose;
  slidingWindow(pose);
}

void imuMsgCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  geometry_msgs::PoseStamped pose = geometry_msgs::PoseStamped();
  pose.header = msg->header;
  // Fill the pose here but leave out the position part.
  pose.pose.orientation = msg->orientation;
  pose.pose.position = geometry_msgs::Point(); // initializes with 0
  slidingWindow(pose);
}

geometry_msgs::PoseStamped compute_pose_avg(void)
{
  geometry_msgs::PoseStamped avg = geometry_msgs::PoseStamped();

  for (std::vector<geometry_msgs::PoseStamped>::iterator pose = current_poses.begin();
       pose != current_poses.end();
       ++pose)
  {
    // Add all orientation values
    avg.pose.orientation.x += pose->pose.orientation.x;
    avg.pose.orientation.y += pose->pose.orientation.y;
    avg.pose.orientation.z += pose->pose.orientation.z;
    avg.pose.orientation.w += pose->pose.orientation.w;
    // Add all position values
    avg.pose.position.x += pose->pose.position.x;
    avg.pose.position.y += pose->pose.position.y;
    avg.pose.position.z += pose->pose.position.z;
  }
  // Divide by length to get the average
  avg.pose.orientation.x /= current_poses.size();
  avg.pose.orientation.y /= current_poses.size();
  avg.pose.orientation.z /= current_poses.size();
  avg.pose.orientation.w /= current_poses.size();

  avg.pose.position.x /= current_poses.size();
  avg.pose.position.y /= current_poses.size();
  avg.pose.position.z /= current_poses.size();

    //avg = current_poses.end()[0];
  return avg;
}

void lidarMsgCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // First Callback
    if (!firstLidarCallback) {
      std::string info_string = std::string("Writing files to ")
                                      + std::string(PATH);
      ROS_INFO("%s", info_string.c_str());
      firstLidarCallback = true;
    }

    // Converting the livox msg to appropriate format
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    // Opening 3d file to write into
    std::ofstream file_3d;
    char* file_name = new char[50]();
    std::sprintf(file_name, "%sscan%03d.3d", PATH, seq);
    ROS_INFO("Writing %s ...", file_name);
    file_3d.open(file_name);

    // Writing the lidar data to the text file
    // This is a left handed coordinate system and we convert the values to cm
    for (int i = 0; i < temp_cloud->points.size(); ++i)
    {
        pcl::PointXYZI p = temp_cloud->points[i];
        // Convert to left handed
        tf::Vector3 v;
        v.setX(-p.y);
        v.setY(p.z);
        v.setZ(p.x);

        file_3d << 100.0 * v.getX() << " "
                << 100.0 * v.getY() << " "
                << 100.0 * v.getZ() << " "
                << temp_cloud->points[i].intensity << std::endl;
    }
    file_3d.close();

    // Opening the pose file to write into
    std::ofstream file_pose;
    std::sprintf(file_name, "%sscan%03d.pose", PATH, seq);

    // Get current pose average
    geometry_msgs::PoseStamped current_pose_avg = compute_pose_avg();

    // Getting the current pose as homgeneous transformation (4x4)
    tf::Quaternion q(current_pose_avg.pose.orientation.x,
                    current_pose_avg.pose.orientation.y,
                    current_pose_avg.pose.orientation.z,
                    current_pose_avg.pose.orientation.w);

    q = q * transform_lidar2pose.getRotation();
    tf::Matrix3x3 m(q);

    const double in_matrix[16] ={m[0][0],m[0][1],m[0][2], current_pose_avg.pose.position.x + transform_lidar2pose.getOrigin().getX(),
                                 m[1][0],m[1][1],m[1][2], current_pose_avg.pose.position.y + transform_lidar2pose.getOrigin().getY(),
                                 m[2][0],m[2][1],m[2][2], current_pose_avg.pose.position.z + transform_lidar2pose.getOrigin().getZ(),
				                         0      ,0      ,0      , 1};

    // Converting to left handed matrix
    double out_matrix[16], rPos[3], rPosTheta[16];
    to3DTKMat(in_matrix, out_matrix, 1);
    Matrix4ToEuler(out_matrix, rPosTheta, rPos);

    // Extracting Position and Orientaion
    double x = 100.0 * rPos[0];
    double y = 100.0 * rPos[1];
    double z = 100.0 * rPos[2];
    double roll  = 1.0 * rPosTheta[0];
    double pitch = 1.0 * rPosTheta[1];
    double yaw   = 1.0 * rPosTheta[2];

    // Writing to the pose file
    // This is also a left hand coordinate system
    // - Thumb (x) to the right
    // - Pointy Finger (y) to  the top
    // - Middle Finger (z) into the room
    file_pose.open(file_name);
    file_pose << x << " " << y << " " << z << " " <<
                 roll * (180.0/M_PI) << " " <<
                 pitch * (180.0/M_PI) << " " <<
                 yaw * (180.0/M_PI) << " " << std::endl;
    file_pose.close();

    seq++;
    delete file_name;
}

int main(int argc, char **argv)
{
    std::cout << "bag_listener - A program that listens to ROS bagfiles" << std::endl;
    std::cout << "               and writes 3DTK files (UOSR-format) on disk." << std::endl;
    std::cout << "               Use -h for more information." << std::endl;
    std::cout << "Note: Remember that you need a running roscore, as well as a running bagfile." << std::endl;
    std::cout << "Tip of the day: Start this program first, it will wait for the bagfile to run." << std::endl;

    string outdir;
    string lidar_topic;
    string pose_topic;
    bool lookup_tf;
    string lidar_frame;
    string source_frame;
    int pose_type;

    parse_options(argc, argv,
        outdir, lidar_topic, pose_topic, lookup_tf, lidar_frame, source_frame, pose_type);

    PATH = outdir.c_str();
    if ( !existsDir( PATH ) )
    {
      boost::filesystem::create_directory( PATH );
      std::cout << "Creating \"" << PATH << "\"." << std::endl;
    } else std::cout << PATH << " exists allready." << std::endl;

    seq = 0;

    ros::init(argc, argv, "file_writer");
    ros::NodeHandle n;
    if (lookup_tf)
    {
        tf::TransformListener listener;
        listener.waitForTransform(lidar_frame, source_frame, ros::Time(0), ros::Duration(5.0) );
        listener.lookupTransform(lidar_frame, source_frame, ros::Time(0), transform_lidar2pose);
        ROS_INFO("Transform found : %f %f %f %f %f %f %f",
            transform_lidar2pose.getOrigin().getX(),
            transform_lidar2pose.getOrigin().getY(),
            transform_lidar2pose.getOrigin().getZ(),
            transform_lidar2pose.getRotation().getX(),
            transform_lidar2pose.getRotation().getY(),
            transform_lidar2pose.getRotation().getZ(),
            transform_lidar2pose.getRotation().getW()
        );
    }

    ros::Subscriber pose_sub;
    switch (pose_type) {
      case 1:
        pose_sub = n.subscribe(pose_topic, 100000, poseMsgCallback);
        break;
      case 2:
        pose_sub = n.subscribe(pose_topic, 100000, imuMsgCallback);
        break;
    }
    ros::Subscriber lidar_sub = n.subscribe(lidar_topic, 100000, lidarMsgCallback);

    ros::spin();
}
