#include <iostream>
#include <fstream>
#include <cassert>
#include <string>
#include <queue>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <omp.h>
#include <chrono>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
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

// checks if the given path exists.
int existsDir(const char* path)
{
    struct stat info;
    if (stat( path, &info ) != 0) return 0;
    else if ( info.st_mode & S_IFDIR ) return 1;
    else return 0;
}

using namespace std;
using namespace chrono;
namespace po = boost::program_options;

const char* PATH; // Current directory
const int POSE_WINDOW_SIZE = 2; // Size of pose sliding window average
bool have_pose_topic = true;

const int PATH_CHAR_LEN = 5000; // what a shit define... maximum buffer size for output path
const double conv_rad2deg = 57.29577951308232;

// Pose stream is usually very fast wrt. LiDAR, so we assign an individual callback queue
ros::CallbackQueue callbacks_poses;
std::queue<geometry_msgs::PoseStamped> current_poses;
bool ignore_stamps = false;
bool skip_points = false;
double min_dist2;
double max_dist2;

uint seq; // this counts the scanfiles, starting from 000, 001, 002, ... , 999, 1000, 1001,...
bool firstLidarCallback = false;
bool verbose;

// Transform between IMU frame and Laserscan frame
tf::StampedTransform transform_lidar2pose;
// These can be filled using the inital callbacks.
string lidar_frame = "";
string source_frame = "";

int parse_options (int argc, char** argv,
  string& outdir,
  string& lidar_topic,
  string& pose_topic,
  bool& skip_points,
  int& pose_type,
  bool& ignore_timestamps,
  bool& verbose,
  double& minDist,
  double& maxDist
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
    ("pose_topic,P", po::value<string>(&pose_topic)->default_value(""),
     "Provide the name of the topic where pose data gets published.")
    ("pose_type", po::value<int>(&pose_type)->default_value(1),
     "Chose the ROS data type of pose input:\n"
     " 1 - geometry_msgs::PoseStamped\n"
     " 2 - sensor_msgs::Imu (this will only use rotation)")
    ("minDist,m", po::value<double>(&minDist)->default_value(0),
      "Ignore points closer to <arg> cm")
    ("maxDist,M", po::value<double>(&maxDist)->default_value(std::numeric_limits<double>::max()),
      "Ignore points further than <arg> cm")
    ("skip_points,s", po::bool_switch(&skip_points)->default_value(false),
    "Exports only pose data. Use if re-run on a bagfile where points have already been exported.")
    ("verbose,v", po::bool_switch(&verbose)->default_value(false),
     "Makes this program talk more. Use to print debug information.")
    ("ignore_timestamps", po::bool_switch(&ignore_timestamps)->default_value(false),
    "Ignores timestamps in header and default to ROS msg receive time. Not recommended.");
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
           << "\t bin/ros_listener dat/your/out/dir --lidar_topic=/livox/lidar --pose_topic=/camera/pose" << endl;
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

inline void waitForSlidingWindow(double t)
{
  // Print the timestamp constraints to check if t_front < t < t_back
  // if (verbose && current_poses.size() > 0) ROS_INFO("%f < %f < %f", current_poses.front().header.stamp.toSec(), t, current_poses.back().header.stamp.toSec());

  // Wait for the queue to be not empty
  while(current_poses.size() < POSE_WINDOW_SIZE && ros::ok()) callbacks_poses.callOne( ros::WallDuration() );

  // If timestamps are not used, we can return at this point.
  if (ignore_stamps) callbacks_poses.callAvailable( ros::WallDuration() );

  // If timestamps are used, we need to check if we are lagging behind the accumulator
  if (t < current_poses.front().header.stamp.toSec()) {
    ROS_WARN_COND(verbose, "Lagging behind because %f [current] < %f [front]", t, current_poses.front().header.stamp.toSec());
    return;
  }

  // If timestamps are used, wait for the LiDAR timestamp to be between the pose timestamps.
  else while( !(current_poses.front().header.stamp.toSec() < t
      && t < current_poses.back().header.stamp.toSec())
      && ros::ok() ) {
        if (verbose) ROS_INFO("Cycling...");
        callbacks_poses.callOne( ros::WallDuration() ); // next pose Callback
        //if (verbose && current_poses.size() > 0) ROS_INFO("%f < %f < %f", current_poses.front().header.stamp.toSec(), t, current_poses.back().header.stamp.toSec());
      }
  return;
}

void slidingWindow(geometry_msgs::PoseStamped& pose)
{
  if (verbose) ROS_INFO("Moving the sliding window...");
  // If we have reached the current window size we remove the first
  // element and pushback the new element
  if(current_poses.size() == POSE_WINDOW_SIZE)
  {
    current_poses.pop();
    current_poses.push(pose);
  }
  // Otherwise we simply pushback
  else
  {
    current_poses.push(pose);
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
  source_frame = msg->header.frame_id;
  slidingWindow(pose);
}

void imuMsgCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  if (verbose) ROS_INFO("IMU callback");
  geometry_msgs::PoseStamped pose = geometry_msgs::PoseStamped();
  pose.header = msg->header;
  // Fill the pose here but leave out the position part.
  pose.pose.orientation = msg->orientation;
  pose.pose.position = geometry_msgs::Point(); // initializes with 0
  source_frame = msg->header.frame_id;
  slidingWindow(pose);
}

geometry_msgs::PoseStamped interpolate_pose_at(double t)
{
  // Construct a new time-stamped pose via interpolation, or default to the origin
  if (verbose) ROS_INFO("Interpolation of pose");
  geometry_msgs::PoseStamped avg = geometry_msgs::PoseStamped();
  tf::quaternionTFToMsg(tf::createIdentityQuaternion(), avg.pose.orientation);
  if (!have_pose_topic) return avg;

  // Get timestamps for interpolation
  if (verbose) ROS_INFO("Waiting...");
  waitForSlidingWindow(t); // we have to wait for the LiDAR timestamp to be between pose measurement timestamps
  if (verbose) ROS_INFO("Finished waiting. Size of pose queue: %ld", current_poses.size());
  double t_res;
  if (ignore_stamps) t_res = 0.5; // use standard average (Slerp using same weighting) of the pose-queue, if stamps are ignored
  else {
    double t1 = current_poses.front().header.stamp.toSec();
    double t2 = current_poses.back().header.stamp.toSec();
    t_res = (t - t1) / (t2 - t1);
  }

  // Perform slerp
  tf::Quaternion q1, q2, qres;
  tf::Vector3 v1, v2, vres;
  tf::quaternionMsgToTF(current_poses.front().pose.orientation, q1);
  tf::quaternionMsgToTF(current_poses.back().pose.orientation, q2);
  tf::pointMsgToTF(current_poses.front().pose.position, v1);
  tf::pointMsgToTF(current_poses.back().pose.position, v2);
  qres = tf::slerp(q1, q2, t_res);
  vres = tf::lerp(v1, v2, t_res); // linear interpolation of position

  // Construct result
  tf::poseTFToMsg(tf::Pose(qres, vres), avg.pose);
  avg.header.stamp = ros::Time(t);
  return avg; // either origin, standard average, or interpolation.
}

void lidarMsgCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  if (verbose) ROS_INFO("Callback LIDAR");

  // First Callback
  if (!firstLidarCallback) {
    std::string info_string = std::string("Writing files to ")
                                    + std::string(PATH);
    ROS_INFO("%s", info_string.c_str());
    lidar_frame=msg->header.frame_id;
    while(source_frame == "") {
      callbacks_poses.callOne(ros::WallDuration());
    }
    ROS_INFO("Lidar frame \"%s\", Pose frame \"%s\"", lidar_frame.c_str(), source_frame.c_str());
    tf::TransformListener listener;
    listener.waitForTransform(lidar_frame, source_frame, ros::Time(0), ros::Duration(30.0) );
    listener.lookupTransform(lidar_frame, source_frame, ros::Time(ros::Time(0)), transform_lidar2pose);
    ROS_INFO("Transform found : %f %f %f %f %f %f %f",
      transform_lidar2pose.getOrigin().getX(),
      transform_lidar2pose.getOrigin().getY(),
      transform_lidar2pose.getOrigin().getZ(),
      transform_lidar2pose.getRotation().getX(),
      transform_lidar2pose.getRotation().getY(),
      transform_lidar2pose.getRotation().getZ(),
      transform_lidar2pose.getRotation().getW()
    );

    firstLidarCallback = true;
  }

  // Opening 3d file to write into
  FILE* file_3d;
  char* file_name = new char[PATH_CHAR_LEN]();
  std::sprintf(file_name, "%sscan%03d.3d", PATH, seq);

  auto start_clock_pts = high_resolution_clock::now();
  if (!skip_points) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    // Writing the lidar data to the text file
    file_3d = fopen(file_name, "wb");;
    // This is a left handed coordinate system and we convert the values to cm
    for (size_t i = 0; i < temp_cloud->points.size(); ++i)
    {
        pcl::PointXYZI p = temp_cloud->points[i];
        double dist2 = 100*100*(p.x*p.x+p.y*p.y+p.z*p.z);
        if (std::isnan(dist2) || dist2 < min_dist2 || dist2 > max_dist2 )
          continue;
        // Convert to left handed
        fprintf(file_3d, "%lf %lf %lf %lf\n", 100*-p.y, 100*p.z, 100*p.x, temp_cloud->points[i].intensity);
    }
    fclose(file_3d);
  }
  auto stop_clock_pts = high_resolution_clock::now();

  // Get current pose average
  geometry_msgs::PoseStamped current_pose_avg = interpolate_pose_at(msg->header.stamp.toSec());

  // Getting the current pose as homgeneous transformation (4x4)
  tf::Quaternion q(current_pose_avg.pose.orientation.x,
                  current_pose_avg.pose.orientation.y,
                  current_pose_avg.pose.orientation.z,
                  current_pose_avg.pose.orientation.w);

  // Get pose of LiDAR sensor by applying transform between pose and LiDAR frame
  q = q * transform_lidar2pose.getRotation();
  tf::Matrix3x3 m(q);
  const double in_matrix[16] ={m[0][0],m[0][1],m[0][2], current_pose_avg.pose.position.x + transform_lidar2pose.getOrigin().getX(),
                               m[1][0],m[1][1],m[1][2], current_pose_avg.pose.position.y + transform_lidar2pose.getOrigin().getY(),
                               m[2][0],m[2][1],m[2][2], current_pose_avg.pose.position.z + transform_lidar2pose.getOrigin().getZ(),
			                         0      ,0      ,0      , 1};

  // Converting to left handed matrix for 3DTK (OpenGL style)
  double out_matrix[16], rPos[3], rPosTheta[16];
  to3DTKMat(in_matrix, out_matrix, 1);
  Matrix4ToEuler(out_matrix, rPosTheta, rPos);

  // Extracting Position and Orientaion
  double x = 100.0 * rPos[0]; // left-handed format in 3DTK uses cm instead of m
  double y = 100.0 * rPos[1];
  double z = 100.0 * rPos[2];
  double roll  = 1.0 * rPosTheta[0];
  double pitch = 1.0 * rPosTheta[1];
  double yaw   = 1.0 * rPosTheta[2];
  if (!have_pose_topic) {
      x = 0;
      y = 0;
      z = 0;
      roll = 0;
      pitch = 0;
      yaw = 0;
  }

  // Writing to the pose file
  // This is also a left hand coordinate system
  // - Thumb (x) to the right
  // - Pointy Finger (y) to  the top
  // - Middle Finger (z) into the room
  auto duration_pts = duration_cast<milliseconds>(stop_clock_pts - start_clock_pts);
  ROS_INFO("Writing %s ... [%d ms]", file_name, duration_pts);
  std::sprintf(file_name, "%sscan%03d.pose", PATH, seq);
  FILE* file_pose = fopen(file_name, "wb");
  fprintf(file_pose, "%lf %lf %lf %lf %lf %lf", x, y, z, roll*conv_rad2deg, pitch*conv_rad2deg, yaw*conv_rad2deg);
  fclose(file_pose);
  seq++;
  delete file_name;
}

int main(int argc, char **argv)
{
    // Program information
    ros::init(argc, argv, "file_writer");
    std::cout << "ros_listener - A program that listens to ROS topics" << std::endl;
    std::cout << "               and writes 3DTK files (UOSR-format) on disk." << std::endl;
    std::cout << "               Use -h for more information." << std::endl;
    std::cout << "Note: Remember that you need a running roscore" << std::endl;
    std::cout << "Tip of the day: Start this program first, it will wait for the topics." << std::endl;

    // Declaration of program parameters
    string outdir;
    string lidar_topic;
    string pose_topic;
    int pose_type;
    double min_dist;
    double max_dist;

   // Definition and allocation of parameters
    parse_options(argc, argv,
        outdir, lidar_topic, pose_topic, skip_points,
        pose_type, ignore_stamps, verbose, min_dist, max_dist);

    min_dist2 = min_dist*min_dist;
    max_dist2 = max_dist*max_dist;

    // Create path on disk (if it does not exist yet)
    PATH = outdir.c_str();
    if ( !existsDir( PATH ) ) {
      ROS_WARN("\"%s\" does not exist, I will create it now.", PATH);
      ROS_INFO("Creating \"%s\".", PATH);
      if (!boost::filesystem::create_directory( PATH )) {
        ROS_ERROR("Could not create \"%s\". Aborting now.", PATH);
        return -1; // Unusual program end
      }
    } else ROS_INFO("\"%s\" exists, exporting there.", PATH);

    // Init iteration variable, count number of LiDAR callbacks
    seq = 0; // counts nrscans

    // Skip pose topic if unavailable (pose will contain all 0 values)
    if (pose_topic.compare("") == 0) {
      ROS_WARN("No pose topic given. LiDAR trajectory defaults to origin.");
      have_pose_topic = false;
    }

    // Setting up the pose stream node handle.
    // NOTE: This node handle gets its own callback queue, such that the poses
    // are always up to date and we dont interfere with the lidar thread.
    ros::NodeHandle nh_lidar;
    ros::NodeHandle nh_pose;
    nh_pose.setCallbackQueue(&callbacks_poses);
    ros::Subscriber pose_sub;
    if (have_pose_topic) {
      switch (pose_type) {
        case 1:
          pose_sub = nh_pose.subscribe(pose_topic, 100000, poseMsgCallback);
          break;
        case 2:
          pose_sub = nh_pose.subscribe(pose_topic, 100000, imuMsgCallback);
          break;
      }
    }

    // Lidar Node Handle, this one uses the default callback queue
    ros::Subscriber lidar_sub = nh_lidar.subscribe(lidar_topic, 100000, lidarMsgCallback);

    // Do the ROS spinning loop manually (we have two callback queues)
    ros::Rate sleeprate(200); // Hz
    while ( ros::ok() ) {
      ros::spinOnce(); // standard callback queue for LiDAR data
      callbacks_poses.callOne();
      sleeprate.sleep();
    }

    return 0; // normal program end
}
