#include "slam6d/globals.icc"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2/buffer_core.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <robot_state_publisher/robot_state_publisher.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/LaserScan.h>

#include <string>
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace boost;

namespace std {
ostream& operator<<(ostream &os, const vector<string> &vec) {
    for (auto item : vec) {
        os << item << " ";
    }
    return os;
}
}

void setTransform(double *transmat, const double *mat, double x, double y, double z) {
    transmat[0] = mat[4];
    transmat[1] = -mat[7];
    transmat[2] = -mat[1];
    transmat[3] = 0.0;

    transmat[4] = -mat[5];
    transmat[5] = mat[8];
    transmat[6] = mat[2];
    transmat[7] = 0.0;

    transmat[8] = -mat[3];
    transmat[9] = mat[6];
    transmat[10] = mat[0];
    transmat[11] = 0.0;

    // translation
    transmat[12] =  -y;
    transmat[13] =  z;
    transmat[14] =  x;
    transmat[15] = 1;
}

class StaticTransformSetter
{
    class SegmentPair
    {
    public:
        SegmentPair(const KDL::Segment& p_segment, const std::string& p_root, const std::string& p_tip):
            segment(p_segment), root(p_root), tip(p_tip){}

        KDL::Segment segment;
        std::string root, tip;
    };

public:
    StaticTransformSetter(const KDL::Tree& tree, const urdf::Model& model = urdf::Model()) : model_(model) {
        addChildren(tree.getRootSegment());
    }

public:
    void setFixedTransforms(tf2::BufferCore* tfBuffer, ros::Time t) {
        std::vector<geometry_msgs::TransformStamped> tf_transforms;
        geometry_msgs::TransformStamped tf_transform;

        for (map<string, SegmentPair>::const_iterator seg=segments_fixed_.begin(); seg != segments_fixed_.end(); seg++) {
            geometry_msgs::TransformStamped tf_transform = tf2::kdlToTransform(seg->second.segment.pose(0));
            const KDL::Frame k = seg->second.segment.pose(0);

            double qx,qy,qz,qw;
            k.M.GetQuaternion(qx, qy, qz, qw);

            geometry_msgs::TransformStamped trafo;
            trafo.header.stamp = t;
            trafo.header.frame_id = seg->second.root;
            trafo.child_frame_id = seg->second.tip;
            trafo.transform.translation = tf2::toMsg(tf2::Vector3(k.p.x(), k.p.y(), k.p.z()));
            trafo.transform.rotation = tf2::toMsg(tf2::Quaternion(qx, qy, qz, qw));
            tfBuffer->setTransform(trafo, "default_authority", true);
        }
    }

protected:
    void addChildren(const KDL::SegmentMap::const_iterator segment) {
        const std::string& root = GetTreeElementSegment(segment->second).getName();

        const std::vector<KDL::SegmentMap::const_iterator>& children = GetTreeElementChildren(segment->second);
        for (unsigned int i=0; i<children.size(); i++) {
            const KDL::Segment& child = GetTreeElementSegment(children[i]->second);
            SegmentPair s(GetTreeElementSegment(children[i]->second), root, child.getName());
            if (child.getJoint().getType() == KDL::Joint::None) {
                if (model_.getJoint(child.getJoint().getName()) && model_.getJoint(child.getJoint().getName())->type == urdf::Joint::FLOATING) {
                    ROS_INFO("Floating joint. Not adding segment from %s to %s. This TF can not be published based on joint_states info", root.c_str(), child.getName().c_str());
                }
                else {
                    segments_fixed_.insert(make_pair(child.getJoint().getName(), s));
                    ROS_DEBUG("Adding fixed segment from %s to %s", root.c_str(), child.getName().c_str());
                }
            }
            else {
                segments_.insert(make_pair(child.getJoint().getName(), s));
                ROS_DEBUG("Adding moving segment from %s to %s", root.c_str(), child.getName().c_str());
            }
            addChildren(children[i]);
        }
    }

    std::map<std::string, SegmentPair> segments_, segments_fixed_;
    const urdf::Model& model_;
};

struct PointCloudWithTransform {
    ros::Time timestamp;
    geometry_msgs::TransformStamped pose;
    geometry_msgs::TransformStamped calibration;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudI;
};

void writePointClouds(const string& outdir, double scale, double minDistance, double maxDistance, const vector<PointCloudWithTransform>& pointClouds) {
    if (pointClouds.size() < 1) return;

    static long index = 0;
    long numPoints = 0;

    string scanFilename = outdir + "/scan" + to_string(index,3) + ".3d";
    FILE* scanFile = fopen(scanFilename.c_str(), "wb");
    cout << "Writing " << scanFilename << endl;

    Eigen::Affine3d firstPose = tf2::transformToEigen(pointClouds.at(0).pose);
    Eigen::Affine3d firstPoseInverse = firstPose.inverse();

    for (size_t i = 0; i < pointClouds.size(); i++) {
        const PointCloudWithTransform& pointCloud = pointClouds.at(i);

        Eigen::Affine3d baseToLaser = tf2::transformToEigen(pointCloud.calibration);
        Eigen::Affine3d currentPose = tf2::transformToEigen(pointClouds.at(i).pose);
        Eigen::MatrixXd mapToLaser = (firstPoseInverse * currentPose * baseToLaser).matrix();

        if (pointCloud.cloud != 0) {
            for (pcl::PointXYZ p : *pointCloud.cloud) {
                double distance = Eigen::Vector3d(p.x, p.y, p.z).norm() / scale;
                if (std::isnan(distance) || distance < minDistance || distance > maxDistance) continue;

                Eigen::Vector4d tmp(p.x, p.y, p.z, 1);
                Eigen::Vector4d pcorr = mapToLaser * tmp;

                fprintf(scanFile, "%lf %lf %lf\n", -pcorr(1) / scale, pcorr(2) / scale, pcorr(0) / scale);
                numPoints++;
            }
        } else if (pointCloud.cloudRGB != 0) {
           for (pcl::PointXYZRGB p : *pointCloud.cloudRGB) {
                double distance = Eigen::Vector3d(p.x, p.y, p.z).norm() / scale;
                if (std::isnan(distance) || distance < minDistance || distance > maxDistance) continue;

                Eigen::Vector4d tmp(p.x, p.y, p.z, 1);
                Eigen::Vector4d pcorr = mapToLaser * tmp;

                fprintf(scanFile, "%lf %lf %lf %d %d %d\n", -pcorr(1) / scale, pcorr(2) / scale, pcorr(0) / scale, (int) p.r, (int) p.g, (int) p.b);
                numPoints++;
            }
        } else if (pointCloud.cloudI != 0) {
           for (pcl::PointXYZI p : *pointCloud.cloudI) {
                double distance = Eigen::Vector3d(p.x, p.y, p.z).norm() / scale;
                if (std::isnan(distance) || distance < minDistance || distance > maxDistance) continue;

                Eigen::Vector4d tmp(p.x, p.y, p.z, 1);
                Eigen::Vector4d pcorr = mapToLaser * tmp;

                fprintf(scanFile, "%lf %lf %lf %lf\n", -pcorr(1) / scale, pcorr(2) / scale, pcorr(0) / scale, p.intensity);

                numPoints++;
            }
        }
    }

    fclose(scanFile);

    Eigen::Affine3d pose = tf2::transformToEigen(pointClouds.at(0).pose);

    double rotmat[9];
    for (int m = 0; m < 3; m++) {
        for (int n = 0; n < 3; n++) {
            rotmat[m*3 + n] = pose.rotation()(m, n);
        }
    }

    double transmat[16];
    setTransform(transmat, rotmat, pose.translation().x() / scale, pose.translation().y() / scale, pose.translation().z() / scale);

    ofstream o;
    string poseFileName = outdir + "/scan" + to_string(index,3) + ".pose";
    double rP[3];
    double rPT[3];
    Matrix4ToEuler(transmat, rPT, rP);
    o.open(poseFileName.c_str());
    o << lexical_cast<string>(rP[0]) << " " << lexical_cast<string>(rP[1]) << " " << lexical_cast<string>(rP[2]) << endl;
    o << lexical_cast<string>(deg(rPT[0])) << " " << lexical_cast<string>(deg(rPT[1])) << " " << lexical_cast<string>(deg(rPT[2])) << endl;
    o << pointClouds.at(0).timestamp << endl;
    o.flush();
    o.close();

    // skip empty scans
    if (numPoints > 0) index++;
}

int main(int argc, char* argv[])
{
    string bagFile;
    vector<string> trajectoryFiles;
    vector<string> topicsPointCloud2;
    vector<string> topicsMultiEchoLaserScan;
    vector<string> topicsLaserScan;
    vector<string> topicsTF;
    vector<string> topicsTFStatic;
    string urdffile;
    string mapFrame;
    string baseFrame;
    double startTime;
    double endTime;
    double scale;
    double minDistance;
    double maxDistance;
    size_t combine;
    string outdir;
    bool exportColor;
    bool exportIntensity;
    bool useReceiveTimestamps;

    program_options::options_description desc("Allowed options");
    desc.add_options()
            ("help,h", "produce help message")
            ("bag,b", program_options::value<string>(&bagFile)->required(), "input ros bag file")
            ("trajectory,t", program_options::value<vector<string> >(&trajectoryFiles)->multitoken()->default_value(vector<string>()), "input trajectory files")
            ("topics-PointCloud2", program_options::value<vector<string> >(&topicsPointCloud2)->multitoken()->default_value(vector<string> {"horizontal_laser_3d", "vertical_laser_3d"}), "Topics with PointCloud2 messages for export")
            ("topics-MultiEchoLaserScan", program_options::value<vector<string> >(&topicsMultiEchoLaserScan)->multitoken()->default_value(vector<string> {"horizontal_laser_2d", "vertical_laser_2d"}), "Topics with MultiEchoLaserScan messages for export")
            ("topics-LaserScan", program_options::value<vector<string> >(&topicsLaserScan)->multitoken()->default_value(vector<string>()), "Topics with LaserScan messages for export")
            ("topics-TF", program_options::value<vector<string> >(&topicsTF)->multitoken()->default_value(vector<string> {"/tf"}), "Topics with TF information")
            ("topics-TF-static", program_options::value<vector<string> >(&topicsTFStatic)->multitoken()->default_value(vector<string> {"/tf_static"}), "Topics with static TF information")
            ("urdf", program_options::value<string>(&urdffile), "input URDF file")
            ("frame-map", program_options::value<string>(&mapFrame)->required()->default_value("map"), "frame id of the map")
            ("frame-base", program_options::value<string>(&baseFrame)->required()->default_value("base_link"), "frame id of the robot base link")

            ("start-time", program_options::value<double>(&startTime)->default_value(0), "Start timestamp of export")
            ("end-time", program_options::value<double>(&endTime)->default_value(4102444800), "End timestamp of export")
            ("scale", program_options::value<double>(&scale)->default_value(0.01), "Scale of exported point cloud")
            ("min,M", program_options::value<double>(&minDistance)->default_value(0.0), "Neglect all points closer than this to the origin")
            ("max,m", program_options::value<double>(&maxDistance)->default_value(std::numeric_limits<double>::max()), "Neglect all points further than this from the origin")
            ("combine", program_options::value<size_t>(&combine)->default_value(1), "Combine n scans")
            ("color,C", "Export with color")
            ("intensity,I", "Export with intensity")
            ("use-receive-timestamps", "Use the timestamps of the messages for bad bagfiles")
            ("output,o", program_options::value<string>(&outdir)->required(), "output folder")
            ;

    program_options::variables_map vm;
    program_options::store(program_options::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        cout << desc << endl;
        return EXIT_SUCCESS;
    }

    exportColor = vm.count("color");
    exportIntensity = vm.count("intensity");
    useReceiveTimestamps = vm.count("use-receive-timestamps");

    program_options::notify(vm);

    boost::filesystem::create_directory(outdir);


    StaticTransformSetter* transformSetter = NULL;
    if (urdffile.size() > 0) {
        urdf::Model model;

        if (!model.initFile(urdffile)) {
            cerr << "Failed parsing URDF file." << endl;
            return -1;
        }

        KDL::Tree tree;
        if (!kdl_parser::treeFromUrdfModel(model, tree)) {
            ROS_ERROR("Failed to extract kdl tree from xml robot description");
            return -1;
        }

        transformSetter = new StaticTransformSetter(tree, model);
    }


    rosbag::Bag bag(bagFile);
    rosbag::View bagView(bag);
    tf2::BufferCore tfBuffer(bagView.getEndTime() - bagView.getBeginTime());

    if (transformSetter) {
        transformSetter->setFixedTransforms(&tfBuffer, bagView.getBeginTime());
    }

    if (trajectoryFiles.size() < 1) {
        trajectoryFiles.push_back(bagFile);
    }

    for (string trajectoryFile : trajectoryFiles) {
        cout << "Reading trajectory from " << trajectoryFile << "." << endl;
#ifdef BOOST_FILESYSTEM_CONVENIENCE_HPP
	string extension = boost::filesystem::extension(trajectoryFile);
#else
	string extension = boost::filesystem::path(trajectoryFile).extension().string();
#endif
        if (extension.compare(".bag") == 0) {
            rosbag::Bag bag(trajectoryFile);

            rosbag::View tfviewStatic(bag, rosbag::TopicQuery(topicsTFStatic));
            for (rosbag::MessageInstance const m : tfviewStatic) {
                if (m.isType<tf2_msgs::TFMessage>()) {
                    tf2_msgs::TFMessageConstPtr tfm = m.instantiate<tf2_msgs::TFMessage>();
                    for (geometry_msgs::TransformStamped t : tfm->transforms) {
                        if (useReceiveTimestamps) {
                            t.header.stamp = m.getTime();
                        }

                        tfBuffer.setTransform(t, "default_authority", true);
                    }
                }
            }

            rosbag::View tfview(bag, rosbag::TopicQuery(topicsTF));
            for (rosbag::MessageInstance const m : tfview) {
                if (m.isType<tf2_msgs::TFMessage>()) {
                    tf2_msgs::TFMessageConstPtr tfm = m.instantiate<tf2_msgs::TFMessage>();
                    for (geometry_msgs::TransformStamped t : tfm->transforms) {
                        if (useReceiveTimestamps) {
                            t.header.stamp = m.getTime();
                        }

                        tfBuffer.setTransform(t, "default_authority", false);
                    }
                }
            }
        } else {
            ifstream data(trajectoryFile);

            string line;
            while(getline(data,line)) {
                trim(line);

                vector<string> words;
                split(words, line, is_any_of("\t "), token_compress_on);

                if (words.at(0).find("\%") != string::npos) {
                    continue;
                }

                if (words.size() != 8) {
                    cout << "ERROR: Read invalid line" << endl;
                    continue;
                }

                double time, x, y, z, qw, qx, qy, qz;

                try {
                    time = stod(words.at(0));
                    x = stod(words.at(1));
                    y = stod(words.at(2));
                    z = stod(words.at(3));
                    qw = stod(words.at(4));
                    qx = stod(words.at(5));
                    qy = stod(words.at(6));
                    qz = stod(words.at(7));
                } catch (...) {
                    cout << "ERROR: Read invalid line" << endl;
                    continue;
                }

                Eigen::Affine3d pose = Eigen::Affine3d::Identity();
                pose.translate(Eigen::Vector3d(x, y, z));
                pose.rotate(Eigen::Quaterniond(qw, qx, qy, qz));

                geometry_msgs::TransformStamped trafo = tf2::eigenToTransform(pose);
                trafo.header.stamp = ros::Time(time);
                trafo.header.frame_id = mapFrame;
                trafo.child_frame_id = baseFrame;
                tfBuffer.setTransform(trafo, "default_authority", false);

                cout << setprecision(20) << time << endl;
            }
        }
    }

    cout << "Done reading trajectory." << endl;


    if (topicsPointCloud2.size() > 0) {
        rosbag::Bag bag(bagFile);
        rosbag::View viewScans(bag, rosbag::TopicQuery(topicsPointCloud2));

        vector<PointCloudWithTransform> pointClouds;

        for (rosbag::MessageInstance const m : viewScans) {
            sensor_msgs::PointCloud2ConstPtr message = m.instantiate<sensor_msgs::PointCloud2>();

            ros::Time stamp = message->header.stamp;
            if (useReceiveTimestamps) {
                stamp = m.getTime();
            }

            if (stamp < ros::Time(startTime) || stamp > ros::Time(endTime)) {
                continue;
            }

            geometry_msgs::TransformStamped baseTransform;
            geometry_msgs::TransformStamped laserTransform;
            try {
                baseTransform = tfBuffer.lookupTransform (mapFrame, baseFrame, stamp);
                laserTransform = tfBuffer.lookupTransform (baseFrame, message->header.frame_id, stamp);
            } catch (tf2::TransformException e) {
                cout << "Failed to look up transforms! " << e.what() << endl;
                continue;
            }

            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(*message,pcl_pc2);

            PointCloudWithTransform pointCloud;
            pointCloud.timestamp = stamp;
            pointCloud.pose = baseTransform;
            pointCloud.calibration = laserTransform;

            if (exportColor) {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud_rgb);

                pointCloud.cloud = 0;
                pointCloud.cloudRGB = temp_cloud_rgb;
                pointCloud.cloudI = 0;
            } else if(exportIntensity) {
                pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud_i(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud_i);

                pointCloud.cloud = 0;
                pointCloud.cloudRGB = 0;
                pointCloud.cloudI = temp_cloud_i;
            } else {
                pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

                pointCloud.cloud = temp_cloud;
                pointCloud.cloudRGB = 0;
                pointCloud.cloudI = 0;
            }

            pointClouds.push_back(pointCloud);

            if (pointClouds.size() >= combine) {
                writePointClouds(outdir, scale, minDistance, maxDistance, pointClouds);
                pointClouds.clear();
            }
        }

        writePointClouds(outdir, scale, minDistance, maxDistance, pointClouds);
        pointClouds.clear();
    }


    if (topicsMultiEchoLaserScan.size() > 0) {
        rosbag::Bag bag(bagFile);
        rosbag::View viewScans(bag, rosbag::TopicQuery(topicsMultiEchoLaserScan));

        vector<PointCloudWithTransform> pointClouds;

        for (rosbag::MessageInstance const m : viewScans) {
            sensor_msgs::MultiEchoLaserScanConstPtr message = m.instantiate<sensor_msgs::MultiEchoLaserScan>();

            ros::Time stamp = message->header.stamp;
            if (useReceiveTimestamps) {
                stamp = m.getTime();
            }


            if (stamp < ros::Time(startTime) || stamp > ros::Time(endTime)) {
                continue;
            }

            geometry_msgs::TransformStamped baseTransform;
            geometry_msgs::TransformStamped laserTransform;
            try {
                baseTransform = tfBuffer.lookupTransform (mapFrame, baseFrame, stamp);
                laserTransform = tfBuffer.lookupTransform (baseFrame, message->header.frame_id, stamp);
            } catch (tf2::TransformException e) {
                cout << "Failed to look up transforms! " << e.what() << endl;
                continue;
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

            for (size_t i = 0; i < message->ranges.size(); i++) {
                double angle = message->angle_min + (double) i * message->angle_increment;
                for (float echo : message->ranges.at(i).echoes) {
                    if (std::isnan(echo) || echo < 0.1 || echo > 30.0) continue;

                    float x = echo * cos(angle);
                    float y = echo * sin(angle);
                    float z = 0;

                    temp_cloud->push_back(pcl::PointXYZ(x, y, z));
                }
            }

            PointCloudWithTransform pointCloud;
            pointCloud.timestamp = stamp;
            pointCloud.pose = baseTransform;
            pointCloud.calibration = laserTransform;
            pointCloud.cloud = temp_cloud;
            pointCloud.cloudRGB = 0;
            pointCloud.cloudI = 0;

            pointClouds.push_back(pointCloud);

            if (pointClouds.size() >= combine) {
                writePointClouds(outdir, scale, minDistance, maxDistance, pointClouds);
                pointClouds.clear();
            }
        }

        writePointClouds(outdir, scale, minDistance, maxDistance, pointClouds);
        pointClouds.clear();
    }


    if (topicsLaserScan.size() > 0) {
        rosbag::Bag bag(bagFile);
        rosbag::View viewScans(bag, rosbag::TopicQuery(topicsLaserScan));

        vector<PointCloudWithTransform> pointClouds;

        for (rosbag::MessageInstance const m : viewScans) {
            sensor_msgs::LaserScanConstPtr message = m.instantiate<sensor_msgs::LaserScan>();

            ros::Time stamp = message->header.stamp;
            if (useReceiveTimestamps) {
                stamp = m.getTime();
            }


            if (stamp < ros::Time(startTime) || stamp > ros::Time(endTime)) {
                continue;
            }

            geometry_msgs::TransformStamped baseTransform;
            geometry_msgs::TransformStamped laserTransform;
            try {
                baseTransform = tfBuffer.lookupTransform (mapFrame, baseFrame, stamp);
                laserTransform = tfBuffer.lookupTransform (baseFrame, message->header.frame_id, stamp);
            } catch (tf2::TransformException e) {
                cout << "Failed to look up transforms! " << e.what() << endl;
                continue;
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

            for (size_t i = 0; i < message->ranges.size(); i++) {
                double angle = message->angle_min + (double) i * message->angle_increment;
                float echo = message->ranges.at(i);

                if (std::isnan(echo) || echo < 0.1 || echo > 30.0) continue;

                float x = echo * cos(angle);
                float y = echo * sin(angle);
                float z = 0;

                temp_cloud->push_back(pcl::PointXYZ(x, y, z));
            }

            PointCloudWithTransform pointCloud;
            pointCloud.timestamp = stamp;
            pointCloud.pose = baseTransform;
            pointCloud.calibration = laserTransform;
            pointCloud.cloud = temp_cloud;
            pointCloud.cloudRGB = 0;
            pointCloud.cloudI = 0;

            pointClouds.push_back(pointCloud);

            if (pointClouds.size() >= combine) {
                writePointClouds(outdir, scale, minDistance, maxDistance, pointClouds);
                pointClouds.clear();
            }
        }

        writePointClouds(outdir, scale, minDistance, maxDistance, pointClouds);
        pointClouds.clear();
    }

    delete transformSetter;

    return 0;
}
