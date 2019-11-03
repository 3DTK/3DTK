#include "slam6d/globals.icc"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <robot_state_publisher/robot_state_publisher.h>
#include <tf2_kdl/tf2_kdl.h>

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

void setTransform(double *transmat, double *mat, double x, double y, double z) {
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
    void setFixedTransforms(tf::Transformer *l, ros::Time t) {
        std::vector<geometry_msgs::TransformStamped> tf_transforms;
        geometry_msgs::TransformStamped tf_transform;

        for (map<string, SegmentPair>::const_iterator seg=segments_fixed_.begin(); seg != segments_fixed_.end(); seg++) {
            geometry_msgs::TransformStamped tf_transform = tf2::kdlToTransform(seg->second.segment.pose(0));
            const KDL::Frame k = seg->second.segment.pose(0);

            double qx,qy,qz,qw;
            k.M.GetQuaternion(qx, qy, qz, qw);
            tf::Quaternion q(qx, qy, qz, qw);

            l->setTransform(tf::StampedTransform(tf::Transform(q, tf::Vector3(k.p.x(), k.p.y(), k.p.z())), t, seg->second.root, seg->second.tip));
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
    tf::Transform pose;
    tf::Transform calibration;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

void writePointClouds(const string& outdir, double scale, double minDistance, double maxDistance, const vector<PointCloudWithTransform>& pointClouds) {
    if (pointClouds.size() < 1) return;

    static long index = 0;

    string scanFilename = outdir + "/scan" + to_string(index,3) + ".3d";
    cout << "Writing " << scanFilename << endl;

    ofstream scanFile(scanFilename);
    scanFile << fixed << setprecision(2);

    Eigen::Affine3d firstPose;
    tf::transformTFToEigen(pointClouds.at(0).pose, firstPose);
    Eigen::Affine3d firstPoseInverse = firstPose.inverse();

    for (size_t i = 0; i < pointClouds.size(); i++) {
        const PointCloudWithTransform& pointCloud = pointClouds.at(i);

        Eigen::Affine3d baseToLaser;
        tf::transformTFToEigen(pointCloud.calibration, baseToLaser);

        Eigen::Affine3d currentPose;
        tf::transformTFToEigen(pointClouds.at(i).pose, currentPose);

        Eigen::MatrixXd mapToLaser = (firstPoseInverse * currentPose * baseToLaser).matrix();

        for (pcl::PointXYZ p : *pointCloud.cloud) {
            double distance = Eigen::Vector3d(p.x, p.y, p.z).norm() / scale;
            if (std::isnan(distance) || distance < minDistance || distance > maxDistance) continue;

            Eigen::Vector4d tmp(p.x, p.y, p.z, 1);
            Eigen::Vector4d pcorr = mapToLaser * tmp;

            float xout = -pcorr(1);
            float yout = pcorr(2);
            float zout = pcorr(0);

            scanFile << xout / scale << " " << yout / scale << " " << zout / scale << endl;
        }
    }

    scanFile.close();

    tf::Transform transform = pointClouds.at(0).pose;

    double X = transform.getOrigin().getX()*100;
    double Y = transform.getOrigin().getY()*100;
    double Z = transform.getOrigin().getZ()*100;

    double rotmat[9];

    rotmat[0] = transform.getBasis().getRow(0).getX();
    rotmat[1] = transform.getBasis().getRow(0).getY();
    rotmat[2] = transform.getBasis().getRow(0).getZ();

    rotmat[3] = transform.getBasis().getRow(1).getX();
    rotmat[4] = transform.getBasis().getRow(1).getY();
    rotmat[5] = transform.getBasis().getRow(1).getZ();

    rotmat[6] = transform.getBasis().getRow(2).getX();
    rotmat[7] = transform.getBasis().getRow(2).getY();
    rotmat[8] = transform.getBasis().getRow(2).getZ();

    double transmat[16];
    setTransform(transmat, rotmat, X, Y, Z);

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

    index++;
}

int main(int argc, char* argv[])
{
    string bagFile;
    vector<string> trajectoryFiles;
    vector<string> topicsPointCloud2;
    vector<string> topicsMultiEchoLaserScan;
    vector<string> topicsLaserScan;
    vector<string> topicsTF;
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

    program_options::options_description desc("Allowed options");
    desc.add_options()
            ("help,h", "produce help message")
            ("bag,b", program_options::value<string>(&bagFile)->required(), "input ros bag file")
            ("trajectory,t", program_options::value<vector<string> >(&trajectoryFiles)->multitoken()->default_value(vector<string>()), "input trajectory files")
            ("topics-PointCloud2", program_options::value<vector<string> >(&topicsPointCloud2)->multitoken()->default_value(vector<string> {"horizontal_laser_3d", "vertical_laser_3d"}), "Topics with PointCloud2 messages for export")
            ("topics-MultiEchoLaserScan", program_options::value<vector<string> >(&topicsMultiEchoLaserScan)->multitoken()->default_value(vector<string> {"horizontal_laser_2d", "vertical_laser_2d"}), "Topics with MultiEchoLaserScan messages for export")
            ("topics-LaserScan", program_options::value<vector<string> >(&topicsLaserScan)->multitoken()->default_value(vector<string>()), "Topics with LaserScan messages for export")
            ("topics-TF", program_options::value<vector<string> >(&topicsTF)->multitoken()->default_value(vector<string> {"/tf"}), "Topics with TF information")
            ("urdf", program_options::value<string>(&urdffile)->required(), "input URDF file")
            ("frame-map", program_options::value<string>(&mapFrame)->required()->default_value("/map"), "frame id of the map")
            ("frame-base", program_options::value<string>(&baseFrame)->required()->default_value("/base_link"), "frame id of the robot base link")

            ("start-time", program_options::value<double>(&startTime)->default_value(0), "Start timestamp of export")
            ("end-time", program_options::value<double>(&endTime)->default_value(4102444800), "End timestamp of export")
            ("scale", program_options::value<double>(&scale)->default_value(0.01), "Scale of exported point cloud")
            ("min,M", program_options::value<double>(&minDistance)->default_value(0.0), "Neglect all points closer than this to the origin")
            ("max,m", program_options::value<double>(&maxDistance)->default_value(std::numeric_limits<double>::max()), "Neglect all points further than this from the origin")
            ("combine", program_options::value<size_t>(&combine)->default_value(1), "Combine n scans")
            ("output,o", program_options::value<string>(&outdir)->required(), "output folder")
            ;

    program_options::variables_map vm;
    program_options::store(program_options::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        cout << desc << endl;
        return EXIT_SUCCESS;
    }

    program_options::notify(vm);

    filesystem::create_directory(outdir);


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


    tf::Transformer *l = NULL;

    if (trajectoryFiles.size() < 1) {
        trajectoryFiles.push_back(bagFile);
    }

    for (string trajectoryFile : trajectoryFiles) {
        cout << "Reading trajectory from " << trajectoryFile << "." << endl;
        string extension = boost::filesystem::extension(trajectoryFile);

        if (extension.compare(".bag") == 0) {
            rosbag::Bag bag(trajectoryFile);
            rosbag::View tfview(bag, rosbag::TopicQuery(topicsTF));

            l = new tf::Transformer(true, ros::Duration(3600));//tfview.getEndTime() - tfview.getBeginTime());

            for (rosbag::MessageInstance const m : tfview) {
                if (m.isType<tf::tfMessage>()) {
                    tf::tfMessageConstPtr tfm = m.instantiate<tf::tfMessage>();
                    for (unsigned int i = 0; i < tfm->transforms.size(); i++) {
                        tf::StampedTransform trans;
                        transformStampedMsgToTF(tfm->transforms[i], trans);

                        l->setTransform(trans);

                        if (transformSetter) { transformSetter->setFixedTransforms(l, trans.stamp_);}
                    }
                }
            }
        } else {

            l = new tf::Transformer(true, ros::Duration(3600));

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

                Eigen::Affine3d transform = Eigen::Affine3d::Identity();
                transform.translate(Eigen::Vector3d(x, y, z));
                transform.rotate(Eigen::Quaterniond(qw, qx, qy, qz));

                Eigen::Matrix4d pose = transform.matrix();

                tf::Transform rosTF;
                tf::transformEigenToTF(Eigen::Affine3d(pose), rosTF);

                tf::StampedTransform trans;
                trans = tf::StampedTransform(rosTF, ros::Time(time), mapFrame, baseFrame);

                l->setTransform(trans);
                if (transformSetter) { transformSetter->setFixedTransforms(l, trans.stamp_);}

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

            if (message->header.stamp < ros::Time(startTime) || message->header.stamp > ros::Time(endTime)) {
                continue;
            }

            tf::StampedTransform baseTransform;
            tf::StampedTransform laserTransform;
            try {
                l->lookupTransform (mapFrame, baseFrame, message->header.stamp, baseTransform);
                l->lookupTransform (baseFrame, message->header.frame_id, message->header.stamp, laserTransform);
            } catch (tf::TransformException e) {
                cout << "Failed to look up transforms! " << e.what() << endl;
                continue;
            }

            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(*message,pcl_pc2);
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

            PointCloudWithTransform pointCloud;
            pointCloud.timestamp = message->header.stamp;
            pointCloud.pose = baseTransform;
            pointCloud.calibration = laserTransform;
            pointCloud.cloud = temp_cloud;

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

            if (message->header.stamp < ros::Time(startTime) || message->header.stamp > ros::Time(endTime)) {
                continue;
            }

            tf::StampedTransform baseTransform;
            tf::StampedTransform laserTransform;
            try {
                l->lookupTransform (mapFrame, baseFrame, message->header.stamp, baseTransform);
                l->lookupTransform (baseFrame, message->header.frame_id, message->header.stamp, laserTransform);
            } catch (...) {
                cout << "Failed to look up transforms!" << endl;
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
            pointCloud.timestamp = message->header.stamp;
            pointCloud.pose = baseTransform;
            pointCloud.calibration = laserTransform;
            pointCloud.cloud = temp_cloud;

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

            if (message->header.stamp < ros::Time(startTime) || message->header.stamp > ros::Time(endTime)) {
                continue;
            }

            tf::StampedTransform baseTransform;
            tf::StampedTransform laserTransform;
            try {
                l->lookupTransform (mapFrame, baseFrame, message->header.stamp, baseTransform);
                l->lookupTransform (baseFrame, message->header.frame_id, message->header.stamp, laserTransform);
            } catch (...) {
                cout << "Failed to look up transforms!" << endl;
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
            pointCloud.timestamp = message->header.stamp;
            pointCloud.pose = baseTransform;
            pointCloud.calibration = laserTransform;
            pointCloud.cloud = temp_cloud;

            pointClouds.push_back(pointCloud);

            if (pointClouds.size() >= combine) {
                writePointClouds(outdir, scale, minDistance, maxDistance, pointClouds);
                pointClouds.clear();
            }
        }

        writePointClouds(outdir, scale, minDistance, maxDistance, pointClouds);
        pointClouds.clear();
    }

    return 0;
}
