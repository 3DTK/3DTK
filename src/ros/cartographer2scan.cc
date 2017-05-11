#include "slam6d/globals.icc"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

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

#include <string>
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace boost;

template <typename It>
typename std::iterator_traits<It>::value_type median(It begin, It end)
{
    auto size = std::distance(begin, end);
    std::nth_element(begin, begin + size / 2, end);
    return *std::next(begin, size / 2);
}

void setTransform(double *transmat, double *mat, double x, double y, double z)
{
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

void addStaticTransforms(tf::Transformer *l, ros::Time t)
{
    tf::Quaternion q;

    q.setRPY(0, -0.1745, 3.1416);
    l->setTransform(tf::StampedTransform(tf::Transform(q, tf::Vector3(0.01, 0, 0.19)), t, "/base_link", "/horizontal_vlp16_link"));

    q.setRPY(0, 1.3963, 0);
    l->setTransform(tf::StampedTransform(tf::Transform(q, tf::Vector3(0.19, 0, 0.04)), t, "/base_link", "/vertical_vlp16_link"));
}

int main(int argc, char* argv[])
{
    string bagfile;
    string trajectoryfile;
    string outdir;

    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
            ("help,h", "produce help message")
            ("bag,b", boost::program_options::value<std::string>(&bagfile)->required(), "input ros bag file")
            ("trajectory,t", boost::program_options::value<std::string>(&trajectoryfile)->required(), "input trajectory file")
            ("output,o", boost::program_options::value<std::string>(&outdir)->required(), "output folder")
            ;

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return EXIT_SUCCESS;
    }

    boost::program_options::notify(vm);

    boost::filesystem::create_directory(outdir);


    std::ifstream data(trajectoryfile);

    tf::Transformer *l = new tf::Transformer(true, ros::Duration(3600));

    std::string line;
    while(std::getline(data,line))
    {
        boost::trim(line);

        std::vector<std::string> words;
        boost::split(words, line, boost::is_any_of("\t "), boost::token_compress_on);

        if (words.at(0).find("\%") != string::npos) {
            continue;
        }

        if (words.size() != 8) {
            cout << "ERROR: Read invalid line" << endl;
            continue;
        }

        double time, x, y, z, qw, qx, qy, qz;

        try {
            time = std::stod(words.at(0));
            x = std::stod(words.at(1));
            y = std::stod(words.at(2));
            z = std::stod(words.at(3));
            qw = std::stod(words.at(4));
            qx = std::stod(words.at(5));
            qy = std::stod(words.at(6));
            qz = std::stod(words.at(7));
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
        trans = tf::StampedTransform(rosTF, ros::Time(time), "/map", "/base_link" );

        l->setTransform(trans);
        addStaticTransforms(l, trans.stamp_);

        cout << std::setprecision(20) << time << endl;
    }

    cout << "Done reading trajectory." << endl;


    rosbag::Bag bag(bagfile);

    vector<string> topicsScans;
    topicsScans.push_back("horizontal_laser_3d");
    topicsScans.push_back("vertical_laser_3d");
    rosbag::View tfviewScans(bag, rosbag::TopicQuery(topicsScans));

    tf::StampedTransform startTransform;
    long index = 0;

    for (rosbag::MessageInstance const m : tfviewScans) {
        sensor_msgs::PointCloud2ConstPtr message = m.instantiate<sensor_msgs::PointCloud2>();

        tf::StampedTransform baseTransform;
        tf::StampedTransform laserTransform;
        try {
            l->lookupTransform ("/map", "/base_link", m.getTime(), baseTransform);
            l->lookupTransform ("/base_link", message->header.frame_id, m.getTime(), laserTransform);
        } catch (...) {
            continue;
        }

        Eigen::Affine3d baseToLaser;
        tf::transformTFToEigen(laserTransform, baseToLaser);

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*message,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

        string scanFilename = outdir + "/scan" + to_string(index,3) + ".3d";
        cout << "Writing " << scanFilename << endl;

        ofstream scanFile(scanFilename);
        scanFile << fixed << setprecision(2);

        for (pcl::PointXYZ p : *temp_cloud) {

            Eigen::Vector4d tmp(p.x, p.y, p.z, 1);
            Eigen::Vector4d pcorr = baseToLaser.matrix() * tmp;

            float xout = -pcorr(1);
            float yout = pcorr(2);
            float zout = pcorr(0);

            scanFile << xout * 100.0 << " " << yout * 100.0 << " " << zout * 100.0 << endl;
        }

        scanFile.close();

        tf::Transform transform = baseTransform;

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

        std::ofstream o;
        string poseFileName = outdir + "/scan" + to_string(index,3) + ".pose";
        double rP[3];
        double rPT[3];
        Matrix4ToEuler(transmat, rPT, rP);
        o.open(poseFileName.c_str());
        o << lexical_cast<string>(rP[0]) << " " << lexical_cast<string>(rP[1]) << " " << lexical_cast<string>(rP[2]) << endl;
        o << lexical_cast<string>(deg(rPT[0])) << " " << lexical_cast<string>(deg(rPT[1])) << " " << lexical_cast<string>(deg(rPT[2])) << endl;
        o << m.getTime() << endl;
        o.flush();
        o.close();

        index++;
    }

    return 0;
}
