#include <iostream>
#include <iomanip>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <Eigen/Eigen>
#include "riegl/scanlib.hpp"
#include "slam6d/globals.icc"

using namespace std;
using namespace scanlib;
namespace po = boost::program_options;

template <typename It>
typename iterator_traits<It>::value_type median(It begin, It end)
{
    auto size = distance(begin, end);
    nth_element(begin, begin + size / 2, end);
    return *next(begin, size / 2);
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

class GPS : public pointcloud
{
public:
    struct GpsInfo {
        GpsInfo() :
            timestamp(0),
            lat(0),
            lon(0),
            alt(0),
            accuracy(numeric_limits<double>::max())
        {}

        double timestamp; // gps time of week in sec
        double lat; // latitude in rad
        double lon; // longitude in rad
        double alt; // altitude above ellipsoid in m
        double accuracy; // position accuracy in m
    };

    struct InclInfo {
        InclInfo() :
            roll(0),
            pitch(0)
        {}

        double roll; // rad
        double pitch; // rad
    };

public:
    GPS()
        : pointcloud(false)
    {}
protected:
    void on_hk_gps_hr(const hk_gps_hr<iterator_type>& arg) {
        pointcloud::on_hk_gps_hr(arg);

        GpsInfo info;
        info.timestamp = (double) arg.TOWms / 1000.0;
        info.lat = (double) arg.LAT * 1e-9 * M_PI / 180;
        info.lon = (double) arg.LONG * 1e-9 * M_PI / 180;
        info.alt = (double) arg.HGT_ELL * 1e-3;
        info.accuracy = (double) arg.POS_ACC * 1e-3;

        _gpsInfo.push_back(info);
    }

    void on_hk_incl(const hk_incl<iterator_type>& arg) {
        pointcloud::on_hk_incl(arg);

        double roll = (double) arg.ROLL * 1e-3 * M_PI / 180;
        double pitch = (double) arg.PITCH * 1e-3 * M_PI / 180;

        _rollVector.push_back(roll);
        _pitchVector.push_back(pitch);
    }

public:
    vector<GpsInfo> _gpsInfo;
    vector<double> _rollVector;
    vector<double> _pitchVector;
};

int main(int argc, char* argv[])
{
    string inDir;
    int startIndex;
    int endIndex;
    double yawAngleDeg;

    po::options_description generic("Generic options");
    generic.add_options()
            ("help,h", "produce help message");

    po::options_description input("Input options");
    input.add_options()
            ("start,s", po::value<int>(&startIndex)->default_value(0),
             "start at scan <arg> (i.e., neglects the first <arg> scans) "
             "[ATTENTION: counting naturally starts with 0]")
            ("end,e", po::value<int>(&endIndex)->default_value(-1),
             "end after scan <arg>")
            ("yaw", po::value<double>(&yawAngleDeg)->default_value(0),
             "yaw angle of the scanner from north in degrees");

    po::options_description hidden("Hidden options");
    hidden.add_options()
            ("input-dir", po::value<string>(&inDir), "input dir");

    po::options_description all;
    all.add(generic).add(input).add(hidden);

    po::options_description cmdline_options;
    cmdline_options.add(generic).add(input);

    po::positional_options_description pd;
    pd.add("input-dir", 1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(all).positional(pd).run(), vm);

    po::notify(vm);

    if (vm.count("help")) {
        cout << cmdline_options;
        return 0;
    }


    Eigen::Affine3d startTransform = Eigen::Affine3d::Identity();

    for (int index = startIndex; index <= endIndex; index++) {
        string filename = inDir + "/scan" + to_string(index,3) + ".rxp";

        cout << "Reading " << filename << "..." << endl;

        shared_ptr<basic_rconnection> rc;
        rc = basic_rconnection::create(filename);
        rc->open();
        decoder_rxpmarker dec(rc);
        buffer buf;
        GPS importer;
        for ( dec.get(buf); !dec.eoi() ; dec.get(buf) ) {
            importer.dispatch(buf.begin(), buf.end());
        }
        rc->close();

        cout << "Read " << importer._gpsInfo.size() << " gps messages." << endl;
        cout << "Read " << importer._rollVector.size() << " inclination sensor messages." << endl;

        if (importer._gpsInfo.size() < 1 || importer._rollVector.size() < 1) {
            cout << "Failed to read gps or inclination sensor info!" << endl;
            cout << "Continue with next scan." << endl << endl;

            continue;
        }

        GPS::GpsInfo bestGpsInfo;
        double bestAccuracy = numeric_limits<double>::max();
        for (GPS::GpsInfo gpsInfo : importer._gpsInfo) {
            if (gpsInfo.accuracy < bestAccuracy) {
                bestAccuracy = gpsInfo.accuracy;
                bestGpsInfo = gpsInfo;
            }
        }

        cout << "Lat: " << bestGpsInfo.lat * 180 / M_PI << " deg" << endl;
        cout << "Lon: " << bestGpsInfo.lon * 180 / M_PI  << " deg" << endl;
        cout << "Alt: " << bestGpsInfo.alt << " m" << endl;
        cout << "Accuracy: " << bestGpsInfo.accuracy << " m" << endl;

        double roll = median(importer._rollVector.begin(), importer._rollVector.end());
        double pitch = median(importer._pitchVector.begin(), importer._pitchVector.end());
        double yaw = yawAngleDeg * M_PI / 180;

        cout << "Roll: " << roll * 180 / M_PI << " deg" << endl;
        cout << "Pitch: " << pitch * 180 / M_PI << " deg" << endl;
        cout << "Yaw: " << yaw * 180 / M_PI << " deg" << endl;

        double lat = bestGpsInfo.lat;
        double lon = bestGpsInfo.lon;
        double alt = bestGpsInfo.alt;

        double Rea=6378137;
        double f=1/298.257223563;
        double Reb=Rea*(1-f);
        double e=sqrt(Rea*Rea - Reb*Reb)/Rea;
        double Ne=Rea/sqrt(1 - e*e*sin(lat)*sin(lat));

        double xe = (Ne + alt) * cos(lat) * cos(lon);
        double ye = (Ne + alt) * cos(lat) * sin(lon);
        double ze = (Ne*(1 -e*e) + alt) * sin(lat);

        Eigen::Affine3d translation(Eigen::Translation3d(Eigen::Vector3d(xe, ye, ze)));

        // Transform ECEF frame to NED frame
        Eigen::Affine3d ecef2ned = Eigen::Affine3d(Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d(0, 1, 0))) *
                Eigen::Affine3d(Eigen::AngleAxisd(lon, Eigen::Vector3d(1, 0, 0))) *
                Eigen::Affine3d(Eigen::AngleAxisd(-lat, Eigen::Vector3d(0, 1, 0)));

        // Transform NED frame to ROS frame (x front, y left, z up)
        Eigen::Affine3d ned2ros = Eigen::Affine3d(Eigen::AngleAxisd(-M_PI, Eigen::Vector3d(1, 0, 0)));

        // Local rotation from inclination sensors
        Eigen::Affine3d rotation = Eigen::Affine3d(Eigen::AngleAxisd(-roll, Eigen::Vector3d(1, 0, 0))) *
                Eigen::Affine3d(Eigen::AngleAxisd(-pitch, Eigen::Vector3d(0, 1, 0))) *
                Eigen::Affine3d(Eigen::AngleAxisd(yaw, Eigen::Vector3d(0, 0, 1)));

        Eigen::Affine3d transform = ((translation * ecef2ned) * ned2ros) * rotation;

        // Use relative pose to ROS frame of pose at startIndex
        if (index == startIndex) {
            startTransform = ((translation * ecef2ned) * ned2ros);
        }

        Eigen::Affine3d pose = startTransform.inverse() * transform;

        // Write pose file
        double X = pose.translation()(0) * 100;
        double Y = pose.translation()(1) * 100;
        double Z = pose.translation()(2) * 100;

        double rotmat[9];
        rotmat[0] = pose.rotation()(0,0);
        rotmat[1] = pose.rotation()(0,1);
        rotmat[2] = pose.rotation()(0,2);

        rotmat[3] = pose.rotation()(1,0);
        rotmat[4] = pose.rotation()(1,1);
        rotmat[5] = pose.rotation()(1,2);

        rotmat[6] = pose.rotation()(2,0);
        rotmat[7] = pose.rotation()(2,1);
        rotmat[8] = pose.rotation()(2,2);

        double transmat[16];
        setTransform(transmat, rotmat, X, Y, Z);

        ofstream o;
        string poseFileName = inDir + "/scan" + to_string(index,3) + ".pose";
        cout << "Writing " << poseFileName << endl;
        double rP[3];
        double rPT[3];
        Matrix4ToEuler(transmat, rPT, rP);
        o.open(poseFileName.c_str());
        o << rP[0] << " " << rP[1] << " " << rP[2] << endl;
        o << deg(rPT[0]) << " " << deg(rPT[1]) << " " << deg(rPT[2]) << endl;
        o.flush();
        o.close();

        cout << endl;
    }

    return 0;
}
