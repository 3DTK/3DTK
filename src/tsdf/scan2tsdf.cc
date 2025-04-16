#include <Eigen/Core>
#include <Eigen/Geometry>

#include "slam6d/scan.h"
#include "slam6d/globals.icc"
#include <boost/program_options.hpp>

#include "tsdf/SensorPolar3D.h"
#include "tsdf/TsdSpaceVDB.h"

using namespace std;
namespace po = boost::program_options;

void validate(boost::any& v, const std::vector<std::string>& values, IOType*, int) {
    if (values.size() == 0)
        throw std::runtime_error("Invalid model specification");
    string arg = values.at(0);
    try {
        v = formatname_to_io_type(arg.c_str());
    } catch (...) { // runtime_error
        throw std::runtime_error("Format " + arg + " unknown.");
    }
}

double deg2rad(double v) {
    return v * M_PI / 180.0;
}

int main(int argc, char **argv)
{
    string inDir;
    string outDir;
    IOType iotype;
    int startIndex;
    int endIndex;
    float voxelSize;

    po::options_description generic("Generic options");
    generic.add_options()
            ("help,h", "output this help message");

    po::options_description input("Input options");
    input.add_options()
            ("start,s", po::value<int>(&startIndex)->default_value(0),
             "start at scan <arg> (i.e., neglects the first <arg> scans) "
             "[ATTENTION: counting naturally starts with 0]")
            ("end,e", po::value<int>(&endIndex)->default_value(-1),
             "end after scan <arg>")
            ("format,f", po::value<IOType>(&iotype)->default_value(UOS),
             "using shared library <arg> for input. (chose F from {uos, uos_map, "
             "uos_rgb, uos_frames, uos_map_frames, old, rts, rts_map, ifp, "
             "riegl_txt, riegl_rgb, riegl_bin, zahn, ply})");

    po::options_description output("Output options");
    output.add_options()
            ("output,O", po::value<std::string>(&outDir)->default_value("."), "Output dir");

    po::options_description tsdf("TSDF options");
    tsdf.add_options()
            ("voxelsize,v", po::value<float>(&voxelSize)->default_value(0.1),
             "voxel size <arg>");

    po::options_description hidden("Hidden options");
    hidden.add_options()
            ("input-dir", po::value<string>(&inDir), "input dir");

    po::options_description all;
    all.add(generic).add(input).add(output).add(tsdf).add(hidden);

    po::options_description cmdline_options;
    cmdline_options.add(generic).add(input).add(output);

    po::positional_options_description pd;
    pd.add("input-dir", 1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(all).positional(pd).run(), vm);

    po::notify(vm);

    if (vm.count("help")) {
        std::cout << cmdline_options;
        return 0;
    }

    TsdSpaceVDB space(voxelSize);
    SensorPolar3D sensor(1, deg2rad(3), deg2rad(-91.5), 480, deg2rad(0.25), deg2rad(-180.0 + (90.0 - 59.87)), 1, 30);

    Scan::openDirectory(false, inDir, iotype, startIndex, endIndex);
    if(Scan::allScans.size() == 0) {
        cerr << "No scans found. Did you use the correct format?" << endl;
        exit(-1);
    }

    for (ScanVector::iterator it = Scan::allScans.begin(); it != Scan::allScans.end(); ++it) {
        Scan* scan = *it;
        scan->setRangeFilter(-1, -1);

        DataXYZ xyz(scan->get("xyz"));

        vector<double> data(480);
        std::fill(data.begin(), data.end(), 0);
        for (uint i = 0; i < xyz.size(); i++) {
            double angle = (atan2(-xyz[i][1], xyz[i][0]) * 180.0 / M_PI);

            unsigned int index = round((angle - (90.0 - 59.87))/0.25);

            if (index >=0 && index < 480)
                data[index] = sqrt(xyz[i][1] * xyz[i][1] + xyz[i][0] * xyz[i][0]) / 100.0;
        }

        double rP[3];
        double rPT[3];
        Matrix4ToEuler(scan->get_transMatOrg(), rPT, rP);

        Eigen::Affine3d trafoM = Eigen::Affine3d::Identity();

        trafoM(0,0) = scan->get_transMatOrg()[10];
        trafoM(0,1) = -scan->get_transMatOrg()[2];
        trafoM(0,2) = scan->get_transMatOrg()[6];

        trafoM(1,0) = -scan->get_transMatOrg()[8];
        trafoM(1,1) = scan->get_transMatOrg()[0];
        trafoM(1,2) = -scan->get_transMatOrg()[4];

        trafoM(2,0) = scan->get_transMatOrg()[9];
        trafoM(2,1) = -scan->get_transMatOrg()[1];
        trafoM(2,2) = scan->get_transMatOrg()[5];

        trafoM(0,3) = scan->get_transMatOrg()[14] / 100.0;
        trafoM(1,3) = -scan->get_transMatOrg()[12] / 100.0;
        trafoM(2,3) = scan->get_transMatOrg()[13] / 100.0;

        trafoM = trafoM * Eigen::Affine3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1, 0, 0))) * Eigen::Affine3d(Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d(0, 0, 1)));

        sensor.setPose(trafoM);
        sensor.setData(data);
        space.integrate(&sensor);

        delete *it;
        *it = 0;
    }

    Scan::closeDirectory();

    return 0;
}
