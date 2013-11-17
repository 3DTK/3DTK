/*
 * collision_sphere
 *
 * Copyright (C) Johannes Schauer
 *
 * Released under the GPL version 3.
 *
 */

#include "slam6d/scan.h"
#include "slam6d/frame.h"
#include "slam6d/globals.icc"
#include "slam6d/kdIndexed.h"
#include "scanio/scan_io.h"

#include <sys/stat.h>
#include <sys/types.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

/*
 * validates input type specification
 */
void validate(boost::any& v, const std::vector<std::string>& values,
    IOType*, int) {
    if (values.size() == 0)
        throw std::runtime_error("Invalid model specification");
    string arg = values.at(0);
    try {
        v = formatname_to_io_type(arg.c_str());
    } catch (...) { // runtime_error
        throw std::runtime_error("Format " + arg + " unknown.");
    }
}

void parse_options(int argc, char **argv, IOType &iotype, string &dir, double &radius)
{
    po::options_description generic("Generic options");
    generic.add_options()
        ("help,h", "output this help message");

    po::options_description input("Input options");
    input.add_options()
        ("format,f", po::value<IOType>(&iotype)->default_value(UOS),
         "using shared library <arg> for input. (chose F from {uos, uos_map, "
         "uos_rgb, uos_frames, uos_map_frames, old, rts, rts_map, ifp, "
         "riegl_txt, riegl_rgb, riegl_bin, zahn, ply})");

    po::options_description prog("Program specific options");
    prog.add_options()
        ("radius,r", po::value<double>(&radius)->default_value(10),
         "radius of sphere");

    po::options_description hidden("Hidden options");
    hidden.add_options()
        ("input-dir", po::value<string>(&dir), "input dir");

    // all options
    po::options_description all;
    all.add(generic).add(input).add(prog).add(hidden);

    // options visible with --help
    po::options_description cmdline_options;
    cmdline_options.add(generic).add(input).add(prog);

    // positional argument
    po::positional_options_description pd;
    pd.add("input-dir", 1);

    // process options
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
              options(all).positional(pd).run(), vm);
    po::notify(vm);

    // display help
    if (vm.count("help")) {
        cout << cmdline_options;
        cout << "\nExample usage:\n"
             << "\t" << argv[0] << " --method POINTCLOUD -s 0 -e 1 -f xyzr ../scantest ../scantest/scan000.path\n";
        exit(0);
    }

    if (!vm.count("input-dir")) {
        cout << "you have to specify an input directory" << endl;
        exit(1);
    }
    if (dir[dir.length()-1] != '/') dir = dir + "/";
}

void createdirectory(string segdir)
{
    int success = mkdir(segdir.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
    if (success == 0 || errno == EEXIST) {
        cout << "Writing colliding points to " << segdir << endl;
    } else {
        cerr << "Creating directory " << segdir << " failed" << endl;
        exit(1);
    }
}

std::vector<Frame> read_trajectory(string filename)
{
    std::ifstream file(filename.c_str());
    if (!file.good()) {
        cout << "can't open " << filename << " for reading" << endl;
        exit(1);
    }
    std::vector<Frame> positions;
    double transformation[16];
    unsigned int type;
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        iss >> transformation >> type;
        positions.push_back(Frame(transformation, type));
    }
    return positions;
}

void write_xyzr(DataXYZ &points, string &dir, std::vector<bool> const &colliding)
{
    cerr << "writing colliding points to " << dir << "scan002.xyz" << endl;
    ofstream fcolliding((dir + "scan002.xyz").c_str());
    cerr << "writing non-colliding points to " << dir << "scan003.xyz" << endl;
    ofstream fnoncolliding((dir + "scan003.xyz").c_str());
    for (size_t i = 0; i < points.size(); ++i) {
        if (colliding[i]) {
            fcolliding << points[i][2] << " " << -points[i][0] << " " << points[i][1] << " 0" << endl;
        } else {
            fnoncolliding << points[i][2] << " " << -points[i][0] << " " << points[i][1] << " 0" << endl;
        }
    }
    fcolliding.close();
    fnoncolliding.close();
    ofstream pcolliding((dir + "scan002.pose").c_str());
    pcolliding << "0 0 0" << endl << "0 0 0" << endl;
    pcolliding.close();
    ofstream pnoncolliding((dir + "scan003.pose").c_str());
    pnoncolliding << "0 0 0" << endl << "0 0 0" << endl;
    pnoncolliding.close();
}

/*
std::vector<Point> read_plymodel(string &pointmodelpath)
{
    IOType m_type = PLY;
    ScanIO* sio = ScanIO::getScanIO(m_type);
    PointFilter filter;
    vector<double> xyz;
    vector<unsigned char> rgb;
    vector<float> reflectance;
    sio->readScan(pointmodelpath.c_str(), "model", filter, &xyz, &rgb, &reflectance, NULL, NULL, NULL, NULL);
    vector<Point> points;
    for(std::vector<double>::iterator it = xyz.begin(); it != xyz.end();) {
        double x = *it; ++it;
        double y = *it; ++it;
        double z = *it; ++it;
        points.push_back(Point(x,y,z));
    }
    return points;
}
*/

void fill_colliding(std::vector<bool> &allcolliding, std::vector<size_t> const &newindices)
{
    for(auto &it : newindices) {
        allcolliding[it] = true;
    }
}

size_t handle_pointcloud(DataXYZ &model, DataXYZ &environ,
                       std::vector<Frame> const &trajectory,
                       std::vector<bool> &colliding,
                       double radius)
{
    cerr << "reading model..." << endl;
    vector<Point> pointmodel;
    pointmodel.reserve(model.size());
    for(unsigned int j = 0; j < model.size(); j++) {
        pointmodel.push_back(Point(model[j][0], model[j][1], model[j][2]));
    }
    cerr << "model: " << pointmodel.size() << endl;

    /* build a KDtree from this scan */
    cerr << "reading environment..." << endl;
    double** pa = new double*[environ.size()];
    size_t i;
    for (i = 0; i < environ.size(); ++i) {
        pa[i] = new double[3];
        pa[i][0] = environ[i][0];
        pa[i][1] = environ[i][1];
        pa[i][2] = environ[i][2];
        colliding[i] = false;
    }
    cerr << "environment: " << i-1 << endl;
    cerr << "building kd tree..." << endl;
    KDtreeIndexed t(pa, environ.size());
    /* initialize variables */
    int thread_num = 0; // add omp later
    double sqRad2 = radius*radius;
    cerr << "computing collisions..." << endl;
    for(auto &it2 : trajectory) {
        for(auto &it : pointmodel) {
            double point1[3] = {it.x, it.y, it.z};
            transform3(it2.transformation, point1);
            vector<size_t> collidingsphere = t.fixedRangeSearch(point1, sqRad2, thread_num);
            fill_colliding(colliding, collidingsphere);
        }
    }
    size_t num_colliding = 0;
    // the actual implementation of std::vector<bool> requires us to use the
    // proxy iterator pattern with &&...
    for (auto &&it : colliding) {
        if (it) {
            num_colliding++;
        }
    }
    for (i = 0; i < environ.size(); ++i) {
        delete[] pa[i];
    }
    delete[] pa;
    return num_colliding;
}

int main(int argc, char **argv)
{
    // commandline arguments
    string dir;
    IOType iotype;
    double radius;

    parse_options(argc, argv, iotype, dir, radius);

    // read scan 0 (model) and 1 (environment) without scanserver
    Scan::openDirectory(false, dir, iotype, 0, 1);

    if(Scan::allScans.size() == 0) {
      cerr << "No scans found. Did you use the correct format?" << endl;
      exit(-1);
    }

    // trajectory is the *.frames file of the model
    string trajectoryfn = dir+"scan000.frames";

    cerr << "reading trajectory from " << trajectoryfn << endl;
    // read trajectory in *.3d file format
    std::vector<Frame> trajectory = read_trajectory(trajectoryfn);

    ScanVector::iterator it = Scan::allScans.begin();

    // if matching against pointcloud, treat the first scan as the model
    if(Scan::allScans.size() != 2) {
        cerr << "must supply two scans, the model and the environment in that order" << endl;
        exit(-1);
    }
    DataXYZ model(it[0]->get("xyz"));
    DataXYZ environ(it[1]->get("xyz"));
    std::vector<bool> colliding;
    colliding.reserve(environ.size());
    size_t num_colliding = handle_pointcloud(model, environ, trajectory, colliding, radius);
    //calculate_collidingdist();
    write_xyzr(environ, dir, colliding);
}
