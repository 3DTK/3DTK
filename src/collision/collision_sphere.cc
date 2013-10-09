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

enum collision_method {CM_SPHERES, CM_MOVING_SPHERE, CM_POINTCLOUD};

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

void validate(boost::any& v, const std::vector<std::string>& values,
        collision_method*, int) {
    if (values.size() == 0)
        throw std::runtime_error("Invalid model specification");
    string arg = values.at(0);
    if(strcasecmp(arg.c_str(), "SPHERES") == 0) v = CM_SPHERES;
    else if(strcasecmp(arg.c_str(), "MOVING_SPHERE") == 0) v = CM_MOVING_SPHERE;
    else if(strcasecmp(arg.c_str(), "POINTCLOUD") == 0) v = CM_POINTCLOUD;
    else throw std::runtime_error(std::string("collision method ") + arg + std::string(" is unknown"));
}

void parse_options(int argc, char **argv, int &start, int &end,
        bool &scanserver, IOType &iotype, string &dir, string &trajectory,
        collision_method &cmethod, double &radius)
{
    po::options_description generic("Generic options");
    generic.add_options()
        ("help,h", "output this help message");

    po::options_description input("Input options");
    input.add_options()
        ("start,s", po::value<int>(&start)->default_value(0),
         "start at scan <arg> (i.e., neglects the first <arg> scans) "
         "[ATTENTION: counting naturally starts with 0]")
        ("end,e", po::value<int>(&end)->default_value(-1),
         "end after scan <arg>")
        ("format,f", po::value<IOType>(&iotype)->default_value(UOS),
         "using shared library <arg> for input. (chose F from {uos, uos_map, "
         "uos_rgb, uos_frames, uos_map_frames, old, rts, rts_map, ifp, "
         "riegl_txt, riegl_rgb, riegl_bin, zahn, ply})")
        ("scanserver,S", po::value<bool>(&scanserver)->default_value(false),
         "Use the scanserver as an input method and handling of scan data");

    po::options_description prog("Program specific options");
    prog.add_options()
        ("method,m", po::value<collision_method>(&cmethod)->
         default_value(CM_SPHERES), "collision method (SPHERES, "
         "MOVING_SPHERE, POINTCLOUD)")
        ("radius,r", po::value<double>(&radius)->default_value(70),
         "radius of sphere");

    po::options_description hidden("Hidden options");
    hidden.add_options()
        ("input-dir", po::value<string>(&dir), "input dir")
        ("trajectory", po::value<string>(&trajectory), "trajectory");;

    // all options
    po::options_description all;
    all.add(generic).add(input).add(prog).add(hidden);

    // options visible with --help
    po::options_description cmdline_options;
    cmdline_options.add(generic).add(input).add(prog);

    // positional argument
    po::positional_options_description pd;
    pd.add("input-dir", 1).add("trajectory", 1);

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
    if (!vm.count("trajectory")) {
        cout << "you have to specify a trajectory" << endl;
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

void write_xyzrgb(DataXYZ &points, string &dir, string id, bool* colliding)
{
    createdirectory(dir+"collides");
    ofstream fcolliding((dir + "collides/scan" + id + ".xyz").c_str());
    createdirectory(dir+"noncollides");
    ofstream fnoncolliding((dir + "noncollides/scan" + id + ".xyz").c_str());
    for (size_t i = 0; i < points.size(); ++i) {
        if (colliding[i]) {
            fcolliding << points[i][2] << " " << -points[i][0] << " " << points[i][1] << " 0" << endl;
        } else {
            fnoncolliding << points[i][2] << " " << -points[i][0] << " " << points[i][1] << " 0" << endl;
        }
    }
    fcolliding.close();
    fnoncolliding.close();
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

void fill_colliding(bool *allcolliding, std::vector<size_t> &newindices)
{
    for(std::vector<size_t>::iterator it = newindices.begin(); it != newindices.end(); ++it) {
        allcolliding[*it] = true;
    }
}

void handle_pointcloud(KDtreeIndexed &t, std::vector<Frame> &trajectory, std::vector<Point> &pointmodel, bool* colliding)
{
    double sqRad2 = 100;
    int thread_num = 0;
    for(std::vector<Frame>::iterator it2 = trajectory.begin(); it2 != trajectory.end(); ++it2) {
        for(std::vector<Point>::iterator it = pointmodel.begin(); it != pointmodel.end(); ++it) {
            Point p = *it;
            double point1[3] = {p.x, p.y, p.z};
            transform3((*it2).transformation, point1);
            vector<size_t> collidingsphere = t.fixedRangeSearch(point1, sqRad2, thread_num);
            fill_colliding(colliding, collidingsphere);
        }
    }
}

int main(int argc, char **argv)
{
    // commandline arguments
    int start, end;
    bool scanserver;
    string dir, trajectoryfn;
    IOType iotype;
    collision_method cmethod;
    double radius;

    parse_options(argc, argv, start, end, scanserver, iotype, dir,
            trajectoryfn, cmethod, radius);

    Scan::openDirectory(scanserver, dir, iotype, start, end);

    if(Scan::allScans.size() == 0) {
      cerr << "No scans found. Did you use the correct format?" << endl;
      exit(-1);
    }

    // read trajectory in *.3d file format
    std::vector<Frame> trajectory = read_trajectory(trajectoryfn);

    ScanVector::iterator it = Scan::allScans.begin();
    vector<Point> pointmodel;

    // if matching against pointcloud, treat the first scan as the model
    if(cmethod == CM_POINTCLOUD) {
        if(Scan::allScans.size() == 1) {
            cerr << "must supply more than one scan (the first is the model)" << endl;
            exit(-1);
        }
        Scan* scan = *it;
        DataXYZ points(scan->get("xyz"));
        pointmodel.reserve(points.size());
        for(unsigned int j = 0; j < points.size(); j++) {
            pointmodel.push_back(Point(points[j][0], points[j][1], points[j][2]));
        }
        ++it;
    }

    for(; it != Scan::allScans.end(); ++it) {
        /* build a KDtree from this scan */
        Scan* scan = *it;
        DataXYZ points(scan->get("xyz"));
        double** pa = new double*[points.size()];
        bool* colliding = new bool[points.size()];
        for (size_t i = 0; i < points.size(); ++i) {
            pa[i] = new double[3];
            pa[i][0] = points[i][0];
            pa[i][1] = points[i][1];
            pa[i][2] = points[i][2];
            colliding[i] = false;
        }
        KDtreeIndexed t(pa, points.size());
        /* initialize variables */
        int thread_num = 0; // add omp later
        double sqRad2 = radius*radius;
        std::vector<Frame>::iterator it2 = trajectory.begin();
        double point1[3] = {0,0,0};
        /* execute according to collision method */
        switch (cmethod) {
            case CM_SPHERES:
                /* for each point in the trajectory, find the colliding points
                 * within the given radius */
                for(; it2 != trajectory.end(); ++it2) {
                    transform3((*it2).transformation, point1);
                    vector<size_t> collidingsphere = t.fixedRangeSearch(point1, sqRad2, thread_num);
                    cout << collidingsphere.size() << endl;
                    fill_colliding(colliding, collidingsphere);
                }
                break;
            case CM_MOVING_SPHERE:
                /* drive the path given by the trajectory and mark all points
                 * within the given radius */
                transform3((*it2).transformation, point1);
                ++it2;
                if (it2 == trajectory.end()) {
                    cout << "need more than one point for moving sphere" << endl;
                    exit(1);
                }
                for(; it2 != trajectory.end(); ++it2) {
                    double point2[3] = {0,0,0};
                    transform3((*it2).transformation, point2);
                    vector<size_t> collidingspherepath = t.fixedRangeSearchBetween2Points(point1, point2, sqRad2, thread_num);
                    cout << collidingspherepath.size() << endl;
                    fill_colliding(colliding, collidingspherepath);
                    point1[0] = point2[0];
                    point1[1] = point2[1];
                    point1[2] = point2[2];
                }
                break;
            case CM_POINTCLOUD:
                handle_pointcloud(t, trajectory, pointmodel, colliding);
                break;
            default:
                cout << "collision method not implemented" << endl;
                exit(1);
        }
        write_xyzrgb(points, dir, scan->getIdentifier(), colliding);
    }

}
