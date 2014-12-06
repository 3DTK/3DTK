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

#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

enum collision_method { CTYPE1, CTYPE2, CTYPE3 };
enum penetrationdepth_method { PDTYPE1, PDTYPE2 };

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
    if (strcasecmp(arg.c_str(), "type1") == 0) v = CTYPE1;
    else if (strcasecmp(arg.c_str(), "type2") == 0) v = CTYPE2;
    else if (strcasecmp(arg.c_str(), "type3") == 0) v = CTYPE3;
    else throw std::runtime_error(std::string("collision method ")
            + arg + std::string(" is unknown"));
}

void validate(boost::any& v, const std::vector<std::string>& values,
        penetrationdepth_method*, int) {
    if (values.size() == 0)
        throw std::runtime_error("Invalid model specification");
    string arg = values.at(0);
    if (strcasecmp(arg.c_str(), "type1") == 0) v = PDTYPE1;
    else if (strcasecmp(arg.c_str(), "type2") == 0) v = PDTYPE2;
    else throw std::runtime_error(std::string("penetration depth method ")
            + arg + std::string(" is unknown"));
}

void parse_options(int argc, char **argv, IOType &iotype, string &dir,
        double &radius, bool &calcdistances, collision_method &cmethod,
        penetrationdepth_method &pdmethod)
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
         "radius of sphere")
        ("calcdistances,d", po::value<bool>(&calcdistances)->zero_tokens(),
         "calculate penetration distance")
        ("collisionmethod,c", po::value<collision_method>(&cmethod)->default_value(CTYPE1))
        ("penetrationdepthmethod,p", po::value<penetrationdepth_method>(&pdmethod)->default_value(PDTYPE1));

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
             << "\t" << argv[0] << " --radius=10 -f xyzr ./dir\n";
        cout << "The directory ./dir must contain the model as the first scan and\n";
        cout << "the environment as the second scan. The frames file of the first\n";
        cout << "scan is the trajectory.";
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

void write_xyzr(DataXYZ &points, string &dir,
        std::vector<bool> const &colliding,
        std::vector<float> const &dist_colliding)
{
    cerr << "writing colliding points to " << dir << "scan002.xyz" << endl;
    ofstream fcolliding((dir + "scan002.xyz").c_str());
    cerr << "writing non-colliding points to " << dir << "scan003.xyz" << endl;
    ofstream fnoncolliding((dir + "scan003.xyz").c_str());
    for (size_t i = 0, j=0; i < points.size(); ++i) {
        if (colliding[i]) {
            fcolliding << points[i][2] << " " << -points[i][0] << " " << points[i][1] << " " << dist_colliding[j] << endl;
            j++;
        } else {
            fnoncolliding << points[i][2] << " " << -points[i][0] << " " << points[i][1] << " 1000" << endl;
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
    for(auto it : newindices) {
        allcolliding[it] = true;
    }
}

size_t handle_pointcloud(std::vector<Point> &pointmodel, DataXYZ &environ,
                       std::vector<Frame> const &trajectory,
                       std::vector<bool> &colliding,
                       double radius, collision_method cmethod)
{
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
    cerr << "environment: " << i << endl;
    cerr << "building kd tree..." << endl;
    KDtreeIndexed t(pa, environ.size());
    /* initialize variables */
    int thread_num = 0; // add omp later
    double sqRad2 = radius*radius;
    cerr << "computing collisions..." << endl;
    time_t before = time(NULL);
    int end;
    i = 0;
    switch (cmethod) {
        case CTYPE1:
            end = trajectory.size();
            for(const auto &it2 : trajectory) {
                cerr << (i*100.0)/end << " %\r";
                cerr.flush();
                for(const auto &it : pointmodel) {
                    double point1[3] = {it.x, it.y, it.z};
                    transform3(it2.transformation, point1);
                    vector<size_t> collidingsphere = t.fixedRangeSearch(point1, sqRad2, thread_num);
                    fill_colliding(colliding, collidingsphere);
                }
                i++;
            }
            break;
        case CTYPE2:
            end = pointmodel.size();
            // we iterate over points instead of points on the trajectory so
            // that we can reuse the previous transformation of the same point
            for(const auto &it : pointmodel) {
                cerr << (i*100.0)/end << " %\r";
                cerr.flush();
                auto it2 = trajectory.begin();
                double point1[3], point2[3];
                point1[0] = it.x;
                point1[1] = it.y;
                point1[2] = it.z;
                transform3(it2->transformation, point1);
                ++it2;
                for (; it2 < trajectory.end(); ++it2) {
                    point2[0] = it.x;
                    point2[1] = it.y;
                    point2[2] = it.z;
                    transform3(it2->transformation, point2);
                    vector<size_t> collidingsegment = t.segmentSearch_all(point1, point2, sqRad2, thread_num);
                    fill_colliding(colliding, collidingsegment);
                    point1[0] = point2[0];
                    point1[1] = point2[1];
                    point1[2] = point2[2];
                }
                i++;
            }
            break;
        default:
            throw std::runtime_error("impossible");
    }
    size_t num_colliding = 0;
    // the actual implementation of std::vector<bool> requires us to use the
    // proxy iterator pattern with &&...
    for (i = 0; i < environ.size(); ++i) {
        if (colliding[i]) {
            num_colliding++;
        }
    }
    time_t after = time(NULL);
    cerr << "colliding: " << num_colliding << endl;
    cerr << "took: " << difftime(after, before) << " seconds" << endl;
    for (i = 0; i < environ.size(); ++i) {
        delete[] pa[i];
    }
    delete[] pa;
    return num_colliding;
}

void calculate_collidingdist(DataXYZ &environ,
                             std::vector<bool> const &colliding,
                             size_t num_colliding,
                             std::vector<float> &dist_colliding)
{
    /* build a kdtree for the non-colliding points */
    cerr << "reading environment..." << endl;
    size_t num_noncolliding = environ.size() - num_colliding;
    double** pa = new double*[num_noncolliding];
    size_t* idxmap = new size_t[num_noncolliding];
    for (size_t i = 0, j = 0; i < environ.size(); ++i) {
        if (colliding[i]) {
            continue;
        }
        pa[j] = new double[3];
        pa[j][0] = environ[i][0];
        pa[j][1] = environ[i][1];
        pa[j][2] = environ[i][2];
        idxmap[j] = i;
        j++;
    }
    cerr << "noncolliding: " << num_noncolliding << endl;
    cerr << "building kd tree..." << endl;
    KDtreeIndexed t(pa, num_noncolliding);
    int thread_num = 0; // add omp later
    cerr << "computing distances..." << endl;
    time_t before = time(NULL);
    int j = 0;
    for (size_t i = 0; i < environ.size(); ++i) {
        if (!colliding[i]) {
            continue;
        }
        cerr << (j*100.0)/num_colliding << " %\r";
        cerr.flush();
        double point1[3] = {environ[i][0], environ[i][1], environ[i][2]};
        // for this colliding point, find the closest non-colliding one
        size_t c = t.FindClosest(point1, 1000000, thread_num);
        /*if (colliding[c]) {
            cerr << "result cannot be part of colliding points" << endl;
            cerr << environ[i][0] << " " << environ[i][1] << " " << environ[i][2] << endl;
            cerr << environ[c][0] << " " << environ[c][1] << " " << environ[c][2] << endl;
            exit(1);
        }*/
        c = idxmap[c];
        double point2[3] = {environ[c][0], environ[c][1], environ[c][2]};
        dist_colliding[j] = sqrt(Dist2(point1, point2));
        j++;
    }
    time_t after = time(NULL);
    cerr << "took: " << difftime(after, before) << " seconds" << endl;
    for (size_t i = 0; i < num_noncolliding; ++i) {
        delete[] pa[i];
    }
    delete[] pa;
    delete[] idxmap;
}

void calculate_collidingdist2(std::vector<Point> &pointmodel, DataXYZ &environ,
                             std::vector<Frame> const &trajectory,
                             std::vector<bool> const &colliding,
                             size_t num_colliding,
                             std::vector<float> &dist_colliding,
                             double radius)
{
    /* build a kdtree for colliding points */
    cerr << "reading environment..." << endl;
    double** pa = new double*[num_colliding];
    size_t* idxmap = new size_t[num_colliding];
    for (size_t i = 0, j = 0; i < environ.size(); ++i) {
        if (!colliding[i]) {
            continue;
        }
        pa[j] = new double[3];
        pa[j][0] = environ[i][0];
        pa[j][1] = environ[i][1];
        pa[j][2] = environ[i][2];
        idxmap[j] = i;
        j++;
    }
    cerr << "colliding: " << num_colliding << endl;
    cerr << "building kd tree..." << endl;
    KDtreeIndexed t(pa, num_colliding);
    double sqRad2 = radius*radius;
    int thread_num = 0; // add omp later
    cerr << "computing distances..." << endl;
    time_t before = time(NULL);
    int i = 0;
    int end = trajectory.size();
    for(const auto &it2 : trajectory) {
        cerr << (i*100.0)/end << " %\r";
        cerr.flush();
        for(const auto &it : pointmodel) {
            double point1[3] = {it.x, it.y, it.z};
            // the second point is the projection of the first point to the
            // y-axis of the model
            double point2[3] = {0, it.y, 0};
            transform3(it2.transformation, point1);
            transform3(it2.transformation, point2);
            size_t c1 = t.segmentSearch_1NearestPoint(point1, point2, sqRad2, thread_num);
            if (c1 < std::numeric_limits<size_t>::max()) {
                size_t c2 = idxmap[c1];
                point2[0] = environ[c2][0];
                point2[1] = environ[c2][1];
                point2[2] = environ[c2][2];
                double dist2 = Dist2(point1, point2);
                // now get all points around this closest point to mark them
                // with the same penetration distance
                vector<size_t> closestsphere = t.fixedRangeSearch(pa[c1], sqRad2, thread_num);
                for (const auto &it3 : closestsphere) {
                    if (dist_colliding[it3] < dist2) {
                        dist_colliding[it3] = dist2;
                    }
                }
            }
        }
        i++;
    }
    for (size_t i = 0; i < dist_colliding.size(); ++i) {
        dist_colliding[i] = sqrt(dist_colliding[i]);
    }
    time_t after = time(NULL);
    cerr << "took: " << difftime(after, before) << " seconds" << endl;
    for (size_t i = 0; i < num_colliding; ++i) {
        delete[] pa[i];
    }
    delete[] pa;
    delete[] idxmap;
}

int main(int argc, char **argv)
{
    // commandline arguments
    string dir;
    IOType iotype;
    double radius;
    bool calcdistances;
    collision_method cmethod;
    penetrationdepth_method pdmethod;

    parse_options(argc, argv, iotype, dir, radius, calcdistances, cmethod, pdmethod);

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
    cerr << "reading model..." << endl;
    vector<Point> pointmodel;
    pointmodel.reserve(model.size());
    for(unsigned int j = 0; j < model.size(); j++) {
        pointmodel.push_back(Point(model[j][0], model[j][1], model[j][2]));
    }
    cerr << "model: " << pointmodel.size() << endl;
    size_t num_colliding = 0;
    if (cmethod == CTYPE3) {
        num_colliding = environ.size();
        for (unsigned int i = 0; i < environ.size(); ++i) {
            colliding[i] = true;
        }
    } else {
        num_colliding = handle_pointcloud(pointmodel, environ, trajectory, colliding, radius, cmethod);
    }
    if (num_colliding == 0) {
        cerr << "nothing collides" << endl;
        exit(0);
    }
    std::vector<float> dist_colliding;
    dist_colliding.reserve(num_colliding);
    for (size_t i = 0; i < num_colliding; ++i) {
        dist_colliding.push_back(0);
    }
    if (calcdistances) {
        switch (pdmethod) {
            case PDTYPE1:
                calculate_collidingdist(environ, colliding, num_colliding, dist_colliding);
                break;
            case PDTYPE2:
                calculate_collidingdist2(pointmodel, environ, trajectory, colliding, num_colliding, dist_colliding, radius);
                break;
        }
    }
    write_xyzr(environ, dir, colliding, dist_colliding);
}
