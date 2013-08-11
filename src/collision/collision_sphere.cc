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
#include "slam6d/kd.h"

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

void parse_options(int argc, char **argv, int &start, int &end,
        bool &scanserver, IOType &iotype, string &dir, string &trajectory)
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


    po::options_description hidden("Hidden options");
    hidden.add_options()
        ("input-dir", po::value<string>(&dir), "input dir")
        ("trajectory", po::value<string>(&trajectory), "trajectory");;

    // all options
    po::options_description all;
    all.add(generic).add(input).add(hidden);

    // options visible with --help
    po::options_description cmdline_options;
    cmdline_options.add(generic).add(input);

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
             << "\t" << argv[0] << " -s 0 -e 0 -f riegl_txt ~/path/to/bremen_city\n"; 
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

void write_xyzrgb(std::vector<Point> &points, string &dir, string id)
{
    ofstream outfile((dir + "/scan" + id + ".xyz").c_str());
    for(std::vector<Point>::iterator it = points.begin(); it != points.end(); ++it) {
        outfile << (*it).x << " " << (*it).y << " " << (*it).z << " 0" << endl;
    }
    outfile.close();

}

int main(int argc, char **argv)
{
    // commandline arguments
    int start, end;
    bool scanserver;
    string dir, trajectoryfn;
    IOType iotype;

    parse_options(argc, argv, start, end, scanserver, iotype, dir, trajectoryfn);

    Scan::openDirectory(scanserver, dir, iotype, start, end);

    if(Scan::allScans.size() == 0) {
      cerr << "No scans found. Did you use the correct format?" << endl;
      exit(-1);
    }

    std::vector<Frame> trajectory = read_trajectory(trajectoryfn);

    string colldir;
    colldir = dir + "collides";
    createdirectory(colldir);
    for(ScanVector::iterator it = Scan::allScans.begin(); it != Scan::allScans.end(); ++it) {
        Scan* scan = *it;
        DataXYZ points(scan->get("xyz"));
        double** pa = new double*[points.size()];
        for (size_t i = 0; i < points.size(); ++i) {
            pa[i] = new double[3];
            pa[i][0] = points[i][0];
            pa[i][1] = points[i][1];
            pa[i][2] = points[i][2];
        }
        KDtree t(pa, points.size());
        int thread_num = 0; // add omp later
        double sqRad2 = 6000;
        vector<Point> colliding;
        for(std::vector<Frame>::iterator it = trajectory.begin(); it != trajectory.end(); ++it) {
            double point[3] = {0,0,0};
            transform3((*it).transformation, point);
            vector<Point> collidingsphere = t.fixedRangeSearch(point, sqRad2, thread_num);
            cout << collidingsphere.size() << endl;
            colliding.insert(colliding.end(), collidingsphere.begin(), collidingsphere.end());
        }
        write_xyzrgb(colliding, colldir, scan->getIdentifier());
    }

}
