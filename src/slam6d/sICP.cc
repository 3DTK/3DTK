/**
 * @file
 * @brief Main programm for matching 3D scans (6D SLAM) with given * correspondences.
 *
 * @author Dorit Borrmann
 */

#ifdef _MSC_VER
#ifdef OPENMP
#define _OPENMP
#endif
#endif

#ifdef _OPENMP
#include <omp.h>
#endif

#define WANT_STREAM ///< define the WANT stream :)

#include <string>
using std::string;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <fstream>
using std::ifstream;

#include "slam6d/scan.h"

#include "slam6d/icp6Dapx.h"
#include "slam6d/icp6Dsvd.h"
#include "slam6d/icp6Dquat.h"
#include "slam6d/icp6Dortho.h"
#include "slam6d/icp6Dhelix.h"
#include "slam6d/icp6Ddual.h"
#include "slam6d/icp6Dlumeuler.h"
#include "slam6d/icp6Dlumquat.h"
#include "slam6d/icp6Dquatscale.h"
#include "slam6d/icp6D.h"
#include "slam6d/globals.icc"

#include <csignal>

#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#else
#include <strings.h>
#endif

#include <boost/program_options.hpp>
namespace po = boost::program_options;

void parse_options(int argc,char **argv, string& global, string& local, string& out, bool& scale, int& count){

  po::options_description generic("Generic options");
  generic.add_options()
    ("help,h", "output this help message");

  po::options_description input("Input options");
  input.add_options()
    ("global,g", po::value<string>(&global)->default_value("global.dat"),
    "global (target) points are given in 'FILE'")
    ("local,l", po::value<string>(&local)->default_value("local.dat"),
    "local (source) points are given in 'FILE'")
    ("out,o", po::value<string>(&out)->default_value("out.dat"),
    "store transformation in 'FILE'")
    ("scale,S", po::bool_switch(&scale)->default_value(false),
    "use ICP with scale'")
    ("nrpoints,n", po::value<int>(&count)->default_value(0),
    "use first NR correspondences'");

  // all options
  po::options_description all;
  all.add(generic).add(input);

  // options visible with --help
  po::options_description cmdline_options;
  cmdline_options.add(generic).add(input);

  // positional argument
  po::positional_options_description pd;
  pd.add("input-dir", 1);

  // process options
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).
            options(all).positional(pd).run(), vm);

  // display help
  if (vm.count("help")) {
    std::cout << cmdline_options;
    std::cout << std::endl
         << "Example usage:" << std::endl
         << "\t./bin/pose2frames -s 0 -e 1 /Your/directory" << std::endl;
    exit(0);
  }
  po::notify(vm);


}

void usage(char* prog)
{
#ifndef _MSC_VER
  const std::string bold("\033[1m");
  const std::string normal("\033[m");
#else
  const std::string bold("");
  const std::string normal("");
#endif
  std::cout << std::endl
	  << bold << "USAGE " << normal << std::endl
	  << "   " << prog << " [options] directory" << std::endl << std::endl;
  std::cout << bold << "OPTIONS" << normal << std::endl
	  << std::endl
	  << bold << "  -n" << normal << " NR, " << bold << "--nrpoints=" << normal << "NR" << std::endl
	  << "         use first NR correspondences'" << std::endl
	  << std::endl
	  << bold << "  -g" << normal << " FILE, " << bold << "--global=" << normal << "FILE" << std::endl
	  << "         global (target) points are given in 'FILE'" << std::endl
	  << std::endl
	  << bold << "  -l" << normal << " FILE, " << bold << "--local=" << normal << "FILE" << std::endl
	  << "         local (source) points are given in 'FILE'" << std::endl
	  << std::endl
	  << bold << "  -o" << normal << " FILE, " << bold << "--out=" << normal << "FILE" << std::endl
	  << "         store transformation in 'FILE'" << std::endl
	  << std::endl
	  << bold << "  -S" << normal << " " << bold << "--scale=" << normal << std::endl
	  << "         use ICP with scale'" << std::endl
	  << std::endl;
}

/**
 * Main program for 6D SLAM.
 * Usage: bin/sICP
 * with 'dir' the directory of a set of scans
 * ...
 */
int main(int argc, char **argv)
{
  string global;
  string local;
  string out;
  bool scale = false;
  int nr_pts = 0;

  parse_options(argc, argv, global, local, out, scale, nr_pts);

  if(nr_pts < 3) {
    cerr << "Cannot work with less than 3 correspondences!";
    usage(argv[0]);
    abort();
  }

  ifstream fileg(global.c_str());
  ifstream filel(local.c_str());

  if(!fileg.is_open()) {
    cerr << global << " not found!" << endl;
    usage(argv[0]);
    abort();
  }
  if(!filel.is_open()) {
    cerr << local << " not found!" << endl;
    usage(argv[0]);
    abort();
  }
  vector<PtPair> Pairs;

  double x1,x2,y1,y2,z1,z2, centroid_m[3], centroid_d[3];
  centroid_m[0]=centroid_m[1] = centroid_m[2] = centroid_d[0] = centroid_d[1] = centroid_d[2] = 0;

  double alignxf[16];
  for(int i = 0; i < nr_pts; i++){
    fileg >> x1 >> y1 >> z1; //gps
    filel >> x2 >> y2 >> z2; //3DTK
    Point pg(x1,y1,z1);
    Point pl(x2,y2,z2);

    PtPair p(pg,pl);

    Pairs.push_back(p);
    cout << p << endl << i << endl;
    centroid_m[0] += 1.0/nr_pts*pg.x;
    centroid_m[1] += 1.0/nr_pts*pg.y;
    centroid_m[2] += 1.0/nr_pts*pg.z;
    centroid_d[0] += 1.0/nr_pts*pl.x;
    centroid_d[1] += 1.0/nr_pts*pl.y;
    centroid_d[2] += 1.0/nr_pts*pl.z;
  }

  if(scale) {
    icp6D_QUAT_SCALE q;
    q.Align(Pairs, alignxf, centroid_m, centroid_d);
  } else {
    icp6Dminimizer *my_icp6Dminimizer = 0;
    my_icp6Dminimizer = new icp6D_SVD(false);   //2
    //my_icp6Dminimizer = new icp6D_QUAT(false);  //3
    //my_icp6Dminimizer = new icp6D_APX(false);   //4
    //my_icp6Dminimizer = new icp6D_ORTHO(false); //5
    //my_icp6Dminimizer = new icp6D_HELIX(false); //6
    //my_icp6Dminimizer = new icp6D_DUAL(false);   //7
    //my_icp6Dminimizer = new icp6D_LUMEULER(false);   //8
    //my_icp6Dminimizer = new icp6D_LUMQUAT(false);   //9
    //my_icp6Dminimizer = new icp6D_QUAT_SCALE(false);   //10
    my_icp6Dminimizer->Align(Pairs, alignxf, centroid_m, centroid_d);
  }

  fileg.close();
  filel.close();
  ofstream output(out.c_str());
  if(!output.is_open())
  {
    cout << out << " not found!" << endl;
  }

  for(int j = 0; j < 3; j++) {
    for(int i = 0; i < 16; i++){
      cout <<std::setprecision(15) << alignxf[i] << " ";
      output<<std::setprecision(15) << alignxf[i] << " ";
    }
    output << "2" << endl;
    cout << endl;
  }
  output.close();

}
