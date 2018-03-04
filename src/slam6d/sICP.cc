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

#ifndef _MSC_VER
#include <getopt.h>
#else
#include "XGetopt.h"
#endif

#include <csignal>

#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#else
#include <strings.h>
#endif

int parseArgs(int argc,char **argv, char global[255], char local[255], char out[255], bool& scale, int& count){
  int  c;
  // from unistd.h
  extern char *optarg;
  extern int optind;
  
  static struct option longopts[] = {
    { "global",          required_argument,   0,  'g' },  
    { "local",           required_argument,   0,  'l' },
    { "out",             required_argument,   0,  'o' },
    { "scale",           no_argument,         0,  'S' },
    { "nrpoints",        required_argument,   0,  'n' },
    { 0,           0,   0,   0}                    // needed, cf. getopt.h
  };

  cout << endl;
  while ((c = getopt (argc, argv, "gloSn:")) != -1)
    switch (c) {
      case 'n':
        count = atoi(optarg);
        if (count < 1) { cerr << "Error: Cannot have a negative number of correspondences.\n"; exit(1); }
        break;
      case 'S':
        scale = true;
        break;
      case 'o':
        strncpy(out,argv[optind],255);
        break;
      case 'g':
        strncpy(global,argv[optind],255);
        break;
      case 'l':
        strncpy(local,argv[optind],255);
        break;
    }
  return 0;
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
  char global[255];
  char local[255];
  char out[255];
  bool scale = false;
  int nr_pts = 0;

  parseArgs(argc, argv, global, local, out, scale, nr_pts);
  
  if(nr_pts < 3) {
    cerr << "Cannot work with less than 3 correspondences!";
    usage(argv[0]);
    abort();
  }

  ifstream fileg(global);
  ifstream filel(local);
  
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
  ofstream output(out);
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
