/*
 * matchMarker implementation
 *
 * Copyright (C) Dorit Borrmann
 *
 * Released under the GPL version 3.
 *
 */

#include <string>
using std::string;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <fstream>
using std::ofstream;
using std::ifstream;
#include <errno.h>
#include <time.h>

#include "slam6d/scan.h"

#include "slam6d/scan_io.h"
#include "slam6d/globals.icc"
#include "slam6d/icp6Dminimizer.h"
#include "slam6d/icp6Dsvd.h"
#include "slam6d/icp6D.h"

#ifndef _MSC_VER
#include <getopt.h>
#else
#include "XGetopt.h"
#endif

struct match {
  double* s_a;
  double* s_b;
  double* s_c;
  double* t_a;
  double* t_b;
  double* t_c;
  int n_o_m;
  double variance;
  double alignxf[16];
};

void refine(int source, int target, match &tmp) {
  // transform scan according to matching triple
  Scan::allScans[target]->transformToMatrix(tmp.alignxf,Scan::INVALID);
  // search for point pairs
  //TODO
  bool quiet = true;
  icp6Dminimizer *my_icp6Dminimizer = 0;
  my_icp6Dminimizer = new icp6D_SVD(quiet);

  icp6D *my_icp = 0;
  double mdm = 25;
  int mni = 100;
  my_icp = new icp6D(my_icp6Dminimizer, mdm, mni, quiet, false, -1, false, 1, 0.0000, false, false);

  my_icp->match(Scan::allScans[source], Scan::allScans[target]);
 
  ////cout << "Point pairs: "<< tmp.n_o_m << endl;
  const double * transMat = Scan::allScans[target]->get_transMat();
  for(int i = 0; i < 16; i++) {
    //cout << tmp.alignxf[i] << " " ;
    tmp.alignxf[i] = transMat[i];
  }
  //cout << endl;
}

void evaluate(int source, int target, match &tmp) {
  //cout << "S1 " << tmp.s_a[0] << " " << tmp.s_a[1] << " " << tmp.s_a[2] << endl;
  //cout << "T1 " << tmp.t_a[0] << " " << tmp.t_a[1] << " " << tmp.t_a[2] << endl;
  //cout << "S2 " << tmp.s_b[0] << " " << tmp.s_b[1] << " " << tmp.s_b[2] << endl;
  //cout << "T2 " << tmp.t_b[0] << " " << tmp.t_b[1] << " " << tmp.t_b[2] << endl;
  //cout << "S3 " << tmp.s_c[0] << " " << tmp.s_c[1] << " " << tmp.s_c[2] << endl;
  //cout << "T3 " << tmp.t_c[0] << " " << tmp.t_c[1] << " " << tmp.t_c[2] << endl;
  
  // Point pairs
  vector<PtPair> pairs_in;
  pairs_in.push_back(PtPair(tmp.s_a, tmp.t_a));
  pairs_in.push_back(PtPair(tmp.s_b, tmp.t_b));
  pairs_in.push_back(PtPair(tmp.s_c, tmp.t_c));
  
  // Calculate centroids
  double centroid_s[3];
  double centroid_t[3];
  centroid_s[0] = ((tmp.s_a[0] + tmp.s_b[0] + tmp.s_c[0])/3.0);
  centroid_s[1] = ((tmp.s_a[1] + tmp.s_b[1] + tmp.s_c[1])/3.0);
  centroid_s[2] = ((tmp.s_a[2] + tmp.s_b[2] + tmp.s_c[2])/3.0);
  centroid_t[0] = ((tmp.t_a[0] + tmp.t_b[0] + tmp.t_c[0])/3.0);
  centroid_t[1] = ((tmp.t_a[1] + tmp.t_b[1] + tmp.t_c[1])/3.0);
  centroid_t[2] = ((tmp.t_a[2] + tmp.t_b[2] + tmp.t_c[2])/3.0);
  
  // transform scan according to matching triple
  icp6Dminimizer *my_icp6Dminimizer = 0;
  my_icp6Dminimizer = new icp6D_SVD(true);
  my_icp6Dminimizer->Point_Point_Align(pairs_in, tmp.alignxf, centroid_s, centroid_t);
  Scan::allScans[target]->transformToMatrix(tmp.alignxf,Scan::INVALID);

  vector<PtPair> pairs_out;
  // search for point pairs
  Scan::getPtPairs(&pairs_out, Scan::allScans[source], Scan::allScans[target], 1, 0, 25.0, centroid_s, centroid_t);  
 
  tmp.n_o_m = pairs_out.size();
  //cout << tmp.n_o_m << endl;
  
  if(tmp.n_o_m > Scan::allScans[target]->get_points_red_size()) {
    //cout << "Point pairs: "<< tmp.n_o_m << endl;
    
    /*
    for(int i = 0; i < 16; i++) {
      cout << tmp.alignxf[i] << " ";
    }
    for(int i = 0; i < pairs_out.size(); i++) {
      cout << pairs_out[i].p1.x << " " << pairs_out[i].p1.y << " " << pairs_out[i].p1.z <<
      " " << pairs_out[i].p2.x << " " << pairs_out[i].p2.y << " " << pairs_out[i].p2.z << endl; 
    }
    */
  }
  
  //cout << "S1 " << tmp.s_a[0] << " " << tmp.s_a[1] << " " << tmp.s_a[2] << endl;
  //cout << "T1 " << tmp.t_a[0] << " " << tmp.t_a[1] << " " << tmp.t_a[2] << endl;
  //cout << "S2 " << tmp.s_b[0] << " " << tmp.s_b[1] << " " << tmp.s_b[2] << endl;
  //cout << "T2 " << tmp.t_b[0] << " " << tmp.t_b[1] << " " << tmp.t_b[2] << endl;
  //cout << "S3 " << tmp.s_c[0] << " " << tmp.s_c[1] << " " << tmp.s_c[2] << endl;
  //cout << "T3 " << tmp.t_c[0] << " " << tmp.t_c[1] << " " << tmp.t_c[2] << endl;
  //cout << endl;
  Scan::allScans[target]->resetPose();

  delete my_icp6Dminimizer;
}

void matchBruteForce(int start, int end) { // BOF
  double* const* source = Scan::allScans[start]->get_points_red();
  double* const* target = Scan::allScans[end]->get_points_red();
  int nr_m_s = Scan::allScans[start]->get_points_red_size();
  int nr_m_t = Scan::allScans[end]->get_points_red_size();
  double dist_s[nr_m_s][nr_m_s];
  double dist_t[nr_m_t][nr_m_t];
  double eps = 5.0;
  vector<match> matching_results;

  // calculate distance graphs
  for(int i = 0; i < nr_m_s; i++) {
    for(int j = i; j < nr_m_s; j++) {
      double dist = sqrt(Dist2(source[i], source[j]));
      dist_s[i][j] = dist;
      dist_s[j][i] = dist;
      //cout << dist << " "; 
    }
    //cout << endl;
  }
  //cout << endl;
  for(int i = 0; i < nr_m_t; i++) {
    for(int j = i; j < nr_m_t; j++) {
      double dist = sqrt(Dist2(target[i], target[j]));
      dist_t[i][j] = dist;
      dist_t[j][i] = dist;
      //cout << dist << " "; 
    }
    //cout << endl;
  }
  //cout << endl;
  //cout << "Done calculating distances" << endl;

  // do matching
  for(int i = 0; i < nr_m_s; i++) {               // 1
    for(int j = i + 1; j < nr_m_s; j++) {         // 2
      double dist_a = dist_s[i][j];
      for(int k = 0; k < nr_m_t; k++) {           // 3
        for(int l = 0; l < nr_m_t; l++) {         // 4
          if(fabs(dist_t[k][l] - dist_a) < eps) { // 5
            for(int m = j + 1; m < nr_m_s; m++) { // 6
              double dist_3_i = dist_s[i][m];
              double dist_3_j = dist_s[j][m];
              for(int n = 0; n < nr_m_t; n++) {   // 7
                if(fabs(dist_t[k][n] - dist_3_i) < eps) { // 8
                  if(fabs(dist_t[l][n] - dist_3_j) < eps) { // 9 
                    //match found calculate error function
                    match tmp;
                    tmp.s_a = source[i];
                    tmp.s_b = source[j];
                    tmp.s_c = source[m];
                    tmp.t_a = target[k];
                    tmp.t_b = target[l];
                    tmp.t_c = target[n];
                    evaluate(start, end, tmp);
                    matching_results.push_back(tmp);
                  }                                         // 9
                }                                         // 8
              }                                   // 7
            }                                     // 6
          }                                       // 5
        }                                         // 4
      }                                           // 3
    }                                             // 2
  }                                               // 1
  int max = 0;
  int max_i = 0;
  for(int i = 0; i < matching_results.size(); i++) {
    if(matching_results[i].n_o_m > max) {
      max_i = i;
      max = matching_results[i].n_o_m;
    }
    //cout << i << ": " << matching_results[i].n_o_m << ", " << matching_results[i].variance << endl;
    for(int j = 0; j < 16; j++) {
      //cout << matching_results[i].alignxf[j] << " ";
    }
  }
  //cout << "Max index: " << max_i << ", points: " << max << endl;
  for(int i = 0; i < 16; i++) {
    //cout << matching_results[max_i].alignxf[i] << " ";
  }
  refine(start, end, matching_results[max_i]);
  //cout << endl << "Refined: " << endl;
  for(int i = 0; i < 16; i++) {
    cout << matching_results[max_i].alignxf[i] << " ";
  }

  cout << " 2" << endl;
} // EOF

void matchMarker(int start, int end) { // BOF
  cout << start << " " << end << endl;
  double* const* source = Scan::allScans[start]->get_points_red();
  double* const* target = Scan::allScans[end]->get_points_red();
  int nr_m_s = Scan::allScans[start]->get_points_red_size();
  int nr_m_t = Scan::allScans[end]->get_points_red_size();
  cout << "Points: " << nr_m_s << " " << nr_m_t << endl;
  double dist_s[nr_m_s][nr_m_s];
  double dist_t[nr_m_t][nr_m_t];
  double eps = 5.0;
  vector<match> matching_results;

  // calculate distance graphs
  for(int i = 0; i < nr_m_s; i++) {
    for(int j = i; j < nr_m_s; j++) {
      double dist = sqrt(Dist2(source[i], source[j]));
      dist_s[i][j] = dist;
      dist_s[j][i] = dist;
      //cout << dist << " "; 
    }
    //cout << endl;
  }
  //cout << endl;
  for(int i = 0; i < nr_m_t; i++) {
    for(int j = i; j < nr_m_t; j++) {
      double dist = sqrt(Dist2(target[i], target[j]));
      dist_t[i][j] = dist;
      dist_t[j][i] = dist;
      //cout << dist << " "; 
    }
    //cout << endl;
  }
  //cout << endl;
  //cout << "Done calculating distances" << endl;
  for(int runs = 0; runs < 100; runs++) {
    int i = (int) (nr_m_s*(rand()/(RAND_MAX+1.0)));
    int j = (int) (nr_m_s*(rand()/(RAND_MAX+1.0)));
    int m = (int) (nr_m_s*(rand()/(RAND_MAX+1.0)));
    double dist_a = dist_s[i][j];
    double dist_3_i = dist_s[i][m];
    double dist_3_j = dist_s[j][m];
  
  
  // do matching
    for(int k = 0; k < nr_m_t; k++) {           // 3
      for(int l = 0; l < nr_m_t; l++) {         // 4
        if(fabs(dist_t[k][l] - dist_a) < eps) { // 5
       
          for(int n = 0; n < nr_m_t; n++) {   // 7
            if(fabs(dist_t[k][n] - dist_3_i) < eps) { // 8
              if(fabs(dist_t[l][n] - dist_3_j) < eps) { // 9 
                //match found calculate error function
                match tmp;
                tmp.s_a = source[i];
                tmp.s_b = source[j];
                tmp.s_c = source[m];
                tmp.t_a = target[k];
                tmp.t_b = target[l];
                tmp.t_c = target[n];
                evaluate(start, end, tmp);
                matching_results.push_back(tmp);
              }                                         // 9
            }                                         // 8
          }                                   // 7
        }                                     // 6
                                              // 5
      }                                         // 4
    }                                           // 3
  }                                               // 1
  int max = 0;
  int max_i = 0;
  for(int i = 0; i < matching_results.size(); i++) {
    if(matching_results[i].n_o_m > max) {
      max_i = i;
      max = matching_results[i].n_o_m;
      //cout << i << " " << max << endl;
    }
    //cout << i << ": " << matching_results[i].n_o_m << ", " << matching_results[i].variance << endl;
    for(int j = 0; j < 16; j++) {
      //cout << matching_results[i].alignxf[j] << " ";
    }
  }
  cout << "Really?" << endl;
  //cout << "Max index: " << max_i << ", points: " << max << endl;
  for(int i = 0; i < 16; i++) {
    //cout << matching_results[max_i].alignxf[i] << " ";
  }
  refine(start, end, matching_results[max_i]);
  //cout << endl << "Refined: " << endl;
  for(int i = 0; i < 16; i++) {
    cout << matching_results[max_i].alignxf[i] << " ";
  }

  cout << " 2" << endl;
} // EOF

/**
 * Explains the usage of this program's command line parameters
 */
void usage(char* prog)
{
#ifndef _MSC_VER
  const string bold("\033[1m");
  const string normal("\033[m");
#else
  const string bold("");
  const string normal("");
#endif
  cout << endl
	  << bold << "USAGE " << normal << endl
	  << "   " << prog << "-s <START> -e <END> [options] directory" << endl << endl;
  cout << bold << "OPTIONS" << normal << endl
	  << endl
	  << bold << "  -s" << normal << " NR, " << bold << "--start=" << normal << "NR" << endl
	  << "         start at scan NR (i.e., neglects the first NR scans)" << endl
	  << "         [ATTENTION: counting naturally starts with 0]" << endl
	  << endl
	  << bold << "  -e" << normal << " NR, " << bold << "--end=" << normal << "NR" << endl
	  << "         end after scan NR" << endl
	  << endl
	  << bold << "  -f" << normal << " F, " << bold << "--format=" << normal << "F" << endl
	  << "         using shared library F for input" << endl
	  << "         (chose F from {uos, uos_map, uos_rgb, uos_frames, uos_map_frames, old, rts, rts_map, ifp, riegl_txt, riegl_rgb, riegl_bin, rxp,zahn, ply})" << endl
	  << endl
	  << bold << "  -d" << normal << " NR, " << bold << "--dist=" << normal << "NR" << endl
	  << "         write all points that have no corresponding point closer than NR 'units'" << endl
    << endl << endl;
  
  cout << bold << "EXAMPLES " << normal << endl
	  << "   " << prog << " -m 500 -d 5 dat" << endl
	  << "   " << prog << " --max=5000 -d 10.2 dat" << endl
	  << endl;
  exit(1);
}

/** A function that parses the command-line arguments and sets the respective flags.
 * @param argc the number of arguments
 * @param argv the arguments
 * @param dir the directory
 * @param start first scan number 'start'
 * @param end last scan number 'end'
 * @param dist the maximal distance for a point pair
 * @param type the scan format
 * @param desc true if start is greater than end
 * @return 0, if the parsing was successful. 1 otherwise
 */
int parseArgs(int argc, char **argv, string &dir, 
		    int &start, int &end, double &dist, 
		    IOType &type, bool &desc)
{
  int  c;
  // from unistd.h:
  extern char *optarg;
  extern int optind;

  /* options descriptor */
  // 0: no arguments, 1: required argument, 2: optional argument
  static struct option longopts[] = {
    { "format",          required_argument,   0,  'f' },  
    { "start",           required_argument,   0,  's' },
    { "end",             required_argument,   0,  'e' },
    { "dist",            required_argument,   0,  'd' },
    { 0,           0,   0,   0}                    // needed, cf. getopt.h
  };

  //cout << endl;
  while ((c = getopt_long(argc, argv, "f:d:s:e:", longopts, NULL)) != -1)
    switch (c)
	 {
	 case 'd':
	   dist = atof(optarg);
	   break;
	 case 's':
	   start = atoi(optarg);
	   if (start < 0) { cerr << "Error: Cannot start at a negative scan number.\n"; exit(1); }
	   break;
	 case 'e':
	   end = atoi(optarg);
	   if (end < 0)     { cerr << "Error: Cannot end at a negative scan number.\n"; exit(1); }
	   break;
	 case 'f':
     try {
       type = formatname_to_io_type(optarg);
     } catch (...) { // runtime_error
       cerr << "Format " << optarg << " unknown." << endl;
       abort();
     }
     break;
   case '?':
	   usage(argv[0]);
	   return 1;
      default:
	   abort ();
      }

  if(start < 0 || end < 0 || start == end) {
    cerr << "\n*** You need two different scans for marker based position calculation ***" << endl;
    usage(argv[0]);
  }
  if (optind != argc-1) {
    cerr << "\n*** Directory missing ***" << endl;
    usage(argv[0]);
  }
  dir = argv[optind];

#ifndef _MSC_VER
  if (dir[dir.length()-1] != '/') dir = dir + "/";
#else
  if (dir[dir.length()-1] != '\\') dir = dir + "\\";
#endif
  if(start > end) {
    double tmp = start;
    start = end; 
    end = tmp;
    desc = true;
  } 

  return 0;
}


/**
 * Main program for calculating the difference of two scans.
 * Usage: bin/scan_diff -d <NR> -s <NR> -e <NR> 'dir',
 * Use -s and -e for the two scans, 
 * -d 
 * and 'dir' the directory of a set of scans
 * Difference scans will be written to 'dir/diff/scan[00]s.3d'
 * 
 */
int main(int argc, char **argv)
{

  cout << "(c) Jacobs University Bremen, gGmbH, 2010" << endl << endl;
  
  if (argc <= 1) {
    usage(argv[0]);
  }

  // parsing the command line parameters
  // init, default values if not specified
  string dir;
  double    dist   = 0;
  int    start = -1,   end = -1;
  int    maxDist    = -1;
  int    minDist    = -1;
  IOType type    = UOS;
  bool desc = false;  

  parseArgs(argc, argv, dir, start, end, dist, type, desc);
  
  Scan::readScansRedSearch(type, start, end, dir, maxDist, minDist, -1, 1, false, false, false);
  cout << "Start match marker ..." << endl; 
  srand(time(0));
  srand(0);
  long starttime = GetCurrentTimeInMilliSec(); 
 // matchBruteForce(start, end-start);
  matchMarker(0, end - start);
  starttime = (GetCurrentTimeInMilliSec() - starttime);
  cout << " done in " << starttime << " ms" << endl; 
  
  vector <Scan*>::iterator Iter = Scan::allScans.begin();
  for( ; Iter != Scan::allScans.end(); ) {
    Iter = Scan::allScans.begin();
    delete (*Iter);
    cout << ".";
    cout.flush(); 
  }
  Scan::allScans.clear();

  cout << endl << endl;
  cout << "Normal program end." << endl << endl;
}
