/*
 * match_with_ground_truth implementation
 *
 * Copyright (C) Andreas Nuechter, Kai Lingemann, Jochen Sprickerhof
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file
 * @brief Main programm for matching registered 3D scans against a 
 * ground_truth_path
 *
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include <string>
using std::string;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <fstream>
using std::ifstream;
using std::ofstream;
#include <vector>
using std::vector;
#include <stdexcept>
using std::exception;

#include "globals.icc"
#include "ptpair.h"
#include "icp6Dapx.h"
#include "icp6Dsvd.h"
#include "icp6Dquat.h"

#ifndef _MSC_VER
#include <getopt.h>
#else
#include "..\Visual_Studio_Projects\6D_SLAM\XGetopt.h"
#endif

/**
 * Explains the usage of this program's command line parameters
 */
void usage(char* prog)
{
  const string bold("\033[1m");
  const string normal("\033[m");

  cout << "to do" << endl;
  
  exit(1);
}

int parseArgs(int argc, char **argv, string &dir, int &mni,
		    int &start, int &end, int &algo, bool &quiet, bool &veryQuiet)
{

  int  c;
  // from unistd.h:
  extern char *optarg;
  extern int optind;

  /* options descriptor */
  // 0: no arguments, 1: required argument, 2: optional argument
  static struct option longopts[] = {
    { "algo",      required_argument,   0,  'a' },
    { "iter",      required_argument,   0,  'i' },
    { "start",     required_argument,   0,  's' },
    { "end",       required_argument,   0,  'e' },
    { 0,           0,   0,   0}                    // needed, cf. getopt.h
  };

  cout << endl;
  while ((c = getopt_long(argc, argv, "a:i:s:e:qQ", longopts, NULL)) != -1) {
    switch (c)
	 {
	 case 'a':
	   algo = atoi(optarg);
	   if ((algo < 0) || (algo > 2)) { cerr << "Error: Algorithm not available.\n"; exit(1); }
	   break;
	 case 's':
	   start = atoi(optarg);
	   if (start < 0) { cerr << "Error: Cannot start at a negative scan number.\n"; exit(1); }
	   break;
	 case 'e':
	   end = atoi(optarg);
	   if (end < 0)     { cerr << "Error: Cannot end at a negative scan number.\n"; exit(1); }
	   if (end < start) { cerr << "Error: <end> cannot be smaller than <start>.\n"; exit(1); }
	   break;
	 case 'i':
	   mni = atoi(optarg);
	   break;
	 case 'q':
	   quiet = true;
	   break;
	 case 'Q':
	   quiet = veryQuiet = true;
	   break;
	 default:
	   abort();
	 }
  }
  if (optind != argc-1) {
    cerr << "\n*** Directory missing ***" << endl;
    usage(argv[0]);
  }
  dir = argv[optind];
 
}

int main(int argc, char **argv)
{
  if (argc <= 1) {
    usage(argv[0]);
  }

  string dir;
  int    mni = 50;
  int    start = 0,   end = -1;
  bool   quiet     = false;
  bool   veryQuiet = false;
  int    algo      = 0;

  parseArgs(argc, argv, dir, mni, start, end, algo, quiet, veryQuiet);

  string groundTruthFileName = dir + "position_sync_interpol.dat";
  ifstream gtruth_in(groundTruthFileName.c_str());
  if (!gtruth_in.good()) {
    cerr << "Cannot open " << groundTruthFileName << endl;
    exit(-1);
  }

  vector<PtPair> pairs;
  
  for (int i = 0; i < end; i++) {
    // read position from ground truth file
    double gtruthPos[3];
    double dummy;
    gtruth_in >> dummy >> gtruthPos[2] >> gtruthPos[0] >> gtruthPos[1] 
		    >> dummy >> dummy >> dummy;

    if (i < start) continue;

    for (unsigned int j = 0; j < 3; j++) gtruthPos[j] *= 0.1;
    
    string frameFileName = dir + "scan" + to_string(i,3) + ".frames";

    cout << "Processing File " << frameFileName << endl;
    
    ifstream frame_in(frameFileName.c_str());
    if (!frame_in.good()) {
	 cerr << "Cannot open " << frameFileName << endl;
	 exit(-1);
    }
    double transMat[16];
    int type;
    while (frame_in.good()) {
	 try {
	   frame_in >> transMat >> type;
	 } catch (const exception &e) {   
	   break;
	 }
    }
    // now transMat contains the last matrix
    double rPos[3], rPosTheta[3];
    Matrix4ToEuler(transMat, rPosTheta, rPos);

    PtPair myPair(gtruthPos, rPos);
    pairs.push_back(myPair);
  }
  
  cout << "Nr poses used for registration: " << pairs.size() << endl
	  << algo << endl;

  icp6Dminimizer *my_icp6Dminimizer = 0;
  switch (algo) {
  case 0 :
    my_icp6Dminimizer = new icp6D_APX(quiet);
    break;
  case 1 :
    my_icp6Dminimizer = new icp6D_QUAT(quiet);
    break;
  case 2 :
    my_icp6Dminimizer = new icp6D_SVD(quiet);
    break;
  }

  double totalalignxf[16];
  M4identity(totalalignxf);
  for (int iteration = 0; iteration < mni; iteration++) {
    double alignxf[16];
    my_icp6Dminimizer->Point_Point_Align(pairs, alignxf);
    for (int i = 0; i < pairs.size(); i++) {
	 pairs[i].p2.transform(alignxf);
    }
    double tempxf[16];
    MMult(alignxf, totalalignxf, tempxf);
    memcpy(totalalignxf, tempxf, sizeof(totalalignxf));
  }

  string initialTransformFileName = dir + "initital.frame";
  ofstream initial_out(initialTransformFileName.c_str());
  initial_out << totalalignxf << endl;
  initial_out.close();

}
