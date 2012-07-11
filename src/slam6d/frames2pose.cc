/*
 * frames2pose implementation
 *
 * Copyright (C) Dorit Borrmann
 *
 * Released under the GPL version 3.
 *
 */


#include <fstream>
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
using std::ifstream;
using std::ofstream;

#include "slam6d/globals.icc"
#include <string.h>

#ifndef _MSC_VER
#include <unistd.h>
#else
#include "XGetopt.h"
#endif

#if WIN32
#define snprintf sprintf_s
#endif 

int parseArgs(int argc,char **argv, char dir[255], int& start, int& end){
  start   = 0;
  end     = -1; // -1 indicates no limitation

  int  c;
  // from unistd.h
  extern char *optarg;
  extern int optind;

  cout << endl;
  while ((c = getopt (argc, argv, "s:e:")) != -1)
    switch (c)
   {
   case 's':
     start = atoi(optarg);
     if (start < 0) { cerr << "Error: Cannot start at a negative scan number.\n"; exit(1); }
     break;
   case 'e':
     end = atoi(optarg);
     if (end < 0)     { cerr << "Error: Cannot end at a negative scan number.\n"; exit(1); }
     if (end < start) { cerr << "Error: <end> cannot be smaller than <start>.\n"; exit(1); }
     break;
   }

  if (optind != argc-1) {
    cerr << "\n*** Directory missing ***\n" << endl; 
    cout << endl
	  << "Usage: " << argv[0] << "  [-s NR] [-e NR] directory" << endl << endl;

    cout << "  -s NR   start at scan NR (i.e., neglects the first NR scans)" << endl
       << "          [ATTENTION: counting starts with 0]" << endl
	  << "  -e NR   end after scan NR" << "" << endl
	  << endl;
    cout << "Reads frame files from directory/scan???.frames and converts them to directory/scan???.pose." << endl;
    abort();
  }
  strncpy(dir,argv[optind],255);

#ifndef _MSC_VER
  if (dir[strlen(dir)-1] != '/') strcat(dir,"/");
#else
  if (dir[strlen(dir)-1] != '\\') strcat(dir,"\\");
#endif
  return 0;
}


int main(int argc, char **argv)
{
  int start = 0, end = -1;
  char dir[255];
  parseArgs(argc, argv, dir, start, end);

  int  fileCounter = start;
  char poseFileName[255];
  char frameFileName[255];

  ifstream pose_in;
  ofstream pose_out;

  double rPos[3],rPosTheta[3];
  
  double tMatrix[17];

  for (;;) {
    if (end > -1 && fileCounter > end) break; // 'nuf read
    snprintf(frameFileName,255,"%sscan%.3d.frames",dir,fileCounter);
    snprintf(poseFileName,255,"%sscan%.3d.pose",dir,fileCounter++);

    pose_in.open(frameFileName);

    if (!pose_in.good()) break; // no more files in the directory
    // read 3D scan

    cout << "Reading frame " << frameFileName << "..." << endl;
    
    while(pose_in.good()) {
      for (unsigned int i = 0; i < 17; pose_in >> tMatrix[i++]);
    }
    
    Matrix4ToEuler(tMatrix, rPosTheta, rPos);
    
    
    pose_in.close();
    pose_in.clear();

    pose_out.open(poseFileName);

    cout << "Writing pose file... " << poseFileName << endl;
    
    for(int i = 0; i < 3; i++) {
      pose_out << rPos[i] << " ";
    }
    pose_out << endl; 

    for(int i = 0; i < 3; i++) {
      pose_out << deg(rPosTheta[i]) << " ";
    }
    pose_out << endl; 

    pose_out.close();
    pose_out.clear();

    
    cout << " done." << endl;
  }

}

