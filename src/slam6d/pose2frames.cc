/*
 * pose2frames implementation
 *
 * Copyright (C) Dorit Borrmann, Jan Elseberg
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

#include "slam6d/scan.h"
#include "slam6d/globals.icc"
#include <string.h>

#ifndef _MSC_VER
#include <getopt.h>
#include <unistd.h>
#else
#include "XGetopt.h"
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
    cout << "Reads pose files from directory/scan???.pose and converts them to directory/scan???.frames to be used by show." << endl;
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
  

  for (;;) {
    if (end > -1 && fileCounter > end) break; // 'nuf read
#ifndef _MSC_VER
    snprintf(frameFileName,255,"%sscan%.3d.frames",dir,fileCounter);
    snprintf(poseFileName,255,"%sscan%.3d.pose",dir,fileCounter++);
# else
    sprintf(frameFileName,"%sscan%.3d.frames",dir,fileCounter);
    sprintf(poseFileName,"%sscan%.3d.pose",dir,fileCounter++);
#endif

    pose_in.open(poseFileName);

    // read 3D scan
    if (!pose_in.good()) break; // no more files in the directory

    cout << "Reading pose " << poseFileName << "..." << endl;
    
    for (unsigned int i = 0; i < 3; pose_in >> rPos[i++]);
    for (unsigned int i = 0; i < 3; pose_in >> rPosTheta[i++]);
    
    // convert angles from deg to rad
    for (unsigned int i=0; i < 3; i++) rPosTheta[i] = rad(rPosTheta[i]);

    cerr << rPos[0] << " " << rPos[1] << " " << rPos[2] << endl;

    pose_in.close();
    pose_in.clear();

    cout << rPos[0] << " "<< rPos[1] << " "<< rPos[2] << endl;
    cout << rPosTheta[0] << " "<< rPosTheta[1] << " "<< rPosTheta[2] << endl;
     
    double tempR[16];
    EulerToMatrix4(rPos, rPosTheta, tempR); // convert previous rPosTheta

    pose_out.open(frameFileName);

    cout << "Writing frame... " << frameFileName << endl;

    for (int j=0;j<10;j++) {
      for (int i=0; i < 16; i++) {
        pose_out << tempR[i] << " ";
      }
      pose_out << " 3" << endl;
    }

    pose_out.close();
    pose_out.clear();

    
    cout << " done." << endl;
  }

}

