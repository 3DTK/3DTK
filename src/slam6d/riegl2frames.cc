/*
 * riegl2frames implementation
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

//#include "scan.h"
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
    cout << "Reads dat files (RIEGL pose files) from directory/scan???.dat and converts them to directory/scan???.frames and directory/scan???.pose in the slam6D standard file format." << endl;
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
  int fileCounter = start; 
 
  char rieglFileName[255];
  char poseFileName[255];
  char frameFileName[255];

  ifstream riegl_in;
  ofstream pose_out;
  ofstream frames_out;
  
  for(;;) { 
    if (end > -1 && fileCounter > end) break; // 'nuf read
  
    snprintf(rieglFileName,255,"%sscan%.3d.dat",dir,fileCounter);
    snprintf(frameFileName,255,"%sscan%.3d.frames",dir,fileCounter);
    snprintf(poseFileName,255,"%sscan%.3d.pose",dir,fileCounter++);
    
    riegl_in.open(rieglFileName);
    // read pose

    if (!riegl_in.good()) return -1; // no more files in the directory
  
    cout << "Processing Scan " << rieglFileName;
    cout.flush();
  
    double rPos[3], rPosTheta[16];
    double inMatrix[16], tMatrix[16];
    for (unsigned int i = 0; i < 16; riegl_in >> inMatrix[i++]);
 
    riegl_in.close();
 
    // transform input pose
    tMatrix[0] = inMatrix[5];
    tMatrix[1] = -inMatrix[9];
    tMatrix[2] = -inMatrix[1];
    tMatrix[3] = -inMatrix[13];
    tMatrix[4] = -inMatrix[6];
    tMatrix[5] = inMatrix[10];
    tMatrix[6] = inMatrix[2];
    tMatrix[7] = inMatrix[14];
    tMatrix[8] = -inMatrix[4];
    tMatrix[9] = inMatrix[8];
    tMatrix[10] = inMatrix[0];
    tMatrix[11] = inMatrix[12];
    tMatrix[12] = -100*inMatrix[7];
    tMatrix[13] = 100*inMatrix[11];
    tMatrix[14] = 100*inMatrix[3];
    tMatrix[15] = inMatrix[15];
  
    Matrix4ToEuler(tMatrix, rPosTheta, rPos);
    
    double euler[6];
     
    euler[0] = rPos[0];
    euler[1] = rPos[1];
    euler[2] = rPos[2];
    euler[3] = rPosTheta[0];
    euler[4] = rPosTheta[1];
    euler[5] = rPosTheta[2];

    pose_out.open(poseFileName);
    frames_out.open(frameFileName);

    cout << "Writing pose... " << poseFileName << endl;

    for(int i = 0; i < 3; i++) {
      pose_out << euler[i] << " ";
    }
    pose_out << endl; 

    for(int i = 3; i < 6; i++) {
      pose_out << deg(euler[i]) << " ";
    }
    pose_out << endl; 

    cout << "Writing frames... " << poseFileName << endl;
  
    for(int j = 0; j < 2; j++) {
      for (int i=0; i < 16; i++) {
        frames_out << tMatrix[i] << " ";
      }
      frames_out << "2" << endl;
    }
    
    pose_out.close();
    pose_out.clear();

    frames_out.close();
    frames_out.clear();

    
    cout << " done." << endl;
  }

}

