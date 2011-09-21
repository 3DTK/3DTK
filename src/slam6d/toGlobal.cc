#include <fstream>
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
using std::ifstream;
using std::ofstream;

#include "slam6d/point.h"
#include "slam6d/globals.icc"
#include <string.h>
#include <exception>
using std::exception;

#ifndef _MSC_VER
#include <unistd.h>
#else
#include "..\Visual_Studio_Projects\6D_SLAM\XGetopt.h"
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
    cerr << "\n*** Directory missing ***" << endl;
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
  char frameFileName[255];
  char scanFileName[255];

  ifstream frame_in;
  ifstream scan_in;

  for (;;) {
    if (end > -1 && fileCounter > end) break; // 'nuf read
    snprintf(frameFileName,255,"%sscan%.3d.frames",dir,fileCounter);
    snprintf(scanFileName,255,"%sscan%.3d.3d",dir,fileCounter++);

    scan_in.open(scanFileName);
    frame_in.open(frameFileName);

    double tmp;
    // read 3D scan
    if (!frame_in.good()) break; // no more files in the directory

    cerr << "Reading frame " << frameFileName << "..." << endl;
    int frameCounter = 0;
    double transMat[16];
    //double transMatOpenGL[16];
    double *colourMat = new double[4];

    while (frame_in.good()) {
     frameCounter++;
     try {
       frame_in >> transMat;
       frame_in >> tmp; 
     
     } catch (const exception &e) {
       break;
     }
    }

    Point p;
    double range, theta, phi, reflectance;
    cout.precision(10);
    while(scan_in.good()) {
  /*    scan_in >> p.z >> p.x >> p.y >> range >> theta >> phi >> reflectance;
   */ //scan_in >> p.z >> p.x >> p.y >> range >> theta >> phi >> reflectance;
      scan_in >> p.x >> p.y >> p.z; // >> p.reflectance;
     /* p.x *= -100;
      p.y *= 100;
      p.z *= 100;
    */
      p.transform(transMat);
      //Matrix4ToEuler(transMat,rPosTheta,rPos);

      //cout << p.y << " " << p.z << " " << -p.x << " " << reflectance << endl;
      //cout << p.y << " " << p.z << " " << -p.x << endl;
      //cout << p.x*0.01 << " " << p.z*0.01 << " " << p.y*0.01 << " " << p.reflectance << endl;
      //cout << (int)p.x << " " << (int)p.y << " " << (int)p.z << endl;// << " " << p.reflectance << endl;
      cout << p.x << " " << p.y << " " << p.z << endl;// << " " << p.reflectance << endl;

    } 

    scan_in.close();
    scan_in.clear();
    frame_in.close();
    frame_in.clear();
    cerr << " done." << endl;
  }

}

