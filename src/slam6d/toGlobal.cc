/*
 * toGlobal implementation
 *
 * Copyright (C) Jan Elseberg, Dorit Borrmann
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

#include "slam6d/point.h"
#include "slam6d/globals.icc"
#include <string.h>
#include <exception>
using std::exception;

#ifndef _MSC_VER
#include <unistd.h>
#endif

#ifdef _MSC_VER
#include "XGetopt.h"
#else
#include <getopt.h>
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
  while ((c = getopt (argc, argv, "s:e:h")) != -1)
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
   case 'h':
     cout << "Usage: " << argv[0] << " [-s START] [-e END] DIRECTORY" << endl;
     cout << endl;
     cout << "Converts scans in UOSR format from START to END in DIRECTORY to" << endl;
     cout << "one single scan in XYZR format in the same directory under a" << endl;
     cout << "filename of the format scan{START}.xyz" << endl;
     exit(0);
   }

  if (optind != argc-1) {
    cerr << "\n*** Directory missing ***" << endl;
    exit(0);
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
  char outFileName[255];

  ifstream frame_in;
  ifstream scan_in;

  snprintf(outFileName,255,"%sscan%.3d.xyz",dir,fileCounter);
  cout << outFileName << endl;
  ofstream redptsout(outFileName);
  stringstream outdat;
  int pointcnt=0;
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
//    double range, theta, phi, reflectance;
    unsigned int rgb[3];
    cout.precision(10);
    while(scan_in.good()) {
  /*    scan_in >> p.z >> p.x >> p.y >> range >> theta >> phi >> reflectance;
   */ //scan_in >> p.z >> p.x >> p.y >> range >> theta >> phi >> reflectance;
      scan_in >> p.x >> p.y >> p.z >> rgb[0] >> rgb[1] >> rgb[2];
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
      outdat << std::setprecision(15) << p.z*0.01 << " " << -p.x*0.01 << " " << p.y*0.01 << " " << rgb[0] << " " << rgb[1] << " " << rgb[2] << endl;
      pointcnt++;
      if(pointcnt > 100) {
        redptsout.write(outdat.str().c_str(), outdat.str().size());
        pointcnt = 0;
        outdat.clear();
        outdat.str("");
      }

    } 
    redptsout.write(outdat.str().c_str(), outdat.str().size());
    pointcnt = 0;
    outdat.clear();
    outdat.str("");


    scan_in.close();
    scan_in.clear();
    frame_in.close();
    frame_in.clear();
    cerr << " done." << endl;
  }
  redptsout.close();
  redptsout.clear();

}

