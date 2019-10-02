/*
 * lla2pose implementation (Not fully tested, yet).
 *
 * Copyright (C) Dorit Borrmann
 *
 * Released under the GPL version 3.
 *
 */


#include <fstream>
#include <iostream>
#include <vector>
using std::cout;
using std::cerr;
using std::endl;
using std::ifstream;
using std::ofstream;
using std::vector;

#include "gps/gps.h"
#include "slam6d/globals.icc"
#include "gps/gps.h"

#include <string.h>

#ifndef _MSC_VER
#include <unistd.h>
#else
#include "XGetopt.h"
#endif

#if WIN32
#define snprintf sprintf_s
#endif


int parseArgs(int argc,char **argv, char dir[255], char filename[255], int& start, int& end, bool& singleFile, bool& nmea, bool& kml){
  start   = 0;
  end     = -1; // -1 indicates no limitation

  int  c;
  // from unistd.h
  extern char *optarg;
  extern int optind;

  while ((c = getopt (argc, argv, "s:e:SNf:k")) != -1)
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
    case 'f':
      strncpy(filename,optarg,255);
      break;
    case 'S':
      singleFile = true;
      break;
    case 'k':
      kml = true;
      break;
    case 'N':
      nmea = true;
      break;
    }

  if (optind != argc-1) {
    cerr << "\n*** Directory missing ***\n" << endl;
    cout << endl
	  << "Usage: " << argv[0] << "  [-s NR] [-e NR] directory" << endl << endl;

    cout << "  -s NR   start at scan NR (i.e., neglects the first NR scans)" << endl
      << "          [ATTENTION: counting starts with 0]" << endl
	    << "  -e NR   end after scan NR" << "" << endl
	    << "  -S      read all poses from a single file" << "" << endl
	    << "  -N      read data from a nmea file" << "" << endl
	    << endl;
    cout << "Reads latitude, longitude, altitude from gps???.txt and converts them to them to directory/scan???.pose." << endl;
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
  char filename[255];
  bool singleFile = false;
  bool nmea = false;
  bool kml = false;
  parseArgs(argc, argv, dir, filename, start, end, singleFile, nmea, kml);

  double fx = 0;
  double fy = 0;
  double fz = 0;
  double cx = 0;
  double cy = 0;
  double cz = 0;

  double * mat;

  int  fileCounter = start;
  char poseFileName[255];
  char gpsFileName[255];

  ifstream gps_in;
  ofstream pose_out;

  double rPos[3],rPosTheta[3];
  double ecefmat[9];

  rPosTheta[0] = rPosTheta[1] = rPosTheta[2] = 0;

  vector<double *> positions;
  vector<double *> orientations;

  std::vector<double *> lla_vec;
  if(singleFile) {
    cerr << "Reading gps " << filename << "..." << endl;
    readRTKPos(filename, lla_vec);
  }

  if(kml) {
    cout << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n <kml xmlns=\"http://www.opengis.net/kml/2.2\">\n <Document>\n <Folder>\n";
  }

  for(int fileCounter = 0; fileCounter < lla_vec.size(); fileCounter++) {
    if(!singleFile) {
      snprintf(gpsFileName,255,"%sgps%.3d.txt",dir,0);
      gps_in.open(gpsFileName);
      if (!gps_in.good()) {
        cout << "No more files in directory " << dir << endl;
      }
    }
    if(kml) {
      cout << "<Placemark>\n<name>S1P" << fileCounter+1 <<
      //"</name>\n<Point>\n<altitudeMode>absolute</altitudeMode>\n<coordinates>";
      "</name>\n<Point>\n<altitudeMode>clampToGround</altitudeMode>\n<coordinates>";
      cout << std::setprecision(15) << lla_vec[fileCounter][1] << "," << lla_vec[fileCounter][0];
      cout << "</coordinates>\n</Point>\n</Placemark>" << endl;
    }

    snprintf(poseFileName,255,"%sscan%.3d.pose",dir,fileCounter);
    double east, north, up;
    double x,y,z;
    LLAtoECEF_r(lla_vec[fileCounter][0], lla_vec[fileCounter][1], lla_vec[fileCounter][2],cx,cy,cz);
    //LLAtoECEF(lla_vec[fileCounter][0], lla_vec[fileCounter][1], lla_vec[fileCounter][2],cx,cy,cz);
    //cout << std::setprecision(15) << cx << " " << cy << " " << cz << endl;
    //ECEF_rtoENU(lla_vec[fileCounter][0], lla_vec[fileCounter][1], lla_vec[fileCounter][2],cx,cy,cz,east,north,up);
    if(fileCounter == 0) {
      fx = cx;
      fy = cy;
      fz = cz;
      calcECEF_rtoENUMat9(lla_vec[0][0], lla_vec[0][1], lla_vec[0][2],ecefmat);
    }

    getENU(ecefmat, cx-fx,cy-fy,cz-fz, east, north, up);
    //ECEF_rtoENU(lla_vec[0][0], lla_vec[0][1], lla_vec[0][2],cx-fx,cy-fy,cz-fz,east,north,up);
    ENUto3DTK(east, north, up, x, y, z);

    pose_out.open(poseFileName);

    //cout << "Writing pose file... " << poseFileName << endl;
    //pose_out << std::setprecision(15) << (x - fx) << " " << (y - fy) << " " << (z - fz) << endl;
    pose_out << std::setprecision(15) << (x) << " " << (y) << " " << (z) << endl;
    pose_out << "0 0 0" << endl;

    pose_out.close();
    pose_out.clear();
    if(!singleFile) {
      gps_in.close();
    }

  }
  if(kml) {
    cout << "</Folder>\n</Document>\n</kml>" << endl;
  }
}

