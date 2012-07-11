/*
 * frames_to_graph implementation
 *
 * Copyright (C) Jochen Sprickerhof
 *
 * Released under the GPL version 3.
 *
 */


/* @file extracts the final poses from the frame files
 * @author Jochen Sprickerhof
 */

#ifndef _MSC_VER
#include <getopt.h>
#else
#include "XGetopt.h"
#endif

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#include <string>
using std::string;

#include <fstream>
using std::ifstream;
using std::ofstream;

#include <stdexcept>
using std::exception;

#include "slam6d/globals.icc"

void read_Frame(string frameFile, ofstream &out_stream)
{
  double transMat[16];
  int type;
  double rPos[3];
  double rPosTheta[3];
  ifstream frame(frameFile.c_str());

  while(frame) {
    try {
      frame >> transMat >> type;
    }
    catch(const exception &e) {
      break;
    }
  }
  Matrix4ToEuler(transMat, rPosTheta, rPos);
  //out_stream << rPos[0] << " " << rPos[1] << " " << rPos[2] << " " << rPosTheta[0] << " " << rPosTheta[1] << " " << rPosTheta[2] << endl;
  double quat[4];
  Matrix4ToQuat(transMat, quat);
  QuatToAA(quat);
  out_stream << rPos[0] << " " << rPos[1] << " " << rPos[2] << " " << quat[0] << " " << quat[1] << " " << quat[2] << " " << quat[3] << endl;
  frame.close();
}

void readFrames(string dir, int start, int end, ofstream &out_stream)
{
  for(;start <= end; start++) {
    read_Frame(dir + "scan" + to_string(start, 3) + ".frames", out_stream);
  }
}

void usage(string progname) {
  cout << "Usage: " << progname << " OPTIONS FILE" << endl <<
    "\t -s \t\t first node in loop" << endl <<
    "\t -e \t\t last node in loop" << endl <<
    "\t -o \t\t output filename (must be given)" << endl;
}

int main(int argc, char *argv[])
{

  if(argc < 2) {
    usage(argv[0]);
    exit(1);
  }

  int start = 0, end = 0;
  string out_file;
  /* options descriptor */
  // 0: no arguments, 1: required argument, 2: optional argument
  static struct option longopts[] = {
    { "end",       required_argument,   0,  'e' },
    { "out",       required_argument,   0,  'o' },
    { "start",     required_argument,   0,  's' },
    { 0,           0,   0,   0}                    // needed, cf. getopt.h
  };

  int c;
  while((c = getopt_long(argc, argv, "e:o:s:", longopts, NULL)) != -1) {
    switch (c) {
      case 'e':
        end = atoi(optarg);
        if(end < 0) {
          cerr << "Error: Cannot end at a negative number." << endl;
          exit(1);
        }
        break;
      case 'o':
        out_file = optarg;
        break;
      case 's':
        start = atoi(optarg);
        if(start < 0) {
          cerr << "Error: Cannot start at a negative number." << endl;
          exit(1);
        }
        break;
      case '?':
      default:
        exit(1);
    }
  }

  if(out_file.empty() || end == 0) {
    usage(argv[0]);
    exit(1);
  }

  string dir = argv[optind];
  
#ifndef _MSC_VER
  if (dir[dir.length()-1] != '/') dir = dir + "/";
#else
  if (dir[dir.length()-1] != '\\') dir = dir + "\\";
#endif
  ofstream out_stream(out_file.c_str());

  readFrames(dir, start, end, out_stream);
  out_stream.close();

  exit(0);
}

