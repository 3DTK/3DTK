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
using std::string;
using std::ifstream;
using std::ofstream;

#include "slam6d/scan.h"
#include "slam6d/globals.icc"
#include <string.h>

#ifndef _MSC_VER
#include <unistd.h>
#endif

#if WIN32
#define snprintf sprintf_s
#endif

#include <boost/program_options.hpp>
namespace po = boost::program_options;

int parse_options(int argc,char **argv, std::string &dir, int& start, int& end){

  po::options_description generic("Generic options");
  generic.add_options()
    ("help,h", "output this help message");

  po::options_description input("Input options");
  input.add_options()
    ("start,s", po::value<int>(&start)->default_value(0),
     "start at scan <arg> (i.e., neglects the first <arg> scans) "
     "[ATTENTION: counting naturally starts with 0]")
    ("end,e", po::value<int>(&end)->default_value(-1),
     "end after scan <arg>");

  po::options_description hidden("Hidden options");
  hidden.add_options()
    ("input-dir", po::value<std::string>(&dir), "input dir");

  // all options
  po::options_description all;
  all.add(generic).add(input).add(hidden);

  // options visible with --help
  po::options_description cmdline_options;
  cmdline_options.add(generic).add(input);

  // positional argument
  po::positional_options_description pd;
  pd.add("input-dir", 1);

  // process options
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).
            options(all).positional(pd).run(), vm);

  // display help
  if (vm.count("help")) {
    std::cout << cmdline_options;
    std::cout << std::endl
         << "Example usage:" << std::endl
         << "\t./bin/riegl2frames -s 0 -e 1 /Your/directory" << std::endl;
    exit(0);
  }
  po::notify(vm);

#ifndef _MSC_VER
  if (dir[dir.length()-1] != '/') dir = dir + "/";
#else
  if (dir[dir.length()-1] != '\\') dir = dir + "\\";
#endif
  return 0;
}


int main(int argc, char **argv)
{
  int start = 0, end = -1;
  string dir;
  parse_options(argc, argv, dir, start, end);

  int fileCounter = start;
  char rieglFileName[255];
  char poseFileName[255];
  char frameFileName[255];

  ifstream riegl_in;
  ofstream pose_out;
  ofstream frames_out;

  for(;;) {
    if (end > -1 && fileCounter > end) break; // 'nuf read

    snprintf(rieglFileName,255,"%sscan%.3d.dat",dir.c_str(),fileCounter);
    snprintf(frameFileName,255,"%sscan%.3d.frames",dir.c_str(),fileCounter);
    snprintf(poseFileName,255,"%sscan%.3d.pose",dir.c_str(),fileCounter++);

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

