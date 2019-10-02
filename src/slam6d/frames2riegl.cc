/*
 * frames2riegl implementation
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
using std::string;
using std::ifstream;
using std::ofstream;

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
         << "\t./bin/frames2riegl -s 0 -e 1 /Your/directory" << std::endl;
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

  int  fileCounter = start;
  char poseFileName[255];
  char frameFileName[255];

  ifstream pose_in;
  ofstream pose_out;

  double inMatrix[16];
  double tMatrix[17];

  for (;;) {
    if (end > -1 && fileCounter > end) break; // 'nuf read
    snprintf(frameFileName,255,"%sscan%.3d.frames",dir.c_str(),fileCounter);
    snprintf(poseFileName,255,"%sscan%.3d.dat",dir.c_str(),fileCounter++);

    pose_in.open(frameFileName);

    if (!pose_in.good()) break; // no more files in the directory
    // read 3D scan

    cout << "Reading frame " << frameFileName << "..." << endl;

    while(pose_in.good()) {
      for (unsigned int i = 0; i < 17; pose_in >> tMatrix[i++]);
    }

    inMatrix[5] = tMatrix[0];
    inMatrix[9] = -tMatrix[1];
    inMatrix[1] = -tMatrix[2];
    inMatrix[13] = -tMatrix[3];
    inMatrix[6] = -tMatrix[4];
    inMatrix[10] = tMatrix[5];
    inMatrix[2] = tMatrix[6];
    inMatrix[14] = tMatrix[7];
    inMatrix[4] = -tMatrix[8];
    inMatrix[8] = tMatrix[9];
    inMatrix[0] = tMatrix[10];
    inMatrix[12] = tMatrix[11];
    inMatrix[7] = -tMatrix[12];
    inMatrix[11] = tMatrix[13];
    inMatrix[3] = tMatrix[14];
    inMatrix[15] = tMatrix[15];

    inMatrix[3] /= 100;
    inMatrix[7] /= 100;
    inMatrix[11] /= 100;

    pose_in.close();
    pose_in.clear();

    pose_out.open(poseFileName);

    cout << "Writing Riegl pose... " << poseFileName << endl;

    for (int i=0; i < 16; i++) {
      pose_out << inMatrix[i] << " ";
      if((i % 4) == 3) pose_out << endl;
    }


    pose_out.close();
    pose_out.clear();


    cout << " done." << endl;
  }

}

