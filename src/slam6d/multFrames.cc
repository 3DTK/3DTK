/**
 * @file
 * @brief Transforms a series of frames according to a replacement for a
 * reference frame
 *
 * @author Dorit Borrmann
 */

#ifdef _MSC_VER
#ifdef OPENMP
#ifndef _OPENMP
#define _OPENMP
#endif
#endif
#endif

#ifdef _OPENMP
#include <omp.h>
#endif

#define WANT_STREAM ///< define the WANT stream :)

#include <string>
using std::string;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <fstream>
using std::ofstream;
using std::ifstream;

#include "slam6d/globals.icc"

#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#else
#include <strings.h>
#endif

#if WIN32
#define snprintf sprintf_s
#else
#ifndef MAX_PATH
#define MAX_PATH 255
#endif
#endif

#include <boost/program_options.hpp>
namespace po = boost::program_options;

int parse_options(int argc, char **argv, string &dir, string &inputMatrix,
    string &outputDir, int &start, int &end, bool& readFromPose, int& anchor)
{
    po::options_description generic("Generic options");
  generic.add_options()
    ("help,h", "output this help message");

  po::options_description input("Input options");
  input.add_options()
    ("start,s", po::value<int>(&start)->default_value(0),
     "start at scan <arg> (i.e., neglects the first <arg> scans) "
     "[ATTENTION: counting naturally starts with 0]")
    ("end,e", po::value<int>(&end)->default_value(-1),
     "end after scan <arg>")
    ("input,i", po::value<string>(&inputMatrix),
     "input frames file containing the transformation from origin to global")
    ("trustpose,p",po::bool_switch(&readFromPose)->default_value(false),
    "use .pose instead of .frames files")
    ("outdir,f", po::value<string>(&outputDir),
     "write new .frames files  to <DIR>")
    ("origin,o", po::value<int>(&anchor)->default_value(0),
     "use scan <NR> as origin for transformation");

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
         << "\t./bin/multFrames -s 0 -e 1 -o 0 -i <frames_file> -f <outdir> <dir>" << std::endl;
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

void printFrames(double * tMatrix) {
  for (unsigned int i = 0; i < 16; cout << tMatrix[i++] << " ");
  cout << endl;
}


bool readFramesFromFile(const char * filename, double * tMatrix) {
  ifstream infile;
  infile.open(filename);
  if(!infile.good()) {
    cerr << "Could not read input " << filename << endl;
    return false;
  }
  float dummy = -1;

  while(infile.peek() != EOF) {
    for (unsigned int i = 0; i < 16; infile >> tMatrix[i++]);
    infile >> dummy;
  }

  infile.close();
  infile.clear();

  return true;
}

bool readFrames(const char * dir, int index, double * tMatrix) {
  ifstream infile;
  char filename[255];

  snprintf(filename,255,"%sscan%.3d.frames",dir,index);
  cout << "Reading... " << filename << endl;
  infile.open(filename);
  if(!infile.good()) {
    cerr << "Could not read input " << filename << endl;
    return false;
  }
  float dummy = -1;

  while(infile.peek() != EOF) {
    for (unsigned int i = 0; i < 16; infile >> tMatrix[i++]);
    infile >> dummy;
  }
  //printFrames(tMatrix);

  infile.close();
  infile.clear();

  return true;
}

bool readPose(const char * dir, int index, double * tMatrix) {
  ifstream infile;
  char filename[255];
  snprintf(filename,255,"%sscan%.3d.pose",dir,index);

  cout << "Reading... " << filename << endl;
  infile.open(filename);

  if(!infile.good()) {
    cerr << "Could not read input " << filename << endl;
    return false;
  }


  double rPos[3];
  double rPosTheta[3];

  if(infile.peek() != EOF) {
    infile >> rPos[0] >> rPos[1] >> rPos[2];
    infile >> rPosTheta[0] >> rPosTheta[1] >> rPosTheta[2];
    for (unsigned int i = 0; i < 3; i++) rPosTheta[i] = rad(rPosTheta[i]);
    EulerToMatrix4(rPos, rPosTheta,tMatrix);
  } else {
    cerr << "Error reading pose file: " << filename << endl;
  }
  //printFrames(tMatrix);
  infile.close();
  infile.clear();

  return true;

}


void writeFrames(const char * dir, int index, double * tMatrix) {
  ofstream outfile;
  char filename[255];
  snprintf(filename,255,"%sscan%.3d.frames",dir,index);
  outfile.open(filename);
  if(outfile.good()) {
    for(unsigned int j = 0; j < 3; j++) {
      for (unsigned int i = 0; i < 16; outfile << std::setprecision(15) << tMatrix[i++] << " ");
      outfile << "2" << endl;
    }
  }
  outfile.close();
  outfile.clear();
}
/**
 * Converts a number of frame files. Takes an input file with a transformation
 * that transforms the anchor scan from the scanner own coordinate system into a
 * global coordinate system. Assuming the scans to be registered, all scans are
 * first transformed by the inverse of the transformation given in the frames of
 * the anchor scan before applying the new transformation.
 * The resulting frames will be written into a new directory.
 */
int main(int argc, char **argv)
{
    string inputdir;
    string outputdir;
    string inmatrix;
    bool readFromPose=false;

    int start = 0, end = -1, anchor = 0;

    parse_options(argc, argv, inputdir, inmatrix, outputdir, start, end, readFromPose,anchor);

    if(inputdir.empty() || outputdir.empty() || inmatrix.empty()) {
      cout << "Please specify all input parameters." << endl;
      exit(1);
    }

    double * inverse = new double[16];
    double * mult = new double[16];
    double * resTrans = new double[16];
    double * in = new double[16];
    bool no_fail = false;
    if(readFromPose) {
      no_fail = readPose(inputdir.c_str(),anchor,in);
    } else {
      no_fail = readFrames(inputdir.c_str(),anchor,in);
    }
    if(!no_fail) {
      exit(1);
    }
    M4inv(in,inverse);
    no_fail = readFramesFromFile(inmatrix.c_str(),in);
    if(!no_fail) exit(1);

    MMult(in,inverse,mult);
    printFrames(mult);

    for(unsigned int i = start; i <= end; i++) {

      if(readFromPose) {
        readPose(inputdir.c_str(),i,in);
      } else {
        readFrames(inputdir.c_str(),i,in);
      }
      printFrames(in);
      MMult(mult,in,resTrans);
      printFrames(resTrans);
      writeFrames(outputdir.c_str(),i,resTrans);
      cout << endl;
    }
    delete[] in;
    delete[] inverse;
    delete[] mult;
    delete[] resTrans;
}
