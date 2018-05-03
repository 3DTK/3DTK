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

#include "slam6d/icp6Dsvd.h"
#include "slam6d/globals.icc"

#ifndef _MSC_VER
#include <getopt.h>
#else
#include "XGetopt.h"
#endif

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

/**
 * Explains the usage of this program's command line parameters
 */
void usage(char* prog)
{
#ifndef _MSC_VER
    const string bold("\033[1m");
    const string normal("\033[m");
#else
    const string bold("");
    const string normal("");
#endif
    cout << endl
        << bold << "USAGE " << normal << endl
        << "   " << prog << " [parameters] directory" << endl << endl;
    cout << bold << "PARAMETERS" << normal << endl

        << bold << "  -s" << normal << " NR, " << bold << "--start=" << normal << "NR" << endl
        << "         start at frame NR (i.e., neglects the first NR frame files)" << endl
        << "         [ATTENTION: counting naturally starts with 0]" << endl
        << bold << "  -e" << normal << " NR, " << bold << "--end=" << normal << "NR" << endl
        << "         end after frame file NR" << endl
        << endl
        << bold << "  -o" << normal << " NR, " << bold << "--origin=" << normal << "NR" << endl
        << "         use scan <NR> as origin for transformation" << endl
        << endl
        << bold << "  -i" << normal << " STR, " << bold << "--input=" << normal << "STR" << endl
        << "         input frames file containing the transformation from origin to global" << endl
        << endl
        << bold << "  -f" << normal <<  " DIR, " << bold << "--outdir=" << normal << "DIR" << endl
        << "         write new .frames files  to <DIR>" << endl
        << endl
        << bold << "  -t" << normal << ", " << bold << "--trustpose" << normal << "" << endl
        << "         use .pose instead of .frames files" << endl
        << endl
        << endl << endl;

    exit(1);
}

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
    ("input,i", po::value<string>(&inputMatrix)->default_value("none"),
     "end after scan <arg>")
    ("trustpose,p",po::bool_switch(&readFromPose)->default_value(true),
    "Trust the pose file, do not extrapolate the last transformation."
    "(just for testing purposes, or gps input.)")
    ("outdir,f", po::value<string>(&outputDir)->default_value("none"),
     "end after scan <arg>")
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
         << "\t./bin/pose2frames -s 0 -e 1 /Your/directory" << std::endl;
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


void readFramesFromFile(const char * filename, double * tMatrix) {
  ifstream infile;
  infile.open(filename);
  float dummy = -1;
  
  while(infile.peek() != EOF) {
    for (unsigned int i = 0; i < 16; infile >> tMatrix[i++]);
    infile >> dummy;
  }

  infile.close();
  infile.clear();
}

void readFrames(const char * dir, int index, double * tMatrix) {
  ifstream infile;
  char filename[255];
  
  snprintf(filename,255,"%sscan%.3d.frames",dir,index);
  infile.open(filename);
  float dummy = -1;
  
  while(infile.peek() != EOF) {
    for (unsigned int i = 0; i < 16; infile >> tMatrix[i++]);
    infile >> dummy;
  }
  //printFrames(tMatrix);

  infile.close();
  infile.clear();
}

void readPose(const char * dir, int index, double * tMatrix) {
  ifstream infile;
  char filename[255];
  snprintf(filename,255,"%sscan%.3d.pose",dir,index);
  infile.open(filename);
 
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
    if (argc <= 1) {
        usage(argv[0]);
    }

    string inputdir;
    string outputdir;
    string inmatrix;
    bool readFromPose=false;

    int start = 0, end = -1, anchor = 0;

    parse_options(argc, argv, inputdir, inmatrix, outputdir, start, end, readFromPose,anchor);

    double * inverse = new double[16];
    double * mult = new double[16];
    double * resTrans = new double[16];
    double * in = new double[16];

    if(readFromPose) {
      readPose(inputdir.c_str(),anchor,in);
    } else {
      readFrames(inputdir.c_str(),anchor,in);
    }
    M4inv(in,inverse);
    readFramesFromFile(inmatrix.c_str(),in);
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
