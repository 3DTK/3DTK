/**
 * @file
 * @brief generating convergence data
 * @author Peter Schneider. Institute of Computer Science, University of Koblenz, Germany.
 */

#ifndef __CONVERGENCE_H__
#define __CONVERGENCE_H__

#ifdef _MSC_VER
  #define  _USE_MATH_DEFINES
  #include <windows.h>
  #include "XGetopt.h"
#else
  #include <unistd.h>
#endif

#include <math.h>
#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#else
#include <strings.h>
#endif
#include <fstream>

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
using std::ofstream;
using std::ios;
using std::ifstream;

#include <string>
using std::string;

/** float value "nearly" zero? */
#define DIV_EPSILON     0.001
/** comparing two floats/doubles */
#define COMPARE_EPSILON 0.000001

#define CONV_LOCAL 1
#define CONV_GLOBAL 0
#define CONV_ALL 2

#define BUFSIZE 1048576 ///< defining the buffer size

#include "slam6d/globals.icc"


int parseArgs(int argc, char **argv, string &dir, int& frame);
void usage(char * prog);

void getGlobalConvergence(ifstream *inputFile, ofstream *outputFile);
void getLocalConvergence(ifstream *inputFile, ofstream *outputFile);
void getAllConvergence(ifstream *inputFile, ofstream *outputFile, int FrameNr);



#endif

