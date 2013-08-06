/*
 * framesdiff2frames implementation
 *
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

int parseArgs(int argc,char **argv, char fframe1[255], char fframe2[255], char sframe[255]) { 
  int  c;
  // from unistd.h
  extern char *optarg;
  extern int optind;

  while ((c = getopt (argc, argv, "a:b:c:")) != -1)
    switch (c)
   {
   case 'a':
     strncpy(fframe1, optarg, 255);
     break;
   case 'b':
     strncpy(fframe2, optarg, 255);
     break;
   case 'c':
     strncpy(sframe, optarg, 255);
     break;
   }

  //cout << fframe1 << " " << fframe2 << " "  << sframe << " " << optind << " " << argc << endl; 

  if (optind != argc) {
    cerr << "\n*** Directory missing ***\n" << endl; 
    cerr << endl
	  << "Usage: " << argv[0] << "  [-a filename] [-b filename] [-c filename]" << endl << endl;

    cerr  << "  -a filename  The name of the first .frames file of the first scan (to be retained)" << endl
          << "  -b filename  The name of the second .frames file of the first scan (to be ignored)" << endl
          << "  -c filename  The name of the .frames file of the second scan (to be replaced)" << endl
	        << endl;
    cerr << "Creates a unified .frames file out of the result of seperate calls to slam6d that involve the same scan." << endl;
    cerr << "The scenario is assumed to be as follows: Scan 1 has been matched to some other scan." << endl;
    cerr << "This results in a .frames file for Scan 1 (a). Scan 1 and Scan 2 are matched in a seperate call to slam6d." << endl;
    cerr << "This results in two .frames files, one for Scan 1 (b) and one for Scan 2 (c). This program computes a" << endl;
    cerr << "frame that can be used in conjunction with .frames file (a)." << endl;
    abort();
  }

  return 0;
}


void readFrame(char *filename, double *transmat) {
  ifstream file_in;
  double junk;
  
  file_in.open(filename);

  if (!file_in.good()) {
    M4identity(transmat);
    return;
  }

 // cout << "Reading frame " << filename << "..." << endl;

  while(file_in.good()) {
    for (unsigned int i = 0; i < 16; file_in >> transmat[i++]);
    file_in >> junk;
  }

//  cout << transmat << endl;

  file_in.close();
  file_in.clear();

}


int main(int argc, char **argv)
{
  char firstframes1[255];
  char firstframes2[255];
  char secondframes[255];
  parseArgs(argc, argv, firstframes1, firstframes2, secondframes);

  double fframe1[16];
  double fframe2[16];
  double sframe[16];



  readFrame(firstframes1, fframe1);
  readFrame(firstframes2, fframe2);
  readFrame(secondframes, sframe);

  double tinv[16];
  M4inv(fframe2, tinv);

  double tmat[16];
  MMult(tinv, sframe, tmat);

  double result[16];
  MMult(fframe1, tmat, result);
  cout << result << " 1" <<  endl;
}

