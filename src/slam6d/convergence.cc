/*
 * convergence implementation
 *
 * Copyright (C) Jochen Sprickerhof, Peter Schneider
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file
 * @brief Implementation for generating a convergence graph of a scanseries frame
 * @author Jochen Sprickerhof. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Peter Schneider. Institute of Computer Science, University of Koblenz, Germany.
 */

#include <stdexcept>

#include "slam6d/scan.h"
#include "slam6d/convergence.h"

/**
 * Storing the base directory
 */
std::string filepath;

/**
 * Explains the usage of this program's command line parameters
 * @param prog name of the program
 */
void usage(char* prog)
{
#ifndef _MSC_VER
  const std::string bold("\033[1m");
  const std::string normal("\033[m");
#else
  const std::string bold("");
  const std::string normal("");
#endif

  std::cout << std::endl
	  << "Usage: " << prog << " [-s NR] filepath, [-z NR] convergence type" << std::endl << std::endl;

  std::cout << "  -s NR   generate convergence-data for frame NR" << std::endl
	  << std::endl;

  std::cout << "  -z NR   type of convergence (0 = global, 1 = local)" << std::endl
	  << std::endl;

  exit(1);
}


/**
 * A function that parses the command-line arguments and sets the respective flags.
 *
 * @param argc the number of arguments
 * @param argv the arguments
 * @param dir parsing result - the directory
 * @param start parsing result - starting at scan number 'start'
 * @param type parsing result - the type of convergence that should be stored (lokal, global)
 * @return 0, if the parsing was successful, 1 otherwise
 */
int parseArgs(int argc,char **argv, std::string &dir, int &frame, int &type)
{
  frame   = 0;
  type = 0;
  int  c;
  // from unistd.h
  extern char *optarg;
  extern int optind;

  std::cout << std::endl;
  while ((c = getopt (argc, argv, "s:z:m:M:p:wt")) != -1)
    switch (c)
	 {
	 case 's':
	   frame = atoi(optarg);
	   if (frame < 0) { std::cerr << "Error: Cannot generate data of a negative frame number.\n"; exit(1); }
	   break;
	 case 'z':
	   type = atoi(optarg);
	   if (type > 2 || type < 0) { std::cerr << "Error: only global (0) or local (1) available.\n"; exit(1); }
	   break;
	 case '?':
	   usage(argv[0]);
	   return 1;
      default:
	   abort ();
      }

  if (optind != argc-1) {
    std::cerr << "\n*** Directory missing ***" << std::endl;
    usage(argv[0]);
  }
  dir = argv[optind];

#ifndef _MSC_VER
  if (dir[dir.length()-1] != '/') dir = dir + "/";
#else
  if (dir[dir.length()-1] != '\\') dir = dir + "\\";
#endif
  return 0;
}


void getLocalConvergence(std::ifstream *inputFile, std::ofstream *outputFile)
{
  double transMat[16];
  int type;
  double rPos[3];
  double rPosTheta[3];
  std::cout<<"starting local convergence"<<std::endl;
  while ((*inputFile).good()) {
    try {
      (*inputFile) >> transMat >> type;
      if(type == Scan::ICP)
      {
        Matrix4ToEuler(transMat, rPosTheta, rPos);
        (*outputFile)<<rPos[0]<<" "<<rPos[1]<<" "<<rPos[2]<<std::endl;
      }
    }
    catch (const std::exception &e) {
      break;
    }
  }
}

void getAllConvergence(std::ifstream *inputFile, std::ofstream *outputFile, int FrameNr)
{
  double transMat[16];
  int type;
  double rPosOrg[3], rPosThetaOrg[3];
  double quatOrg[4];
  //108
  //double truth[16] = {-0.125888, 0.0122898, -0.991968, 0, 0.00384827, 0.999922, 0.0118999, 0, 0.992037, -0.00231931, -0.125925, 0, -1702.16, 34.255, 285.929, 1};
  //344
  //double truth[16] = {-0.548304, -0.00375733, 0.83627, 0, 0.0294523, 0.999283, 0.0238003, 0, -0.83576, 0.0376799, -0.5478, 0, 691.717, 55.8939, 216.401, 1};
  //556
  double truth[16] = {0.996696, 0.0336843, -0.073908, 0, -0.0325397, 0.999332, 0.016638, 0, 0.0744191, -0.0141781, 0.997126, 0, 6211.62, 364.291, -3468.65, 1};
  //708
  //double truth[16] = {0.000564156, -0.0283691, -0.999597, 0, 0.00348951, 0.999591, -0.0283669, 0, 0.999994, -0.0034721, 0.00066292, 0, 7591.08, -31.354, 6938.24, 1};
  Matrix4ToEuler(truth, rPosThetaOrg, rPosOrg);
  Matrix4ToQuat(truth, quatOrg);
  double quat[4];
  double rPos[3];
  double rPosTheta[3];
  bool initial = true;

  std::cout<<"starting all convergence"<<std::endl;
  while ((*inputFile).good()) {
    try {
      (*inputFile) >> transMat >> type;
      Matrix4ToEuler(transMat, rPosTheta, rPos);
      Matrix4ToQuat(transMat, quat);
      if((initial || type != Scan::INVALID) && type != Scan::ICPINACTIVE) {
        initial = false;
        (*outputFile) << sqrt(Dist2(rPosOrg, rPos))/100.0 << " " << quat_dist(quatOrg, quat) << " " << type << std::endl;
      }
      //(*outputFile) << sqrt(Dist2(rPosOrg, rPos))/100.0 << " " << quat_dist(quatOrg, quat) << " " << type << endl;
    }
    catch (const std::exception &e) {
      break;
    }
  }
}

void getGlobalConvergence(std::ifstream *inputFile, std::ofstream *outputFile)
{
  double transMat[16];
  int type;
  bool lumYetFound = false;
  double rPos[3];
  double rPosTheta[3];
  std::cout<<"starting global convergence"<<std::endl;
  while ((*inputFile).good()) {
    try {
      (*inputFile) >> transMat >> type;
      if(type == Scan::LUM)
      {
        lumYetFound = true;
        Matrix4ToEuler(transMat, rPosTheta, rPos);
        (*outputFile)<<rPos[0]<<" "<<rPos[1]<<" "<<rPos[2]<<std::endl;
      } else
      {
        if(lumYetFound)             //we only want to write the last lum-correction into the file
        {
          lumYetFound = false;
          outputFile->close();
          outputFile->open("xyz.con", std::ios::trunc);
        }
      }
    }
    catch (const std::exception &e) {
      break;
    }
  }
}


/*
 * A function that read the .frame files created by slam6D
 *
 * @param dir the directory
 * @param frameNr frame number that should be read
 */
void readFrames(std::string dir, int frameNr, int convType)
{
  std::ifstream frame_in;
  std::ofstream xyz_out;
  xyz_out.open("xyz.con");
  std::string frameFileName;
  frameFileName = dir + "scan" + to_string(frameNr,3) + ".frames";
    std::cout << "Reading Frame for convergence data " << frameFileName << "..."<<std::endl;
  frame_in.open(frameFileName.c_str());

  // read frame
  if (!frame_in.good()) std::cout<<"could not open file!"<<std::endl; // no more files in the directory
  else
  {
      if(convType == CONV_LOCAL)
      {
        getLocalConvergence(&frame_in, &xyz_out);
      } else if (convType == CONV_GLOBAL)
      {
        getGlobalConvergence(&frame_in, &xyz_out);
      } else if (convType == CONV_ALL)
      {
        getAllConvergence(&frame_in, &xyz_out, frameNr);
      }

    frame_in.close();
    frame_in.clear();
  }
  xyz_out.close();
}

//-----------------------------------------------------------------------------------



/**
 * Main function.
 * Reads a frames file (scan000.frames, ...) from the data directory.
 * The frame is used for generating the convergencedata of a scan.
 */

int main(int argc, char **argv){

  std::cout << "(c) University of Osnabrueck, 2008" << std::endl << std::endl
	  << "Restricted Usage" << std::endl
	  << "Don't use without permission" << std::endl;

  if(argc <= 1){
    usage(argv[0]);
  }
  int convT;
  int frameNumber = 0;
  std::string dir;

  parseArgs(argc, argv, dir, frameNumber, convT);
//  scandir = dir;

  // Get frame-data
  readFrames(dir, frameNumber, convT);
}


