/**
 * @file
 * @brief Implementation for generating a convergence graph of a scanseries frame
 * @author Jochen Sprickerhof
 * @author Peter Schneider. Institute of Computer Science, University of Koblenz, Germany.
 */

#include <stdexcept>
using std::exception;

#include "convergence.h"

/**
 * Storing the base directory
 */
string filepath;

/**
 * Explains the usage of this program's command line parameters
 * @param prog name of the program
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
	  << "Usage: " << prog << " [-s NR] filepath, [-z NR] convergence type" << endl << endl;

  cout << "  -s NR   generate convergence-data for frame NR" << endl
	  << endl;

  cout << "  -z NR   type of convergence (0 = global, 1 = local)" << endl
	  << endl;

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
int parseArgs(int argc,char **argv, string &dir, int &frame, int &type)
{
  frame   = 0;
  type = 0;
  int  c;
  // from unistd.h
  extern char *optarg;
  extern int optind;

  cout << endl;
  while ((c = getopt (argc, argv, "s:z:m:M:p:wt")) != -1)
    switch (c)
	 {
	 case 's':
	   frame = atoi(optarg);
	   if (frame < 0) { cerr << "Error: Cannot generate data of a negative frame number.\n"; exit(1); }
	   break;
	 case 'z':
	   type = atoi(optarg);
	   if (type > 1 || type < 0) { cerr << "Error: only global (0) or local (1) available.\n"; exit(1); }
	   break;
	 case '?':
	   usage(argv[0]);
	   return 1;
      default:
	   abort ();
      }

  if (optind != argc-1) {
    cerr << "\n*** Directory missing ***" << endl;
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


void getLocalConvergence(ifstream *inputFile, ofstream *outputFile)
{
  double transMat[16];
  double temp;
  double first, second;
  first = second = -2;
  double rPos[3];
  double rPosTheta[3];
  cout<<"starting local convergence"<<endl;
  while ((*inputFile).good()) {
    try {
      (*inputFile) >> transMat;
      (*inputFile) >> first;
      (*inputFile) >> second;
      for (int i = 0; i < 2 ; i++) {
        (*inputFile) >> temp;
      }
      if(first == 0 && second == 0)
      {
        Matrix4ToEuler(transMat, rPosTheta, rPos);
        (*outputFile)<<rPos[0]<<" "<<rPos[1]<<" "<<rPos[2]<<endl;
      }
    }
    catch (const exception &e) {   
      break;
    }
  }
}

/* aus ROOT (CERN)
 * auch in:
 * Effective Sampling and Distance Metrics for 3D Rigid Body Path Planning
 * James J. Kuffner
 *
 * Distance between two rotations in Quaternion form
 * Note:  The rotation group is isomorphic to a 3-sphere
 * with diametrically opposite points identified.
 * The (rotation group-invariant) is the smaller
 * of the two possible angles between the images of
 * the two rotations on that sphere.  Thus the distance
 * is never greater than pi/2.
 */
double quat_dist(double quat1[4], double quat2[4]) {
  double chordLength = std::fabs(quat1[0]*quat2[0] + quat1[1]*quat2[1] + quat1[2]*quat2[2] + quat1[3]*quat2[3]);
  if (chordLength > 1) chordLength = 1; // in case roundoff fouls us up
  return acos(chordLength) / M_PI * 180.0;
}

double dist2(double quat1[4], double quat2[4]) {
  return sqrt(sqr(quat1[0] - quat2[0]) + sqr(quat1[1] - quat2[1]) + sqr(quat1[2] - quat2[2]) + sqr(quat1[3] - quat2[3]));
}

double dist3(double quat1[4], double quat2[4]) {
  double tmp1[4], tmp2[4];
  tmp1[0] = quat1[0];
  tmp1[1] = -quat1[1];
  tmp1[2] = -quat1[2];
  tmp1[3] = -quat1[3];
  tmp2[0] = -tmp1[1] * quat2[1] - tmp1[2] * quat2[2] - tmp1[3] * quat2[3] + tmp1[0] * quat2[0];
  tmp2[1] =  tmp1[1] * quat2[0] + tmp1[2] * quat2[3] - tmp1[3] * quat2[2] + tmp1[0] * quat2[1];
  tmp2[2] = -tmp1[1] * quat2[3] + tmp1[2] * quat2[0] + tmp1[3] * quat2[1] + tmp1[0] * quat2[2];
  tmp2[3] =  tmp1[1] * quat2[2] - tmp1[2] * quat2[1] + tmp1[3] * quat2[0] + tmp1[0] * quat2[3];
  QuatToAA(tmp2);
  return tmp2[0]; // /M_PI*180.0;
}

/* die ist nicht gut/gleich zu dist2
 * bi-invariant distance metric on SO(3)
 * in:
 * Effective Sampling and Distance Metrics for 3D Rigid Body Path Planning
 * James J. Kuffner
  */
double dist4(double quat1[4], double quat2[4]) {
  double tmp1[4], tmp2[4];
  tmp1[0] = quat1[0];
  tmp1[1] = -quat1[1];
  tmp1[2] = -quat1[2];
  tmp1[3] = -quat1[3];
  tmp2[0] = -tmp1[1] * quat2[1] - tmp1[2] * quat2[2] - tmp1[3] * quat2[3] + tmp1[0] * quat2[0];
  tmp2[1] =  tmp1[1] * quat2[0] + tmp1[2] * quat2[3] - tmp1[3] * quat2[2] + tmp1[0] * quat2[1];
  tmp2[2] = -tmp1[1] * quat2[3] + tmp1[2] * quat2[0] + tmp1[3] * quat2[1] + tmp1[0] * quat2[2];
  tmp2[3] =  tmp1[1] * quat2[2] - tmp1[2] * quat2[1] + tmp1[3] * quat2[0] + tmp1[0] * quat2[3];
  QuatToAA(tmp2);
  Normalize3(tmp2 + 1);
  tmp2[1] *= tmp2[0];
  tmp2[2] *= tmp2[0];
  tmp2[3] *= tmp2[0];
  return sqrt(sqr(tmp2[1]) + sqr(tmp2[2]) + sqr(tmp2[3]));
}

//Andreas
double dist5(double quat1[4], double quat2[4]) {
  double tmp1[4], tmp2[4];
  tmp1[0] = quat1[0];
  tmp1[1] = quat1[1];
  tmp1[2] = quat1[2];
  tmp1[3] = quat1[3];
  QuatToAA(tmp1);
  Normalize3(tmp1 + 1);
  tmp1[1] *= tmp1[0];
  tmp1[2] *= tmp1[0];
  tmp1[3] *= tmp1[0];
  tmp2[0] = quat2[0];
  tmp2[1] = quat2[1];
  tmp2[2] = quat2[2];
  tmp2[3] = quat2[3];
  QuatToAA(tmp2);
  Normalize3(tmp2 + 1);
  tmp2[1] *= tmp2[0];
  tmp2[2] *= tmp2[0];
  tmp2[3] *= tmp2[0];
  return sqrt(Dist2(tmp1 + 1, tmp2 + 1));
}

void getGlobalConvergence(ifstream *inputFile, ofstream *outputFile)
{
  double transMat[16];
  double rPosOrg[3], rPosThetaOrg[3];
  double quatOrg[4];
  //108
  //double tmp[16] = {-0.125888, 0.0122898, -0.991968, 0, 0.00384827, 0.999922, 0.0118999, 0, 0.992037, -0.00231931, -0.125925, 0, -1702.16, 34.255, 285.929, 1};
  //344
  //double tmp[16] = {-0.548304, -0.00375733, 0.83627, 0, 0.0294523, 0.999283, 0.0238003, 0, -0.83576, 0.0376799, -0.5478, 0, 691.717, 55.8939, 216.401, 1};
  //556
  double tmp[16] = {0.996696, 0.0336843, -0.073908, 0, -0.0325397, 0.999332, 0.016638, 0, 0.0744191, -0.0141781, 0.997126, 0, 6211.62, 364.291, -3468.65, 1};
  //708
  //double tmp[16] = {0.000564156, -0.0283691, -0.999597, 0, 0.00348951, 0.999591, -0.0283669, 0, 0.999994, -0.0034721, 0.00066292, 0, 7591.08, -31.354, 6938.24, 1};
  Matrix4ToEuler(tmp, rPosThetaOrg, rPosOrg);
  Matrix4ToQuat(tmp, quatOrg);
  double quat[4];
  double temp;
  double first, second;
  double rPos[3];
  double rPosTheta[3];
  bool initial = true;

  cout<<"starting global convergence"<<endl;
  while ((*inputFile).good()) {
    try {
      (*inputFile) >> transMat;
      (*inputFile) >> first;
      (*inputFile) >> second;
      for (int i = 0; i < 2 ; i++) {
        (*inputFile) >> temp;
      }
      Matrix4ToEuler(transMat, rPosTheta, rPos);
      Matrix4ToQuat(transMat, quat);
      if((initial || first != -1) && second != 1)
      {
        initial = false;
        (*outputFile) << sqrt(Dist2(rPosOrg, rPos))/100.0 << " " << quat_dist(quatOrg, quat) << endl;
      }
    }
    catch (const exception &e) {   
      break;
    }
  }
}

void getGlobalConvergence_org(ifstream *inputFile, ofstream *outputFile)
{
  double transMat[16];
  double temp;
  bool lumYetFound = false;
  double first, second;
  double rPos[3];
  double rPosTheta[3];
  cout<<"starting global convergence"<<endl;
  while ((*inputFile).good()) {
    try {
      (*inputFile) >> transMat;
      (*inputFile) >> first;
      (*inputFile) >> second;
      for (int i = 0; i < 2 ; i++) {
        (*inputFile) >> temp;
      }
      if((int)first == 1 && (int)second == 0)
      {
        lumYetFound = true;
        Matrix4ToEuler(transMat, rPosTheta, rPos);
        (*outputFile)<<rPos[0]<<" "<<rPos[1]<<" "<<rPos[2]<<endl;
      } else 
      {
        if(lumYetFound)             //we only want to write the last lum-correction into the file
        {
          lumYetFound = false;
          outputFile->close();
          outputFile->open("xyz.con", ios::trunc);
        }
      }
    }
    catch (const exception &e) {   
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
void readFrames(string dir, int frameNr, int convType)
{
  ifstream frame_in;
  ofstream xyz_out;
  xyz_out.open("xyz.con");
  string frameFileName;
  frameFileName = dir + "scan" + to_string(frameNr,3) + ".frames";
    cout << "Reading Frame for convergence data " << frameFileName << "..."<<endl;
  frame_in.open(frameFileName.c_str());

  // read frame
  if (!frame_in.good()) cout<<"could not open file!"<<endl; // no more files in the directory
  else
  {
      if(convType == CONV_LOCAL)
      {
        getLocalConvergence(&frame_in, &xyz_out);
      } else if (convType == CONV_GLOBAL)
      {
        getGlobalConvergence(&frame_in, &xyz_out);
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
  
  cout << "(c) University of Osnabrueck, 2008" << endl << endl
	  << "Restricted Usage" << endl
	  << "Don't use without permission" << endl;

  if(argc <= 1){
    usage(argv[0]);
  }
  int convT;
  int frameNumber = 0;
  string dir;

  parseArgs(argc, argv, dir, frameNumber, convT);
//  scandir = dir;

  // Get frame-data
  readFrames(dir, frameNumber, convT);
}


