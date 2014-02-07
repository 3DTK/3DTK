/*
 * average6DoFposes implementation
 *
 * Copyright (C) Andreas Nuechter
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file
 * @brief averaging 6DoF poses
 *
 * @author Andreas Nuechter. University of Wuerzburg, Germany
 */

#include <fstream>
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
using std::ifstream;
using std::ofstream;

#include <vector>
using std::vector;

#include <string.h>
#include <cmath>

#include "newmat/newmat.h"
#include "newmat/newmatap.h"
using namespace NEWMAT;

#ifndef _MSC_VER
#include <unistd.h>
#else
#include "XGetopt.h"
#endif

#if WIN32
#define snprintf sprintf_s
#endif

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
	  << "   " << prog << "filename"
	  << endl << endl
	  << "  a file containg in every line a 4x4 matrix is read and an" << endl
	  << "  average 6DoF pose is computed" << endl << endl;
}
    

int main(int argc, char **argv)
{
  double *tMatrix;
  vector<double *> tMatrices;
  ifstream pose_in;
  char FileName[255];

  if (argc != 2) usage(argv[0]);
  
  snprintf(FileName,255,argv[1]);
  pose_in.open(FileName);
  if (!pose_in.good()) {
    usage(argv[0]);
    return -1;
  }

  while(pose_in.good()) {
    tMatrix = new double[16];
    for(unsigned int i = 0; i < 16; i++) pose_in >> tMatrix[i];
    tMatrices.push_back(tMatrix);
  }


  double avgMatrix[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  for (unsigned int i = 0; i < tMatrices.size(); i++) {
    for(unsigned int j = 0; j < 16; j++) avgMatrix[j] += tMatrices[i][j];
  }

  for(unsigned int j = 0; j < 16; j++) avgMatrix[j] /= (tMatrices.size()-1);

  cout << endl
	  << "average matrix before finding nearest orthonormal matrix" << endl;
  for(unsigned int j = 0; j < 16; j++) cout << avgMatrix[j] << " ";
  cout << endl;

  Matrix M(3, 3); M = 0.0;
  M(1,1) = avgMatrix[0];
  M(2,1) = avgMatrix[1];
  M(3,1) = avgMatrix[2];
  M(1,2) = avgMatrix[4];
  M(2,2) = avgMatrix[5];
  M(3,2) = avgMatrix[6];
  M(1,3) = avgMatrix[8];
  M(2,3) = avgMatrix[9];
  M(3,3) = avgMatrix[10];

  cout << " det= " << M.Determinant() << endl;
 
  SymmetricMatrix HHs(3);
  HHs << M.t() * M;
  
  /// get the eigenvalues of HH
  DiagonalMatrix eigenvalues(3);
  Matrix eigenvectorsMatrix(3, 3);
  EigenValues(HHs, eigenvalues, eigenvectorsMatrix);

  /// extract the eigenvectors from the matrix to separate column vectors
  ColumnVector ev1 = eigenvectorsMatrix.Column(1);
  ColumnVector ev2 = eigenvectorsMatrix.Column(2);
  ColumnVector ev3 = eigenvectorsMatrix.Column(3);


  /// compute the rotation matrix as H * ( SUM ( 1/lambda_i * ev_i * ev_i^T ))
  Matrix R(3, 3);
  Matrix ev11 = ev1 * ev1.t() * 1/sqrt( eigenvalues(1));
  Matrix ev22 = ev2 * ev2.t() * 1/sqrt( eigenvalues(2));
  Matrix ev33 = ev3 * ev3.t() * 1/sqrt( eigenvalues(3));
  R = M * (ev11 + ev22 + ev33);

  avgMatrix[0] = R(1,1);
  avgMatrix[1] = R(2,1);
  avgMatrix[2] = R(3,1);
  avgMatrix[4] = R(1,2);
  avgMatrix[5] = R(2,2);
  avgMatrix[6] = R(3,2);
  avgMatrix[8] = R(1,3);
  avgMatrix[9] = R(2,3);
  avgMatrix[10] = R(3,3);

  cout << endl << "average matrix" << endl;
  for(unsigned int j = 0; j < 16; j++) cout << avgMatrix[j] << " ";
  cout << endl;

  cout << " det= " << R.Determinant() << endl << endl;

  
  return 0;
}

