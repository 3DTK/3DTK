/*
 * slam6D implementation
 *
 * Copyright (C) by the 3DTK contributors
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file
 * @brief Program for generating feature files of 3D scans
 *
 * @author Michael Neumann. Julius-Maximilians University Wuerzburg, Germany
 */

#include "slam6d/scan.h"
#include "slam6d/metaScan.h"
#include "slam6d/io_utils.h"

#include "slam6d/kd.h"

#include "slam6d/gapx6D.h"
#include "slam6d/graph.h"
#include "slam6d/globals.icc"

#include "newmat/newmat.h"
#include "newmat/newmatap.h"

#include <csignal>

#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#else
#include <strings.h>
#endif

#ifdef WITH_METRICS
#include "slam6d/metrics.h"
#endif //WITH_METRICS


#ifdef _MSC_VER
#if !defined _OPENMP && defined OPENMP
#define _OPENMP
#endif
#endif

#ifdef _OPENMP
#include <omp.h>
#endif

#define WANT_STREAM ///< define the WANT stream :)

#define FEATURE_VECTOR_SIZE     17
#define FEATURE_CYLINDER_VECTOR_SIZE     4
// To replace infinite features
#define BIG_NUMBER     99999

using namespace NEWMAT;
#include <string>
using std::string;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <fstream>
using std::ifstream;

// This piece of code is needed for vector default_values in po library. There might be a better/more general place for it...
namespace std
{
  std::ostream& operator<<(std::ostream &os, const std::vector<int> &vec)
  {
    for (auto item : vec)
    {
      os << item << " ";
    }
    return os;
  }
}


#include <boost/filesystem.hpp>

#include <boost/program_options.hpp>
#include <cfloat>
namespace po = boost::program_options;

//  Handling Segmentation faults and CTRL-C
void sigSEGVhandler (int v)
{
  static bool segfault = false;
  if(!segfault) {
    segfault = true;
    cout << endl
         << "# **************************** #" << endl
         << "  Segmentation fault or Ctrl-C" << endl
         << "# **************************** #" << endl
         << endl;
    // save frames and close scans
    // Attention!! IF WE DONT HANDLE THIS AND CREATE EMPTY .FRAMES FILE WE WILL HAVE PROBLEMS WITH EXPORT_POINTS
    /*for(ScanVector::iterator it = Scan::allScans.begin();
     it != Scan::allScans.end();
     ++it) {
      (*it)->saveFrames(Scan::continue_processing);
    }
    cout << "Frames saved." << endl;
    */
    Scan::closeDirectory();
  }
  exit(-1);
}

void validate(boost::any& v, const std::vector<std::string>& values,
              IOType*, int) {
  if (values.size() == 0)
    throw std::runtime_error("Invalid model specification");
  std::string arg = values.at(0);
  try {
    v = formatname_to_io_type(arg.c_str());
  } catch (...) { // runtime_error
    throw std::runtime_error("Format " + arg + " unknown.");
  }
}

void to_cout(const std::vector<std::string> &v)
{
  std::copy(v.begin(), v.end(), std::ostream_iterator<std::string>{
    std::cout, "\n"});
}


/** A function that parses the command-line arguments and sets the respective flags.
 * @param argc the number of arguments
 * @param argv the arguments
 * @param dir the directory
 * @param start starting at scan number 'start'
 * @param end stopping at scan number 'end'
 * @param quiet switches on/off the quiet mode
 * @param veryQuiet switches on/off the 'very quiet' mode
 * @return 0, if the parsing was successful. 1 otherwise
 */

int parse_options(int argc, char **argv, string &dir, IOType &type,
	      int &start, int &end, int &jobs, int &job, int &algorithm, double &ref1, double &ref2, double &ref3, int &reduction, vector<int> &ignoredClasses, bool &useDistance, vector<int> &kReductions, vector<int> &kMins, vector<int> &kMaxs, vector<int> &kDeltas, vector<int> &cylReductions, vector<int> &cylRadii, bool &csv, bool &quiet, bool &veryQuiet)
{

po::options_description generic("Generic options");
  generic.add_options()
    ("help,h", "output this help message");

  //bool point_to_plane = false;
  //bool normal_shoot = false;
  po::options_description input("Input options");
  input.add_options()
    ("format,f", po::value<IOType>(&type)->default_value(UOS, "uos"),
     "using shared library <arg> for input. (can be used "
     "with semantic classes <arg> in {uosc, xyzc} or "
     "without semantic classes {uos, xyz, ply, ...})")
    ("start,s", po::value<int>(&start)->default_value(0),
     "start at scan <arg> (i.e., neglects the first <arg> scans) "
     "[ATTENTION: counting naturally starts with 0]")
    ("end,e", po::value<int>(&end)->default_value(-1),
     "end after scan <arg>");
  po::options_description algo("Algorithm options");
  algo.add_options()
    ("algorithm,a", po::value<int>(&algorithm)->default_value(0),
     "selects the algorithm mode (more my follow)\n"
     "0 = fixed kNN/optimal neighborhood (Weinmann 2014) on the whole scan.\n"
     "      use with kMin, kDelta, kMax, cylRadius\n"
     "      set kMin=kMax for fixed size");
     //The following lines where used for a debug point. Information about one single point in space were calculated. However, not implemented anymore.
     /*"1 = show kOpt-information about point (x,y,z)")

    ("ref1,x",po::value<double>(&ref1)->default_value(0),
    "first coordinate of reference point (in left-handed uos-format in cm)")
    ("ref2,y",po::value<double>(&ref2)->default_value(0),
    "fecond coordinate of reference point (in left-handed uos-format in cm)")
    ("ref3,z",po::value<double>(&ref3)->default_value(0),
    "third coordinate of reference point (in left-handed uos-format in cm)");*/
  po::options_description featConstruction("Feature construction options");
  featConstruction.add_options()
    ("reduce,r",po::value<int>(&reduction)->default_value(10),
    "defines for which points features are calculated (changes number of output points). One point per voxel with size <arg> is kept.")
    ("ignoredClasses,i",po::value<vector<int>>(&ignoredClasses)->multitoken()->default_value(vector<int>{})->zero_tokens()->composing(),
    "points with class labels within this list are not used as reference points")
    //("useDistance",po::bool_switch(&useDistance)->default_value(false),
    //"kMins, kMaxs, and kDeltas are not number of neighbors but distances/sizes (in cm) of neighborhoods")
    ("kReductions,R",po::value<vector<int>>(&kReductions)->multitoken()->default_value(vector<int>{10})->zero_tokens()->composing(),
    "defines reductions to be done before calculating features (impacts neighborhoods/feature values)")
    ("kMins,k",po::value<vector<int>>(&kMins)->multitoken()->default_value(vector<int>{10})->zero_tokens()->composing(),
    "list defining how many neighbors are used as a minimum for optimal neighborhood minimization function (Weinmann 2014)")
    ("kMaxs,K",po::value<vector<int>>(&kMaxs)->multitoken()->default_value(vector<int>{100})->zero_tokens()->composing(),
    "list defining how many neighbors are used as a maximum for optimal neighborhood minimization function (Weinmann 2014)")
    ("kDeltas,d",po::value<vector<int>>(&kDeltas)->multitoken()->default_value(vector<int>{1})->zero_tokens()->composing(),
    "list defining the step size between kMin and kMax")
    ("cylReductions",po::value<vector<int>>(&cylReductions)->multitoken()->default_value(vector<int>{10})->zero_tokens()->composing(),
    "list defining reductions to be done before calculating cylindrical features. For better run time chose (a subset of) kReductions.")
    ("cylRadii,c",po::value<vector<int>>(&cylRadii)->multitoken()->default_value(vector<int>{25})->zero_tokens()->composing(),
    "list defining the radii of the vertical cylinders that are used to construct cylindrical features (uos-cm-format)");

  po::options_description output("Output options");
  output.add_options()
    ("csv",po::bool_switch(&csv)->default_value(false),
    "Output features as .csv-file instead of .arff (for Weka). Problems when importing to Weka might appear.");
    //("jobs,J",po::value<int>(&jobs)->default_value(1),
    //"Number of jobs the feature generation process is divided to (run one job with -j from 0 to J-1)")
    //("job,j",po::value<int>(&job)->default_value(0),
    //"Nr of this job. Must be between 0 and J-1. Job 0 contains the header for CSV/ARFF");

  po::options_description hidden("Hidden options");
  hidden.add_options()
    ("input-dir", po::value<std::string>(&dir), "input dir");

  // all options
  po::options_description all;
  all.add(generic).add(input).add(algo).add(featConstruction).add(output).add(hidden);

  // options visible with --help
  po::options_description cmdline_options;
  cmdline_options.add(generic).add(input).add(algo).add(featConstruction).add(output);

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
         << "\t./bin/scan2features dat" << std::endl
         << "\t./bin/scan2features  -r 10 --kReductions 2 20 80 --kMins 10 10 10 --kMaxs 100 100 100 --kDeltas 1 1 1 --cylReductions 2 20 80 --cylRadii 25 150 500 -f xyzc ." << std::endl;
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

/**
 * Determines optimal neighborhood-size with an adaptive reference point (mid between original point and neighbors)
 * kDelta is step size from kMin to kMax
 * Was implemented for testing purposes. Did not perform much better and had a long run time => not used
*/
unsigned int determineKOptAdaptive(const KDtree &t, double *reference, unsigned int kMin, unsigned int kMax, unsigned int kDelta, double *newReference) {
  //We later need a small epsilon
  double epsilon = std::numeric_limits<double>::epsilon();

  // Determine optimal neighborhood size
  // For all sizes k of neighborhoods with kMin <= k <= kMax, do PCA. Here: neighbors.size() >= kMax
  std::vector<double> eigenentropies;
  double bestEigenentropy = DBL_MAX;
  double bestReference[] = {0,0,0};
  unsigned int kOpt = 1;

  Point sum(0.0,0.0,0.0);
  Point mean(0.0,0.0,0.0);

  double adaptiveReference[3] = {reference[0], reference[1], reference[2]};
  for (unsigned int k = kMin; k <= kMax; k+=kDelta) {
    vector<Point> neighbors = t.kNearestNeighbors(adaptiveReference, k, 0);
    sum.x = 0;
    sum.y = 0;
    sum.z = 0;
    for (unsigned int i = 0; i < neighbors.size(); i++) {
      sum.x += neighbors.at(i).x;
      sum.y += neighbors.at(i).y;
      sum.z += neighbors.at(i).z;
    }
    //cout << "Ref is now " << adaptiveReference[0] << endl;
    mean.x = sum.x/k;
    mean.y = sum.y/k;
    mean.z = sum.z/k;

    // Subtract mean from every point
    Matrix X(k, 3);
    for (unsigned int i = 0; i < k; i++) { // Indexing in newmat beginns with 1
      X(i+1, 1) = neighbors.at(i).x - mean.x;
      X(i+1, 2) = neighbors.at(i).y - mean.y;
      X(i+1, 3) = neighbors.at(i).z - mean.z;
    }

    // Do the magic
    SymmetricMatrix A(3);
    A << 1.0/k * X.t() * X;

    DiagonalMatrix D(3);
    EigenValues(A, D);
    //Matrix U(3,3);
    //EigenValues(A, D, U);

    // Add small epsilon
    D(1) += epsilon;
    D(2) += epsilon;
    D(3) += epsilon;
    //cout << "Eigenvalues: " <<endl << D << endl;

    // Normalize eigenvalues
    double eigenSum = D(1) + D(2) + D(3);
    double e1 = D(1)/eigenSum;
    double e2 = D(2)/eigenSum;
    double e3 = D(3)/eigenSum;

    double eigenentropy = -1.*(e1*log(e1)+e2*log(e2)+e3*log(e3));
    //double eigenentropy = -1.*(D(1)*log(D(1))+D(2)*log(D(2))+D(3)*log(D(3)));

    //cout << k << " | " << eigenentropy << endl;

    cout << k << ": " << eigenentropy << " Ref: " << adaptiveReference[0] << ", " << adaptiveReference[1] << ", " << adaptiveReference[2];

    if (eigenentropy < bestEigenentropy) {
      cout << " <-- better";
      bestEigenentropy = eigenentropy;
      kOpt = k;
      bestReference[0] = adaptiveReference[0];
      bestReference[1] = adaptiveReference[1];
      bestReference[2] = adaptiveReference[2];
    }

    cout << endl;
// Set adaptiveReference to mid between original reference point and mean of neighborhood points
    adaptiveReference[0] = (mean.x + reference[0])/2;
    adaptiveReference[1] = (mean.y + reference[1])/2;
    adaptiveReference[2] = (mean.z + reference[2])/2;

  }
  newReference[0] = bestReference[0];
  newReference[1] = bestReference[1];
  newReference[2] = bestReference[2];
  return kOpt;
}


/**
 * Determines optimal neighborhood-size (Weinmann 2013/2014)
 * neighbors contains kMax ordered neighbors
 * kDelta is step size from kMin to kMax
*/
unsigned int determineKOpt(const KDtree &t, double *reference, vector<Point> &neighbors, unsigned int kMin, unsigned int kDelta) {
  //We later need a small epsilon
  double epsilon = std::numeric_limits<double>::epsilon();

  // Determine optimal neighborhood size
  // For all sizes k of neighborhoods with kMin <= k <= kMax, do PCA.
  std::vector<double> eigenentropies;
  unsigned int nextK = 0;
  Point sum(0.0,0.0,0.0);
  Point mean(0.0,0.0,0.0);
  for (unsigned int k = kMin; k <= neighbors.size(); k+=kDelta) {
    // Keep accumulated sum to calculate means - better runtime
    for (unsigned int i = nextK; i < k; i++) {
      sum.x += neighbors.at(i).x;
      sum.y += neighbors.at(i).y;
      sum.z += neighbors.at(i).z;
    }
    nextK = k;
    mean.x = sum.x/k;
    mean.y = sum.y/k;
    mean.z = sum.z/k;

    // Subtract mean from every point
    Matrix X(k, 3);
    for (unsigned int i = 0; i < k; i++) { // Indexing in newmat beginns with 1
      X(i+1, 1) = neighbors.at(i).x - mean.x;
      X(i+1, 2) = neighbors.at(i).y - mean.y;
      X(i+1, 3) = neighbors.at(i).z - mean.z;
    }

    // Do the magic
    SymmetricMatrix A(3);
    A << 1.0/k * X.t() * X;

    DiagonalMatrix D(3);
    EigenValues(A, D);
    //Matrix U(3,3);
    //EigenValues(A, D, U);

    // Add small epsilon
    D(1) += epsilon;
    D(2) += epsilon;
    D(3) += epsilon;
    //cout << "Eigenvalues: " <<endl << D << endl;

    // Normalize eigenvalues
    double eigenSum = D(1) + D(2) + D(3);
    double e1 = D(1)/eigenSum;
    double e2 = D(2)/eigenSum;
    double e3 = D(3)/eigenSum;

    double eigenentropy = -1.*(e1*log(e1)+e2*log(e2)+e3*log(e3));
    //cout << k << " | " << eigenentropy << endl;

    eigenentropies.push_back(eigenentropy);
  }
  // Determine best k (that minimizes eigenentropy)
  unsigned int kOpt = kMin + kDelta * (std::min_element(eigenentropies.begin(), eigenentropies.end()) - eigenentropies.begin());
  //cout << "Smallest eigenentropy with k=" << kOpt <<endl;

  return kOpt;
}


/**
 * Gets neighbors within cylinder with some radius
**/
vector<Point> getCylindricalNeighborhood(const KDtree &t, double *reference, double cylinderMaxDist2, int thread_num) {
  double searchDir[3] = {0.0, 1.0, 0.0}; // vertical cylinder
  return t.fixedRangeSearchAlongDir(reference, searchDir, cylinderMaxDist2, thread_num);
}



/**
 * Generates features for a cylindrical neighborhood.
 * neighbors has a number of neighboring points, ordered, while nr. 0 is next to the reference point
*/
void generateCylindricalFeatures(double features[FEATURE_CYLINDER_VECTOR_SIZE], double *reference, vector<Point> &neighbors) {
  double cylNumberPoints = neighbors.size(), cylMaxHeightDifference, cylHeightVariance = 0, cylHeightAboveMin;
  double cylYMin = DBL_MAX, cylYMax = -DBL_MAX;
  Point cylMean(0.0,0.0,0.0);
  for (unsigned int i=0; i<neighbors.size(); i++) {
    //cylMean.x += neighborsCylinder.at(i).x;
    cylMean.y += neighbors.at(i).y;
    //cylMean.z += neighborsCylinder.at(i).z;

    cylYMin = min(cylYMin, neighbors.at(i).y);
    cylYMax = max(cylYMax, neighbors.at(i).y);
  }

  //cylMean.x /= neighborsCylinder.size();
  cylMean.y /= neighbors.size();
  //cylMean.z /= neighborsCylinder.size();

  cylMaxHeightDifference = cylYMax - cylYMin;
  cylHeightAboveMin = reference[1] - cylYMin;

  for (unsigned int i=0; i<neighbors.size(); i++) {
    cylHeightVariance += sqr(neighbors.at(i).y - cylMean.y); // divide it later by number of points within the
  }
  cylHeightVariance /= neighbors.size();

  features[0]=cylNumberPoints;
  features[1]=cylMaxHeightDifference;
  features[2]=cylHeightVariance;
  features[3]=cylHeightAboveMin;
}


/**
 * Generates features for a neighborhood.
 * neighbors has a number of kMax neighboring points, ordered, while nr. 0 is next to the reference point
*/
void generateNeighborhoodFeatures(double features[FEATURE_VECTOR_SIZE], double *reference, vector<Point> &neighbors, unsigned int kOpt) {

  // Get some stuff we need later
  double epsilon = std::numeric_limits<double>::epsilon();
  //double searchDir[3] = {0.0, 1.0, 0.0}; // vertical cylinder search

  // 3D features (8x eigenvalue-based, 6x further features)

  double linearity, planarity, scatter, omnivariance, anisotropy, eigenentropy, sumOfEigenvalues, changeOfCurvature, verticality, localDensity, radius = 0, maxHeightDifference, heightVariance = 0;
  // 2D features (Projeting points from 3D to horizontally aligned plane)
  double radius2D = 0, localDensity2D, sumOfEigenvalues2D, ratio2D;


  Point mean(0.0,0.0,0.0);
  for (unsigned int i = 0; i < kOpt; i++) {
    mean.x += neighbors.at(i).x;
    mean.y += neighbors.at(i).y;
    mean.z += neighbors.at(i).z;

    double newRadius2D = sqr(neighbors.at(i).x - neighbors.at(0).x) + sqr(neighbors.at(i).z - neighbors.at(0).z);

    radius2D = max(radius2D, newRadius2D);
  }

  // Now take squareroot to save computation time
  radius2D = sqrt(radius2D);

  mean.x /= kOpt;
  mean.y /= kOpt;
  mean.z /= kOpt;

  // Subtract mean from every point and get collect some information for building more features while looping through the neighbors
  Matrix X(kOpt, 3);
  double yMin = DBL_MAX, yMax = -DBL_MAX;
  for (unsigned int i = 0; i < kOpt; i++) { // Indexing in newmat beginns with 1
    X(i+1, 1) = neighbors.at(i).x - mean.x;
    X(i+1, 2) = neighbors.at(i).y - mean.y;
    X(i+1, 3) = neighbors.at(i).z - mean.z;

    heightVariance += sqr(X(i+1, 2)); // divide it later by kOpt

    yMin = min(yMin, neighbors.at(i).y);
    yMax = max(yMax, neighbors.at(i).y);
  }

  heightVariance /= kOpt;
  maxHeightDifference = yMax - yMin;

  // Do the magic in 3D
  SymmetricMatrix A(3);
  A << 1.0/kOpt * X.t() * X; // Covariance matrix
  DiagonalMatrix D(3);
  Matrix U(3,3);
  EigenValues(A, D, U);
  // And now in 2D
  Matrix X2D(kOpt, 2);
  X2D = X.Column(1) | X.Column(3);
  SymmetricMatrix A2D(2);
  A2D << 1.0/kOpt * X2D.t() * X2D;
  DiagonalMatrix D2D(2);
  Matrix U2D(2,2);
  EigenValues(A2D, D2D, U2D);

  // Be sure that eigenvalues are sorted
  // Documentation says its descending... but it was ascending... So we check this here and exit on unexpected behaviour
  //cout.precision(50);
  if (!(D(1)<=D(2) && D(2)<=D(3) && D2D(1)<=D2D(2))) {
    cout << "Assumed ascending order of eigenvalues from newmat. This was not the case! Exiting (Documentation says its descending... but seems to be usually ascending)" << endl;
    exit(-1);
  }

  // Be sure that all are positive!
  ColumnVector lambda(3);
  lambda(1) = max(epsilon, D(3));
  lambda(2) = max(epsilon, D(2));
  lambda(3) = max(epsilon, D(1)); // Smallest eigenvalue

  ColumnVector lambda2D(2);
  lambda2D(1) = max(epsilon, D2D(2));
  lambda2D(2) = max(epsilon, D2D(1));

  //cout << "Eigenvalues" << endl;
  //cout << lambda << endl;
  // Normalize eigenvalues
  sumOfEigenvalues = lambda.Sum(); // This feature is described wrongly in Weinmann 2014 (e1+e2+e3). This is always 1. Weinmann 2013 should be correct.
  ColumnVector e(3);
  e(1) = lambda(1)/sumOfEigenvalues;
  e(2) = lambda(2)/sumOfEigenvalues;
  e(3) = lambda(3)/sumOfEigenvalues;

  sumOfEigenvalues2D = lambda2D.Sum();
  ratio2D = lambda2D(1)/lambda2D(2);


  linearity = (e(1) - e(2))/e(1);
  planarity = (e(2) - e(3))/e(1);
  scatter = e(3)/e(1);
  omnivariance = cbrt(e(1)*e(2)*e(3));
  anisotropy = (e(1) - e(3))/e(1);
  eigenentropy = -1.*(e(1)*log(e(1))+e(2)*log(e(2))+e(3)*log(e(3)));
  changeOfCurvature = e(3);
  //cout << endl;
  //cout << "Matrix: " << endl;
  //cout << A << endl;
  //cout << "e3 (vector belonging to smallest eigenvalue): ";
  //cout << U(1,1) << " " << U(2,1) << " " << U(3,1) << endl;
  //cout << "Eigenvalues: " << e(1) << " " << e(2) << " " << e(3) << endl;
  verticality = 1. - abs(U(2,1)); // Keep in mind that newmat does ascending ordering (by eigenvalue). So (2,1) is second component (Y in UOS - so up/down) of eigenvector belonging to smallest eigenvalue (normal of the plane). Length is already 1
  radius = sqrt(sqr(neighbors[kOpt-1].x - reference[0]) + sqr(neighbors[kOpt-1].y - reference[1]) + sqr(neighbors[kOpt-1].z - reference[2])); // We can do this because neighbors are sorted (kOpt-1 is most far away point)
  localDensity = ((double)kOpt)/((4./3)*M_PI*radius*radius*radius);

  localDensity2D = ((double)kOpt)/(M_PI*sqr(radius2D));

  // Mapping to the feature vector
  // We do it manually as we don't have reflection...
  features[0]=linearity;
  features[1]=planarity;
  features[2]=scatter;
  features[3]=omnivariance;
  features[4]=anisotropy;
  features[5]=eigenentropy;
  features[6]=sumOfEigenvalues;
  features[7]=changeOfCurvature;
  features[8]=verticality;
  features[9]=localDensity;
  features[10]=radius;
  features[11]=maxHeightDifference;
  features[12]=heightVariance;
  features[13]=radius2D;
  features[14]=localDensity2D;
  features[15]=sumOfEigenvalues2D;
  features[16]=ratio2D;
}

/**
* Prints ARFF-Header to ofstream.
* Attention: You need to know the classlabels before
*/
void printArffHeader_new(ofstream& of, const int &nrFeaturesReductions, const int &nrCylinderReductions, set<int> &classLabels, const bool &useAbsHeight) {
  of << "@relation features" << endl
  << endl;
  //<< "@attribute x numeric" << endl
  //<< "@attribute y numeric" << endl
  //<< "@attribute z numeric" << endl
  //<< "@attribute kOpt numeric" << endl
  for (int i = 0; i < nrFeaturesReductions; i++) {
    of  << "@attribute linearity"<< i <<" numeric" << endl
        << "@attribute planarity"<< i <<" numeric" << endl
        << "@attribute scatter"<< i <<" numeric" << endl
        << "@attribute omnivariance"<< i <<" numeric" << endl
        << "@attribute anisotropy"<< i <<" numeric" << endl
        << "@attribute eigenentropy"<< i <<" numeric" << endl
        << "@attribute sumOfEigenvalues"<< i <<" numeric" << endl
        << "@attribute changeOfCurvature"<< i <<" numeric" << endl
        << "@attribute verticality"<< i <<" numeric" << endl
        << "@attribute localDensity"<< i <<" numeric" << endl
        << "@attribute radius"<< i <<" numeric" << endl
        << "@attribute maxHeightDifference"<< i <<" numeric" << endl
        << "@attribute heightVariance"<< i <<" numeric" << endl
        << "@attribute radius2D"<< i <<" numeric" << endl
        << "@attribute localDensity2D"<< i <<" numeric" << endl
        << "@attribute sumOfEigenvalues2D"<< i <<" numeric" << endl
        << "@attribute ratio2D"<< i <<" numeric" << endl;
  }
  for (int i = 0; i < nrCylinderReductions; i++) {
    of  << "@attribute cylNumberPoints"<< i <<" numeric" << endl
        << "@attribute cylMaxHeightDifference"<< i <<" numeric" << endl
        << "@attribute cylHeightVariance"<< i <<" numeric" << endl
        << "@attribute cylHeightAboveMin"<< i <<" numeric" << endl;
  }
  if(useAbsHeight) {
    of  << "@attribute absHeight numeric" << endl;
  }
  of  << "@attribute class {";
  for (auto i:classLabels) {
    of << i << ",";
  }
  if (!classLabels.empty()) {
    of.seekp(-1,of.cur); // Bring write pointer to last-but-one character (before ','). Will be overwritten by '}'.
  }
  of  << "}" << endl
      << endl
      << "@data"
      << endl;
}

void printCsvHeader_new(ofstream& of, const int &nrFeaturesReductions, const int &nrCylinderReductions, const bool &useAbsHeight) {
  for (int i = 0; i < nrFeaturesReductions; i++) {
    of  << "linearity"<< i
        << ",planarity"<< i
        << ",scatter"<< i
        << ",omnivariance"<< i
        << ",anisotropy"<< i
        << ",eigenentropy"<< i
        << ",sumOfEigenvalues"<< i
        << ",changeOfCurvature"<< i
        << ",verticality"<< i
        << ",localDensity"<< i
        << ",radius"<< i
        << ",maxHeightDifference"<< i
        << ",heightVariance"<< i
        << ",radius2D"<< i
        << ",localDensity2D"<< i
        << ",sumOfEigenvalues2D"<< i
        << ",ratio2D"<< i;
  }
  for (int i = 0; i < nrCylinderReductions; i++) {
    of  << ",cylNumberPoints"<< i
        << ",cylMaxHeightDifference"<< i
        << ",cylHeightVariance"<< i
        << ",cylHeightAboveMin"<< i;
  }
  if (useAbsHeight) {
    of  << ",absHeight";
  }
  of  << ",class" << endl;
}


int returnWithErrormessage(int retval, string message) {
  cerr << message << endl;
  cout << "EXITING" << endl;
  return -1;
}

void vector2stringstream(stringstream &stream, double* vec, int size, string delim=",") {
  for (int i = 0; i <  size; i++) {
    if (isfinite(vec[i])) {
      stream << vec[i];
    } else {
      stream << BIG_NUMBER;
#pragma omp critical
      cout << "Attention! Feature " << i << " (in vector of size " << size << ") was not finite. Replaced it with very BIG_NUMBER" << endl;
    }

    if (i+1 < size) {
      stream << delim;
    }
  }
}


/**
 * Main program for scan2feature.
 * Usage: bin/scan2features -f xyzc 'dir',
 * with 'dir' the directory of a set of scans
 */
int main(int argc, char **argv)
{
  signal (SIGSEGV, sigSEGVhandler);
  signal (SIGINT,  sigSEGVhandler);

  cout << "scan2features - "
       << "This program extracts features of 3D scans."
       << "Attention: Features are only calculated between points within one single scan. You may want to merge the point clouds before."
       << endl
       << "created by Michael Neumann" << endl << endl;

  // init, default values if not specified
  string dir;
  int    start = 0,   end = -1;
  int    jobs = 1, job = 0;
  int    algorithm = 0;
  double ref1 = 0;
  double ref2 = 0;
  double ref3 = 0;
  IOType type    = UOS;

  bool csv = false;

  int reduction = 10; // Reduction for reference points (used to calculate features for)
  vector<int> ignoredClasses; // Ignore all points with this class (they are still used for computation of features for other points)
  // The following variables describe how features are calculated (on a kreduced tree, from kMin to kMax with stepsize kDelta). At the end calculate features for different cylinders
  bool useDistance = false; // Currently unused - Future work. Use spherical neighborhoods instead of KNN
  vector<int> kReductions = {10};
  vector<int> kMins = {10};
  vector<int> kMaxs = {100};
  vector<int> kDeltas = {1};
  vector<int> cylReductions = {10};
  vector<int> cylRadii = {25};


  // fixed parameters - only changed in code
  bool scanserver = false;
  bool quiet      = false;
  bool veryQuiet  = false;
  bool debug    = false;
  bool useAbsHeight = false; // When true, value of Y is also part of the feature set.

  // now parse command line parameters
  parse_options(argc, argv, dir, type, start, end, jobs, job, algorithm, ref1, ref2, ref3, reduction, ignoredClasses, useDistance, kReductions, kMins, kMaxs, kDeltas, cylReductions, cylRadii, csv,
            quiet, veryQuiet);

  cout << "Starting..." << endl;

  //if (continue_processing) Scan::continueProcessing();
  //Scan::setProcessingCommand(argc, argv);

  // For better performance keeps squares
  vector<int> cylRadiiSqr;
  for (unsigned int i=0; i<cylRadii.size(); i++) {
    cylRadiiSqr.push_back(sqr(cylRadii[i]));
  }

  if (algorithm == 99) { //For debugging
    cout << "Entering Debug-Mode" << endl;
    cout << "mins are: " << endl;
    for (unsigned int i=0; i<kMins.size(); i++) {
      cout << kMins[i] << endl;
    }
    return 0;
  }
  else {
    Scan::openDirectory(scanserver, dir, type, start, end);
    string fileAppendix = "";
    string fileEnding = "";

    // Lets first do some checks
    if (kMins.size() != kMaxs.size() || kMins.size() != kDeltas.size() || kMins.size() != kReductions.size()) {
      return returnWithErrormessage(-1, "kMins, kMax, kDeltas, and kReductions need to be of same length!");
    }
    if (cylReductions.size() != cylRadii.size()) {
      return returnWithErrormessage(-1, "cylReductions and cylRadii need to be of same length!");
    }
    for (unsigned int i=0; i<kMins.size(); i++) {
      if (kMins[i]>kMaxs[i]) {
        return returnWithErrormessage(-1, "kMins was bigger than kMax");
      }
    }
    if(Scan::allScans.size() == 0) {
      return returnWithErrormessage(-1, "No scans found. Did you use the correct format?");
    }


    cout << "Reading files from nr. " << start << " to nr. " << end << " (found " << Scan::allScans.size() << " scans)" << endl;
    unsigned int types = PointType::USE_NONE;
    if(supportsType(type)) types |= PointType::USE_TYPE;
    PointType pointtype(types);

    bool use_type = false;
    if(supportsType(type)) use_type = true;


    if (jobs > 1) {
      fileAppendix += ("-" + to_string(job) + "of" + to_string(jobs));
    }
    if (csv) {
      fileEnding = ".csv";
    }
    else {
      fileEnding = ".arff";
    }

    // Build all the reduced trees
    vector<int> neededReductions;
    neededReductions.reserve(1+kReductions.size()+cylReductions.size()); //+1 for reduction itself
    neededReductions.push_back(reduction);
    neededReductions.insert(neededReductions.end(), kReductions.begin(), kReductions.end());
    neededReductions.insert(neededReductions.end(), cylReductions.begin(), cylReductions.end());
    // Remove duplicates
    sort(neededReductions.begin(), neededReductions.end());
    neededReductions.erase( unique( neededReductions.begin(), neededReductions.end() ), neededReductions.end() );


    vector <map <int, vector<double*>>> reducedDataXYZ;
    vector <map <int, vector<int>>> reducedDataTypes;

    //int debug_nr = 0;

    for(ScanVector::iterator it = Scan::allScans.begin();
          it != Scan::allScans.end();
          ++it) {

      map <int, KDtree*> kdTreesMap; // NEW

      Scan* scan = *it;

      map <int, vector<double*>> reducedDataXYZMap;
      map <int, vector<int>> reducedDataTypesMap;
      for(unsigned int i = 0; i < neededReductions.size(); i++) {
        vector<double*> dataXYZ;
        DataType dataType;
        string reducedString = "";
        if (neededReductions[i]!=0) {
          reducedString = " reduced";
          scan->setReductionParameter(neededReductions[i], 1, pointtype); // nrpts: -1: take average of points in voxel (problem with class labels), 0: Use centroid each, 1: 1 random point per voxel
          scan->calcReducedPoints();
        }
        DataXYZ xyz(scan->get("xyz"+reducedString)); //just to know the size...

        if (use_type) {
          DataType dataType = (((DataType)scan->get("type" + reducedString)).size() == 0) ?
              scan->create("type" + reducedString, sizeof(int)*xyz.size()) :
              scan->get("type" + reducedString);
          vector<int> typesOfThisReduction;
          //typesOfThisReduction.reserve(dataType.size());
          ofstream debug_out;
          //debug_out.open("scan00"+to_string(debug_nr)+"_classes.3d");
          for (unsigned int typeId = 0; typeId < dataType.size(); typeId++) {
            typesOfThisReduction.push_back(dataType[typeId]);
            //debug_out << dataType[typeId] << endl;
            /*if (dataType[typeId]<0 || dataType[typeId]>8) {
              cout << "ERROR! Type was " << dataType[typeId] << endl;
            }*/
          }
          //debug_out.close();
          reducedDataTypesMap.insert({neededReductions[i],typesOfThisReduction});


          xyz = scan->get("xyz"+reducedString); // Get it again. Maybe order changed...
        }

        cout << "Scan " << scan->scanNr << ": Size of reduced " << neededReductions[i] << " pointcloud is " << xyz.size() << endl;

        // NEW
        //dataXYZ.reserve(xyz.size());
        //double max=0;
        for(unsigned int j = 0; j < xyz.size(); j++) {
          dataXYZ.push_back(new double[3]{xyz[j][0], xyz[j][1], xyz[j][2]});
        }
        reducedDataXYZMap.insert({neededReductions[i],dataXYZ});


        //ofstream debug_out;
        //debug_out.open("scan00"+to_string(debug_nr)+"_points.3d");
        //debug_nr++;
        /*for (unsigned int deb=0; deb<dataXYZ.size();deb++) {
          debug_out << dataXYZ.at(deb)[0] << " " << dataXYZ.at(deb)[1] << " " << dataXYZ.at(deb)[2] << " " << endl;
        }*/
        //debug_out.close();


      }

      reducedDataXYZ.push_back(reducedDataXYZMap);
      if (use_type) reducedDataTypes.push_back(reducedDataTypesMap);
    }

    // Build set of all used types/classes in all scans
    cout << "Building set of used types/classes..." << endl;
    set<int> usedTypes;
    usedTypes.insert(0);
    if (use_type) {
      for(unsigned int scanIndex = 0; scanIndex < Scan::allScans.size(); scanIndex++) {
        for(unsigned int i = 0; i < neededReductions.size(); i++) {
          vector<int>* types = &(reducedDataTypes.at(scanIndex).find(neededReductions[i])->second);
          for (unsigned int typeId=0; typeId<types->size(); typeId++) {
            usedTypes.insert(types->at(typeId));
          }
        }
      }
    }
    cout << "Found the following classes/types in all scans:" << endl;
    for (int i:usedTypes) {
      cout << i << endl;
    }

    vector <map <int, KDtree*>> reducedKDtrees;
    for(unsigned int scanIndex = 0; scanIndex < Scan::allScans.size(); scanIndex++) {
      map <int, KDtree*> kdTreesMap;
      for(unsigned int i = 0; i < neededReductions.size(); i++) {
        vector<double*> pointCopy(reducedDataXYZ.at(scanIndex).find(neededReductions[i])->second); // We need to work with a copy of the data (at least double* copy)...
        KDtree* t = new KDtree(pointCopy.data(), pointCopy.size()); // Build tree

        kdTreesMap.insert({neededReductions[i], t});
      }
      reducedKDtrees.push_back(kdTreesMap);
    }

    ofstream featuresOut;
    ofstream xyzOut;
    ofstream kOptOut;
    featuresOut.open("features" + fileAppendix + fileEnding);
    featuresOut << std::setprecision(3);
    xyzOut.open("features_points" + fileAppendix + ".3d");
    kOptOut.open("features_kOpt" + fileAppendix + ".txt");

    if (csv) {
      printCsvHeader_new(featuresOut, kReductions.size(), cylReductions.size(), useAbsHeight);
    }
    else {
      printArffHeader_new(featuresOut, kReductions.size(), cylReductions.size(), usedTypes, useAbsHeight);
    }

    // For each scan
    for(unsigned int scanIndex = 0; scanIndex < Scan::allScans.size(); scanIndex++) {
      cout << "Processing scan nr. " << scanIndex << endl;
      //std::map<int,DataXYZ>::iterator itPoints;
      //itPoints = reducedPoints.at(scanIndex).find(reduction);

      vector<double*>* referencePoints = &(reducedDataXYZ.at(scanIndex).find(reduction)->second);
      vector<int>* referenceDataTypes;
      if (use_type) {
        referenceDataTypes = &(reducedDataTypes.at(scanIndex).find(reduction)->second);
      }
      cout << "Number of reference points: " << referencePoints->size() << endl;
#ifdef _OPENMP
      omp_set_num_threads(OPENMP_NUM_THREADS);
#pragma omp parallel for schedule(guided, 100)
#endif
      // For each point
      for(unsigned int referenceId = 0; referenceId < referencePoints->size(); referenceId++) {
#ifdef _OPENMP
          int thread_num = omp_get_thread_num();
#else
          int thread_num = 0;
#endif
        if (referenceId % 100000 == 0) {
#pragma omp critical
          cout << "Calculating features for point nr. " << scanIndex << "." << referenceId << " (by thread " << thread_num << ")" << endl;
        }
        stringstream features_out;
        stringstream kOpt_out;
        stringstream xyz_out;
        vector<Point> neighbors;
        double features[FEATURE_VECTOR_SIZE];
        double cylFeatures[FEATURE_CYLINDER_VECTOR_SIZE];
        int classification;
        double reference[3];
        classification = use_type? referenceDataTypes->at(referenceId) : 0;
        if (find(ignoredClasses.begin(), ignoredClasses.end(), classification) == ignoredClasses.end()) { // Point is not ignored!
          // Clear stringstreams for this point
          features_out.str(std::string());
          kOpt_out.str(std::string());
          xyz_out.str(std::string());

          //cout << "test0" << endl;

          reference[0] = referencePoints->at(referenceId)[0];
          reference[1] = referencePoints->at(referenceId)[1];
          reference[2] = referencePoints->at(referenceId)[2];

          //cout << "test1" << endl;

          //cout << "Handling " << reference[0] << " - " << reference[1] << " - " << reference[2] << endl;
          //std::map<int,KDtree*>::iterator reductionStepIterator;
          vector2stringstream(xyz_out, reference, 3, " ");
          xyz_out << " " << classification << endl;
          //cout << "test2" << endl;
          // For each kReduction
          for(unsigned int reductionStep = 0; reductionStep < kReductions.size(); reductionStep++) {
            KDtree *t = reducedKDtrees.at(scanIndex).find(kReductions[reductionStep])->second;
            //reductionStepIterator = reducedKDtrees.at(scanIndex).find(kReductions[reductionStep]);
            neighbors = t->kNearestNeighbors(reference, kMaxs[reductionStep], thread_num);
            unsigned int kOpt = determineKOpt(*t, reference, neighbors, kMins[reductionStep], kDeltas[reductionStep]);
            kOpt_out << kOpt << " ";
            generateNeighborhoodFeatures(features, reference, neighbors, kOpt);
            vector2stringstream(features_out, features, FEATURE_VECTOR_SIZE);
            features_out << ",";
          }
          //cout << "test3" << endl;
          kOpt_out << endl;
          for(unsigned int reductionStep = 0; reductionStep < cylReductions.size(); reductionStep++) {
            //reductionStepIterator = reducedKDtrees.at(scanIndex).find(cylReductions[reductionStep]);
            KDtree *t = reducedKDtrees.at(scanIndex).find(cylReductions[reductionStep])->second;
            neighbors = getCylindricalNeighborhood(*t, reference, cylRadiiSqr[reductionStep], thread_num);
            //cout << "Size of cyl neighbors is " << neighbors.size() << endl;
            generateCylindricalFeatures(cylFeatures, reference, neighbors);
            vector2stringstream(features_out, cylFeatures, FEATURE_CYLINDER_VECTOR_SIZE);
            features_out << ",";
          }
          if (useAbsHeight) {
            features_out << reference[1] << ",";
          }
          features_out << classification << endl; // Height and class label
// To keep order of points consistent between all files we have to use critical section
#pragma omp critical
          {
            featuresOut << features_out.rdbuf();
            kOptOut << kOpt_out.rdbuf();
            xyzOut << xyz_out.rdbuf();
          }
        }

      }
    }
    featuresOut.close();
    kOptOut.close();
    xyzOut.close();

/*    for(unsigned int scanIndex = 0; scanIndex < Scan::allScans.size(); scanIndex++) {
      for(unsigned int reductionStep = 0; reductionStep < neededReductions.size(); reductionStep++) {
        cout << "Last Element is " << reducedDataXYZ.at(scanIndex).find(neededReductions[reductionStep])->second.at(reducedDataXYZ.at(scanIndex).find(neededReductions[reductionStep])->second.size()-1)[0] << endl;
        cout << "Type was " << reducedDataTypes.at(scanIndex).find(neededReductions[reductionStep])->second.at(reducedDataTypes.at(scanIndex).find(neededReductions[reductionStep])->second.size()-1) << endl;
      }
    }*/
  }
}
