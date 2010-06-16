/**
 * @file
 * @brief Implementation of a 3D scan and of 3D scan matching in all variants
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifdef _MSC_VER
#ifdef OPENMP
#define _OPENMP
#endif
#endif

#include <fstream>
using std::ifstream;
using std::ofstream;

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#include <sstream>
using std::stringstream;

#include "scan.h"
#include "Boctree.h"
#include "scan_io.h"
#include "d2tree.h"
#include "kd.h"
#include "kdc.h"

#ifdef _OPENMP
#include <omp.h>
#endif

#ifdef _MSC_VER
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#include <cstring>
using std::flush;

vector <Scan *>  Scan::allScans;
unsigned int     Scan::numberOfScans = 0;
bool             Scan::outputFrames = false;
string           Scan::dir;
vector<KDCache*> Scan::closest_cache;

/**
 * default Constructor
 */
Scan::Scan()
{
  kd = 0;
  scanNr = fileNr = 0;
  rPosOrg[0] = rPos[0] = 0;
  rPosOrg[1] = rPos[1] = 0;
  rPosOrg[2] = rPos[2] = 0;
  rPosThetaOrg[0] = rPosTheta[0] = 0;
  rPosThetaOrg[1] = rPosTheta[1] = 0;
  rPosThetaOrg[2] = rPosTheta[2] = 0;
  M4identity(transMat);
  M4identity(dalignxf);

  points_red_size = 0;
  points_red = points_red_lum = 0; 
}


/**
 * Constructor
 * @param *euler 6D pose: estimation of the scan location, e.g. based on odometry
 * @param maxDist Regard only points up to an (Euclidean) distance of maxDist
 * transformation matrices when match (default: false)
 */
Scan::Scan(const double* euler, int maxDist)
{
  kd = 0;
  maxDist2 = (maxDist != -1 ? sqr(maxDist) : maxDist);
  
  if (dir == "") {
    cerr << "ERROR: Directory has to be set before!" << endl;
    exit(1);
  }

  if (euler == 0) {
    rPosOrg[0] = rPos[0] = 0;
    rPosOrg[1] = rPos[1] = 0;
    rPosOrg[2] = rPos[2] = 0;
    rPosThetaOrg[0] = rPosTheta[0] = 0;
    rPosThetaOrg[1] = rPosTheta[1] = 0;
    rPosThetaOrg[2] = rPosTheta[2] = 0;
    M4identity(transMat);
  } else {
    rPosOrg[0] = rPos[0] = euler[0];
    rPosOrg[1] = rPos[1] = euler[1];
    rPosOrg[2] = rPos[2] = euler[2];
    rPosThetaOrg[0] = rPosTheta[0] = euler[3];
    rPosThetaOrg[1] = rPosTheta[1] = euler[4];
    rPosThetaOrg[2] = rPosTheta[2] = euler[5];
    EulerToMatrix4(rPos, rPosTheta, transMat);
  }

  fileNr = 0;
  scanNr = numberOfScans++;
  
  points_red_size = 0;
  points_red = points_red_lum = 0; 
  M4identity(dalignxf);
}

/**
 * Constructor
 * @param _rPos[3] 3D position: estimation of the scan location, e.g. based on odometry
 * @param _rPosTheta[3] 3D orientation: estimation of the scan location, e.g. based on odometry
 * @param maxDist Regard only points up to an (Euclidean) distance of maxDist
 */
Scan::Scan(const double _rPos[3], const double _rPosTheta[3], const int maxDist)
{
  kd = 0;
  maxDist2 = (maxDist != -1 ? sqr(maxDist) : maxDist);
  rPosOrg[0] = rPos[0] = _rPos[0];
  rPosOrg[1] = rPos[1] = _rPos[1];
  rPosOrg[2] = rPos[2] = _rPos[2];
  rPosThetaOrg[0] = rPosTheta[0] = _rPosTheta[0];
  rPosThetaOrg[1] = rPosTheta[1] = _rPosTheta[1];
  rPosThetaOrg[2] = rPosTheta[2] = _rPosTheta[2];
  EulerToMatrix4(rPos, rPosTheta, transMat);

  fileNr = 0;
  scanNr = numberOfScans++;
  
  points_red_size = 0;
  M4identity(dalignxf);
}

/**
 * Constructor for creating a metascan from a list of scans
 * It joins all the points and contructs a new search tree
 * and reinitializes the cache, iff needed
 *
 * @param MetaScan Vector that contains the 3D scans
 * @param use_cache Indicates if cached versions of the search tree has to be build
 */
Scan::Scan(const vector < Scan* >& MetaScan, bool use_cache)
{
  kd = 0;
  scanNr = numberOfScans++;
  rPosOrg[0] = rPos[0] = 0;
  rPosOrg[1] = rPos[1] = 0;
  rPosOrg[2] = rPos[2] = 0;
  rPosThetaOrg[0] = rPosTheta[0] = 0;
  rPosThetaOrg[1] = rPosTheta[1] = 0;
  rPosThetaOrg[2] = rPosTheta[2] = 0;
  M4identity(transMat);
  M4identity(dalignxf);

  // copy points
  int numpts = 0;
  int end_loop = (int)MetaScan.size();
  for (int i = 0; i < end_loop; i++) {
    numpts += MetaScan[i]->points_red_size;
  }
  points_red_size = numpts;
  points_red = new double*[numpts];  
  int k = 0;
  for (int i = 0; i < end_loop; i++) {
    for (int j = 0; j < MetaScan[i]->points_red_size; j++) {
	 points_red[k] = new double[3];
	 points_red[k][0] = MetaScan[i]->points_red[j][0];
	 points_red[k][1] = MetaScan[i]->points_red[j][1];
	 points_red[k][2] = MetaScan[i]->points_red[j][2];
	 k++;
    }
  }

  fileNr = -1; // no need to store something from a meta scan!
  scanNr = numberOfScans++;

  // build new search tree
  createTree(use_cache);

  // add Scan to ScanList
  allScans.push_back(this);

  meta_parts = MetaScan;
}

  
/**
 * Desctuctor
 */
Scan::~Scan()
{
  if (outputFrames && fileNr != -1) {
    string filename = dir + "scan" + to_string(fileNr, 3) + ".frames";
    
    ofstream fout(filename.c_str());
    if (!fout.good()) {
	 cerr << "ERROR: Cannot open file " << filename << endl;
	 exit(1);
    }

    // write into file
    fout << sout.str();

    fout.close();
    fout.clear();
  }

  if (this->kd != 0) deleteTree();

  // delete entries in cache
  vector <KDCache*>::iterator Iter1;
  for(Iter1 = closest_cache.begin(); Iter1 != closest_cache.end();) {
    if (((*Iter1)->SourceScanNr == scanNr) ||
	   ((*Iter1)->TargetScanNr == scanNr)) {
	 delete [] (*Iter1)->item;
	 delete *Iter1;
	 closest_cache.erase(Iter1);
    } else {
	 Iter1++;
    }
  }

  // delete Scan from ScanList
  vector <Scan*>::iterator Iter;
  for(Iter = allScans.begin(); Iter != allScans.end();) {
    if (*Iter == this) {
	 allScans.erase(Iter);
	 break;
    } else {
	 Iter++;
    }
  }

  for (int i = 0; i < points_red_size; i++) {
    delete [] points_red[i];
  }
  delete [] points_red;
  
  points.clear();
}

/**
 * Copy constructor
 */
Scan::Scan(const Scan& s)
{
  rPos[0] = s.rPos[0];
  rPos[1] = s.rPos[1];
  rPos[2] = s.rPos[2];
  rPosTheta[0] = s.rPosTheta[0];
  rPosTheta[1] = s.rPosTheta[1];
  rPosTheta[2] = s.rPosTheta[2];
  rPosOrg[0]   = s.rPosOrg[0];
  rPosOrg[1]   = s.rPosOrg[1];
  rPosOrg[2]   = s.rPosOrg[2];
  rPosThetaOrg[0] = s.rPosThetaOrg[0];
  rPosThetaOrg[1] = s.rPosThetaOrg[1];
  rPosThetaOrg[2] = s.rPosThetaOrg[2];

  memcpy(transMat, s.transMat, sizeof(transMat));

  // copy data points
  for (unsigned int i = 0; i < s.points.size(); points.push_back(s.points[i++]));    
  // copy reduced data
  points_red_size = s.points_red_size;
  points_red = new double*[points_red_size];
  for (int i = 0; i < points_red_size; i++) {
    points_red[i] = new double[3];
    points_red[i][0] = s.points_red[i][0];
    points_red[i][1] = s.points_red[i][1];
    points_red[i][2] = s.points_red[i][2];
  }
  memcpy(dalignxf, s.dalignxf, sizeof(dalignxf));
  if (s.kd != 0) {
    createTree(false);
  }

  scanNr = s.scanNr;
  sout << s.sout.str();
  maxDist2 = s.maxDist2;
}

/**
 * Merges the scan's intrinsic coordinates with the robot position.
 * @param prev_transMat The previous transformation matrix
 * @param prev_rPosOrg The previous position (x,y,z)
 * @param prev_rPosThetaOrg The previous heading (theta_x, theta_y, theta_z)
 */
void Scan::mergeCoordinatesWithRoboterPosition(const double prev_transMat[16],
									  const double prev_rPosOrg[3],
									  const double prev_rPosThetaOrg[3])
{
  double drPos[3], drPosTheta[3];
  // reset values  
  rPos[0] = 0;
  rPos[1] = 0;
  rPos[2] = 0;
  rPosTheta[0] = 0;
  rPosTheta[1] = 0;
  rPosTheta[2] = 0;
  M4identity(transMat);

  drPos[0] = rPosOrg[0] - prev_rPosOrg[0];
  drPos[1] = rPosOrg[1] - prev_rPosOrg[1];
  drPos[2] = rPosOrg[2] - prev_rPosOrg[2];
  
  drPosTheta[0] = rPosThetaOrg[0] - prev_rPosThetaOrg[0];
  drPosTheta[1] = rPosThetaOrg[1] - prev_rPosThetaOrg[1];
  drPosTheta[2] = rPosThetaOrg[2] - prev_rPosThetaOrg[2];

  double tempR[9], Rinv[9];
  EulerToMatrix3(prev_rPosThetaOrg, tempR); // convert previous rPosTheta
                                            // to a rotation matrix tempR
  M3inv(tempR, Rinv);                       // ... and invert it ...

  // use old drPos and the inverted rotation matrix to get the new rPos
  // that is the real delta
  double drPosNeu[3];
  drPosNeu[0] = drPos[0] * Rinv[0] + drPos[1] * Rinv[3] + drPos[2] * Rinv[6];
  drPosNeu[1] = drPos[0] * Rinv[1] + drPos[1] * Rinv[4] + drPos[2] * Rinv[7];
  drPosNeu[2] = drPos[0] * Rinv[2] + drPos[1] * Rinv[5] + drPos[2] * Rinv[8];
  drPos[0] = drPosNeu[0];
  drPos[1] = drPosNeu[1];
  drPos[2] = drPosNeu[2];

  // convert the deltas to a matrix
  double alignxf[16];
  EulerToMatrix4(drPos, drPosTheta, alignxf); 

  // and transform the 3D scan  
  transform(alignxf, INVALID);
  
  // and finally apply the transformation matrix of the last 3D scan
  transform(prev_transMat, INVALID);
}

/**
 * Merges the scan's intrinsic coordinates with the robot position.
 * @param prevScan The scan that's transformation is extrapolated,
 * i.e., odometry extrapolation
 *
 * For additional information see the follwoing paper (jfr2007.pdf):
 *
 * Andreas Nüchter, Kai Lingemann, Joachim Hertzberg, and Hartmut Surmann,
 * 6D SLAM - 3D Mapping Outdoor Environments Journal of Field Robotics (JFR),
 * Special Issue on Quantitative Performance Evaluation of Robotic and Intelligent
 * Systems, Wiley & Son, ISSN 1556-4959, Volume 24, Issue 8-9, pages 699 - 722,
 * August/September, 2007
 *
 */
void Scan::mergeCoordinatesWithRoboterPosition(const Scan* prevScan)
{
  mergeCoordinatesWithRoboterPosition(prevScan->transMat, 
				      prevScan->rPosOrg,
				      prevScan->rPosThetaOrg);
}


/**
 * Wrapper function, setting the vectors to (0, 0, 0)
 * applicable to first scan!!!
 */
void Scan::mergeCoordinatesWithRoboterPosition()
{
  double last_transMat[16];
  double last_rPosOrg[3];
  double last_rPosThetaOrg[3];
  M4identity(last_transMat);
  last_rPosOrg[0] = last_rPosOrg[1] = last_rPosOrg[2] = 0.0; 
  last_rPosThetaOrg[0] = last_rPosThetaOrg[1] = last_rPosThetaOrg[2] = 0.0;
  mergeCoordinatesWithRoboterPosition(last_transMat,
				      last_rPosOrg, 
				      last_rPosThetaOrg);
}

/**
 * The method transforms all points (and only the points)
 * with the given transformationmatrix.
 * Unlike transform, points_red, colourmatrixes etc. are
 * not touched!
 * @param alignxf The transformationmatrix
 */ 
void Scan::transformAll(const double alignxf[16])
{
  int end_loop = (int)points.size();
  
#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (int i = 0; i < end_loop; ++i)
    {
	 
	 points[i].transform(alignxf);
    }
}

/**
 * Transforms the scan by a given transformation and writes a new frame. The idea 
 * is to write for every transformation in all files, such that the show program
 * is able to determine, whcih scans have to be drawn in which color. Hidden scans
 * (or later processed scans) are written with INVALID.
 *
 * @param alignxf Transformation matrix
 * @param colour Specifies which colour should the written to the frames file
 * @param islum Is the transformtion part of LUM, i.e., all scans are transformed?
 *        In this case only LUM transformation is stored, otherwise all scans are processed
 *        -1  no transformation is stored
 *         0  ICP transformation
 *         1  LUM transformation, all scans except last scan
 *         2  LUM transformation, last scan only
 */
void Scan::transform(const double alignxf[16], const AlgoType type, int islum)
{
  int end_meta = (int)meta_parts.size();
  for(int i = 0; i < end_meta; i++) {
    meta_parts[i]->transform(alignxf, type, -1);
  }
  int end_loop = (int)points.size();
#ifdef TRANSFORM_ALL_POINTS
  /*
   * We dont't need to transform all points, since we use the array
   * points_red all the time.
   *
   * We do need of course the transformation of the points_red array, since
   * thats a moving point cloud. The get ptPairs methods do _not_ consider
   * transformation of target points
   */   
#ifdef _OPENMP
#pragma omp parallel for
#endif
   for (int i = 0; i < end_loop; i++) {
     points[i].transform(alignxf);
   }
#endif
   
#ifdef DEBUG  
  cerr << alignxf << endl;
  cerr << "(" << rPos[0] << ", " << rPos[1] << ", " << rPos[2] << ", "
	  << rPosTheta[0] << ", " << rPosTheta[1] << ", " << rPosTheta[2] << ") ---> ";
#endif

#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (int i = 0; i < points_red_size; i++) {
    double x_neu, y_neu, z_neu;
    x_neu = points_red[i][0] * alignxf[0] + points_red[i][1] * alignxf[4] + points_red[i][2] * alignxf[8];
    y_neu = points_red[i][0] * alignxf[1] + points_red[i][1] * alignxf[5] + points_red[i][2] * alignxf[9];
    z_neu = points_red[i][0] * alignxf[2] + points_red[i][1] * alignxf[6] + points_red[i][2] * alignxf[10];
    points_red[i][0] = x_neu + alignxf[12];
    points_red[i][1] = y_neu + alignxf[13];
    points_red[i][2] = z_neu + alignxf[14];
  }
  
  double tempxf[16];
  MMult(alignxf, transMat, tempxf);
  memcpy(transMat, tempxf, sizeof(transMat));
  Matrix4ToEuler(transMat, rPosTheta, rPos);
  Matrix4ToQuat(transMat, rQuat);
  
#ifdef DEBUG  
  cerr << "(" << rPos[0] << ", " << rPos[1] << ", " << rPos[2] << ", "
	  << rPosTheta[0] << ", " << rPosTheta[1] << ", " << rPosTheta[2] << ")" << endl;

  cerr << transMat << endl;
#endif

  MMult(alignxf, dalignxf, tempxf);
  memcpy(dalignxf, tempxf, sizeof(transMat));

  // store transformation
  bool in_meta;
  int  found = 0;
  if (type != INVALID) {

    switch (islum) {
    case -1:
	 // write no tranformation
	 break;
    case 0:
	 end_loop = (int)allScans.size();
	 for (int iter = 0; iter < end_loop; iter++) {
	   in_meta = false;
	   for(int i = 0; i < end_meta; i++) {
		if(meta_parts[i] == allScans[iter]) {
		  found = iter;
		  in_meta = true;
		}
	   }
	   if (allScans[iter]->sout.good()) {
		allScans[iter]->sout << allScans[iter]->transMat;
		if (allScans[iter] == this || in_meta) {
		  found = iter;
		  allScans[iter]->sout << type << endl;
		} else {
		  if (found == 0) {
		    allScans[iter]->sout << ICPINACTIVE << endl;
		  } else {
		    allScans[iter]->sout << INVALID << endl;
		  }
		}
	   } else {
		cerr << "ERROR: Cannot store frames." << endl;
		exit(1);
	   }
	 }
	 break;
    case 1:
	 if (sout.good()) {
	   sout << transMat << type << endl;
	 } else {
	   cerr << "ERROR: Cannot store frames." << endl;
	   exit(1);
	 }
	 break;
    case 2:
	 end_loop = (int)allScans.size();
	  for (int iter = 0; iter < end_loop; iter++) {
	   if (allScans[iter] == this) {
		found = iter;
		if (sout.good()) {
		  sout << transMat << type << endl;
		} else {
		  cerr << "ERROR: Cannot store frames." << endl;
		  exit(1);
		}
		
		if (allScans[0]->sout.good()) {
		  allScans[0]->sout << allScans[0]->transMat << type << endl;
		} else {
		  cerr << "ERROR: Cannot store frames." << endl;
		  exit(1);
		}
		continue;
	   }
	   if (found != 0) {
		allScans[iter]->sout << allScans[iter]->transMat << INVALID << endl;
	   }
	 }
	 break;
    default:
	 cerr << "invalid point transformation mode" << endl;
    }
  }
}


/**
 * Transforms the scan by a given transformation and writes a new frame. The idea 
 * is to write for every transformation in all files, such that the show program
 * is able to determine, whcih scans have to be drawn in which color. Hidden scans
 * (or later processed scans) are written with INVALID.
 *
 * @param alignQuat Quaternion for the rotation
 * @param alignt    Translation vector
 * @param colour Specifies which colour should the written to the frames file
 * @param islum Is the transformtion part of LUM, i.e., all scans are transformed?
 *        In this case only LUM transformation is stored, otherwise all scans are processed
 *        -1  no transformation is stored
 *         0  ICP transformation
 *         1  LUM transformation, all scans except last scan
 *         2  LUM transformation, last scan only
 */
void Scan::transform(const double alignQuat[4], const double alignt[3],
				 const AlgoType type, int islum)
{

  double alignxf[16];
  QuatToMatrix4(alignQuat, alignt, alignxf);
  transform(alignxf, type, islum);
}


/**
 * Transforms the scan, so that the given Matrix
 * prepresent the next pose.
 *
 * @param alignxf Transformation matrix to which this scan will be set to
 * @param islum Is the transformation part of LUM?
 */
void Scan::transformToMatrix(double alignxf[16], const AlgoType type, int islum)
{
  double tinv[16];
  M4inv(transMat, tinv);
  transform(tinv, INVALID);
  transform(alignxf, type, islum);
}

/**
 * Transforms the scan, so that the given Euler angles
 * prepresent the next pose.
 *
 * @param rP Translation to which this scan will be set to
 * @param rPT Orientation as Euler angle to which this scan will be set
 * @param islum Is the transformation part of LUM?
 */
void Scan::transformToEuler(double rP[3], double rPT[3], const AlgoType type, int islum)
{
  double tinv[16];
  double alignxf[16];
  M4inv(transMat, tinv);
  transform(tinv, INVALID);
  EulerToMatrix4(rP, rPT, alignxf);
  transform(alignxf, type, islum);
}

/**
 * Transforms the scan, so that the given Euler angles
 * prepresent the next pose.
 *
 * @param rP Translation to which this scan will be set to
 * @param rPQ Orientation as Quaternion to which this scan will be set
 * @param islum Is the transformation part of LUM?
 */
void Scan::transformToQuat(double rP[3], double rPQ[4], const AlgoType type, int islum)
{
  double tinv[16];
  double alignxf[16];
  M4inv(transMat, tinv);
  transform(tinv, INVALID);
  QuatToMatrix4(rPQ, rP, alignxf);
  transform(alignxf, type, islum);
}


/**
 * Computes an octtree of the current scan, then getting the
 * reduced points as the centers of the octree voxels.
 * @param voxelSize The maximal size of each voxel
 */
void Scan::calcReducedPoints(double voxelSize, int nrpts)
{
  // no reduction needed
  // copy vector of points to array of points to avoid
  // further copying
  if (voxelSize <= 0.0) {
    points_red = new double*[points.size()];

    int end_loop = points_red_size = (int)points.size();
    for (int i = 0; i < end_loop; i++) {
	  points_red[i] = new double[3];
	  points_red[i][0] = points[i].x;
	  points_red[i][1] = points[i].y;
	  points_red[i][2] = points[i].z;
    }
    return;
  }

  // start reduction
  
  // build octree-tree from CurrentScan
  double **ptsOct = 0;
  ptsOct = new double*[points.size()];

  int num_pts = 0;
  int end_loop = (int)points.size();
  for (int i = 0; i < end_loop; i++) {
    ptsOct[num_pts] = new double[3];
    ptsOct[num_pts][0] = points[i].x;
    ptsOct[num_pts][1] = points[i].y;
    ptsOct[num_pts][2] = points[i].z;
    num_pts++;
  }

  BOctTree *oct = new BOctTree(ptsOct, num_pts, voxelSize);

  //  OctTree *oct   // get points from the scan's octree
  // & store them in the points_red vector as the reduced version of the scan
  vector<double*> center;
  
  center.clear();

  if (nrpts > 0) {
    if (nrpts == 1) {
      oct->GetOctTreeRandom(center);
    }else {
      oct->GetOctTreeRandom(center, nrpts);
    }
  } else {
    oct->GetOctTreeCenter(center);
  }

  // storing it as reduced scan
  points_red = new double*[center.size()];

  end_loop = (int)center.size();
  for (int i = 0; i < end_loop; i++) {
    points_red[i] = new double[3];
    points_red[i][0] = center[i][0];
    points_red[i][1] = center[i][1];
    points_red[i][2] = center[i][2];
  }
  points_red_size = center.size();

  delete oct;
  
  end_loop = (int)points.size();
  for (int i = 0; i < num_pts; i++) {
    delete [] ptsOct[i];
  }
  delete [] ptsOct;
}

/**
 * Calculates the search trees for all scans
 */
void Scan::createTrees(bool use_cache)
{
  cerr << "create " << allScans.size() << " k-d trees " << flush;
  int i;
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
    for (i = 0; i < (int)allScans.size(); i++) {
	 allScans[i]->createTree(use_cache); 
  }
  cerr << "... done." << endl;
  return;
}


/**
 * Deletes all search trees
 */
void Scan::deleteTrees()
{
  int i;
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
  for (i = 0; i < (int)allScans.size(); i++) { 
    allScans[i]->deleteTree();
  }
  return;
}


/**
 * This function returns a pointer to the cache memory. If the
 * cache already exists, the pointer to the memory is returned,
 * otheriwse a new memory chun is allocated
 * resp. afterwards.
 *
 * @param Source Pointer to first scan
 * @param Target Pointer to second scan
 * @return Pointer to cache memory
 */ 
KDCacheItem* Scan::initCache(const Scan* Source, const Scan* Target)
{
  KDCacheItem *closest = 0;
  
  // determine cache
  for(unsigned int i = 0; i < closest_cache.size(); i++) {
    if ((closest_cache[i]->SourceScanNr == Source->scanNr) &&
	   (closest_cache[i]->TargetScanNr == Target->scanNr)) {
	 closest = closest_cache[i]->item; 	                 // cache found
	 break;
    }
  }
  // cache for this source/target pair not initialized
  if (closest == 0) {
    closest = new KDCacheItem[Target->points_red_size];
    KDCache *nc = new KDCache;
    nc->item = closest;
    nc->SourceScanNr = Source->scanNr;
    nc->TargetScanNr = Target->scanNr;
    closest_cache.push_back(nc);                              // append cache
  }

  return closest;
}


/**
 * Calculates a set of corresponding point pairs and returns them. It
 * computes the k-d trees and deletes them after the pairs have been
 * found. This slow function should be used only for testing
 * 
 * @param pairs The resulting point pairs (vector will be filled)
 * @param Source The scan whose points are matched to Targets' points
 * @param Target The scan to whiche the opints are matched
 * @param thread_num number of the thread (for parallelization)
 * @param rnd randomized point selection
 * @param max_dist_match2 maximal allowed distance for matching
 */
void Scan::getPtPairsSimple(vector <PtPair> *pairs, 
					   Scan* Source, Scan* Target, 
					   int thread_num,
					   int rnd, double max_dist_match2,
					   double *centroid_m, double *centroid_d)
{
  unsigned int numpts_target;
  KDtree *kd = 0;

  kd = new KDtree(Source->points_red, Source->points_red_size);
  numpts_target = Target->points_red_size;
  
  for (unsigned int i = 0; i < numpts_target; i++) {
    if (rnd > 1 && rand(rnd) != 0) continue;  // take about 1/rnd-th of the numbers only

    double p[3];
    p[0] = Target->points_red[i][0];
    p[1] = Target->points_red[i][1];
    p[2] = Target->points_red[i][2];

    double *closest = kd->FindClosest(p, max_dist_match2, thread_num);
    if (closest) {
	 centroid_m[0] += p[0];
	 centroid_m[1] += p[1];
	 centroid_m[2] += p[2];
	 centroid_d[0] += closest[0];
	 centroid_d[1] += closest[1];
	 centroid_d[2] += closest[2];	 
	 PtPair myPair(closest, p);
	 pairs->push_back(myPair);
    }
  }
  centroid_m[0] /= pairs[thread_num].size();
  centroid_m[1] /= pairs[thread_num].size();
  centroid_m[2] /= pairs[thread_num].size();
  centroid_d[0] /= pairs[thread_num].size();
  centroid_d[1] /= pairs[thread_num].size();
  centroid_d[2] /= pairs[thread_num].size();
  
  delete kd;
  return;
}


/**
 * Calculates a set of corresponding point pairs and returns them.
 * The function uses the k-d trees stored the the scan class, thus
 * the function createTrees and deletTrees have to be called before
 * resp. afterwards.
 * Here we implement the so called "fast corresponding points"; k-d
 * trees are not recomputed, instead the apply the inverse transformation
 * to to the given point set.
 * 
 * @param pairs The resulting point pairs (vector will be filled)
 * @param Source The scan whose points are matched to Targets' points
 * @param Target The scan to whiche the opints are matched
 * @param thread_num number of the thread (for parallelization)
 * @param rnd randomized point selection
 * @param max_dist_match2 maximal allowed distance for matching
 * @return a set of corresponding point pairs
 */
void Scan::getPtPairs(vector <PtPair> *pairs, 
				  Scan* Source, Scan* Target, 
				  int thread_num,
				  int rnd, double max_dist_match2,
				  double *centroid_m, double *centroid_d)
{
  centroid_m[0] = 0.0;
  centroid_m[1] = 0.0;
  centroid_m[2] = 0.0;
  centroid_d[0] = 0.0;
  centroid_d[1] = 0.0;
  centroid_d[2] = 0.0;
  
  double local_alignxf_inv[16];
  M4inv(Source->dalignxf, local_alignxf_inv);

  for (unsigned int i = 0; i < (unsigned int)Target->points_red_size; i++) {
    if (rnd > 1 && rand(rnd) != 0) continue;  // take about 1/rnd-th of the numbers only

    double p[3];
    p[0] = Target->points_red[i][0];
    p[1] = Target->points_red[i][1];
    p[2] = Target->points_red[i][2];
    
    double x_neu, y_neu, z_neu;
    x_neu = p[0] * local_alignxf_inv[0] + p[1] * local_alignxf_inv[4] + p[2] * local_alignxf_inv[8];
    y_neu = p[0] * local_alignxf_inv[1] + p[1] * local_alignxf_inv[5] + p[2] * local_alignxf_inv[9];
    z_neu = p[0] * local_alignxf_inv[2] + p[1] * local_alignxf_inv[6] + p[2] * local_alignxf_inv[10];
    p[0] = x_neu + local_alignxf_inv[12];
    p[1] = y_neu + local_alignxf_inv[13];
    p[2] = z_neu + local_alignxf_inv[14];

    //    cout << endl << "query Point " << p[0] << " " << p[1] << " " << p[2];
    
    double *closest = ((KDtree*)Source->kd)->FindClosest(p, max_dist_match2, thread_num);
    if (closest) {

	 x_neu = closest[0] * Source->dalignxf[0] + closest[1] * Source->dalignxf[4] + closest[2] * Source->dalignxf[8];
	 y_neu = closest[0] * Source->dalignxf[1] + closest[1] * Source->dalignxf[5] + closest[2] * Source->dalignxf[9];
	 z_neu = closest[0] * Source->dalignxf[2] + closest[1] * Source->dalignxf[6] + closest[2] * Source->dalignxf[10];
	 p[0] = x_neu + Source->dalignxf[12];
	 p[1] = y_neu + Source->dalignxf[13];
	 p[2] = z_neu + Source->dalignxf[14];

	 //	 cout << endl << "closest point " << p[0] << " " << p[1] << " " << p[2] << endl;

	 centroid_d[0] += Target->points_red[i][0];
	 centroid_d[1] += Target->points_red[i][1];
	 centroid_d[2] += Target->points_red[i][2];
	 centroid_m[0] += p[0];
	 centroid_m[1] += p[1];
	 centroid_m[2] += p[2];	 
	 
	 PtPair myPair(p, Target->points_red[i]);

	 pairs->push_back(myPair);
    }
  }

  centroid_m[0] /= pairs->size();
  centroid_m[1] /= pairs->size();
  centroid_m[2] /= pairs->size();
  centroid_d[0] /= pairs->size();
  centroid_d[1] /= pairs->size();
  centroid_d[2] /= pairs->size();

  return;
}


/**
 * Calculates a set of corresponding point pairs and returns them.
 * The function uses also the fast corresponding points method (see 
 * getPtPairs) and uses the cached version of the search.
 *
 * Reference: "Cached k-d tree search for ICP algorithms" by A. Nuechter
 *            et al., Proceedings IEEE 3DIM, Montreal, Canada, 2007.
 * 
 * @param Source The scan whose points are matched to Targets' points
 * @param Target The scan to whiche the opints are matched
 * @param pairs the resulting point pairs
 * @param thread_num number of the thread (for parallelization)
 * @param rnd randomized point selection
 * @param max_dist_match2 maximal allowed distance for matching
 * @return a set of corresponding point pairs
 */
void Scan::getPtPairsCache(vector <PtPair> *pairs, KDCacheItem *closest, 
					  Scan* Source, Scan* Target, 
					  int thread_num,
					  int rnd, double max_dist_match2,
					  double *centroid_m, double *centroid_d)
{
  double local_alignxf_inv[16];
  M4inv(Source->dalignxf, local_alignxf_inv);

  for (unsigned int i = 0; i < (unsigned int)Target->points_red_size; i++) {
    if (rnd > 1 && rand(rnd) != 0) continue;  // take about 1/rnd-th of the numbers only

    double p[3];
    p[0] = Target->points_red[i][0];
    p[1] = Target->points_red[i][1];
    p[2] = Target->points_red[i][2];
    
    double x_neu, y_neu, z_neu;
    x_neu = p[0] * local_alignxf_inv[0] + p[1] * local_alignxf_inv[4] + p[2] * local_alignxf_inv[8];
    y_neu = p[0] * local_alignxf_inv[1] + p[1] * local_alignxf_inv[5] + p[2] * local_alignxf_inv[9];
    z_neu = p[0] * local_alignxf_inv[2] + p[1] * local_alignxf_inv[6] + p[2] * local_alignxf_inv[10];
    p[0] = x_neu + local_alignxf_inv[12];
    p[1] = y_neu + local_alignxf_inv[13];
    p[2] = z_neu + local_alignxf_inv[14];

    if (closest[i].node) {
	 closest[i] = *(closest[i].node->FindClosestCache(p, max_dist_match2, thread_num));
    } else {
	 closest[i] = *(((KDtree_cache*)Source->kd)->FindClosestCacheInit(p, max_dist_match2, thread_num));
    }

    if (closest[i].param.closest_d2 < max_dist_match2) {
	 
	 x_neu = closest[i].param.closest[0] * Source->dalignxf[0]
	   + closest[i].param.closest[1] * Source->dalignxf[4]
	   + closest[i].param.closest[2] * Source->dalignxf[8];
	 y_neu = closest[i].param.closest[0] * Source->dalignxf[1]
	   + closest[i].param.closest[1] * Source->dalignxf[5]
	   + closest[i].param.closest[2] * Source->dalignxf[9];
	 z_neu = closest[i].param.closest[0] * Source->dalignxf[2]
	   + closest[i].param.closest[1] * Source->dalignxf[6]
	   + closest[i].param.closest[2] * Source->dalignxf[10];
	 
	 p[0] = x_neu + Source->dalignxf[12];
	 p[1] = y_neu + Source->dalignxf[13];
	 p[2] = z_neu + Source->dalignxf[14];

	 centroid_m[0] += Target->points_red[i][0];
	 centroid_m[1] += Target->points_red[i][1];
	 centroid_m[2] += Target->points_red[i][2];
	 centroid_d[0] += p[0];
	 centroid_d[1] += p[1];
	 centroid_d[2] += p[2];	 
	 
	 PtPair myPair(p, Target->points_red[i]);
	 pairs->push_back(myPair);
    }
  }

  centroid_m[0] /= pairs[thread_num].size();
  centroid_m[1] /= pairs[thread_num].size();
  centroid_m[2] /= pairs[thread_num].size();
  centroid_d[0] /= pairs[thread_num].size();
  centroid_d[1] /= pairs[thread_num].size();
  centroid_d[2] /= pairs[thread_num].size();
  
  return;
}


/**
 * Calculates a set of corresponding point pairs and returns them.
 * The function uses the k-d trees stored the the scan class, thus
 * the function createTrees and delteTrees have to be called before
 * resp. afterwards.
 * 
 * @param pairs The resulting point pairs (vector will be filled)
 * @param Source The scan whose points are matched to Targets' points
 * @param Target The scan to whiche the opints are matched
 * @param thread_num The number of the thread that is computing ptPairs in parallel 
 * @param step The number of steps for parallelization
 * @param rnd randomized point selection
 * @param max_dist_match2 maximal allowed distance for matching
 * @param sum The sum of distances of the points
 *
 * These intermediate values are for the parallel ICP algorithm 
 * introduced in the paper  
 * "The Parallel Iterative Closest Point Algorithm"
 *  by Langis / Greenspan / Godin, IEEE 3DIM 2001
 *
 */
void Scan::getPtPairsParallel(vector <PtPair> *pairs, Scan* Source, Scan* Target,
						int thread_num, int step,
						int rnd, double max_dist_match2,
						double *sum,
						double centroid_m[OPENMP_NUM_THREADS][3], double centroid_d[OPENMP_NUM_THREADS][3])
{
  double local_alignxf_inv[16];
  M4inv(Source->dalignxf, local_alignxf_inv);

  for (int i = thread_num * step; i < thread_num * step + step; i++) {
    if (rnd > 1 && rand(rnd) != 0) continue;  // take about 1/rnd-th of the numbers only

    double p[3];
    p[0] = Target->points_red[i][0];
    p[1] = Target->points_red[i][1];
    p[2] = Target->points_red[i][2];

    double x_neu, y_neu, z_neu;
    x_neu = p[0] * local_alignxf_inv[0] + p[1] * local_alignxf_inv[4] + p[2] * local_alignxf_inv[8];
    y_neu = p[0] * local_alignxf_inv[1] + p[1] * local_alignxf_inv[5] + p[2] * local_alignxf_inv[9];
    z_neu = p[0] * local_alignxf_inv[2] + p[1] * local_alignxf_inv[6] + p[2] * local_alignxf_inv[10];
    p[0] = x_neu + local_alignxf_inv[12];
    p[1] = y_neu + local_alignxf_inv[13];
    p[2] = z_neu + local_alignxf_inv[14];
	  
    double *closest = ((KDtree*)Source->kd)->FindClosest( p,
											   max_dist_match2,
											   thread_num );
    if (closest) {

	 x_neu = closest[0] * Source->dalignxf[0] + closest[1] * Source->dalignxf[4] + closest[2] * Source->dalignxf[8];
	 y_neu = closest[0] * Source->dalignxf[1] + closest[1] * Source->dalignxf[5] + closest[2] * Source->dalignxf[9];
	 z_neu = closest[0] * Source->dalignxf[2] + closest[1] * Source->dalignxf[6] + closest[2] * Source->dalignxf[10];
	 p[0] = x_neu + Source->dalignxf[12];
	 p[1] = y_neu + Source->dalignxf[13];
	 p[2] = z_neu + Source->dalignxf[14];

	 centroid_m[thread_num][0] += Target->points_red[i][0];
	 centroid_m[thread_num][1] += Target->points_red[i][1];
	 centroid_m[thread_num][2] += Target->points_red[i][2];
	 centroid_d[thread_num][0] += p[0];
	 centroid_d[thread_num][1] += p[1];
	 centroid_d[thread_num][2] += p[2];	 
	 
	 PtPair myPair(p, Target->points_red[i]);
	 pairs[thread_num].push_back(myPair);
	 
	 double p12[3] = { myPair.p1.x - myPair.p2.x, 
				    myPair.p1.y - myPair.p2.y,
				    myPair.p1.z - myPair.p2.z };
	 sum[thread_num] += Len2(p12);
    }
  }

  if(pairs[thread_num].size() == 0) // do not divide by zero
    return;

  centroid_m[thread_num][0] /= pairs[thread_num].size();
  centroid_m[thread_num][1] /= pairs[thread_num].size();
  centroid_m[thread_num][2] /= pairs[thread_num].size();
  centroid_d[thread_num][0] /= pairs[thread_num].size();
  centroid_d[thread_num][1] /= pairs[thread_num].size();
  centroid_d[thread_num][2] /= pairs[thread_num].size();
  
  return;
}


/**
 * Calculates a set of corresponding point pairs and returns them.
 * The function uses the k-d trees stored the the scan class, thus
 * the function createTrees and delteTrees have to be called before
 * resp. afterwards.
 * 
 * @param pairs The resulting point pairs (vector will be filled)
 * @param Source The scan whose points are matched to Targets' points
 * @param Target The scan to whiche the opints are matched
 * @param thread_num The number of the thread that is computing ptPairs in parallel 
 * @param step The number of steps for parallelization
 * @param rnd randomized point selection
 * @param max_dist_match2 maximal allowed distance for matching
 * @param sum The sum of distances of the points
 *
 * These intermediate values are for the parallel ICP algorithm 
 * introduced in the paper  
 * "The Parallel Iterative Closest Point Algorithm"
 *  by Langis / Greenspan / Godin, IEEE 3DIM 2001
 * see also : "Cached k-d tree search for ICP algorithms" by A. Nuechter
 *             et al., Proceedings IEEE 3DIM, Montreal, Canada, 2007.
 *
 */
void Scan::getPtPairsCacheParallel(vector <PtPair> *pairs, KDCacheItem *closest,
							Scan* Source, Scan* Target,
							int thread_num, int step,
							int rnd, double max_dist_match2,
							double *sum,
							double centroid_m[OPENMP_NUM_THREADS][3], double centroid_d[OPENMP_NUM_THREADS][3])
{
  double local_alignxf_inv[16];
  M4inv(Source->dalignxf, local_alignxf_inv);

  for (int i = thread_num * step; i < thread_num * step + step; i++) {

    if (rnd > 1 && rand(rnd) != 0) continue;  // take about 1/rnd-th of the numbers only

    double p[3];
    p[0] = Target->points_red[i][0];
    p[1] = Target->points_red[i][1];
    p[2] = Target->points_red[i][2];

    double x_neu, y_neu, z_neu;
    x_neu = p[0] * local_alignxf_inv[0] + p[1] * local_alignxf_inv[4] + p[2] * local_alignxf_inv[8];
    y_neu = p[0] * local_alignxf_inv[1] + p[1] * local_alignxf_inv[5] + p[2] * local_alignxf_inv[9];
    z_neu = p[0] * local_alignxf_inv[2] + p[1] * local_alignxf_inv[6] + p[2] * local_alignxf_inv[10];
    p[0] = x_neu + local_alignxf_inv[12];
    p[1] = y_neu + local_alignxf_inv[13];
    p[2] = z_neu + local_alignxf_inv[14];
	  
    if (closest[i].node) {
	 closest[i] = *(closest[i].node->FindClosestCache(p, max_dist_match2, thread_num));
    } else {
	 closest[i] = *(((KDtree_cache*)Source->kd)->FindClosestCacheInit(p, max_dist_match2, thread_num));
    }

    if (closest[i].param.closest_d2 < max_dist_match2) {
	 
	 x_neu = closest[i].param.closest[0] * Source->dalignxf[0]
	   + closest[i].param.closest[1] * Source->dalignxf[4]
	   + closest[i].param.closest[2] * Source->dalignxf[8];
	 y_neu = closest[i].param.closest[0] * Source->dalignxf[1]
	   + closest[i].param.closest[1] * Source->dalignxf[5]
	   + closest[i].param.closest[2] * Source->dalignxf[9];
	 z_neu = closest[i].param.closest[0] * Source->dalignxf[2]
	   + closest[i].param.closest[1] * Source->dalignxf[6]
	   + closest[i].param.closest[2] * Source->dalignxf[10];
	 
	 p[0] = x_neu + Source->dalignxf[12];
	 p[1] = y_neu + Source->dalignxf[13];
	 p[2] = z_neu + Source->dalignxf[14];

	 centroid_m[thread_num][0] += Target->points_red[i][0];
	 centroid_m[thread_num][1] += Target->points_red[i][1];
	 centroid_m[thread_num][2] += Target->points_red[i][2];
	 centroid_d[thread_num][0] += p[0];
	 centroid_d[thread_num][1] += p[1];
	 centroid_d[thread_num][2] += p[2];	 
	 
	 PtPair myPair(p, Target->points_red[i]);
	 pairs[thread_num].push_back(myPair);
	 double p12[3] = { myPair.p1.x - myPair.p2.x, 
			   myPair.p1.y - myPair.p2.y,
			   myPair.p1.z - myPair.p2.z };
	 sum[thread_num] += Len2(p12);
    }
  }

  if(pairs[thread_num].size() == 0) // do not divide by zero
    return;

  centroid_m[thread_num][0] /= pairs[thread_num].size();
  centroid_m[thread_num][1] /= pairs[thread_num].size();
  centroid_m[thread_num][2] /= pairs[thread_num].size();
  centroid_d[thread_num][0] /= pairs[thread_num].size();
  centroid_d[thread_num][1] /= pairs[thread_num].size();
  centroid_d[thread_num][2] /= pairs[thread_num].size();
  
  return;
}

/**
 * Computes a search tree depending on the type this can be 
 * a k-d tree od a cached k-d tree
 */
void Scan::createTree(bool use_cache)
{
  M4identity(dalignxf);
  double temp[16];
  memcpy(temp, transMat, sizeof(transMat));
  M4inv(temp, treeTransMat_inv);

  points_red_lum = new double*[points_red_size];
  for (int j = 0; j < points_red_size; j++) {
    points_red_lum[j] = new double[3];
    points_red_lum[j][0] = points_red[j][0];
    points_red_lum[j][1] = points_red[j][1];
    points_red_lum[j][2] = points_red[j][2];
  }

  //  cout << "d2 tree" << endl;
  //  kd = new D2Tree(points_red_lum, points_red_size, 105);
  //  cout << "successfull" << endl;

  
 if (use_cache) {
   kd = new KDtree_cache(points_red_lum, points_red_size);
 } else {
   kd = new KDtree(points_red_lum, points_red_size);
 }
  
  return;
}

/**
 * Delete the search tree
 */
void Scan::deleteTree()
{
  for (int j = 0; j < points_red_size; j++) {
    delete [] points_red_lum[j];
  }
  delete [] points_red_lum;
  
  delete kd;
  
  return;
}




/**
 * Reads specified scans from given directory. 
 * Scan poses will NOT be initialized after a call 
 * to this function. It loads a shared lib where the 
 * actual file processing takes place
 * 
 * @param type Specifies the type of the flies to be loaded
 * @param start Starts to read with this scan
 * @param end Stops with this scan
 * @param _dir The drectory containing the data
 * @param maxDist Reads only Points up to this distance
 * @param minDist Reads only Points from this distance
 * @param openFileForWriting Opens .frames files to store the 
 *        scan matching results
 */
void Scan::readScans(reader_type type,
		     int start, int end, string &_dir, int maxDist, int minDist, 
		     bool openFileForWriting)
{
  outputFrames = openFileForWriting;
  dir = _dir;

  // load the lib
  string lib_string;
  switch (type) {
  case UOS:
    lib_string = "./bin/scan_io_uos";
    break;
  case UOS_MAP:
    lib_string = "./bin/scan_io_uos_map";
    break;
  case UOS_FRAMES:
    lib_string = "./bin/scan_io_uos_frames";
    break;
   case UOS_MAP_FRAMES:
    lib_string = "./bin/scan_io_uos_map_frames";
    break;
  case OLD:
    lib_string = "./bin/scan_io_old";
    break;
  case RTS:
    lib_string = "./bin/scan_io_rts";
    break;
  case RTS_MAP:
    lib_string = "./bin/scan_io_rts_map";
    break;
  case IFP:
    lib_string = "./bin/scan_io_ifp";
    break;
  case RIEGL_TXT:
    lib_string = "./bin/scan_io_riegl_txt";
    break;
  case RIEGL_BIN:
    lib_string = "./bin/scan_io_riegl_BIN";
    break;
  case ZAHN:
    lib_string = "./bin/scan_io_zahn";
    break;
  case PLY:
    lib_string = "./bin/scan_io_ply";
    break;
  case WRL:
    lib_string = "./bin/scan_io_wrl";
    break;
  case XYZ:
    lib_string = "./bin/scan_io_xyz";
    break;
  case ZUF:
    lib_string = "./bin/scan_io_zuf";
    break;
  case ASC:
    lib_string = "./bin/scan_io_asc";
    break;
  case IAIS:
    lib_string = "./bin/scan_io_iais";
    break;
  case FRONT:
    lib_string = "./bin/scan_io_front";
    break;
  case X3D:
    lib_string = "./bin/scan_io_x3d";
    break;
  case RXP:
    lib_string = "./bin/scan_io_rxp";
    break;
  default:
    cerr << "Don't recognize format " << type << endl;
    exit(1);
  }

#ifdef WIN32
  lib_string += ".dll";	
#else
  lib_string += ".so";	
#endif
  cerr << "Loading shared lib " << lib_string;
  
#ifdef _MSC_VER
  HINSTANCE hinstLib = LoadLibrary(lib_string.c_str());
  if (!hinstLib) {
    cerr << "Cannot load library: " << lib_string.c_str() << endl;
    exit(-1);
  }

  cerr << " ... done." << endl << endl;

  create_sio* create_ScanIO = (create_sio*)GetProcAddress(hinstLib, "create");

  if (!create_ScanIO) {
    cerr << "Cannot load symbol create " << endl;
	FreeLibrary(hinstLib);
    exit(-1);
  }
 
#else
  void *ptrScanIO = dlopen(lib_string.c_str(), RTLD_LAZY);

  if (!ptrScanIO) {
    cerr << "Cannot load library: " << dlerror() << endl;
    exit(-1);
  }

  cerr << " ... done." << endl << endl;
  
  // reset the errors
  dlerror();

  // load the symbols
  create_sio* create_ScanIO = (create_sio*)dlsym(ptrScanIO, "create");
  const char* dlsym_error = dlerror();
  if (dlsym_error) {
    cerr << "Cannot load symbol create: " << dlsym_error << endl;
    exit(-1);
  }
#endif

  // create an instance of ScanIO
  ScanIO *my_ScanIO = create_ScanIO();

  double eu[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  vector <Point> ptss;
  int _fileNr;

  // read Scan-by-scan until no scan is available anymore
  while ((_fileNr = my_ScanIO->readScans(start, end, dir, maxDist, minDist, eu, ptss)) != -1) {
    Scan *currentScan = new Scan(eu, maxDist);
    currentScan->fileNr = _fileNr;

    currentScan->points = ptss;    // copy points
    ptss.clear();                  // clear points
    allScans.push_back(currentScan);
  }

#ifdef _MSC_VER
  destroy_sio* destroy_ScanIO = (destroy_sio*)GetProcAddress(hinstLib, "destroy");

  if (!destroy_ScanIO) {
    cerr << "Cannot load symbol destroy " << endl;
    FreeLibrary(hinstLib);
    exit(-1);
  }
  destroy_ScanIO(my_ScanIO);
#else
  destroy_sio* destroy_ScanIO = (destroy_sio*)dlsym(ptrScanIO, "destroy");
  dlsym_error = dlerror();
  if (dlsym_error) {
    cerr << "Cannot load symbol create: " << dlsym_error << endl;
    exit(-1);
  }
  destroy_ScanIO(my_ScanIO);
#endif
  return;
}

