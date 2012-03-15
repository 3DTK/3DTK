/**
 * @file
 * @brief Implementation of a 3D scan and of 3D scan matching in all variants
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */
  
#include "slam6d/scan.h"

#ifdef _MSC_VER
#if !defined _OPENMP && defined OPENMP 
#define _OPENMP
#endif
#endif 
 
#ifdef _OPENMP
#include <omp.h>
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
#include <cstring>
using std::flush;

#include "slam6d/kd.h"
#ifdef WITH_SCANSERVER
#include "slam6d/kdMeta.h"
#endif //WITH_SCANSERVER
#include "slam6d/Boctree.h"
#include "slam6d/ann_kd.h"
#include "slam6d/kdc.h"
#include "slam6d/d2tree.h"

#ifndef WITH_SCANSERVER
#include "slam6d/scan_io.h"

#ifdef _MSC_VER
#include <windows.h>
#else
#include <dlfcn.h>
#endif
#endif //WITH_SCANSERVER

#ifdef WITH_SCANSERVER
#include "scanserver/clientInterface.h"
#endif

#ifdef WITH_METRICS
#include "scanserver/metrics.h"
#endif



vector <Scan *>  Scan::allScans;
unsigned int     Scan::numberOfScans = 0;
#ifndef WITH_SCANSERVER
unsigned int     Scan::max_points_red_size = 0;
bool             Scan::outputFrames = false;
string           Scan::dir;
#else //WITH_SCANSERVER
ScanVector* Scan::shared_scans = 0;
#endif //WITH_SCANSERVER



/**
 * default Constructor
 */
Scan::Scan()
{
#ifndef WITH_SCANSERVER
  kd = 0;
  ann_kd_tree = 0;
  nns_method = 1;
  fileNr = 0;
  scanNr = 0;
  cuda_enabled = false;
  rPos[0] = 0;
  rPos[1] = 0;
  rPos[2] = 0;
  rPosTheta[0] = 0;
  rPosTheta[1] = 0;
  rPosTheta[2] = 0;
  M4identity(transMat);
  M4identity(transMatOrg);
  M4identity(dalignxf);

  points_red_size = 0;
  points_red = points_red_lum = 0;
#else //WITH_SCANSERVER
  init();
#endif //WITH_SCANSERVER
}


/**
 * Constructor
 * @param *euler 6D pose: estimation of the scan location, e.g. based on odometry
 * @param maxDist Regard only points up to an (Euclidean) distance of maxDist
 * transformation matrices when match (default: false)
 */
Scan::Scan(const double* euler, int maxDist)
{
#ifndef WITH_SCANSERVER
  kd = 0;
  ann_kd_tree = 0;
  nns_method = 1;
  cuda_enabled = false;
  maxDist2 = (maxDist != -1 ? sqr(maxDist) : maxDist);
  
  if (dir == "") {
    cerr << "ERROR: Directory has to be set before!" << endl;
    exit(1);
  }
  
  rPos[0] = euler[0];
  rPos[1] = euler[1];
  rPos[2] = euler[2];
  rPosTheta[0] = euler[3];
  rPosTheta[1] = euler[4];
  rPosTheta[2] = euler[5];
  M4identity(transMat);
  if (euler == 0) {
    M4identity(transMatOrg);
  } else {
    EulerToMatrix4(euler, &euler[3], transMatOrg);
  }
  
  fileNr = 0;
  scanNr = numberOfScans++;
  
  points_red_size = 0;
  points_red = points_red_lum = 0;

  M4identity(dalignxf);
#else //WITH_SCANSERVER
  init();
  scanNr = numberOfScans++;
  maxDist2 = (maxDist != -1 ? sqr(maxDist) : maxDist);
  
  rPos[0] = euler[0];
  rPos[1] = euler[1];
  rPos[2] = euler[2];
  rPosTheta[0] = euler[3];
  rPosTheta[1] = euler[4];
  rPosTheta[2] = euler[5];
  
  EulerToMatrix4(euler, &euler[3], transMatOrg);
#endif //WITH_SCANSERVER
}

Scan::Scan(const double _rPos[3], const double _rPosTheta[3], vector<double *> &pts)
{
  // TODO: change vector<double[3]> to something more simpler
#ifndef WITH_SCANSERVER
  kd = 0;
  ann_kd_tree = 0;
  nns_method = 1;
  cuda_enabled = false;
  maxDist2 = -1;
  rPos[0] = _rPos[0];
  rPos[1] = _rPos[1];
  rPos[2] = _rPos[2];
  rPosTheta[0] = _rPosTheta[0];
  rPosTheta[1] = _rPosTheta[1];
  rPosTheta[2] = _rPosTheta[2];
  M4identity(transMat);
  EulerToMatrix4(_rPos, _rPosTheta, transMatOrg);

  fileNr = 0;
  scanNr = numberOfScans++;

  points_red_size = 0;
  M4identity(dalignxf);
#else //WITH_SCANSERVER
  init();
  scanNr = numberOfScans++;
  
  rPos[0] = _rPos[0];
  rPos[1] = _rPos[1];
  rPos[2] = _rPos[2];
  rPosTheta[0] = _rPosTheta[0];
  rPosTheta[1] = _rPosTheta[1];
  rPosTheta[2] = _rPosTheta[2];
  
  EulerToMatrix4(_rPos, _rPosTheta, transMatOrg);
#endif //WITH_SCANSERVER

#ifndef WITH_SCANSERVER
  points_red = new double*[pts.size()];

  points_red_size = (int)pts.size();
  for (int i = 0; i < points_red_size; i++) {
    points_red[i] = pts[i];
  }
#else //WITH_SCANSERVER
  unsigned int size = pts.size();
  // copy the points
  DataXYZ xyz(m_shared_scan->createXYZReduced(size));
  for(unsigned int i = 0; i < size; ++i) {
    for(unsigned int j = 0; j < 3; ++j) {
      xyz[i][j] = pts[i][j];
    }
    // clear the double[3]'s from pts because they were copied only, not ownership taken
    delete[] pts[i];
  }
#endif //WITH_SCANSERVER
  
  transform(transMatOrg, INVALID); //transform points to initial position
#ifndef WITH_SCANSERVER
  // update max num point in scan iff you have to do so
  if (points_red_size > (int)max_points_red_size) max_points_red_size = points_red_size;
#endif //WITH_SCANSERVER

  pts.clear();
}


/**
 * Constructor
 * @param _rPos[3] 3D position: estimation of the scan location, e.g. based on odometry
 * @param _rPosTheta[3] 3D orientation: estimation of the scan location, e.g. based on odometry
 * @param maxDist Regard only points up to an (Euclidean) distance of maxDist
 */
Scan::Scan(const double _rPos[3], const double _rPosTheta[3], const int maxDist)
{
#ifndef WITH_SCANSERVER
  kd = 0;
  ann_kd_tree = 0;
  nns_method = 1;
  cuda_enabled = false;
  maxDist2 = (maxDist != -1 ? sqr(maxDist) : maxDist);
  rPos[0] = _rPos[0];
  rPos[1] = _rPos[1];
  rPos[2] = _rPos[2];
  rPosTheta[0] = _rPosTheta[0];
  rPosTheta[1] = _rPosTheta[1];
  rPosTheta[2] = _rPosTheta[2];
  M4identity(transMat);
  EulerToMatrix4(_rPos, _rPosTheta, transMatOrg);

  fileNr = 0;
  scanNr = numberOfScans++;
  
  points_red_size = 0;
  M4identity(dalignxf);
#else //WITH_SCANSERVER
  init();
  scanNr = numberOfScans++;
  maxDist2 = (maxDist != -1 ? sqr(maxDist) : maxDist);
  
  rPos[0] = _rPos[0];
  rPos[1] = _rPos[1];
  rPos[2] = _rPos[2];
  rPosTheta[0] = _rPosTheta[0];
  rPosTheta[1] = _rPosTheta[1];
  rPosTheta[2] = _rPosTheta[2];
  
  EulerToMatrix4(_rPos, _rPosTheta, transMatOrg);
#endif //WITH_SCANSERVER
}

/**
 * Constructor for creating a metascan from a list of scans
 * It joins all the points and contructs a new search tree
 * and reinitializes the cache, iff needed
 *
 * @param MetaScan Vector that contains the 3D scans
 * @param nns_method Indicates the version of the tree to be built
 * @param cuda_enabled indicated, if cuda should be used for NNS
 */
Scan::Scan(const vector < Scan* >& MetaScan, int nns_method, bool cuda_enabled)
{
#ifndef WITH_SCANSERVER
  kd = 0;
  ann_kd_tree = 0;
  this->cuda_enabled = cuda_enabled;
  this->nns_method = nns_method;
  scanNr = numberOfScans++;
  rPos[0] = 0;
  rPos[1] = 0;
  rPos[2] = 0;
  rPosTheta[0] = 0;
  rPosTheta[1] = 0;
  rPosTheta[2] = 0;
  M4identity(transMat);
  M4identity(transMatOrg);
  M4identity(dalignxf);
  fileNr = -1; // no need to store something from a meta scan!
#else //WITH_SCANSERVER
  init();
  scanNr = numberOfScans++;
  this->cuda_enabled = cuda_enabled;
  this->nns_method = nns_method;
#endif //WITH_SCANSERVER

  // copy points
  meta_parts = MetaScan;
  int numpts = 0;
#ifndef WITH_SCANSERVER
#ifdef WITH_METRICS
  Timer t = ClientMetric::copy_original_time.start();
#endif //WITH_METRICS

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
  
#ifdef WITH_METRICS
  ClientMetric::copy_original_time.end(t);
#endif //WITH_METRICS
  
  // update max num point in scan iff you have to do so
  if (points_red_size > (int)max_points_red_size) max_points_red_size = points_red_size;

  // build new search tree
  createTree(nns_method, cuda_enabled);
#else //WITH_SCANSERVER
  for(vector<Scan*>::const_iterator it = MetaScan.begin(); it != MetaScan.end(); ++it) {
    numpts += (*it)->getCountReduced();
  }
  meta_count = numpts;
#endif //WITH_SCANSERVER

  // add Scan to ScanList
  allScans.push_back(this);
}
  
/**
 * Desctuctor
 */
Scan::~Scan()
{
#ifndef WITH_SCANSERVER
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
#endif //WITH_SCANSERVER

  if (this->kd != 0) deleteTree();

#ifndef WITH_SCANSERVER
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
#else //WITH_SCANSERVER
  // :cripes:
#endif //WITH_SCANSERVER

#ifndef WITH_SCANSERVER
  for (int i = 0; i < points_red_size; i++) {
    delete [] points_red[i];
  }
  delete [] points_red;
  
  points.clear();
#endif //WITH_SCANSERVER
}

/**
 * Copy constructor
 */
Scan::Scan(const Scan& s)
{
#ifdef WITH_SCANSERVER
  init();
#endif //WITH_SCANSERVER
  rPos[0] = s.rPos[0];
  rPos[1] = s.rPos[1];
  rPos[2] = s.rPos[2];
  rPosTheta[0] = s.rPosTheta[0];
  rPosTheta[1] = s.rPosTheta[1];
  rPosTheta[2] = s.rPosTheta[2];

  memcpy(transMat, s.transMat, sizeof(transMat));
  memcpy(transMatOrg, s.transMatOrg, sizeof(transMatOrg));

#ifndef WITH_SCANSERVER
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
#endif //WITH_SCANSERVER
  memcpy(dalignxf, s.dalignxf, sizeof(dalignxf));
  nns_method = s.nns_method;
  cuda_enabled = s.cuda_enabled;
  if (s.kd != 0) {
    createTree(nns_method, cuda_enabled);
#ifndef WITH_SCANSERVER
    // update max num point in scan iff you have to do so
    if (points_red_size > (int)max_points_red_size) max_points_red_size = points_red_size;
#endif //WITH_SCANSERVER
  }

  scanNr = s.scanNr;
  maxDist2 = s.maxDist2;
  
#ifdef WITH_SCANSERVER
  m_shared_scan = s.m_shared_scan;
#else
  sout << s.sout.str();
#endif //WITH_SCANSERVER
  
  // TODO: make sure everything is copied
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
void Scan::mergeCoordinatesWithRoboterPosition(Scan* prevScan)
{
  double tempMat[16], deltaMat[16];
  M4inv(prevScan->transMatOrg, tempMat);
  MMult(prevScan->transMat, tempMat, deltaMat);
  transform(deltaMat, INVALID); //apply delta transformation of the previous scan
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
#ifndef WITH_SCANSERVER
  int end_loop = (int)points.size();
  
#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (int i = 0; i < end_loop; ++i) {
    points[i].transform(alignxf);
  }
#else //WITH_SCANSERVER
  DataXYZ xyz(m_shared_scan->getXYZ());
  int size = xyz.size();
  
#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (int i = 0; i < size; i++) {
    // there is absolutely no guarantee that after a cache trash and reload these points remain the same
    // better punish the developer for bad coding
    throw runtime_error("THOU SHALT NOT TOUCH XYZ");
    //transform3(alignxf, xyz[i]);
  }
#endif //WITH_SCANSERVER
}

//! Internal function of transform which alters the reduced points
void Scan::transformReduced(const double alignxf[16])
{
#if defined WITH_METRICS && (defined WITH_SCANSERVER || !defined _OPENMP)
  Timer t = ClientMetric::transform_time.start();
#endif //WITH_METRICS
  
#ifdef WITH_SCANSERVER
  // can't transform points you don't have
  if(meta_count)
    return;
#endif //WITH_SCANSERVER
  
#ifndef WITH_SCANSERVER
#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (int i = 0; i < points_red_size; i++) {
    transform3(alignxf, points_red[i]);
  }
#else //WITH_SCANSERVER
  DataXYZ xyz_r(getXYZReduced());
#ifdef _OPENMP
#pragma omp parallel for
#endif
  for(int i = 0; i < (int)xyz_r.size(); ++i) {
    transform3(alignxf, xyz_r[(unsigned int)i]);
  }
#endif //WITH_SCANSERVER
  
#if defined WITH_METRICS && (defined WITH_SCANSERVER || !defined _OPENMP)
  ClientMetric::transform_time.end(t);
#endif //WITH_METRICS
}

//! Internal function of transform which handles the matrices
void Scan::transformMatrix(const double alignxf[16])
{
  double tempxf[16];
  
  // apply alignxf to transMat and update pose vectors
  MMult(alignxf, transMat, tempxf);
  memcpy(transMat, tempxf, sizeof(transMat));
  Matrix4ToEuler(transMat, rPosTheta, rPos);
  Matrix4ToQuat(transMat, rQuat);
  
#ifdef DEBUG  
  cerr << "(" << rPos[0] << ", " << rPos[1] << ", " << rPos[2] << ", "
	  << rPosTheta[0] << ", " << rPosTheta[1] << ", " << rPosTheta[2] << ")" << endl;

  cerr << transMat << endl;
#endif

  // apply alignxf to dalignxf
  MMult(alignxf, dalignxf, tempxf);
  memcpy(dalignxf, tempxf, sizeof(transMat));
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
//  cout << "Scan::transform [" << (m_shared_scan != 0? m_shared_scan->getIdentifier() : "---") << "]" << endl;
  unsigned int end_meta = meta_parts.size();
  for(unsigned int i = 0; i < end_meta; i++) {
    meta_parts[i]->transform(alignxf, type, -1);
  }

#ifdef TRANSFORM_ALL_POINTS
  // scanserver is incompatible with transforming the full scan data since changes are never changed
#ifndef WITH_SCANSERVER
  /*
   * We dont't need to transform all points, since we use the array
   * points_red all the time.
   *
   * We do need of course the transformation of the points_red array, since
   * thats a moving point cloud. The get ptPairs methods do _not_ consider
   * transformation of target points
   */
  transformAll(alignxf);
#endif // WITH_SCANSERVER
#endif //TRANSFORM_ALL_POINTS
   
#ifdef DEBUG  
  cerr << alignxf << endl;
  cerr << "(" << rPos[0] << ", " << rPos[1] << ", " << rPos[2] << ", "
	  << rPosTheta[0] << ", " << rPosTheta[1] << ", " << rPosTheta[2] << ") ---> ";
#endif

  // transform points
  transformReduced(alignxf);
  
  // update matrices
  transformMatrix(alignxf);
  // store transformation in frames
  bool in_meta;
  int  found = 0;
  int end_loop;
  if (type != INVALID) {
#ifdef WITH_METRICS
    Timer t = ClientMetric::add_frames_time.start();
#endif //WITH_METRICS

    switch (islum) {
    case -1:
      // write no tranformation
      break;
    case 0:
      end_loop = (int)allScans.size();
      for (int iter = 0; iter < end_loop; iter++) {
        in_meta = false;
        for(unsigned int i = 0; i < end_meta; i++) {
          if(meta_parts[i] == allScans[iter]) {
            found = iter;
            in_meta = true;
          }
        }
#ifndef WITH_SCANSERVER
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
#else //WITH_SCANSERVER
        if(allScans[iter] == this || in_meta) {
          found = iter;
          allScans[iter]->addFrame(type);
        } else {
          if (found == 0) {
            allScans[iter]->addFrame(ICPINACTIVE);
          } else {
            allScans[iter]->addFrame(INVALID);
          }
        }
#endif //WITH_SCANSERVER
      }
      break;
    case 1:
#ifndef WITH_SCANSERVER
      if (sout.good()) {
        sout << transMat << type << endl;
      } else {
        cerr << "ERROR: Cannot store frames." << endl;
        exit(1);
      }
#else //WITH_SCANSERVER
      addFrame(type);
#endif //WITH_SCANSERVER
      break;
    case 2:
      end_loop = (int)allScans.size();
      for (int iter = 0; iter < end_loop; iter++) {
        if (allScans[iter] == this) {
          found = iter;
#ifndef WITH_SCANSERVER
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
#else //WITH_SCANSERVER
          addFrame(type);
          allScans[0]->addFrame(type);
#endif //WITH_SCANSERVER
          continue;
        }
        if (found != 0) {
#ifndef WITH_SCANSERVER
          allScans[iter]->sout << allScans[iter]->transMat << INVALID << endl;
#else //WITH_SCANSERVER
          allScans[iter]->addFrame(INVALID);
#endif //WITH_SCANSERVER
        }
      }
      break;
    default:
      cerr << "invalid point transformation mode" << endl;
    }
    
#ifdef WITH_METRICS
    ClientMetric::add_frames_time.end(t);
#endif //WITH_METRICS
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
 * Removes all points from the vector that are above or below a threshold
 * @param top upper threshold
 * @param bottom lower threshold
 */
void Scan::trim(double top, double bottom)
{
#ifndef WITH_SCANSERVER
  vector <Point> ptss;
  for(vector<Point>::iterator it = this->points.begin(); it != this->points.end(); it++) {
  //  cout << (*it).y << endl;
    if((*it).y > bottom && (*it).y < top) {
      ptss.push_back(*it);
    }
  }
  points = ptss;
#else //WITH_SCANSERVER
  // trim by height on loading the scan, invalidate if already loaded
  m_shared_scan->setHeightParameters(top, bottom);
#endif //WITH_SCANSERVER
}

/**
 * Computes an octtree of the current scan, then getting the
 * reduced points as the centers of the octree voxels.
 * @param voxelSize The maximal size of each voxel
 */
void Scan::calcReducedPoints(double voxelSize, int nrpts, PointType pointtype)
{
#ifdef WITH_SCANSERVER
  // update the reduction parameter identification before writing the reduced output
  // if called from calcReducedOnDemand, this will be the same string, causing no resets
  // tools calling toGlobal or this function directly will reset old reduced points correctly
  stringstream s;
  s << voxelSize << " " << nrpts << " " << transMatOrg;
  m_shared_scan->setReductionParameters(s.str().c_str());
  
#ifdef WITH_METRICS
  Timer t = ClientMetric::scan_load_time.start();
#endif //WITH_METRICS
  
  // get xyz to start the scan load, separated here for time measurement
  DataXYZ xyz(m_shared_scan->getXYZ());
  // update buffered count
  m_count = xyz.size();
  
  // if the scan hasn't been loaded we can't calculate anything
  if(m_count == 0)
    throw runtime_error("Could not calculate reduced points, XYZ data is empty");
  
#ifdef WITH_METRICS
  ClientMetric::scan_load_time.end(t);
#endif //WITH_METRICS
#endif //WITH_SCANSERVER
  
#if defined WITH_METRICS && (defined WITH_SCANSERVER || !defined _OPENMP)
  Timer tl = ClientMetric::calc_reduced_points_time.start();
#endif //WITH_METRICS
  
  // no reduction needed
  // copy vector of points to array of points to avoid
  // further copying
  if (voxelSize <= 0.0) {
#ifndef WITH_SCANSERVER
    points_red = new double*[points.size()];
    
    int end_loop = points_red_size = (int)points.size();
    for (int i = 0; i < end_loop; i++) {
      points_red[i] = new double[3];
      points_red[i][0] = points[i].x;
      points_red[i][1] = points[i].y;
      points_red[i][2] = points[i].z;
    }
    // update max num point in scan iff you have to do so
    if (points_red_size > (int)max_points_red_size) max_points_red_size = points_red_size;
#else //WITH_SCANSERVER
    // copy the points
    DataXYZ xyz_r(m_shared_scan->createXYZReduced(m_count));
    for(unsigned int i = 0; i < m_count; ++i) {
      for(unsigned int j = 0; j < 3; ++j) {
        xyz_r[i][j] = xyz[i][j];
      }
    }
    // update buffered count
    m_count_reduced = xyz_r.size();
#endif //WITH_SCANSERVER
  } else {
    // start reduction

    // build octree-tree from CurrentScan
#ifndef WITH_SCANSERVER
    double **ptsOct = 0;
    ptsOct = new double*[points.size()];

    // copy points into ptsOct array
    int num_pts = 0;
    int end_loop = (int)points.size();
    for (int i = 0; i < end_loop; i++) {
      ptsOct[num_pts] = new double[3];
      ptsOct[num_pts][0] = points[i].x;
      ptsOct[num_pts][1] = points[i].y;
      ptsOct[num_pts][2] = points[i].z;
      num_pts++;
    }
    
    BOctTree<double> *oct = new BOctTree<double>(ptsOct, num_pts, voxelSize, pointtype);
#else //WITH_SCANSERVER
    // put full data into the octtree
    BOctTree<double> *oct = new BOctTree<double>(Array<double>(xyz).get(), m_count, voxelSize, pointtype);
#endif //WITH_SCANSERVER

    vector<double*> center;
    center.clear();

    if (nrpts > 0) {
      if (nrpts == 1) {
        oct->GetOctTreeRandom(center);
      } else {
        oct->GetOctTreeRandom(center, nrpts);
      }
    } else {
      oct->GetOctTreeCenter(center);
    }

    // storing it as reduced scan
#ifndef WITH_SCANSERVER
    points_red = new double*[center.size()];

    end_loop = (int)center.size();
    for (int i = 0; i < end_loop; i++) {
      points_red[i] = new double[3];
      points_red[i][0] = center[i][0];
      points_red[i][1] = center[i][1];
      points_red[i][2] = center[i][2];
    }
    points_red_size = center.size();
    // update max num point in scan iff you have to do so
    if (points_red_size > (int)max_points_red_size) max_points_red_size = points_red_size;
    
    for (int i = 0; i < num_pts; i++) {
      delete [] ptsOct[i];
    }
    delete [] ptsOct;
#else //WITH_SCANSERVER
    unsigned int size = center.size();
    DataXYZ xyz_r(m_shared_scan->createXYZReduced(size));
    for(unsigned int i = 0; i < size; ++i) {
      for(unsigned int j = 0; j < 3; ++j) {
        xyz_r[i][j] = center[i][j];
      }
    }
    // update buffered count reduced
    m_count_reduced = xyz_r.size();
#endif //WITH_SCANSERVER
    
    delete oct;
  }
  
#if defined WITH_METRICS && (defined WITH_SCANSERVER || !defined _OPENMP)
  ClientMetric::calc_reduced_points_time.end(tl);
#endif //WITH_METRICS
}

/**
 * Calculates the search trees for all scans
 */
void Scan::createTrees(int nns_method, bool cuda_enabled)
{
  cerr << "create " << allScans.size() << " k-d trees " << flush;
  int i;
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
  for (i = 0; i < (int)allScans.size(); i++) {
    allScans[i]->createTree(nns_method, cuda_enabled);
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
/*
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
#ifndef WITH_SCANSERVER
    closest = new KDCacheItem[Target->points_red_size];
#else //WITH_SCANSERVER
    closest = new KDCacheItem[const_cast<Scan*>(Target)->getCountReduced()];
#endif //WITH_SCANSERVER
    KDCache *nc = new KDCache;
    nc->item = closest;
    nc->SourceScanNr = Source->scanNr;
    nc->TargetScanNr = Target->scanNr;
    closest_cache.push_back(nc);                              // append cache
  }

  return closest;
  return 0;
}
  */

/**
 * Calculates Source\Target 
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

void Scan::getNoPairsSimple(vector <double*> &diff, 
					   Scan* Source, Scan* Target, 
					   int thread_num,
					   double max_dist_match2)
{
#ifndef WITH_SCANSERVER
  unsigned int numpts_target;
  KDtree *kd = 0;

  kd = new KDtree(Target->points_red, Target->points_red_size);
  numpts_target = Source->points_red_size;
  
    cout << "Max: " << max_dist_match2 << endl;
  for (unsigned int i = 0; i < numpts_target; i++) {

    double p[3];
    p[0] = Source->points_red[i][0];
    p[1] = Source->points_red[i][1];
    p[2] = Source->points_red[i][2];


    double *closest = kd->FindClosest(p, max_dist_match2, thread_num);
    if (!closest) {
	    diff.push_back(Source->points_red[i]);
	    //diff.push_back(closest);
    } 
  }
  
  delete kd;
  return;
#else //WITH_SCANSERVER
  throw runtime_error("Can't use Scan::getNoPairsSimple: kd-trees are incompatible with the new reduced points format");
#endif //WITH_SCANSERVER
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
#ifndef WITH_SCANSERVER
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
      centroid_m[0] += closest[0];
      centroid_m[1] += closest[1];
      centroid_m[2] += closest[2];	 
      centroid_d[0] += p[0];
      centroid_d[1] += p[1];
      centroid_d[2] += p[2];
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
#else //WITH_SCANSERVER
  throw runtime_error("Can't use Scan::getPtPairsSimple: kd-trees are incompatible with the new reduced points format");
#endif //WITH_SCANSERVER
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
				  int rnd, double max_dist_match2, double &sum,
				  double *centroid_m, double *centroid_d)
{
  // initialize centroids
  for(unsigned int i = 0; i < 3; ++i) {
    centroid_m[i] = 0;
    centroid_d[i] = 0;
  }
  
  // get point pairs
#ifndef WITH_SCANSERVER
  Source->kd->getPtPairs(pairs, Source->dalignxf,
      Target->points_red, 0, Target->points_red_size,
      thread_num,
      rnd, max_dist_match2, sum, centroid_m, centroid_d);
#else //WITH_SCANSERVER
  Source->getSearchTree()->getPtPairs(pairs, Source->dalignxf,
      Target->getXYZReduced(), 0, Target->getCountReduced(),
      thread_num,
      rnd, max_dist_match2, sum, centroid_m, centroid_d);
#endif //WITH_SCANSERVER
  
  // normalize centroids
  unsigned int size = pairs->size();
  if(size != 0) {
    for(unsigned int i = 0; i < 3; ++i) {
      centroid_m[i] /= size;
      centroid_d[i] /= size;
    }
  }
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
  // initialize centroids
  for(unsigned int i = 0; i < 3; ++i) {
    centroid_m[thread_num][i] = 0;
    centroid_d[thread_num][i] = 0;
  }
  
  // get point pairs
#ifndef WITH_SCANSERVER
  Source->kd->getPtPairs(&pairs[thread_num], Source->dalignxf,
    Target->points_red, thread_num * step, thread_num * step + step,
    thread_num,
    rnd, max_dist_match2, sum[thread_num],
    centroid_m[thread_num], centroid_d[thread_num]);
#else //WITH_SCANSERVER
  SearchTree* search = Source->getSearchTree();
  // differentiate between a meta scan (which has no reduced points) and a normal scan
  // if Source is also a meta scan it already has a special meta-kd-tree
  if(Target->meta_count) {
    for(unsigned int i = 0; i < Target->meta_parts.size(); ++i) {
      // determine step for each scan individually
      unsigned int max = Target->meta_parts[i]->getCountReduced();
      unsigned int step = max / OPENMP_NUM_THREADS;
      // call ptpairs for each scan and accumulate ptpairs, centroids and sum
      search->getPtPairs(&pairs[thread_num], Source->dalignxf,
        Target->meta_parts[i]->getXYZReduced(), step * thread_num, step * thread_num + step,
        thread_num,
        rnd, max_dist_match2, sum[thread_num],
        centroid_m[thread_num], centroid_d[thread_num]);
    }
  } else {
    search->getPtPairs(&pairs[thread_num], Source->dalignxf,
      Target->getXYZReduced(), thread_num * step, thread_num * step + step,
      thread_num,
      rnd, max_dist_match2, sum[thread_num],
      centroid_m[thread_num], centroid_d[thread_num]);
  }
#endif //WITH_SCANSERVER
  
  // normalize centroids
  unsigned int size = pairs[thread_num].size();
  if(size != 0) {
    for(unsigned int i = 0; i < 3; ++i) {
      centroid_m[thread_num][i] /= size;
      centroid_d[thread_num][i] /= size;
    }
  }
}


/**
 * Computes a search tree depending on the type this can be 
 * a k-d tree od a cached k-d tree
 */
void Scan::createTree(int nns_method, bool cuda_enabled)
{
#ifdef WITH_SCANSERVER
  // multiple threads will call this function at the same time because they all work on one pair of Scans, just let the first one (who sees a nullpointer) do the creation
  boost::lock_guard<boost::mutex> lock(m_mutex_create_tree);
  if(kd != 0) return;
  
  // make sure the original points are created before starting the measurement
  DataXYZ xyz_orig(getXYZReducedOriginal());
#else //WITH_SCANSERVER
  this->nns_method = nns_method;
  this->cuda_enabled = cuda_enabled;
  
  M4identity(dalignxf);
  
#if defined WITH_METRICS && !defined _OPENMP
  Timer t = ClientMetric::copy_original_time.start();
#endif //WITH_METRICS
  
  points_red_lum = new double*[points_red_size];
  for (int j = 0; j < points_red_size; j++) {
    points_red_lum[j] = new double[3];
    points_red_lum[j][0] = points_red[j][0];
    points_red_lum[j][1] = points_red[j][1];
    points_red_lum[j][2] = points_red[j][2];
  }

#if defined WITH_METRICS && !defined _OPENMP
  ClientMetric::copy_original_time.end(t);
#endif //WITH_METRICS
#endif //WITH_SCANSERVER

#if defined WITH_METRICS && (defined WITH_SCANSERVER || !defined _OPENMP)
  Timer tc = ClientMetric::create_tree_time.start();
#endif //WITH_METRICS
  
  switch(nns_method)
  {
#ifdef WITH_SCANSERVER
    case simpleKD:
      kd = new KDtreeManaged(this);
    break;
    case -1:
      throw runtime_error("Trying to build a SearchTree without setting a type, fix your program/tool for scanserver compability");
    break;
    default:
      throw runtime_error("SearchTree type not implemented");
// TODO: implement others
#else //WITH_SCANSERVER
    case simpleKD:
      kd = new KDtree(points_red_lum, points_red_size);
    break;
    case cachedKD:
      kd = new KDtree_cache(points_red_lum, points_red_size);
    break;
    case ANNTree:
      kd = new ANNtree(points_red_lum, points_red_size);
    break;
    case BOCTree:
      PointType pointtype;
      kd = new BOctTree<double>(points_red_lum, points_red_size, 10.0, pointtype, true);
    break;
#endif //WITH_SCANSERVER
  }
  
  if (cuda_enabled) createANNTree();

#if defined WITH_METRICS && (defined WITH_SCANSERVER || !defined _OPENMP)
  ClientMetric::create_tree_time.end(tc);
#endif //WITH_METRICS
}

#ifdef WITH_SCANSERVER
void Scan::createMetaTree(const vector<Scan*>& scans)
{
  // multiple threads will call this function at the same time because they all work on one pair of Scans, just let the first one (who sees a nullpointer) do the creation
  boost::lock_guard<boost::mutex> lock(m_mutex_create_tree);
  if(kd != 0) return;
  
#ifdef WITH_METRICS
  Timer t = ClientMetric::create_metatree_time.start();
#endif //WITH_METRICS
  
  kd = new KDtreeMetaManaged(scans);
  
#ifdef WITH_METRICS
  ClientMetric::create_metatree_time.end(t);
#endif //WITH_METRICS
}
#endif //WITH_SCANSERVER

void Scan::createANNTree()
{
  // TODO: metrics
#ifdef WITH_CUDA
#ifdef WITH_SCANSERVER
  // lazy
  unsigned int points_red_size = getCountReduced();
#endif //WITH_SCANSERVER
  if(!ann_kd_tree){
    // TODO: make this tree available with the new scanserver, if compiled with CUDA this will break
    ann_kd_tree = new ANNkd_tree(points_red_lum, points_red_size, 3, 1, ANN_KD_STD);
    cout << "Cuda tree was generated with " << points_red_size << " points" << endl;
  } else {
    cout << "Cuda tree exists. No need for another creation" << endl;
  }
#endif
}


/**
 * Delete the search tree
 */
void Scan::deleteTree()
{
#ifndef WITH_SCANSERVER
  for (unsigned int j = 0; j < points_red_size; j++) {
    delete [] points_red_lum[j];
  }
  delete [] points_red_lum;
#endif //WITH_SCANSERVER
  
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
void Scan::readScans(IOType type,
		     int start, int end, string &_dir, int maxDist, int minDist, 
		     bool openFileForWriting)
{
#ifdef WITH_METRICS
  Timer t = ClientMetric::read_scan_time.start();
#endif //WITH_METRICS
  
#ifndef WITH_SCANSERVER
  outputFrames = openFileForWriting;
  dir = _dir;
  double eu[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  vector <Point> ptss;
  int _fileNr;
  scanIOwrapper my_ScanIO(type);

  // read Scan-by-scan until no scan is available anymore
  while ((_fileNr = my_ScanIO.readScans(start, end, dir, maxDist, minDist, eu, ptss)) != -1) {
    Scan *currentScan = new Scan(eu, maxDist);
    
    currentScan->fileNr = _fileNr;

    currentScan->points = ptss;    // copy points
    ptss.clear();                  // clear points
    allScans.push_back(currentScan);
  }
#else //WITH_SCANSERVER
  ClientInterface* client = ClientInterface::getInstance();
  shared_scans = client->readDirectory(_dir.c_str(), type, start, end);
  
  for(ScanVector::iterator it = shared_scans->begin(); it != shared_scans->end(); ++it) {
    // add a scan with reference on the shared scan
    SharedScan* shared = it->get();
    Scan* scan = new Scan(shared);
    allScans.push_back(scan);
    
    shared->setRangeParameters(maxDist, minDist);
  }
#endif //WITH_SCANSERVER
  
#ifdef WITH_METRICS
  ClientMetric::read_scan_time.end(t);
#endif //WITH_METRICS
}

void Scan::toGlobal(double voxelSize, int nrpts) {
  calcReducedPoints(voxelSize, nrpts);
  transform(transMatOrg, INVALID);
}

void Scan::readScansRedSearch(IOType type,
		     int start, int end, string &_dir, int maxDist, int minDist, 
						double voxelSize, int nrpts, // reduction parameters
						int nns_method, bool cuda_enabled, 
						bool openFileForWriting)
{
#ifdef WITH_METRICS
  Timer t = ClientMetric::read_scan_time.start();
#endif //WITH_METRICS
  
#ifndef WITH_SCANSERVER
  outputFrames = openFileForWriting;
  dir = _dir;
  double eu[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  vector <Point> ptss;
  int _fileNr;
  scanIOwrapper my_ScanIO(type);

#ifndef _MSC_VER 
#ifdef _OPENMP
#pragma omp parallel
  {
#pragma omp single nowait
    {
#endif
#endif
      // read Scan-by-scan until no scan is available anymore
      while ((_fileNr = my_ScanIO.readScans(start, end, dir, maxDist, minDist, eu, ptss)) != -1) {
        Scan *currentScan = new Scan(eu, maxDist);
        currentScan->fileNr = _fileNr;

        currentScan->points = ptss;    // copy points
        ptss.clear();                  // clear points
        allScans.push_back(currentScan);

#ifndef _MSC_VER 
#ifdef _OPENMP
#pragma omp task
#endif
#endif
        {
          cout << "reducing scan " << currentScan->fileNr << " and creating searchTree" << endl;
          currentScan->calcReducedPoints(voxelSize, nrpts);
          currentScan->transform(currentScan->transMatOrg, INVALID); //transform points to initial position
          currentScan->clearPoints();
          currentScan->createTree(nns_method, cuda_enabled);
        }
      }
#ifndef _MSC_VER 
#ifdef _OPENMP
    }
  }
#pragma omp taskwait
#endif
#endif

#else //WITH_SCANSERVER
  ClientInterface* client = ClientInterface::getInstance();
  shared_scans = client->readDirectory(_dir.c_str(), type, start, end);
  
  for(ScanVector::iterator it = shared_scans->begin(); it != shared_scans->end(); ++it) {
    // add a scan with reference on the shared scan
    SharedScan* shared = it->get();
    Scan* scan = new Scan(shared);
    allScans.push_back(scan);
    
    // Reduction and tree creation are on-demand
    scan->reduction_voxelSize = voxelSize;
    scan->reduction_nrpts = nrpts;
    scan->nns_method = nns_method;
    scan->cuda_enabled = cuda_enabled;
    
    // set parameters to invalidate old cache data
    shared->setRangeParameters(maxDist, minDist);
    stringstream s;
    s << voxelSize << " " << nrpts << " " << scan->transMatOrg;
    shared->setReductionParameters(s.str().c_str());
  }
#endif //WITH_SCANSERVER
  
#ifdef WITH_METRICS
  ClientMetric::read_scan_time.end(t);
#endif //WITH_METRICS
}

#ifndef WITH_SCANSERVER
Scan::scanIOwrapper::scanIOwrapper(IOType type){
  // load the lib
  string lib_string;
  try {
    lib_string = io_type_to_libname(type);
  } catch (...) { // runtime_error
    cerr << "Don't recognize format " << type << endl;
    return;
  }

#ifdef WIN32
  lib_string += ".dll";	
#elif __APPLE__
  lib_string = "lib/lib" + lib_string + ".dylib";	
#else
  lib_string = "lib" + lib_string + ".so";	
#endif
  cerr << "Loading shared lib " << lib_string;
  
#ifdef _MSC_VER
  hinstLib = LoadLibrary(lib_string.c_str());
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
  ptrScanIO = dlopen(lib_string.c_str(), RTLD_LAZY);

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
  my_ScanIO = create_ScanIO();
}

Scan::scanIOwrapper::~scanIOwrapper(){
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
  const char* dlsym_error = dlerror();
  if (dlsym_error) {
    cerr << "Cannot load symbol create: " << dlsym_error << endl;
    exit(-1);
  }
  destroy_ScanIO(my_ScanIO);
#endif
}

int Scan::scanIOwrapper::readScans(int start, int end, string &dir, int maxDist, int minDist,double *euler, vector<Point> &ptss) {
  return my_ScanIO->readScans(start, end, dir, maxDist, minDist, euler, ptss);
}
#endif //WITH_SCANSERVER

#ifdef WITH_SCANSERVER

void Scan::clearScans()
{
  // clean up the scan vector
  for(std::vector<Scan*>::iterator it = allScans.begin(); it != allScans.end(); ++it)
    delete *it;
  allScans.clear();
  // remove the shared scan vector
  ClientInterface* client = ClientInterface::getInstance();
  client->closeDirectory(shared_scans);
}

void Scan::remove(Scan* scan)
{
  // delete scan from the vector
  vector <Scan*>::iterator Iter;
  for(vector<Scan*>::iterator it = allScans.begin(); it != allScans.end(); ++it) {
    if (*it == scan) {
      allScans.erase(it);
      break;
    }
  }
  delete scan;
}

unsigned int Scan::getMaxCountReduced()
{
  unsigned int max = 0;
  for(std::vector<Scan*>::iterator it = allScans.begin(); it != allScans.end(); ++it) {
    unsigned int count = (*it)->getCountReduced();
    if(count > max)
      max = count;
  }
  return max;
}

Scan::Scan(SharedScan* shared_scan)
{
  init();
  scanNr = numberOfScans++;
  
  m_shared_scan = shared_scan;
  
  // importing
  int maxDist = (int)(m_shared_scan->getMaxDist());
  maxDist2 = (maxDist != -1 ? sqr(maxDist) : maxDist);
  
  // request pose from the shared scan
  double* euler = getPose();
  rPos[0] = euler[0];
  rPos[1] = euler[1];
  rPos[2] = euler[2];
  rPosTheta[0] = euler[3];
  rPosTheta[1] = euler[4];
  rPosTheta[2] = euler[5];
  
  // write original pose matrix
  EulerToMatrix4(euler, &euler[3], transMatOrg);
  
  // initialize transform matrices from the original one, could just copy transMatOrg to transMat instead
  transformMatrix(transMatOrg);
  
  // reset the delta align matrix to represent only the transformations after local-to-global (transMatOrg) one
  M4identity(dalignxf);
}

void Scan::init()
{
  // managed stuff
  m_shared_scan = 0;
  m_count = 0;
  m_count_reduced = 0;
  m_show_count = 0;
  
  // meta stuff
  meta_count = 0;
  
  // poses and transformations
  rPos[0] = 0;
  rPos[1] = 0;
  rPos[2] = 0;
  rPosTheta[0] = 0;
  rPosTheta[1] = 0;
  rPosTheta[2] = 0;
  M4identity(transMat);
  M4identity(transMatOrg);
  M4identity(dalignxf);
  
  // trees and methods
  cuda_enabled = false;
  nns_method = -1;
  kd = 0;
  ann_kd_tree = 0;
  
  // reduction on-demand
  reduction_voxelSize = 0.0;
  reduction_nrpts = 0;
  
  // small stuff
  scanNr = 0;
  maxDist2 = -1;
}

double* Scan::getPose() const
{
  // for artificially created Scan instances this operation is invalid
  if(m_shared_scan == 0)
    throw runtime_error("No shared scan set.");
  return m_shared_scan->getPose();
}

unsigned int Scan::getCount()
{
  // if not already buffered (which it should be), read the whole scan to determine point count
  if(m_count == 0)
    m_count = m_shared_scan->getXYZ().size();
  return m_count;
}

DataXYZ Scan::getXYZ() const
{
  // for artificially created Scan instances this operation is invalid
  if(m_shared_scan == 0)
    throw runtime_error("No shared scan set.");
  return m_shared_scan->getXYZ();
}

DataRGB Scan::getRGB() const
{
  // for artificially created Scan instances this operation is invalid
  if(m_shared_scan == 0)
    throw runtime_error("No shared scan set.");
  return m_shared_scan->getRGB();
}

DataReflectance Scan::getReflectance() const
{
  // for artificially created Scan instances this operation is invalid
  if(m_shared_scan == 0)
    throw runtime_error("No shared scan set.");
  return m_shared_scan->getReflectance();
}

DataAmplitude Scan::getAmplitude() const
{
  // for artificially created Scan instances this operation is invalid
  if(m_shared_scan == 0)
    throw runtime_error("No shared scan set.");
  return m_shared_scan->getAmplitude();
}

DataType Scan::getType() const
{
  // for artificially created Scan instances this operation is invalid
  if(m_shared_scan == 0)
    throw runtime_error("No shared scan set.");
  return m_shared_scan->getType();
}

DataDeviation Scan::getDeviation() const
{
  // for artificially created Scan instances this operation is invalid
  if(m_shared_scan == 0)
    throw runtime_error("No shared scan set.");
  return m_shared_scan->getDeviation();
}

unsigned int Scan::getCountReduced()
{
  // if this is a meta scan we return the meta count (no shared scan exists)
  if(meta_count != 0)
    return meta_count;
  // if not already buffered (which it should be), read the reduced scan to determine point count
  if(m_count_reduced == 0)
    calcReducedOnDemand();
  return m_count_reduced;
}

DataXYZ Scan::getXYZReduced()
{
  // for artificially created Scan instances this operation is invalid
  if(m_shared_scan == 0)
    throw runtime_error("No shared scan set.");
  // if the count is zero the reduced points have to be created
  if(m_count_reduced == 0)
    calcReducedOnDemand();
  return m_shared_scan->getXYZReduced();
}

DataXYZ Scan::getXYZReducedOriginal()
{
  // for artificially created Scan instances this operation is invalid
  if(m_shared_scan == 0)
    throw runtime_error("No shared scan set.");
  // if the count is zero the reduced points have to be created
  if(m_count_reduced == 0)
    calcReducedOnDemand();
  return m_shared_scan->getXYZReducedOriginal();
}

SearchTree* Scan::getSearchTree()
{
  // if the search tree hasn't been created yet, calculate everything
  if(kd == 0) {
    // calculate the meta tree on demand, points in the contained scans may not change anyways
    if(meta_count)
      createMetaTree(meta_parts);
    else
      createTree(nns_method, cuda_enabled);
  }
  return kd;
}

void Scan::calcReducedShow(double voxelSize, int nrpts)
{
  // set show reduction parameters and cause invalidation event in case of mismatch
  stringstream str;
  str << voxelSize << " " << nrpts;
  m_shared_scan->setShowReductionParameters(str.str().c_str());
  show_red_voxelSize = voxelSize;
  show_red_nrpts = nrpts;
}
  
TripleArray<float> Scan::getXYZReducedShow()
{
// TODO: multithread guard and onDemand function
  if(m_show_count == 0) {
    if(m_shared_scan->getXYZReducedShow()) {
      // check the cache for a valid version after show reduction parameters have been set
      m_show_count = m_shared_scan->getXYZReducedShow().size();
    } else {
      // create an octtree reduction from full points
      BOctTree<double> *oct = new BOctTree<double>(Array<double>(getXYZ()).get(), getCount(), show_red_voxelSize);

      vector<double*> center;
      center.clear();

      if(show_red_nrpts > 0) {
        if(show_red_nrpts == 1) {
          oct->GetOctTreeRandom(center);
        } else {
          oct->GetOctTreeRandom(center, show_red_nrpts);
        }
      } else {
        oct->GetOctTreeCenter(center);
      }

      unsigned int size = center.size();
      TripleArray<float> xyz_r(m_shared_scan->createXYZReducedShow(size));
      for(unsigned int i = 0; i < size; ++i) {
        for(unsigned int j = 0; j < 3; ++j) {
          xyz_r[i][j] = center[i][j];
        }
      }
      m_show_count = size;
      
      delete oct;
    }
  }
  return m_shared_scan->getXYZReducedShow();
}

void Scan::setOcttreeParameters(const char* params)
{
  m_shared_scan->setOcttreeParameters(params);
}

CacheDataAccess Scan::getOcttree()
{
  return m_shared_scan->getOcttree();
}

CacheDataAccess Scan::copyOcttreeToCache(BOctTree<float>* oct)
{
  // set parameters and stuff
  // TODO?!
  // check if already available
  if(!m_shared_scan->getOcttree()) {
    unsigned int size = oct->getMemorySize();
    unsigned char* mem_ptr = reinterpret_cast<unsigned char*>(
      &( DataOct(m_shared_scan->createOcttree(size)).get() )
    );
    new(mem_ptr) BOctTree<float>(*oct, mem_ptr, size);
  }
  return m_shared_scan->getOcttree();
}

void Scan::copyReducedToOriginal()
{
#ifdef WITH_METRICS
  Timer t = ClientMetric::copy_original_time.start();
#endif //WITH_METRICS
  
  DataXYZ xyz_r(m_shared_scan->getXYZReduced());
  unsigned int size = xyz_r.size();
  DataXYZ xyz_r_orig(m_shared_scan->createXYZReducedOriginal(size));
  for(unsigned int i = 0; i < size; ++i) {
    for(unsigned int j = 0; j < 3; ++j) {
      xyz_r_orig[i][j] = xyz_r[i][j];
    }
  }
  
#ifdef WITH_METRICS
  ClientMetric::copy_original_time.end(t);
#endif //WITH_METRICS
}  

void Scan::copyOriginalToReduced()
{
#ifdef WITH_METRICS
  Timer t = ClientMetric::copy_original_time.start();
#endif //WITH_METRICS
  
  DataXYZ xyz_r_orig(m_shared_scan->getXYZReducedOriginal());
  unsigned int size = xyz_r_orig.size();
  DataXYZ xyz_r(m_shared_scan->createXYZReduced(size));
  for(unsigned int i = 0; i < size; ++i) {
    for(unsigned int j = 0; j < 3; ++j) {
      xyz_r[i][j] = xyz_r_orig[i][j];
    }
  }
  // update buffered count reduced, in case this is a cached copy-back
  m_count_reduced = xyz_r.size();
  
#ifdef WITH_METRICS
  ClientMetric::copy_original_time.end(t);
#endif //WITH_METRICS
}

void Scan::calcReducedOnDemand()
{
  // multiple threads will call this function at the same time because they all work on one pair of Scans, just let the first one (who sees count as zero) do the reduction
  boost::lock_guard<boost::mutex> lock(m_mutex_reduction);
  if(m_count_reduced != 0) return;
  
#ifdef WITH_METRICS
  Timer t = ClientMetric::on_demand_reduction_time.start();
#endif //WITH_METRICS
  
  // try to load existing reduced points first
  DataXYZ xyz_r_orig(m_shared_scan->getXYZReducedOriginal());
  if(xyz_r_orig) {
  // if a valid (in terms of parameters) copy exists, use it instead of loading all points and reducing them
    copyOriginalToReduced();
    // no need for transform, the copy already is, and so is the matrix
  } else {
    // create reduced points and transform to initial position
    calcReducedPoints(reduction_voxelSize, reduction_nrpts);
    // only transform the points, matrix is initialized already
    transformReduced(transMatOrg);
    // copy reduced points as original points which aren't transformed in matching iterations
    // original points are copied back into reduced points on repeated startups if the parameters match
    copyReducedToOriginal();
  }
  
#ifdef WITH_METRICS
  ClientMetric::on_demand_reduction_time.end(t);
#endif //WITH_METRICS
}

void Scan::clearFrames()
{
  // for artificially created Scan instances this operation is ignored
  if(m_shared_scan == 0)
    return;
  m_shared_scan->clearFrames();
}

const FrameVector& Scan::getFrames()
{
  // for artificially created Scan instances this operation is invalid
  if(m_shared_scan == 0)
    throw runtime_error("No shared scan set.");
  return m_shared_scan->getFrames();
}

void Scan::saveFrames()
{
  // for artificially created Scan instances this operation is ignored
  if(m_shared_scan == 0)
    return;
  m_shared_scan->saveFrames();
}

void Scan::addFrame(AlgoType type)
{
  // for artificially created Scan instances this operation is ignored
  if(m_shared_scan == 0)
    return;
  m_shared_scan->addFrame(transMat, static_cast<unsigned int>(type));
}

#endif //WITH_SCANSERVER
