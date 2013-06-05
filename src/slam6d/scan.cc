/*
 * scan implementation
 *
 * Copyright (C) by the 3DTK contributors
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file scan.cc
 * @brief the implementation for all scans (basic/managed)
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany. 
 * @author Kai Lingemann. Inst. of CS. University of Osnabrueck, Germany.
 * @author Dorit Borrmann. Jacobs University Bremen gGmbH, Germany. 
 * @author Jan Elseberg. Jacobs University Bremen gGmbH, Germany. 
 * @author Thomas Escher. Inst. of CS. University of Osnabrueck, Germany.
 */

#include "slam6d/scan.h"

#include "slam6d/basicScan.h"
#include "slam6d/managedScan.h"
#include "slam6d/metaScan.h"
#include "slam6d/searchTree.h"
#include "slam6d/kd.h"
#include "slam6d/Boctree.h"
#include "slam6d/globals.icc"

#include "slam6d/normals.h"

#ifdef WITH_METRICS
#include "slam6d/metrics.h"
#endif

#ifdef _MSC_VER
#define _NO_PARALLEL_READ
#endif

#ifdef __APPLE__
#define _NO_PARALLEL_READ
#endif

using std::vector;


vector<Scan*> Scan::allScans;
bool Scan::scanserver = false;


void Scan::openDirectory(bool scanserver,
                         const std::string& path,
                         IOType type,
                         int start,
                         int end)
{
  Scan::scanserver = scanserver;
  if (scanserver)
    ManagedScan::openDirectory(path, type, start, end);
  else
    BasicScan::openDirectory(path, type, start, end);
}

void Scan::closeDirectory()
{
  if (scanserver)
    ManagedScan::closeDirectory();
  else
    BasicScan::closeDirectory();
}

Scan::Scan()
{
  unsigned int i;

  // pose and transformations
  for(i = 0; i < 3; ++i) rPos[i] = 0;
  for(i = 0; i < 3; ++i) rPosTheta[i] = 0;
  for(i = 0; i < 4; ++i) rQuat[i] = 0;
  M4identity(transMat);
  M4identity(transMatOrg);
  M4identity(dalignxf);

  // trees and reduction methods
  nns_method = -1;
  kd = 0;

  // reduction on-demand
  reduction_voxelSize = 0.0;
  reduction_nrpts = 0;
  reduction_pointtype = PointType();

  // flags
  m_has_reduced = false;

  // octtree
  octtree_reduction_voxelSize = 0.0;
  octtree_voxelSize = 0.0;
  octtree_pointtype = PointType();
  octtree_loadOct = false;
  octtree_saveOct = false;
}

Scan::~Scan()
{
  if (kd) delete kd;
}

void Scan::setReductionParameter(double voxelSize,
                                 int nrpts,
                                 PointType pointtype)
{
  reduction_voxelSize = voxelSize;
  reduction_nrpts = nrpts;
  reduction_pointtype = pointtype;
}

void Scan::setSearchTreeParameter(int nns_method)
{
  searchtree_nnstype = nns_method;
}

void Scan::setOcttreeParameter(double reduction_voxelSize,
                               double voxelSize,
                               PointType pointtype,
                               bool loadOct,
                               bool saveOct)
{
  octtree_reduction_voxelSize = reduction_voxelSize;
  octtree_voxelSize = voxelSize;
  octtree_pointtype = pointtype;
  octtree_loadOct = loadOct;
  octtree_saveOct = saveOct;
}

void Scan::clear(unsigned int types)
{
  if(types & DATA_XYZ) clear("xyz");
  if(types & DATA_RGB) clear("rgb");
  if(types & DATA_REFLECTANCE) clear("reflectance");
  if(types & DATA_TEMPERATURE) clear("temperature");
  if(types & DATA_AMPLITUDE) clear("amplitude");
  if(types & DATA_TYPE) clear("type");
  if(types & DATA_DEVIATION) clear("deviation");
}

SearchTree* Scan::getSearchTree()
{
  // if the search tree hasn't been created yet, calculate everything
  if(kd == 0) {
    createSearchTree();
  }
  return kd;
}

void Scan::toGlobal() {
  calcReducedPoints();
  transform(transMatOrg, INVALID);
}

/**
 * Computes a search tree depending on the type.
 */
void Scan::createSearchTree()
{
  // multiple threads will call this function at the same time because they
  // all work on one pair of Scans, just let the first one
  // (who sees a null pointer)
  // do the creation
  boost::lock_guard<boost::mutex> lock(m_mutex_create_tree);
  if(kd != 0) return;

  // make sure the original points are created before starting the measurement
  DataXYZ xyz_orig(get("xyz reduced original"));

#ifdef WITH_METRICS
  Timer tc = ClientMetric::create_tree_time.start();
#endif //WITH_METRICS

  createSearchTreePrivate();
  
#ifdef WITH_METRICS
  ClientMetric::create_tree_time.end(tc);
#endif //WITH_METRICS
}

void Scan::calcReducedOnDemand()
{
  // multiple threads will call this function at the same time
  // because they all work on one pair of Scans,
  // just let the first one (who sees count as zero) do the reduction
  boost::lock_guard<boost::mutex> lock(m_mutex_reduction);
  if(m_has_reduced) return;

#ifdef WITH_METRICS
  Timer t = ClientMetric::on_demand_reduction_time.start();
#endif //WITH_METRICS

  calcReducedOnDemandPrivate();

  m_has_reduced = true;

#ifdef WITH_METRICS
  ClientMetric::on_demand_reduction_time.end(t);
#endif //WITH_METRICS
}

void Scan::calcNormalsOnDemand()
{
  // multiple threads will call this function at the same time
  // because they all work on one pair of Scans,
  // just let the first one (who sees count as zero) do the reduction
  boost::lock_guard<boost::mutex> lock(m_mutex_normals);
  if(m_has_normals) return;
  calcNormalsOnDemandPrivate();
  m_has_normals = true;
}

void Scan::copyReducedToOriginal()
{
#ifdef WITH_METRICS
  Timer t = ClientMetric::copy_original_time.start();
#endif //WITH_METRICS

  DataXYZ xyz_reduced(get("xyz reduced"));
  unsigned int size = xyz_reduced.size();
  DataXYZ xyz_reduced_orig(create("xyz reduced original",
                                  sizeof(double)*3*size));
  for(unsigned int i = 0; i < size; ++i) {
    for(unsigned int j = 0; j < 3; ++j) {
      xyz_reduced_orig[i][j] = xyz_reduced[i][j];
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

  DataXYZ xyz_reduced_orig(get("xyz reduced original"));
  unsigned int size = xyz_reduced_orig.size();
  DataXYZ xyz_reduced(create("xyz reduced", sizeof(double)*3*size));
  for(unsigned int i = 0; i < size; ++i) {
    for(unsigned int j = 0; j < 3; ++j) {
      xyz_reduced[i][j] = xyz_reduced_orig[i][j];
    }
  }

#ifdef WITH_METRICS
  ClientMetric::copy_original_time.end(t);
#endif //WITH_METRICS
}


/**
 * Computes normals for all points
 */
void Scan::calcNormals()
{
  cout << "calcNormals" << endl;
  DataXYZ xyz(get("xyz"));
  DataNormal xyz_normals(create("normal", sizeof(double)*3*xyz.size()));
  if(xyz.size() == 0) throw
      runtime_error("Could not calculate reduced points, XYZ data is empty");
    
  vector<Point> points;
  points.reserve(xyz.size());
  vector<Point> normals;
  normals.reserve(xyz.size());
  for(unsigned int j = 0; j < xyz.size(); j++) {
    points.push_back(Point(xyz[j][0], xyz[j][1], xyz[j][2]));
  }
  const int K_NEIGHBOURS = 10;  //@FIXME
  calculateNormalsApxKNN(normals, points, K_NEIGHBOURS, get_rPos(), 1.0);
  // calculateNormalsKNN(normals, points, K_NEIGHBOURS, get_rPos());
  for (unsigned int i = 0; i < normals.size(); ++i) {
    xyz_normals[i][0] = normals[i].x;
    xyz_normals[i][1] = normals[i].y;
    xyz_normals[i][2] = normals[i].z;
  }
  cout << "calcNormals done" << endl;
}
    
/**
 * Computes an octtree of the current scan, then getting the
 * reduced points as the centers of the octree voxels.
 */
void Scan::calcReducedPoints()
{
#ifdef WITH_METRICS
  Timer t = ClientMetric::scan_load_time.start();
#endif //WITH_METRICS


  // get xyz to start the scan load, separated here for time measurement
  DataXYZ xyz(get("xyz"));
  DataXYZ xyz_normals(DataPointer(0, 0));
  if (reduction_pointtype.hasNormal()) {
    DataXYZ my_xyz_normals(get("normal"));
    xyz_normals =  my_xyz_normals;
  }
  DataReflectance reflectance(DataPointer(0, 0));
  if (reduction_pointtype.hasReflectance()) {
    DataReflectance my_reflectance(get("reflectance"));
    reflectance = my_reflectance;
  }
  
#ifdef WITH_METRICS
    ClientMetric::scan_load_time.end(t);
    Timer tl = ClientMetric::calc_reduced_points_time.start();
#endif //WITH_METRICS
  
  if(reduction_voxelSize <= 0.0) {
    // copy the points
    DataXYZ xyz_reduced(create("xyz reduced", sizeof(double)*3*xyz.size()));
    for(unsigned int i = 0; i < xyz.size(); ++i) {
      for(unsigned int j = 0; j < 3; ++j) {
        xyz_reduced[i][j] = xyz[i][j];
      }
    }
    if (reduction_pointtype.hasReflectance()) {
      DataReflectance reflectance_reduced(create("reflectance reduced",
                                        sizeof(float)*reflectance.size()));
      for(unsigned int i = 0; i < xyz.size(); ++i) {
           reflectance_reduced[i] = reflectance[i];
        }
    }
    if (reduction_pointtype.hasNormal()) {
      DataNormal normal_reduced(create("normal reduced",
                                       sizeof(double)*3*xyz.size()));      
        for(unsigned int i = 0; i < xyz.size(); ++i) {
          for(unsigned int j = 0; j < 3; ++j) {
            normal_reduced[i][j] = xyz_normals[i][j];
          }
        }
    }

  } else {

    double **xyz_in = new double*[xyz.size()];
    for (unsigned int i = 0; i < xyz.size(); ++i) {
      xyz_in[i] = new double[reduction_pointtype.getPointDim()];
      unsigned int j = 0;
      for (; j < 3; ++j) 
        xyz_in[i][j] = xyz[i][j];
      if (reduction_pointtype.hasReflectance())
        xyz_in[i][j++] = reflectance[i];
      if (reduction_pointtype.hasNormal())
        for (unsigned int l = 0; l < 3; ++l) 
          xyz_in[i][j++] = xyz_normals[i][l];
    }

    // start reduction
    // build octree-tree from CurrentScan
    // put full data into the octtree
    BOctTree<double> *oct = new BOctTree<double>(xyz_in,
                                                 xyz.size(),
                                                 reduction_voxelSize,
                                                 reduction_pointtype);      

    vector<double*> center;
    center.clear();
    if (reduction_nrpts > 0) {
      if (reduction_nrpts == 1) {
        oct->GetOctTreeRandom(center);
      } else {
        oct->GetOctTreeRandom(center, reduction_nrpts);
      }
    } else {
        oct->GetOctTreeCenter(center);
    }
    
    // storing it as reduced scan
    unsigned int size = center.size();
    DataXYZ xyz_reduced(create("xyz reduced", sizeof(double)*3*size));
    DataReflectance reflectance_reduced(DataPointer(0, 0));
    DataNormal normal_reduced(DataPointer(0, 0)); 
    if (reduction_pointtype.hasReflectance()) {
      DataReflectance my_reflectance_reduced(create("reflectance reduced",
                                                    sizeof(float)*size));
      reflectance_reduced = my_reflectance_reduced;
    }
    if (reduction_pointtype.hasNormal()) {
      DataNormal my_normal_reduced(create("normal reduced",
                                          sizeof(double)*3*size));
      normal_reduced = my_normal_reduced; 
    }
    for(unsigned int i = 0; i < size; ++i) {
      unsigned int j = 0;
      for (; j < 3; ++j) 
        xyz_reduced[i][j] = center[i][j];
      if (reduction_pointtype.hasReflectance())
        reflectance_reduced[i] = center[i][j++];
      if (reduction_pointtype.hasNormal())
        for (unsigned int l = 0; l < 3; ++l) 
          normal_reduced[i][l] = center[i][j++];
    }
    delete oct;
    for(size_t i = 0; i < xyz.size(); i++) {
      delete[] xyz_in[i];
    }
    delete[] xyz_in;
  }

#ifdef WITH_METRICS
    ClientMetric::calc_reduced_points_time.end(tl);
#endif //WITH_METRICS  
}


/**
 * Merges the scan's intrinsic coordinates with the robot position.
 * @param prevScan The scan that's transformation is extrapolated,
 * i.e., odometry extrapolation
 *
 * For additional information see the following paper (jfr2007.pdf):
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
  M4inv(prevScan->get_transMatOrg(), tempMat);
  MMult(prevScan->get_transMat(), tempMat, deltaMat);
  // apply delta transformation of the previous scan
  transform(deltaMat, INVALID); 
}

/**
 * The method transforms all points with the given transformation matrix.
 */
void Scan::transformAll(const double alignxf[16])
{
  DataXYZ xyz(get("xyz"));
  unsigned int i=0 ;
  //  #pragma omp parallel for
  for(; i < xyz.size(); ++i) {
    transform3(alignxf, xyz[i]);
  }
  // TODO: test for ManagedScan compability,
  // may need a touch("xyz") to mark saving the new values
}

//! Internal function of transform which alters the reduced points
void Scan::transformReduced(const double alignxf[16])
{
#ifdef WITH_METRICS
  Timer t = ClientMetric::transform_time.start();
#endif //WITH_METRICS

  DataXYZ xyz_reduced(get("xyz reduced"));
  unsigned int i=0;
  // #pragma omp parallel for
  for( ; i < xyz_reduced.size(); ++i) {
    transform3(alignxf, xyz_reduced[i]);
  }

  DataNormal normal_reduced(get("normal reduced"));
  for (unsigned int i = 0; i < normal_reduced.size(); ++i) {
    transform3normal(alignxf, normal_reduced[i]);
  }


#ifdef WITH_METRICS
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
 * Transforms the scan by a given transformation and writes a new frame.
 * The idea is to write for every transformation in all files,
 * such that the show program is able to determine,
 * which scans have to be drawn in which color. Hidden scans
 * (or later processed scans) are written with INVALID.
 *
 * @param alignxf Transformation matrix
 * @param colour Specifies which colour should the written to the frames file
 * @param islum Is the transformtion part of LUM, i.e., all scans
 *              are transformed?
 *              In this case only LUM transformation is stored, otherwise all
 *              scans are processed
 *        -1  no transformation is stored
 *         0  ICP transformation
 *         1  LUM transformation, all scans except last scan
 *         2  LUM transformation, last scan only
 */
void Scan::transform(const double alignxf[16], const AlgoType type, int islum)
{
  MetaScan* meta = dynamic_cast<MetaScan*>(this);
  
  if(meta) {
    for(unsigned int i = 0; i < meta->size(); ++i) {
      meta->getScan(i)->transform(alignxf, type, -1);
    }
  }

#ifdef TRANSFORM_ALL_POINTS
  transformAll(alignxf);
#endif //TRANSFORM_ALL_POINTS

#ifdef DEBUG
  cerr << alignxf << endl;
  cerr << "(" << rPos[0] << ", " << rPos[1] << ", " << rPos[2] << ", "
       << rPosTheta[0] << ", " << rPosTheta[1] << ", " << rPosTheta[2]
       << ") ---> ";
#endif

  // transform points
  transformReduced(alignxf);

  // update matrices
  transformMatrix(alignxf);

  // store transformation in frames
  if(type != INVALID) {
#ifdef WITH_METRICS
    Timer t = ClientMetric::add_frames_time.start();
#endif //WITH_METRICS
    bool in_meta;
    MetaScan* meta = dynamic_cast<MetaScan*>(this);
    int found = 0;
    unsigned int scans_size = allScans.size();

    switch (islum) {
    case -1:
      // write no tranformation
      break;
    case 0:
      for(unsigned int i = 0; i < scans_size; ++i) {
        Scan* scan = allScans[i];
        in_meta = false;
        if(meta) {
          for(unsigned int j = 0; j < meta->size(); ++j) {
            if(meta->getScan(j) == scan) {
              found = i;
              in_meta = true;
            }
          }
        }

        if(scan == this || in_meta) {
          found = i;
          scan->addFrame(type);
        } else {
          if(found == 0) {
            scan->addFrame(ICPINACTIVE);
          } else {
            scan->addFrame(INVALID);
          }
        }
      }
      break;
    case 1:
      addFrame(type);
      break;
    case 2:
      for(unsigned int i = 0; i < scans_size; ++i) {
        Scan* scan = allScans[i];
        if(scan == this) {
          found = i;
          addFrame(type);
          allScans[0]->addFrame(type);
          continue;
        }
        if (found != 0) {
          scan->addFrame(INVALID);
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
 * Transforms the scan by a given transformation and writes a new frame.
 * The idea is to write for every transformation in all files, such that
 * the show program is able to determine, whcih scans have to be drawn
 * in which color. Hidden scans (or later processed scans) are written
 * with INVALID.
 *
 * @param alignQuat Quaternion for the rotation
 * @param alignt    Translation vector
 * @param colour Specifies which colour should the written to the frames file
 * @param islum Is the transformtion part of LUM, i.e., all scans are
 *              transformed?
 *              In this case only LUM transformation is stored, otherwise
 *              all scans are processed
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
void Scan::transformToEuler(double rP[3],
                            double rPT[3],
                            const AlgoType type,
                            int islum)
{
#ifdef WITH_METRICS
  // called in openmp context in lum6Deuler.cc:422
  ClientMetric::transform_time.set_threadsafety(true);
  ClientMetric::add_frames_time.set_threadsafety(true);
#endif //WITH_METRICS

  double tinv[16];
  double alignxf[16];
  M4inv(transMat, tinv);
  transform(tinv, INVALID);
  EulerToMatrix4(rP, rPT, alignxf);
  transform(alignxf, type, islum);

#ifdef WITH_METRICS
  ClientMetric::transform_time.set_threadsafety(false);
  ClientMetric::add_frames_time.set_threadsafety(false);
#endif //WITH_METRICS
}

/**
 * Transforms the scan, so that the given Euler angles
 * prepresent the next pose.
 *
 * @param rP Translation to which this scan will be set to
 * @param rPQ Orientation as Quaternion to which this scan will be set
 * @param islum Is the transformation part of LUM?
 */
void Scan::transformToQuat(double rP[3],
                           double rPQ[4],
                           const AlgoType type,
                           int islum)
{
  double tinv[16];
  double alignxf[16];
  M4inv(transMat, tinv);
  transform(tinv, INVALID);
  QuatToMatrix4(rPQ, rP, alignxf);
  transform(alignxf, type, islum);
}

/**
 * Calculates Source\Target
 * Calculates a set of corresponding point pairs and returns them. It
 * computes the k-d trees and deletes them after the pairs have been
 * found. This slow function should be used only for testing
 *
 * @param pairs The resulting point pairs (vector will be filled)
 * @param Target The scan to whiche the points are matched
 * @param thread_num number of the thread (for parallelization)
 * @param rnd randomized point selection
 * @param max_dist_match2 maximal allowed distance for matching
 */

void Scan::getNoPairsSimple(vector <double*> &diff,
                            Scan* Source, Scan* Target,
                            int thread_num,
                            double max_dist_match2)
{
  DataXYZ xyz_reduced(Source->get("xyz reduced"));
  KDtree* kd = new KDtree(
                 PointerArray<double>(Target->get("xyz reduced")).get(),
                 Target->size<DataXYZ>("xyz reduced"));

  cout << "Max: " << max_dist_match2 << endl;
  for (unsigned int i = 0; i < xyz_reduced.size(); i++) {

    double p[3];
    p[0] = xyz_reduced[i][0];
    p[1] = xyz_reduced[i][1];
    p[2] = xyz_reduced[i][2];


    double *closest = kd->FindClosest(p, max_dist_match2, thread_num);
    if (!closest) {
         diff.push_back(xyz_reduced[i]);
         //diff.push_back(closest);
    }
  }

  delete kd;
}

/**
 * Calculates a set of corresponding point pairs and returns them. It
 * computes the k-d trees and deletes them after the pairs have been
 * found. This slow function should be used only for testing
 *
 * @param pairs The resulting point pairs (vector will be filled)
 * @param Source The scan whose points are matched to Targets' points
 * @param Target The scan to whiche the points are matched
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
  KDtree* kd = new KDtree(
                 PointerArray<double>(Source->get("xyz reduced")).get(),
                 Source->size<DataXYZ>("xyz reduced"));
  DataXYZ xyz_reduced(Target->get("xyz reduced"));

  for (unsigned int i = 0; i < xyz_reduced.size(); i++) {
    // take about 1/rnd-th of the numbers only
    if (rnd > 1 && rand(rnd) != 0) continue;  

    double p[3];
    p[0] = xyz_reduced[i][0];
    p[1] = xyz_reduced[i][1];
    p[2] = xyz_reduced[i][2];

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
 * @param Target The scan to whiche the points are matched
 * @param thread_num number of the thread (for parallelization)
 * @param rnd randomized point selection
 * @param max_dist_match2 maximal allowed distance for matching
 * @return a set of corresponding point pairs
 */
void Scan::getPtPairs(vector <PtPair> *pairs,
                      Scan* Source, Scan* Target,
                      int thread_num,
                      int rnd, double max_dist_match2, double &sum,
                      double *centroid_m, double *centroid_d,
                      PairingMode pairing_mode)
{
  // initialize centroids
  for(unsigned int i = 0; i < 3; ++i) {
    centroid_m[i] = 0;
    centroid_d[i] = 0;
  }

  // get point pairs
  DataXYZ xyz_reduced(Target->get("xyz reduced"));
  DataNormal normal_reduced(Target->get("normal reduced"));
  Source->getSearchTree()->getPtPairs(pairs, Source->dalignxf,
                                      xyz_reduced,
                                      normal_reduced,
                                      0,
                                      xyz_reduced.size(),
                                      thread_num,
                                      rnd,
                                      max_dist_match2,
                                      sum,
                                      centroid_m, centroid_d,
                                      pairing_mode);

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
 * @param Target The scan to whiche the points are matched
 * @param thread_num The number of the thread that is computing ptPairs
 *                   in parallel
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
void Scan::getPtPairsParallel(vector <PtPair> *pairs,
                              Scan* Source, Scan* Target,
                              int thread_num, int step,
                              int rnd, double max_dist_match2,
                              double *sum,
                              double centroid_m[OPENMP_NUM_THREADS][3],
                              double centroid_d[OPENMP_NUM_THREADS][3],
                              PairingMode pairing_mode)
{
  // initialize centroids
  for(unsigned int i = 0; i < 3; ++i) {
    centroid_m[thread_num][i] = 0;
    centroid_d[thread_num][i] = 0;
  }

  // get point pairs
  SearchTree* search = Source->getSearchTree();
  // differentiate between a meta scan (which has no reduced points)
  // and a normal scan
  // if Source is also a meta scan it already has a special meta-kd-tree
  MetaScan* meta = dynamic_cast<MetaScan*>(Target);
  if(meta) {
    for(unsigned int i = 0; i < meta->size(); ++i) {
      // determine step for each scan individually
      DataXYZ xyz_reduced(meta->getScan(i)->get("xyz reduced"));
      DataNormal normal_reduced(Target->get("normal reduced"));
      unsigned int max = xyz_reduced.size();
      unsigned int step = max / OPENMP_NUM_THREADS;
      // call ptpairs for each scan and accumulate ptpairs, centroids and sum
      search->getPtPairs(&pairs[thread_num], Source->dalignxf,
                         xyz_reduced, normal_reduced,
                         step * thread_num, step * thread_num + step,
                         thread_num,
                         rnd, max_dist_match2, sum[thread_num],
                         centroid_m[thread_num], centroid_d[thread_num],
                         pairing_mode);
    }
  } else {
    DataXYZ xyz_reduced(Target->get("xyz reduced"));
    DataNormal normal_reduced(Target->get("normal reduced"));
    search->getPtPairs(&pairs[thread_num], Source->dalignxf,
                       xyz_reduced, normal_reduced,
                       thread_num * step, thread_num * step + step,
                       thread_num,
                       rnd, max_dist_match2, sum[thread_num],
                       centroid_m[thread_num], centroid_d[thread_num],
                       pairing_mode);
  }

  // normalize centroids
  unsigned int size = pairs[thread_num].size();
  if(size != 0) {
    for(unsigned int i = 0; i < 3; ++i) {
      centroid_m[thread_num][i] /= size;
      centroid_d[thread_num][i] /= size;
    }
  }
}

unsigned int Scan::getMaxCountReduced(ScanVector& scans)
{
  unsigned int max = 0;
  for(std::vector<Scan*>::iterator it = scans.begin();
      it != scans.end();
      ++it) {
    unsigned int count = (*it)->size<DataXYZ>("xyz reduced");
    if(count > max)
      max = count;
  }
  return max;
}
