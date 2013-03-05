/*
 * scan definition
 *
 * Copyright (C) by the 3DTK contributors
 *
 * Released under the GPL version 3.
 *
 */

#ifndef __SCAN_H__
#define __SCAN_H__

#include "io_types.h"
#include "data_types.h"
#include "point_type.h"
#include "ptpair.h"
#include "pairingMode.h"

#include <string>
#include <vector>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

//! SearchTree types
enum nns_type {
  simpleKD, ANNTree, BOCTree
};

class Scan;
typedef std::vector<Scan*> ScanVector;

class SearchTree;
class ANNkd_tree;

/** HOWTO scan

  First: Load scans (if you want to use the scanmanager, use ManagedScan)

  BasicScan/ManagedScan::openDirectory(path, type, start, end);

  Pass it to functions (by reference to link it to the same instance)
  or store it in a global variable

  After loading you might want to set parameters

  for(ScanVector::iterator it = Scan::allScans.begin();
      it != Scan::allScans.end();
      ++it) {
    Scan* scan = *it;
    scan->setRangeFilter(maxDist, minDist);
    scan->setHeightFilter(top, bottom); // thermo
    scan->setReductionParameter(voxelSize, nrpts[, pointtype]);
    scan->setSearchTreeParameter(nns_method);
  }

  Access the contained data, will be loaded and calculated on demand

  DataXYZ xyz = scan->get("xyz");
  DataXYZ reduced = scan->get("xyz reduced");
  DataRGB rgb = scan->get("rgb");
  
  xyz[i][0..2]
  reflectance[i]
  
  unsigned int size = scan->size("xyz reduced");

  To use the prefetching of all requested data field in the scanserver,
  mark them for use. This is relevant for efficiency, which would
  otherwise cause loading the files each time another data field is
  requested.

  scan->get(DATA_XYZ | DATA_RGB | ...);

  Under circumstances the data fields are not available (e.g. no color in
  uos-type scans)

  DataRGB rgb = scan->get("rgb");
  if (rgb.valid()) { ok, do something }
  
  If backward-compability to pointer arrays is needed, the PointerArray
  class can adapt

  BOctTree(PointerArray(scan->get("xyz")).get(), scan->size("xyz"), ...);

  If data isn't needed anymore, flag it for removal

  scan->clear("xyz");
  scan->clear(DATA_XYZ | DATA_RGB | ...);
  
  Creating data fields with the correct byte size

  scan->create("xyz somethingelse", sizeof(double)*3*N);

  Reading frames in show:

  unsigned int size = scan->readFrames();
  
  const double* pose;
  AlgoType type;
  scan->getFrame(i, pose, type);

  Last, if program ends, clean up

  Scan::closeDirectory(scans);

**/



/**
 * This class bundles common features of different scan implementations by
 * abstraction. It handles the algorithmic parts and leaves IO and other
 * features to the deriving classes.
 */
class Scan {
public:
  enum AlgoType { INVALID, ICP, ICPINACTIVE, LUM, ELCH };
  
  // delete copy-ctor and assignment, scans shouldn't be copied by basic class
#ifdef _MSC_VER
  // FIXME
#else
  Scan(const Scan& other) = delete;
  Scan& operator=(const Scan& other) = delete;
#endif
  
  virtual ~Scan();

  //! Holder of all scans
  //  also used in transform for adding frames for each scan at the same time
  static std::vector<Scan*> allScans;
  
  /**
    * Attempt to read a directory under \a path and return its read scans.
    * No scans are loaded at this point, only checked if all exist.
    * 
    * @param scanserver whether to use managed scans in the scanserver or not
    * @param path to the directory containing the scans
    * @param type determining which ScanIO to use
    * @param start first scan to use
    * @param end last scan to use, -1 means from start to last available
    */
  static void openDirectory(bool scanserver,
                            const std::string& path,
                            IOType type,
                            int start,
                            int end = -1);
  
  /**
   * "Close" a directory by deleting all its scans and emptying the
   * Scan::allScans vector.
   */
  static void closeDirectory();
  
  
  /* Input filtering and parameter functions */
  
  //! Input filtering for all points based on their euclidean length
  virtual void setRangeFilter(double max, double min) = 0;
  
  //! Input filtering for all points based on their height
  virtual void setHeightFilter(double top, double bottom) = 0;

  //! Input mutation to set range of all points to a constant value;
  virtual void setRangeMutation(double range) { }
  
  //! Set reduction parameters, but don't reduce yet
  virtual void setReductionParameter(double voxelSize, int nrpts = 0,
    PointType pointtype = PointType());
  
  //! Set SearchTree type, but don't create it yet
  void setSearchTreeParameter(int nns_method);
  
  /**
   * Set octtree parameters for show
   * @param loadOct will load the serialized octtree from disk regardless
   * @param saveOct serialize octtree if not loaded by loadOct after creation
   */
  virtual void setOcttreeParameter(double reduction_voxelSize,
                                   double octtree_voxelSize,
                                   PointType pointtype,
                                   bool loadOct,
                                   bool saveOct);

  /* Basic getter functions */

  inline const double* get_rPos() const;
  inline const double* get_rPosTheta() const;
  inline const double* get_rPosQuat() const;
  //! Pose matrix after initial and match transformations (org+dalign)
  inline const double* get_transMat() const;
  //! Original pose matrix after initial transform
  inline const double* get_transMatOrg() const;
  //! Accumulated delta transformation matrix 
  inline const double* getDAlign() const;
  
  inline SearchTree* getSearchTree();
  //  inline ANNkd_tree* getANNTree() const;
  
  virtual const char* getIdentifier() const = 0;
  
  //! Determine the maximum number of reduced points in \a scans
  static unsigned int getMaxCountReduced(ScanVector& scans);
  
  /* Functions for altering data fields, implementation specific */
  
  /**
   * Get the data field \a identifier, calculate it on demand if neccessary.
   *
   * If "xyz reduced" or "xyz reduced original" is requested, the reduction is
   * started with "xyz" as input.
   */
  virtual DataPointer get(const std::string& identifier) = 0;

  /**
   * Load the requested IODataTypes, joined by |, from the scan file.
   *
   * This feature is neccessary to load multiple data fields at once, not all
   * one by one with each get("...") access.
   */
  virtual void get(unsigned int types) = 0;
  
  /**
   * Creates a data field \a identifier with \a size bytes.
   */
  virtual DataPointer create(const std::string& identifier,
                             unsigned int size) = 0;
  
  /**
   * Clear the data field \a identifier, removing its allocated memory if
   * possible or marking it for prioritized removal.
   */
  virtual void clear(const std::string& identifier) = 0;

  //! Extension to clear for more than one identifier, e.g.
  //  clear(DATA_XYZ | DATA_RGB);
  void clear(unsigned int types);
  
  /**
   * Get the size of \a identifier as if it were requested and size() called
   * upon its type specialized DataPointer class.
   * e.g size<DataXYZ>("xyz reduced")
   */
  template<typename T>
  unsigned int size(const std::string& identifier) {
    return (T(get(identifier))).size();
  }

  /* Frame handling functions */

  /**
   * Open the .frames-file and read its contents. If not read, the frame list
   * will be empty.
   * @return count of frames if file has been read, zero otherwise
   */
  virtual unsigned int readFrames() = 0;
  
  /**
   * Write the accumulated frames into a .frames-file.
   */
  virtual void saveFrames() = 0;
  
  //! Count of frames
  virtual unsigned int getFrameCount() = 0;
  
  //! Get contents of a frame, pass matrix pointer and type by reference
  virtual void getFrame(unsigned int i,
                        const double*& pose_matrix,
                        AlgoType& type) = 0;
  
protected:
  /**
   * Called from transform, this will add its current transMat pose with
   * the given type as a frame into the list of frames
   */
  virtual void addFrame(AlgoType type) = 0;
  
public:
  
  /* Direct creation of reduced points and search tree */
  
  //! Apply reduction and initial transMatOrg transformation
  void toGlobal();
  
  //! Copy reduced points to original and create search tree on it
  void createSearchTree();

  /* Common transformation and matching functions */
  void mergeCoordinatesWithRoboterPosition(Scan* prevScan);
  void transformAll(const double alignxf[16]);
  void transformAll(const double alignQuat[4],
                    const double alignt[3]);
  
  void transform(const double alignxf[16],
                 const AlgoType type, 
                 int islum = 0);
  void transform(const double alignQuat[4],
                 const double alignt[3], 
                 const AlgoType type, 
                 int islum = 0);
  void transformToMatrix(double alignxf[16],
                         const AlgoType type, 
                         int islum = 0);
  void transformToEuler(double rP[3], 
                        double rPT[3],
                        const AlgoType type, 
                        int islum = 0);
  void transformToQuat(double rP[3], 
                       double rPQ[4],
                       const AlgoType type, 
                       int islum = 0);
  
  // Scan matching functions
  static void getPtPairs(std::vector<PtPair> *pairs,
                         Scan* Source,
                         Scan* Target,
                         int thread_num,
                         int rnd,
                         double max_dist_match2,
                         double &sum,
                         double *centroid_m,
                         double *centroid_d,
                         PairingMode pairing_mode = CLOSEST_POINT);
  static void getNoPairsSimple(std::vector<double*> &diff,
                               Scan* Source, Scan* Target,
                               int thread_num,
                               double max_dist_match2);
  static void getPtPairsSimple(std::vector<PtPair> *pairs,
                               Scan* Source,
                               Scan* Target,
                               int thread_num,
                               int rnd,
                               double max_dist_match2,
                               double *centroid_m,
                               double *centroid_d);
  static void getPtPairsParallel(std::vector<PtPair> *pairs,
                                 Scan* Source,
                                 Scan* Target,
                                 int thread_num,
                                 int step,
                                 int rnd,
                                 double max_dist_match2,
                                 double *sum,
                                 double centroid_m[OPENMP_NUM_THREADS][3],
                                 double centroid_d[OPENMP_NUM_THREADS][3],
                                 PairingMode pairing_mode);

protected:
  /**
   * The pose of the scan
   * Note: rPos/rPosTheta and transMat _should_
   *       always represent the same pose!!!
   */
  double rPos[3],    //!< 3D position
    rPosTheta[3],    //!< 3D rotation in Euler representation 
    rQuat[4],        //!< 3D rotation in Quaternion representation
    transMat[16],    //!< (4x4) transformation matrix
    transMatOrg[16]; //!< The original pose of the scan, e.g., from odometry

  /**
   * The dalignxf transformation represents the delta transformation
   * virtually applied to the tree and is used to compute are actual
   * corresponding points.
   */
  double dalignxf[16];

  //! Defines the method used for nearest neighbor search and which tree to use
  int nns_method;
  
  //! SearchTree for point pair matching, works on the search points
  SearchTree* kd;
  
  //! Voxelsize of the octtree used for reduction
  double reduction_voxelSize;
  
  //! Which point to take out of the reduction octtree, 0 for center
  int reduction_nrpts;
  
  //! Pointtype used for the reduction octtree
  PointType reduction_pointtype;
  
  //! Type of the searchtree to be created
  int searchtree_nnstype;
  
  //! Flag whether "xyz reduced" has been initialized for this Scan yet
  bool m_has_reduced;

  //! Flag whether "normals" has been initialized for this Scan yet
  bool m_has_normals;
  
  //! Reduction value used for octtree input
  double octtree_reduction_voxelSize;
  
  //! Voxelsize used in the octtree itself
  double octtree_voxelSize;
  
  //! Pointtype for the Octtree
  PointType octtree_pointtype;
  
  //! Flags to load or save the octtrees from/to storage
  bool octtree_loadOct, octtree_saveOct;
  
  /**
   * Basic initializing constructor calling the initalization function.
   * Can only be called from deriving classes.
   */
  Scan();
  
  /**
   * This function handles the reduction of points. It builds a lock for
   * multithread-safety and calls calcReducedOnDemandPrivate.
   *
   * The intention is to reduce points, transforme them to the initial pose and
   * then copy them to original for the SearchTree.
   */
  void calcReducedOnDemand();

  /**
   * This function handles the computation of the normals. It builds a lock for
   * multithread-safety and calls caldNormalsOnDemandPrivate.
   */
  void calcNormalsOnDemand();
  
  //! Create specific SearchTree variants matching the capability of the Scan
  virtual void createSearchTreePrivate() = 0;
  
  //! Create reduced points in a multithread-safe environment matching
  //  the capability of the Scan
  virtual void calcReducedOnDemandPrivate() = 0;

  //! Create normals in a multithread-safe environment matching
  //  the capability of the Scan
  virtual void calcNormalsOnDemandPrivate() = 0;
  
  //! Creating normals
  void calcNormals();
  
  //! Internal function of transform which alters the reduced points
  void transformReduced(const double alignxf[16]);
  
  //! Internal function of transform which handles the matrices
  void transformMatrix(const double alignxf[16]);

  //@FIXME
public:  
  //! Creating reduced points
  void calcReducedPoints();

protected:  
  //! Copies reduced points to original points without any transformation.
  void copyReducedToOriginal();
  
  //! Inverse functionality of copyReducedToOriginal.
  void copyOriginalToReduced();

private:
  //! flag for openDirectory and closeDirectory to distinguish the scans
  static bool scanserver;

public:
  //! Mutex for safely reducing points and creating the search tree
  //  just once in a multithreaded environment it can not be compiled
  //  in win32 use boost 1.48, therefore we remeove it temporarily
  boost::mutex m_mutex_reduction, m_mutex_create_tree, m_mutex_normals;
};

#include "scan.icc"

#endif // __SCAN_H__
