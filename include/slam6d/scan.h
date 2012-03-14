/**
 * @file
 * @brief Representation of a 3D scan and implementation of scan matching
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __SCAN_H__
#define __SCAN_H__

#ifdef _MSC_VER
#define snprintf _snprintf
#undef _STDIO_DEFINED
#define  _USE_MATH_DEFINES
#endif

#include <cstring>
#include <string>
using std::string;
#include <vector>
using std::vector;
#include <sstream>
using std::stringstream;

// ANN k-d tree library
#include "ANN/ANN.h"				// ANN declarations
#include "ANN/ANNperf.h"				// k-d tree printing

#include "slam6d/ptpair.h"
#include "slam6d/point.h"
#include "slam6d/point_type.h"
#include "slam6d/searchTree.h"
#include "slam6d/kdcache.h"
#include "slam6d/globals.icc"
#include "slam6d/scan_io.h"

#ifdef WITH_SCANSERVER
#include "scanserver/sharedScan.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#endif


#include "scanserver/io_types.h"

enum nns_type {
  simpleKD, cachedKD, ANNTree, BOCTree //, NaboKD
};


class SearchTree;
template<typename T>
class BOctTree;

/**
 * @brief 3D scan representation and implementation of scan matching
 */
class Scan {

  friend class SearchTree;

public:
  Scan();
  Scan(const double *euler, int maxDist = -1);
  Scan(const double rPos[3], const double rPosTheta[3], int maxDist = -1);
  Scan(const double _rPos[3], const double _rPosTheta[3], vector<double *> &pts);
  Scan(const vector < Scan* >& MetaScan, int nns_method, bool cuda_enabled);
  Scan(const Scan& s);

  ~Scan();

  void mergeCoordinatesWithRoboterPosition(Scan *prevScan);

  inline const double* get_transMat() const;
  inline const double* get_rPos() const;
  inline const double* get_rPosTheta() const;
  inline const double* get_rPosQuat() const;

  void transformAll(const double alignxf[16]);
  void transformAll(const double alignQuat[4], const double alignt[3]);
 
  enum AlgoType {
    INVALID, ICP, ICPINACTIVE, LUM, ELCH, LOOPTORO, LOOPHOGMAN, GRAPHTORO, GRAPHHOGMAN
  };

  void transform(const double alignxf[16],
			  const AlgoType type, int islum = 0);
  void transform(const double alignQuat[4], const double alignt[3],
			  const AlgoType type, int islum = 0);
  void transformToMatrix(double alignxf[16],
            const AlgoType type, int islum = 0);
  void transformToEuler(double rP[3], double rPT[3],
				    const AlgoType type, int islum = 0);
  void transformToQuat(double rP[3], double rPQ[4],
				   const AlgoType type, int islum = 0);
  
  void toGlobal(double voxelSize, int nrpts);
  void calcReducedPoints(double voxelSize, int nrpts = 0, PointType _pointtype = PointType());
  void trim(double top, double bottom);
  
  void createTree(int nns_method, bool cuda_enabled);
  static void createTrees(int nns_method, bool cuda_enabled);
  static void deleteTrees();

//  static KDCacheItem* initCache(const Scan* Source, const Scan* Target);
  
  static void getPtPairs(vector <PtPair> *pairs, 
					Scan* Source, Scan* Target, 
					int thread_num,
					int rnd, double max_dist_match2, double &sum,
					double *centroid_m, double *centroid_d);
  static void getNoPairsSimple(vector <double*> &diff, 
					   Scan* Source, Scan* Target, 
					   int thread_num,
					   double max_dist_match2);
  static void getPtPairsSimple(vector <PtPair> *pairs, 
						 Scan* Source, Scan* Target, 
						 int thread_num,
						 int rnd, double max_dist_match2,
						 double *centroid_m, double *centroid_d);
  static void getPtPairsParallel(vector <PtPair> *pairs, 
						   Scan* Source, Scan* Target,
						   int thread_num, int step,
						   int rnd, double max_dist_match2,
						   double *sum,
						   double centroid_m[OPENMP_NUM_THREADS][3], double centroid_d[OPENMP_NUM_THREADS][3]);
   
  inline friend ostream& operator<<(ostream& os, Scan& s); 
  inline friend ostream& operator<<(ostream& os, const double matrix[16]);

#ifndef WITH_SCANSERVER
  inline int get_points_red_size() const;
#endif //WITH_SCANSERVER

  inline void resetPose();
  
  /**
   * storing a list of (pointers to) all scans here 
   */
  static vector <Scan *> allScans;

#ifndef WITH_SCANSERVER
  /**
   * max number of (reduced) points in the scans
   */
  static unsigned int max_points_red_size;
  
  /**
   * The output directory
   */  
  static string dir;
#endif //WITH_SCANSERVER

  static void readScans(IOType type,
				    int start, int end, string &dir, int maxDist, int minDist,
				    bool openFileForWriting = false);
  static void readScansRedSearch(IOType type,
						   int start, int end, string &dir, int maxDist, int minDist,
						   double voxelSize, int nrpts, // reduction parameters
						   int nns_method = -1, bool cuda_enabled = false,
						   bool openFileForWriting = false);
#ifndef WITH_SCANSERVER
  inline const vector <Point>* get_points() const;
  inline double* const* get_points_red() const;
  inline void setPoints(vector <Point> *_points);
  inline double* const* get_points_reduced() const;
  inline void setFileNr(int _fileNr);
  inline int  getFileNr() const;
  
  // stub for making it compatible with scanserver getCount
  inline unsigned int getCount() const { return get_points()->size(); }

  inline void clearPoints();
#endif //WITH_SCANSERVER

  inline const SearchTree *get_tree() const;
  inline const double * getTransMatOrg() const;
  
  //FIXME
  inline const ANNkd_tree* getANNTree() const;
  inline const double* getDAlign() const;
  inline const double* getDAlign_inv() const;
#ifndef WITH_SCANSERVER
  inline double** get_org_points_red() const;
#endif //WITH_SCANSERVER

protected:

#ifndef WITH_SCANSERVER
  class scanIOwrapper : public ScanIO {
    public:

    scanIOwrapper(IOType type );
    ~scanIOwrapper();

    virtual int readScans(int start, int end, string &dir, int maxDist, int mindist,
				    double *euler, vector<Point> &ptss); 
    private:
    ScanIO *my_ScanIO;

#ifdef _MSC_VER
    HINSTANCE hinstLib;
#else
    void *ptrScanIO;
#endif

  };
#endif //WITH_SCANSERVER
  
  /**
   * The pose of the scan
   * Note: rPos/rPosTheta and transMat _should_
   *       always represent the same pose!!!
   */
  double rPos[3],       ///< 3D position
         rPosTheta[3],  ///< 3D rotation in Euler representation 
         rQuat[4],      ///< 3D rotation in Quaternion representation
         transMat[16];  ///< (4x4) transformation matrix

  
  //! Run ICP on GPU instead of CPU
  bool cuda_enabled;
  
  //! Defines the method used for nearest neighbor search and which tree to use
  int nns_method;

  /**
   * The original pose of the scan, e.g., from odometry
   */
  double transMatOrg[16];
  
  //! Internal function of transform which alters the reduced points
  void transformReduced(const double alignxf[16]);
  
  //! Internal function of transform which handles the matrices
  void transformMatrix(const double alignxf[16]);

#ifndef WITH_SCANSERVER
  /**
   * Vector for storing the scan data
   */
  vector <Point> points;
#endif //WITH_SCANSERVER

  /**
   * Vector storing single scans of a metascan
   */
  vector <Scan *> meta_parts;

#ifndef WITH_SCANSERVER
  /**
   * Array for storing reduced points. In case there is no reduction the points will
   * be copied. All transformations are applied only to this array, sving a lot of
   * time, since the input point cloud, i.e., the vector does not need to be
   * transformed.
   *
   * ATTENTION: points_red is NOT a vector of "Points", an array of "double*" instead,
   * since this data structure is necessary in later functions; storing a vector<Points>
   * here would mean too many conversions, therefore loss of speed for LUM.
   */
  double **points_red;

  /** 
   * number elements of the array 
   */
  int points_red_size;
#endif //WITH_SCANSERVER

  /**
   * The search tree
   *
   * It can be a k-d tree of cached k-d tree.
   */
  SearchTree *kd;

  /**
   * This KD tree is created only for the CUDA usages
   */
  ANNkd_tree *ann_kd_tree;
  void createANNTree();
  
#ifndef WITH_SCANSERVER
  /**
   * Array for storing reduced points. The reduced points are copied and k-d trees 
   * can be constructed with pointers to these trees. This allows the computation
   * of fast corresponding points. Fast corresponding points exploint that a transformation
   * in one coordinate systems equals an inverse transformation in the other frame.
   * Therefore, search, trees are build only once per 3D scan.
   *
   * ATTENTION: points_red is NOT a vector of "Points", an array of "double*" instead,
   * since this data structure is necessary in later functions; storing a vector<Points>
   * here would mean too many conversions, therefore loss of speed for LUM.
   */
  double** points_red_lum;
#endif //WITH_SCANSERVER

  /**
   * The dalignxf transformation represents the delta transformation virtually applied
   * to the tree and is used to compute are actual corresponding points.
   */
  double dalignxf[16];

#ifndef WITH_SCANSERVER
  /**
   * The flag outputFrames specifies if .frames files should be created and written.
   * In show or conversion tools this is usually unwanted, in slam6D we certainly 
   * need it.
   */
  static bool outputFrames;
#endif //WITH_SCANSERVER

  /**
   * counter for the number of 3D Scans (including the metascans)
   */
  static unsigned int numberOfScans;

  /**
   * The actual scan number
   */
  unsigned int scanNr;
  
#ifndef WITH_SCANSERVER
  /**
   * The actual file number for the *.frames file
   */
  int fileNr;
  
  /*
   * The stringstream sout buffers the frame file. It will be written to disk at
   * once at the end of the program. This reduces file operations and saves time.
   */
  stringstream sout;
#endif //WITH_SCANSERVER

  /** 
   * regard only points up to an (Euclidean) distance of maxDist
   * points are filtered during input)
   * store sqr(maxDist) here; saves taking the square root later!
   * value of maxDist2 = -1 indicates no limit
   */
  int maxDist2;

  void deleteTree();
  
#ifdef WITH_SCANSERVER
public:
  //! Delete scans, shared scans, the directory and empty the vector
  static void clearScans();

  //! Delete a single scan and its entry from the vector
  static void remove(Scan* scan);

  //! Determine the maximum number of reduced points of all scans
  static unsigned int getMaxCountReduced();
private:
  static ScanVector* shared_scans;
public:
  //! Create a scan with its internal data hold by a shared scan
  Scan(SharedScan* shared_scan);

  //! Identifier (counter variable in the ScanIO) from the shared scan
  inline const char* getIdentifier() const { if(m_shared_scan != 0) return m_shared_scan->getIdentifier(); else return "none"; }

  //! Expose raw contents
  double* getPose() const;
  DataXYZ getXYZ() const;
  DataRGB getRGB() const;
  DataReflectance getReflectance() const;
  DataAmplitude getAmplitude() const;
  DataType getType() const;
  DataDeviation getDeviation() const;
  
  //! Set prefetch flags (routed to SharedScan)
  void prefetch(IODataType type) { m_shared_scan->prefetch(type); }
  
  //! Return prefetch flags (routed to SharedScan)
  unsigned int getPrefetch() const { return m_shared_scan->getPrefetch(); }
  
  //! Clear prefetch flags (routed to SharedScan)
  void clearPrefetch() { m_shared_scan->clearPrefetch(); }
  
  
  //! Count of (filtered) points
  unsigned int getCount();

  //! Expose reduced points
  unsigned int getCountReduced();

  //! Get reduced points which are affected by transform and used as target points in matching
  DataXYZ getXYZReduced();

  //! Get the tree and calculate it on demand if not available
  SearchTree* getSearchTree();

  /**
   * Get reduced points which are only transformed to the initial transMatOrg pose and serve as a basis for SearchTree.
   * In case of a meta-scan this doesn't use a SharedScan (cause none exists for a self-created Scan) and uses meta_points instead.
   */
  DataXYZ getXYZReducedOriginal();

  //! Creates a reduced version of xyz in local coordinates for show to use
  void calcReducedShow(double voxelSize, int nrpts = 0);

  //! Individual reduced data for show
  TripleArray<float> getXYZReducedShow();
  
  //! Identify an octtree how it was created and if it can be fetched from cache
  void setOcttreeParameters(const char* params);
  
  //! Get saved octtree (for show)
  CacheDataAccess getOcttree();
  
  //! Copy an octtree into cache and attach it to this scan
  CacheDataAccess copyOcttreeToCache(BOctTree<float>* oct);

  /**
   * Copies reduced points to original points without any transformation.
   * Is called in the on-demand reduction after the transMatOrg transformation of the reduced points. Otherwise, use at your own risk.
   */
  void copyReducedToOriginal();
  
  /**
   * Inverse functionality of copyReducedToOriginal.
   * Used in the on-demand reduction if originals fitting the reduction and filter parameters and same pose were found and can skip the reduction step.
   */
  void copyOriginalToReduced();
  
  //! Called from the metascan constructor this function handles matrices and the creation of the meta searchtree
  void createMetaTree(const vector<Scan*>& scans);

  //! Clear previous stored frames to start from anew
  void clearFrames();
  
  //! Get contained frames
  const FrameVector& getFrames();

  //! Save frames into a file
  void saveFrames();

private:
  //! Constructor initialization stub
  void init();

  //! Add a new frame with the current transformation and given type, only called from transform
  void addFrame(AlgoType type);

  //! Calculate reduced points and an untransformed copy of it plus the search trees on demand
  void calcReducedOnDemand();
  
private:
  //! Reference to the shared scan
  SharedScan* m_shared_scan;

  //! Array holding the meta points instead of having a SharedScan reference
  unsigned int meta_count;

  //! Buffered value of count and its reduced part
  unsigned int m_count, m_count_reduced;
  
  //! Buffered size of reduced show data
  unsigned int m_show_count;
  
  //! Reduction on-demand parameter for voxel size
  double reduction_voxelSize;
  
  //! Reduction on-demand parameter for octree center
  int reduction_nrpts;
  
  //! Reduction on-demand parameter for voxel size for show
  double show_red_voxelSize;
  
  //! Reduction on-demand parameter for octree center for show
  int show_red_nrpts;

  //! Mutex for safely reducing points and creating the search tree just once in a multithreaded environment
  boost::mutex m_mutex_reduction, m_mutex_create_tree;
#endif //WITH_SCANSERVER
};

#include "scan.icc"

#endif
