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
#include "slam6d/searchTree.h"
#include "slam6d/kdcache.h"
#include "slam6d/globals.icc"
#include "slam6d/scan_io.h"


enum reader_type {
  UOS, UOS_MAP, UOS_FRAMES, UOS_MAP_FRAMES, UOS_RGB, OLD, RTS, RTS_MAP, RIEGL_TXT, RIEGL_PROJECT, RIEGL_RGB, RIEGL_BIN, IFP, ZAHN, PLY, WRL, XYZ, ZUF, ASC, IAIS, FRONT, X3D, RXP, KIT, AIS, OCT, TXYZR, XYZR, XYZ_RGB, KS, KS_RGB, STL, LEICA };

enum nns_type {
  simpleKD, cachedKD, ANNTree, BOCTree //, NaboKD
};


/**
 * @brief 3D scan representation and implementation of scan matching
 */
class Scan {

public:
  Scan();
  Scan(const double *euler, int maxDist = -1);
  Scan(const double rPos[3], const double rPosTheta[3], int maxDist = -1);
  Scan(const double _rPos[3], const double _rPosTheta[3], vector<double *> &pts);
  Scan(const vector < Scan* >& MetaScan, int nns_method, bool cuda_enabled);
  Scan(const Scan& s);

  ~Scan();

  void mergeCoordinatesWithRoboterPosition(const Scan *prevScan);

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
  void calcReducedPoints(double voxelSize, int nrpts = 0);
  
  void createTree(int nns_method, bool cuda_enabled);
  static void createTrees(int nns_method, bool cuda_enabled);
  static void deleteTrees();

  static KDCacheItem* initCache(const Scan* Source, const Scan* Target);
  
  static void getPtPairs(vector <PtPair> *pairs, 
					Scan* Source, Scan* Target, 
					int thread_num,
					int rnd, double max_dist_match2,
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
  static void getPtPairsCache(vector <PtPair> *pairs, KDCacheItem *closest,
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
  static void getPtPairsCacheParallel(vector <PtPair> *pairs, KDCacheItem *closest,
							   Scan* Source, Scan* Target,
							   int thread_num, int step,
							   int rnd, double max_dist_match2,
							   double *sum,
							   double centroid_m[OPENMP_NUM_THREADS][3], double centroid_d[OPENMP_NUM_THREADS][3]);
   
  inline friend ostream& operator<<(ostream& os, const Scan& s); 
  inline friend ostream& operator<<(ostream& os, const double matrix[16]);

  inline int get_points_red_size() const;

  inline void resetPose();
  
  /**
   * storing a list of (pointers to) all scans here 
   */
  static vector <Scan *> allScans;

  /**
   * max number of (reduced) points in the scans
   */
  static unsigned int max_points_red_size;
  
  /**
   * The output directory
   */  
  static string dir;

  static bool toType(const char* string, reader_type &type);

  static void readScans(reader_type type,
				    int start, int end, string &dir, int maxDist, int minDist,
				    bool openFileForWriting = false);  
  static void readScansRedSearch(reader_type type,
						   int start, int end, string &dir, int maxDist, int minDist,
						   double voxelSize, int nrpts, // reduction parameters
						   int nns_method, bool cuda_enabled, 
						   bool openFileForWriting = false);  
  inline const vector <Point>* get_points() const;
  inline double* const* get_points_red() const;
  inline void setPoints(vector <Point> *_points);
  inline double* const* get_points_reduced() const;

  inline void clearPoints();

  //FIXME
  inline const ANNkd_tree* getANNTree() const;
  inline const double* getDAlign() const;
  inline const double* getDAlign_inv() const;
  inline double** get_org_points_red() const;

private:


  class scanIOwrapper : public ScanIO {
    public:

    scanIOwrapper(reader_type type );
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
  
  /**
   * The pose of the scan
   * Note: rPos/rPosTheta and transMat _should_
   *       always represent the same pose!!!
   */
  double rPos[3],       ///< 3D position
         rPosTheta[3],  ///< 3D rotation in Euler representation 
         rQuat[4],      ///< 3D rotation in Quaternion representation
         transMat[16];  ///< (4x4) transformation matrix

  
  /**
    * run ICP on GPU instead of CPU
    */

  bool cuda_enabled;
  /**
    * Defines the method used for nearest neighbor search
    */

  int nns_method;

  /**
   * The original pose of the scan, e.g., from odometry
   */
  double transMatOrg[16];

  /**
   * Vector for storing the scan data
   */
  vector <Point> points;

  /**
   * Vector storing single scans of a metascan
   */
  vector <Scan *> meta_parts;

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

  /** 
   * This vector holds the cache, i.e., the pointers to the leafs.
   * It is build during the first ICP iteration and used in the 
   * following ones.
   *
   * Reference: "Cached k-d tree search for ICP algorithms" by A. Nuechter
   *            et al., Proceedings IEEE 3DIM, Montreal, Canada, 2007.
   */
  static vector<KDCache*> closest_cache;

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

  /**
   * The treeTransMat_inv holds the current transformation of a 3D scan that is stored 
   * in a search tree. 
   */
  double treeTransMat_inv[16];

  /**
   * The dalignxf transformation represents the delta transformation virtually applied
   * to the tree and is used to compute are actual corresponding points.
   */
  double dalignxf[16];

  /**
   * The flag outputFrames specifies if .frames files should be created and written.
   * In show or conversion tools this is usually unwanted, in slam6D we certainly 
   * need it.
   */
  static bool outputFrames;

  /**
   * counter for the number of 3D Scans (including the metascans)
   */
  static unsigned int numberOfScans;

  /**
   * The actual scan number
   */
  unsigned int scanNr;
  /**
   * The actual file number for the *.frames file
   */
  int fileNr;

  /*
   * The stringstream sout buffers the frame file. It will be written to disk at
   * once at the end of the program. This reduces file operations and saves time.
   */
  stringstream sout;

  /** 
   * regard only points up to an (Euclidean) distance of maxDist
   * points are filtered during input)
   * store sqr(maxDist) here; saves taking the square root later!
   * value of maxDist2 = -1 indicates no limit
   */
  int maxDist2;

  void deleteTree();
};

#include "scan.icc"

#endif
