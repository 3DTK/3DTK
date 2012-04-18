/**
 * @file
 * @brief Representation of a 3D scan and implementation of scan matching
 * @author Andreas Nuechter. Jacobs University Bremen, Germany
 * @author Li Wei, Wuhan University, China
 * @author Li Ming, Wuhan University, China
 */

#ifndef __VELOSCAN_H__
#define __VELOSCAN_H__

#ifdef _MSC_VER
#define snprintf _snprintf
#undef _STDIO_DEFINED
#define  _USE_MATH_DEFINES
#endif

#include "slam6d/scan.h"
#include "veloslam/gridcell.h"
#include "veloslam/gridcluster.h"

 bool FilterNOMovingObjcets(clusterFeature &glu, cluster &gluData);
class Trajectory
{
  public:
     Trajectory();
  public:
     vector <Point> path;
};

/**
 * @brief 3D scan representation and implementation of dynamic velodyne scan matching
 */
class VeloScan : public Scan {

public:
  VeloScan();
  VeloScan(const double *euler, int maxDist = -1);
  VeloScan(const double rPos[3], const double rPosTheta[3], int maxDist = -1);
  VeloScan(const double _rPos[3], const double _rPosTheta[3], vector<double *> &pts);
  VeloScan(const vector < VeloScan* >& MetaScan, int nns_method, bool cuda_enabled);
  VeloScan(const VeloScan& s);
  void setPoints(vector <Point>* _points) ;
  int dumpFrames();

  ~VeloScan();

  static void readScans(IOType type,
				    int start, int end, string &dir, int maxDist, int minDist,
<<<<<<< HEAD
				    bool openFileForWriting = false);  
  static void readScansRedSearch(IOType type,
=======
				    bool openFileForWriting = false);
  static void readScansRedSearch(reader_type type,
>>>>>>> kure/master
						   int start, int end, string &dir, int maxDist, int minDist,
						   double voxelSize, int nrpts, // reduction parameters
						   int nns_method, bool cuda_enabled,
						   bool openFileForWriting = false);

public:

  /**
   * Vector storing single scans of a metascan
   */
//  vector <VeloScan *> meta_parts;

  // FIXME
  void FindingAllofObject(int maxDist, int minDist);
  void TrackingAllofObject(int trackingAlgo);
  void ExchangePointCloud();
  void ClassifiAllofObject();

  int DumpScan(string filename);
  int DumpScanRedPoints(string filename);
  int DeletePoints();

  int CalcRadAndTheta();
  int TransferToCellArray(int maxDist, int minDist);

  void MarkStaticorMovingPointCloud();
  void FreeAllCellAndCluterMemory();
  void ClassifiAllObject();
  void ClassifibyTrackingAllObject(int currentNO ,int windowsize);
  void calcReducedPoints_byClassifi(double voxelSize, int nrpts, PointType pointtype);

  int CalcScanCellFeature();
  int CalcCellFeature(cell& cellobj,cellFeature& f);
  int FindAndCalcScanClusterFeature();
  int SearchNeigh(cluster& clu,charvv& flagvv,int i,int j);
  int CalcClusterFeature(cluster& clu,clusterFeature& f);
  void SaveObjectsInPCD(int index, cluster &gClusterData );
  void SaveFrameInPCD( );

  /** scanCellFeatureArray */
  cellArray scanCellArray;
  cellFeatureArray scanCellFeatureArray;

  clusterArray scanClusterArray;
  clusterFeatureArray scanClusterFeatureArray;

   // log the type of points_red;
  int  *points_red_type;

  bool isTrackerHandled;
  long scanid;

  int clusterNum;//the number of clusters to be tracked, added by yuanjun
};

#endif
