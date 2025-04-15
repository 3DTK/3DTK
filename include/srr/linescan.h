#ifndef __LINESCAN_H__
#define __LINESCAN_H__

#include <iomanip>
#include <iostream>
#include <fstream>
#include <queue>
using std::queue;
#include <vector>
using std::vector;
#include <sstream>
using std::stringstream;
using std::string;
using std::cout;
using std::endl;
using std::ifstream;
#include <boost/filesystem/operations.hpp>

#ifndef _MSC_VER
#include <getopt.h>
#else
#include "XGetopt.h"
#endif
#include "slam6d/point_type.h"
#include "slam6d/io_types.h"
#include "slam6d/globals.icc"
#include "slam6d/scan.h"
#include "slam6d/icp6Dquat.h"
#include "slam6d/icp6D.h"
#include "map"

typedef pair<double *, double *> lppair;
typedef vector<lppair> vlppair;
typedef map<unsigned int, vlppair > Mcorr;

class LineScan {
public:

  ~LineScan();
  LineScan() {};
  LineScan(Scan *scan);

  double **points;
  int nrpts;
  double timestamp;
  //////////////
  double rPos[3];
  double rPosTheta[3];
  double transMat[16];
  double orig_transMat[16];
  double theta;
  ////////////////
  double ground_truth_mat[16];
  double gtrPos[3];
  double gtrPosTheta[3];
  ////////////////
  double frames_mat[17];

  bool applyCustomFilter(const Point& point, int &filterMode, int &nrOfParams, double* custFiltParams);

  void resetTransform();
  void transform(const double alignxf[16], const Scan::AlgoType type, int islum);
  void transformToEuler(double rP[3], double rPT[3], const Scan::AlgoType type, int islum);

  static bool writeFrames(std::string basedir, std::string suffix, IOType type, bool append = false);
  static void setWriteFrames(bool output) { outputFrames = output; };

  static double *FindClosest(double *p, double maxdist2, int threadNum);

  static vector<LineScan*> allLineScans;

  static void computePtPairs(double max_dist_match2);
  static void computePtPairsOct(double max_dist_match2);

  void findPtPairs(double max_dist_match2);
  void findPtPairs(int i, double max_dist_match2);
  static void getPtPairs(vector <PtPair> *pairs,
      LineScan *Source, LineScan* Target,
      int thread_num,
      int rnd, double max_dist_match2,
      double *centroid_m, double *centroid_d);


  static inline void setMinDist(unsigned int mdist) {
    mindist = mdist;
  }

  static inline void setMaxDist(unsigned int mdist) {
    maxdist = mdist;
  }
  static void clearAllPtPairs() {
    for (unsigned int i = 0; i < allLineScans.size(); i++) {
      allLineScans[i]->clearPtPairs();
    }
  }

  void addPtPair(lppair &pair, unsigned int i);

  int getIndex() {
    return index;
  }
  static void printPtPairs();

  double *getQuery(unsigned int i) { return query[i]; }
  unsigned int nrQuery() { return query.size(); }
  void clearQuery() {query.clear();}
  void addQuery(double *q) { query.push_back(q); }

  double **getPoints() { return points;}
  int getNrPoints() { return nrpts;}

  std::string getFramesPath() {return boost::filesystem::path(basedir + "scan" + identifier + ".frames").string();};

  std::string getFramesData() {return sout.str();};

  void resetFramesData() {
    sout.str(std::string());
    sout.clear();
  };

private:

  void clearPtPairs();

  vector<double*> query;

  // contains the correspondences for each linescan
public:  Mcorr correspondences;

  /*
   * The stringstream sout buffers the frame file. It will be written to disk at
   * once at the end of the program. This reduces file operations and saves time.
   */
  stringstream sout;

  static bool outputFrames;
  std::string identifier;
  int index;
  string basedir;

  static unsigned int counter;
  static unsigned int mindist;
  static unsigned int maxdist;
  static std::string error;
};


class LScan {
public:
  LScan(int begin, int end, int rep);
  ~LScan();


  void transform(const double alignxf[16],
			  const Scan::AlgoType type, int islum = 0);



  static void writeAllPtPairs(vector<LScan *> &scans, int rnd, double max_dist_match2, string filename);
  static void writePtPairs(LScan *Source, LScan* Target, int rnd, double max_dist_match2, string filename);
  static void getPtPairs(vector <PtPair> *pairs,
					LScan *Source, LScan* Target,
					int thread_num,
					int rnd, double max_dist_match2,
					double *centroid_m, double *centroid_d);

  static void getOdomPairs(vector <PtPair> *pairs, LScan *Source, LScan* Target,
              double *centroid_m, double *centroid_d, bool ground_truth = false);

  static void getPtPairsParallel(vector <PtPair> *pairs,
						   LScan* Source, LScan* Target,
						   int thread_num, int step,
						   int rnd, double max_dist_match2,
						   double *sum,
						   double centroid_m[OPENMP_NUM_THREADS][3], double centroid_d[OPENMP_NUM_THREADS][3]);

   /**
    *  * Accessor for roboter position
    *   * @return Roboter position as double[3]
    *    */
  inline const double* get_rPos() const
  {
      return LineScan::allLineScans[representative]->rPos;
  }

  /**
   *  * Accessor for roboter oritentation
   *   * @return Roboter orientation as double[3]
   *    */
  inline const double* get_rPosTheta() const
  {
      return LineScan::allLineScans[representative]->rPosTheta;
  }

  void transformToEuler(double rP[3], double rPT[3], const Scan::AlgoType type, int islum = 0);

  void resetTransform();
  inline unsigned int getRepresentative() { return representative; }
  inline unsigned int getBegin() { return begin; }
  inline unsigned int getEnd() { return end; }

  /**
   *  * Accessor for roboters transMat
   *   * @return Roboter transformation as double[16]
   *    */
  inline const double* get_transMat() const
  {
      return transMat;
  }

private:
  unsigned int representative;
  unsigned int begin;
  unsigned int end;
  vector<LineScan*> scans;
  LineScan *repscan;
  double transMat[16];


};

#endif
