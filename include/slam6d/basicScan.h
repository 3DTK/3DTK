#ifndef BASIC_SCAN_H
#define BASIC_SCAN_H

#include "scan.h"
#include "frame.h"

#include <vector>
#include <map>

#include "slam6d/Boctree.h"

class BasicScan : public Scan {
public:
  BasicScan() {};

  static void openDirectory(const std::string& path, IOType type, int start, int end);
  static void closeDirectory();
/*
  Scan(const double *euler, int maxDist = -1);
  Scan(const double rPos[3], const double rPosTheta[3], int maxDist = -1);
  Scan(const double _rPos[3], const double _rPosTheta[3], vector<double *> &pts);
*/
  virtual ~BasicScan();

  virtual void setRangeFilter(double max, double min);
  virtual void setHeightFilter(double top, double bottom);
  virtual void setRangeMutation(double range);

  virtual const char* getIdentifier() const { return m_identifier.c_str(); }

  virtual DataPointer get(const std::string& identifier);
  virtual void get(unsigned int types);
  virtual DataPointer create(const std::string& identifier, unsigned int size);
  virtual void clear(const std::string& identifier);
  virtual unsigned int readFrames();
  virtual void saveFrames();
  virtual unsigned int getFrameCount();
  virtual void getFrame(unsigned int i, const double*& pose_matrix, AlgoType& type);
  
  //! Constructor for creation of Scans without openDirectory
  BasicScan(double * rPos, double * rPosTheta, std::vector<double*> points);

  //! Convert Scan to Octree
  BOctTree<float>* convertScanToShowOcttree();

protected:
  virtual void createSearchTreePrivate();
  virtual void calcReducedOnDemandPrivate();
  virtual void calcNormalsOnDemandPrivate();
  virtual void addFrame(AlgoType type);

private:
  //! Path and identifier of where the scan is located
  std::string m_path, m_identifier;

  IOType m_type;

  double m_filter_max, m_filter_min, m_filter_top, m_filter_bottom, m_range_mutation;
  bool m_filter_range_set, m_filter_height_set, m_range_mutation_set;

  std::map<std::string, std::pair<unsigned char*, unsigned int>> m_data;

  std::vector<Frame> m_frames;


  //! Constructor for openDirectory
  BasicScan(const std::string& path, const std::string& identifier, IOType type);

  //! Initialization function
  void init();

  void createANNTree();

  void createOcttree();
};

#endif //BASIC_SCAN_H
