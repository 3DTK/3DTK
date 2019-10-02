#ifndef BASIC_SCAN_H
#define BASIC_SCAN_H

#include "scan.h"
#include "frame.h"

#include <vector>
#include <map>

#include "slam6d/Boctree.h"
#include "scan_settings.h"

#ifdef WITH_MMAP_SCAN
#include "boost/filesystem.hpp"
#endif

class BasicScan : public Scan {
public:
  BasicScan() {};

  static void openDirectory(const std::string& path, IOType type, int start, int end
#ifdef WITH_MMAP_SCAN
    , boost::filesystem::path cache = boost::filesystem::path()
#endif
  );
  static void openDirectory(dataset_settings& ss
#ifdef WITH_MMAP_SCAN
    , boost::filesystem::path cache = boost::filesystem::path()
#endif
  );
  static void closeDirectory();
/*
  Scan(const double *euler, int maxDist = -1);
  Scan(const double rPos[3], const double rPosTheta[3], int maxDist = -1);
  Scan(const double _rPos[3], const double _rPosTheta[3], vector<double *> &pts);
*/
  virtual ~BasicScan();

  virtual void setRangeFilter(double max, double min);
  virtual void setHeightFilter(double top, double bottom);
  virtual void setCustomFilter(std::string& cFiltStr);
  virtual void setRangeMutation(double range);
  virtual void setScaleFilter(double scale);

  virtual const char* getIdentifier() const { return m_identifier.c_str(); }
  virtual const std::string getPath() const { return m_path; }
  virtual const double getTimeStamp() const { return m_timestamp; }

  virtual time_t getLastModified();
  virtual DataPointer get(const std::string& identifier);
  virtual void get(IODataType types);
  virtual DataPointer create(const std::string& identifier, size_t size);
  virtual void clear(const std::string& identifier);
  virtual size_t readFrames();
  virtual void saveFrames(bool append = false);
  virtual void saveBOctTree(std::string & filename);

  virtual size_t getFrameCount();
  virtual void getFrame(size_t i, const double*& pose_matrix, AlgoType& type);

  //! Constructor for creation of Scans without openDirectory
  BasicScan(double * rPos, double * rPosTheta, std::vector<double*> points);
  //! Constructor for creation of Scans without openDirectory,
  // insert the points with the DataPointer
  BasicScan(double * rPos, double * rPosTheta);

  virtual void updateTransform(double *_rPos, double *_rPosTheta);

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

  double m_filter_max, m_filter_min, m_filter_top, m_filter_bottom, m_range_mutation, m_filter_scale, m_timestamp;
  bool m_filter_range_set, m_filter_height_set, m_filter_custom_set, m_range_mutation_set, m_filter_scale_set;
  std::string customFilterStr;

  // map from identifier (xyz, rgb, reflectance...) to a data pointer plus
  // length
  std::map<std::string, std::pair<unsigned char*, size_t> > m_data;

#ifdef WITH_MMAP_SCAN
  std::map<std::string, int> m_mmap_fds;

  boost::filesystem::path m_mmap_cache;
#endif

  std::vector<Frame> m_frames;


  //! Constructor for openDirectory
  BasicScan(const std::string& path, const std::string& identifier, IOType type
#ifdef WITH_MMAP_SCAN
    , boost::filesystem::path cache = boost::filesystem::path()
#endif
  );
  BasicScan(const dataset_settings& ss, std::string& identifier
#ifdef WITH_MMAP_SCAN
    , boost::filesystem::path cache = boost::filesystem::path()
#endif
  ) : BasicScan(ss.data_source, identifier, ss.format
#ifdef WITH_MMAP_SCAN
    , cache
#endif
  ) {}

  //! Initialization function
  void init();

  void createANNTree();

  void createOcttree();
};

#endif //BASIC_SCAN_H
