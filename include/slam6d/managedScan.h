#ifndef MANAGED_SCAN_H
#define MANAGED_SCAN_H

#include "scan.h"
#include "scanserver/sharedScan.h"



class ManagedScan : public Scan {
public:
  static void openDirectory(const std::string& path, IOType type,
    int start, int end = -1);
  static void closeDirectory();

  static std::size_t getMemorySize();

  virtual ~ManagedScan();

  virtual void setRangeFilter(double max, double min);
  virtual void setHeightFilter(double top, double bottom);
  virtual void setRangeMutation(double range);
  
  virtual void setReductionParameter(double voxelSize, int nrpts = 0,
    PointType pointtype = PointType());
  void setShowReductionParameter(double voxelSize, int nrpts = 0,
    PointType pointtype = PointType());
  virtual void setOcttreeParameter(double reduction_voxelSize,
    double octtree_voxelSize, PointType pointtype,
    bool loadOct, bool saveOct);

  virtual const char* getIdentifier() const { return m_shared_scan->getIdentifier(); }

  virtual DataPointer get(const std::string& identifier);
  virtual void get(unsigned int types);
  virtual DataPointer create(const std::string& identifier, unsigned int size);
  virtual void clear(const std::string& identifier);

  virtual unsigned int readFrames();
  virtual void saveFrames();
  virtual unsigned int getFrameCount();
  virtual void getFrame(unsigned int i, const double*& pose_matrix, AlgoType& type);

protected:
  virtual void createSearchTreePrivate();
  virtual void calcReducedOnDemandPrivate();
  virtual void calcNormalsOnDemandPrivate() {};
  virtual void addFrame(AlgoType type);

private:
  //! Reference to the shared scan
  SharedScan* m_shared_scan;

  //! SharedScan vector to be deleted on closeDirectory
  static SharedScanVector* shared_scans;

  //! Flag to keep track of whether we have to update/create reduced points at the start
  bool m_reduced_ready;

  //! Flag to reset the persistent frames on write actions in slam to avoid a clear call
  bool m_reset_frames_on_write;

  //! Voxelsize of the octtree used for reduction
  double show_reduction_voxelSize;

  //! Which point to take out of the reduction octtree, 0 for center
  int show_reduction_nrpts;

  //! Pointtype used for the reduction octtree
  PointType show_reduction_pointtype;


  ManagedScan(SharedScan* shared_scan);

  //! Create reduced points for show
  void calcReducedShow();

  //! Create Octtree for show
  void createOcttree();
};

#endif //MANAGED_SCAN_H
