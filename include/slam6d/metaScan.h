/*
 * metascan definition
 *
 * Copyright (C) by the 3DTK contributors
 *
 * Released under the GPL version 3.
 *
 */

#ifndef __META_SCAN_H__
#define __META_SCAN_H__

#include "scan.h"

#include <vector>

class MetaScan : public Scan {
public:
  MetaScan(std::vector<Scan*> scans, int nns_method = -1);
  virtual ~MetaScan();

  //! How many scans this meta scan contains
  size_t size() const;

  //! Return the contained scan
  Scan* getScan(size_t i) const;

  virtual void setRangeFilter(double max, double min) {}
  virtual void setHeightFilter(double top, double bottom) {}
  virtual void setCustomFilter(std::string& cFiltStr) {}
  virtual void setScaleFilter(double scale) {}

  virtual const char* getIdentifier() const { return "metascan"; }

  virtual DataPointer get(const std::string& identifier)
  { return DataPointer(0, 0); }

  virtual void get(IODataType types) {}

  virtual DataPointer create(const std::string& identifier, size_t size)
  { return DataPointer(0, 0); }

  virtual void clear(const std::string& identifier) {}

  virtual size_t readFrames() { return 0; }

  virtual void saveFrames(bool append = false) {}

  virtual size_t getFrameCount() { return 0; }

  virtual void getFrame(size_t i,
                        const double*& pose_matrix,
                        AlgoType& type) {}

protected:
  virtual void createSearchTreePrivate();
  virtual void calcReducedOnDemandPrivate() {}
  virtual void calcNormalsOnDemandPrivate() {}
  virtual void addFrame(AlgoType type) {}

private:
  std::vector<Scan*> m_scans;
};

#endif // __META_SCAN_H__
