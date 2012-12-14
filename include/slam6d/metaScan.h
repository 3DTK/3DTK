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
  unsigned int size() const;

  //! Return the contained scan
  Scan* getScan(unsigned int i) const;

  virtual void setRangeFilter(double max, double min) {}
  virtual void setHeightFilter(double top, double bottom) {}

  virtual const char* getIdentifier() const { return "metascan"; }
  
  virtual DataPointer get(const std::string& identifier) 
  { return DataPointer(0, 0); }

  virtual void get(unsigned int types) {}

  virtual DataPointer create(const std::string& identifier, unsigned int size)
  { return DataPointer(0, 0); }

  virtual void clear(const std::string& identifier) {}

  virtual unsigned int readFrames() { return 0; }

  virtual void saveFrames() {}

  virtual unsigned int getFrameCount() { return 0; }

  virtual void getFrame(unsigned int i,
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
