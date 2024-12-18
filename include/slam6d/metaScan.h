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
#include <map>

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
  {
    std::map<std::string, std::pair<unsigned char*, size_t>>::iterator it = m_pairs.find(identifier);
    // Create data
    if (it == m_pairs.end())
    {
      size_t nrPts = 0;
      for (Scan *s : m_scans) {
        nrPts += s->size<DataXYZ>(identifier);
      }
      if (identifier == "xyz") {
        vector<double> xyz;
        xyz.reserve(nrPts);
        for (Scan *s : m_scans) {
          DataXYZ data_ptr = s->get("xyz");
          for(int i = 0; i < data_ptr.size(); ++i) {
            // MetaScan uses the transformed points!
            double p[3] = {data_ptr[i][0], data_ptr[i][1], data_ptr[i][2]};
            transform3(s->get_transMat(), p);
            xyz.push_back(p[0]);
            xyz.push_back(p[1]);
            xyz.push_back(p[2]);
          }
        }
        double* data = reinterpret_cast<double*>(create("xyz",
          sizeof(double)*3*nrPts).get_raw_pointer());
        for(size_t i = 0; i < xyz.size(); ++i) data[i] = xyz[i];
      } else if (identifier == "xyz reduced"
        || identifier == "xyz reduced original") {
        calcReducedOnDemand();
      } else if (identifier == "xyz reduced show") {
        calcReducedPoints();
        m_pairs["xyz reduced show"] = m_pairs["xyz reduced"];
      }
      it = m_pairs.find(identifier);
    }

    // If the data still does not exist (failure) return Datapointer to 0
    if (it == m_pairs.end())
      return DataPointer(0, 0);
    else {
      return DataPointer(it->second.first, it->second.second);
    }
  }

  virtual void get(IODataType types) {}

  virtual DataPointer create(const std::string& identifier, size_t size)
  {
    std::map<std::string, std::pair<unsigned char*, size_t>>::iterator it = m_pairs.find(identifier);
    if(it != m_pairs.end() && it->second.second != size) {
      delete[] it->second.first;
    }
    unsigned char *data;
    if(it == m_pairs.end() || it->second.second != size) {
      data = new unsigned char[size];
    }
    if(it == m_pairs.end()) {
      it = m_pairs.insert(std::make_pair(identifier,
        std::make_pair(data, size))).first;
    } else if (it->second.second != size) {
      it->second.first = data;
      it->second.second = size;
    }
    return DataPointer(it->second.first, it->second.second);
  }

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
  std::map<std::string, std::pair<unsigned char*, size_t> > m_pairs;
};

#endif // __META_SCAN_H__
