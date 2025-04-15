#ifndef __LSEGMENT_H__
#define __LSEGMENT_H__

#include<vector>

#include "srr/linescan.h"

class LSegment {
public:

  LSegment(int start_, int end_);
  LSegment(int start_, int end_, int inverval);
  LSegment(int start_, int end_, int interval, int overlap);

  ~LSegment() {
    clearLscans();
  }

  int size;
  std::vector<LScan*> lscans;
  int start;
  int end;

  inline int getBegin() { return lscans[0]->getBegin(); }   // TODO: check if lscans.size() > 0
  inline int getEnd() { return lscans[lscans.size()-1]->getEnd(); }

  void createLScans(int interval);
  void createLScans(int interval, int width, bool evenRepDistribution = false);

private:

  inline void clearLscans() {
    for(size_t i=0; i<lscans.size(); i++) {
      delete lscans[i];
    }
    lscans.clear();
  }

};


class LSegmentParser {
public:
  LSegmentParser();

  bool loadCfg(const std::string &filename);

  std::vector<std::pair<int, int>> indices;
  std::vector<int> intervals;
  std::vector<int> width;

};

#endif
