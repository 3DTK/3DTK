#include<srr/lsegment.h>

#include<vector>
#include<fstream>
#include<algorithm>

#include<boost/algorithm/string.hpp>
#include<string>

LSegment::LSegment(int start_, int end_):start(start_),end(end_) {
  // TODO: check parameter, start >= end;
  size = end - start + 1;
}

LSegment::LSegment(int start_, int end_, int interval, int overlap):LSegment(start_,end_) {
  createLScans(interval,overlap);
}

LSegment::LSegment(int start_, int end_, int interval):LSegment(start_,end_) {
  createLScans(interval);
}

void LSegment::createLScans(int interval) {
  clearLscans();
  int parts = static_cast<unsigned int>(ceil((double) size / (double) interval));
  std::cout << "split segment " << start << " to " << end << " in " << parts << " parts" << std::flush;
  for(int i=0; i < parts; i++) {
    int firstIndex =  start + i * interval;
    int lastIndex  =  start + (i+1) * interval -1;
    if(lastIndex > end)
      lastIndex = end;
    int referenceIndex = firstIndex + (lastIndex - firstIndex)/2; // alternatively lastIndex
    lscans.push_back(new LScan(firstIndex,lastIndex,referenceIndex));
  }
  std::cout << std::endl;
}

void LSegment::createLScans(int interval, int width, bool evenRepDistribution) {
  clearLscans();
  int parts = size / interval + 1;
  std::cout << "split segment " << start << " to " << end << " in " << parts << " parts" << std::endl;

  int start_offset = start;
  if(evenRepDistribution) {
    int rest = size - (parts-1)*interval;
    start_offset += rest/2;
  }

  for(int i=0;i<parts;i++) {
    int representative = start_offset + i * interval;
    int earliest =  std::max(representative - width, start);
    int latest = std::min(representative + width, end);
    if(i==parts-1) {
      if(!evenRepDistribution) {
        latest=end;
      }
      representative = std::min(representative, end);
    }
    lscans.push_back(new LScan(earliest,latest,representative));
  }
  std::cout << "created LScans from " << lscans[0]->getBegin() << " to " << lscans[lscans.size()-1]->getEnd() << std::endl;
}

LSegmentParser::LSegmentParser() {

}

/* Segment Format, similiar to ply files

1: SEGMENTS 7
2: FORMAT start end interval overlap
3: 0 417 20 15
4: 418 488 10 5
5: ...

first line in file contains nr of segments
second line specifies the line format. interval and overlap are optional
line 3ff contain the segments
*/

bool LSegmentParser::loadCfg(const std::string &filename) {

  indices.clear();
  intervals.clear();
  width.clear();

  std::ifstream config_in;
  config_in.open(filename);
  if(!config_in) {
    std::cerr << "filename invalid" << std::endl;
    return false;
  }

  std::string line="";

  std::getline(config_in,line);
  std::vector<std::string> words;
  boost::split(words,line, boost::is_any_of(" "));
  if(words.size() != 2 || words.at(0) != "SEGMENTS") {
    std::cerr << "first line invalid" << std::endl;
    return false;
  }
  int nrOfSegments;
  try {
    nrOfSegments = std::stoi(words.at(1));
  } catch(...) {
    std::cerr << "nr of segments is not an integer" << std::endl;
    return false;
  }
  words.clear();
  line="";

  std::getline(config_in,line);
  boost::split(words,line, boost::is_any_of(" "));
  if(words.size() < 2 || words.at(0) != "FORMAT") { // TODO: check which propertys are available
    std::cerr << "format line invalid" << std::endl;
    return false;
  }
  words.clear();
  line="";

  indices.resize(nrOfSegments);
  intervals.resize(nrOfSegments,-1);
  width.resize(nrOfSegments,-1);
  int segmentcount = 0;

  while(getline(config_in,line) && segmentcount < nrOfSegments) {
    boost::trim(line);
    boost::split(words,line, boost::is_any_of(" "));
    if(words.size() < 2) {
      std::cerr << "less values than required in segment " << segmentcount << "." << std::endl;
      return false;
    } else if(words.size() > 4) { // TODO: extend if prereginterval also used
      std::cerr << "more values than allowed in segment " << segmentcount << "." << std::endl;
      return false;
    }

    int start, end;
    try {
      start = std::stoi(words.at(0));
      end   = std::stoi(words.at(1));

      if(start < 0 || end < start) {

        indices.clear();
        intervals.clear();
        width.clear();
        return false;
      }
      indices[segmentcount]=std::make_pair(start, end);

      if(words.size() >= 3)
        intervals[segmentcount] = std::stoi(words.at(2));
      if(words.size() == 4)
        width[segmentcount] = std::stoi(words.at(3));
    } catch(...) {
      std::cerr << "value is not an integer: segment " << segmentcount << std::endl;
      indices.clear();
      intervals.clear();
      width.clear();
      return false;
    }

    line="";
    segmentcount++;
  }

  if(segmentcount != nrOfSegments) {
    std::cerr << "number of segments and number of segment lines do not match" << std::endl;
    indices.clear();
    intervals.clear();
    width.clear();
    return false;
  }

  return true;

}
