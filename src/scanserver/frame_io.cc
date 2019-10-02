/*
 * frame_io implementation
 *
 * Copyright (C) Thomas Escher, Kai Lingemann
 *
 * Released under the GPL version 3.
 *
 */

#include "scanserver/frame_io.h"

#include <vector>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
using namespace boost::filesystem;



#define FRAMES_PATH_PREFIX "scan"
#define FRAMES_PATH_SUFFIX ".frames"

void FrameIO::loadFile(const char* dir, const char* identifier, FrameVector& frames)
{
  // assemble path and check for existence
  path frames_path(dir);
  frames_path /= (std::string(FRAMES_PATH_PREFIX) + identifier + FRAMES_PATH_SUFFIX);
  if(!exists(frames_path)) return;

  // read the file
  ifstream frames_file(frames_path);
  std::string line;
  std::vector<double> transformations;
  double dvalue;
  unsigned int uivalue;
  std::vector<unsigned int> types;
  while(std::getline(frames_file, line) ) {
    unsigned int i;
    //ignore comment lines starting with #
    if (line[0] == '#') continue;
    std::istringstream line_stream(line);
    for(i = 0; i < 16 && line_stream.good(); ++i) {
      line_stream >> dvalue;
      transformations.push_back(dvalue);
    }
    // sanity check if no more values were read
    if(i != 16) break;
    line_stream >> uivalue;
    types.push_back(uivalue);
  }
  frames_file.close();

  // allocate shared memory for the frames and move the values in there
  unsigned int s = types.size();
  frames.resize(s);
  for(unsigned int i = 0; i < s; ++i)
    frames[i].set(&(transformations[i*16]), types[i]);
}

void FrameIO::saveFile(const char* dir, const char* identifier, const FrameVector& frames, bool append)
{
  // assemble path
  path frames_path(dir);
  frames_path /= (std::string(FRAMES_PATH_PREFIX) + identifier + FRAMES_PATH_SUFFIX);

  // write into the file
  std::ios_base::openmode open_mode = append ? std::ios_base::app : std::ios_base::out;
  ofstream frames_file(frames_path, open_mode);
  for(FrameVector::const_iterator it = frames.begin(); it != frames.end(); ++it) {
    for(unsigned int i = 0; i < 16; ++i)
      frames_file << it->transformation[i] << " ";
    frames_file << it->type << std::endl;
  }
  frames_file.close();
}
