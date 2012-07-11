/*
 * cacheIO implementation
 *
 * Copyright (C) Thomas Escher, Kai Lingemann
 *
 * Released under the GPL version 3.
 *
 */

#include "scanserver/cacheIO.h"

#include <fstream>
#include <sstream>
#include <iomanip>
using namespace std;
#include <boost/filesystem/operations.hpp>
using namespace boost::filesystem;

#ifdef WITH_METRICS
#include "slam6d/metrics.h"
#endif //WITH_METRICS



string CacheIO::path(".");
unsigned int CacheIO::free_id = 0;



void CacheIO::createTemporaryDirectory(std::string& path)
{
  CacheIO::path = path;
  if(path.rfind('/') != path.size() - 1)
    CacheIO::path += '/';
  
  // check and create
  if(!exists(CacheIO::path))
    create_directory(CacheIO::path);
}

void CacheIO::removeTemporaryDirectory()
{
  // this is going to be fun: rm -rf /
  remove_all(path);
}

CacheIO::IDType CacheIO::getId()
{
  stringstream ss;
  ss << setfill('0') << setw(5) << free_id++ << ".tco";
  return ss.str();
}

unsigned int CacheIO::check(CacheIO::IDType& id)
{
  if(exists(path+id))
    return file_size(path+id);
  else
    return 0;
}

void CacheIO::read(CacheIO::IDType& id, char* data)
{
#ifdef WITH_METRICS
  Timer t = ServerMetric::cacheio_read_time.start();
#endif //WITH_METRICS
  ifstream file((path+id).c_str(), ios_base::in|ios_base::binary);
  file.read(data, file_size(path+id));
#ifdef WITH_METRICS
  ServerMetric::cacheio_read_time.end(t);
  ServerMetric::cacheio_read_size.add(file_size(path+id));
#endif //WITH_METRICS
}

void CacheIO::write(CacheIO::IDType& id, char* data, unsigned int size)
{
#ifdef WITH_METRICS
  Timer t = ServerMetric::cacheio_write_time.start();
#endif //WITH_METRICS
  ofstream file((path+id).c_str(), ios_base::out|ios_base::binary);
  file.write(data, size);
  file.flush();
  file.close();
#ifdef WITH_METRICS
  ServerMetric::cacheio_write_time.end(t);
  ServerMetric::cacheio_write_size.add(size);
#endif //WITH_METRICS
}
