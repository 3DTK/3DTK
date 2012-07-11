/*
 * temporaryHandler implementation
 *
 * Copyright (C) Thomas Escher, Kai Lingemann
 *
 * Released under the GPL version 3.
 *
 */

#include "scanserver/temporaryHandler.h"

#include <sstream>
#include <stdexcept>
using namespace std;

#include "scanserver/cache/cacheManager.h"



TemporaryHandler::TemporaryHandler(CacheObject* obj, CacheManager* cm, SharedScan* scan, bool static_data) :
  CacheHandler(obj, cm),
  m_scan(scan),
  m_written(false), m_static_data(static_data)
{
  m_id = CacheIO::getId();
}

bool TemporaryHandler::load()
{
  // INFO
  //cout << "[" << m_scan->getIdentifier() << "][" << m_id << "] TemporaryHandler::load";
  
  // if the file was not written (equals invalidated) or file doesn't exist we can't load anything
  unsigned int size = 0;
  if(!m_written || (size = CacheIO::check(m_id)) == 0) {
    // INFO
    //cout << ", no file found" << endl;
    
    return false;
  }
  
  // INFO
  //cout << endl;
  
  // get space
  unsigned char* data_ptr;
  try {
    data_ptr = m_manager->allocateCacheObject(m_object, size);
  } catch(runtime_error& e) {
    // INFO
    // cerr << "[" << m_scan->getIdentifier() << "][" << m_id << "] CacheManager error: " << e.what() << endl;
    // rethrow
    throw e;
  }
  
  // write data into the CO
  CacheIO::read(m_id, reinterpret_cast<char*>(data_ptr));
  // TODO: check errors
  
  // INFO
  //cout << "[" << m_scan->getIdentifier() << "][" << m_id << "] TemporaryHandler::load successful" << endl;
  
  return true;
}

void TemporaryHandler::save(unsigned char* data, unsigned int size)
{
  // INFO
  //cout << "[" << m_scan->getIdentifier() << "][" << m_id << "] TemporaryHandler::save";
  
  // save if the cached file doesn't exist yet or data is dynamic and file content has to be updated
  if(!m_written || !m_static_data) {
    // write to file and flag for cached reads from here on
    CacheIO::write(m_id, reinterpret_cast<char*>(data), size);
    m_written = true;
  } else {
    // INFO
    //cout << ", file already cached";
  }
  
  // INFO
  //cout << endl;
  return;
}
